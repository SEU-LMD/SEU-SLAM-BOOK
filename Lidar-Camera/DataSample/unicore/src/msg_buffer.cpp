//
// Created by xu on 2020/7/7.
//

#include "../include/unicore/msg_buffer.hpp"
#include <novatel_gps_driver/parsers/parse_exception.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>
#include <swri_roscpp/publisher.h>

void MsgBuffer::GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr> &gpgga_messages) {
  gpgga_messages.clear();
  if (!gpgga_msgs_.empty()){
    gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(), gpgga_msgs_.end());
    gpgga_msgs_.clear();
  }
}

void MsgBuffer::GetInspvaMessages(std::vector<novatel_gps_msgs::InspvaPtr> &inspva_messages) {
  inspva_messages.clear();
  if (!inspva_msgs_.empty()){
    inspva_messages.insert(inspva_messages.end(), inspva_msgs_.begin(), inspva_msgs_.end());
    inspva_msgs_.clear();
  }
}

void MsgBuffer::GetRawImuMessages(std::vector<sensor_msgs::ImuPtr> &rawimu_messages) {
  rawimu_messages.clear();
  //std::cerr << "rawimu_msgs_.empty()="<<rawimu_msgs_.empty()<< std::endl;
  if (!rawimu_msgs_.empty()){
    rawimu_messages.insert(rawimu_messages.end(), rawimu_msgs_.begin(), rawimu_msgs_.end());
    rawimu_msgs_.clear();
  }
}

bool MsgBuffer::CreateSerialConnection(const std::string &device, const NovatelMessageOpts &opts) {
  swri_serial_util::SerialConfig config;
  config.baud = serial_baud_;
  config.parity = swri_serial_util::SerialConfig::NO_PARITY;
  config.flow_control = false;
  config.data_bits = 8;
  config.stop_bits = 1;
  config.low_latency_mode = false;
  config.writable = true; // Assume that we can write to this port
  bool success = serial_.Open(device, config);

  if (success)
  {
    is_connected_ = true;
    if (!Configure(opts))
    {
      // We will not kill the connection here, because the device may already
      // be setup to communicate correctly, but we will print a warning
      ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
                "device may not be functioning as expected; however, the "
                "driver may still function correctly if the port has already "
                "been pre-configured.");
    }
  }
  else
  {
    std::cerr << serial_.ErrorMsg() << std::endl;
  }
  return success;
}

MsgBuffer::ReadResult MsgBuffer::ReadData() {
    swri_serial_util::SerialPort::Result result = serial_.ReadBytes(data_buffer_, 0, 5000);
    if (result == swri_serial_util::SerialPort::ERROR)
    {
      error_msg_ = serial_.ErrorMsg();
      return READ_ERROR;
    }
    else if (result == swri_serial_util::SerialPort::TIMEOUT)
    {
      error_msg_ = "Timed out waiting for serial device.";
      return READ_TIMEOUT;
    }
    else if (result == swri_serial_util::SerialPort::INTERRUPTED)
    {
      error_msg_ = "Interrupted during read from serial device.";
      return READ_INTERRUPTED;
    }
    return READ_SUCCESS;
}

// 写入command
bool MsgBuffer::Write(const std::string &command) {
  std::vector<uint8_t> bytes(command.begin(), command.end());
  int32_t written = serial_.Write(bytes);
  if (written != (int32_t)command.length())
  {
    ROS_ERROR("Failed to send command: %s", command.c_str());
  }
  return written == (int32_t)command.length();
}

MsgBuffer::ReadResult MsgBuffer::ParseNmeaSentence(
                                  const novatel_gps_driver::NmeaSentence &sentence,
                                  const ros::Time &stamp,
                                  double most_recent_utc_time) noexcept(false) {

  if (sentence.id == novatel_gps_driver::GpggaParser::MESSAGE_NAME)
  {
    novatel_gps_msgs::GpggaPtr gpgga = gpgga_parser_.ParseAscii(sentence);
    auto gpgga_time = novatel_gps_driver::UtcFloatToSeconds(gpgga->utc_seconds);
    if (most_recent_utc_time < gpgga_time)
    {
      most_recent_utc_time = gpgga_time;
    }
    gpgga->header.stamp = stamp - ros::Duration(most_recent_utc_time - gpgga_time);
    gpgga_msgs_.push_back(gpgga);
  }
  else
  {
    ROS_DEBUG_STREAM("Unrecognized NMEA sentence " << sentence.id);
  }
  return READ_SUCCESS;
}

MsgBuffer::ReadResult MsgBuffer::ParseNovatelSentence(
                                      const novatel_gps_driver::NovatelSentence &sentence,
                                      const ros::Time &stamp) noexcept(false) {
  if (sentence.id == "INSPVASA"){
    novatel_gps_msgs::InspvaPtr inspva = inspva_parser_.ParseAscii(sentence);
    inspva->header.stamp = stamp;
    inspva_msgs_.push_back(inspva);
  }
  
  if (sentence.id == "RAWIMUSA"){
    sensor_msgs::ImuPtr rawimu = rawimu_parser_.ParseAscii(sentence);
    rawimu->header.stamp = stamp;
    rawimu_msgs_.push_back(rawimu);
  }
  return READ_SUCCESS;
}

// 将opts里面数据写入串口
bool MsgBuffer::Configure(const NovatelMessageOpts &opts) {
  bool configured = true;
  configured = configured && Write("unlog\r\n");
  for(const auto& option : opts)
  {
    std::stringstream command;
    command << std::setprecision(3);
    command << "log " << option.first << " ontime " << option.second << "\r\n";
    configured = configured && Write(command.str());
  }
  std::cout << " 是否写入成功: " << configured << std::endl;
  return configured;
}

bool MsgBuffer::Connect(const std::string &device, const std::string &connection) {
  NovatelMessageOpts opts;
  // todo: here to edit log name and frequency
  opts["gpgga"] = 1;
  opts["inspvasa"] = 0.1;
  opts["rawimusa"] = 0.01;
  Disconnect();
  if (connection == "SERIAL")
  {
    return CreateSerialConnection(device, opts);
  }
  std::cout << "Invalid connection type." << std::endl;
  return false;
}

void MsgBuffer::Disconnect() {
  serial_.Close();
}

MsgBuffer::~MsgBuffer() {
  Disconnect();
}

MsgBuffer::MsgBuffer():
    gpgga_msgs_(MAX_BUFFER_SIZE),
    inspva_msgs_(MAX_BUFFER_SIZE),
    rawimu_msgs_(MAX_BUFFER_SIZE){
}

MsgBuffer::ReadResult MsgBuffer::ProcessData() {
    MsgBuffer::ReadResult read_result = ReadData();
    if (read_result != READ_SUCCESS)
    {
      return read_result;
    }
    ros::Time stamp = ros::Time::now();
    std::vector<novatel_gps_driver::NmeaSentence> nmea_sentences;
    std::vector<novatel_gps_driver::NovatelSentence> novatel_sentences;

    if (!data_buffer_.empty())
    {
      nmea_buffer_.insert(nmea_buffer_.end(),
                          data_buffer_.begin(),
                          data_buffer_.end());
      data_buffer_.clear();
      std::string remaining_buffer;

      if (!extractor_.ExtractCompleteMessages(
          nmea_buffer_,
          nmea_sentences,
          novatel_sentences,
          remaining_buffer))
      {
        read_result = READ_PARSE_FAILED;
        error_msg_ = "Parse failure extracting sentences.";
      }
      nmea_buffer_ = remaining_buffer;  // 可能剩下半截，需要留着下次用

      if (!nmea_buffer_.empty())
      {
        ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
      }
    }

    double most_recent_utc_time = extractor_.GetMostRecentUtcTime(nmea_sentences);
    for(const auto& sentence : nmea_sentences)
    {
      try
      {
        MsgBuffer::ReadResult result = ParseNmeaSentence(sentence, stamp, most_recent_utc_time);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const novatel_gps_driver::ParseException& p)
      {
        error_msg_ = p.what();
        ROS_WARN("%s", p.what());
        ROS_WARN("For sentence: [%s]", boost::algorithm::join(sentence.body, ",").c_str());
        read_result = READ_PARSE_FAILED;
      }
    }

    for(const auto& sentence : novatel_sentences)
    {
      try
      {
        MsgBuffer::ReadResult result = ParseNovatelSentence(sentence, stamp);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const novatel_gps_driver::ParseException& p)
      {
        error_msg_ = p.what();
        ROS_WARN("%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }
    return read_result;
}


