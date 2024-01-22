//
// Created by xu on 2020/7/7.
// 这个函数就是接受数据

#ifndef SRC_MSG_BUFFER_HPP
#define SRC_MSG_BUFFER_HPP

#include "parse_sentence.hpp"
#include <vector>
#include <ros/ros.h>
#include <swri_serial_util/serial_port.h>
#include <novatel_gps_driver/novatel_message_extractor.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#include <novatel_gps_driver/parsers/inspva.h>
#include <boost/circular_buffer.hpp>
#include  "unicore_message_extractor.hpp"

/// Define NovatelMessageOpts as a map from message name to log period (seconds)
/// A negative period will be logged as "onchanged" rather than "ontime"
typedef std::map<std::string, double> NovatelMessageOpts;

class MsgBuffer {
public:

  enum ReadResult
  {
    READ_SUCCESS = 0,
    READ_INSUFFICIENT_DATA = 1,
    READ_TIMEOUT = 2,
    READ_INTERRUPTED = 3,
    READ_ERROR = -1,
    READ_PARSE_FAILED = -2
  };

  MsgBuffer();
  ~MsgBuffer();

void GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr> &gpgga_messages);

void GetInspvaMessages(std::vector<novatel_gps_msgs::InspvaPtr> &inspva_messages);

void GetRawImuMessages(std::vector<sensor_msgs::ImuPtr> &rawimu_messages);

  bool Connect(const std::string& device, const std::string& connection);
  void Disconnect();
  bool CreateSerialConnection(const std::string& device, NovatelMessageOpts const& opts);

  ReadResult ReadData();

  ReadResult ProcessData();

  bool Write(const std::string& command);

  ReadResult ParseNmeaSentence(const novatel_gps_driver::NmeaSentence& sentence,
                                           const ros::Time& stamp,
                                           double most_recent_utc_time) noexcept(false);

  ReadResult ParseNovatelSentence(const novatel_gps_driver::NovatelSentence& sentence,
                                              const ros::Time& stamp) noexcept(false);

private:
  bool Configure(NovatelMessageOpts const& opts);


public:
  std::string error_msg_;
  double imu_rate_ = 100;
  swri_serial_util::SerialPort serial_;
  int32_t serial_baud_ = 115200;  // 设备要求115200
  bool is_connected_ = false;
  novatel_gps_driver::UnicoreMessageExtractor extractor_; // extract message from the imcoming data stream

  // message parser
  novatel_gps_driver::GpggaParser gpgga_parser_;
  ParseInspva inspva_parser_;
  ParseRawImu rawimu_parser_;

  // message buffer
  boost::circular_buffer<novatel_gps_msgs::GpggaPtr> gpgga_msgs_;
  boost::circular_buffer<novatel_gps_msgs::InspvaPtr> inspva_msgs_;
  boost::circular_buffer<sensor_msgs::ImuPtr> rawimu_msgs_;

  // Data buffers
  std::vector<uint8_t> data_buffer_;
  // Buffer containing incomplete data from message parsing
  std::string nmea_buffer_;

  static constexpr size_t MAX_BUFFER_SIZE = 100;
};



#endif //SRC_MSG_BUFFER_HPP
