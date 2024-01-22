//
// Created by xu on 2020/7/6.
//

#include "../include/unicore/msg_buffer.hpp"
#include <swri_roscpp/publisher.h>

MsgBuffer all_msg_buffer_;
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "gps_node");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  ros::Publisher gpgga_pub_ = swri::advertise<novatel_gps_msgs::Gpgga>(nh, "gpgga", 100);
  ros::Publisher inspva_pub_ = swri::advertise<novatel_gps_msgs::Inspva>(nh, "inspva", 100);
  ros::Publisher rawimu_pub_ = swri::advertise<sensor_msgs::Imu>(nh, "rawimu", 100);

  std::vector<sensor_msgs::ImuPtr> rawimu_msgs;
  std::vector<novatel_gps_msgs::InspvaPtr> inspva_msgs;
  std::vector<novatel_gps_msgs::GpggaPtr> gpgga_msgs;
  all_msg_buffer_.Connect("/dev/ttyUSB0", "SERIAL");
  // todo: 还有raw_imu的部分
  while (ros::ok()) {
    ros::spinOnce();
      MsgBuffer::ReadResult result = all_msg_buffer_.ProcessData();
      // This call appears to block if the serial device is disconnected
      if (result == MsgBuffer::READ_ERROR)
      {
        std::cerr << "未连接GNSS/INS组合导航仪器" << std::endl;
        continue;
      }
      all_msg_buffer_.GetRawImuMessages(rawimu_msgs);
      all_msg_buffer_.GetGpggaMessages(gpgga_msgs);
      all_msg_buffer_.GetInspvaMessages(inspva_msgs);
      for (const auto& msg : gpgga_msgs)
      {
        msg->header.frame_id = "map";
        gpgga_pub_.publish(msg);
      }
      for (const auto& msg : inspva_msgs)
      {
        msg->header.frame_id = "inspva";
        inspva_pub_.publish(msg);
      }
      for (const auto& msg : rawimu_msgs)
      {
       msg->header.frame_id = "imu";
        rawimu_pub_.publish(msg);
	//std::cerr << "发布了IMU" << std::endl;
      }

    rate.sleep();
    }


}
