//
// Created by xu on 2020/7/6.
//

#include "../include/unicore/parse_sentence.hpp"
#include <boost/make_shared.hpp>
#include <swri_roscpp/publisher.h>

sensor_msgs::ImuPtr ParseRawImu::ParseAscii(const novatel_gps_driver::NovatelSentence &sentence){
  sensor_msgs::ImuPtr msg ;
  msg = boost::make_shared<sensor_msgs::Imu>();
  bool valid = true;
 
  valid &= novatel_gps_driver::ParseDouble(sentence.body[8], msg->angular_velocity.x);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[7], msg->angular_velocity.y);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[6], msg->angular_velocity.z);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[5], msg->linear_acceleration.x);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[4], msg->linear_acceleration.y);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[3], msg->linear_acceleration.z);
  /*valid &= novatel_gps_driver::ParseUInt32(sentence.body[0], msg->week);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[1], msg->seconds_into_week);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[2], msg->z_accel_output);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[3], msg->y_accel_output);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[4], msg->x_accel_output);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[5], msg->z_gyro_output);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[6], msg->y_gyro_output);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[7], msg->x_gyro_output);*/
  //msg->orientation.x = msg->orientation.x * 100 * 400 / 2147483648;
  //msg->orientation.y = msg->orientation.y * 100 * 400 / 2147483648;
  //msg->orientation.z = msg->orientation.z * 100 * 400 / 2147483648;
  //msg->orientation.w = msg->orientation.w * 100 * 400 / 2147483648;
  
  msg->angular_velocity.x  = msg->angular_velocity.x * 100 * 400 / 2147483648;
  msg->angular_velocity.y  = msg->angular_velocity.y * 100 * 400 / 2147483648;
  msg->angular_velocity.z  = msg->angular_velocity.z * 100 * 400 / 2147483648;
  msg->linear_acceleration.x = msg->linear_acceleration.x * 2160 * 100 / 2147483648;
  msg->linear_acceleration.y = msg->linear_acceleration.y * 2160 * 100 / 2147483648;
  msg->linear_acceleration.z = msg->linear_acceleration.z * 2160 * 100 / 2147483648;
  msg->orientation.x=0.0;
  msg->orientation.y=0.0;
  msg->orientation.z=0.0;
  msg->orientation.w=0.0;
 // msg->status = sentence.body[9];
 // std::cerr<<"valid=" << valid << std::endl;
 /* if (!valid)
  {
    //std::cerr << valid << std::endl;
    std::cerr << "xuzhi: rawimu log was invalid." << std::endl;
  }*/
  return msg;
}

novatel_gps_msgs::InspvaPtr ParseInspva::ParseAscii(const novatel_gps_driver::NovatelSentence &sentence) {
  novatel_gps_msgs::InspvaPtr msg = boost::make_shared<novatel_gps_msgs::Inspva>();

  bool valid = true;
  valid &= novatel_gps_driver::ParseUInt32(sentence.body[0], msg->week);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[1], msg->seconds);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[2], msg->latitude);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[3], msg->longitude);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[4], msg->height);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[5], msg->north_velocity);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[6], msg->east_velocity);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[7], msg->up_velocity);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[8], msg->roll);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[9], msg->pitch);
  valid &= novatel_gps_driver::ParseDouble(sentence.body[10], msg->azimuth);
  msg->status = sentence.body[11];

  if (!valid)
  {
    std::cerr << "xuzhi: inspva log was invalid." << std::endl;
  }
  return msg;
}
