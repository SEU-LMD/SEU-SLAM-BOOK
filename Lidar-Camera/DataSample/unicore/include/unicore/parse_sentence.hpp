//
// Created by xu on 2020/7/6.
// 谁传进来的sentence struct，这个类不负责;
// 我们只管做解析的工作

#ifndef SRC_PARSE_SENTENCE_HPP
#define SRC_PARSE_SENTENCE_HPP


#include <novatel_gps_driver/nmea_sentence.h>
#include <novatel_gps_driver/novatel_sentence.h>
#include <novatel_gps_msgs/Gpgga.h>
#include "novatel_gps_msgs/Inspva.h"
#include "novatel_gps_driver/parsers/parsing_utils.h"
#include "Imu.h"

// imu
class ParseRawImu{
public:
  sensor_msgs::ImuPtr ParseAscii(const novatel_gps_driver::NovatelSentence &sentence);
};

class ParseInspva{
public:
  novatel_gps_msgs::InspvaPtr ParseAscii(const novatel_gps_driver::NovatelSentence &sentence);
};


#endif //SRC_PARSE_SENTENCE_HPP
