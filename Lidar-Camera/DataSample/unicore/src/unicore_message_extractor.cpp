// 这部分只需要分开就好，分开放到对应的容器

#include "unicore/unicore_message_extractor.hpp"

#include <sstream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <ros/ros.h>

#include <swri_string_util/string_util.h>
#include <novatel_gps_driver/parsers/header.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#include <novatel_gps_driver/parsers/gprmc.h>

namespace novatel_gps_driver
{
const std::string UnicoreMessageExtractor::CHECKSUM_FLAG = "*";
const std::string UnicoreMessageExtractor::FIELD_SEPARATOR = ",";
const std::string UnicoreMessageExtractor::HEADER_SEPARATOR = ";";
const std::string UnicoreMessageExtractor::NMEA_SENTENCE_FLAG = "$";
const std::string UnicoreMessageExtractor::NOVATEL_SENTENCE_FLAG = "%";
const std::string UnicoreMessageExtractor::NOVATEL_ASCII_FLAGS = "$%";
const std::string UnicoreMessageExtractor::NOVATEL_ENDLINE = "\r\n";

uint8_t UnicoreMessageExtractor::NmeaChecksum(const std::string& sentence)
{
  uint8_t checksum = 0;
  std::string::const_iterator it = sentence.begin();
  for (; it != sentence.end(); ++it)
  {
    checksum ^= *it;
  }
  return checksum;
}

size_t UnicoreMessageExtractor::GetSentenceChecksumStart(const std::string& str, size_t start_idx)
{
  return str.find(CHECKSUM_FLAG, start_idx);
}

void UnicoreMessageExtractor::VectorizeString(
    const std::string& str,
    std::vector<std::string>& vectorized_message,
    const std::string& delimiters)
{
  boost::algorithm::split(vectorized_message, str, boost::algorithm::is_any_of(delimiters));
}

bool UnicoreMessageExtractor::GetNovatelMessageParts(
    const std::string& sentence,
    std::string& message_id,
    std::vector<std::string>& header,
    std::vector<std::string>& body)
{
  message_id.clear();
  header.clear();
  body.clear();

  std::vector<std::string> vectorized_message;
  VectorizeString(sentence, vectorized_message, HEADER_SEPARATOR);

  if (vectorized_message.size() != 2)
  {
    return false;
  }

  VectorizeString(vectorized_message[0], header, FIELD_SEPARATOR);

  VectorizeString(vectorized_message[1], body, FIELD_SEPARATOR);

  if (!header.empty())
  {
    message_id = header.front();
  }
  else
  {
    return false;
  }
  return true;
}

// 没有把sentence放进来
int32_t UnicoreMessageExtractor::GetNovatelSentence(
    const std::string& str,
    size_t start_idx,
    std::string& sentence)
{
  sentence.clear();

  size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
  if (checksum_start == std::string::npos)
  {
    // Sentence not complete. Just return.
    return -1;
  }
  else if (checksum_start + 8 >= str.size())
  {
    // Sentence not complete. Just return.
    return -1;
  }
  else
  {
    // Compare the checksums
    sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
    return 0;
  }
}

int32_t UnicoreMessageExtractor::GetNmeaSentence(
    const std::string& str,
    size_t start_idx,
    std::string& sentence,
    bool keep_container)
{
  sentence.clear();
  size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
  if (checksum_start == std::string::npos)
  {
    // Sentence not complete. Just return.
    return -1;
  }
  else if (checksum_start + 2 >= str.size())
  {
    // Sentence not complete. Just return.
    return -1;
  }
  else
  {
    // Compare the checksums
    sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
    std::string checksum_str = str.substr(checksum_start + 1, 2);
    uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
    uint64_t calculated_checksum = NmeaChecksum(sentence);
    if (checksum == ULONG_MAX)
    {
      // Invalid checksum
      return 1;
    }
    else if(static_cast<uint32_t>(checksum) == calculated_checksum)
    {
      if (keep_container)
      {
        sentence.insert(0, "$");
        std::string recreated_checksum_str("*");
        recreated_checksum_str += checksum_str;
        sentence.insert(sentence.end(),
                        recreated_checksum_str.begin(),
                        recreated_checksum_str.end());
      }
      return 0;
    }
    else
    {
      ROS_WARN("Expected: [%lx]", calculated_checksum);
      // Invalid checksum
      return 1;
    }
  }
}

void UnicoreMessageExtractor::FindAsciiSentence(const std::string& sentence,
						
                                                size_t current_idx,
                                                size_t& start_idx,
                                                size_t& end_idx,
                                                size_t& invalid_char_idx)
{
  start_idx = sentence.find_first_of(NOVATEL_ASCII_FLAGS, current_idx);
  end_idx = std::string::npos;
  invalid_char_idx = std::string::npos;

  if (start_idx == std::string::npos)
  {
    return;
  }

  // todo: 检测不到尾巴
  end_idx = sentence.find(NOVATEL_ENDLINE, start_idx);
  /// 检查是否存在非法字符
//  size_t search_stop_idx = std::min(end_idx, sentence.length());
//  for (size_t i = start_idx; i < search_stop_idx; i++)
//  {
//    if (sentence[i] == 9 || sentence[i] == 10 || sentence[i] == 11 || sentence[i] == 13 ||
//        (sentence[i] >= 32 && sentence[i] <= 126))
//    {
//      continue;
//    }
//
//    invalid_char_idx = i;
//    std::cerr << "invalid char!!!!!!!!!!!!!!!!! " << std::endl;
//    break;
//  }
}

bool UnicoreMessageExtractor::VectorizeNovatelSentence(
    const std::string& data,
    NovatelSentence& sentence)
{
  return GetNovatelMessageParts(
      data, sentence.id, sentence.header, sentence.body);
}

void UnicoreMessageExtractor::VectorizeNmeaSentence(
    const std::string& sentence,
    NmeaSentence& vectorized_message)
{
  VectorizeString(sentence, vectorized_message.body, FIELD_SEPARATOR);
  if (!vectorized_message.body.empty())
  {
    vectorized_message.id = vectorized_message.body.front();
  }
}

// 这个input是当前存的全部
bool UnicoreMessageExtractor::ExtractCompleteMessages(
    const std::string& input,
    std::vector<NmeaSentence>& nmea_sentences,
    std::vector<NovatelSentence>& novatel_sentences,
    std::string& remaining,
    bool keep_nmea_container)
{
  bool parse_error = false;
  size_t sentence_start = 0;
  while(sentence_start != std::string::npos && sentence_start < input.size())
  {
    size_t ascii_start_idx; // nmea & novatel的头
    size_t ascii_end_idx;
    size_t invalid_ascii_idx;

    FindAsciiSentence(input, sentence_start, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);

    if (ascii_start_idx == std::string::npos)
    {
      // If we don't see either a binary or an ASCII message, just give up.
      break;
    }

    // If we saw the beginning of an ASCII message, try to parse it.
    size_t ascii_len = ascii_end_idx - ascii_start_idx;
    if (invalid_ascii_idx != std::string::npos) // 我把invalid_ascii的判定去掉了
    {
      std::cout << "Invalid ASCII char: " << input.substr(ascii_start_idx, ascii_len) << std::endl;
      sentence_start += invalid_ascii_idx + 1;
    }
    else if (ascii_end_idx != std::string::npos)
    {
      // If we've got a start, an end, and no invalid characters, we've
      // got a valid ASCII message.
      ROS_DEBUG("ASCII sentence:\n[%s]", input.substr(ascii_start_idx, ascii_len).c_str());
      if (input[ascii_start_idx] == NMEA_SENTENCE_FLAG[0])
      {
        std::string cur_sentence;
        int32_t result = GetNmeaSentence(
            input,
            ascii_start_idx,
            cur_sentence,
            keep_nmea_container); /// 做奇偶校验
        if (result == 0)
        {
          nmea_sentences.emplace_back(NmeaSentence());  // 占位置
          VectorizeNmeaSentence(cur_sentence, nmea_sentences.back());//todo: body里面有16个 等出去测试的时候看一下
          // std::cout << nmea_sentences.back().id << nmea_sentences.back().body.size() << std::endl;
          sentence_start = ascii_end_idx;
        }
        else if (result < 0)
        {
          // Sentence is not complete, add it to the remaining and break.
          // This is legacy code from before FindAsciiSentence was implemented,
          // and it will probably never happen, but it doesn't hurt anything to
          // have it here.
          remaining = input.substr(ascii_start_idx);
          ROS_DEBUG("Waiting for more NMEA data.");
          break;
        }
        else
        {
          ROS_WARN("Invalid NMEA checksum for: [%s]",
                   input.substr(ascii_start_idx, ascii_len).c_str());
          // Sentence had an invalid checksum, just iterate to the next sentence
          sentence_start += 1;
          parse_error = true;
        }
      }
      else if (input[ascii_start_idx] == NOVATEL_SENTENCE_FLAG[0])
      {
        std::string cur_sentence;
        int32_t result = GetNovatelSentence(input, ascii_start_idx, cur_sentence);
//        std::cout << "result: " << result << std::endl;
        if (result == 0)
        {
          // Send to parser for testing:
//          std::cout << "cur_sentence: " << cur_sentence << std::endl;
          novatel_sentences.emplace_back(NovatelSentence());
          if (!VectorizeNovatelSentence(cur_sentence, novatel_sentences.back()))
          {
            novatel_sentences.pop_back();
            parse_error = true;
            ROS_ERROR_THROTTLE(1.0, "Unable to vectorize novatel sentence");
          }
          sentence_start = ascii_end_idx;
        }
        else if (result < 0)
        {
          // Sentence is not complete, add it to the remaining and break.
          // This is legacy code from before FindAsciiSentence was implemented,
          // and it will probably never happen, but it doesn't hurt anything to
          // have it here.
          remaining = input.substr(ascii_start_idx);
          ROS_DEBUG("Waiting for more NovAtel data.");
          break;
        }
        else
        {
          // Sentence had an invalid checksum, just iterate to the next sentence
          sentence_start += 1;
          ROS_WARN("Invalid NovAtel checksum for: [%s]",
                   input.substr(ascii_start_idx, ascii_len).c_str());
          parse_error = true;
        }
      }
    }
    else
    {
      ROS_DEBUG("Incomplete ASCII sentence, waiting for more.");
      remaining = input.substr(ascii_start_idx);
      break;
    }
  }

  return !parse_error;
}

double UnicoreMessageExtractor::GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences)
{
  std::vector<NmeaSentence>::const_reverse_iterator iter;
  for (iter = sentences.rbegin(); iter != sentences.rend(); iter++)
  {
    if (iter->id == GpggaParser::MESSAGE_NAME || iter->id == GprmcParser::MESSAGE_NAME)
    {
      if (iter->body.size() > 1)
      {
        if (iter->body[1].empty() || iter->body[1] == "0")
        {
          return 0;
        }
        else
        {
          double utc_float;
          if (swri_string_util::ToDouble(iter->body[1], utc_float))
          {
            return UtcFloatToSeconds(utc_float);
          }
          return 0;
        }
      }
    }
  }

  return 0;
}
}
