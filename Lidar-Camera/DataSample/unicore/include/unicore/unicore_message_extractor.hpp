//
// Created by xu on 2020/7/10.
// 这个类主要实现对输入的string进行分割，然后提取对应的setence结构

#ifndef SRC_UNICORE_MESSAGE_EXTRACTOR_HPP
#define SRC_UNICORE_MESSAGE_EXTRACTOR_HPP
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gps_common/GPSFix.h>
#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gpgsa.h>
#include <novatel_gps_msgs/Gpgsv.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelMessageHeader.h>
#include <novatel_gps_msgs/NovatelReceiverStatus.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_gps_msgs/Trackstat.h>

#include <novatel_gps_driver/binary_message.h>
#include <novatel_gps_driver/nmea_sentence.h>
#include <novatel_gps_driver/novatel_sentence.h>

namespace novatel_gps_driver
{
  class UnicoreMessageExtractor
  {
  public:
    /**
      * @brief Extracts messages from a buffer of NovAtel data.
      *
      * This will search through the "input" string for any valid NMEA or NovAtel ASCII
      * messages as well as any binary NovAtel messages, place them into the provided
      * containers, and also provide any leftover bytes at the end of the buffer that
      * were not part of a valid sentence.
      *
      * @param[in] input A buffer of data to search for messages
      * @param[out] nmea_sentences NMEA sentences found in the buffer
      * @param[out] novatel_sentences ASCII NovAtel sentences found in the buffer
      * @param[out] remaining Any bytes left after the last complete sentence
      * in the buffer
      * @param[in] keep_nmea_container "true" to preserve message begin & end markers
      * around parsed NMEA sentences
      * @return false if there were any errors parsing sentences
      */
    bool ExtractCompleteMessages(
        const std::string& input,
        std::vector<NmeaSentence>& nmea_sentences,
        std::vector<NovatelSentence>& novatel_sentences,
        std::string& remaining,
        bool keep_nmea_container = false);

    /**
     * @brief Iterates through the provided messages to find the first GPGGA
     * or GPRMC message with a valid UTC time, then returns it.
     * @param[in] sentences A list of NMEA sentences to search for UTC times.
     * @return The UTC time in seconds, if found; 0 if not found.
     */
    double GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences);


  private:
    // Constants for parsing message structures
    /// Precedes checkums in ASCII messages
    static const std::string CHECKSUM_FLAG;
    /// Separates data fields in ASCII messages
    static const std::string FIELD_SEPARATOR;
    /// Separates header from body in ASCII NovAtel messages
    static const std::string HEADER_SEPARATOR;
    /// Indicates the beginning of a NMEA sentence
    static const std::string NMEA_SENTENCE_FLAG;
    /// Indicates the beginning of an ASCII NovAtel message
    static const std::string NOVATEL_SENTENCE_FLAG;
    /// Used to search for both types of ASCII messages
    static const std::string NOVATEL_ASCII_FLAGS;
    /// Indicates the end of an ASCII message
    static const std::string NOVATEL_ENDLINE;

    /**
     * @brief Searches for a valid ASCII message within a string.
     *
     * What constitutes a valid ASCII message is defined by:
     * http://docs.novatel.com/OEM7/Content/Messages/ASCII.htm
     *
     * We check whether a substring:
     * 1) Starts with '#' or '$'
     * 2) Ends with "\r\n"
     * 3) Between those values, only contains characters with an ASCII value
     *    of 9, 10, 11, 13, or between 32 and 126 (inclusive)
     *
     * @param[in] sentence The string to search within
     * @param[in] current_idx The position in the string to search from
     * @param[out] start_idx The position of the earliest '#' or '$', if found
     * @param[out] end_idx The position of the first '\r' after the start, if "\r\n" was found
     * @param[out] invalid_char_idx If an invalid character was found between the
     *        start and end, its index
     */
    void FindAsciiSentence(const std::string& sentence,
                           size_t current_idx,
                           size_t& start_idx,
                           size_t& end_idx,
                           size_t& invalid_char_idx);

    /**
     * @brief Splits an ASCII NovAtel message up into header and body parts.
     * @param[in] sentence The NovAtel message to parse.
     * @param[out] message_id The message's ID.
     * @param[out] header All of the fields in the message's header.
     * @param[out] body All of the fields in the message's body.
     * @return false if there were errors parsing the message, true otherwise.
     */
    bool GetNovatelMessageParts(
        const std::string& sentence,
        std::string& message_id,
        std::vector<std::string>& header,
        std::vector<std::string>& body);

    /**
     * @brief Extracts an NMEA sentence from an input string.
     * @param[in] str The string to extract an NMEA sentence from.
     * @param[in] start_idx The index to begin searching at.
     * @param[out] sentence If a valid sentence was found, it will be placed here.
     * @param[out] keep_container "true" to keep sentence & checksum position marks
     * in the output sentence.
     * @return 0: Success
     *         -1: Not enough data to parse a complete sentence
     *         1: Sentence's checksum was invalid
     */
    int32_t GetNmeaSentence(
        const std::string& str,
        size_t start_idx,
        std::string& sentence,
        bool keep_container = false);

    /**
     * @brief Extracts a NovAtel sentence from an input string.
     * @param[in] str The string to search for a NovAtel sentence.
     * @param[in] start_idx The index to begin searching from.
     * @param[out] sentence The extracted sentence.
     * @return 0: Success
     *         -1: Not eenough data to parse a complete sentence
     *         1: Checksum was invalid
     */
    int32_t GetNovatelSentence(
        const std::string& str,
        size_t start_idx,
        std::string& sentence);

    /**
     * @brief Finds the location of the next checksum in a valid ASCII sentence.
     * @param str A buffer to search for a checksum.
     * @param start_idx The location to begin searching from.
     * @return The index of that sentence's checksum.
     */
    size_t GetSentenceChecksumStart(
        const std::string& str,
        size_t start_idx);

    /**
     * @brief Calculates the checksum of a NMEA sentence.
     * @param sentence The sentence to check.
     * @return That sentence's checksum.
     */
    uint8_t NmeaChecksum(const std::string& sentence);

    /**
     * @brief Takes a sentence extracted by GetNovatelSentence and converts it
     * into a data structure.
     * @param[in] data A valid, extracted ASCII NovAtel sentence.
     * @param[out] sentence A data structure containing the sentence.
     * @return false if it failed to parse the sentence, true otherwise.
     */
    bool VectorizeNovatelSentence(
        const std::string& data,
        NovatelSentence& sentence);

    /**
     * @brief Takes a sentence extracted by GetNmeaSentence and converts it
     * into a data structure.
     * @param[in] data A valid, extracted ASCII NMEA sentence.
     * @param[out] sentence A data structure containing the sentence.
     * @return false if it failed to parse the sentence, true otherwise.
     */
    void VectorizeNmeaSentence(
        const std::string& sentence,
        NmeaSentence& vectorized_message);

    /**
     * Splits a string into a vector using any of the provided characters
     * as delimiters.
     * @param[in] str The string to parse.
     * @param[out] vectorized_message The split results.
     * @param[in] delimiters A set of characters to use as delimiters.
     */
    void VectorizeString(
        const std::string& str,
        std::vector<std::string>& vectorized_message,
        const std::string& delimiters);
  };
}


#endif //SRC_UNICORE_MESSAGE_EXTRACTOR_HPP
