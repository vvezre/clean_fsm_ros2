/*
 * 文件作用：NMEA解析实现：校验并提取定位语句中的时间、坐标和质量字段。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/nmea_parser.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

namespace cleanbot {
namespace rtk {

namespace {

// 将单个十六进制字符转换为数值，非法字符返回负一。
int hexValue(const char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  return -1;
}

// 校验 NMEA 起始符与异或校验和，并提取不含包装符的载荷。
bool validateAndExtractPayload(
    const std::string& sentence, std::string& payload, std::string& error) {
  if (sentence.empty() || sentence[0] != '$') {
    error = "nmea_start_invalid";
    return false;
  }
  std::size_t star = 1u;
  while (star < sentence.size() && sentence[star] != '*') {
    ++star;
  }
  if (star == sentence.size() || star + 2u >= sentence.size()) {
    error = "nmea_checksum_missing";
    return false;
  }
  const int high = hexValue(sentence[star + 1u]);
  const int low = hexValue(sentence[star + 2u]);
  if (high < 0 || low < 0) {
    error = "nmea_checksum_invalid";
    return false;
  }
  std::uint8_t checksum = 0u;
  for (std::size_t index = 1u; index < star; ++index) {
    checksum = static_cast<std::uint8_t>(
        checksum ^ static_cast<std::uint8_t>(sentence[index]));
  }
  if (checksum != static_cast<std::uint8_t>((high << 4) | low)) {
    error = "nmea_checksum_invalid";
    return false;
  }
  payload = sentence.substr(1u, star - 1u);
  return true;
}

// 按逗号切分 NMEA 字段，同时保留空字段的位置。
std::vector<std::string> splitFields(const std::string& payload) {
  std::vector<std::string> fields;
  std::size_t start = 0u;
  for (std::size_t index = 0u; index <= payload.size(); ++index) {
    if (index == payload.size() || payload[index] == ',') {
      fields.push_back(payload.substr(start, index - start));
      start = index + 1u;
    }
  }
  return fields;
}

// 严格解析有限浮点数，拒绝空串和尾随字符。
bool parseDouble(const std::string& text, double& value) {
  if (text.empty()) {
    return false;
  }
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0' && std::isfinite(value);
}

// 解析可由 int 表示的非负整数文本。
bool parseUnsigned(const std::string& text, int& value) {
  double parsed = 0.0;
  if (!parseDouble(text, parsed) || parsed < 0.0 || std::floor(parsed) != parsed) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

// 将 NMEA 的时分秒字段转换为当天累计秒数。
bool parseUtc(const std::string& text, double& seconds) {
  double raw = 0.0;
  if (!parseDouble(text, raw)) {
    return false;
  }
  const int hour = static_cast<int>(raw / 10000.0);
  const int minute = static_cast<int>((raw - hour * 10000.0) / 100.0);
  const double second = raw - hour * 10000.0 - minute * 100.0;
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
      second < 0.0 || second >= 61.0) {
    return false;
  }
  seconds = hour * 3600.0 + minute * 60.0 + second;
  return true;
}

// 将度分格式坐标和半球标志转换为带符号十进制度。
bool parseCoordinate(
    const std::string& raw_text,
    const std::string& hemisphere,
    const bool latitude,
    double& degrees) {
  double raw = 0.0;
  if (!parseDouble(raw_text, raw) || hemisphere.size() != 1u) {
    return false;
  }
  const int whole_degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - whole_degrees * 100.0;
  const int max_degrees = latitude ? 90 : 180;
  if (whole_degrees < 0 || whole_degrees > max_degrees ||
      minutes < 0.0 || minutes >= 60.0) {
    return false;
  }
  degrees = whole_degrees + minutes / 60.0;
  const char direction = hemisphere[0];
  if ((latitude && direction == 'S') || (!latitude && direction == 'W')) {
    degrees = -degrees;
  } else if ((latitude && direction != 'N') || (!latitude && direction != 'E')) {
    return false;
  }
  return std::abs(degrees) <= static_cast<double>(max_degrees);
}

}  // namespace

// 校验校验和并解析 GGA 定位坐标、时间和固定解质量。
GgaParseResult NmeaParser::parse_gga(const std::string& sentence) const {
  GgaParseResult result;
  std::string payload;
  if (!validateAndExtractPayload(sentence, payload, result.error)) {
    return result;
  }
  const auto fields = splitFields(payload);
  if (fields.size() < 9u || (fields[0] != "GNGGA" && fields[0] != "GPGGA")) {
    result.error = "gga_fields_invalid";
    return result;
  }
  if (!parseUtc(fields[1], result.data.utc_seconds)) {
    result.error = "gga_utc_invalid";
    return result;
  }
  int quality = 0;
  int satellites = 0;
  if (!parseUnsigned(fields[6], quality) || quality > 255 ||
      !parseUnsigned(fields[7], satellites) || satellites > 255 ||
      !parseDouble(fields[8], result.data.hdop) || result.data.hdop < 0.0) {
    result.error = "gga_quality_invalid";
    return result;
  }
  result.data.fix_quality = static_cast<std::uint8_t>(quality);
  result.data.satellite_count = static_cast<std::uint8_t>(satellites);
  result.data.sentence = sentence;

  const bool coordinate_fields_empty =
      fields[2].empty() && fields[3].empty() && fields[4].empty() && fields[5].empty();
  if (coordinate_fields_empty && quality == 0) {
    result.parsed = true;
    return result;
  }
  if (!parseCoordinate(fields[2], fields[3], true, result.data.lat) ||
      !parseCoordinate(fields[4], fields[5], false, result.data.lon)) {
    result.error = "gga_coordinate_invalid";
    return result;
  }
  result.data.position_valid = quality > 0;
  result.parsed = true;
  return result;
}

// 校验并解析接收机航向句中的航向和可选俯仰角。
HeadingParseResult NmeaParser::parse_heading(const std::string& sentence) const {
  HeadingParseResult result;
  std::string payload;
  if (!validateAndExtractPayload(sentence, payload, result.error)) {
    return result;
  }
  const auto fields = splitFields(payload);
  if (fields.empty()) {
    result.error = "heading_fields_invalid";
    return result;
  }
  if (fields[0] == "GNHPR" || fields[0] == "GPHPR") {
    if (fields.size() < 4u || !parseUtc(fields[1], result.data.utc_seconds) ||
        !parseDouble(fields[2], result.data.heading_deg) ||
        !parseDouble(fields[3], result.data.pitch_deg)) {
      result.error = "hpr_fields_invalid";
      return result;
    }
    result.data.pitch_valid = true;
  } else if (fields[0] == "GNTHS" || fields[0] == "GPTHS") {
    if (fields.size() < 3u || fields[2] != "A" ||
        !parseDouble(fields[1], result.data.heading_deg)) {
      result.error = "ths_fields_invalid";
      return result;
    }
  } else {
    result.error = "heading_type_invalid";
    return result;
  }
  if (result.data.heading_deg < 0.0 || result.data.heading_deg >= 360.0) {
    result.error = "heading_range_invalid";
    return result;
  }
  result.data.sentence = sentence;
  result.parsed = true;
  return result;
}

}  // namespace rtk
}  // namespace cleanbot
