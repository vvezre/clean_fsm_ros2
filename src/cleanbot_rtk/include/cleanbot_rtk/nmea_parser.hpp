#ifndef CLEANBOT_RTK__NMEA_PARSER_HPP_
#define CLEANBOT_RTK__NMEA_PARSER_HPP_

#include <cstdint>
#include <string>

// 文件作用：声明 GGA 定位句和航向 NMEA 句的解析结果及解析器接口。
namespace cleanbot {
namespace rtk {

// 从 GGA 句提取的定位质量、坐标和卫星信息。
struct GgaData {
  double utc_seconds{-1.0};
  double lat{0.0};
  double lon{0.0};
  std::uint8_t fix_quality{0};
  std::uint8_t satellite_count{0};
  double hdop{0.0};
  bool position_valid{false};
  std::string sentence;
};

// GGA 句解析是否成功、失败原因和解析数据。
struct GgaParseResult {
  bool parsed{false};
  std::string error;
  GgaData data;
};

// 从航向句提取的航向、俯仰和时间信息。
struct HeadingData {
  double utc_seconds{-1.0};
  double heading_deg{0.0};
  double pitch_deg{0.0};
  bool pitch_valid{false};
  std::string sentence;
};

// 航向句解析是否成功、失败原因和解析数据。
struct HeadingParseResult {
  bool parsed{false};
  std::string error;
  HeadingData data;
};

class NmeaParser {
 public:
  // 解析一条 GGA 定位语句。
  GgaParseResult parse_gga(const std::string& sentence) const;
  // 解析一条接收机航向语句。
  HeadingParseResult parse_heading(const std::string& sentence) const;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NMEA_PARSER_HPP_
