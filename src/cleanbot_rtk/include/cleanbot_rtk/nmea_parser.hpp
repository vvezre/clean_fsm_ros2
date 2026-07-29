#ifndef CLEANBOT_RTK__NMEA_PARSER_HPP_
#define CLEANBOT_RTK__NMEA_PARSER_HPP_

#include <cstdint>
#include <string>

namespace cleanbot {
namespace rtk {

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

struct GgaParseResult {
  bool parsed{false};
  std::string error;
  GgaData data;
};

struct HeadingData {
  double utc_seconds{-1.0};
  double heading_deg{0.0};
  double pitch_deg{0.0};
  bool pitch_valid{false};
  std::string sentence;
};

struct HeadingParseResult {
  bool parsed{false};
  std::string error;
  HeadingData data;
};

class NmeaParser {
 public:
  GgaParseResult parse_gga(const std::string& sentence) const;
  HeadingParseResult parse_heading(const std::string& sentence) const;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NMEA_PARSER_HPP_
