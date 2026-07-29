#ifndef CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_
#define CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace hardware {

struct HardwareStatusPatch {
  bool full_frame{false};
  std::uint8_t status{0};
  std::uint8_t power_on{0};
  std::uint8_t hardware_state{0};
  std::uint16_t x_speed_raw{0};
  std::uint16_t z_speed_raw{0};
  std::int16_t x_speed{0};
  std::int16_t z_speed{0};
  std::uint8_t brush_speed{0};
  bool edge_clear{false};
  double battery_percent{0.0};
  std::uint8_t air_state{0};
  bool move_finished{false};
  bool rotate_finished{false};
  double pack_voltage{0.0};
  std::uint16_t angle{0};
  std::uint16_t odometer{0};
  std::uint16_t command_sequence{0};
};

struct StatusParseResult {
  bool parsed{false};
  std::string error;
  HardwareStatusPatch status;
};

class StatusFrameParser {
 public:
  /// 校验并解析固定23字节下位机状态帧，返回结构化硬件状态或错误原因。
  StatusParseResult parse(const std::vector<std::uint8_t>& frame) const;

 private:
  /// 从指定偏移读取一个协议大端无符号16位整数。
  static std::uint16_t readU16(const std::vector<std::uint8_t>& frame, std::size_t index);
  /// 从指定偏移读取一个协议大端有符号16位补码整数。
  static std::int16_t readI16(const std::vector<std::uint8_t>& frame, std::size_t index);
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_
