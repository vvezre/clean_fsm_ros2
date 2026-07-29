#ifndef CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_
#define CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleanbot {
namespace hardware {

struct CommandFields {
  std::uint8_t status{0};
  std::uint8_t power_on{1};
  std::uint8_t hardware_control{0};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t steering_offset{0};
  std::int32_t brush_speed{0};
  std::int32_t target_distance{0};
  std::int32_t target_rotation{0};
  double heading_deg{0.0};
  std::uint16_t sequence{0};
  bool brake{false};
  bool charge{false};
};

class CommandEncoder {
 public:
  static constexpr std::size_t kCommandLength = 21u;

  /// 将结构化车辆控制字段编码成下位机21字节命令帧。
  std::vector<std::uint8_t> encode(const CommandFields& fields) const;

 private:
  /// 将有符号16位数按协议高低字节顺序写入命令帧，并执行范围限制。
  static void writeI16(std::vector<std::uint8_t>& frame, std::size_t index, std::int32_t value);
  /// 将无符号16位数按协议高低字节顺序写入命令帧，并执行范围限制。
  static void writeU16(std::vector<std::uint8_t>& frame, std::size_t index, std::int32_t value);
  /// 计算命令帧协议使用的逐字节异或校验值。
  static std::uint8_t checksum(const std::vector<std::uint8_t>& frame);
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_
