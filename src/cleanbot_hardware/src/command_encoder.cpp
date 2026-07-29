#include "cleanbot_hardware/command_encoder.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace hardware {

namespace {
constexpr std::uint8_t kCommandFrameStart = 0x7b;
constexpr std::uint8_t kCommandFrameEnd = 0x7d;
constexpr std::uint8_t kStopStatus = 0u;
constexpr std::uint8_t kChargeStatus = 5u;
}  // namespace

// 将ROS2控制字段映射到固定21字节命令帧，并统一处理刹车、充电和校验字段。
std::vector<std::uint8_t> CommandEncoder::encode(const CommandFields& fields) const {
  std::vector<std::uint8_t> frame(kCommandLength, 0u);
  frame[0] = kCommandFrameStart;
  frame[1] = fields.brake ? kStopStatus : (fields.charge ? kChargeStatus : fields.status);
  frame[2] = fields.power_on;
  frame[3] = fields.hardware_control;

  if (!fields.brake) {
    writeI16(frame, 4u, fields.x_speed);
    writeI16(frame, 6u, fields.z_speed);
    writeU16(frame, 8u, std::min(fields.target_distance, 50000));
    const auto brush = std::max(-100, std::min(100, fields.brush_speed));
    frame[10] = static_cast<std::uint8_t>(static_cast<std::int8_t>(brush));
    writeI16(frame, 11u, fields.steering_offset);
    writeI16(frame, 13u, fields.target_rotation);

    double normalized_heading = std::fmod(fields.heading_deg, 360.0);
    if (normalized_heading < 0.0) {
      normalized_heading += 360.0;
    }
    const auto heading_centidegrees =
        static_cast<std::int32_t>(std::round(normalized_heading * 100.0)) % 36000;
    writeU16(frame, 15u, heading_centidegrees);
  }

  writeU16(frame, 17u, fields.sequence);
  frame[19] = checksum(frame);
  frame[20] = kCommandFrameEnd;
  return frame;
}

// 将输入限制在int16范围后，按高字节在前的协议格式写入有符号补码。
void CommandEncoder::writeI16(
    std::vector<std::uint8_t>& frame, const std::size_t index, const std::int32_t value) {
  const auto clamped = std::max(-32768, std::min(32767, value));
  const auto encoded = static_cast<std::uint16_t>(static_cast<std::int16_t>(clamped));
  frame[index] = static_cast<std::uint8_t>((encoded >> 8u) & 0xffu);
  frame[index + 1u] = static_cast<std::uint8_t>(encoded & 0xffu);
}

// 将输入限制在uint16范围后，按高字节在前的协议格式写入无符号数。
void CommandEncoder::writeU16(
    std::vector<std::uint8_t>& frame, const std::size_t index, const std::int32_t value) {
  const auto clamped = std::max(0, std::min(65535, value));
  const auto encoded = static_cast<std::uint16_t>(clamped);
  frame[index] = static_cast<std::uint8_t>((encoded >> 8u) & 0xffu);
  frame[index + 1u] = static_cast<std::uint8_t>(encoded & 0xffu);
}

// 对命令帧校验字段之前的字节执行异或，得到协议校验值。
std::uint8_t CommandEncoder::checksum(const std::vector<std::uint8_t>& frame) {
  std::uint8_t value = 0u;
  for (std::size_t index = 0u; index < 19u; ++index) {
    value = static_cast<std::uint8_t>(value ^ frame[index]);
  }
  return value;
}

}  // namespace hardware
}  // namespace cleanbot
