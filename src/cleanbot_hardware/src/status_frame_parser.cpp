#include "cleanbot_hardware/status_frame_parser.hpp"

#include <cmath>

namespace cleanbot {
namespace hardware {

namespace {
constexpr std::uint8_t kFrameStart = 0x7b;
constexpr std::uint8_t kFrameEnd = 0x7d;
constexpr std::uint8_t kFinished = 0xbb;
constexpr std::size_t kFrameLength = 23u;
}  // namespace

// 校验23字节状态帧并按协议偏移解析速度、硬件、完成事件和里程等字段。
StatusParseResult StatusFrameParser::parse(const std::vector<std::uint8_t>& frame) const {
  StatusParseResult result;
  if (frame.empty() || frame.front() != kFrameStart) {
    result.error = "frame_start_missing";
    return result;
  }
  if (frame.size() != kFrameLength) {
    result.error = "frame_length_invalid";
    return result;
  }
  if (frame.back() != kFrameEnd) {
    result.error = "frame_end_missing";
    return result;
  }

  result.status.full_frame = true;
  result.status.status = frame[1];
  result.status.power_on = frame[2];
  result.status.hardware_state = frame[3];
  result.status.x_speed_raw = readU16(frame, 4u);
  result.status.z_speed_raw = readU16(frame, 6u);
  result.status.x_speed = readI16(frame, 4u);
  result.status.z_speed = readI16(frame, 6u);
  result.status.brush_speed = frame[8];
  result.status.edge_clear = frame[9] == 0x00;
  result.status.battery_percent = static_cast<double>(frame[10]);
  result.status.air_state = frame[11];
  result.status.move_finished = frame[12] == kFinished;
  result.status.rotate_finished = frame[13] == kFinished;
  result.status.pack_voltage = std::round(static_cast<double>(readU16(frame, 14u)) * 0.1) / 10.0;
  result.status.angle = readU16(frame, 16u);
  result.status.odometer = readU16(frame, 18u);
  result.status.command_sequence = readU16(frame, 20u);
  result.parsed = true;
  return result;
}

// 读取大端16位数据，并按二进制补码解释成有符号速度值。
std::int16_t StatusFrameParser::readI16(
    const std::vector<std::uint8_t>& frame, const std::size_t index) {
  const std::uint16_t raw = readU16(frame, index);
  const std::int32_t signed_value = raw >= 0x8000u
      ? static_cast<std::int32_t>(raw) - 0x10000
      : static_cast<std::int32_t>(raw);
  return static_cast<std::int16_t>(signed_value);
}

// 从指定偏移读取大端无符号16位数据。
std::uint16_t StatusFrameParser::readU16(
    const std::vector<std::uint8_t>& frame, const std::size_t index) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame[index]) << 8u) |
      static_cast<std::uint16_t>(frame[index + 1u]));
}

}  // namespace hardware
}  // namespace cleanbot
