#include "cleanbot_hardware/ack_codec.hpp"

namespace cleanbot {
namespace hardware {

// 解析下位机A1/A2帧：先验证固定格式，再提取sequence、主题类型和处理结果。
AckParseResult AckCodec::parse(const std::vector<std::uint8_t>& frame) const {
  AckParseResult result;
  if (frame.size() != kFrameLength) {
    result.error = "ack_length_invalid";
    return result;
  }
  if (frame.front() != kFrameStart || frame.back() != kFrameEnd) {
    result.error = "ack_boundary_invalid";
    return result;
  }
  if (frame[1] != kCommandAck && frame[1] != kCompletionAck) {
    result.error = "ack_type_invalid";
    return result;
  }
  if (checksum(frame) != frame[6]) {
    result.error = "ack_checksum_invalid";
    return result;
  }

  result.ack.frame_type = frame[1];
  result.ack.sequence = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame[2]) << 8u) |
      static_cast<std::uint16_t>(frame[3]));
  result.ack.subject_type = frame[4];
  result.ack.result = frame[5];
  result.parsed = true;
  return result;
}

// 生成上位机A2完成确认帧，使下位机停止重复上报同一完成事件。
std::vector<std::uint8_t> AckCodec::encode_completion_ack(
    const std::uint16_t sequence, const std::uint8_t event_type) const {
  std::vector<std::uint8_t> frame(kFrameLength, 0u);
  frame[0] = kFrameStart;
  frame[1] = kCompletionAck;
  frame[2] = static_cast<std::uint8_t>((sequence >> 8u) & 0xffu);
  frame[3] = static_cast<std::uint8_t>(sequence & 0xffu);
  frame[4] = event_type;
  frame[5] = kAccepted;
  frame[6] = checksum(frame);
  frame[7] = kFrameEnd;
  return frame;
}

// 对校验字段之前的字节执行异或，得到协议校验值。
std::uint8_t AckCodec::checksum(const std::vector<std::uint8_t>& frame) {
  std::uint8_t value = 0u;
  for (std::size_t index = 0u; index < 6u; ++index) {
    value = static_cast<std::uint8_t>(value ^ frame[index]);
  }
  return value;
}

}  // namespace hardware
}  // namespace cleanbot
