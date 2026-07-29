#ifndef CLEANBOT_HARDWARE__ACK_CODEC_HPP_
#define CLEANBOT_HARDWARE__ACK_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace hardware {

struct AckFrame {
  std::uint8_t frame_type{0};
  std::uint16_t sequence{0};
  std::uint8_t subject_type{0};
  std::uint8_t result{0};
};

struct AckParseResult {
  bool parsed{false};
  std::string error;
  AckFrame ack;
};

class AckCodec {
 public:
  static constexpr std::uint8_t kFrameStart = 0x7c;
  static constexpr std::uint8_t kCommandAck = 0xa1;
  static constexpr std::uint8_t kCompletionAck = 0xa2;
  static constexpr std::uint8_t kAccepted = 0x00;
  static constexpr std::uint8_t kFrameEnd = 0x7e;
  static constexpr std::size_t kFrameLength = 8u;

  /// 解析下位机上报的固定8字节ACK帧，并校验帧头、帧尾、类型和校验和。
  AckParseResult parse(const std::vector<std::uint8_t>& frame) const;
  /// 编码A2完成确认帧，通知下位机上位机已收到指定命令的完成事件。
  std::vector<std::uint8_t> encode_completion_ack(
      std::uint16_t sequence, std::uint8_t event_type) const;

 private:
  /// 计算ACK帧协议使用的逐字节异或校验值。
  static std::uint8_t checksum(const std::vector<std::uint8_t>& frame);
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__ACK_CODEC_HPP_
