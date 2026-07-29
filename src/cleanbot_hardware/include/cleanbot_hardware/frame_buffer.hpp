#ifndef CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_
#define CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleanbot {
namespace hardware {

class FrameBuffer {
 public:
  static constexpr std::uint8_t kStatusFrameStart = 0x7b;
  static constexpr std::uint8_t kStatusFrameEnd = 0x7d;
  static constexpr std::size_t kStatusFrameLength = 23u;
  static constexpr std::uint8_t kAckFrameStart = 0x7c;
  static constexpr std::uint8_t kAckFrameEnd = 0x7e;
  static constexpr std::size_t kAckFrameLength = 8u;

  /// 创建串口接收缓冲区，并设置允许保留的最大字节数。
  explicit FrameBuffer(std::size_t capacity = 512u);

  /// 追加一次串口读取到的原始字节，并在超限时保留最有可能的有效帧起点。
  void append(const std::vector<std::uint8_t>& data);
  /// 从缓存中提取一帧完整状态帧或ACK帧，同时处理噪声、分包和错误帧重同步。
  bool pop(std::vector<std::uint8_t>& frame);
  /// 返回当前尚未解析的缓存字节数量。
  std::size_t size() const;
  /// 清空全部缓存字节，通常在串口关闭或重连时调用。
  void clear();

 private:
  /// 将缓存限制在容量内，并优先从最近的候选帧头开始保留数据。
  void trim();

  std::size_t capacity_;
  std::vector<std::uint8_t> data_;
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_
