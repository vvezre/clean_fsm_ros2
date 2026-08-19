/*
 * 文件作用：在串口任意分包、粘包和噪声字节中重组固定23字节状态帧。
 *
 * 输入：Boost.Asio每次读到的任意长度原始字节。
 * 输出：以0x7B开头、固定位置0x7D结尾的候选状态帧。
 * 边界：只负责定长重组，不解释字段，也不替代StatusFrameParser的语义校验。
 */
#ifndef CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_
#define CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleanbot {
namespace hardware {

// 有容量上限的流式帧缓冲，防止串口噪声造成内存无限增长。
class FrameBuffer {
 public:
  static constexpr std::uint8_t kStatusFrameStart = 0x7b;
  static constexpr std::uint8_t kStatusFrameEnd = 0x7d;
  static constexpr std::size_t kStatusFrameLength = 23u;

  // 创建有容量上限的串口接收缓存，并尽量保留可能的帧头。
  explicit FrameBuffer(std::size_t capacity = 512u);

  // 追加串口原始字节，支持噪声、分包以及一次读取多帧。
  void append(const std::vector<std::uint8_t>& data);

  // 如果缓存中存在完整数据，则取出一帧严格23字节的旧协议状态帧。
  bool pop(std::vector<std::uint8_t>& frame);

  // 返回当前尚未组成完整帧的字节数。
  std::size_t size() const;
  // 清除残帧；串口断开和重新连接时必须调用。
  void clear();

 private:
  // 超过容量时优先保留最近帧头之后的数据，再执行硬截断。
  void trim();

  std::size_t capacity_;
  std::vector<std::uint8_t> data_;
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__FRAME_BUFFER_HPP_
