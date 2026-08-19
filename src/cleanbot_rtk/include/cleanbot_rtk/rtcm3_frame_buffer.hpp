#ifndef CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_
#define CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

// 文件作用：声明 RTCM3 字节流的帧边界识别、CRC 校验和有界缓存。
namespace cleanbot {
namespace rtk {

class Rtcm3FrameBuffer {
 public:
  // 创建最大缓存长度受限的 RTCM3 帧缓存器。
  explicit Rtcm3FrameBuffer(std::size_t max_buffer_size = 8192u);

  // 追加从 NTRIP 或串口接收的原始 RTCM 字节。
  void append(const std::vector<std::uint8_t>& bytes);
  // 取出一帧 CRC 合法的 RTCM3 数据。
  bool pop(std::vector<std::uint8_t>& frame);
  // 清空缓存和待处理数据。
  void clear();
  // 返回当前待处理字节数。
  std::size_t size() const;
  // 返回累计丢弃的 CRC 错误帧数量。
  std::size_t invalid_crc_count() const;

 private:
  // 计算指定前缀字节的 RTCM CRC-24Q 校验值。
  static std::uint32_t crc24q(
      const std::vector<std::uint8_t>& bytes, std::size_t length);

  std::size_t max_buffer_size_;
  std::size_t invalid_crc_count_{0u};
  std::vector<std::uint8_t> data_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_
