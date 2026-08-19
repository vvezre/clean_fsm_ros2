/*
 * 文件作用：RTCM3帧缓冲实现：按长度和CRC24Q重组差分数据帧。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/rtcm3_frame_buffer.hpp"

namespace cleanbot {
namespace rtk {

namespace {
constexpr std::uint8_t kRtcm3Preamble = 0xd3u;
constexpr std::size_t kHeaderLength = 3u;
constexpr std::size_t kCrcLength = 3u;
}  // namespace

// 以给定最大字节数创建 RTCM3 流帧缓存。
Rtcm3FrameBuffer::Rtcm3FrameBuffer(const std::size_t max_buffer_size)
    : max_buffer_size_(max_buffer_size < kHeaderLength + kCrcLength
          ? kHeaderLength + kCrcLength
          : max_buffer_size) {}

// 追加网络或串口来源的 RTCM 原始字节。
void Rtcm3FrameBuffer::append(const std::vector<std::uint8_t>& bytes) {
  if (bytes.empty()) {
    return;
  }
  data_.insert(data_.end(), bytes.begin(), bytes.end());
  if (data_.size() > max_buffer_size_) {
    data_.erase(
        data_.begin(),
        data_.begin() + static_cast<std::ptrdiff_t>(data_.size() - max_buffer_size_));
  }
}

// 搜索完整 RTCM3 帧，校验 CRC 后输出合法帧。
bool Rtcm3FrameBuffer::pop(std::vector<std::uint8_t>& frame) {
  while (true) {
    std::size_t start_index = 0u;
    while (start_index < data_.size() && data_[start_index] != kRtcm3Preamble) {
      ++start_index;
    }
    if (start_index == data_.size()) {
      data_.clear();
      return false;
    }
    if (start_index > 0u) {
      data_.erase(
          data_.begin(),
          data_.begin() + static_cast<std::ptrdiff_t>(start_index));
    }
    if (data_.size() < kHeaderLength) {
      return false;
    }
    if ((data_[1] & 0xfcu) != 0u) {
      data_.erase(data_.begin());
      continue;
    }

    const std::size_t payload_length =
        (static_cast<std::size_t>(data_[1] & 0x03u) << 8u) |
        static_cast<std::size_t>(data_[2]);
    const std::size_t frame_length = kHeaderLength + payload_length + kCrcLength;
    if (data_.size() < frame_length) {
      return false;
    }

    const std::uint32_t expected_crc =
        (static_cast<std::uint32_t>(data_[frame_length - 3u]) << 16u) |
        (static_cast<std::uint32_t>(data_[frame_length - 2u]) << 8u) |
        static_cast<std::uint32_t>(data_[frame_length - 1u]);
    if (crc24q(data_, frame_length - kCrcLength) != expected_crc) {
      ++invalid_crc_count_;
      data_.erase(data_.begin());
      continue;
    }

    frame.assign(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
    return true;
  }
}

// 清空待解析 RTCM 字节。
void Rtcm3FrameBuffer::clear() { data_.clear(); }

// 返回当前缓存的字节数。
std::size_t Rtcm3FrameBuffer::size() const { return data_.size(); }

// 返回累计检测到的 CRC 错误帧数量。
std::size_t Rtcm3FrameBuffer::invalid_crc_count() const {
  return invalid_crc_count_;
}

// 计算 RTCM3 帧使用的 CRC-24Q 校验值。
std::uint32_t Rtcm3FrameBuffer::crc24q(
    const std::vector<std::uint8_t>& bytes, const std::size_t length) {
  std::uint32_t crc = 0u;
  const std::size_t end = length < bytes.size() ? length : bytes.size();
  for (std::size_t index = 0u; index < end; ++index) {
    crc ^= static_cast<std::uint32_t>(bytes[index]) << 16u;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1u;
      if ((crc & 0x1000000u) != 0u) {
        crc ^= 0x1864cfbu;
      }
    }
  }
  return crc & 0x00ffffffu;
}

}  // namespace rtk
}  // namespace cleanbot
