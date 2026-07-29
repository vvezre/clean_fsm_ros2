#include "cleanbot_hardware/frame_buffer.hpp"

namespace cleanbot {
namespace hardware {

// 初始化串口接收缓存容量；容量只限制未解析的历史字节，不限制单次读取长度。
FrameBuffer::FrameBuffer(const std::size_t capacity) : capacity_(capacity) {}

// 追加新字节并及时裁剪，防止持续噪声导致缓存无限增长。
void FrameBuffer::append(const std::vector<std::uint8_t>& data) {
  if (data.empty()) {
    return;
  }
  data_.insert(data_.end(), data.begin(), data.end());
  trim();
}

// 循环寻找合法帧头和固定帧尾，支持分包、粘包、前导噪声及错误帧重新同步。
bool FrameBuffer::pop(std::vector<std::uint8_t>& frame) {
  frame.clear();
  while (true) {
    std::size_t first_start = 0u;
    while (first_start < data_.size() &&
           data_[first_start] != kStatusFrameStart &&
           data_[first_start] != kAckFrameStart) {
      ++first_start;
    }
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(first_start));
    if (data_.empty()) {
      return false;
    }

    const bool status_frame = data_.front() == kStatusFrameStart;
    const std::size_t frame_length = status_frame ? kStatusFrameLength : kAckFrameLength;
    const std::uint8_t frame_end = status_frame ? kStatusFrameEnd : kAckFrameEnd;
    if (data_.size() < frame_length) {
      return false;
    }

    if (data_[frame_length - 1u] == frame_end) {
      frame.assign(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
      data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
      return true;
    }

    // The byte at the fixed tail position is invalid. Drop only this candidate
    // start and search again, preserving every other buffered byte.
    data_.erase(data_.begin());
  }
}

// 返回尚未组成完整帧的缓存字节数。
std::size_t FrameBuffer::size() const { return data_.size(); }

// 丢弃全部未解析字节，避免断线前残帧影响重连后的新数据。
void FrameBuffer::clear() { data_.clear(); }

// 缓存超限时优先保留最近候选帧头之后的数据，再执行硬容量截断。
void FrameBuffer::trim() {
  if (data_.size() <= capacity_) {
    return;
  }

  std::size_t start_index = data_.size();
  for (std::size_t index = data_.size(); index > 0u; --index) {
    if (data_[index - 1u] == kStatusFrameStart || data_[index - 1u] == kAckFrameStart) {
      start_index = index - 1u;
      break;
    }
  }
  if (start_index < data_.size()) {
    if (start_index > 0u) {
      data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(start_index));
    }
  }

  if (data_.size() > capacity_) {
    const auto remove_count = data_.size() - capacity_;
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(remove_count));
  }
}

}  // namespace hardware
}  // namespace cleanbot
