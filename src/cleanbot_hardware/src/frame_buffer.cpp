/*
 * 文件作用：把异步串口读取产生的字节流重组为固定23字节旧状态帧。
 *
 * 支持半帧、连续多帧、帧前噪声和错误候选帧重新同步。
 * 这里只识别固定帧头、长度和尾部位置，字段合法性由StatusFrameParser负责。
 */
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
    while (first_start < data_.size() && data_[first_start] != kStatusFrameStart) {
      ++first_start;
    }
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(first_start));
    if (data_.empty()) {
      return false;
    }

    if (data_.size() < kStatusFrameLength) {
      return false;
    }

    if (data_[kStatusFrameLength - 1u] == kStatusFrameEnd) {
      frame.assign(
          data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(kStatusFrameLength));
      data_.erase(
          data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(kStatusFrameLength));
      return true;
    }

    // 固定尾位置无效时只删除当前候选帧头，保留其余字节继续寻找下一帧头。
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
    if (data_[index - 1u] == kStatusFrameStart) {
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
