/*
 * 文件作用：实现串口命令的有界稳定优先级队列和连续命令合并。
 *
 * 刹车/安全命令优先于有限动作，有限动作优先于高频连续速度命令。
 * 队首可能正在Boost.Asio异步写入，preserve_front为true时绝不移动或替换它。
 */
#include "cleanbot_hardware/write_queue.hpp"

#include <algorithm>
#include <utility>

namespace cleanbot {
namespace hardware {

// 根据已编码帧中的状态和刹车字段，将重发帧还原为安全、有限或连续优先级。
WritePriority classify_command_frame(const std::vector<std::uint8_t>& frame) {
  if (frame.size() <= 10u) {
    return WritePriority::kFinite;
  }
  const std::uint8_t status = frame[1];
  if (status == 0u) {
    return frame[10] == 0u ? WritePriority::kSafety : WritePriority::kContinuous;
  }
  if (status == 2u || status == 3u || status == 5u) {
    return WritePriority::kFinite;
  }
  return WritePriority::kContinuous;
}

// 初始化有界优先级队列，并保证容量至少为一。
PriorityWriteQueue::PriorityWriteQueue(const std::size_t max_size)
    : max_size_(std::max<std::size_t>(1u, max_size)) {}

// 合并同类连续命令、必要时淘汰低优先级帧，再按优先级稳定插入新帧。
bool PriorityWriteQueue::enqueue(
    const std::vector<std::uint8_t>& frame,
    const WritePriority priority,
    const std::string& coalescing_key,
    const bool preserve_front,
    WriteQueueCallback callback) {
  if (frame.empty()) {
    return false;
  }

  // 正在写入的队首从可修改区排除，后续合并和淘汰只能操作等待项。
  const std::size_t first_mutable = preserve_front && !items_.empty() ? 1u : 0u;
  if (!coalescing_key.empty()) {
    for (std::size_t index = first_mutable; index < items_.size(); ++index) {
      if (items_[index].coalescing_key == coalescing_key) {
        if (items_[index].callback) {
          items_[index].callback(WriteQueueEvent::kSuperseded);
        }
        items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(index));
        break;
      }
    }
  }

  if (items_.size() >= max_size_) {
    // 队列满时只淘汰严格低于新命令优先级的等待项；Safety及以上不会被低级命令挤掉。
    std::size_t discard_index = items_.size();
    WritePriority discard_priority = WritePriority::kCritical;
    for (std::size_t index = first_mutable; index < items_.size(); ++index) {
      if (items_[index].priority >= WritePriority::kSafety ||
          items_[index].priority >= priority) {
        continue;
      }
      if (discard_index == items_.size() || items_[index].priority < discard_priority) {
        discard_index = index;
        discard_priority = items_[index].priority;
      }
    }
    if (discard_index != items_.size()) {
      if (items_[discard_index].callback) {
        items_[discard_index].callback(WriteQueueEvent::kEvicted);
      }
      items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(discard_index));
    } else {
      return false;
    }
  }

  Item item;
  item.frame = std::make_shared<std::vector<std::uint8_t>>(frame);
  item.priority = priority;
  item.coalescing_key = coalescing_key;
  item.callback = std::move(callback);
  // 同优先级保持先来先发；新命令插入到所有同级旧命令之后。
  auto position = items_.begin() + static_cast<std::ptrdiff_t>(first_mutable);
  while (position != items_.end() && position->priority >= priority) {
    ++position;
  }
  items_.insert(position, std::move(item));
  return true;
}

// 返回当前应最先发送的队首帧引用。
const std::vector<std::uint8_t>& PriorityWriteQueue::front() const {
  return *items_.front().frame;
}

// 返回队首共享句柄，确保Boost.Asio异步写入完成前帧不会被释放。
std::shared_ptr<const std::vector<std::uint8_t>> PriorityWriteQueue::front_handle() const {
  return items_.front().frame;
}

// 返回当前异步写入帧对应的结果回调，由串口层在写成功后触发。
WriteQueueCallback PriorityWriteQueue::front_callback() const {
  return items_.empty() ? WriteQueueCallback() : items_.front().callback;
}

// 移除已成功发送的队首帧。
void PriorityWriteQueue::pop_front() {
  if (!items_.empty()) {
    items_.pop_front();
  }
}

// 清空所有尚未发送的协议帧。
void PriorityWriteQueue::clear() { items_.clear(); }

// 判断发送队列是否为空。
bool PriorityWriteQueue::empty() const { return items_.empty(); }

// 返回当前等待发送的帧数量。
std::size_t PriorityWriteQueue::size() const { return items_.size(); }

}  // namespace hardware
}  // namespace cleanbot
