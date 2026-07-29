#include "cleanbot_hardware/command_lifecycle.hpp"

#include <algorithm>

namespace cleanbot {
namespace hardware {

// 保存命令可靠性参数，并保证历史容量至少可以容纳一条命令。
CommandLifecycle::CommandLifecycle(
    const std::uint64_t ack_timeout_ms,
    const std::size_t max_retries,
    const std::size_t history_size)
    : ack_timeout_ms_(std::max<std::uint64_t>(1u, ack_timeout_ms)),
      max_retries_(max_retries),
      history_size_(std::max<std::size_t>(1u, history_size)) {}

// 登记新命令：连续命令只保留最新值，有限命令保存原帧等待ACK和重试。
void CommandLifecycle::track(
    const std::uint16_t sequence,
    const std::uint8_t command_type,
    const std::vector<std::uint8_t>& frame,
    const std::uint64_t now_ms,
    const CommandDeliveryPolicy policy) {
  if (policy == CommandDeliveryPolicy::kLatestOnly) {
    for (auto pending = pending_.begin(); pending != pending_.end();) {
      if (pending->second.policy == CommandDeliveryPolicy::kLatestOnly) {
        pending = pending_.erase(pending);
      } else {
        ++pending;
      }
    }
  }
  PendingCommand pending;
  pending.command_type = command_type;
  pending.frame = frame;
  pending.next_retry_at_ms = now_ms + ack_timeout_ms_;
  pending.policy = policy;
  pending_[sequence] = pending;
  remember(sequence, command_type);
}

// 用sequence和命令类型匹配待确认记录，匹配后结束等待并保存到历史。
AckDisposition CommandLifecycle::handle_command_ack(const AckFrame& ack) {
  if (ack.frame_type != AckCodec::kCommandAck) {
    return AckDisposition::kUnmatched;
  }
  const auto pending = pending_.find(ack.sequence);
  if (pending == pending_.end() || pending->second.command_type != ack.subject_type) {
    return AckDisposition::kUnmatched;
  }
  pending_.erase(pending);
  return ack.result == AckCodec::kAccepted
      ? AckDisposition::kAccepted
      : AckDisposition::kRejected;
}

// 扫描待确认命令，输出到期重发帧，并移除超过最大重试次数的命令。
RetryBatch CommandLifecycle::collect_due_retries(const std::uint64_t now_ms) {
  RetryBatch batch;
  for (auto pending = pending_.begin(); pending != pending_.end();) {
    if (now_ms < pending->second.next_retry_at_ms) {
      ++pending;
      continue;
    }
    if (pending->second.retries_sent < max_retries_) {
      batch.frames.push_back(pending->second.frame);
      ++pending->second.retries_sent;
      pending->second.next_retry_at_ms = now_ms + ack_timeout_ms_;
      ++pending;
      continue;
    }
    batch.timed_out_sequences.push_back(pending->first);
    batch.timed_out_commands.push_back(
        TimedOutCommand{pending->first, pending->second.command_type});
    pending = pending_.erase(pending);
  }
  return batch;
}

// 返回并清空尚未收到ACK的命令，让节点能够逐条发布传输中断状态。
std::vector<InterruptedCommand> CommandLifecycle::clear_pending() {
  std::vector<InterruptedCommand> interrupted;
  interrupted.reserve(pending_.size());
  for (const auto& item : pending_) {
    interrupted.push_back(InterruptedCommand{
        item.first,
        item.second.command_type,
        item.second.policy});
  }
  pending_.clear();
  return interrupted;
}

// 取消指定待确认命令；命令已经结束或不存在时返回false。
bool CommandLifecycle::cancel_pending(const std::uint16_t sequence) {
  return pending_.erase(sequence) != 0u;
}

// 对完成事件执行幂等判断：未知sequence拒绝，已见事件判重，首次事件登记。
CompletionDisposition CommandLifecycle::observe_completion(
    const std::uint16_t sequence, const std::uint8_t event_type) {
  if (history_.find(sequence) == history_.end()) {
    return CompletionDisposition::kUnknownCommand;
  }
  const auto inserted = completions_.insert(completionKey(sequence, event_type));
  return inserted.second
      ? CompletionDisposition::kFirstSeen
      : CompletionDisposition::kDuplicate;
}

// 将16位sequence和8位事件类型组合成无冲突的32位去重键。
std::uint32_t CommandLifecycle::completionKey(
    const std::uint16_t sequence, const std::uint8_t event_type) {
  return (static_cast<std::uint32_t>(sequence) << 8u) |
      static_cast<std::uint32_t>(event_type);
}

// 将确认后的命令加入有界历史，并同步清理该sequence对应的旧完成事件。
void CommandLifecycle::remember(
    const std::uint16_t sequence, const std::uint8_t command_type) {
  if (history_.find(sequence) != history_.end()) {
    history_order_.erase(
        std::remove(history_order_.begin(), history_order_.end(), sequence),
        history_order_.end());
    for (auto completion = completions_.begin(); completion != completions_.end();) {
      if ((*completion >> 8u) == sequence) {
        completion = completions_.erase(completion);
      } else {
        ++completion;
      }
    }
  }
  history_[sequence] = command_type;
  history_order_.push_back(sequence);
  while (history_order_.size() > history_size_) {
    const auto expired = history_order_.front();
    history_order_.pop_front();
    history_.erase(expired);
    for (auto completion = completions_.begin(); completion != completions_.end();) {
      if ((*completion >> 8u) == expired) {
        completion = completions_.erase(completion);
      } else {
        ++completion;
      }
    }
  }
}

}  // namespace hardware
}  // namespace cleanbot
