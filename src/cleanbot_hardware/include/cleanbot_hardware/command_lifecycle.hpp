#ifndef CLEANBOT_HARDWARE__COMMAND_LIFECYCLE_HPP_
#define CLEANBOT_HARDWARE__COMMAND_LIFECYCLE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cleanbot_hardware/ack_codec.hpp"

namespace cleanbot {
namespace hardware {

enum class AckDisposition {
  kUnmatched = 0,
  kAccepted = 1,
  kRejected = 2,
};

enum class CompletionDisposition {
  kUnknownCommand = 0,
  kFirstSeen = 1,
  kDuplicate = 2,
};

enum class CommandDeliveryPolicy {
  kRetryUntilAck = 0,
  kLatestOnly = 1,
};

struct TimedOutCommand {
  std::uint16_t sequence{0};
  std::uint8_t command_type{0};
};

struct InterruptedCommand {
  std::uint16_t sequence{0};
  std::uint8_t command_type{0};
  CommandDeliveryPolicy policy{CommandDeliveryPolicy::kRetryUntilAck};
};

struct RetryBatch {
  std::vector<std::vector<std::uint8_t>> frames;
  std::vector<std::uint16_t> timed_out_sequences;
  std::vector<TimedOutCommand> timed_out_commands;
};

class CommandLifecycle {
 public:
  /// 创建命令生命周期管理器，配置ACK等待时间、最大重试次数和历史容量。
  CommandLifecycle(
      std::uint64_t ack_timeout_ms = 200u,
      std::size_t max_retries = 3u,
      std::size_t history_size = 64u);

  /// 登记已发送命令，保存原始帧并开始计算下一次ACK重试时间。
  void track(
      std::uint16_t sequence,
      std::uint8_t command_type,
      const std::vector<std::uint8_t>& frame,
      std::uint64_t now_ms,
      CommandDeliveryPolicy policy = CommandDeliveryPolicy::kRetryUntilAck);
  /// 根据sequence和命令类型匹配A1，返回接受、拒绝或无法匹配。
  AckDisposition handle_command_ack(const AckFrame& ack);
  /// 收集当前时刻应重发的原始帧以及已超过重试上限的命令。
  RetryBatch collect_due_retries(std::uint64_t now_ms);
  /// 清空所有仍在等待ACK的命令，并返回被中断记录供上层逐条结束业务状态。
  std::vector<InterruptedCommand> clear_pending();
  /// 取消一条尚未收到ACK的命令；用于旧命令在发送前被更新命令替换时停止重试。
  bool cancel_pending(std::uint16_t sequence);
  /// 登记下位机完成事件，并判断它是首次、重复还是未知命令事件。
  CompletionDisposition observe_completion(
      std::uint16_t sequence, std::uint8_t event_type);

 private:
  struct PendingCommand {
    std::uint8_t command_type{0};
    std::vector<std::uint8_t> frame;
    std::uint64_t next_retry_at_ms{0};
    std::size_t retries_sent{0};
    CommandDeliveryPolicy policy{CommandDeliveryPolicy::kRetryUntilAck};
  };

  /// 将命令sequence和完成事件类型组合成用于幂等去重的唯一键。
  static std::uint32_t completionKey(std::uint16_t sequence, std::uint8_t event_type);
  /// 将已确认命令写入有限长度历史，并清理超出容量的旧记录。
  void remember(std::uint16_t sequence, std::uint8_t command_type);

  std::uint64_t ack_timeout_ms_;
  std::size_t max_retries_;
  std::size_t history_size_;
  std::unordered_map<std::uint16_t, PendingCommand> pending_;
  std::unordered_map<std::uint16_t, std::uint8_t> history_;
  std::deque<std::uint16_t> history_order_;
  std::unordered_set<std::uint32_t> completions_;
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__COMMAND_LIFECYCLE_HPP_
