/*
 * 文件作用：摇杆序列保护实现：防止旧序号或重复请求覆盖最新控制意图。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_http/joystick_sequence_guard.hpp"

#include <ctime>

namespace cleanbot {
namespace http {

// 保存会话容量和过期时间，并保证缓存容量至少为一。
JoystickSequenceGuard::JoystickSequenceGuard(
    const std::size_t maximum_sessions,
    const std::uint64_t session_ttl_seconds)
    : maximum_sessions_(maximum_sessions == 0u ? 1u : maximum_sessions),
      session_ttl_seconds_(session_ttl_seconds) {}

// 使用当前 Unix 时间校验并登记会话摇杆序号。
bool JoystickSequenceGuard::accept(
    const std::string& session_id,
    const std::uint64_t sequence) {
  const auto now = std::time(nullptr);
  return acceptAt(
      session_id,
      sequence,
      now < 0 ? 0u : static_cast<std::uint64_t>(now));
}

// 清理过期会话后，仅接受指定会话中严格递增的序号。
bool JoystickSequenceGuard::acceptAt(
    const std::string& session_id,
    const std::uint64_t sequence,
    const std::uint64_t now_seconds) {
  removeExpiredLocked(now_seconds);

  for (auto& entry : entries_) {
    if (entry.session_id != session_id) {
      continue;
    }
    entry.last_seen_seconds = now_seconds;
    if (sequence <= entry.maximum_sequence) {
      return false;
    }
    entry.maximum_sequence = sequence;
    return true;
  }

  if (entries_.size() >= maximum_sessions_) {
    removeOldestLocked();
  }
  entries_.push_back(Entry{session_id, sequence, now_seconds});
  return true;
}

// 删除最后活动时间超过会话有效期的记录。
void JoystickSequenceGuard::removeExpiredLocked(
    const std::uint64_t now_seconds) {
  for (auto iterator = entries_.begin(); iterator != entries_.end();) {
    const bool expired =
        now_seconds >= iterator->last_seen_seconds &&
        now_seconds - iterator->last_seen_seconds >= session_ttl_seconds_;
    if (expired) {
      iterator = entries_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

// 容量达到上限时移除最久未活动的会话。
void JoystickSequenceGuard::removeOldestLocked() {
  if (entries_.empty()) {
    return;
  }
  auto oldest = entries_.begin();
  for (auto iterator = entries_.begin(); iterator != entries_.end(); ++iterator) {
    if (iterator->last_seen_seconds < oldest->last_seen_seconds) {
      oldest = iterator;
    }
  }
  entries_.erase(oldest);
}

}  // namespace http
}  // namespace cleanbot
