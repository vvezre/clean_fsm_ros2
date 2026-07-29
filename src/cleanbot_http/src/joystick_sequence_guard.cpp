#include "cleanbot_http/joystick_sequence_guard.hpp"

#include <ctime>

namespace cleanbot {
namespace http {

JoystickSequenceGuard::JoystickSequenceGuard(
    const std::size_t maximum_sessions,
    const std::uint64_t session_ttl_seconds)
    : maximum_sessions_(maximum_sessions == 0u ? 1u : maximum_sessions),
      session_ttl_seconds_(session_ttl_seconds) {}

bool JoystickSequenceGuard::accept(
    const std::string& session_id,
    const std::uint64_t sequence) {
  const auto now = std::time(nullptr);
  return acceptAt(
      session_id,
      sequence,
      now < 0 ? 0u : static_cast<std::uint64_t>(now));
}

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
