#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace http {

// 按前端控制会话记录已接收的最大摇杆序号。
// 调用方需要在同一临界区内完成 accept() 和后续命令发布决策。
class JoystickSequenceGuard {
 public:
  explicit JoystickSequenceGuard(
      std::size_t maximum_sessions = 1024u,
      std::uint64_t session_ttl_seconds = 600u);

  bool accept(const std::string& session_id, std::uint64_t sequence);
  bool acceptAt(
      const std::string& session_id,
      std::uint64_t sequence,
      std::uint64_t now_seconds);

 private:
  struct Entry {
    std::string session_id;
    std::uint64_t maximum_sequence{0u};
    std::uint64_t last_seen_seconds{0u};
  };

  void removeExpiredLocked(std::uint64_t now_seconds);
  void removeOldestLocked();

  std::size_t maximum_sessions_;
  std::uint64_t session_ttl_seconds_;
  std::vector<Entry> entries_;
};

}  // namespace http
}  // namespace cleanbot
