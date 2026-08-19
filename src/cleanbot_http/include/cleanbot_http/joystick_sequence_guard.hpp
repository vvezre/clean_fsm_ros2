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
  // 创建会话序号防重放器，并设置缓存容量和会话有效期。
  explicit JoystickSequenceGuard(
      std::size_t maximum_sessions = 1024u,
      std::uint64_t session_ttl_seconds = 600u);

  // 使用当前系统时间校验并登记某会话的摇杆序号。
  bool accept(const std::string& session_id, std::uint64_t sequence);
  // 使用调用方提供的时间校验并登记序号，便于测试和统一时钟控制。
  bool acceptAt(
      const std::string& session_id,
      std::uint64_t sequence,
      std::uint64_t now_seconds);

 private:
  // 保存某个前端会话最近接受的序号和最后活跃时间。
  struct Entry {
    std::string session_id;
    std::uint64_t maximum_sequence{0u};
    std::uint64_t last_seen_seconds{0u};
  };

  // 清理超过会话存活时间的序号记录。
  void removeExpiredLocked(std::uint64_t now_seconds);
  // 在容量达到上限时删除最久未使用的记录。
  void removeOldestLocked();

  std::size_t maximum_sessions_;
  std::uint64_t session_ttl_seconds_;
  std::vector<Entry> entries_;
};

}  // namespace http
}  // namespace cleanbot
