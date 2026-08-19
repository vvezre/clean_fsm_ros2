#ifndef CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

// 文件作用：声明维护模式进入前的刹车确认、下位机静止确认和状态快照逻辑。
namespace cleanbot {
namespace mission {

// 由控制仲裁器输出的最终控制命令证据。
struct FinalCommandEvidence {
  std::uint64_t generation{0u};
  std::uint64_t request_id{0u};
  std::uint64_t command_id{0u};
  std::string source;
  bool active{false};
  bool brake{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

// 下位机对控制命令执行状态的回执证据。
struct CommandStatusEvidence {
  std::uint64_t generation{0u};
  std::uint64_t request_id{0u};
  std::uint64_t command_id{0u};
  std::string source;
  std::uint8_t state{0u};
};

struct HardwareEvidence {
  // 发布者会话编号；同一会话内恒定，替换发布者时严格递增，已退役编号会被忽略。
  std::uint64_t publisher_epoch{0u};
  std::uint64_t frame_sequence{0u};
  bool fresh{false};
  bool connected{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

// 维护门当前各项安全确认结果和阻塞原因。
struct MaintenanceGateSnapshot {
  std::uint64_t generation{0u};
  bool gate_active{false};
  bool mission_idle{false};
  bool command_gate_applied{false};
  bool brake_acknowledged{false};
  bool hardware_fresh{false};
  bool linear_speed_zero{false};
  bool angular_speed_zero{false};
  bool brush_off{false};
  bool ready{false};
  std::string phase;
  std::string blocker_code;
  std::string message;
};

class MaintenanceGate {
 public:
  // 仅恢复上次代次和任务空闲状态，用于旧版持久化数据。
  bool restorePersistentState(
      std::uint64_t last_generation,
      bool mission_idle);
  // 恢复完整持久化状态，包括正在生效的维护代次。
  bool restorePersistentState(
      std::uint64_t last_generation,
      std::uint64_t active_generation,
      bool mission_idle);
  // 请求进入指定代次的维护模式。
  bool request(std::uint64_t generation, bool mission_idle);
  // 释放指定代次的维护模式。
  bool release(std::uint64_t generation);

  // 更新任务是否空闲的前置条件。
  void setMissionIdle(bool mission_idle);
  // 接收仲裁后的最终控制命令作为刹车证据。
  void observeFinalCommand(const FinalCommandEvidence& command);
  // 接收下位机命令状态回执作为刹车确认依据。
  void observeCommandStatus(const CommandStatusEvidence& status);
  // 接收下位机速度和刷盘状态作为静止确认依据。
  void observeHardware(const HardwareEvidence& hardware);

  // 返回当前维护门安全条件和阻塞原因的快照。
  MaintenanceGateSnapshot snapshot() const;
  // 返回最后处理过的维护代次。
  std::uint64_t lastGeneration() const;
  // 返回待匹配命令回执缓存的最大容量。
  std::size_t pendingStatusCapacity() const;

 private:
  // 将刹车未确认的原因归类为可诊断状态。
  enum class BrakeFailure {
    kNone,
    kRejected,
    kTimedOut,
    kTransportLost,
    kSuperseded,
    kCorrelationOverflow,
  };

  // 缓存尚未与当前命令匹配的回执状态。
  struct PendingCommandStatus {
    bool acknowledged{false};
    BrakeFailure failure{BrakeFailure::kNone};
  };

  static constexpr std::size_t kPendingStatusCapacity = 16u;

  // 执行持久化状态恢复的共用实现。
  bool restorePersistentStateImpl(
      std::uint64_t last_generation,
      const std::uint64_t* active_generation,
      bool mission_idle);
  // 判断证据中的代次、请求号和来源是否匹配当前维护请求。
  bool matches(
      std::uint64_t generation,
      std::uint64_t request_id,
      const std::string& source) const;
  // 将下位机命令状态转换为刹车失败原因。
  static BrakeFailure failureForState(std::uint8_t state);
  // 保存先到达的命令状态回执。
  void rememberPendingStatus(
      std::uint64_t command_id,
      bool acknowledged,
      BrakeFailure failure);
  // 将已缓存回执应用到当前等待的命令。
  void applyPendingStatus(std::uint64_t command_id);
  // 标记当前命令已获得刹车确认。
  void acknowledgeCurrentCommand();
  // 锁存当前维护申请的刹车失败原因。
  void latchFailure(BrakeFailure failure);
  // 重置所有与一次维护申请相关的证据。
  void resetEvidence();
  // 重置下位机静止确认计数。
  void resetHardwareConfirmation();
  // 清空下位机状态样本。
  void resetHardwareEvidence();

  bool pristine_{true};
  bool active_{false};
  std::uint64_t generation_{0u};
  std::uint64_t last_generation_{0u};
  bool mission_idle_{false};
  bool command_gate_applied_{false};
  std::uint64_t current_command_id_{0u};
  bool brake_acknowledged_{false};
  BrakeFailure brake_failure_{BrakeFailure::kNone};
  std::map<std::uint64_t, PendingCommandStatus> pending_command_statuses_;

  bool hardware_sample_observed_{false};
  bool hardware_fresh_{false};
  bool linear_speed_zero_{false};
  bool angular_speed_zero_{false};
  bool brush_off_{false};
  bool has_publisher_epoch_{false};
  std::uint64_t publisher_epoch_{0u};
  bool has_last_frame_sequence_{false};
  std::uint64_t last_frame_sequence_{0u};
  std::uint8_t consecutive_zero_frames_{0u};
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_GATE_HPP_
