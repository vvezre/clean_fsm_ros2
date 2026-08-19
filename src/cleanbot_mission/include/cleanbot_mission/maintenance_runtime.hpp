#ifndef CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "cleanbot_common/publisher_epoch_tracker.hpp"
#include "cleanbot_mission/maintenance_gate.hpp"
#include "cleanbot_mission/maintenance_store.hpp"

// 文件作用：声明维护模式的持久化恢复、准入控制、硬件确认和故障锁存运行时。
namespace cleanbot {
namespace mission {

// 维护运行时向上层发布的完整状态快照。
struct MaintenanceRuntimeSnapshot {
  bool initialized{false};
  bool store_fault{false};
  bool admission_closed{true};
  std::uint64_t generation{0u};
  std::uint64_t last_generation{0u};
  bool gate_active{false};
  bool mission_idle{false};
  bool command_gate_applied{false};
  bool brake_acknowledged{false};
  bool hardware_fresh{false};
  bool linear_speed_zero{false};
  bool angular_speed_zero{false};
  bool brush_off{false};
  bool ready{false};
  std::string requester;
  std::string reason;
  std::string phase;
  std::string blocker_code;
  std::string message;
};

// 进入或退出维护模式请求的处理结果。
struct MaintenanceTransitionResult {
  bool accepted{false};
  bool gate_active{false};
  bool ready{false};
  std::uint64_t generation{0u};
  std::string code;
  std::string message;
};

// 从一个下位机状态消息提取的速度与连接状态样本。
struct MaintenanceHardwareSample {
  std::uint64_t frame_sequence{0u};
  bool connected{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

// 下位机消息发布者会话观测的接受结果。
enum class MaintenanceHardwareStatus {
  kAccepted,
  kRetired,
  kRevoked,
};

// 下位机发布者观测结果及分配的会话代次。
struct MaintenanceHardwareObservation {
  MaintenanceHardwareStatus status{MaintenanceHardwareStatus::kRevoked};
  std::uint64_t publisher_epoch{0u};
  bool session_changed{false};
};

class MaintenanceRuntime {
 public:
  // 使用状态文件和发布者会话容量创建维护运行时。
  explicit MaintenanceRuntime(
      std::filesystem::path state_path,
      std::size_t maximum_retired_publishers = 1024u,
      std::uint64_t maximum_publisher_epoch =
          cleanbot::common::PublisherEpochTracker::
              default_maximum_epoch());

  // 禁止复制构造，避免维护门、存储句柄和发布者会话状态出现两个所有者。
  MaintenanceRuntime(const MaintenanceRuntime&) = delete;
  // 禁止复制赋值，防止覆盖正在使用的维护运行时资源。
  MaintenanceRuntime& operator=(const MaintenanceRuntime&) = delete;
  // 禁止移动构造，保证内部存储与维护门在固定对象中持续有效。
  MaintenanceRuntime(MaintenanceRuntime&&) = delete;
  // 禁止移动赋值，避免运行中状态被隐式转移或丢失。
  MaintenanceRuntime& operator=(MaintenanceRuntime&&) = delete;

  // 初始化只执行一次；重复调用返回锁存结果，不会重新读取文件或改变已恢复的维护门。
  bool initialize(bool mission_idle) noexcept;
  // 更新任务空闲状态，以重新计算维护门准入条件。
  void setMissionIdle(bool mission_idle) noexcept;
  // 请求进入维护模式，并持久化申请者和原因。
  MaintenanceTransitionResult enable(
      const std::string& requester,
      const std::string& reason,
      bool mission_idle) noexcept;
  // 使用代次和申请者身份请求退出维护模式。
  MaintenanceTransitionResult disable(
      std::uint64_t generation,
      const std::string& requester) noexcept;
  // 转交最终控制命令证据给维护门。
  void observeFinalCommand(
      const FinalCommandEvidence& command) noexcept;
  // 转交下位机命令状态证据给维护门。
  void observeCommandStatus(
      const CommandStatusEvidence& status) noexcept;
  // 识别下位机发布者会话并记录最新硬件样本。
  MaintenanceHardwareObservation observeHardware(
      const cleanbot::common::PublisherIdentity& publisher,
      const MaintenanceHardwareSample& sample,
      std::uint64_t observed_at_nanoseconds) noexcept;
  // 根据新鲜度超时刷新下位机证据。
  void refreshHardware(
      std::uint64_t now_nanoseconds,
      std::uint64_t freshness_timeout_nanoseconds) noexcept;

  // 返回当前维护运行时快照。
  const MaintenanceRuntimeSnapshot& snapshot() const noexcept;
  // 返回初始化是否已完成。
  bool initialized() const noexcept;
  // 返回持久化存储是否发生不可恢复故障。
  bool storeFault() const noexcept;
  // 返回是否已关闭新的维护准入。
  bool admissionClosed() const noexcept;
  // 返回最后分配的维护代次。
  std::uint64_t lastGeneration() const noexcept;

 private:
  // 创建未初始化时的默认快照。
  static MaintenanceRuntimeSnapshot initialSnapshot();
  // 创建存储紧急故障时的保守快照。
  static MaintenanceRuntimeSnapshot emergencyFaultSnapshot();
  // 校验从存储中读取的记录是否在可用范围内。
  static bool validRecord(const MaintenanceStoreRecord& record) noexcept;
  // 将存储结果枚举转换为稳定的业务错误码名称。
  static const char* storeCodeName(MaintenanceStoreCode code) noexcept;
  // 判断存储错误是否需要锁存为运行时故障。
  static bool isOperationalFailure(
      MaintenanceStoreCode code) noexcept;
  // 以不抛异常的方式交换两个状态快照。
  static void swapSnapshots(
      MaintenanceRuntimeSnapshot& lhs,
      MaintenanceRuntimeSnapshot& rhs) noexcept;

  // 用持久化记录恢复维护门和对外快照。
  bool restore(
      const MaintenanceStoreRecord& record,
      bool mission_idle) noexcept;
  // 依据健康存储记录刷新快照。
  bool refreshHealthySnapshot(
      const MaintenanceStoreRecord& record) noexcept;
  // 依据新申请者和原因刷新快照。
  bool refreshHealthySnapshot(
      const std::string& requester,
      const std::string& reason) noexcept;
  // 统一构造维护模式切换结果。
  MaintenanceTransitionResult transitionResult(
      bool accepted,
      MaintenanceStoreCode code,
      const std::string& message,
      std::uint64_t generation) const noexcept;
  // 构造运行时不可用时的拒绝结果。
  MaintenanceTransitionResult rejectUnavailable(
      const char* code,
      const char* message) const noexcept;
  // 根据恢复记录确定是否应关闭新请求准入。
  void restoreAdmissionFromRecord() noexcept;
  // 在激活操作结果不确定时采取保守锁定策略。
  void applyUncertainActivation(
      const MaintenanceStoreResult& result,
      bool mission_idle) noexcept;
  // 锁存存储故障及其说明。
  void latchStoreFault(const std::string& message) noexcept;
  // 锁存最保守的紧急故障快照。
  void latchEmergencyFault() noexcept;

  MaintenanceStore store_;
  MaintenanceGate gate_;
  bool initialized_{false};
  bool store_fault_{false};
  bool admission_closed_{true};
  std::optional<MaintenanceStoreRecord> record_;
  cleanbot::common::PublisherEpochTracker hardware_publishers_;
  bool has_latest_hardware_{false};
  HardwareEvidence latest_hardware_;
  std::uint64_t latest_hardware_observed_at_nanoseconds_{0u};
  MaintenanceRuntimeSnapshot snapshot_;
  MaintenanceRuntimeSnapshot emergency_fault_snapshot_;
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
