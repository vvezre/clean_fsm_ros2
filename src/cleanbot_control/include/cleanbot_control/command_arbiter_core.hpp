#ifndef CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_
#define CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "cleanbot_common/publisher_epoch_tracker.hpp"

// 文件作用：声明控制指令仲裁、维护状态缓存与发布者会话协调的核心数据结构。
namespace cleanbot {
namespace control {

// 指令来源的优先级枚举，数值越小优先级越高。
enum class CommandSource {
  kEmergency = 0,
  kSafety = 1,
  kManual = 2,
  kMission = 3,
  kVision = 4,
};

// 仲裁器中各类控制指令的有效租约时长。
struct ArbiterParameters {
  std::uint64_t manual_lease_ms{800u};
  std::uint64_t mission_lease_ms{1000u};
  std::uint64_t vision_lease_ms{500u};
};

// 下发到下位机前的统一控制指令载体。
struct ControlCommand {
  std::uint64_t stamp_ms{0u};
  std::uint64_t command_id{0u};
  std::uint64_t request_id{0u};
  std::string source;
  std::uint8_t priority{0u};
  bool active{false};
  bool operator_intent{false};
  std::uint8_t status{0u};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t steering_offset{0};
  std::int32_t brush_speed{0};
  std::int32_t target_distance{0};
  std::int32_t target_rotation{0};
  double heading_deg{0.0};
  bool brake{false};
  bool charge{false};
};

// 保存单个指令来源最近一次有效输入及其接收时间。
struct CommandSlot {
  ControlCommand command;
  std::uint64_t received_at_ms{0u};
  bool present{false};
};

class MaintenanceGateCache {
 public:
  // 更新维护门状态；generation 递增时接受新的状态。
  bool update(bool active, std::uint64_t generation);
  // 返回是否已经收到过维护状态。
  bool has_state() const;
  // 返回当前缓存的维护门开关。
  bool active() const;
  // 返回当前缓存状态的代次。
  std::uint64_t generation() const;

 private:
  bool has_state_{false};
  bool active_{false};
  std::uint64_t generation_{0u};
};

// 一次维护状态发布者观测的判定结果。
struct MaintenancePublisherObservation {
  bool accepted{false};
  bool gate_active{false};
  std::uint64_t generation{0u};
  bool session_changed{false};
  bool force_republish{false};
};

class MaintenancePublisherCoordinator {
 public:
  // 创建发布者协调器，并限制可记忆的历史发布者数量。
  explicit MaintenancePublisherCoordinator(
      std::size_t maximum_retired_identities = 1024u,
      std::uint64_t maximum_epoch =
          common::PublisherEpochTracker::default_maximum_epoch());

  // 根据发布者身份与维护状态判断消息是否应被接受和转发。
  MaintenancePublisherObservation observe(
      bool gate_active,
      std::uint64_t generation,
      const common::PublisherIdentity& publisher_identity);
  // 返回是否已保存维护门状态。
  bool has_state() const;
  // 返回已保存的维护门状态。
  bool active() const;
  // 返回已保存的维护状态代次。
  std::uint64_t generation() const;

 private:
  // 将内部缓存转换成统一的观测结果。
  MaintenancePublisherObservation result(
      bool accepted,
      bool session_changed = false,
      bool force_republish = false) const;

  MaintenanceGateCache gate_;
  common::PublisherEpochTracker publisher_tracker_;
  bool has_tracked_publisher_{false};
};

class CommandArbiterCore {
 public:
  // 使用给定的指令租约参数创建仲裁器。
  explicit CommandArbiterCore(const ArbiterParameters& parameters = ArbiterParameters());

  // 接收某来源指令，并按时间戳与停机边界判断是否有效。
  bool update(CommandSource source, const ControlCommand& command, std::uint64_t now_ms);
  // 设置刷盘开关、速度和是否由人工意图触发。
  void set_brush(bool enabled, std::int32_t speed, bool operator_intent);
  // 写入维护模式状态；维护时清理不允许的控制输入。
  bool set_maintenance(bool active, std::uint64_t generation);
  // 返回维护模式是否正在生效。
  bool maintenance_active() const;
  // 返回当前维护状态代次。
  std::uint64_t maintenance_generation() const;
  // 按优先级和租约计算本周期应输出的控制指令。
  ControlCommand output(std::uint64_t now_ms);
  // 返回软件急停是否处于锁定状态。
  bool software_stopped() const;

 private:
  // 按来源取得对应的指令槽位。
  CommandSlot& slot(CommandSource source);
  // 判断槽位中的指令是否仍在租约内。
  bool slotActive(
      const CommandSlot& slot, std::uint64_t lease_ms, std::uint64_t now_ms) const;
  // 清除人工操作来源的输入。
  void clearOperatorSlots();
  // 清除所有待仲裁的控制输入。
  void clearAllSlots();
  // 清除维护状态下不应继续执行的输入。
  void clearMaintenanceInputs();
  // 判断指令是否晚于最近一次软件急停边界。
  bool isNewerThanStop(const ControlCommand& command) const;
  // 构造安全制动输出。
  static ControlCommand brakingOutput(const std::string& source);
  // 构造维护模式专用的安全输出。
  static ControlCommand maintenanceOutput(std::uint64_t generation);

  ArbiterParameters parameters_;
  CommandSlot emergency_;
  CommandSlot safety_;
  CommandSlot manual_;
  CommandSlot mission_;
  CommandSlot vision_;
  bool software_stopped_{false};
  std::uint64_t stop_boundary_command_id_{0u};
  std::uint64_t stop_boundary_stamp_ms_{0u};
  CommandSource operator_mode_{CommandSource::kMission};
  bool has_operator_mode_{false};
  bool brush_enabled_{false};
  std::int32_t brush_speed_{0};
  MaintenanceGateCache maintenance_gate_;
};

}  // namespace control
}  // namespace cleanbot

#endif  // CLEANBOT_CONTROL__COMMAND_ARBITER_CORE_HPP_
