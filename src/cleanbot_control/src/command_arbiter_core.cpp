/*
 * 文件作用：命令仲裁核心实现：按优先级和租约选择当前唯一控制命令。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_control/command_arbiter_core.hpp"

#include <algorithm>
#include <utility>

namespace cleanbot {
namespace control {

// 仅接受更高代次的维护状态，防止过期消息回滚安全门。
bool MaintenanceGateCache::update(
    const bool active, const std::uint64_t generation) {
  if (generation == 0u) {
    return false;
  }

  if (active) {
    if (has_state_ && active_ && generation == generation_) {
      return true;
    }
    if (has_state_ && generation <= generation_) {
      return false;
    }
    has_state_ = true;
    active_ = true;
    generation_ = generation;
    return true;
  }

  if (!has_state_ || !active_ || generation != generation_) {
    return false;
  }
  active_ = false;
  return true;
}

// 返回缓存是否已收到过有效维护状态。
bool MaintenanceGateCache::has_state() const {
  return has_state_;
}

// 返回缓存的维护模式开关。
bool MaintenanceGateCache::active() const {
  return active_;
}

// 返回缓存状态对应的维护代次。
std::uint64_t MaintenanceGateCache::generation() const {
  return generation_;
}

// 初始化发布者会话跟踪器，限制已退役身份缓存大小。
MaintenancePublisherCoordinator::MaintenancePublisherCoordinator(
    const std::size_t maximum_retired_identities,
    const std::uint64_t maximum_epoch)
    : publisher_tracker_(maximum_retired_identities, maximum_epoch) {}

// 按发布者会话和维护代次过滤消息，并决定是否需强制重发布。
MaintenancePublisherObservation MaintenancePublisherCoordinator::observe(
    const bool gate_active,
    const std::uint64_t generation,
    const common::PublisherIdentity& publisher_identity) {
  auto next_gate = gate_;
  auto next_tracker = publisher_tracker_;
  const auto publisher_result = next_tracker.observe(publisher_identity);

  if (publisher_result.status == common::PublisherEpochStatus::kInvalid) {
    if (has_tracked_publisher_ || !gate_active ||
        !next_gate.update(true, generation)) {
      return result(false);
    }
    gate_ = std::move(next_gate);
    return result(true);
  }

  if (!has_tracked_publisher_ && !gate_active) {
    return result(false);
  }

  const bool repeated_current_release =
      publisher_result.status == common::PublisherEpochStatus::kAccepted &&
      !publisher_result.session_changed &&
      gate_.has_state() && !gate_.active() && !gate_active &&
      generation == gate_.generation();
  if (repeated_current_release) {
    return result(true);
  }

  if (publisher_result.status != common::PublisherEpochStatus::kAccepted ||
      !next_gate.update(gate_active, generation)) {
    return result(false);
  }

  gate_ = std::move(next_gate);
  publisher_tracker_ = std::move(next_tracker);
  has_tracked_publisher_ = true;
  const bool session_changed = publisher_result.session_changed;
  return result(
      true,
      session_changed,
      session_changed && gate_.active());
}

// 返回是否已经接受过维护状态。
bool MaintenancePublisherCoordinator::has_state() const {
  return gate_.has_state();
}

// 返回当前已接受的维护门开关。
bool MaintenancePublisherCoordinator::active() const {
  return gate_.active();
}

// 返回当前已接受的维护状态代次。
std::uint64_t MaintenancePublisherCoordinator::generation() const {
  return gate_.generation();
}

// 使用内部维护门缓存构造统一的发布者观测结果。
MaintenancePublisherObservation MaintenancePublisherCoordinator::result(
    const bool accepted,
    const bool session_changed,
    const bool force_republish) const {
  return MaintenancePublisherObservation{
      accepted,
      gate_.active(),
      gate_.generation(),
      session_changed,
      force_republish};
}

// 保存各来源租约参数，创建初始空仲裁状态。
CommandArbiterCore::CommandArbiterCore(const ArbiterParameters& parameters)
    : parameters_(parameters) {}

// 接收一条控制命令，处理急停边界并更新对应优先级槽位。
bool CommandArbiterCore::update(
    const CommandSource source,
    const ControlCommand& command,
    const std::uint64_t now_ms) {
  if (source == CommandSource::kEmergency && command.active && command.brake) {
    stop_boundary_command_id_ = std::max(stop_boundary_command_id_, command.command_id);
    stop_boundary_stamp_ms_ = std::max(stop_boundary_stamp_ms_, command.stamp_ms);
    software_stopped_ = true;
    brush_enabled_ = false;
    brush_speed_ = 0;
    has_operator_mode_ = false;
    clearAllSlots();
    return true;
  }

  if (maintenance_gate_.active()) {
    return false;
  }

  CommandSlot& destination = slot(source);
  if (!command.active) {
    destination.present = false;
    return true;
  }

  const bool operator_source = source == CommandSource::kManual ||
      source == CommandSource::kMission || source == CommandSource::kVision;
  if (software_stopped_) {
    if (!operator_source || !command.operator_intent || !isNewerThanStop(command)) {
      return false;
    }
    software_stopped_ = false;
    clearOperatorSlots();
    operator_mode_ = source;
    has_operator_mode_ = true;
  } else if (operator_source) {
    if (command.operator_intent) {
      clearOperatorSlots();
      operator_mode_ = source;
      has_operator_mode_ = true;
    } else if (has_operator_mode_ && source != operator_mode_) {
      return false;
    }
  }

  destination.command = command;
  destination.received_at_ms = now_ms;
  destination.present = true;
  return true;
}

// 更新刷盘命令；维护模式中拒绝可能造成作业的刷盘输入。
void CommandArbiterCore::set_brush(
    const bool enabled, const std::int32_t speed, const bool operator_intent) {
  if (maintenance_gate_.active()) {
    return;
  }
  if (!operator_intent && software_stopped_) {
    return;
  }
  brush_enabled_ = enabled;
  brush_speed_ = enabled ? std::max(-100, std::min(100, speed)) : 0;
}

// 更新维护安全门状态，并在进入维护时清除活动控制输入。
bool CommandArbiterCore::set_maintenance(
    const bool active, const std::uint64_t generation) {
  const bool state_changed =
      active != maintenance_gate_.active() ||
      generation != maintenance_gate_.generation();
  if (!maintenance_gate_.update(active, generation)) {
    return false;
  }
  if (state_changed) {
    clearMaintenanceInputs();
  }
  return true;
}

// 返回维护安全门当前是否有效。
bool CommandArbiterCore::maintenance_active() const {
  return maintenance_gate_.active();
}

// 返回维护安全门的当前代次。
std::uint64_t CommandArbiterCore::maintenance_generation() const {
  return maintenance_gate_.generation();
}

// 按急停、维护、优先级和租约计算本周期唯一安全输出。
ControlCommand CommandArbiterCore::output(const std::uint64_t now_ms) {
  if (maintenance_gate_.active()) {
    return maintenanceOutput(maintenance_gate_.generation());
  }

  if (software_stopped_) {
    return brakingOutput("software_emergency_stop");
  }

  CommandSlot* selected = nullptr;
  if (slotActive(emergency_, 0u, now_ms)) {
    selected = &emergency_;
  } else if (slotActive(safety_, 0u, now_ms)) {
    selected = &safety_;
  } else if (slotActive(manual_, parameters_.manual_lease_ms, now_ms)) {
    selected = &manual_;
  } else if (slotActive(mission_, parameters_.mission_lease_ms, now_ms)) {
    selected = &mission_;
  } else if (slotActive(vision_, parameters_.vision_lease_ms, now_ms)) {
    selected = &vision_;
  }

  if (selected == nullptr) {
    if (brush_enabled_) {
      ControlCommand result;
      result.source = "brush_only";
      result.active = true;
      result.brush_speed = brush_speed_;
      result.brake = false;
      return result;
    }
    return brakingOutput("idle_brake");
  }

  ControlCommand result = selected->command;
  if (result.brake) {
    result.x_speed = 0;
    result.z_speed = 0;
    result.steering_offset = 0;
    result.target_distance = 0;
    result.target_rotation = 0;
    result.brush_speed = 0;
  } else {
    result.brush_speed = brush_enabled_ ? brush_speed_ : 0;
  }
  return result;
}

// 返回软件急停锁存是否仍未解除。
bool CommandArbiterCore::software_stopped() const { return software_stopped_; }

// 将命令来源映射到对应的内部槽位。
CommandSlot& CommandArbiterCore::slot(const CommandSource source) {
  if (source == CommandSource::kEmergency) {
    return emergency_;
  }
  if (source == CommandSource::kSafety) {
    return safety_;
  }
  if (source == CommandSource::kManual) {
    return manual_;
  }
  if (source == CommandSource::kMission) {
    return mission_;
  }
  return vision_;
}

// 判断槽位已有指令是否尚处于来源租约有效期。
bool CommandArbiterCore::slotActive(
    const CommandSlot& slot_value,
    const std::uint64_t lease_ms,
    const std::uint64_t now_ms) const {
  if (!slot_value.present || !slot_value.command.active) {
    return false;
  }
  if (lease_ms == 0u || now_ms < slot_value.received_at_ms) {
    return true;
  }
  return now_ms - slot_value.received_at_ms <= lease_ms;
}

// 清除手动、任务和视觉等操作性输入。
void CommandArbiterCore::clearOperatorSlots() {
  manual_.present = false;
  mission_.present = false;
  vision_.present = false;
}

// 清除所有来源槽位，保留安全状态本身。
void CommandArbiterCore::clearAllSlots() {
  emergency_.present = false;
  safety_.present = false;
  clearOperatorSlots();
}

// 进入维护模式后清除会造成运动或作业的输入。
void CommandArbiterCore::clearMaintenanceInputs() {
  clearAllSlots();
  has_operator_mode_ = false;
  brush_enabled_ = false;
  brush_speed_ = 0;
}

// 判断命令是否晚于最近软件急停边界，避免旧命令恢复运动。
bool CommandArbiterCore::isNewerThanStop(const ControlCommand& command) const {
  if (stop_boundary_stamp_ms_ != 0u && command.stamp_ms != 0u) {
    return command.stamp_ms > stop_boundary_stamp_ms_;
  }
  return command.command_id != 0u && command.command_id > stop_boundary_command_id_;
}

// 构造包含制动标志的零速安全输出。
ControlCommand CommandArbiterCore::brakingOutput(const std::string& source) {
  ControlCommand result;
  result.source = source;
  result.priority = source == "software_emergency_stop" ? 100u : 0u;
  result.active = true;
  result.brake = true;
  return result;
}

// 构造维护模式使用的停机、刹车和关闭刷盘输出。
ControlCommand CommandArbiterCore::maintenanceOutput(
    const std::uint64_t generation) {
  ControlCommand result;
  result.request_id = generation;
  result.source = "maintenance_gate";
  result.priority = 80u;
  result.active = true;
  result.brake = true;
  return result;
}

}  // namespace control
}  // namespace cleanbot
