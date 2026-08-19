/*
 * 文件作用：维护门控实现：协调维护请求、刹车证据和硬件零状态确认。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_mission/maintenance_gate.hpp"

#include <algorithm>

namespace cleanbot {
namespace mission {

namespace {

constexpr std::uint8_t kStateSent = 1u;
constexpr std::uint8_t kStateAcknowledged = 2u;
constexpr std::uint8_t kStateRejected = 3u;
constexpr std::uint8_t kStateTimedOut = 4u;
constexpr std::uint8_t kStateTransportLost = 7u;
constexpr std::uint8_t kStateSuperseded = 8u;

}  // namespace

// 以无活动维护代次的旧格式恢复持久化状态。
bool MaintenanceGate::restorePersistentState(
    const std::uint64_t last_generation,
    const bool mission_idle) {
  return restorePersistentStateImpl(
      last_generation,
      nullptr,
      mission_idle);
}

// 以包含活动维护代次的新格式恢复持久化状态。
bool MaintenanceGate::restorePersistentState(
    const std::uint64_t last_generation,
    const std::uint64_t active_generation,
    const bool mission_idle) {
  return restorePersistentStateImpl(
      last_generation,
      &active_generation,
      mission_idle);
}

// 校验递增代次后申请进入维护安全门。
bool MaintenanceGate::request(
    const std::uint64_t generation,
    const bool mission_idle) {
  if (generation == 0u || generation <= last_generation_) {
    return false;
  }

  pristine_ = false;
  active_ = true;
  generation_ = generation;
  last_generation_ = generation;
  mission_idle_ = mission_idle;
  resetEvidence();
  return true;
}

// 仅允许匹配当前代次的调用方退出维护模式。
bool MaintenanceGate::release(const std::uint64_t generation) {
  if (!active_ || generation != generation_) {
    return false;
  }

  active_ = false;
  generation_ = 0u;
  mission_idle_ = false;
  resetEvidence();
  return true;
}

// 更新任务是否空闲，影响维护模式是否可以就绪。
void MaintenanceGate::setMissionIdle(const bool mission_idle) {
  if (mission_idle != mission_idle_) {
    pristine_ = false;
  }
  mission_idle_ = mission_idle;
}

// 验证最终输出已成为零速、刹车、停刷盘的维护命令。
void MaintenanceGate::observeFinalCommand(
    const FinalCommandEvidence& command) {
  if (!active_ || brake_failure_ != BrakeFailure::kNone ||
      !matches(command.generation, command.request_id, command.source)) {
    return;
  }

  const bool safe_brake = command.active && command.brake &&
      command.command_id != 0u &&
      command.x_speed == 0 && command.z_speed == 0 &&
      command.brush_speed == 0;
  if (!safe_brake) {
    command_gate_applied_ = false;
    current_command_id_ = 0u;
    brake_acknowledged_ = false;
    resetHardwareConfirmation();
    return;
  }

  const bool command_changed =
      !command_gate_applied_ || command.command_id != current_command_id_;
  command_gate_applied_ = true;
  if (command_changed) {
    current_command_id_ = command.command_id;
    brake_acknowledged_ = false;
    resetHardwareConfirmation();
  }
  applyPendingStatus(command.command_id);
}

// 根据下位机命令状态确认刹车，或锁存不可恢复的刹车失败。
void MaintenanceGate::observeCommandStatus(
    const CommandStatusEvidence& status) {
  if (!active_ || brake_failure_ != BrakeFailure::kNone ||
      !matches(status.generation, status.request_id, status.source) ||
      status.command_id == 0u) {
    return;
  }

  // 旧 Python 下位机协议没有在线 ACK；STATE_SENT 仅证明完整刹车帧已交给串口驱动。
  // 是否就绪仍必须等待后续新鲜硬件帧确认所有输出均为零。
  const bool acknowledged =
      status.state == kStateSent || status.state == kStateAcknowledged;
  const BrakeFailure failure = failureForState(status.state);
  if (!acknowledged && failure == BrakeFailure::kNone) {
    return;
  }

  if (command_gate_applied_ &&
      status.command_id == current_command_id_) {
    if (failure != BrakeFailure::kNone) {
      latchFailure(failure);
    } else {
      acknowledgeCurrentCommand();
    }
    return;
  }

  rememberPendingStatus(status.command_id, acknowledged, failure);
}

// 以同一发布者会话中连续两帧零输出硬件状态确认车辆已安全静止。
void MaintenanceGate::observeHardware(const HardwareEvidence& hardware) {
  if (!active_ || brake_failure_ != BrakeFailure::kNone) {
    return;
  }

  if (hardware.publisher_epoch == 0u) {
    resetHardwareConfirmation();
    hardware_sample_observed_ = brake_acknowledged_;
    return;
  }

  if (!has_publisher_epoch_ ||
      hardware.publisher_epoch > publisher_epoch_) {
    resetHardwareConfirmation();
    has_publisher_epoch_ = true;
    publisher_epoch_ = hardware.publisher_epoch;
    has_last_frame_sequence_ = false;
    last_frame_sequence_ = 0u;
  } else if (hardware.publisher_epoch < publisher_epoch_) {
    return;
  }

  if (!brake_acknowledged_) {
    if (!has_last_frame_sequence_ ||
        hardware.frame_sequence > last_frame_sequence_) {
      has_last_frame_sequence_ = true;
      last_frame_sequence_ = hardware.frame_sequence;
    }
    return;
  }

  const bool sample_fresh = hardware.fresh && hardware.connected;
  const bool linear_zero = hardware.x_speed == 0;
  const bool angular_zero = hardware.z_speed == 0;
  const bool brush_off = hardware.brush_speed == 0;
  const bool safe_zero =
      sample_fresh && linear_zero && angular_zero && brush_off;

  if (!safe_zero) {
    hardware_sample_observed_ = true;
    hardware_fresh_ = sample_fresh;
    linear_speed_zero_ = linear_zero;
    angular_speed_zero_ = angular_zero;
    brush_off_ = brush_off;
    consecutive_zero_frames_ = 0u;
    if (!has_last_frame_sequence_ ||
        hardware.frame_sequence > last_frame_sequence_) {
      has_last_frame_sequence_ = true;
      last_frame_sequence_ = hardware.frame_sequence;
    }
    return;
  }

  if (has_last_frame_sequence_ &&
      hardware.frame_sequence <= last_frame_sequence_) {
    return;
  }

  hardware_sample_observed_ = true;
  hardware_fresh_ = true;
  linear_speed_zero_ = true;
  angular_speed_zero_ = true;
  brush_off_ = true;
  has_last_frame_sequence_ = true;
  last_frame_sequence_ = hardware.frame_sequence;
  consecutive_zero_frames_ =
      std::min<std::uint8_t>(2u, consecutive_zero_frames_ + 1u);
}

// 汇总维护门安全条件，生成对外可诊断的阶段、阻塞码和就绪状态。
MaintenanceGateSnapshot MaintenanceGate::snapshot() const {
  MaintenanceGateSnapshot result;
  result.generation = generation_;
  result.gate_active = active_;
  result.mission_idle = mission_idle_;
  result.command_gate_applied = command_gate_applied_;
  result.brake_acknowledged = brake_acknowledged_;
  result.hardware_fresh = hardware_fresh_;
  result.linear_speed_zero = linear_speed_zero_;
  result.angular_speed_zero = angular_speed_zero_;
  result.brush_off = brush_off_;
  result.ready = active_ && mission_idle_ && command_gate_applied_ &&
      brake_acknowledged_ && brake_failure_ == BrakeFailure::kNone &&
      hardware_sample_observed_ && hardware_fresh_ &&
      linear_speed_zero_ && angular_speed_zero_ && brush_off_ &&
      consecutive_zero_frames_ >= 2u;

  if (!active_) {
    result.phase = "INACTIVE";
    result.blocker_code = "MAINTENANCE_INACTIVE";
    result.message = "maintenance gate is inactive";
  } else if (brake_failure_ == BrakeFailure::kRejected) {
    result.phase = "BRAKE_REJECTED";
    result.blocker_code = "BRAKE_REJECTED";
    result.message = "maintenance brake command was rejected";
  } else if (brake_failure_ == BrakeFailure::kTimedOut) {
    result.phase = "BRAKE_TIMED_OUT";
    result.blocker_code = "BRAKE_TIMED_OUT";
    result.message = "maintenance brake acknowledgement timed out";
  } else if (brake_failure_ == BrakeFailure::kTransportLost) {
    result.phase = "TRANSPORT_LOST";
    result.blocker_code = "TRANSPORT_LOST";
    result.message = "maintenance brake transport was lost";
  } else if (brake_failure_ == BrakeFailure::kSuperseded) {
    result.phase = "BRAKE_SUPERSEDED";
    result.blocker_code = "BRAKE_SUPERSEDED";
    result.message = "maintenance brake command was superseded";
  } else if (brake_failure_ == BrakeFailure::kCorrelationOverflow) {
    result.phase = "CORRELATION_OVERFLOW";
    result.blocker_code = "CORRELATION_OVERFLOW";
    result.message = "maintenance command-status correlation overflowed";
  } else if (!mission_idle_) {
    result.phase = "MISSION_BUSY";
    result.blocker_code = "MISSION_BUSY";
    result.message = "a mission or goal reservation is active";
  } else if (!command_gate_applied_) {
    result.phase = "WAITING_FOR_COMMAND_GATE";
    result.blocker_code = "COMMAND_GATE_NOT_APPLIED";
    result.message = "waiting for the maintenance brake at final command";
  } else if (!brake_acknowledged_) {
    result.phase = "WAITING_FOR_BRAKE_ACK";
    result.blocker_code = "BRAKE_ACK_PENDING";
    result.message = "waiting for lower-machine brake acknowledgement";
  } else if (!hardware_sample_observed_) {
    result.phase = "WAITING_FOR_HARDWARE";
    result.blocker_code = "HARDWARE_SAMPLE_PENDING";
    result.message = "waiting for post-acknowledgement hardware status";
  } else if (!hardware_fresh_) {
    result.phase = "WAITING_FOR_HARDWARE";
    result.blocker_code = "HARDWARE_NOT_FRESH";
    result.message = "latest hardware status is stale or disconnected";
  } else if (!linear_speed_zero_ || !angular_speed_zero_) {
    result.phase = "WAITING_FOR_HARDWARE";
    result.blocker_code = "HARDWARE_NOT_STOPPED";
    result.message = "latest hardware status reports vehicle motion";
  } else if (!brush_off_) {
    result.phase = "WAITING_FOR_HARDWARE";
    result.blocker_code = "BRUSH_NOT_OFF";
    result.message = "latest hardware status reports brush motion";
  } else if (consecutive_zero_frames_ < 2u) {
    result.phase = "WAITING_FOR_HARDWARE";
    result.blocker_code = "HARDWARE_CONFIRMATION_PENDING";
    result.message = "waiting for a second distinct zero-state hardware frame";
  } else {
    result.phase = "READY";
    result.blocker_code.clear();
    result.message = "maintenance readiness confirmed";
  }

  return result;
}

// 返回历史上最后被接受的维护申请代次。
std::uint64_t MaintenanceGate::lastGeneration() const {
  return last_generation_;
}

// 返回命令回执关联缓存的固定容量。
std::size_t MaintenanceGate::pendingStatusCapacity() const {
  return kPendingStatusCapacity;
}

// 执行一次性恢复，拒绝格式不合法或已被运行时状态污染的请求。
bool MaintenanceGate::restorePersistentStateImpl(
    const std::uint64_t last_generation,
    const std::uint64_t* const active_generation,
    const bool mission_idle) {
  const bool has_active_generation = active_generation != nullptr;
  const bool invalid_active_generation = has_active_generation &&
      (last_generation == 0u || *active_generation != last_generation);
  if (!pristine_ || invalid_active_generation) {
    return false;
  }

  pristine_ = false;
  active_ = has_active_generation;
  generation_ = active_ ? *active_generation : 0u;
  last_generation_ = last_generation;
  mission_idle_ = mission_idle;
  resetEvidence();
  return true;
}

// 检查证据是否属于当前维护代次和约定的维护命令来源。
bool MaintenanceGate::matches(
    const std::uint64_t generation,
    const std::uint64_t request_id,
    const std::string& source) const {
  return generation == generation_ && request_id == generation_ &&
      source == "maintenance_gate";
}

// 将下位机命令状态转换为维护门使用的失败类型。
MaintenanceGate::BrakeFailure MaintenanceGate::failureForState(
    const std::uint8_t state) {
  if (state == kStateRejected) {
    return BrakeFailure::kRejected;
  }
  if (state == kStateTimedOut) {
    return BrakeFailure::kTimedOut;
  }
  if (state == kStateTransportLost) {
    return BrakeFailure::kTransportLost;
  }
  if (state == kStateSuperseded) {
    return BrakeFailure::kSuperseded;
  }
  return BrakeFailure::kNone;
}

// 暂存先于最终命令到达的回执，并限制关联缓存容量。
void MaintenanceGate::rememberPendingStatus(
    const std::uint64_t command_id,
    const bool acknowledged,
    const BrakeFailure failure) {
  auto pending = pending_command_statuses_.find(command_id);
  if (pending == pending_command_statuses_.end()) {
    if (pending_command_statuses_.size() >= kPendingStatusCapacity) {
      latchFailure(BrakeFailure::kCorrelationOverflow);
      return;
    }
    pending = pending_command_statuses_
        .emplace(command_id, PendingCommandStatus{})
        .first;
  }

  if (failure != BrakeFailure::kNone) {
    pending->second.failure = failure;
    pending->second.acknowledged = false;
  } else if (acknowledged &&
      pending->second.failure == BrakeFailure::kNone) {
    pending->second.acknowledged = true;
  }
}

// 当最终命令出现时应用此前缓存的同命令回执。
void MaintenanceGate::applyPendingStatus(const std::uint64_t command_id) {
  const auto pending = pending_command_statuses_.find(command_id);
  if (pending == pending_command_statuses_.end()) {
    return;
  }

  const PendingCommandStatus status = pending->second;
  pending_command_statuses_.erase(pending);
  if (status.failure != BrakeFailure::kNone) {
    latchFailure(status.failure);
  } else if (status.acknowledged) {
    acknowledgeCurrentCommand();
  }
}

// 首次确认当前刹车命令后，开始等待新的硬件静止证据。
void MaintenanceGate::acknowledgeCurrentCommand() {
  if (!brake_acknowledged_) {
    brake_acknowledged_ = true;
    resetHardwareConfirmation();
  }
}

// 锁存刹车失败并丢弃所有等待中的回执关联。
void MaintenanceGate::latchFailure(const BrakeFailure failure) {
  brake_failure_ = failure;
  brake_acknowledged_ = false;
  pending_command_statuses_.clear();
  resetHardwareConfirmation();
}

// 清空一次维护申请的命令、回执和硬件证据。
void MaintenanceGate::resetEvidence() {
  command_gate_applied_ = false;
  current_command_id_ = 0u;
  brake_acknowledged_ = false;
  brake_failure_ = BrakeFailure::kNone;
  pending_command_statuses_.clear();
  resetHardwareEvidence();
}

// 清空连续零帧确认计数，但保留发布者会话信息。
void MaintenanceGate::resetHardwareConfirmation() {
  hardware_sample_observed_ = false;
  hardware_fresh_ = false;
  linear_speed_zero_ = false;
  angular_speed_zero_ = false;
  brush_off_ = false;
  consecutive_zero_frames_ = 0u;
}

// 清空硬件确认以及发布者会话和帧序号记录。
void MaintenanceGate::resetHardwareEvidence() {
  resetHardwareConfirmation();
  has_publisher_epoch_ = false;
  publisher_epoch_ = 0u;
  has_last_frame_sequence_ = false;
  last_frame_sequence_ = 0u;
}

}  // namespace mission
}  // namespace cleanbot
