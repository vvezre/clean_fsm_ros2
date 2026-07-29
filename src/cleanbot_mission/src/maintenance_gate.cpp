#include "cleanbot_mission/maintenance_gate.hpp"

#include <algorithm>

namespace cleanbot {
namespace mission {

namespace {

constexpr std::uint8_t kStateAcknowledged = 2u;
constexpr std::uint8_t kStateRejected = 3u;
constexpr std::uint8_t kStateTimedOut = 4u;
constexpr std::uint8_t kStateTransportLost = 7u;
constexpr std::uint8_t kStateSuperseded = 8u;

}  // namespace

bool MaintenanceGate::request(
    const std::uint64_t generation,
    const bool mission_idle) {
  if (generation == 0u || generation <= last_generation_) {
    return false;
  }

  active_ = true;
  generation_ = generation;
  last_generation_ = generation;
  mission_idle_ = mission_idle;
  resetEvidence();
  return true;
}

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

void MaintenanceGate::setMissionIdle(const bool mission_idle) {
  mission_idle_ = mission_idle;
}

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

void MaintenanceGate::observeCommandStatus(
    const CommandStatusEvidence& status) {
  if (!active_ || brake_failure_ != BrakeFailure::kNone ||
      !matches(status.generation, status.request_id, status.source) ||
      status.command_id == 0u) {
    return;
  }

  const bool acknowledged = status.state == kStateAcknowledged;
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

std::uint64_t MaintenanceGate::lastGeneration() const {
  return last_generation_;
}

std::size_t MaintenanceGate::pendingStatusCapacity() const {
  return kPendingStatusCapacity;
}

bool MaintenanceGate::matches(
    const std::uint64_t generation,
    const std::uint64_t request_id,
    const std::string& source) const {
  return generation == generation_ && request_id == generation_ &&
      source == "maintenance_gate";
}

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

void MaintenanceGate::acknowledgeCurrentCommand() {
  if (!brake_acknowledged_) {
    brake_acknowledged_ = true;
    resetHardwareConfirmation();
  }
}

void MaintenanceGate::latchFailure(const BrakeFailure failure) {
  brake_failure_ = failure;
  brake_acknowledged_ = false;
  pending_command_statuses_.clear();
  resetHardwareConfirmation();
}

void MaintenanceGate::resetEvidence() {
  command_gate_applied_ = false;
  current_command_id_ = 0u;
  brake_acknowledged_ = false;
  brake_failure_ = BrakeFailure::kNone;
  pending_command_statuses_.clear();
  resetHardwareEvidence();
}

void MaintenanceGate::resetHardwareConfirmation() {
  hardware_sample_observed_ = false;
  hardware_fresh_ = false;
  linear_speed_zero_ = false;
  angular_speed_zero_ = false;
  brush_off_ = false;
  consecutive_zero_frames_ = 0u;
}

void MaintenanceGate::resetHardwareEvidence() {
  resetHardwareConfirmation();
  has_publisher_epoch_ = false;
  publisher_epoch_ = 0u;
  has_last_frame_sequence_ = false;
  last_frame_sequence_ = 0u;
}

}  // namespace mission
}  // namespace cleanbot
