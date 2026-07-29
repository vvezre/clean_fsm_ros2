#include "cleanbot_control/command_arbiter_core.hpp"

#include <algorithm>

namespace cleanbot {
namespace control {

CommandArbiterCore::CommandArbiterCore(const ArbiterParameters& parameters)
    : parameters_(parameters) {}

bool CommandArbiterCore::update(
    const CommandSource source,
    const ControlCommand& command,
    const std::uint64_t now_ms) {
  CommandSlot& destination = slot(source);
  if (!command.active) {
    destination.present = false;
    return true;
  }

  if (source == CommandSource::kEmergency && command.brake) {
    stop_boundary_command_id_ = std::max(stop_boundary_command_id_, command.command_id);
    stop_boundary_stamp_ms_ = std::max(stop_boundary_stamp_ms_, command.stamp_ms);
    software_stopped_ = true;
    brush_enabled_ = false;
    brush_speed_ = 0;
    has_operator_mode_ = false;
    clearAllSlots();
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

void CommandArbiterCore::set_brush(
    const bool enabled, const std::int32_t speed, const bool operator_intent) {
  if (!operator_intent && software_stopped_) {
    return;
  }
  brush_enabled_ = enabled;
  brush_speed_ = enabled ? std::max(-100, std::min(100, speed)) : 0;
}

ControlCommand CommandArbiterCore::output(const std::uint64_t now_ms) {
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

bool CommandArbiterCore::software_stopped() const { return software_stopped_; }

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

void CommandArbiterCore::clearOperatorSlots() {
  manual_.present = false;
  mission_.present = false;
  vision_.present = false;
}

void CommandArbiterCore::clearAllSlots() {
  emergency_.present = false;
  safety_.present = false;
  clearOperatorSlots();
}

bool CommandArbiterCore::isNewerThanStop(const ControlCommand& command) const {
  if (stop_boundary_stamp_ms_ != 0u && command.stamp_ms != 0u) {
    return command.stamp_ms > stop_boundary_stamp_ms_;
  }
  return command.command_id != 0u && command.command_id > stop_boundary_command_id_;
}

ControlCommand CommandArbiterCore::brakingOutput(const std::string& source) {
  ControlCommand result;
  result.source = source;
  result.priority = source == "software_emergency_stop" ? 100u : 0u;
  result.active = true;
  result.brake = true;
  return result;
}

}  // namespace control
}  // namespace cleanbot
