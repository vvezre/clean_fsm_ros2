#include "cleanbot_control/command_arbiter_core.hpp"

#include <algorithm>
#include <utility>

namespace cleanbot {
namespace control {

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

bool MaintenanceGateCache::has_state() const {
  return has_state_;
}

bool MaintenanceGateCache::active() const {
  return active_;
}

std::uint64_t MaintenanceGateCache::generation() const {
  return generation_;
}

MaintenancePublisherCoordinator::MaintenancePublisherCoordinator(
    const std::size_t maximum_retired_identities,
    const std::uint64_t maximum_epoch)
    : publisher_tracker_(maximum_retired_identities, maximum_epoch) {}

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

bool MaintenancePublisherCoordinator::has_state() const {
  return gate_.has_state();
}

bool MaintenancePublisherCoordinator::active() const {
  return gate_.active();
}

std::uint64_t MaintenancePublisherCoordinator::generation() const {
  return gate_.generation();
}

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

CommandArbiterCore::CommandArbiterCore(const ArbiterParameters& parameters)
    : parameters_(parameters) {}

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

bool CommandArbiterCore::maintenance_active() const {
  return maintenance_gate_.active();
}

std::uint64_t CommandArbiterCore::maintenance_generation() const {
  return maintenance_gate_.generation();
}

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

void CommandArbiterCore::clearMaintenanceInputs() {
  clearAllSlots();
  has_operator_mode_ = false;
  brush_enabled_ = false;
  brush_speed_ = 0;
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
