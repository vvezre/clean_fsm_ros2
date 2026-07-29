#include "cleanbot_mission/maintenance_runtime.hpp"

#include <exception>
#include <string>
#include <utility>

namespace cleanbot {
namespace mission {

MaintenanceRuntime::MaintenanceRuntime(std::filesystem::path state_path)
    : store_(std::move(state_path)),
      snapshot_(initialSnapshot()),
      emergency_fault_snapshot_(emergencyFaultSnapshot()) {}

bool MaintenanceRuntime::initialize(const bool mission_idle) noexcept {
  if (initialized_) {
    return !store_fault_;
  }

  initialized_ = true;
  snapshot_.initialized = true;
  const MaintenanceStoreResult loaded = store_.load();
  if (loaded.code != MaintenanceStoreCode::kOk) {
    try {
      latchStoreFault(
          std::string("maintenance state restore failed (") +
          storeCodeName(loaded.code) + "): " + loaded.message);
    } catch (...) {
      latchEmergencyFault();
    }
    return false;
  }
  if (!loaded.record.has_value() || !validRecord(*loaded.record)) {
    latchStoreFault("maintenance state record invariant is invalid");
    return false;
  }
  return restore(*loaded.record, mission_idle);
}

void MaintenanceRuntime::setMissionIdle(const bool mission_idle) noexcept {
  if (store_fault_) {
    return;
  }

  try {
    gate_.setMissionIdle(mission_idle);
    if (!initialized_) {
      snapshot_.mission_idle = mission_idle;
      return;
    }
    if (!refreshHealthySnapshot(snapshot_.requester, snapshot_.reason)) {
      latchEmergencyFault();
    }
  } catch (...) {
    latchEmergencyFault();
  }
}

const MaintenanceRuntimeSnapshot& MaintenanceRuntime::snapshot()
    const noexcept {
  return snapshot_;
}

bool MaintenanceRuntime::initialized() const noexcept {
  return initialized_;
}

bool MaintenanceRuntime::storeFault() const noexcept {
  return store_fault_;
}

bool MaintenanceRuntime::admissionClosed() const noexcept {
  return admission_closed_;
}

std::uint64_t MaintenanceRuntime::lastGeneration() const noexcept {
  return gate_.lastGeneration();
}

MaintenanceRuntimeSnapshot MaintenanceRuntime::initialSnapshot() {
  MaintenanceRuntimeSnapshot result;
  result.admission_closed = true;
  result.phase = "STARTUP";
  result.blocker_code = "MAINTENANCE_NOT_INITIALIZED";
  result.message =
      "maintenance state has not been restored; mission admission is closed";
  return result;
}

MaintenanceRuntimeSnapshot MaintenanceRuntime::emergencyFaultSnapshot() {
  MaintenanceRuntimeSnapshot result;
  result.initialized = true;
  result.store_fault = true;
  result.admission_closed = true;
  result.phase = "STORE_FAULT";
  result.blocker_code = "MAINTENANCE_STORE_FAULT";
  result.message =
      "maintenance state restore failed; mission admission is closed";
  return result;
}

bool MaintenanceRuntime::validRecord(
    const MaintenanceStoreRecord& record) noexcept {
  if (record.schema_version != 1u) {
    return false;
  }
  if (!record.inhibitor.has_value()) {
    return true;
  }
  const auto& inhibitor = *record.inhibitor;
  return inhibitor.generation != 0u &&
      inhibitor.generation == record.last_generation &&
      !inhibitor.requester.empty() &&
      inhibitor.requester.size() <=
          MaintenanceStore::kMaximumRequesterBytes &&
      inhibitor.reason.size() <= MaintenanceStore::kMaximumReasonBytes;
}

const char* MaintenanceRuntime::storeCodeName(
    const MaintenanceStoreCode code) noexcept {
  switch (code) {
    case MaintenanceStoreCode::kOk:
      return "OK";
    case MaintenanceStoreCode::kMissing:
      return "MISSING";
    case MaintenanceStoreCode::kInvalid:
      return "INVALID";
    case MaintenanceStoreCode::kUnsupportedSchema:
      return "UNSUPPORTED_SCHEMA";
    case MaintenanceStoreCode::kIoError:
      return "IO_ERROR";
    case MaintenanceStoreCode::kAlreadyActive:
      return "ALREADY_ACTIVE";
    case MaintenanceStoreCode::kOwnedByOther:
      return "OWNED_BY_OTHER";
    case MaintenanceStoreCode::kTokenMismatch:
      return "TOKEN_MISMATCH";
    case MaintenanceStoreCode::kGenerationExhausted:
      return "GENERATION_EXHAUSTED";
  }
  return "UNKNOWN";
}

void MaintenanceRuntime::swapSnapshots(
    MaintenanceRuntimeSnapshot& lhs,
    MaintenanceRuntimeSnapshot& rhs) noexcept {
  using std::swap;
  swap(lhs.initialized, rhs.initialized);
  swap(lhs.store_fault, rhs.store_fault);
  swap(lhs.admission_closed, rhs.admission_closed);
  swap(lhs.generation, rhs.generation);
  swap(lhs.last_generation, rhs.last_generation);
  swap(lhs.gate_active, rhs.gate_active);
  swap(lhs.mission_idle, rhs.mission_idle);
  swap(lhs.command_gate_applied, rhs.command_gate_applied);
  swap(lhs.brake_acknowledged, rhs.brake_acknowledged);
  swap(lhs.hardware_fresh, rhs.hardware_fresh);
  swap(lhs.linear_speed_zero, rhs.linear_speed_zero);
  swap(lhs.angular_speed_zero, rhs.angular_speed_zero);
  swap(lhs.brush_off, rhs.brush_off);
  swap(lhs.ready, rhs.ready);
  lhs.requester.swap(rhs.requester);
  lhs.reason.swap(rhs.reason);
  lhs.phase.swap(rhs.phase);
  lhs.blocker_code.swap(rhs.blocker_code);
  lhs.message.swap(rhs.message);
}

bool MaintenanceRuntime::restore(
    const MaintenanceStoreRecord& record,
    const bool mission_idle) noexcept {
  bool restored = false;
  if (record.inhibitor.has_value()) {
    restored = gate_.restorePersistentState(
        record.last_generation,
        record.inhibitor->generation,
        mission_idle);
  } else {
    restored = gate_.restorePersistentState(
        record.last_generation,
        mission_idle);
  }
  if (!restored) {
    latchStoreFault(
        "maintenance gate rejected the persistent state restore");
    return false;
  }
  if (!refreshHealthySnapshot(record)) {
    latchEmergencyFault();
    return false;
  }
  return true;
}

bool MaintenanceRuntime::refreshHealthySnapshot(
    const MaintenanceStoreRecord& record) noexcept {
  try {
    const std::string requester = record.inhibitor.has_value()
        ? record.inhibitor->requester
        : std::string();
    const std::string reason = record.inhibitor.has_value()
        ? record.inhibitor->reason
        : std::string();
    return refreshHealthySnapshot(requester, reason);
  } catch (...) {
    return false;
  }
}

bool MaintenanceRuntime::refreshHealthySnapshot(
    const std::string& requester,
    const std::string& reason) noexcept {
  try {
    const MaintenanceGateSnapshot gate_snapshot = gate_.snapshot();
    MaintenanceRuntimeSnapshot next;
    next.initialized = true;
    next.store_fault = false;
    next.admission_closed = gate_snapshot.gate_active;
    next.generation = gate_snapshot.gate_active
        ? gate_snapshot.generation
        : gate_.lastGeneration();
    next.last_generation = gate_.lastGeneration();
    next.gate_active = gate_snapshot.gate_active;
    next.mission_idle = gate_snapshot.mission_idle;
    next.command_gate_applied = gate_snapshot.command_gate_applied;
    next.brake_acknowledged = gate_snapshot.brake_acknowledged;
    next.hardware_fresh = gate_snapshot.hardware_fresh;
    next.linear_speed_zero = gate_snapshot.linear_speed_zero;
    next.angular_speed_zero = gate_snapshot.angular_speed_zero;
    next.brush_off = gate_snapshot.brush_off;
    next.ready = gate_snapshot.ready;
    next.requester = requester;
    next.reason = reason;
    next.phase = gate_snapshot.phase;
    next.blocker_code = gate_snapshot.blocker_code;
    next.message = gate_snapshot.message;
    swapSnapshots(snapshot_, next);
    store_fault_ = false;
    admission_closed_ = snapshot_.admission_closed;
    return true;
  } catch (...) {
    return false;
  }
}

void MaintenanceRuntime::latchStoreFault(
    const std::string& message) noexcept {
  initialized_ = true;
  store_fault_ = true;
  admission_closed_ = true;
  try {
    MaintenanceRuntimeSnapshot fault = emergencyFaultSnapshot();
    fault.message = message;
    swapSnapshots(snapshot_, fault);
  } catch (...) {
    latchEmergencyFault();
  }
}

void MaintenanceRuntime::latchEmergencyFault() noexcept {
  initialized_ = true;
  store_fault_ = true;
  admission_closed_ = true;
  swapSnapshots(snapshot_, emergency_fault_snapshot_);
}

}  // namespace mission
}  // namespace cleanbot
