#include "cleanbot_mission/maintenance_runtime.hpp"

#include <exception>
#include <string>
#include <utility>

namespace cleanbot {
namespace mission {

MaintenanceRuntime::MaintenanceRuntime(
    std::filesystem::path state_path,
    const std::size_t maximum_retired_publishers,
    const std::uint64_t maximum_publisher_epoch)
    : store_(std::move(state_path)),
      hardware_publishers_(
          maximum_retired_publishers,
          maximum_publisher_epoch),
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

MaintenanceTransitionResult MaintenanceRuntime::enable(
    const std::string& requester,
    const std::string& reason,
    const bool mission_idle) noexcept {
  if (!initialized_) {
    return rejectUnavailable(
        "MAINTENANCE_NOT_INITIALIZED",
        "maintenance state has not been restored");
  }
  if (store_fault_) {
    return rejectUnavailable(
        "MAINTENANCE_STORE_FAULT",
        "maintenance store fault is latched");
  }
  if (requester.empty() ||
      requester.size() > MaintenanceStore::kMaximumRequesterBytes ||
      reason.size() > MaintenanceStore::kMaximumReasonBytes) {
    return transitionResult(
        false,
        MaintenanceStoreCode::kInvalid,
        "maintenance request fields are invalid",
        snapshot_.generation);
  }

  try {
    admission_closed_ = true;
    snapshot_.admission_closed = true;
    const MaintenanceStoreResult stored =
        store_.activate(requester, reason);

    if (stored.code == MaintenanceStoreCode::kOk) {
      if (!stored.committed || !stored.record.has_value() ||
          !validRecord(*stored.record) ||
          !stored.record->inhibitor.has_value()) {
        latchStoreFault(
            "maintenance activation returned an invalid committed record");
        return transitionResult(
            false, MaintenanceStoreCode::kInvalid,
            snapshot_.message, snapshot_.generation);
      }
      const auto& inhibitor = *stored.record->inhibitor;
      if (!gate_.request(inhibitor.generation, mission_idle)) {
        latchStoreFault(
            "maintenance gate rejected the committed activation");
        return transitionResult(
            false, MaintenanceStoreCode::kInvalid,
            snapshot_.message, snapshot_.generation);
      }
      record_ = *stored.record;
      if (!refreshHealthySnapshot(*record_)) {
        latchEmergencyFault();
        return rejectUnavailable(
            "MAINTENANCE_STORE_FAULT",
            "maintenance activation snapshot failed");
      }
      return transitionResult(
          true, stored.code, stored.message, inhibitor.generation);
    }

    if (stored.code == MaintenanceStoreCode::kAlreadyActive) {
      if (!stored.record.has_value() ||
          !validRecord(*stored.record) ||
          !stored.record->inhibitor.has_value() ||
          !snapshot_.gate_active ||
          stored.record->inhibitor->generation != snapshot_.generation ||
          stored.record->inhibitor->requester != requester) {
        latchStoreFault(
            "idempotent maintenance activation does not match runtime state");
        return transitionResult(
            false, MaintenanceStoreCode::kInvalid,
            snapshot_.message, snapshot_.generation);
      }
      record_ = *stored.record;
      if (!refreshHealthySnapshot(*record_)) {
        latchEmergencyFault();
        return rejectUnavailable(
            "MAINTENANCE_STORE_FAULT",
            "maintenance activation snapshot failed");
      }
      return transitionResult(
          true, stored.code, stored.message,
          stored.record->inhibitor->generation);
    }

    if (stored.committed || isOperationalFailure(stored.code)) {
      applyUncertainActivation(stored, mission_idle);
      latchStoreFault(
          std::string("maintenance activation failed (") +
          storeCodeName(stored.code) + "): " + stored.message);
      return transitionResult(
          false, stored.code, snapshot_.message, snapshot_.generation);
    }

    restoreAdmissionFromRecord();
    return transitionResult(
        false, stored.code, stored.message, snapshot_.generation);
  } catch (...) {
    latchEmergencyFault();
    return rejectUnavailable(
        "MAINTENANCE_STORE_FAULT",
        "maintenance activation failed unexpectedly");
  }
}

MaintenanceTransitionResult MaintenanceRuntime::disable(
    const std::uint64_t generation,
    const std::string& requester) noexcept {
  if (!initialized_) {
    return rejectUnavailable(
        "MAINTENANCE_NOT_INITIALIZED",
        "maintenance state has not been restored");
  }
  if (store_fault_) {
    return rejectUnavailable(
        "MAINTENANCE_STORE_FAULT",
        "maintenance store fault is latched");
  }
  if (generation == 0u || requester.empty() ||
      requester.size() > MaintenanceStore::kMaximumRequesterBytes) {
    return transitionResult(
        false,
        MaintenanceStoreCode::kInvalid,
        "maintenance release fields are invalid",
        snapshot_.generation);
  }

  try {
    admission_closed_ = true;
    snapshot_.admission_closed = true;
    const MaintenanceStoreResult stored =
        store_.release(generation, requester);

    if (stored.code == MaintenanceStoreCode::kOk) {
      if (!stored.committed || !stored.record.has_value() ||
          !validRecord(*stored.record) ||
          stored.record->inhibitor.has_value() ||
          stored.record->last_generation != generation) {
        latchStoreFault(
            "maintenance release returned an invalid committed record");
        return transitionResult(
            false, MaintenanceStoreCode::kInvalid,
            snapshot_.message, snapshot_.generation);
      }
      if (!gate_.release(generation)) {
        latchStoreFault(
            "maintenance gate rejected the committed release");
        return transitionResult(
            false, MaintenanceStoreCode::kInvalid,
            snapshot_.message, snapshot_.generation);
      }
      record_ = *stored.record;
      if (!refreshHealthySnapshot(*record_)) {
        latchEmergencyFault();
        return rejectUnavailable(
            "MAINTENANCE_STORE_FAULT",
            "maintenance release snapshot failed");
      }
      return transitionResult(
          true, stored.code, stored.message, generation);
    }

    if (stored.committed || isOperationalFailure(stored.code)) {
      latchStoreFault(
          std::string("maintenance release failed (") +
          storeCodeName(stored.code) + "): " + stored.message);
      return transitionResult(
          false, stored.code, snapshot_.message, snapshot_.generation);
    }

    restoreAdmissionFromRecord();
    return transitionResult(
        false, stored.code, stored.message, snapshot_.generation);
  } catch (...) {
    latchEmergencyFault();
    return rejectUnavailable(
        "MAINTENANCE_STORE_FAULT",
        "maintenance release failed unexpectedly");
  }
}

void MaintenanceRuntime::observeFinalCommand(
    const FinalCommandEvidence& command) noexcept {
  if (!initialized_ || store_fault_) {
    return;
  }
  try {
    gate_.observeFinalCommand(command);
    restoreAdmissionFromRecord();
  } catch (...) {
    latchEmergencyFault();
  }
}

void MaintenanceRuntime::observeCommandStatus(
    const CommandStatusEvidence& status) noexcept {
  if (!initialized_ || store_fault_) {
    return;
  }
  try {
    gate_.observeCommandStatus(status);
    restoreAdmissionFromRecord();
  } catch (...) {
    latchEmergencyFault();
  }
}

MaintenanceHardwareObservation MaintenanceRuntime::observeHardware(
    const cleanbot::common::PublisherIdentity& publisher,
    const MaintenanceHardwareSample& sample,
    const std::uint64_t observed_at_nanoseconds) noexcept {
  if (!initialized_ || store_fault_) {
    return MaintenanceHardwareObservation{};
  }

  try {
    auto candidate = hardware_publishers_;
    const auto observed = candidate.observe(publisher);
    if (observed.status ==
        cleanbot::common::PublisherEpochStatus::kRetired) {
      return MaintenanceHardwareObservation{
          MaintenanceHardwareStatus::kRetired,
          observed.epoch,
          false};
    }
    if (observed.status !=
        cleanbot::common::PublisherEpochStatus::kAccepted) {
      HardwareEvidence revoked;
      revoked.publisher_epoch = 0u;
      revoked.frame_sequence = sample.frame_sequence;
      revoked.fresh = false;
      revoked.connected = false;
      revoked.x_speed = sample.x_speed;
      revoked.z_speed = sample.z_speed;
      revoked.brush_speed = sample.brush_speed;
      gate_.observeHardware(revoked);
      restoreAdmissionFromRecord();
      return MaintenanceHardwareObservation{
          MaintenanceHardwareStatus::kRevoked, 0u, false};
    }

    hardware_publishers_ = std::move(candidate);
    HardwareEvidence evidence;
    evidence.publisher_epoch = observed.epoch;
    evidence.frame_sequence = sample.frame_sequence;
    evidence.fresh = true;
    evidence.connected = sample.connected;
    evidence.x_speed = sample.x_speed;
    evidence.z_speed = sample.z_speed;
    evidence.brush_speed = sample.brush_speed;
    latest_hardware_ = evidence;
    latest_hardware_observed_at_nanoseconds_ =
        observed_at_nanoseconds;
    has_latest_hardware_ = true;
    gate_.observeHardware(evidence);
    restoreAdmissionFromRecord();
    return MaintenanceHardwareObservation{
        MaintenanceHardwareStatus::kAccepted,
        observed.epoch,
        observed.session_changed};
  } catch (...) {
    latchEmergencyFault();
    return MaintenanceHardwareObservation{};
  }
}

void MaintenanceRuntime::refreshHardware(
    const std::uint64_t now_nanoseconds,
    const std::uint64_t freshness_timeout_nanoseconds) noexcept {
  if (!initialized_ || store_fault_ || !has_latest_hardware_) {
    return;
  }

  try {
    HardwareEvidence refreshed = latest_hardware_;
    refreshed.fresh =
        now_nanoseconds >= latest_hardware_observed_at_nanoseconds_ &&
        now_nanoseconds - latest_hardware_observed_at_nanoseconds_ <=
            freshness_timeout_nanoseconds;
    gate_.observeHardware(refreshed);
    restoreAdmissionFromRecord();
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

bool MaintenanceRuntime::isOperationalFailure(
    const MaintenanceStoreCode code) noexcept {
  return code == MaintenanceStoreCode::kMissing ||
      code == MaintenanceStoreCode::kInvalid ||
      code == MaintenanceStoreCode::kUnsupportedSchema ||
      code == MaintenanceStoreCode::kIoError;
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
  try {
    record_ = record;
  } catch (...) {
    latchEmergencyFault();
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

MaintenanceTransitionResult MaintenanceRuntime::transitionResult(
    const bool accepted,
    const MaintenanceStoreCode code,
    const std::string& message,
    const std::uint64_t generation) const noexcept {
  try {
    MaintenanceTransitionResult result;
    result.accepted = accepted;
    result.gate_active = snapshot_.gate_active;
    result.ready = snapshot_.ready;
    result.generation = generation;
    result.code = storeCodeName(code);
    result.message = message;
    return result;
  } catch (...) {
    return MaintenanceTransitionResult{};
  }
}

MaintenanceTransitionResult MaintenanceRuntime::rejectUnavailable(
    const char* const code,
    const char* const message) const noexcept {
  try {
    MaintenanceTransitionResult result;
    result.gate_active = snapshot_.gate_active;
    result.ready = false;
    result.generation = snapshot_.generation;
    result.code = code;
    result.message = message;
    return result;
  } catch (...) {
    return MaintenanceTransitionResult{};
  }
}

void MaintenanceRuntime::restoreAdmissionFromRecord() noexcept {
  if (!record_.has_value() ||
      !refreshHealthySnapshot(*record_)) {
    latchEmergencyFault();
  }
}

void MaintenanceRuntime::applyUncertainActivation(
    const MaintenanceStoreResult& result,
    const bool mission_idle) noexcept {
  if (!result.committed || !result.record.has_value() ||
      !validRecord(*result.record) ||
      !result.record->inhibitor.has_value()) {
    return;
  }

  try {
    const auto& inhibitor = *result.record->inhibitor;
    if (!snapshot_.gate_active) {
      if (!gate_.request(inhibitor.generation, mission_idle)) {
        return;
      }
    } else if (snapshot_.generation != inhibitor.generation) {
      return;
    }
    record_ = *result.record;
    refreshHealthySnapshot(*record_);
  } catch (...) {
    latchEmergencyFault();
  }
}

void MaintenanceRuntime::latchStoreFault(
    const std::string& message) noexcept {
  initialized_ = true;
  store_fault_ = true;
  admission_closed_ = true;
  try {
    MaintenanceRuntimeSnapshot fault = snapshot_;
    const MaintenanceGateSnapshot gate_snapshot = gate_.snapshot();
    fault.initialized = true;
    fault.store_fault = true;
    fault.admission_closed = true;
    fault.generation = gate_snapshot.gate_active
        ? gate_snapshot.generation
        : gate_.lastGeneration();
    fault.last_generation = gate_.lastGeneration();
    fault.gate_active = gate_snapshot.gate_active;
    fault.mission_idle = gate_snapshot.mission_idle;
    fault.command_gate_applied = gate_snapshot.command_gate_applied;
    fault.brake_acknowledged = gate_snapshot.brake_acknowledged;
    fault.hardware_fresh = gate_snapshot.hardware_fresh;
    fault.linear_speed_zero = gate_snapshot.linear_speed_zero;
    fault.angular_speed_zero = gate_snapshot.angular_speed_zero;
    fault.brush_off = gate_snapshot.brush_off;
    fault.ready = false;
    fault.phase = "STORE_FAULT";
    fault.blocker_code = "MAINTENANCE_STORE_FAULT";
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
