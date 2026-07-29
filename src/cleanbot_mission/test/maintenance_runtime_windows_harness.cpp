#include "cleanbot_mission/maintenance_runtime.hpp"
#include "cleanbot_common/publisher_epoch_tracker.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cleanbot {
namespace mission {
namespace {

unsigned int load_count = 0u;

MaintenanceStoreResult result(
    const MaintenanceStoreCode code,
    std::string message,
    std::optional<MaintenanceStoreRecord> record = std::nullopt,
    const bool committed = false) {
  MaintenanceStoreResult value;
  value.code = code;
  value.message = std::move(message);
  value.record = std::move(record);
  value.committed = committed;
  return value;
}

MaintenanceStoreRecord inactive_record(const std::uint64_t generation) {
  MaintenanceStoreRecord record;
  record.last_generation = generation;
  return record;
}

MaintenanceStoreRecord active_record(
    const std::uint64_t generation,
    std::string requester,
    std::string reason) {
  MaintenanceStoreRecord record;
  record.last_generation = generation;
  record.inhibitor = MaintenanceInhibitor{
      generation, std::move(requester), std::move(reason)};
  return record;
}

}  // namespace

MaintenanceStore::MaintenanceStore(std::filesystem::path state_path)
    : state_path_(std::move(state_path)) {}

MaintenanceStoreResult MaintenanceStore::load() const noexcept {
  ++load_count;
  const auto name = state_path_.filename().string();
  if (name == "inactive" || name == "duplicate" ||
      name == "restore_failure" || name == "transition") {
    return result(
        MaintenanceStoreCode::kOk,
        "loaded",
        inactive_record(
            name == "duplicate" ? 8u :
            name == "restore_failure" ? 5u : 17u));
  }
  if (name == "active" || name == "mission_idle" ||
      name == "uncertain_release") {
    const auto generation =
        name == "active" ? 29u : name == "mission_idle" ? 51u : 6u;
    return result(
        MaintenanceStoreCode::kOk,
        "loaded",
        active_record(
            generation,
            name == "active" ? "ota-updater" : "updater",
            name == "active" ? "install-v3" : "install"));
  }
  if (name == "exhausted") {
    return result(
        MaintenanceStoreCode::kOk,
        "loaded",
        inactive_record(std::numeric_limits<std::uint64_t>::max()));
  }
  if (name == "uncertain_activate") {
    return result(
        MaintenanceStoreCode::kOk,
        "loaded",
        inactive_record(5u));
  }
  if (name == "malformed") {
    MaintenanceStoreRecord record;
    record.last_generation = 2u;
    MaintenanceInhibitor inhibitor;
    inhibitor.generation = 1u;
    inhibitor.requester = "updater";
    inhibitor.reason = "bad invariant";
    record.inhibitor = std::move(inhibitor);
    return result(MaintenanceStoreCode::kOk, "loaded", record);
  }
  if (name == "missing") {
    return result(MaintenanceStoreCode::kMissing, "missing");
  }
  if (name == "invalid") {
    return result(MaintenanceStoreCode::kInvalid, "invalid");
  }
  if (name == "unsupported") {
    return result(
        MaintenanceStoreCode::kUnsupportedSchema,
        "unsupported");
  }
  return result(MaintenanceStoreCode::kIoError, "io error");
}

MaintenanceStoreResult MaintenanceStore::activate(
    const std::string& requester,
    const std::string& reason) noexcept {
  const auto name = state_path_.filename().string();
  if (name == "transition") {
    return result(
        MaintenanceStoreCode::kOk,
        "activated",
        active_record(18u, requester, reason),
        true);
  }
  if (name == "active" && requester == "ota-updater") {
    return result(
        MaintenanceStoreCode::kAlreadyActive,
        "already active",
        active_record(29u, "ota-updater", "install-v3"));
  }
  if (name == "active") {
    return result(
        MaintenanceStoreCode::kOwnedByOther,
        "owned by other",
        active_record(29u, "ota-updater", "install-v3"));
  }
  if (name == "exhausted") {
    return result(
        MaintenanceStoreCode::kGenerationExhausted,
        "exhausted",
        inactive_record(std::numeric_limits<std::uint64_t>::max()));
  }
  if (name == "uncertain_activate") {
    return result(
        MaintenanceStoreCode::kIoError,
        "directory sync failed",
        active_record(6u, requester, reason),
        true);
  }
  return result(MaintenanceStoreCode::kInvalid, "invalid activate");
}

MaintenanceStoreResult MaintenanceStore::release(
    const std::uint64_t generation,
    const std::string& requester) noexcept {
  const auto name = state_path_.filename().string();
  if (name == "transition") {
    if (generation == 18u && requester == "ota-updater") {
      return result(
          MaintenanceStoreCode::kOk,
          "released",
          inactive_record(18u),
          true);
    }
    return result(
        MaintenanceStoreCode::kTokenMismatch,
        "token mismatch",
        active_record(18u, "ota-updater", "install-v4"));
  }
  if (name == "uncertain_release") {
    return result(
        MaintenanceStoreCode::kIoError,
        "directory sync failed",
        inactive_record(6u),
        true);
  }
  return result(MaintenanceStoreCode::kTokenMismatch, "token mismatch");
}

}  // namespace mission
}  // namespace cleanbot

namespace {

using cleanbot::mission::MaintenanceRuntime;
using cleanbot::mission::MaintenanceHardwareSample;
using cleanbot::mission::MaintenanceHardwareStatus;

void check(const bool condition, const char* const message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

MaintenanceRuntime runtime(const std::string& name) {
  return MaintenanceRuntime(
      std::filesystem::path("C:\\maintenance-runtime-test") / name);
}

void initial_case() {
  auto value = runtime("initial");
  const auto& snapshot = value.snapshot();
  check(!snapshot.initialized, "initial runtime must not be initialized");
  check(snapshot.admission_closed, "initial admission must be closed");
  check(!snapshot.ready, "initial runtime must not be ready");
  check(snapshot.phase == "STARTUP", "initial phase must be explicit");
  check(
      snapshot.blocker_code == "MAINTENANCE_NOT_INITIALIZED",
      "initial blocker must be explicit");
}

void inactive_case() {
  auto value = runtime("inactive");
  check(value.initialize(true), "inactive restore must succeed");
  const auto& snapshot = value.snapshot();
  check(!snapshot.store_fault, "inactive restore must be healthy");
  check(!snapshot.admission_closed, "inactive restore must open admission");
  check(snapshot.generation == 17u, "generation must expose last generation");
  check(snapshot.last_generation == 17u, "last generation must restore");
  check(!snapshot.gate_active, "inactive record must keep gate inactive");
  check(snapshot.mission_idle, "mission idle must restore");
  check(!snapshot.ready, "inactive gate is not maintenance-ready");
  check(snapshot.phase == "INACTIVE", "inactive phase must be coherent");
}

void active_case() {
  auto value = runtime("active");
  check(value.initialize(false), "active restore must succeed");
  const auto& snapshot = value.snapshot();
  check(!snapshot.store_fault, "active restore must be healthy");
  check(snapshot.admission_closed, "active restore must close admission");
  check(snapshot.generation == 29u, "active generation must restore");
  check(snapshot.last_generation == 29u, "last generation must restore");
  check(snapshot.gate_active, "active gate must restore");
  check(snapshot.requester == "ota-updater", "requester must restore");
  check(snapshot.reason == "install-v3", "reason must restore");
  check(!snapshot.command_gate_applied, "command evidence must be empty");
  check(!snapshot.brake_acknowledged, "brake evidence must be empty");
  check(!snapshot.hardware_fresh, "hardware evidence must be empty");
  check(!snapshot.ready, "active restore must not invent readiness");
  check(snapshot.phase == "MISSION_BUSY", "active phase must be coherent");
}

void fault_case(const std::string& name) {
  auto value = runtime(name);
  check(!value.initialize(true), "faulting restore must fail");
  const auto snapshot = value.snapshot();
  check(snapshot.initialized, "faulting restore counts as initialized");
  check(snapshot.store_fault, "store fault must latch");
  check(snapshot.admission_closed, "store fault must close admission");
  check(!snapshot.ready, "store fault must not be ready");
  check(snapshot.phase == "STORE_FAULT", "fault phase must be explicit");
  check(
      snapshot.blocker_code == "MAINTENANCE_STORE_FAULT",
      "fault blocker must be explicit");
  check(!snapshot.message.empty(), "fault message must be explicit");
  check(
      !value.initialize(false),
      "repeated faulting initialization must remain failed");
  check(cleanbot::mission::load_count == 1u, "fault must not be reread");
  check(
      value.snapshot().message == snapshot.message,
      "latched fault snapshot must not mutate");
}

void duplicate_case() {
  auto value = runtime("duplicate");
  check(value.initialize(true), "first initialization must succeed");
  const auto before = value.snapshot();
  check(value.initialize(false), "second initialization is idempotent");
  const auto& after = value.snapshot();
  check(cleanbot::mission::load_count == 1u, "store must be read once");
  check(after.generation == before.generation, "generation must not mutate");
  check(
      after.mission_idle == before.mission_idle,
      "mission idle must not mutate");
  check(
      after.admission_closed == before.admission_closed,
      "admission must not mutate");
}

void mission_idle_case() {
  auto value = runtime("mission_idle");
  check(value.initialize(false), "active restore must succeed");
  value.setMissionIdle(true);
  const auto& idle = value.snapshot();
  check(idle.mission_idle, "mission idle must forward true");
  check(idle.admission_closed, "active gate still closes admission");
  check(!idle.ready, "mission idle cannot invent evidence");
  check(
      idle.phase == "WAITING_FOR_COMMAND_GATE",
      "phase must reflect forwarded mission idle");
  value.setMissionIdle(false);
  const auto& busy = value.snapshot();
  check(!busy.mission_idle, "mission idle must forward false");
  check(busy.phase == "MISSION_BUSY", "busy phase must remain coherent");
}

void restore_failure_case() {
  auto value = runtime("restore_failure");
  value.setMissionIdle(true);
  check(!value.initialize(true), "non-pristine gate restore must fail");
  check(value.storeFault(), "restore failure must latch store fault");
  check(value.admissionClosed(), "restore failure must close admission");
  check(
      value.snapshot().phase == "STORE_FAULT",
      "restore failure phase must be explicit");
}

void transition_case() {
  auto value = runtime("transition");
  check(value.initialize(true), "inactive restore must succeed");
  const auto enabled =
      value.enable("ota-updater", "install-v4", true);
  check(enabled.accepted, "enable must be accepted");
  check(enabled.gate_active, "enable must activate gate");
  check(enabled.generation == 18u, "enable must return new generation");
  check(value.admissionClosed(), "enable must close admission");
  const auto wrong = value.disable(17u, "ota-updater");
  check(!wrong.accepted, "wrong token must be rejected");
  check(value.snapshot().gate_active, "wrong token must keep gate active");
  const auto released = value.disable(18u, "ota-updater");
  check(released.accepted, "exact release must be accepted");
  check(released.generation == 18u, "release must return released token");
  check(!released.gate_active, "release must deactivate gate");
  check(!value.admissionClosed(), "release must open admission");
  check(
      value.snapshot().generation == 18u,
      "inactive snapshot must expose last generation");
}

void idempotent_case() {
  auto value = runtime("active");
  check(value.initialize(true), "active restore must succeed");
  const auto retried =
      value.enable("ota-updater", "replacement", true);
  check(retried.accepted, "same owner retry must be accepted");
  check(retried.code == "ALREADY_ACTIVE", "retry code must be explicit");
  check(retried.generation == 29u, "retry must preserve generation");
  check(
      value.snapshot().reason == "install-v3",
      "retry must preserve original reason");
  const auto competing =
      value.enable("operator", "diagnostics", true);
  check(!competing.accepted, "other owner must be rejected");
  check(value.snapshot().gate_active, "other owner must keep gate active");
}

void exhausted_case() {
  auto value = runtime("exhausted");
  check(value.initialize(true), "inactive restore must succeed");
  const auto rejected = value.enable("updater", "install", true);
  check(!rejected.accepted, "exhausted generation must reject");
  check(
      rejected.code == "GENERATION_EXHAUSTED",
      "exhaustion code must be explicit");
  check(!value.storeFault(), "semantic exhaustion is not a store fault");
  check(
      !value.admissionClosed(),
      "healthy inactive exhaustion must keep admission open");
}

void uncertain_activate_case() {
  auto value = runtime("uncertain_activate");
  check(value.initialize(true), "inactive restore must succeed");
  const auto rejected = value.enable("updater", "install", true);
  check(!rejected.accepted, "uncertain activation must reject");
  check(value.storeFault(), "uncertain activation must latch fault");
  check(value.admissionClosed(), "uncertain activation must stay closed");
  check(
      value.snapshot().gate_active,
      "committed active record must retain safety clamp");
  check(
      value.snapshot().generation == 6u,
      "uncertain active generation must remain visible");
}

void uncertain_release_case() {
  auto value = runtime("uncertain_release");
  check(value.initialize(true), "active restore must succeed");
  const auto rejected = value.disable(6u, "updater");
  check(!rejected.accepted, "uncertain release must reject");
  check(value.storeFault(), "uncertain release must latch fault");
  check(value.admissionClosed(), "uncertain release must stay closed");
  check(
      value.snapshot().gate_active,
      "uncertain release must not release in-memory gate");
  check(
      value.snapshot().generation == 6u,
      "uncertain release must retain active generation");
}

void validation_case() {
  auto value = runtime("transition");
  check(value.initialize(true), "inactive restore must succeed");
  const auto invalid_enable = value.enable("", "install", true);
  check(!invalid_enable.accepted, "empty requester must reject");
  check(invalid_enable.code == "INVALID", "invalid code must be explicit");
  check(!value.storeFault(), "caller validation is not a store fault");
  check(
      !value.admissionClosed(),
      "invalid enable must keep healthy inactive admission open");
  check(
      value.enable("ota-updater", "install-v4", true).accepted,
      "valid enable must still succeed");
  const auto invalid_release = value.disable(18u, "");
  check(!invalid_release.accepted, "empty release owner must reject");
  check(!value.storeFault(), "invalid release is not a store fault");
  check(
      value.admissionClosed(),
      "invalid release must keep active admission closed");
}

cleanbot::common::PublisherIdentity publisher(const std::uint8_t tag) {
  cleanbot::common::PublisherIdentity identity;
  identity.implementation_identifier = "rmw_fastrtps_cpp";
  identity.gid = {tag, 0x44u};
  return identity;
}

cleanbot::mission::FinalCommandEvidence brake(
    const std::uint64_t generation,
    const std::uint64_t command_id) {
  cleanbot::mission::FinalCommandEvidence command;
  command.generation = generation;
  command.request_id = generation;
  command.command_id = command_id;
  command.source = "maintenance_gate";
  command.active = true;
  command.brake = true;
  return command;
}

cleanbot::mission::CommandStatusEvidence ack(
    const std::uint64_t generation,
    const std::uint64_t command_id) {
  cleanbot::mission::CommandStatusEvidence status;
  status.generation = generation;
  status.request_id = generation;
  status.command_id = command_id;
  status.source = "maintenance_gate";
  status.state = 2u;
  return status;
}

MaintenanceHardwareSample stopped(const std::uint64_t sequence) {
  MaintenanceHardwareSample sample;
  sample.frame_sequence = sequence;
  sample.connected = true;
  return sample;
}

void evidence_case() {
  auto value = runtime("transition");
  check(value.initialize(true), "inactive restore must succeed");
  check(
      value.enable("ota-updater", "install-v4", true).accepted,
      "enable must succeed");
  value.observeFinalCommand(brake(18u, 900u));
  value.observeCommandStatus(ack(18u, 900u));
  const auto first =
      value.observeHardware(publisher(1u), stopped(10u), 100u);
  check(
      first.status == MaintenanceHardwareStatus::kAccepted,
      "first publisher must be accepted");
  check(first.publisher_epoch == 1u, "first publisher epoch must be one");
  check(!value.snapshot().ready, "one frame must not be ready");
  value.refreshHardware(150u, 100u);
  check(!value.snapshot().ready, "duplicate refresh must not count twice");
  value.observeHardware(publisher(1u), stopped(11u), 160u);
  check(value.snapshot().ready, "second distinct frame must be ready");
  value.refreshHardware(261u, 100u);
  check(!value.snapshot().ready, "stale refresh must revoke ready");
  check(
      value.snapshot().blocker_code == "HARDWARE_NOT_FRESH",
      "stale blocker must be explicit");
}

void publisher_case() {
  auto value = runtime("transition");
  check(value.initialize(true), "inactive restore must succeed");
  check(
      value.enable("ota-updater", "install-v4", true).accepted,
      "enable must succeed");
  value.observeFinalCommand(brake(18u, 901u));
  value.observeCommandStatus(ack(18u, 901u));
  value.observeHardware(publisher(1u), stopped(1u), 10u);
  value.observeHardware(publisher(1u), stopped(2u), 20u);
  check(value.snapshot().ready, "first publisher must become ready");
  const auto switched =
      value.observeHardware(publisher(2u), stopped(1u), 30u);
  check(switched.session_changed, "new publisher must change session");
  check(!value.snapshot().ready, "new session must revoke old evidence");
  MaintenanceHardwareSample moving = stopped(999u);
  moving.x_speed = 50;
  const auto retired =
      value.observeHardware(publisher(1u), moving, 40u);
  check(
      retired.status == MaintenanceHardwareStatus::kRetired,
      "old publisher must retire");
  value.observeHardware(publisher(2u), stopped(2u), 50u);
  check(value.snapshot().ready, "current publisher must become ready");
  auto invalid = publisher(0u);
  invalid.gid = {0u, 0u};
  const auto revoked =
      value.observeHardware(invalid, stopped(3u), 60u);
  check(
      revoked.status == MaintenanceHardwareStatus::kRevoked,
      "invalid publisher must revoke readiness");
  check(!value.snapshot().ready, "invalid publisher must fail closed");
}

}  // namespace

int main(const int argc, const char* const argv[]) {
  try {
    check(argc == 2, "one case name is required");
    const std::string name(argv[1]);
    if (name == "initial") {
      initial_case();
    } else if (name == "inactive") {
      inactive_case();
    } else if (name == "active") {
      active_case();
    } else if (
        name == "missing" || name == "invalid" ||
        name == "unsupported" || name == "io" ||
        name == "malformed") {
      fault_case(name);
    } else if (name == "duplicate") {
      duplicate_case();
    } else if (name == "mission_idle") {
      mission_idle_case();
    } else if (name == "restore_failure") {
      restore_failure_case();
    } else if (name == "transition") {
      transition_case();
    } else if (name == "idempotent") {
      idempotent_case();
    } else if (name == "exhausted") {
      exhausted_case();
    } else if (name == "uncertain_activate") {
      uncertain_activate_case();
    } else if (name == "uncertain_release") {
      uncertain_release_case();
    } else if (name == "validation") {
      validation_case();
    } else if (name == "evidence") {
      evidence_case();
    } else if (name == "publisher") {
      publisher_case();
    } else {
      throw std::runtime_error("unknown case");
    }
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
