#include "cleanbot_mission/maintenance_runtime.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace cleanbot {
namespace mission {
namespace {

unsigned int load_count = 0u;

MaintenanceStoreResult result(
    const MaintenanceStoreCode code,
    std::string message,
    std::optional<MaintenanceStoreRecord> record = std::nullopt) {
  MaintenanceStoreResult value;
  value.code = code;
  value.message = std::move(message);
  value.record = std::move(record);
  return value;
}

}  // namespace

MaintenanceStore::MaintenanceStore(std::filesystem::path state_path)
    : state_path_(std::move(state_path)) {}

MaintenanceStoreResult MaintenanceStore::load() const noexcept {
  ++load_count;
  const auto name = state_path_.filename().string();
  if (name == "inactive" || name == "duplicate" ||
      name == "restore_failure") {
    MaintenanceStoreRecord record;
    record.last_generation = name == "duplicate" ? 8u :
        name == "restore_failure" ? 5u : 17u;
    return result(MaintenanceStoreCode::kOk, "loaded", record);
  }
  if (name == "active" || name == "mission_idle") {
    MaintenanceStoreRecord record;
    record.last_generation = name == "active" ? 29u : 51u;
    MaintenanceInhibitor inhibitor;
    inhibitor.generation = record.last_generation;
    inhibitor.requester =
        name == "active" ? "ota-updater" : "updater";
    inhibitor.reason =
        name == "active" ? "install-v3" : "install";
    record.inhibitor = std::move(inhibitor);
    return result(MaintenanceStoreCode::kOk, "loaded", record);
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

}  // namespace mission
}  // namespace cleanbot

namespace {

using cleanbot::mission::MaintenanceRuntime;

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
    } else {
      throw std::runtime_error("unknown case");
    }
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
