#include "cleanbot_mission/maintenance_runtime.hpp"
#include "cleanbot_common/publisher_epoch_tracker.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cleanbot {
namespace mission {
namespace {

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<std::uint64_t> sequence{0u};
    const auto nonce =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
        ("cleanbot-maintenance-runtime-" + std::to_string(nonce) + "-" +
        std::to_string(sequence.fetch_add(1u)));
    std::filesystem::create_directory(path_);
  }

  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::permissions(
        path_,
        std::filesystem::perms::owner_all,
        std::filesystem::perm_options::add,
        ignored);
    for (const auto& entry : std::filesystem::directory_iterator(
             path_, std::filesystem::directory_options::skip_permission_denied,
             ignored)) {
      std::filesystem::permissions(
          entry.path(),
          std::filesystem::perms::owner_all,
          std::filesystem::perm_options::add,
          ignored);
    }
    std::filesystem::remove_all(path_, ignored);
  }

  const std::filesystem::path& path() const {
    return path_;
  }

 private:
  std::filesystem::path path_;
};

void write_bytes(
    const std::filesystem::path& path,
    const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(output.good());
}

std::string read_bytes(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(input),
      std::istreambuf_iterator<char>());
}

std::string inactive_json(const std::uint64_t generation) {
  return "{\"schemaVersion\":1,\"lastGeneration\":" +
      std::to_string(generation) +
      ",\"state\":\"inactive\",\"inhibitor\":null}\n";
}

std::string active_json(
    const std::uint64_t generation,
    const std::string& requester,
    const std::string& reason) {
  return "{\"schemaVersion\":1,\"lastGeneration\":" +
      std::to_string(generation) +
      ",\"state\":\"active\",\"inhibitor\":{\"generation\":" +
      std::to_string(generation) + ",\"requester\":\"" + requester +
      "\",\"reason\":\"" + reason + "\"}}\n";
}

static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().initialize(true)));
static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().setMissionIdle(true)));
static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().enable(
        std::declval<const std::string&>(),
        std::declval<const std::string&>(),
        true)));
static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().disable(
        1u, std::declval<const std::string&>())));
static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().observeFinalCommand(
        std::declval<const FinalCommandEvidence&>())));
static_assert(noexcept(
    std::declval<MaintenanceRuntime&>().observeCommandStatus(
        std::declval<const CommandStatusEvidence&>())));
static_assert(noexcept(
    std::declval<const MaintenanceRuntime&>().snapshot()));
static_assert(noexcept(
    std::declval<const MaintenanceRuntime&>().admissionClosed()));
static_assert(noexcept(
    std::declval<const MaintenanceRuntime&>().storeFault()));
static_assert(noexcept(
    std::declval<const MaintenanceRuntime&>().lastGeneration()));

TEST(MaintenanceRuntimeTest, StartsFailClosedBeforeInitialization) {
  TemporaryDirectory temporary;
  MaintenanceRuntime runtime(temporary.path() / "maintenance.json");

  const auto& snapshot = runtime.snapshot();

  EXPECT_FALSE(snapshot.initialized);
  EXPECT_TRUE(snapshot.admission_closed);
  EXPECT_FALSE(snapshot.ready);
  EXPECT_EQ(snapshot.phase, "STARTUP");
  EXPECT_EQ(snapshot.blocker_code, "MAINTENANCE_NOT_INITIALIZED");
}

TEST(MaintenanceRuntimeTest, RestoresInactiveRecordAndOpensAdmission) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(17u));
  MaintenanceRuntime runtime(state_path);

  ASSERT_TRUE(runtime.initialize(true));
  const auto& snapshot = runtime.snapshot();

  EXPECT_TRUE(snapshot.initialized);
  EXPECT_FALSE(snapshot.store_fault);
  EXPECT_FALSE(runtime.storeFault());
  EXPECT_FALSE(snapshot.admission_closed);
  EXPECT_FALSE(runtime.admissionClosed());
  EXPECT_EQ(snapshot.generation, 17u);
  EXPECT_EQ(snapshot.last_generation, 17u);
  EXPECT_FALSE(snapshot.gate_active);
  EXPECT_TRUE(snapshot.mission_idle);
  EXPECT_FALSE(snapshot.ready);
  EXPECT_EQ(snapshot.phase, "INACTIVE");
}

TEST(MaintenanceRuntimeTest, RestoresExactActiveOwnerWithoutEvidence) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, active_json(29u, "ota-updater", "install-v3"));
  MaintenanceRuntime runtime(state_path);

  ASSERT_TRUE(runtime.initialize(false));
  const auto& snapshot = runtime.snapshot();

  EXPECT_FALSE(snapshot.store_fault);
  EXPECT_TRUE(snapshot.admission_closed);
  EXPECT_EQ(snapshot.generation, 29u);
  EXPECT_EQ(snapshot.last_generation, 29u);
  EXPECT_TRUE(snapshot.gate_active);
  EXPECT_EQ(snapshot.requester, "ota-updater");
  EXPECT_EQ(snapshot.reason, "install-v3");
  EXPECT_FALSE(snapshot.mission_idle);
  EXPECT_FALSE(snapshot.command_gate_applied);
  EXPECT_FALSE(snapshot.brake_acknowledged);
  EXPECT_FALSE(snapshot.hardware_fresh);
  EXPECT_FALSE(snapshot.ready);
  EXPECT_EQ(snapshot.phase, "MISSION_BUSY");
}

TEST(MaintenanceRuntimeTest, MissingInvalidAndUnsupportedRecordsLatchFault) {
  const std::string invalid =
      "{\"schemaVersion\":1,\"lastGeneration\":2,\"state\":\"active\","
      "\"inhibitor\":{\"generation\":1,\"requester\":\"updater\","
      "\"reason\":\"bad invariant\"}}\n";
  const std::string unsupported =
      "{\"schemaVersion\":2,\"lastGeneration\":0,\"state\":\"inactive\","
      "\"inhibitor\":null}\n";

  for (const auto& bytes : {std::string(), invalid, unsupported}) {
    TemporaryDirectory temporary;
    const auto state_path = temporary.path() / "maintenance.json";
    if (!bytes.empty()) {
      write_bytes(state_path, bytes);
    }
    MaintenanceRuntime runtime(state_path);

    EXPECT_FALSE(runtime.initialize(true));
    const auto& snapshot = runtime.snapshot();
    EXPECT_TRUE(snapshot.initialized);
    EXPECT_TRUE(snapshot.store_fault);
    EXPECT_TRUE(snapshot.admission_closed);
    EXPECT_FALSE(snapshot.ready);
    EXPECT_EQ(snapshot.phase, "STORE_FAULT");
    EXPECT_EQ(snapshot.blocker_code, "MAINTENANCE_STORE_FAULT");
    EXPECT_FALSE(std::filesystem::exists(state_path) && bytes.empty());
    if (!bytes.empty()) {
      EXPECT_EQ(read_bytes(state_path), bytes);
    }
  }
}

#if !defined(_WIN32)
TEST(MaintenanceRuntimeTest, IoFailureLatchesFaultWithoutRepair) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  const auto bytes = inactive_json(3u);
  write_bytes(state_path, bytes);
  std::filesystem::permissions(
      state_path, std::filesystem::perms::none);
  MaintenanceRuntime runtime(state_path);

  const bool initialized = runtime.initialize(true);
  std::filesystem::permissions(
      state_path, std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write);
  if (initialized) {
    GTEST_SKIP() << "test process can bypass file permissions";
  }

  EXPECT_TRUE(runtime.storeFault());
  EXPECT_TRUE(runtime.admissionClosed());
  EXPECT_EQ(read_bytes(state_path), bytes);
}
#endif

TEST(MaintenanceRuntimeTest, SecondInitializeDoesNotRereadOrMutate) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(8u));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));
  const auto before = runtime.snapshot();
  write_bytes(state_path, active_json(9u, "other", "changed"));

  EXPECT_TRUE(runtime.initialize(false));
  const auto& after = runtime.snapshot();

  EXPECT_EQ(after.generation, before.generation);
  EXPECT_EQ(after.last_generation, before.last_generation);
  EXPECT_EQ(after.gate_active, before.gate_active);
  EXPECT_EQ(after.mission_idle, before.mission_idle);
  EXPECT_EQ(after.admission_closed, before.admission_closed);
}

TEST(MaintenanceRuntimeTest, MissionIdleUpdatesHealthyRestoredGateCoherently) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, active_json(51u, "operator", "service"));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(false));

  runtime.setMissionIdle(true);
  const auto& idle = runtime.snapshot();
  EXPECT_TRUE(idle.mission_idle);
  EXPECT_TRUE(idle.admission_closed);
  EXPECT_FALSE(idle.ready);
  EXPECT_EQ(idle.phase, "WAITING_FOR_COMMAND_GATE");
  EXPECT_EQ(idle.blocker_code, "COMMAND_GATE_NOT_APPLIED");

  runtime.setMissionIdle(false);
  const auto& busy = runtime.snapshot();
  EXPECT_FALSE(busy.mission_idle);
  EXPECT_TRUE(busy.admission_closed);
  EXPECT_FALSE(busy.ready);
  EXPECT_EQ(busy.phase, "MISSION_BUSY");
}

TEST(MaintenanceRuntimeTest, GateRestoreFailureLatchesStoreFault) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(5u));
  MaintenanceRuntime runtime(state_path);
  runtime.setMissionIdle(true);

  EXPECT_FALSE(runtime.initialize(true));
  EXPECT_TRUE(runtime.storeFault());
  EXPECT_TRUE(runtime.admissionClosed());
  EXPECT_EQ(runtime.snapshot().phase, "STORE_FAULT");
  EXPECT_EQ(
      runtime.snapshot().blocker_code,
      "MAINTENANCE_STORE_FAULT");
}

TEST(MaintenanceRuntimeTest, EnablePersistsNewGenerationAndClosesAdmission) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(17u));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));

  const auto result = runtime.enable("ota-updater", "install-v4", true);

  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.gate_active);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.generation, 18u);
  EXPECT_EQ(result.code, "OK");
  const auto& snapshot = runtime.snapshot();
  EXPECT_TRUE(snapshot.admission_closed);
  EXPECT_TRUE(snapshot.gate_active);
  EXPECT_EQ(snapshot.generation, 18u);
  EXPECT_EQ(snapshot.last_generation, 18u);
  EXPECT_EQ(snapshot.requester, "ota-updater");
  EXPECT_EQ(snapshot.reason, "install-v4");
  EXPECT_NE(
      read_bytes(state_path).find("\"lastGeneration\":18"),
      std::string::npos);
  EXPECT_NE(
      read_bytes(state_path).find("\"state\":\"active\""),
      std::string::npos);
}

TEST(MaintenanceRuntimeTest, SameOwnerEnableIsIdempotentAndKeepsOriginalReason) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, active_json(29u, "ota-updater", "original"));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));
  const auto before = read_bytes(state_path);

  const auto result =
      runtime.enable("ota-updater", "replacement", true);

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.generation, 29u);
  EXPECT_EQ(result.code, "ALREADY_ACTIVE");
  EXPECT_TRUE(runtime.admissionClosed());
  EXPECT_EQ(runtime.snapshot().reason, "original");
  EXPECT_EQ(read_bytes(state_path), before);
}

TEST(MaintenanceRuntimeTest, ExactReleasePersistsTombstoneBeforeOpeningAdmission) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, active_json(29u, "ota-updater", "install"));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));

  const auto wrong = runtime.disable(28u, "ota-updater");
  EXPECT_FALSE(wrong.accepted);
  EXPECT_TRUE(runtime.admissionClosed());
  EXPECT_TRUE(runtime.snapshot().gate_active);

  const auto released = runtime.disable(29u, "ota-updater");

  EXPECT_TRUE(released.accepted);
  EXPECT_FALSE(released.gate_active);
  EXPECT_EQ(released.generation, 29u);
  EXPECT_EQ(released.code, "OK");
  EXPECT_FALSE(runtime.admissionClosed());
  EXPECT_FALSE(runtime.snapshot().gate_active);
  EXPECT_EQ(runtime.snapshot().generation, 29u);
  EXPECT_EQ(runtime.snapshot().last_generation, 29u);
  EXPECT_TRUE(runtime.snapshot().requester.empty());
  EXPECT_TRUE(runtime.snapshot().reason.empty());
  EXPECT_NE(
      read_bytes(state_path).find("\"state\":\"inactive\""),
      std::string::npos);
}

TEST(MaintenanceRuntimeTest, GenerationExhaustionDoesNotCloseHealthyInactiveState) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  write_bytes(state_path, inactive_json(maximum));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));

  const auto result = runtime.enable("ota-updater", "install", true);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "GENERATION_EXHAUSTED");
  EXPECT_FALSE(runtime.storeFault());
  EXPECT_FALSE(runtime.admissionClosed());
  EXPECT_FALSE(runtime.snapshot().gate_active);
  EXPECT_EQ(runtime.snapshot().generation, maximum);
}

TEST(MaintenanceRuntimeTest, InvalidCallerInputDoesNotLatchStoreFault) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(17u));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));
  const std::string long_reason(
      MaintenanceStore::kMaximumReasonBytes + 1u, 'x');

  const auto empty_owner = runtime.enable("", "install", true);
  EXPECT_FALSE(empty_owner.accepted);
  EXPECT_EQ(empty_owner.code, "INVALID");
  EXPECT_FALSE(runtime.storeFault());
  EXPECT_FALSE(runtime.admissionClosed());

  const auto oversized =
      runtime.enable("updater", long_reason, true);
  EXPECT_FALSE(oversized.accepted);
  EXPECT_EQ(oversized.code, "INVALID");
  EXPECT_FALSE(runtime.storeFault());
  EXPECT_FALSE(runtime.admissionClosed());

  ASSERT_TRUE(runtime.enable("updater", "install", true).accepted);
  const auto empty_release = runtime.disable(18u, "");
  EXPECT_FALSE(empty_release.accepted);
  EXPECT_EQ(empty_release.code, "INVALID");
  EXPECT_FALSE(runtime.storeFault());
  EXPECT_TRUE(runtime.admissionClosed());
  EXPECT_TRUE(runtime.snapshot().gate_active);
}

FinalCommandEvidence maintenance_brake(
    const std::uint64_t generation,
    const std::uint64_t command_id) {
  FinalCommandEvidence command;
  command.generation = generation;
  command.request_id = generation;
  command.command_id = command_id;
  command.source = "maintenance_gate";
  command.active = true;
  command.brake = true;
  return command;
}

CommandStatusEvidence acknowledged_brake(
    const std::uint64_t generation,
    const std::uint64_t command_id) {
  CommandStatusEvidence status;
  status.generation = generation;
  status.request_id = generation;
  status.command_id = command_id;
  status.source = "maintenance_gate";
  status.state = 2u;
  return status;
}

cleanbot::common::PublisherIdentity publisher(const std::uint8_t tag) {
  cleanbot::common::PublisherIdentity identity;
  identity.implementation_identifier = "rmw_fastrtps_cpp";
  identity.gid = {tag, 0x55u};
  return identity;
}

MaintenanceHardwareSample stopped_hardware(
    const std::uint64_t frame_sequence) {
  MaintenanceHardwareSample sample;
  sample.frame_sequence = frame_sequence;
  sample.connected = true;
  return sample;
}

TEST(MaintenanceRuntimeTest, EvidenceRequiresTwoFramesAndStaleRefreshRevokesReady) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(17u));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));
  ASSERT_TRUE(runtime.enable("updater", "install", true).accepted);
  runtime.observeFinalCommand(maintenance_brake(18u, 900u));
  runtime.observeCommandStatus(acknowledged_brake(18u, 900u));
  ASSERT_TRUE(runtime.snapshot().brake_acknowledged);

  const auto first =
      runtime.observeHardware(publisher(1u), stopped_hardware(10u), 100u);
  EXPECT_EQ(first.status, MaintenanceHardwareStatus::kAccepted);
  EXPECT_EQ(first.publisher_epoch, 1u);
  EXPECT_TRUE(first.session_changed);
  EXPECT_FALSE(runtime.snapshot().ready);

  runtime.refreshHardware(150u, 100u);
  EXPECT_FALSE(runtime.snapshot().ready);
  const auto second =
      runtime.observeHardware(publisher(1u), stopped_hardware(11u), 160u);
  EXPECT_EQ(second.status, MaintenanceHardwareStatus::kAccepted);
  EXPECT_FALSE(second.session_changed);
  ASSERT_TRUE(runtime.snapshot().ready);

  runtime.refreshHardware(261u, 100u);
  EXPECT_FALSE(runtime.snapshot().ready);
  EXPECT_EQ(runtime.snapshot().blocker_code, "HARDWARE_NOT_FRESH");
}

TEST(
    MaintenanceRuntimeTest,
    PublisherSwitchNeedsNewFramesAndRetiredPublisherIsIgnored) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(3u));
  MaintenanceRuntime runtime(state_path);
  ASSERT_TRUE(runtime.initialize(true));
  ASSERT_TRUE(runtime.enable("updater", "install", true).accepted);
  runtime.observeFinalCommand(maintenance_brake(4u, 44u));
  runtime.observeCommandStatus(acknowledged_brake(4u, 44u));
  ASSERT_EQ(
      runtime.observeHardware(
          publisher(1u), stopped_hardware(1u), 10u).status,
      MaintenanceHardwareStatus::kAccepted);
  runtime.observeHardware(publisher(1u), stopped_hardware(2u), 20u);
  ASSERT_TRUE(runtime.snapshot().ready);

  const auto switched =
      runtime.observeHardware(publisher(2u), stopped_hardware(1u), 30u);
  EXPECT_EQ(switched.status, MaintenanceHardwareStatus::kAccepted);
  EXPECT_TRUE(switched.session_changed);
  EXPECT_FALSE(runtime.snapshot().ready);

  MaintenanceHardwareSample moving = stopped_hardware(999u);
  moving.x_speed = 50;
  const auto retired =
      runtime.observeHardware(publisher(1u), moving, 40u);
  EXPECT_EQ(retired.status, MaintenanceHardwareStatus::kRetired);
  EXPECT_FALSE(runtime.snapshot().ready);

  runtime.observeHardware(publisher(2u), stopped_hardware(2u), 50u);
  EXPECT_TRUE(runtime.snapshot().ready);
}

TEST(MaintenanceRuntimeTest, InvalidOrExhaustedPublisherRevokesReadiness) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json(7u));
  MaintenanceRuntime runtime(state_path, 4u, 1u);
  ASSERT_TRUE(runtime.initialize(true));
  ASSERT_TRUE(runtime.enable("updater", "install", true).accepted);
  runtime.observeFinalCommand(maintenance_brake(8u, 80u));
  runtime.observeCommandStatus(acknowledged_brake(8u, 80u));
  runtime.observeHardware(publisher(1u), stopped_hardware(1u), 10u);
  runtime.observeHardware(publisher(1u), stopped_hardware(2u), 20u);
  ASSERT_TRUE(runtime.snapshot().ready);

  auto invalid = publisher(0u);
  invalid.gid = {0u, 0u};
  const auto invalid_result =
      runtime.observeHardware(invalid, stopped_hardware(3u), 30u);
  EXPECT_EQ(invalid_result.status, MaintenanceHardwareStatus::kRevoked);
  EXPECT_FALSE(runtime.snapshot().ready);

  runtime.observeHardware(publisher(1u), stopped_hardware(3u), 40u);
  runtime.observeHardware(publisher(1u), stopped_hardware(4u), 50u);
  ASSERT_TRUE(runtime.snapshot().ready);
  const auto exhausted =
      runtime.observeHardware(publisher(2u), stopped_hardware(1u), 60u);
  EXPECT_EQ(exhausted.status, MaintenanceHardwareStatus::kRevoked);
  EXPECT_FALSE(runtime.snapshot().ready);
}

}  // namespace
}  // namespace mission
}  // namespace cleanbot
