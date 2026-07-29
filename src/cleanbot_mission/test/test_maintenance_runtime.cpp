#include "cleanbot_mission/maintenance_runtime.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <type_traits>
#include <utility>

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

}  // namespace
}  // namespace mission
}  // namespace cleanbot
