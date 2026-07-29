#include "cleanbot_mission/maintenance_store.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <nlohmann/json.hpp>

namespace cleanbot {
namespace mission {
namespace {

using Json = nlohmann::json;

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<std::uint64_t> sequence{0u};
    const auto nonce =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
        ("cleanbot-maintenance-store-" + std::to_string(nonce) + "-" +
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

std::string read_bytes(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(input),
      std::istreambuf_iterator<char>());
}

void write_bytes(
    const std::filesystem::path& path,
    const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(output.good());
}

std::string inactive_json(const std::string& generation) {
  return "{\"schemaVersion\":1,\"lastGeneration\":" + generation +
      ",\"state\":\"inactive\",\"inhibitor\":null}\n";
}

std::string active_json(
    const std::string& generation,
    const std::string& requester = "updater",
    const std::string& reason = "install") {
  return "{\"schemaVersion\":1,\"lastGeneration\":" + generation +
      ",\"state\":\"active\",\"inhibitor\":{\"generation\":" +
      generation + ",\"requester\":\"" + requester +
      "\",\"reason\":\"" + reason + "\"}}\n";
}

static_assert(noexcept(
    std::declval<const MaintenanceStore&>().load()));
static_assert(noexcept(
    std::declval<MaintenanceStore&>().initializeGenesis()));
static_assert(noexcept(
    std::declval<MaintenanceStore&>().activate(
        std::declval<const std::string&>(),
        std::declval<const std::string&>())));
static_assert(noexcept(
    std::declval<MaintenanceStore&>().release(
        std::uint64_t{1u},
        std::declval<const std::string&>())));
static_assert(
    std::is_nothrow_move_constructible<MaintenanceStoreResult>::value);
static_assert(
    std::is_nothrow_move_assignable<MaintenanceStoreResult>::value);

TEST(MaintenanceStoreTest, MissingIsDistinctAndNeverCreatesState) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);

  const auto result = store.load();

  EXPECT_EQ(result.code, MaintenanceStoreCode::kMissing);
  EXPECT_FALSE(result.record.has_value());
  EXPECT_FALSE(result.committed);
  EXPECT_FALSE(std::filesystem::exists(state_path));
}

TEST(MaintenanceStoreTest, GenesisRoundTripsAndIsIdempotent) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);

  const auto initialized = store.initializeGenesis();
  ASSERT_EQ(initialized.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(initialized.record.has_value());
  EXPECT_TRUE(initialized.committed);
  EXPECT_EQ(initialized.record->schema_version, 1u);
  EXPECT_EQ(initialized.record->last_generation, 0u);
  EXPECT_FALSE(initialized.record->inhibitor.has_value());

  const auto bytes = read_bytes(state_path);
  const auto parsed = Json::parse(bytes);
  EXPECT_EQ(
      parsed,
      Json({
          {"schemaVersion", 1u},
          {"lastGeneration", 0u},
          {"state", "inactive"},
          {"inhibitor", nullptr},
      }));

  MaintenanceStore restarted(state_path);
  const auto loaded = restarted.load();
  ASSERT_EQ(loaded.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(loaded.record.has_value());
  EXPECT_EQ(loaded.record->last_generation, 0u);
  EXPECT_FALSE(loaded.record->inhibitor.has_value());

  const auto second = restarted.initializeGenesis();
  EXPECT_EQ(second.code, MaintenanceStoreCode::kOk);
  EXPECT_FALSE(second.committed);
  EXPECT_EQ(read_bytes(state_path), bytes);
}

TEST(MaintenanceStoreTest, GenesisPreservesValidActiveRecord) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, active_json("7"));
  const auto before = read_bytes(state_path);
  MaintenanceStore store(state_path);

  const auto result = store.initializeGenesis();

  ASSERT_EQ(result.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(result.record.has_value());
  ASSERT_TRUE(result.record->inhibitor.has_value());
  EXPECT_EQ(result.record->last_generation, 7u);
  EXPECT_EQ(result.record->inhibitor->generation, 7u);
  EXPECT_EQ(result.record->inhibitor->requester, "updater");
  EXPECT_EQ(result.record->inhibitor->reason, "install");
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(read_bytes(state_path), before);
}

TEST(MaintenanceStoreTest, ActiveReleaseRestartAndNextGenerationRoundTrip) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);
  ASSERT_EQ(
      store.initializeGenesis().code,
      MaintenanceStoreCode::kOk);

  const auto activated = store.activate("updater", "install v1");
  ASSERT_EQ(activated.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(activated.record.has_value());
  ASSERT_TRUE(activated.record->inhibitor.has_value());
  EXPECT_TRUE(activated.committed);
  EXPECT_EQ(activated.record->last_generation, 1u);
  EXPECT_EQ(activated.record->inhibitor->generation, 1u);
  EXPECT_EQ(activated.record->inhibitor->requester, "updater");
  EXPECT_EQ(activated.record->inhibitor->reason, "install v1");

  MaintenanceStore restarted(state_path);
  const auto reloaded_active = restarted.load();
  ASSERT_EQ(reloaded_active.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(reloaded_active.record->inhibitor.has_value());
  EXPECT_EQ(reloaded_active.record->inhibitor->generation, 1u);

  const auto released = restarted.release(1u, "updater");
  ASSERT_EQ(released.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(released.record.has_value());
  EXPECT_TRUE(released.committed);
  EXPECT_EQ(released.record->last_generation, 1u);
  EXPECT_FALSE(released.record->inhibitor.has_value());

  MaintenanceStore restarted_again(state_path);
  const auto inactive = restarted_again.load();
  ASSERT_EQ(inactive.code, MaintenanceStoreCode::kOk);
  EXPECT_EQ(inactive.record->last_generation, 1u);
  EXPECT_FALSE(inactive.record->inhibitor.has_value());

  const auto next = restarted_again.activate("updater", "install v2");
  ASSERT_EQ(next.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(next.record->inhibitor.has_value());
  EXPECT_EQ(next.record->last_generation, 2u);
  EXPECT_EQ(next.record->inhibitor->generation, 2u);
}

TEST(MaintenanceStoreTest, SameRequesterIsIdempotentAndKeepsOriginalReason) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);
  ASSERT_EQ(
      store.initializeGenesis().code,
      MaintenanceStoreCode::kOk);
  ASSERT_EQ(
      store.activate("updater", "original").code,
      MaintenanceStoreCode::kOk);
  const auto before = read_bytes(state_path);

  const auto retried = store.activate("updater", "replacement");

  EXPECT_EQ(retried.code, MaintenanceStoreCode::kAlreadyActive);
  ASSERT_TRUE(retried.record.has_value());
  ASSERT_TRUE(retried.record->inhibitor.has_value());
  EXPECT_EQ(retried.record->inhibitor->generation, 1u);
  EXPECT_EQ(retried.record->inhibitor->reason, "original");
  EXPECT_FALSE(retried.committed);
  EXPECT_EQ(read_bytes(state_path), before);
}

TEST(MaintenanceStoreTest, DifferentRequesterAndTokenMismatchLeaveBytesUnchanged) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);
  ASSERT_EQ(
      store.initializeGenesis().code,
      MaintenanceStoreCode::kOk);
  ASSERT_EQ(
      store.activate("updater", "install").code,
      MaintenanceStoreCode::kOk);
  const auto active_bytes = read_bytes(state_path);

  const auto competing = store.activate("operator", "diagnostics");
  EXPECT_EQ(competing.code, MaintenanceStoreCode::kOwnedByOther);
  EXPECT_FALSE(competing.committed);
  EXPECT_EQ(read_bytes(state_path), active_bytes);

  const auto wrong_generation = store.release(2u, "updater");
  EXPECT_EQ(wrong_generation.code, MaintenanceStoreCode::kTokenMismatch);
  EXPECT_FALSE(wrong_generation.committed);
  EXPECT_EQ(read_bytes(state_path), active_bytes);

  const auto wrong_requester = store.release(1u, "operator");
  EXPECT_EQ(wrong_requester.code, MaintenanceStoreCode::kTokenMismatch);
  EXPECT_FALSE(wrong_requester.committed);
  EXPECT_EQ(read_bytes(state_path), active_bytes);
}

TEST(MaintenanceStoreTest, RejectsMalformedRecordsWithoutThrowing) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);
  const std::string long_requester(
      MaintenanceStore::kMaximumRequesterBytes + 1u, 'r');
  const std::string long_reason(
      MaintenanceStore::kMaximumReasonBytes + 1u, 'x');
  const std::vector<std::string> invalid_records{
      "",
      "{",
      "[]",
      "{\"schemaVersion\":1,\"lastGeneration\":0,"
          "\"state\":\"inactive\"}",
      "{\"schemaVersion\":1,\"lastGeneration\":0,"
          "\"state\":\"inactive\",\"inhibitor\":null,\"extra\":1}",
      "{\"schemaVersion\":1,\"lastGeneration\":-1,"
          "\"state\":\"inactive\",\"inhibitor\":null}",
      "{\"schemaVersion\":1,\"lastGeneration\":1.0,"
          "\"state\":\"inactive\",\"inhibitor\":null}",
      "{\"schemaVersion\":1,\"lastGeneration\":18446744073709551616,"
          "\"state\":\"inactive\",\"inhibitor\":null}",
      "{\"schemaVersion\":\"1\",\"lastGeneration\":0,"
          "\"state\":\"inactive\",\"inhibitor\":null}",
      "{\"schemaVersion\":1,\"lastGeneration\":0,"
          "\"state\":\"other\",\"inhibitor\":null}",
      "{\"schemaVersion\":1,\"lastGeneration\":1,"
          "\"state\":\"inactive\",\"inhibitor\":{}}",
      "{\"schemaVersion\":1,\"lastGeneration\":1,"
          "\"state\":\"active\",\"inhibitor\":null}",
      "{\"schemaVersion\":1,\"lastGeneration\":0,"
          "\"state\":\"active\",\"inhibitor\":{\"generation\":0,"
          "\"requester\":\"updater\",\"reason\":\"install\"}}",
      "{\"schemaVersion\":1,\"lastGeneration\":2,"
          "\"state\":\"active\",\"inhibitor\":{\"generation\":1,"
          "\"requester\":\"updater\",\"reason\":\"install\"}}",
      "{\"schemaVersion\":1,\"lastGeneration\":1,"
          "\"state\":\"active\",\"inhibitor\":{\"generation\":1,"
          "\"requester\":\"\",\"reason\":\"install\"}}",
      active_json("1", long_requester, "install"),
      active_json("1", "updater", long_reason),
      "{\"schemaVersion\":1,\"lastGeneration\":1,"
          "\"state\":\"active\",\"inhibitor\":{\"generation\":1,"
          "\"requester\":\"updater\",\"reason\":\"install\","
          "\"extra\":true}}",
      "{\"schemaVersion\":1,\"lastGeneration\":1,"
          "\"state\":\"active\",\"inhibitor\":{\"generation\":-1,"
          "\"requester\":\"updater\",\"reason\":\"install\"}}",
  };

  for (const auto& bytes : invalid_records) {
    SCOPED_TRACE(bytes.substr(0u, 200u));
    write_bytes(state_path, bytes);
    const auto result = store.load();
    EXPECT_EQ(result.code, MaintenanceStoreCode::kInvalid);
    EXPECT_FALSE(result.record.has_value());
    EXPECT_FALSE(result.committed);
  }
}

TEST(MaintenanceStoreTest, UnsupportedSchemaIsDistinct) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(
      state_path,
      "{\"schemaVersion\":2,\"lastGeneration\":0,"
      "\"state\":\"inactive\",\"inhibitor\":null}");
  MaintenanceStore store(state_path);

  const auto result = store.load();

  EXPECT_EQ(result.code, MaintenanceStoreCode::kUnsupportedSchema);
  EXPECT_FALSE(result.record.has_value());
  EXPECT_FALSE(result.committed);
}

TEST(MaintenanceStoreTest, RejectsOversizedFileAndInputs) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(
      state_path,
      std::string(MaintenanceStore::kMaximumFileBytes + 1u, 'x'));
  MaintenanceStore store(state_path);
  EXPECT_EQ(store.load().code, MaintenanceStoreCode::kInvalid);

  std::filesystem::remove(state_path);
  ASSERT_EQ(
      store.initializeGenesis().code,
      MaintenanceStoreCode::kOk);
  const auto before = read_bytes(state_path);
  EXPECT_EQ(
      store.activate("", "reason").code,
      MaintenanceStoreCode::kInvalid);
  EXPECT_EQ(
      store.activate(
          std::string(
              MaintenanceStore::kMaximumRequesterBytes + 1u, 'r'),
          "reason").code,
      MaintenanceStoreCode::kInvalid);
  EXPECT_EQ(
      store.activate(
          "updater",
          std::string(
              MaintenanceStore::kMaximumReasonBytes + 1u, 'x')).code,
      MaintenanceStoreCode::kInvalid);
  EXPECT_EQ(read_bytes(state_path), before);
}

TEST(MaintenanceStoreTest, MaximumGenerationIsLosslessAndNeverWraps) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  write_bytes(
      state_path,
      inactive_json(std::to_string(maximum - 1u)));
  MaintenanceStore store(state_path);

  const auto maximum_active = store.activate("updater", "last");
  ASSERT_EQ(maximum_active.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(maximum_active.record->inhibitor.has_value());
  EXPECT_EQ(maximum_active.record->last_generation, maximum);
  EXPECT_EQ(maximum_active.record->inhibitor->generation, maximum);

  MaintenanceStore restarted(state_path);
  const auto loaded = restarted.load();
  ASSERT_EQ(loaded.code, MaintenanceStoreCode::kOk);
  EXPECT_EQ(loaded.record->last_generation, maximum);
  ASSERT_TRUE(loaded.record->inhibitor.has_value());
  EXPECT_EQ(loaded.record->inhibitor->generation, maximum);

  ASSERT_EQ(
      restarted.release(maximum, "updater").code,
      MaintenanceStoreCode::kOk);
  const auto inactive_bytes = read_bytes(state_path);
  const auto exhausted = restarted.activate("updater", "overflow");
  EXPECT_EQ(
      exhausted.code,
      MaintenanceStoreCode::kGenerationExhausted);
  EXPECT_FALSE(exhausted.committed);
  EXPECT_EQ(read_bytes(state_path), inactive_bytes);
}

#if !defined(_WIN32)
TEST(MaintenanceStoreTest, RejectsSymlinkDirectoryAndNonRegularState) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  const auto target_path = temporary.path() / "target.json";
  write_bytes(target_path, inactive_json("0"));
  std::error_code error;
  std::filesystem::create_symlink(target_path, state_path, error);
  if (error) {
    GTEST_SKIP() << "symlink creation unavailable: " << error.message();
  }

  MaintenanceStore store(state_path);
  EXPECT_EQ(store.load().code, MaintenanceStoreCode::kInvalid);

  std::filesystem::remove(state_path);
  std::filesystem::create_directory(state_path);
  EXPECT_EQ(store.load().code, MaintenanceStoreCode::kInvalid);

  std::filesystem::remove(state_path);
  ASSERT_EQ(::mkfifo(state_path.c_str(), 0600), 0);
  EXPECT_EQ(store.load().code, MaintenanceStoreCode::kInvalid);
}

TEST(MaintenanceStoreTest, PermissionFailureIsIoError) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  write_bytes(state_path, inactive_json("0"));
  ASSERT_EQ(::chmod(state_path.c_str(), 0000), 0);
  MaintenanceStore store(state_path);

  const auto result = store.load();
  ASSERT_EQ(::chmod(state_path.c_str(), 0600), 0);
  if (result.code == MaintenanceStoreCode::kOk) {
    GTEST_SKIP() << "test process can bypass file permissions";
  }
  EXPECT_EQ(result.code, MaintenanceStoreCode::kIoError);
}

TEST(MaintenanceStoreTest, CreatedFilesArePrivateAndNoTempsRemain) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore store(state_path);
  ASSERT_EQ(
      store.initializeGenesis().code,
      MaintenanceStoreCode::kOk);
  ASSERT_EQ(
      store.activate("updater", "install").code,
      MaintenanceStoreCode::kOk);

  struct stat state_stat {};
  ASSERT_EQ(::stat(state_path.c_str(), &state_stat), 0);
  EXPECT_EQ(state_stat.st_mode & 0777, 0600);
  const auto guard_path =
      std::filesystem::path(state_path.string() + ".guard");
  struct stat guard_stat {};
  ASSERT_EQ(::stat(guard_path.c_str(), &guard_stat), 0);
  EXPECT_EQ(guard_stat.st_mode & 0777, 0600);

  const auto temporary_prefix =
      state_path.filename().string() + ".tmp.";
  for (const auto& entry :
      std::filesystem::directory_iterator(temporary.path())) {
    EXPECT_NE(
        entry.path().filename().string().rfind(temporary_prefix, 0u),
        0u);
  }
}
#endif

TEST(MaintenanceStoreTest, ConcurrentActivationsAllocateOnlyOneOwner) {
  TemporaryDirectory temporary;
  const auto state_path = temporary.path() / "maintenance.json";
  MaintenanceStore genesis(state_path);
  ASSERT_EQ(
      genesis.initializeGenesis().code,
      MaintenanceStoreCode::kOk);

  std::atomic<unsigned int> ready{0u};
  std::atomic<bool> start{false};
  MaintenanceStoreResult first;
  MaintenanceStoreResult second;
  auto activate = [&](const std::string& requester,
                      MaintenanceStoreResult* result) {
    MaintenanceStore store(state_path);
    ready.fetch_add(1u);
    while (!start.load()) {
      std::this_thread::yield();
    }
    *result = store.activate(requester, "concurrent");
  };

  std::thread first_thread(activate, "first", &first);
  std::thread second_thread(activate, "second", &second);
  while (ready.load() != 2u) {
    std::this_thread::yield();
  }
  start.store(true);
  first_thread.join();
  second_thread.join();

  const bool first_won =
      first.code == MaintenanceStoreCode::kOk &&
      second.code == MaintenanceStoreCode::kOwnedByOther;
  const bool second_won =
      second.code == MaintenanceStoreCode::kOk &&
      first.code == MaintenanceStoreCode::kOwnedByOther;
  EXPECT_TRUE(first_won || second_won);

  const auto loaded = genesis.load();
  ASSERT_EQ(loaded.code, MaintenanceStoreCode::kOk);
  ASSERT_TRUE(loaded.record->inhibitor.has_value());
  EXPECT_EQ(loaded.record->last_generation, 1u);
}

}  // namespace
}  // namespace mission
}  // namespace cleanbot
