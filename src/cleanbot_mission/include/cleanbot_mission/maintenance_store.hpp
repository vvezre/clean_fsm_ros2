#ifndef CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace cleanbot {
namespace mission {

struct MaintenanceInhibitor {
  std::uint64_t generation{0u};
  std::string requester;
  std::string reason;
};

struct MaintenanceStoreRecord {
  std::uint32_t schema_version{1u};
  std::uint64_t last_generation{0u};
  std::optional<MaintenanceInhibitor> inhibitor;
};

enum class MaintenanceStoreCode {
  kOk,
  kMissing,
  kInvalid,
  kUnsupportedSchema,
  kIoError,
  kAlreadyActive,
  kOwnedByOther,
  kTokenMismatch,
  kGenerationExhausted,
};

struct MaintenanceStoreResult {
  MaintenanceStoreCode code{MaintenanceStoreCode::kIoError};
  std::optional<MaintenanceStoreRecord> record;
  std::string message;
  bool committed{false};
};

class MaintenanceStore {
 public:
  static constexpr std::size_t kMaximumFileBytes = 64u * 1024u;
  static constexpr std::size_t kMaximumRequesterBytes = 128u;
  static constexpr std::size_t kMaximumReasonBytes = 4096u;

  explicit MaintenanceStore(std::filesystem::path state_path);

  MaintenanceStoreResult load() const noexcept;
  MaintenanceStoreResult initializeGenesis() noexcept;
  MaintenanceStoreResult activate(
      const std::string& requester,
      const std::string& reason) noexcept;
  MaintenanceStoreResult release(
      std::uint64_t generation,
      const std::string& requester) noexcept;

 private:
  std::filesystem::path state_path_;
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_
