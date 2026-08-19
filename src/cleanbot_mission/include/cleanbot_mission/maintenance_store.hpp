#ifndef CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

// 文件作用：声明维护模式持久化记录的读取、初始化、占用和释放接口。
namespace cleanbot {
namespace mission {

// 维护模式占用者及其代次、申请理由。
struct MaintenanceInhibitor {
  std::uint64_t generation{0u};
  std::string requester;
  std::string reason;
};

// 写入状态文件的维护模式完整记录。
struct MaintenanceStoreRecord {
  std::uint32_t schema_version{1u};
  std::uint64_t last_generation{0u};
  std::optional<MaintenanceInhibitor> inhibitor;
};

// 持久化存储操作的结果类型。
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

// 存储操作的结果、最新记录、说明和提交状态。
struct MaintenanceStoreResult {
  MaintenanceStoreCode code{MaintenanceStoreCode::kIoError};
  std::optional<MaintenanceStoreRecord> record;
  std::string message;
  bool committed{false};
};

class MaintenanceStore {
 public:
  // 限制状态文件及外部输入字段的最大字节数。
  static constexpr std::size_t kMaximumFileBytes = 64u * 1024u;
  static constexpr std::size_t kMaximumRequesterBytes = 128u;
  static constexpr std::size_t kMaximumReasonBytes = 4096u;

  // 使用指定状态文件路径创建存储对象。
  explicit MaintenanceStore(std::filesystem::path state_path);

  // 读取并校验当前持久化记录。
  MaintenanceStoreResult load() const noexcept;
  // 当状态文件不存在时创建初始空记录。
  MaintenanceStoreResult initializeGenesis() noexcept;
  // 以申请者身份原子地占用维护模式。
  MaintenanceStoreResult activate(
      const std::string& requester,
      const std::string& reason) noexcept;
  // 使用匹配代次和申请者身份释放维护模式。
  MaintenanceStoreResult release(
      std::uint64_t generation,
      const std::string& requester) noexcept;

 private:
  std::filesystem::path state_path_;
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_STORE_HPP_
