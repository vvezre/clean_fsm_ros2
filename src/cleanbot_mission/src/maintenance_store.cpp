/*
 * 文件作用：使用 JSON 状态文件、跨平台文件锁和原子替换持久化维护模式。
 * 说明：同时防御符号链接、Windows 重解析点、路径替换和异常中断造成的状态损坏。
 */
#if defined(_WIN32) && !defined(NOMINMAX)
#define NOMINMAX
#endif

#include "cleanbot_mission/maintenance_store.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <nlohmann/json.hpp>

namespace cleanbot {
namespace mission {
namespace {

using Json = nlohmann::json;

constexpr std::uint32_t kSchemaVersion = 1u;

static_assert(
    std::is_nothrow_move_constructible<MaintenanceStoreResult>::value);
static_assert(
    std::is_nothrow_move_assignable<MaintenanceStoreResult>::value);
static_assert(
    std::is_nothrow_destructible<MaintenanceStoreResult>::value);
static_assert(noexcept(
    std::declval<std::string&>().swap(
        std::declval<std::string&>())));

// 统一构造维护存储操作结果，并携带可选记录和提交状态。
MaintenanceStoreResult make_result(
    const MaintenanceStoreCode code,
    std::string message,
    std::optional<MaintenanceStoreRecord> record = std::nullopt,
    const bool committed = false) {
  MaintenanceStoreResult result;
  result.code = code;
  result.record = std::move(record);
  result.message = std::move(message);
  result.committed = committed;
  return result;
}

// 检查 JSON 对象是否只包含指定字段，拒绝未知或缺失字段。
bool has_exact_fields(
    const Json& value,
    const std::initializer_list<const char*> fields) {
  if (!value.is_object() || value.size() != fields.size()) {
    return false;
  }
  for (const auto* field : fields) {
    if (!value.contains(field)) {
      return false;
    }
  }
  return true;
}

// 解析并严格校验维护状态 JSON、架构版本和代次不变式。
MaintenanceStoreResult parse_record(const std::string& bytes) {
  if (bytes.empty()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state file is empty");
  }

  const Json root = Json::parse(bytes, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state is not a JSON object");
  }

  if (!root.contains("schemaVersion") ||
      !root.at("schemaVersion").is_number_unsigned()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance schemaVersion must be an unsigned integer");
  }
  if (root.at("schemaVersion").get<std::uint64_t>() != kSchemaVersion) {
    return make_result(
        MaintenanceStoreCode::kUnsupportedSchema,
        "maintenance state schema is unsupported");
  }
  if (!has_exact_fields(
          root,
          {"schemaVersion", "lastGeneration", "state", "inhibitor"})) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state fields do not match the schema");
  }
  if (!root.at("lastGeneration").is_number_unsigned() ||
      !root.at("state").is_string()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state field types are invalid");
  }

  MaintenanceStoreRecord record;
  record.schema_version = kSchemaVersion;
  record.last_generation =
      root.at("lastGeneration").get<std::uint64_t>();
  const std::string state = root.at("state").get<std::string>();

  if (state == "inactive") {
    if (!root.at("inhibitor").is_null()) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "inactive maintenance state must have a null inhibitor");
    }
    return make_result(
        MaintenanceStoreCode::kOk,
        "maintenance state loaded",
        record);
  }

  if (state != "active") {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state is unknown");
  }

  const Json& inhibitor = root.at("inhibitor");
  if (!has_exact_fields(
          inhibitor, {"generation", "requester", "reason"}) ||
      !inhibitor.at("generation").is_number_unsigned() ||
      !inhibitor.at("requester").is_string() ||
      !inhibitor.at("reason").is_string()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "active maintenance inhibitor fields are invalid");
  }

  MaintenanceInhibitor active;
  active.generation = inhibitor.at("generation").get<std::uint64_t>();
  active.requester = inhibitor.at("requester").get<std::string>();
  active.reason = inhibitor.at("reason").get<std::string>();
  if (active.generation == 0u ||
      active.generation != record.last_generation) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "active maintenance generation invariant is invalid");
  }
  if (active.requester.empty() ||
      active.requester.size() >
          MaintenanceStore::kMaximumRequesterBytes) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance requester length is invalid");
  }
  if (active.reason.size() > MaintenanceStore::kMaximumReasonBytes) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance reason is too long");
  }
  record.inhibitor = std::move(active);
  return make_result(
      MaintenanceStoreCode::kOk,
      "maintenance state loaded",
      std::move(record));
}

// 将内存记录序列化为稳定、换行结尾的 JSON 文本。
std::string serialize_record(const MaintenanceStoreRecord& record) {
  Json root{
      {"schemaVersion", kSchemaVersion},
      {"lastGeneration", record.last_generation},
      {"state", record.inhibitor.has_value() ? "active" : "inactive"},
      {"inhibitor", nullptr},
  };
  if (record.inhibitor.has_value()) {
    root["inhibitor"] = Json{
        {"generation", record.inhibitor->generation},
        {"requester", record.inhibitor->requester},
        {"reason", record.inhibitor->reason},
    };
  }
  return root.dump() + "\n";
}

#if defined(_WIN32)

// 将 Windows 错误码附加到可读的操作说明中。
std::string windows_error(
    const std::string& prefix,
    const DWORD error = ::GetLastError()) {
  return prefix + " (Windows error " + std::to_string(error) + ")";
}

constexpr DWORD kDirectoryShareMode =
    FILE_SHARE_READ | FILE_SHARE_WRITE;

struct WindowsFileIdentity {
  DWORD volume_serial_number{0u};
  DWORD file_index_high{0u};
  DWORD file_index_low{0u};
};

struct DirectoryAnchor {
  HANDLE handle{INVALID_HANDLE_VALUE};
  WindowsFileIdentity identity;
};

// 比较两个 Windows 文件标识是否指向同一卷上的同一对象。
bool same_identity(
    const WindowsFileIdentity& lhs,
    const WindowsFileIdentity& rhs) noexcept {
  return lhs.volume_serial_number == rhs.volume_serial_number &&
      lhs.file_index_high == rhs.file_index_high &&
      lhs.file_index_low == rhs.file_index_low;
}

// 验证句柄指向普通目录而非重解析点，并提取稳定文件标识。
bool inspect_directory(
    const HANDLE handle,
    WindowsFileIdentity* identity) noexcept {
  BY_HANDLE_FILE_INFORMATION information {};
  if (!::GetFileInformationByHandle(handle, &information) ||
      (information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0u ||
      (information.dwFileAttributes &
          FILE_ATTRIBUTE_REPARSE_POINT) != 0u) {
    return false;
  }
  identity->volume_serial_number =
      information.dwVolumeSerialNumber;
  identity->file_index_high = information.nFileIndexHigh;
  identity->file_index_low = information.nFileIndexLow;
  return true;
}

// 判断字符是否为本地盘符允许的 ASCII 英文字母。
bool ascii_drive_letter(const wchar_t value) noexcept {
  return (value >= L'A' && value <= L'Z') ||
      (value >= L'a' && value <= L'z');
}

// 判断路径分量是否命中 Windows 保留设备名称。
bool reserved_windows_component(const std::wstring& component) {
  const auto separator = component.find(L'.');
  std::wstring stem = component.substr(0u, separator);
  std::transform(
      stem.begin(),
      stem.end(),
      stem.begin(),
      // 匿名函数作用：封装当前局部回调或判定逻辑，供调用方在本作用域内执行。
      [](const wchar_t value) {
        return value >= L'a' && value <= L'z'
               ? static_cast<wchar_t>(value - L'a' + L'A')
               : value;
      });
  if (stem == L"CON" || stem == L"PRN" ||
      stem == L"AUX" || stem == L"NUL" ||
      stem == L"CLOCK$") {
    return true;
  }
  if (stem.size() == 4u &&
      (stem.compare(0u, 3u, L"COM") == 0 ||
       stem.compare(0u, 3u, L"LPT") == 0) &&
      stem[3] >= L'1' && stem[3] <= L'9') {
    return true;
  }
  return false;
}

// 校验单个 Windows 路径分量不含保留名称、控制字符或非法字符。
bool safe_windows_component(
    const std::filesystem::path& component) {
  const std::wstring name = component.wstring();
  if (name.empty() || name == L"." || name == L".." ||
      name.back() == L'.' || name.back() == L' ' ||
      reserved_windows_component(name)) {
    return false;
  }
  for (const wchar_t value : name) {
    if (value < 32 || value == L':' || value == L'<' ||
        value == L'>' || value == L'"' || value == L'|' ||
        value == L'?' || value == L'*') {
      return false;
    }
  }
  return true;
}

// 仅接受带本地盘符的绝对路径，并逐级校验所有路径分量。
bool safe_local_windows_path(
    const std::filesystem::path& path) {
  if (!path.is_absolute() || !path.has_root_name() ||
      !path.has_root_directory()) {
    return false;
  }
  const std::wstring root_name = path.root_name().wstring();
  if (root_name.size() != 2u ||
      !ascii_drive_letter(root_name[0]) ||
      root_name[1] != L':') {
    return false;
  }
  for (const auto& component : path.relative_path()) {
    if (!safe_windows_component(component)) {
      return false;
    }
  }
  return !path.filename().empty();
}

class LockedStore {
 public:
  // 创建尚未绑定路径和文件锁的存储会话。
  LockedStore() = default;
  // 文件锁会话绑定操作系统句柄，禁止复制。
  LockedStore(const LockedStore&) = delete;
  // 禁止复制赋值，避免两个对象重复释放同一锁。
  LockedStore& operator=(const LockedStore&) = delete;
  // 禁止移动构造，保证已锚定句柄地址和生命周期稳定。
  LockedStore(LockedStore&&) = delete;
  // 禁止移动赋值，防止锁所有权发生隐式转移。
  LockedStore& operator=(LockedStore&&) = delete;

  // 析构时以不抛异常方式释放锁、文件句柄和目录锚点。
  ~LockedStore() {
    close_unchecked();
  }

  // 校验 Windows 路径、锚定父目录链并独占锁定守卫文件。
  MaintenanceStoreResult open(
      const std::filesystem::path& requested_path) {
    if (!safe_local_windows_path(requested_path)) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "unsupported Windows maintenance state path");
    }
    state_path_ = requested_path.lexically_normal();
    parent_path_ = state_path_.parent_path();

    auto anchored = anchor_directory_chain();
    if (anchored.code != MaintenanceStoreCode::kOk) {
      return anchored;
    }
    if (!validate_parent_anchor()) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "Windows maintenance parent anchor changed");
    }

    guard_path_ =
        std::filesystem::path(state_path_.wstring() + L".guard");
    guard_handle_ = ::CreateFileW(
        guard_path_.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
        nullptr);
    if (guard_handle_ == INVALID_HANDLE_VALUE) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error("failed to open maintenance guard"));
    }

    BY_HANDLE_FILE_INFORMATION guard_information {};
    if (!::GetFileInformationByHandle(
            guard_handle_, &guard_information) ||
        (guard_information.dwFileAttributes &
            (FILE_ATTRIBUTE_DIRECTORY |
             FILE_ATTRIBUTE_REPARSE_POINT)) != 0u) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance guard is not a regular file");
    }

    OVERLAPPED overlapped {};
    if (!::LockFileEx(
            guard_handle_,
            LOCKFILE_EXCLUSIVE_LOCK,
            0u,
            MAXDWORD,
            MAXDWORD,
            &overlapped)) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error("failed to lock maintenance guard"));
    }
    locked_ = true;
    return make_result(
        MaintenanceStoreCode::kOk,
        "maintenance guard locked");
  }

  // 显式解锁并关闭句柄；关闭失败会覆盖为 I/O 错误结果。
  void finish(
      MaintenanceStoreResult* result,
      std::string* prepared_error_message) noexcept {
    DWORD first_error = ERROR_SUCCESS;
    if (locked_) {
      OVERLAPPED overlapped {};
      if (!::UnlockFileEx(
              guard_handle_, 0u, MAXDWORD, MAXDWORD, &overlapped)) {
        first_error = ::GetLastError();
      }
      locked_ = false;
    }
    if (guard_handle_ != INVALID_HANDLE_VALUE) {
      if (!::CloseHandle(guard_handle_) &&
          first_error == ERROR_SUCCESS) {
        first_error = ::GetLastError();
      }
      guard_handle_ = INVALID_HANDLE_VALUE;
    }
    for (auto anchor = directory_anchors_.rbegin();
        anchor != directory_anchors_.rend();
        ++anchor) {
      if (anchor->handle != INVALID_HANDLE_VALUE) {
        if (!::CloseHandle(anchor->handle) &&
            first_error == ERROR_SUCCESS) {
          first_error = ::GetLastError();
        }
        anchor->handle = INVALID_HANDLE_VALUE;
      }
    }
    if (first_error != ERROR_SUCCESS) {
      result->code = MaintenanceStoreCode::kIoError;
      result->message.swap(*prepared_error_message);
    }
  }

  // 返回经过规范化并受目录锚点保护的状态文件路径。
  const std::filesystem::path& state_path() const {
    return state_path_;
  }

  // 重新打开父目录并比对文件标识，检测路径被替换的情况。
  bool validate_parent_anchor() const noexcept {
    if (directory_anchors_.empty()) {
      return false;
    }
    const HANDLE validation_handle = ::CreateFileW(
        parent_path_.c_str(),
        FILE_READ_ATTRIBUTES,
        kDirectoryShareMode,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
        nullptr);
    if (validation_handle == INVALID_HANDLE_VALUE) {
      return false;
    }
    WindowsFileIdentity identity;
    const bool inspected =
        inspect_directory(validation_handle, &identity);
    const bool closed = ::CloseHandle(validation_handle) != 0;
    return inspected && closed &&
        same_identity(
            identity,
            directory_anchors_.back().identity);
  }

 private:
  // 从盘符根目录开始逐级锚定状态文件父目录链。
  MaintenanceStoreResult anchor_directory_chain() {
    std::filesystem::path anchored_path = state_path_.root_path();
    auto anchored = anchor_directory(anchored_path);
    if (anchored.code != MaintenanceStoreCode::kOk) {
      return anchored;
    }
    for (const auto& component : parent_path_.relative_path()) {
      anchored_path /= component;
      anchored = anchor_directory(anchored_path);
      if (anchored.code != MaintenanceStoreCode::kOk) {
        return anchored;
      }
    }
    return make_result(
        MaintenanceStoreCode::kOk,
        "Windows maintenance directory chain anchored");
  }

  // 打开并保存一个非重解析点目录的句柄和稳定身份。
  MaintenanceStoreResult anchor_directory(
      const std::filesystem::path& directory_path) {
    const HANDLE handle = ::CreateFileW(
        directory_path.c_str(),
        FILE_READ_ATTRIBUTES,
        kDirectoryShareMode,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
        nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error(
              "failed to anchor Windows maintenance directory"));
    }
    DirectoryAnchor anchor;
    anchor.handle = handle;
    if (!inspect_directory(handle, &anchor.identity)) {
      ::CloseHandle(handle);
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "Windows maintenance directory is a reparse point");
    }
    try {
      directory_anchors_.push_back(anchor);
    } catch (...) {
      ::CloseHandle(handle);
      throw;
    }
    return make_result(
        MaintenanceStoreCode::kOk,
        "Windows maintenance directory anchored");
  }

  // 在析构路径中尽力释放全部 Windows 资源，不传播错误。
  void close_unchecked() noexcept {
    if (locked_) {
      OVERLAPPED overlapped {};
      ::UnlockFileEx(
          guard_handle_, 0u, MAXDWORD, MAXDWORD, &overlapped);
      locked_ = false;
    }
    if (guard_handle_ != INVALID_HANDLE_VALUE) {
      ::CloseHandle(guard_handle_);
      guard_handle_ = INVALID_HANDLE_VALUE;
    }
    for (auto anchor = directory_anchors_.rbegin();
        anchor != directory_anchors_.rend();
        ++anchor) {
      if (anchor->handle != INVALID_HANDLE_VALUE) {
        ::CloseHandle(anchor->handle);
        anchor->handle = INVALID_HANDLE_VALUE;
      }
    }
  }

  std::filesystem::path state_path_;
  std::filesystem::path parent_path_;
  std::filesystem::path guard_path_;
  std::vector<DirectoryAnchor> directory_anchors_;
  HANDLE guard_handle_{INVALID_HANDLE_VALUE};
  bool locked_{false};
};

// 在 Windows 守卫锁内读取有界普通文件并解析维护记录。
MaintenanceStoreResult read_locked(const LockedStore& store) {
  if (!store.validate_parent_anchor()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "Windows maintenance parent anchor changed");
  }
  HANDLE file = ::CreateFileW(
      store.state_path().c_str(),
      GENERIC_READ,
      FILE_SHARE_READ,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
      nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD error = ::GetLastError();
    if (error == ERROR_FILE_NOT_FOUND ||
        error == ERROR_PATH_NOT_FOUND) {
      return make_result(
          MaintenanceStoreCode::kMissing,
          "maintenance state file is missing");
    }
    return make_result(
        MaintenanceStoreCode::kIoError,
        windows_error("failed to open maintenance state", error));
  }

  BY_HANDLE_FILE_INFORMATION information {};
  if (!::GetFileInformationByHandle(file, &information) ||
      (information.dwFileAttributes &
          (FILE_ATTRIBUTE_DIRECTORY |
           FILE_ATTRIBUTE_REPARSE_POINT)) != 0u ||
      information.nFileSizeHigh != 0u ||
      information.nFileSizeLow >
          MaintenanceStore::kMaximumFileBytes) {
    ::CloseHandle(file);
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state is not a bounded regular file");
  }

  std::string bytes;
  bytes.reserve(information.nFileSizeLow);
  char buffer[4096];
  for (;;) {
    DWORD count = 0u;
    if (!::ReadFile(
            file, buffer, static_cast<DWORD>(sizeof(buffer)),
            &count, nullptr)) {
      const DWORD error = ::GetLastError();
      ::CloseHandle(file);
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error("failed to read maintenance state", error));
    }
    if (count == 0u) {
      break;
    }
    if (bytes.size() + count >
        MaintenanceStore::kMaximumFileBytes) {
      ::CloseHandle(file);
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance state file is too large");
    }
    bytes.append(buffer, count);
  }
  if (!::CloseHandle(file)) {
    return make_result(
        MaintenanceStoreCode::kIoError,
        windows_error("failed to close maintenance state"));
  }
  return parse_record(bytes);
}

// 在 Windows 守卫锁内写临时文件、同步并原子替换正式状态文件。
MaintenanceStoreResult write_locked(
    const LockedStore& store,
    const MaintenanceStoreRecord& record) {
  static std::atomic<std::uint64_t> sequence{0u};
  const std::string bytes = serialize_record(record);
  if (bytes.size() > MaintenanceStore::kMaximumFileBytes) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "serialized maintenance state is too large");
  }
  auto committed_success = make_result(
      MaintenanceStoreCode::kOk,
      "maintenance state committed",
      record,
      true);
  if (!store.validate_parent_anchor()) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "Windows maintenance parent anchor changed");
  }

  std::filesystem::path temporary_path;
  HANDLE temporary = INVALID_HANDLE_VALUE;
  for (unsigned int attempt = 0u; attempt < 64u; ++attempt) {
    temporary_path = std::filesystem::path(
        store.state_path().wstring() + L".tmp." +
        std::to_wstring(::GetCurrentProcessId()) + L"." +
        std::to_wstring(sequence.fetch_add(1u)));
    temporary = ::CreateFileW(
        temporary_path.c_str(),
        GENERIC_WRITE,
        0u,
        nullptr,
        CREATE_NEW,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
        nullptr);
    if (temporary != INVALID_HANDLE_VALUE) {
      break;
    }
    if (::GetLastError() != ERROR_FILE_EXISTS) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error("failed to create maintenance temp file"));
    }
  }
  if (temporary == INVALID_HANDLE_VALUE) {
    return make_result(
        MaintenanceStoreCode::kIoError,
        "failed to allocate a unique maintenance temp file");
  }

  std::size_t offset = 0u;
  while (offset < bytes.size()) {
    DWORD written = 0u;
    const DWORD remaining = static_cast<DWORD>(
        std::min<std::size_t>(
            bytes.size() - offset,
            (std::numeric_limits<DWORD>::max)()));
    if (!::WriteFile(
            temporary,
            bytes.data() + offset,
            remaining,
            &written,
            nullptr) ||
        written == 0u) {
      const DWORD error = ::GetLastError();
      ::CloseHandle(temporary);
      ::DeleteFileW(temporary_path.c_str());
      return make_result(
          MaintenanceStoreCode::kIoError,
          windows_error("failed to write maintenance temp file", error));
    }
    offset += written;
  }
  if (!::FlushFileBuffers(temporary)) {
    const DWORD error = ::GetLastError();
    ::CloseHandle(temporary);
    ::DeleteFileW(temporary_path.c_str());
    return make_result(
        MaintenanceStoreCode::kIoError,
        windows_error("failed to sync maintenance temp file", error));
  }
  if (!::CloseHandle(temporary)) {
    const DWORD error = ::GetLastError();
    ::DeleteFileW(temporary_path.c_str());
    return make_result(
        MaintenanceStoreCode::kIoError,
        windows_error("failed to close maintenance temp file", error));
  }
  if (!store.validate_parent_anchor()) {
    ::DeleteFileW(temporary_path.c_str());
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "Windows maintenance parent anchor changed");
  }
  if (!::MoveFileExW(
          temporary_path.c_str(),
          store.state_path().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    const DWORD error = ::GetLastError();
    ::DeleteFileW(temporary_path.c_str());
    return make_result(
        MaintenanceStoreCode::kIoError,
        windows_error("failed to replace maintenance state", error));
  }
  // Windows maintenance state commit point. 中文说明：原子替换成功后不得再报告“未提交”。
  return std::move(committed_success);
}

#else

// 将当前或指定 errno 转换为带操作前缀的错误说明。
std::string posix_error(
    const std::string& prefix,
    const int error = errno) {
  return prefix + ": " + std::strerror(error);
}

// 关闭 POSIX 文件描述符，并把关闭失败的 errno 返回给调用方。
bool close_descriptor(const int descriptor, int* error) noexcept {
  if (::close(descriptor) == 0) {
    return true;
  }
  *error = errno;
  return false;
}

class LockedStore {
 public:
  // 创建尚未打开目录和守卫文件的存储会话。
  LockedStore() = default;
  // 文件描述符和 flock 锁只能由单个对象持有，禁止复制。
  LockedStore(const LockedStore&) = delete;
  // 禁止复制赋值，避免重复关闭同一描述符。
  LockedStore& operator=(const LockedStore&) = delete;
  // 禁止移动构造，保持文件锁所有者生命周期固定。
  LockedStore(LockedStore&&) = delete;
  // 禁止移动赋值，防止锁所有权被隐式覆盖。
  LockedStore& operator=(LockedStore&&) = delete;

  // 析构时尽力解锁并关闭守卫文件和父目录描述符。
  ~LockedStore() {
    close_unchecked();
  }

  // 使用 O_NOFOLLOW 打开可信父目录和守卫文件，并获取独占 flock 锁。
  MaintenanceStoreResult open(
      const std::filesystem::path& state_path) {
    const auto filename_path = state_path.filename();
    if (filename_path.empty() ||
        filename_path == "." ||
        filename_path == "..") {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance state path has no safe filename");
    }
    filename_ = filename_path.string();
    guard_name_ = filename_ + ".guard";
    auto parent = state_path.parent_path();
    if (parent.empty()) {
      parent = ".";
    }

    directory_fd_ = ::open(
        parent.c_str(),
        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (directory_fd_ < 0) {
      const int error = errno;
      const auto code =
          error == ELOOP || error == ENOTDIR
          ? MaintenanceStoreCode::kInvalid
          : MaintenanceStoreCode::kIoError;
      return make_result(
          code,
          posix_error(
              "failed to open trusted maintenance parent", error));
    }

    guard_fd_ = ::openat(
        directory_fd_,
        guard_name_.c_str(),
        O_RDWR | O_CREAT | O_NOFOLLOW | O_CLOEXEC,
        0600);
    if (guard_fd_ < 0) {
      const int error = errno;
      const auto code =
          error == ELOOP
          ? MaintenanceStoreCode::kInvalid
          : MaintenanceStoreCode::kIoError;
      return make_result(
          code,
          posix_error("failed to open maintenance guard", error));
    }

    struct stat guard_stat {};
    if (::fstat(guard_fd_, &guard_stat) != 0) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          posix_error("failed to inspect maintenance guard"));
    }
    if (!S_ISREG(guard_stat.st_mode)) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance guard is not a regular file");
    }
    if (::fchmod(guard_fd_, 0600) != 0) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          posix_error("failed to secure maintenance guard"));
    }

    while (::flock(guard_fd_, LOCK_EX) != 0) {
      if (errno == EINTR) {
        continue;
      }
      return make_result(
          MaintenanceStoreCode::kIoError,
          posix_error("failed to lock maintenance guard"));
    }
    locked_ = true;
    return make_result(
        MaintenanceStoreCode::kOk,
        "maintenance guard locked");
  }

  // 显式释放 flock 和描述符，并把首个关闭错误写回结果。
  void finish(
      MaintenanceStoreResult* result,
      std::string* prepared_error_message) noexcept {
    int first_error = 0;
    if (locked_) {
      while (::flock(guard_fd_, LOCK_UN) != 0) {
        if (errno == EINTR) {
          continue;
        }
        first_error = errno;
        break;
      }
      locked_ = false;
    }
    if (guard_fd_ >= 0) {
      int error = 0;
      if (!close_descriptor(guard_fd_, &error) &&
          first_error == 0) {
        first_error = error;
      }
      guard_fd_ = -1;
    }
    if (directory_fd_ >= 0) {
      int error = 0;
      if (!close_descriptor(directory_fd_, &error) &&
          first_error == 0) {
        first_error = error;
      }
      directory_fd_ = -1;
    }
    if (first_error != 0) {
      result->code = MaintenanceStoreCode::kIoError;
      result->message.swap(*prepared_error_message);
    }
  }

  // 返回已验证父目录的文件描述符，供 openat/renameat 使用。
  int directory_fd() const {
    return directory_fd_;
  }

  // 返回相对于可信父目录的状态文件名。
  const std::string& filename() const {
    return filename_;
  }

 private:
  // 在析构路径中忽略错误地释放锁和全部文件描述符。
  void close_unchecked() noexcept {
    if (locked_) {
      while (::flock(guard_fd_, LOCK_UN) != 0 &&
          errno == EINTR) {
      }
      locked_ = false;
    }
    if (guard_fd_ >= 0) {
      ::close(guard_fd_);
      guard_fd_ = -1;
    }
    if (directory_fd_ >= 0) {
      ::close(directory_fd_);
      directory_fd_ = -1;
    }
  }

  int directory_fd_{-1};
  int guard_fd_{-1};
  bool locked_{false};
  std::string filename_;
  std::string guard_name_;
};

// 在 POSIX 守卫锁内拒绝符号链接，读取有界普通文件并解析记录。
MaintenanceStoreResult read_locked(const LockedStore& store) {
  struct stat path_stat {};
  if (::fstatat(
          store.directory_fd(),
          store.filename().c_str(),
          &path_stat,
          AT_SYMLINK_NOFOLLOW) != 0) {
    const int error = errno;
    if (error == ENOENT) {
      return make_result(
          MaintenanceStoreCode::kMissing,
          "maintenance state file is missing");
    }
    return make_result(
        MaintenanceStoreCode::kIoError,
        posix_error("failed to inspect maintenance state", error));
  }
  if (!S_ISREG(path_stat.st_mode)) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state is not a regular file");
  }
  if (path_stat.st_size < 0 ||
      static_cast<std::uintmax_t>(path_stat.st_size) >
          MaintenanceStore::kMaximumFileBytes) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state file is too large");
  }

  const int state_fd = ::openat(
      store.directory_fd(),
      store.filename().c_str(),
      O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC);
  if (state_fd < 0) {
    const int error = errno;
    if (error == ENOENT) {
      return make_result(
          MaintenanceStoreCode::kMissing,
          "maintenance state file is missing");
    }
    const auto code =
        error == ELOOP
        ? MaintenanceStoreCode::kInvalid
        : MaintenanceStoreCode::kIoError;
    return make_result(
        code,
        posix_error("failed to open maintenance state", error));
  }

  struct stat opened_stat {};
  if (::fstat(state_fd, &opened_stat) != 0) {
    const int error = errno;
    int ignored = 0;
    close_descriptor(state_fd, &ignored);
    return make_result(
        MaintenanceStoreCode::kIoError,
        posix_error("failed to inspect open maintenance state", error));
  }
  if (!S_ISREG(opened_stat.st_mode) ||
      opened_stat.st_size < 0 ||
      static_cast<std::uintmax_t>(opened_stat.st_size) >
          MaintenanceStore::kMaximumFileBytes) {
    int ignored = 0;
    close_descriptor(state_fd, &ignored);
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "maintenance state is not a bounded regular file");
  }

  std::string bytes;
  bytes.reserve(static_cast<std::size_t>(opened_stat.st_size));
  char buffer[4096];
  for (;;) {
    const ssize_t count =
        ::read(state_fd, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int error = errno;
      int ignored = 0;
      close_descriptor(state_fd, &ignored);
      return make_result(
          MaintenanceStoreCode::kIoError,
          posix_error("failed to read maintenance state", error));
    }
    if (count == 0) {
      break;
    }
    if (bytes.size() + static_cast<std::size_t>(count) >
        MaintenanceStore::kMaximumFileBytes) {
      int ignored = 0;
      close_descriptor(state_fd, &ignored);
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance state file grew beyond its size bound");
    }
    bytes.append(buffer, static_cast<std::size_t>(count));
  }

  int close_error = 0;
  if (!close_descriptor(state_fd, &close_error)) {
    return make_result(
        MaintenanceStoreCode::kIoError,
        posix_error(
            "failed to close maintenance state", close_error));
  }
  return parse_record(bytes);
}

// 写入唯一临时文件并依次同步文件、原子重命名和同步父目录。
MaintenanceStoreResult write_locked(
    const LockedStore& store,
    const MaintenanceStoreRecord& record) {
  static std::atomic<std::uint64_t> sequence{0u};
  const std::string bytes = serialize_record(record);
  if (bytes.size() > MaintenanceStore::kMaximumFileBytes) {
    return make_result(
        MaintenanceStoreCode::kInvalid,
        "serialized maintenance state is too large");
  }
  auto committed_success = make_result(
      MaintenanceStoreCode::kOk,
      "maintenance state committed",
      record,
      true);
  auto post_commit_failure = make_result(
      MaintenanceStoreCode::kIoError,
      "maintenance state was renamed but directory sync failed",
      record,
      true);

  std::string temporary_name;
  int temporary_fd = -1;
  for (unsigned int attempt = 0u; attempt < 64u; ++attempt) {
    temporary_name =
        store.filename() + ".tmp." +
        std::to_string(static_cast<unsigned long long>(::getpid())) +
        "." + std::to_string(sequence.fetch_add(1u));
    temporary_fd = ::openat(
        store.directory_fd(),
        temporary_name.c_str(),
        O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
        0600);
    if (temporary_fd >= 0) {
      break;
    }
    if (errno != EEXIST) {
      return make_result(
          MaintenanceStoreCode::kIoError,
          posix_error("failed to create maintenance temp file"));
    }
  }
  if (temporary_fd < 0) {
    return make_result(
        MaintenanceStoreCode::kIoError,
        "failed to allocate a unique maintenance temp file");
  }

  // 清理回调作用：删除尚未提交的临时状态文件并返回清理结果。
  const auto cleanup_temporary = [&]() {
    if (::unlinkat(
            store.directory_fd(),
            temporary_name.c_str(),
            0) != 0 &&
        errno != ENOENT) {
      return errno;
    }
    return 0;
  };
  const auto fail_before_commit =
      // 清理回调作用：删除尚未提交的临时状态文件并返回清理结果。
      [&](const std::string& message) {
        const int cleanup_error = cleanup_temporary();
        if (cleanup_error != 0) {
          return make_result(
              MaintenanceStoreCode::kIoError,
              message + "; " +
              posix_error(
                  "failed to remove maintenance temp file",
                  cleanup_error));
        }
        return make_result(
            MaintenanceStoreCode::kIoError,
            message);
      };

  if (::fchmod(temporary_fd, 0600) != 0) {
    const int error = errno;
    int ignored = 0;
    close_descriptor(temporary_fd, &ignored);
    temporary_fd = -1;
    return fail_before_commit(
        posix_error("failed to secure maintenance temp file", error));
  }

  std::size_t offset = 0u;
  while (offset < bytes.size()) {
    const ssize_t count = ::write(
        temporary_fd,
        bytes.data() + offset,
        bytes.size() - offset);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int error = errno;
      int ignored = 0;
      close_descriptor(temporary_fd, &ignored);
      temporary_fd = -1;
      return fail_before_commit(
          posix_error("failed to write maintenance temp file", error));
    }
    if (count == 0) {
      int ignored = 0;
      close_descriptor(temporary_fd, &ignored);
      temporary_fd = -1;
      return fail_before_commit(
          "maintenance temp file write made no progress");
    }
    offset += static_cast<std::size_t>(count);
  }

  while (::fsync(temporary_fd) != 0) {
    if (errno == EINTR) {
      continue;
    }
    const int error = errno;
    int ignored = 0;
    close_descriptor(temporary_fd, &ignored);
    temporary_fd = -1;
    return fail_before_commit(
        posix_error("failed to sync maintenance temp file", error));
  }

  int close_error = 0;
  if (!close_descriptor(temporary_fd, &close_error)) {
    temporary_fd = -1;
    return fail_before_commit(
        posix_error(
            "failed to close maintenance temp file", close_error));
  }
  temporary_fd = -1;

  if (::renameat(
          store.directory_fd(),
          temporary_name.c_str(),
          store.directory_fd(),
          store.filename().c_str()) != 0) {
    const int error = errno;
    return fail_before_commit(
        posix_error("failed to replace maintenance state", error));
  }

  // POSIX maintenance state commit point. 中文说明：renameat 成功后记录已对其他进程可见。
  while (::fsync(store.directory_fd()) != 0) {
    if (errno == EINTR) {
      continue;
    }
    return std::move(post_commit_failure);
  }
  return std::move(committed_success);
}

#endif

template<typename Operation>
// 在持有跨进程守卫锁期间执行读写操作，并统一完成资源收尾。
MaintenanceStoreResult with_locked_store(
    const std::filesystem::path& state_path,
    Operation&& operation) {
  LockedStore store;
  auto opened = store.open(state_path);
  if (opened.code != MaintenanceStoreCode::kOk) {
    return opened;
  }
  std::string finish_error_message =
      "failed to close maintenance store handles";
  auto result = operation(store);
  store.finish(&result, &finish_error_message);
  return std::move(result);
}

MaintenanceStoreResult exception_result(
    const std::exception& exception) noexcept {
  // 将标准异常安全地转换为不抛异常的 I/O 失败结果。
  try {
    return make_result(
        MaintenanceStoreCode::kIoError,
        std::string("maintenance store exception: ") +
        exception.what());
  } catch (...) {
    MaintenanceStoreResult result;
    result.code = MaintenanceStoreCode::kIoError;
    return result;
  }
}

// 将未知异常转换为保守的 I/O 失败结果。
MaintenanceStoreResult unknown_exception_result() noexcept {
  try {
    return make_result(
        MaintenanceStoreCode::kIoError,
        "maintenance store raised an unknown exception");
  } catch (...) {
    MaintenanceStoreResult result;
    result.code = MaintenanceStoreCode::kIoError;
    return result;
  }
}

}  // namespace

// 保存维护状态文件路径，不在构造阶段执行任何文件系统操作。
MaintenanceStore::MaintenanceStore(std::filesystem::path state_path)
    : state_path_(std::move(state_path)) {}

// 加锁读取并校验当前维护状态，对所有异常返回稳定错误结果。
MaintenanceStoreResult MaintenanceStore::load() const noexcept {
  try {
    return with_locked_store(
        state_path_,
        [](const LockedStore& store) {
          return read_locked(store);
        });
  } catch (const std::exception& exception) {
    return exception_result(exception);
  } catch (...) {
    return unknown_exception_result();
  }
}

// 状态文件缺失时原子创建初始非活动记录，已有合法记录则保持不变。
MaintenanceStoreResult MaintenanceStore::initializeGenesis() noexcept {
  try {
    return with_locked_store(
        state_path_,
        [](const LockedStore& store) {
          auto existing = read_locked(store);
          if (existing.code == MaintenanceStoreCode::kOk) {
            existing.message =
                "valid maintenance state already exists";
            return existing;
          }
          if (existing.code != MaintenanceStoreCode::kMissing) {
            return existing;
          }

          MaintenanceStoreRecord genesis;
          return write_locked(store, genesis);
        });
  } catch (const std::exception& exception) {
    return exception_result(exception);
  } catch (...) {
    return unknown_exception_result();
  }
}

// 校验申请者和理由，递增代次并原子写入活动维护抑制器。
MaintenanceStoreResult MaintenanceStore::activate(
    const std::string& requester,
    const std::string& reason) noexcept {
  try {
    if (requester.empty() ||
        requester.size() > kMaximumRequesterBytes) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance requester length is invalid");
    }
    if (reason.size() > kMaximumReasonBytes) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance reason is too long");
    }

    return with_locked_store(
        state_path_,
        // 存储操作回调作用：在持有跨进程文件锁期间执行本次状态读写事务。
        [&](const LockedStore& store) {
          auto current = read_locked(store);
          if (current.code != MaintenanceStoreCode::kOk) {
            return current;
          }
          if (current.record->inhibitor.has_value()) {
            if (current.record->inhibitor->requester == requester) {
              current.code = MaintenanceStoreCode::kAlreadyActive;
              current.message =
                  "maintenance inhibitor is already active for requester";
              return current;
            }
            current.code = MaintenanceStoreCode::kOwnedByOther;
            current.message =
                "maintenance inhibitor is owned by another requester";
            return current;
          }
          if (current.record->last_generation ==
              std::numeric_limits<std::uint64_t>::max()) {
            current.code =
                MaintenanceStoreCode::kGenerationExhausted;
            current.message =
                "maintenance generation space is exhausted";
            return current;
          }

          MaintenanceStoreRecord next = *current.record;
          ++next.last_generation;
          next.inhibitor = MaintenanceInhibitor{
              next.last_generation,
              requester,
              reason,
          };
          return write_locked(store, next);
        });
  } catch (const std::exception& exception) {
    return exception_result(exception);
  } catch (...) {
    return unknown_exception_result();
  }
}

// 仅在代次和申请者均匹配时清除维护抑制器并持久化。
MaintenanceStoreResult MaintenanceStore::release(
    const std::uint64_t generation,
    const std::string& requester) noexcept {
  try {
    if (requester.empty() ||
        requester.size() > kMaximumRequesterBytes) {
      return make_result(
          MaintenanceStoreCode::kInvalid,
          "maintenance requester length is invalid");
    }

    return with_locked_store(
        state_path_,
        // 存储操作回调作用：在持有跨进程文件锁期间执行本次状态读写事务。
        [&](const LockedStore& store) {
          auto current = read_locked(store);
          if (current.code != MaintenanceStoreCode::kOk) {
            return current;
          }
          if (!current.record->inhibitor.has_value() ||
              current.record->inhibitor->generation != generation ||
              current.record->inhibitor->requester != requester) {
            current.code = MaintenanceStoreCode::kTokenMismatch;
            current.message =
                "maintenance release token does not match";
            return current;
          }

          MaintenanceStoreRecord next = *current.record;
          next.inhibitor.reset();
          return write_locked(store, next);
        });
  } catch (const std::exception& exception) {
    return exception_result(exception);
  } catch (...) {
    return unknown_exception_result();
  }
}

}  // namespace mission
}  // namespace cleanbot
