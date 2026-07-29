#include "cleanbot_config/sqlite_config_repository.hpp"

#include <cstdlib>
#include <utility>

#ifdef _WIN32
#include <sys/stat.h>
#include <windows.h>
#else
#include <sys/stat.h>
#include <sys/statvfs.h>
#endif

#include <sqlite3.h>

namespace cleanbot {
namespace config {
namespace {

constexpr std::uint32_t kSchemaVersion = 1u;

std::string value_type_name(const ConfigValueType type) {
  switch (type) {
    case ConfigValueType::kInteger:
      return "integer";
    case ConfigValueType::kDouble:
      return "double";
    case ConfigValueType::kBoolean:
      return "boolean";
    case ConfigValueType::kString:
    default:
      return "string";
  }
}

std::string apply_policy_name(const ApplyPolicy policy) {
  return policy == ApplyPolicy::kImmediate ? "immediate" : "restart";
}

std::string sqlite_error(sqlite3* database) {
  return database == nullptr ? "database is not open" : sqlite3_errmsg(database);
}

RepositoryStatus repository_error(
    const std::string& code,
    const std::string& message,
    const std::uint32_t schema_version = 0u) {
  RepositoryStatus status;
  status.code = code;
  status.message = message;
  status.schema_version = schema_version;
  return status;
}

std::string parent_directory(const std::string& path) {
  const auto separator = path.find_last_of("/\\");
  if (separator == std::string::npos) {
    return ".";
  }
  if (separator == 0u) {
    return path.substr(0u, 1u);
  }
  return path.substr(0u, separator);
}

#ifdef _WIN32
std::wstring utf8_to_wide(const std::string& value) {
  if (value.empty()) {
    return std::wstring();
  }
  const int length = MultiByteToWideChar(
      CP_UTF8, MB_ERR_INVALID_CHARS, value.c_str(), -1, nullptr, 0);
  if (length <= 0) {
    return std::wstring();
  }
  std::wstring converted(static_cast<std::size_t>(length), L'\0');
  MultiByteToWideChar(
      CP_UTF8, MB_ERR_INVALID_CHARS, value.c_str(), -1, &converted[0], length);
  if (!converted.empty() && converted.back() == L'\0') {
    converted.pop_back();
  }
  return converted;
}
#endif

bool directory_exists(const std::string& path) {
#ifdef _WIN32
  const auto wide_path = utf8_to_wide(path);
  if (wide_path.empty()) {
    return false;
  }
  struct _stat64 status;
  return _wstat64(wide_path.c_str(), &status) == 0 &&
      (status.st_mode & _S_IFDIR) != 0;
#else
  struct stat status;
  return stat(path.c_str(), &status) == 0 && S_ISDIR(status.st_mode);
#endif
}

bool available_space(
    const std::string& path,
    std::uint64_t& bytes,
    std::string& error) {
#ifdef _WIN32
  const auto wide_path = utf8_to_wide(path);
  if (wide_path.empty()) {
    error = "database path is not valid UTF-8";
    return false;
  }
  ULARGE_INTEGER available;
  if (!GetDiskFreeSpaceExW(wide_path.c_str(), &available, nullptr, nullptr)) {
    error = "GetDiskFreeSpaceEx failed";
    return false;
  }
  bytes = static_cast<std::uint64_t>(available.QuadPart);
  return true;
#else
  struct statvfs status;
  if (statvfs(path.c_str(), &status) != 0) {
    error = "statvfs failed";
    return false;
  }
  bytes = static_cast<std::uint64_t>(status.f_bavail) *
      static_cast<std::uint64_t>(status.f_frsize);
  return true;
#endif
}

}  // namespace

SqliteConfigRepository::SqliteConfigRepository(
    std::string database_path,
    const std::uint64_t minimum_free_space_bytes)
    : database_path_(std::move(database_path)),
      minimum_free_space_bytes_(minimum_free_space_bytes) {}

SqliteConfigRepository::~SqliteConfigRepository() {
  close();
}

RepositoryStatus SqliteConfigRepository::open_and_initialize(
    const ConfigRegistry& registry) {
  close();
  const auto parent = parent_directory(database_path_);
  if (!directory_exists(parent)) {
    return repository_error(
        "CONFIG_DIRECTORY_MISSING",
        "configuration database directory does not exist: " + parent);
  }
  std::uint64_t available_bytes = 0u;
  std::string space_error;
  if (!available_space(parent, available_bytes, space_error)) {
    return repository_error(
        "CONFIG_SPACE_CHECK_FAILED", space_error);
  }
  if (available_bytes < minimum_free_space_bytes_) {
    return repository_error(
        "CONFIG_DISK_SPACE_LOW", "configuration database disk space is too low");
  }

  const int open_result = sqlite3_open_v2(
      database_path_.c_str(),
      &database_,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
      nullptr);
  if (open_result != SQLITE_OK) {
    const auto message = sqlite_error(database_);
    close();
    return repository_error("CONFIG_DATABASE_OPEN_FAILED", message);
  }
  sqlite3_busy_timeout(database_, 3000);
  if (sqlite3_db_readonly(database_, "main") == 1) {
    close();
    return repository_error(
        "CONFIG_DATABASE_READ_ONLY", "configuration database is read-only");
  }

  sqlite3_stmt* version_statement = nullptr;
  std::uint32_t schema_version = 0u;
  if (sqlite3_prepare_v2(
          database_, "PRAGMA user_version", -1, &version_statement, nullptr) != SQLITE_OK ||
      sqlite3_step(version_statement) != SQLITE_ROW) {
    const auto message = sqlite_error(database_);
    sqlite3_finalize(version_statement);
    close();
    return repository_error("CONFIG_SCHEMA_READ_FAILED", message);
  }
  schema_version = static_cast<std::uint32_t>(sqlite3_column_int(version_statement, 0));
  sqlite3_finalize(version_statement);

  std::string error;
  if (schema_version == 0u) {
    if (!create_schema(registry, error)) {
      close();
      return repository_error("CONFIG_SCHEMA_CREATE_FAILED", error);
    }
    schema_version = kSchemaVersion;
  } else if (schema_version != kSchemaVersion) {
    close();
    return repository_error(
        "CONFIG_SCHEMA_VERSION_UNSUPPORTED",
        "unsupported configuration schema version: " + std::to_string(schema_version),
        schema_version);
  }

  if (!quick_check(error)) {
    close();
    return repository_error("CONFIG_DATABASE_CORRUPT", error, schema_version);
  }
  if (!read_revision(revision_, error)) {
    close();
    return repository_error("CONFIG_REVISION_READ_FAILED", error, schema_version);
  }

  RepositoryStatus status;
  status.healthy = true;
  status.writable = true;
  status.code = "OK";
  status.message = "configuration database ready";
  status.schema_version = schema_version;
  status.revision = revision_;
  return status;
}

std::map<std::string, std::string> SqliteConfigRepository::read_all() const {
  std::map<std::string, std::string> values;
  if (database_ == nullptr) {
    return values;
  }
  sqlite3_stmt* statement = nullptr;
  if (sqlite3_prepare_v2(
          database_,
          "SELECT config_key, value_text FROM system_config ORDER BY config_key",
          -1, &statement, nullptr) != SQLITE_OK) {
    return values;
  }
  while (sqlite3_step(statement) == SQLITE_ROW) {
    const auto* key = reinterpret_cast<const char*>(sqlite3_column_text(statement, 0));
    const auto* value = reinterpret_cast<const char*>(sqlite3_column_text(statement, 1));
    if (key != nullptr) {
      values[key] = value == nullptr ? "" : value;
    }
  }
  sqlite3_finalize(statement);
  return values;
}

WriteResult SqliteConfigRepository::write_values(
    const std::map<std::string, std::string>& values,
    const std::string& updated_by,
    const ConfigRegistry& registry) {
  WriteResult result;
  result.revision = revision_;
  if (database_ == nullptr) {
    result.code = "CONFIG_DATABASE_NOT_OPEN";
    result.message = "configuration database is not open";
    return result;
  }
  for (const auto& item : values) {
    const auto validation = registry.validate_value(item.first, item.second);
    if (!validation.valid) {
      result.code = validation.code;
      result.message = validation.message;
      return result;
    }
  }

  const auto existing = read_all();
  for (const auto& item : values) {
    const auto found = existing.find(item.first);
    if (found == existing.end() || found->second != item.second) {
      result.changed_keys.push_back(item.first);
      const auto* definition = registry.find(item.first);
      if (definition != nullptr && definition->apply_policy == ApplyPolicy::kRestart) {
        result.restart_required = true;
      }
    }
  }
  if (result.changed_keys.empty()) {
    result.success = true;
    result.code = "OK";
    result.message = "configuration unchanged";
    return result;
  }

  std::string error;
  if (!execute("BEGIN IMMEDIATE", error)) {
    result.code = "CONFIG_TRANSACTION_BEGIN_FAILED";
    result.message = error;
    return result;
  }

  sqlite3_stmt* statement = nullptr;
  const char* sql =
      "INSERT INTO system_config "
      "(config_key, value_type, value_text, revision, apply_policy, is_secret, updated_at, updated_by) "
      "VALUES (?, ?, ?, ?, ?, ?, datetime('now'), ?) "
      "ON CONFLICT(config_key) DO UPDATE SET "
      "value_type=excluded.value_type, value_text=excluded.value_text, "
      "revision=excluded.revision, apply_policy=excluded.apply_policy, "
      "is_secret=excluded.is_secret, updated_at=excluded.updated_at, updated_by=excluded.updated_by";
  if (sqlite3_prepare_v2(database_, sql, -1, &statement, nullptr) != SQLITE_OK) {
    execute("ROLLBACK", error);
    result.code = "CONFIG_WRITE_PREPARE_FAILED";
    result.message = sqlite_error(database_);
    return result;
  }

  const auto next_revision = revision_ + 1u;
  bool write_ok = true;
  for (const auto& key : result.changed_keys) {
    const auto* definition = registry.find(key);
    const auto value = values.find(key);
    sqlite3_reset(statement);
    sqlite3_clear_bindings(statement);
    sqlite3_bind_text(statement, 1, key.c_str(), -1, SQLITE_TRANSIENT);
    const auto type = value_type_name(definition->value_type);
    sqlite3_bind_text(statement, 2, type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(statement, 3, value->second.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(statement, 4, static_cast<sqlite3_int64>(next_revision));
    const auto policy = apply_policy_name(definition->apply_policy);
    sqlite3_bind_text(statement, 5, policy.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(statement, 6, definition->sensitive ? 1 : 0);
    sqlite3_bind_text(statement, 7, updated_by.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(statement) != SQLITE_DONE) {
      write_ok = false;
      error = sqlite_error(database_);
      break;
    }
  }
  sqlite3_finalize(statement);

  if (write_ok) {
    sqlite3_stmt* metadata_statement = nullptr;
    if (sqlite3_prepare_v2(
            database_,
            "UPDATE config_metadata SET meta_value=? WHERE meta_key='revision'",
            -1, &metadata_statement, nullptr) != SQLITE_OK) {
      write_ok = false;
      error = sqlite_error(database_);
    } else {
      const auto revision_text = std::to_string(next_revision);
      sqlite3_bind_text(
          metadata_statement, 1, revision_text.c_str(), -1, SQLITE_TRANSIENT);
      if (sqlite3_step(metadata_statement) != SQLITE_DONE) {
        write_ok = false;
        error = sqlite_error(database_);
      }
    }
    sqlite3_finalize(metadata_statement);
  }

  if (!write_ok || !execute("COMMIT", error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    result.code = "CONFIG_WRITE_FAILED";
    result.message = error;
    return result;
  }

  revision_ = next_revision;
  result.success = true;
  result.code = "OK";
  result.message = "configuration updated";
  result.revision = revision_;
  return result;
}

void SqliteConfigRepository::close() {
  if (database_ != nullptr) {
    sqlite3_close(database_);
    database_ = nullptr;
  }
  revision_ = 0u;
}

bool SqliteConfigRepository::execute(
    const std::string& sql,
    std::string& error) const {
  char* sqlite_message = nullptr;
  const auto result = sqlite3_exec(
      database_, sql.c_str(), nullptr, nullptr, &sqlite_message);
  if (result == SQLITE_OK) {
    return true;
  }
  error = sqlite_message == nullptr ? sqlite_error(database_) : sqlite_message;
  sqlite3_free(sqlite_message);
  return false;
}

bool SqliteConfigRepository::create_schema(
    const ConfigRegistry& registry,
    std::string& error) {
  const char* schema =
      "BEGIN IMMEDIATE;"
      "CREATE TABLE config_metadata ("
      "meta_key TEXT PRIMARY KEY, meta_value TEXT NOT NULL);"
      "CREATE TABLE system_config ("
      "config_key TEXT PRIMARY KEY, value_type TEXT NOT NULL, value_text TEXT NOT NULL, "
      "revision INTEGER NOT NULL, apply_policy TEXT NOT NULL, is_secret INTEGER NOT NULL, "
      "updated_at TEXT NOT NULL, updated_by TEXT NOT NULL);"
      "INSERT INTO config_metadata(meta_key, meta_value) VALUES('revision', '0');"
      "PRAGMA user_version=1;";
  if (!execute(schema, error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }
  for (const auto& item : registry.definitions()) {
    if (item.second.has_default && !insert_default(item.second, error)) {
      std::string rollback_error;
      execute("ROLLBACK", rollback_error);
      return false;
    }
  }
  if (!execute("COMMIT", error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }
  return true;
}

bool SqliteConfigRepository::insert_default(
    const ConfigDefinition& definition,
    std::string& error) {
  sqlite3_stmt* statement = nullptr;
  const char* sql =
      "INSERT INTO system_config "
      "(config_key, value_type, value_text, revision, apply_policy, is_secret, updated_at, updated_by) "
      "VALUES (?, ?, ?, 0, ?, ?, datetime('now'), 'factory-default')";
  if (sqlite3_prepare_v2(database_, sql, -1, &statement, nullptr) != SQLITE_OK) {
    error = sqlite_error(database_);
    return false;
  }
  const auto type = value_type_name(definition.value_type);
  const auto policy = apply_policy_name(definition.apply_policy);
  sqlite3_bind_text(statement, 1, definition.key.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(statement, 2, type.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(statement, 3, definition.default_value.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(statement, 4, policy.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(statement, 5, definition.sensitive ? 1 : 0);
  const bool success = sqlite3_step(statement) == SQLITE_DONE;
  if (!success) {
    error = sqlite_error(database_);
  }
  sqlite3_finalize(statement);
  return success;
}

bool SqliteConfigRepository::read_revision(
    std::uint64_t& revision,
    std::string& error) const {
  sqlite3_stmt* statement = nullptr;
  if (sqlite3_prepare_v2(
          database_,
          "SELECT meta_value FROM config_metadata WHERE meta_key='revision'",
          -1, &statement, nullptr) != SQLITE_OK) {
    error = sqlite_error(database_);
    return false;
  }
  const auto step = sqlite3_step(statement);
  if (step != SQLITE_ROW) {
    error = "configuration revision metadata is missing";
    sqlite3_finalize(statement);
    return false;
  }
  const auto* value = reinterpret_cast<const char*>(sqlite3_column_text(statement, 0));
  if (value == nullptr) {
    error = "configuration revision metadata is invalid";
    sqlite3_finalize(statement);
    return false;
  }
  char* end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0') {
    error = "configuration revision metadata is invalid";
    sqlite3_finalize(statement);
    return false;
  }
  revision = static_cast<std::uint64_t>(parsed);
  sqlite3_finalize(statement);
  return true;
}

bool SqliteConfigRepository::quick_check(std::string& error) const {
  sqlite3_stmt* statement = nullptr;
  if (sqlite3_prepare_v2(
          database_, "PRAGMA quick_check", -1, &statement, nullptr) != SQLITE_OK) {
    error = sqlite_error(database_);
    return false;
  }
  const auto step = sqlite3_step(statement);
  const auto* value = step == SQLITE_ROW
      ? reinterpret_cast<const char*>(sqlite3_column_text(statement, 0))
      : nullptr;
  const bool healthy = value != nullptr && std::string(value) == "ok";
  if (!healthy) {
    error = value == nullptr ? sqlite_error(database_) : value;
  }
  sqlite3_finalize(statement);
  return healthy;
}

}  // namespace config
}  // namespace cleanbot
