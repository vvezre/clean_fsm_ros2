/*
 * 文件作用：模型仓库实现：持久化区域、采样点和任务规划数据。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/sqlite_model_repository.hpp"

#include <sqlite3.h>

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_modeling/geometry.hpp"

namespace cleanbot {
namespace modeling {
namespace sqlite_model_repository_detail {

constexpr std::uint32_t kModelingSchemaVersion = 3u;

struct MigratedPointRecord {
  sqlite3_int64 rowid{0};
  double x_cm{0.0};
  double y_cm{0.0};
};

// 返回 SQLite 最近错误，连接为空时给出明确说明。
std::string modeling_sqlite_error(sqlite3* database) {
  return database == nullptr ? "database is not open" : sqlite3_errmsg(database);
}

// 以 SQLITE_TRANSIENT 生命周期将字符串绑定到预编译语句参数。
void bind_text(
    sqlite3_stmt* statement,
    const int index,
    const std::string& value) {
  sqlite3_bind_text(
      statement, index, value.c_str(), -1, SQLITE_TRANSIENT);
}

// 安全读取结果列文本，SQL NULL 转换为空字符串。
std::string column_text(sqlite3_stmt* statement, const int index) {
  const auto* value =
      reinterpret_cast<const char*>(sqlite3_column_text(statement, index));
  return value == nullptr ? "" : value;
}

// 查询表结构信息，判断迁移目标列是否已经存在。
bool table_has_column(
    sqlite3* database,
    const std::string& table,
    const std::string& column,
    std::string& error) {
  sqlite3_stmt* statement = nullptr;
  const std::string sql = "PRAGMA table_info(" + table + ")";
  if (sqlite3_prepare_v2(database, sql.c_str(), -1, &statement, nullptr) != SQLITE_OK) {
    error = modeling_sqlite_error(database);
    return false;
  }
  bool found = false;
  while (sqlite3_step(statement) == SQLITE_ROW) {
    if (column_text(statement, 1) == column) {
      found = true;
      break;
    }
  }
  sqlite3_finalize(statement);
  return found;
}

// 将模型点角色列表编码为数据库文本字段。
std::string join_roles(const std::vector<std::string>& roles) {
  std::ostringstream stream;
  for (std::size_t index = 0u; index < roles.size(); ++index) {
    if (index > 0u) {
      stream << ',';
    }
    stream << roles[index];
  }
  return stream.str();
}

// 将数据库角色文本恢复为有序角色列表。
std::vector<std::string> split_roles(const std::string& value) {
  std::vector<std::string> roles;
  std::string role;
  for (const char character : value) {
    if (character == ',') {
      if (!role.empty()) {
        roles.push_back(role);
        role.clear();
      }
    } else {
      role.push_back(character);
    }
  }
  if (!role.empty()) {
    roles.push_back(role);
  }
  return roles;
}

// 构造仓库初始化或健康检查失败状态。
ModelRepositoryStatus repository_failure(
    const std::string& code,
    const std::string& message) {
  ModelRepositoryStatus status;
  status.code = code;
  status.message = message;
  return status;
}

// 构造模型或计划写入失败结果。
ModelWriteResult write_failure(
    const std::string& code,
    const std::string& message) {
  ModelWriteResult result;
  result.code = code;
  result.message = message;
  return result;
}

// 预编译 SQL 语句，并统一返回 SQLite 错误信息。
bool prepare(
    sqlite3* database,
    const char* sql,
    sqlite3_stmt** statement,
    std::string& error) {
  if (sqlite3_prepare_v2(database, sql, -1, statement, nullptr) == SQLITE_OK) {
    return true;
  }
  error = modeling_sqlite_error(database);
  return false;
}

// 执行预编译语句并确认其以 SQLITE_DONE 正常完成。
bool step_done(
    sqlite3* database,
    sqlite3_stmt* statement,
    std::string& error) {
  if (sqlite3_step(statement) == SQLITE_DONE) {
    return true;
  }
  error = modeling_sqlite_error(database);
  return false;
}

}  // namespace sqlite_model_repository_detail

using namespace sqlite_model_repository_detail;

// 保存模型数据库路径，延迟到初始化时打开连接。
SqliteModelRepository::SqliteModelRepository(std::string database_path)
    : database_path_(std::move(database_path)) {}

// 析构时关闭仍打开的 SQLite 连接。
SqliteModelRepository::~SqliteModelRepository() {
  close();
}

// 打开数据库，读取版本并创建或迁移到当前架构。
ModelRepositoryStatus SqliteModelRepository::open_and_initialize() {
  close();
  if (database_path_.empty()) {
    return repository_failure(
        "MODELING_DATABASE_PATH_EMPTY",
        "modeling database path is empty");
  }
  if (sqlite3_open_v2(
          database_path_.c_str(),
          &database_,
          SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
          nullptr) != SQLITE_OK) {
    const auto message = modeling_sqlite_error(database_);
    close();
    return repository_failure("MODELING_DATABASE_OPEN_FAILED", message);
  }
  sqlite3_busy_timeout(database_, 3000);

  std::string error;
  if (!execute("PRAGMA foreign_keys=ON", error) ||
      !execute("PRAGMA journal_mode=WAL", error) ||
      !create_schema(error)) {
    close();
    return repository_failure("MODELING_SCHEMA_CREATE_FAILED", error);
  }

  sqlite3_stmt* statement = nullptr;
  if (!prepare(
          database_,
          "SELECT meta_value FROM modeling_metadata "
          "WHERE meta_key='schema_version'",
          &statement,
          error)) {
    close();
    return repository_failure("MODELING_SCHEMA_READ_FAILED", error);
  }
  const bool has_version = sqlite3_step(statement) == SQLITE_ROW;
  const auto version_text =
      has_version ? column_text(statement, 0) : std::string();
  auto version = has_version
      ? static_cast<std::uint32_t>(
            std::strtoul(version_text.c_str(), nullptr, 10))
      : 0u;
  sqlite3_finalize(statement);
  if (version < kModelingSchemaVersion) {
    if (!migrate_schema(version, error)) {
      close();
      return repository_failure("MODELING_SCHEMA_MIGRATION_FAILED", error);
    }
    version = kModelingSchemaVersion;
  }
  if (version != kModelingSchemaVersion) {
    close();
    return repository_failure(
        "MODELING_SCHEMA_VERSION_UNSUPPORTED",
        "unsupported modeling schema version");
  }

  ModelRepositoryStatus status;
  status.healthy = true;
  status.writable = true;
  status.code = "OK";
  status.message = "modeling database ready";
  status.schema_version = version;
  return status;
}

// 在事务中创建模型、区域、点、连接、计划和计划段表。
bool SqliteModelRepository::create_schema(std::string& error) {
  const char* schema =
      "BEGIN IMMEDIATE;"
      "CREATE TABLE IF NOT EXISTS modeling_metadata("
      "meta_key TEXT PRIMARY KEY, meta_value TEXT NOT NULL);"
      "INSERT OR IGNORE INTO modeling_metadata(meta_key,meta_value)"
      "VALUES('schema_version','3');"
      "CREATE TABLE IF NOT EXISTS cleaning_models("
      "model_id TEXT PRIMARY KEY,name TEXT NOT NULL,status TEXT NOT NULL,"
      "current_version INTEGER NOT NULL DEFAULT 0,"
      "origin_lat REAL NOT NULL DEFAULT 0,origin_lon REAL NOT NULL DEFAULT 0,"
      "origin_valid INTEGER NOT NULL DEFAULT 0,"
      "created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,"
      "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);"
      "CREATE TABLE IF NOT EXISTS model_drafts("
      "model_id TEXT PRIMARY KEY,recognition_confirmed INTEGER NOT NULL,"
      "preview_confirmed INTEGER NOT NULL,"
      "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);"
      "CREATE TABLE IF NOT EXISTS model_versions("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,name TEXT NOT NULL,"
      "status TEXT NOT NULL,recognition_confirmed INTEGER NOT NULL,"
      "preview_confirmed INTEGER NOT NULL,"
      "created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,"
      "PRIMARY KEY(model_id,version));"
      "CREATE TABLE IF NOT EXISTS model_groups("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,"
      "group_order INTEGER NOT NULL,name TEXT NOT NULL,area_number INTEGER NOT NULL,"
      "sweep_mode TEXT NOT NULL,sweep_angle_deg REAL NOT NULL,"
      "recognition_status TEXT NOT NULL DEFAULT 'unrecognized',"
      "recognition_message TEXT NOT NULL DEFAULT '',"
      "recognition_confidence REAL NOT NULL DEFAULT 0,"
      "PRIMARY KEY(model_id,version,group_id));"
      "CREATE TABLE IF NOT EXISTS model_points("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,"
      "point_id TEXT NOT NULL,sequence INTEGER NOT NULL,"
      "lat REAL NOT NULL,lon REAL NOT NULL,x_cm REAL NOT NULL,y_cm REAL NOT NULL,"
      "heading_deg REAL NOT NULL,heading_valid INTEGER NOT NULL,"
      "capture_type TEXT NOT NULL DEFAULT 'boundary',"
      "role TEXT NOT NULL,roles TEXT NOT NULL,sample_count INTEGER NOT NULL,"
      "sample_radius_m REAL NOT NULL,fix_quality INTEGER NOT NULL,"
      "gga_age_sec REAL NOT NULL,source TEXT NOT NULL,"
      "PRIMARY KEY(model_id,version,group_id,point_id));"
      "CREATE TABLE IF NOT EXISTS model_sub_areas("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,"
      "sub_area_id TEXT NOT NULL,area_order INTEGER NOT NULL,name TEXT NOT NULL,"
      "confirmed INTEGER NOT NULL,"
      "PRIMARY KEY(model_id,version,group_id,sub_area_id));"
      "CREATE TABLE IF NOT EXISTS model_sub_area_points("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,"
      "sub_area_id TEXT NOT NULL,point_order INTEGER NOT NULL,point_id TEXT NOT NULL,"
      "PRIMARY KEY(model_id,version,group_id,sub_area_id,point_order));"
      "CREATE TABLE IF NOT EXISTS model_connectors("
      "model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,"
      "connector_id TEXT NOT NULL,connector_order INTEGER NOT NULL,type TEXT NOT NULL,"
      "start_point_id TEXT NOT NULL,end_point_id TEXT NOT NULL,"
      "from_sub_area_id TEXT NOT NULL DEFAULT '',"
      "to_sub_area_id TEXT NOT NULL DEFAULT '',"
      "length_cm REAL NOT NULL,confirmed INTEGER NOT NULL,"
      "PRIMARY KEY(model_id,version,group_id,connector_id));"
      "CREATE TABLE IF NOT EXISTS cleaning_plans("
      "plan_id TEXT PRIMARY KEY,model_id TEXT NOT NULL,model_version INTEGER NOT NULL,"
      "plan_hash TEXT NOT NULL,generated_at INTEGER NOT NULL,"
      "brush_width_cm REAL NOT NULL,minimum_overlap_cm REAL NOT NULL,"
      "actual_overlap_cm REAL NOT NULL,cleaning_lane_count INTEGER NOT NULL,"
      "transfer_segment_count INTEGER NOT NULL,total_length_cm REAL NOT NULL,"
      "preview_confirmed INTEGER NOT NULL,"
      "created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);"
      "CREATE TABLE IF NOT EXISTS plan_segments("
      "plan_id TEXT NOT NULL,segment_index INTEGER NOT NULL,segment_id TEXT NOT NULL,"
      "segment_type INTEGER NOT NULL,group_id TEXT NOT NULL,sub_area_id TEXT NOT NULL,"
      "source_lane_id TEXT NOT NULL,start_x_cm REAL NOT NULL,start_y_cm REAL NOT NULL,"
      "end_x_cm REAL NOT NULL,end_y_cm REAL NOT NULL,start_lat REAL NOT NULL,"
      "start_lon REAL NOT NULL,end_lat REAL NOT NULL,end_lon REAL NOT NULL,"
      "heading_deg REAL NOT NULL,turn_angle_deg REAL NOT NULL,speed INTEGER NOT NULL,"
      "mode INTEGER NOT NULL,brush_enabled INTEGER NOT NULL,"
      "PRIMARY KEY(plan_id,segment_index));"
      "COMMIT;";
  if (execute(schema, error)) {
    return true;
  }
  std::string rollback_error;
  execute("ROLLBACK", rollback_error);
  return false;
}

// 按当前版本选择并执行连续数据库迁移步骤。
bool SqliteModelRepository::migrate_schema(
    const std::uint32_t from_version,
    std::string& error) {
  if (from_version == 1u) {
    if (!migrate_v1_to_v2(error)) {
      return false;
    }
    return migrate_v2_to_v3(error);
  }
  if (from_version == 2u) {
    return migrate_v2_to_v3(error);
  }
  error = "unsupported modeling schema migration source";
  return false;
}

// 将 v2 架构升级到 v3，补充模型原点和计划相关字段。
bool SqliteModelRepository::migrate_v2_to_v3(std::string& error) {
  if (!execute("BEGIN IMMEDIATE", error)) {
    return false;
  }
  const auto add_column = [this, &error](
      const std::string& table,
      const std::string& column,
      const std::string& definition) {
    if (table_has_column(database_, table, column, error)) {
      return true;
    }
    return execute(
        "ALTER TABLE " + table + " ADD COLUMN " + column + " " + definition,
        error);
  };
  if (!add_column(
          "model_points", "capture_type",
          "TEXT NOT NULL DEFAULT 'boundary'") ||
      !add_column(
          "model_groups", "recognition_status",
          "TEXT NOT NULL DEFAULT 'unrecognized'") ||
      !add_column(
          "model_groups", "recognition_message",
          "TEXT NOT NULL DEFAULT ''") ||
      !add_column(
          "model_groups", "recognition_confidence",
          "REAL NOT NULL DEFAULT 0") ||
      !add_column(
          "model_connectors", "from_sub_area_id",
          "TEXT NOT NULL DEFAULT ''") ||
      !add_column(
          "model_connectors", "to_sub_area_id",
          "TEXT NOT NULL DEFAULT ''")) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }
  if (!execute(
          "UPDATE model_points SET capture_type="
          "CASE WHEN role='connection_point' THEN 'connection' "
          "ELSE 'boundary' END;"
          "UPDATE model_groups SET recognition_status='unrecognized',"
          "recognition_message='requires re-recognition',"
          "recognition_confidence=0;"
          "UPDATE model_connectors SET confirmed=0,"
          "from_sub_area_id='',to_sub_area_id='';"
          "UPDATE modeling_metadata SET meta_value='3' "
          "WHERE meta_key='schema_version';COMMIT;",
          error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }
  return true;
}

// 将 v1 架构升级到 v2，补充点角色和区域连接数据。
bool SqliteModelRepository::migrate_v1_to_v2(std::string& error) {
  if (!execute(
          "BEGIN IMMEDIATE;"
          "ALTER TABLE cleaning_models ADD COLUMN origin_lat REAL NOT NULL DEFAULT 0;"
          "ALTER TABLE cleaning_models ADD COLUMN origin_lon REAL NOT NULL DEFAULT 0;"
          "ALTER TABLE cleaning_models ADD COLUMN origin_valid INTEGER NOT NULL DEFAULT 0;",
          error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }

  sqlite3_stmt* model_statement = nullptr;
  if (!prepare(
          database_,
          "SELECT model_id FROM cleaning_models ORDER BY model_id",
          &model_statement,
          error)) {
    execute("ROLLBACK", error);
    return false;
  }
  std::vector<std::string> model_ids;
  while (sqlite3_step(model_statement) == SQLITE_ROW) {
    model_ids.push_back(column_text(model_statement, 0));
  }
  sqlite3_finalize(model_statement);

  for (const auto& model_id : model_ids) {
    sqlite3_stmt* origin_statement = nullptr;
    if (!prepare(
            database_,
            "SELECT lat,lon FROM model_points WHERE model_id=? "
            "AND lat BETWEEN -90 AND 90 AND lon BETWEEN -180 AND 180 "
            "ORDER BY CASE WHEN version=0 THEN 0 ELSE 1 END,version,group_id,sequence "
            "LIMIT 1",
            &origin_statement,
            error)) {
      execute("ROLLBACK", error);
      return false;
    }
    bind_text(origin_statement, 1, model_id);
    const bool has_origin = sqlite3_step(origin_statement) == SQLITE_ROW;
    const double origin_lat = has_origin ? sqlite3_column_double(origin_statement, 0) : 0.0;
    const double origin_lon = has_origin ? sqlite3_column_double(origin_statement, 1) : 0.0;
    sqlite3_finalize(origin_statement);
    if (!has_origin || !std::isfinite(origin_lat) || !std::isfinite(origin_lon)) {
      continue;
    }

    sqlite3_stmt* update_model = nullptr;
    if (!prepare(
            database_,
            "UPDATE cleaning_models SET origin_lat=?,origin_lon=?,origin_valid=1 "
            "WHERE model_id=?",
            &update_model,
            error)) {
      execute("ROLLBACK", error);
      return false;
    }
    sqlite3_bind_double(update_model, 1, origin_lat);
    sqlite3_bind_double(update_model, 2, origin_lon);
    bind_text(update_model, 3, model_id);
    const bool model_updated = step_done(database_, update_model, error);
    sqlite3_finalize(update_model);
    if (!model_updated) {
      execute("ROLLBACK", error);
      return false;
    }

    sqlite3_stmt* point_statement = nullptr;
    if (!prepare(
            database_,
            "SELECT rowid,lat,lon FROM model_points WHERE model_id=?",
            &point_statement,
            error)) {
      execute("ROLLBACK", error);
      return false;
    }
    std::vector<MigratedPointRecord> migrated_points;
    bind_text(point_statement, 1, model_id);
    while (sqlite3_step(point_statement) == SQLITE_ROW) {
      const auto local = lat_lon_to_local_cm(
          origin_lat,
          origin_lon,
          sqlite3_column_double(point_statement, 1),
          sqlite3_column_double(point_statement, 2));
      migrated_points.push_back(MigratedPointRecord{
          sqlite3_column_int64(point_statement, 0), local.x_cm, local.y_cm});
    }
    sqlite3_finalize(point_statement);

    for (const auto& point : migrated_points) {
      sqlite3_stmt* update_point = nullptr;
      if (!prepare(
              database_,
              "UPDATE model_points SET x_cm=?,y_cm=? WHERE rowid=?",
              &update_point,
              error)) {
        execute("ROLLBACK", error);
        return false;
      }
      sqlite3_bind_double(update_point, 1, point.x_cm);
      sqlite3_bind_double(update_point, 2, point.y_cm);
      sqlite3_bind_int64(update_point, 3, point.rowid);
      const bool point_updated = step_done(database_, update_point, error);
      sqlite3_finalize(update_point);
      if (!point_updated) {
        execute("ROLLBACK", error);
        return false;
      }
    }
  }

  if (!execute(
          "UPDATE modeling_metadata SET meta_value='2' "
          "WHERE meta_key='schema_version';COMMIT;",
          error)) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return false;
  }
  return true;
}

// 在事务中覆盖保存指定模型的当前草稿及完整子结构。
ModelWriteResult SqliteModelRepository::save_draft(
    const CleaningModel& model) {
  if (database_ == nullptr) {
    return write_failure(
        "MODELING_DATABASE_NOT_OPEN",
        "modeling database is not open");
  }
  if (model.id.empty() || model.name.empty()) {
    return write_failure(
        "MODEL_IDENTITY_INVALID",
        "model id and name are required");
  }
  std::string error;
  if (!execute("BEGIN IMMEDIATE", error)) {
    return write_failure("MODELING_TRANSACTION_FAILED", error);
  }

  sqlite3_stmt* statement = nullptr;
  const char* model_sql =
      "INSERT INTO cleaning_models("
      "model_id,name,status,current_version,origin_lat,origin_lon,origin_valid) "
      "VALUES(?,?,?,0,?,?,?) ON CONFLICT(model_id) DO UPDATE SET "
      "name=excluded.name,status=excluded.status,origin_lat=excluded.origin_lat,"
      "origin_lon=excluded.origin_lon,origin_valid=excluded.origin_valid,"
      "updated_at=CURRENT_TIMESTAMP";
  bool success = prepare(database_, model_sql, &statement, error);
  if (success) {
    bind_text(statement, 1, model.id);
    bind_text(statement, 2, model.name);
    bind_text(statement, 3, model.status.empty() ? "draft" : model.status);
    sqlite3_bind_double(statement, 4, model.origin_lat);
    sqlite3_bind_double(statement, 5, model.origin_lon);
    sqlite3_bind_int(statement, 6, model.origin_valid ? 1 : 0);
    success = step_done(database_, statement, error);
  }
  sqlite3_finalize(statement);

  const char* draft_sql =
      "INSERT INTO model_drafts(model_id,recognition_confirmed,preview_confirmed) "
      "VALUES(?,?,?) ON CONFLICT(model_id) DO UPDATE SET "
      "recognition_confirmed=excluded.recognition_confirmed,"
      "preview_confirmed=excluded.preview_confirmed,"
      "updated_at=CURRENT_TIMESTAMP";
  statement = nullptr;
  if (success) {
    success = prepare(database_, draft_sql, &statement, error);
  }
  if (success) {
    bind_text(statement, 1, model.id);
    sqlite3_bind_int(statement, 2, model.recognition_confirmed ? 1 : 0);
    sqlite3_bind_int(statement, 3, model.preview_confirmed ? 1 : 0);
    success = step_done(database_, statement, error);
  }
  sqlite3_finalize(statement);

  if (success) {
    success = save_snapshot(model, 0u, error);
  }
  if (success) {
    success = execute("COMMIT", error);
  }
  if (!success) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return write_failure("MODEL_DRAFT_WRITE_FAILED", error);
  }

  ModelWriteResult result;
  result.success = true;
  result.code = "OK";
  result.message = "model draft saved";
  return result;
}

// 分配新版本号并保存不可变的正式模型快照。
ModelWriteResult SqliteModelRepository::save_formal_version(
    const CleaningModel& model) {
  if (database_ == nullptr) {
    return write_failure(
        "MODELING_DATABASE_NOT_OPEN",
        "modeling database is not open");
  }
  if (model.id.empty() || model.name.empty()) {
    return write_failure(
        "MODEL_IDENTITY_INVALID",
        "model id and name are required");
  }
  std::uint64_t version = 0u;
  std::string error;
  if (!next_version(model.id, version, error) ||
      !execute("BEGIN IMMEDIATE", error)) {
    return write_failure("MODEL_VERSION_BEGIN_FAILED", error);
  }

  sqlite3_stmt* statement = nullptr;
  const char* version_sql =
      "INSERT INTO model_versions("
      "model_id,version,name,status,recognition_confirmed,preview_confirmed)"
      "VALUES(?,?,?,?,?,?)";
  bool success = prepare(database_, version_sql, &statement, error);
  if (success) {
    bind_text(statement, 1, model.id);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
    bind_text(statement, 3, model.name);
    bind_text(statement, 4, "formal");
    sqlite3_bind_int(statement, 5, model.recognition_confirmed ? 1 : 0);
    sqlite3_bind_int(statement, 6, model.preview_confirmed ? 1 : 0);
    success = step_done(database_, statement, error);
  }
  sqlite3_finalize(statement);

  if (success) {
    success = save_snapshot(model, version, error);
  }

  const char* update_sql =
      "INSERT INTO cleaning_models("
      "model_id,name,status,current_version,origin_lat,origin_lon,origin_valid) "
      "VALUES(?,?,?,?,?,?,?) ON CONFLICT(model_id) DO UPDATE SET "
      "name=excluded.name,status=excluded.status,"
      "current_version=excluded.current_version,origin_lat=excluded.origin_lat,"
      "origin_lon=excluded.origin_lon,origin_valid=excluded.origin_valid,"
      "updated_at=CURRENT_TIMESTAMP";
  statement = nullptr;
  if (success) {
    success = prepare(database_, update_sql, &statement, error);
  }
  if (success) {
    bind_text(statement, 1, model.id);
    bind_text(statement, 2, model.name);
    bind_text(statement, 3, "formal");
    sqlite3_bind_int64(statement, 4, static_cast<sqlite3_int64>(version));
    sqlite3_bind_double(statement, 5, model.origin_lat);
    sqlite3_bind_double(statement, 6, model.origin_lon);
    sqlite3_bind_int(statement, 7, model.origin_valid ? 1 : 0);
    success = step_done(database_, statement, error);
  }
  sqlite3_finalize(statement);

  if (success) {
    success = execute("COMMIT", error);
  }
  if (!success) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return write_failure("MODEL_VERSION_WRITE_FAILED", error);
  }

  ModelWriteResult result;
  result.success = true;
  result.code = "OK";
  result.message = "formal model version saved";
  result.version = version;
  return result;
}

// 将模型组、点、子区域和连接关系写入指定版本快照。
bool SqliteModelRepository::save_snapshot(
    const CleaningModel& model,
    const std::uint64_t version,
    std::string& error) {
  const char* delete_tables[] = {
      "model_sub_area_points",
      "model_connectors",
      "model_sub_areas",
      "model_points",
      "model_groups",
  };
  for (const auto* table : delete_tables) {
    const std::string sql =
        "DELETE FROM " + std::string(table) +
        " WHERE model_id=? AND version=?";
    sqlite3_stmt* statement = nullptr;
    if (!prepare(database_, sql.c_str(), &statement, error)) {
      return false;
    }
    bind_text(statement, 1, model.id);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
    const bool deleted = step_done(database_, statement, error);
    sqlite3_finalize(statement);
    if (!deleted) {
      return false;
    }
  }

  for (std::size_t group_index = 0u;
       group_index < model.groups.size();
       ++group_index) {
    const auto& group = model.groups[group_index];
    sqlite3_stmt* group_statement = nullptr;
    if (!prepare(
            database_,
            "INSERT INTO model_groups("
            "model_id,version,group_id,group_order,name,area_number,"
            "sweep_mode,sweep_angle_deg,recognition_status,"
            "recognition_message,recognition_confidence) "
            "VALUES(?,?,?,?,?,?,?,?,?,?,?)",
            &group_statement,
            error)) {
      return false;
    }
    bind_text(group_statement, 1, model.id);
    sqlite3_bind_int64(
        group_statement, 2, static_cast<sqlite3_int64>(version));
    bind_text(group_statement, 3, group.id);
    sqlite3_bind_int64(
        group_statement, 4, static_cast<sqlite3_int64>(group_index));
    bind_text(group_statement, 5, group.name);
    sqlite3_bind_int64(group_statement, 6, group.area_number);
    bind_text(group_statement, 7, group.sweep_mode);
    sqlite3_bind_double(group_statement, 8, group.sweep_angle_deg);
    bind_text(group_statement, 9, group.recognition_status);
    bind_text(group_statement, 10, group.recognition_message);
    sqlite3_bind_double(group_statement, 11, group.recognition_confidence);
    const bool group_saved =
        step_done(database_, group_statement, error);
    sqlite3_finalize(group_statement);
    if (!group_saved) {
      return false;
    }

    for (const auto& point : group.points) {
      sqlite3_stmt* statement = nullptr;
      if (!prepare(
              database_,
              "INSERT INTO model_points("
              "model_id,version,group_id,point_id,sequence,lat,lon,x_cm,y_cm,"
              "heading_deg,heading_valid,capture_type,role,roles,sample_count,"
              "sample_radius_m,fix_quality,gga_age_sec,source) "
              "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
              &statement,
              error)) {
        return false;
      }
      bind_text(statement, 1, model.id);
      sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
      bind_text(statement, 3, group.id);
      bind_text(statement, 4, point.id);
      sqlite3_bind_int64(
          statement, 5, static_cast<sqlite3_int64>(point.sequence));
      sqlite3_bind_double(statement, 6, point.lat);
      sqlite3_bind_double(statement, 7, point.lon);
      sqlite3_bind_double(statement, 8, point.x_cm);
      sqlite3_bind_double(statement, 9, point.y_cm);
      sqlite3_bind_double(statement, 10, point.heading_deg);
      sqlite3_bind_int(statement, 11, point.heading_valid ? 1 : 0);
      bind_text(statement, 12, point.capture_type);
      bind_text(statement, 13, point.role);
      bind_text(statement, 14, join_roles(point.roles));
      sqlite3_bind_int64(
          statement, 15, static_cast<sqlite3_int64>(point.sample_count));
      sqlite3_bind_double(statement, 16, point.sample_radius_m);
      sqlite3_bind_int(statement, 17, point.fix_quality);
      sqlite3_bind_double(statement, 18, point.gga_age_sec);
      bind_text(statement, 19, point.source);
      const bool point_saved = step_done(database_, statement, error);
      sqlite3_finalize(statement);
      if (!point_saved) {
        return false;
      }
    }

    for (std::size_t area_index = 0u;
         area_index < group.sub_areas.size();
         ++area_index) {
      const auto& area = group.sub_areas[area_index];
      sqlite3_stmt* statement = nullptr;
      if (!prepare(
              database_,
              "INSERT INTO model_sub_areas("
              "model_id,version,group_id,sub_area_id,area_order,name,confirmed)"
              "VALUES(?,?,?,?,?,?,?)",
              &statement,
              error)) {
        return false;
      }
      bind_text(statement, 1, model.id);
      sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
      bind_text(statement, 3, group.id);
      bind_text(statement, 4, area.id);
      sqlite3_bind_int64(
          statement, 5, static_cast<sqlite3_int64>(area_index));
      bind_text(statement, 6, area.name);
      sqlite3_bind_int(statement, 7, area.confirmed ? 1 : 0);
      const bool area_saved = step_done(database_, statement, error);
      sqlite3_finalize(statement);
      if (!area_saved) {
        return false;
      }

      for (std::size_t point_index = 0u;
           point_index < area.point_ids.size();
           ++point_index) {
        statement = nullptr;
        if (!prepare(
                database_,
                "INSERT INTO model_sub_area_points("
                "model_id,version,group_id,sub_area_id,point_order,point_id)"
                "VALUES(?,?,?,?,?,?)",
                &statement,
                error)) {
          return false;
        }
        bind_text(statement, 1, model.id);
        sqlite3_bind_int64(
            statement, 2, static_cast<sqlite3_int64>(version));
        bind_text(statement, 3, group.id);
        bind_text(statement, 4, area.id);
        sqlite3_bind_int64(
            statement, 5, static_cast<sqlite3_int64>(point_index));
        bind_text(statement, 6, area.point_ids[point_index]);
        const bool link_saved = step_done(database_, statement, error);
        sqlite3_finalize(statement);
        if (!link_saved) {
          return false;
        }
      }
    }

    for (std::size_t connector_index = 0u;
         connector_index < group.connectors.size();
         ++connector_index) {
      const auto& connector = group.connectors[connector_index];
      sqlite3_stmt* statement = nullptr;
      if (!prepare(
              database_,
              "INSERT INTO model_connectors("
              "model_id,version,group_id,connector_id,connector_order,type,"
              "start_point_id,end_point_id,from_sub_area_id,to_sub_area_id,"
              "length_cm,confirmed)"
              "VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
              &statement,
              error)) {
        return false;
      }
      bind_text(statement, 1, model.id);
      sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
      bind_text(statement, 3, group.id);
      bind_text(statement, 4, connector.id);
      sqlite3_bind_int64(
          statement, 5, static_cast<sqlite3_int64>(connector_index));
      bind_text(statement, 6, connector.type);
      bind_text(statement, 7, connector.start_point_id);
      bind_text(statement, 8, connector.end_point_id);
      bind_text(statement, 9, connector.from_sub_area_id);
      bind_text(statement, 10, connector.to_sub_area_id);
      sqlite3_bind_double(statement, 11, connector.length_cm);
      sqlite3_bind_int(statement, 12, connector.confirmed ? 1 : 0);
      const bool connector_saved = step_done(database_, statement, error);
      sqlite3_finalize(statement);
      if (!connector_saved) {
        return false;
      }
    }
  }
  return true;
}

// 按模型标识读取当前草稿及其关联结构。
bool SqliteModelRepository::load_draft(
    const std::string& model_id,
    CleaningModel& model) const {
  if (database_ == nullptr) {
    return false;
  }
  sqlite3_stmt* statement = nullptr;
  std::string error;
  if (!prepare(
          database_,
          "SELECT m.name,m.status,m.origin_lat,m.origin_lon,m.origin_valid,"
          "d.recognition_confirmed,d.preview_confirmed "
          "FROM cleaning_models m JOIN model_drafts d ON d.model_id=m.model_id "
          "WHERE m.model_id=?",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, model_id);
  if (sqlite3_step(statement) != SQLITE_ROW) {
    sqlite3_finalize(statement);
    return false;
  }
  model = CleaningModel();
  model.id = model_id;
  model.name = column_text(statement, 0);
  model.status = column_text(statement, 1);
  model.origin_lat = sqlite3_column_double(statement, 2);
  model.origin_lon = sqlite3_column_double(statement, 3);
  model.origin_valid = sqlite3_column_int(statement, 4) != 0;
  model.recognition_confirmed = sqlite3_column_int(statement, 5) != 0;
  model.preview_confirmed = sqlite3_column_int(statement, 6) != 0;
  sqlite3_finalize(statement);
  return load_snapshot(model_id, 0u, model);
}

// 按模型标识和版本号读取正式快照。
bool SqliteModelRepository::load_version(
    const std::string& model_id,
    const std::uint64_t version,
    CleaningModel& model) const {
  if (database_ == nullptr || version == 0u) {
    return false;
  }
  sqlite3_stmt* statement = nullptr;
  std::string error;
  if (!prepare(
          database_,
          "SELECT v.name,v.status,v.recognition_confirmed,v.preview_confirmed,"
          "m.origin_lat,m.origin_lon,m.origin_valid "
          "FROM model_versions v JOIN cleaning_models m ON m.model_id=v.model_id "
          "WHERE v.model_id=? AND v.version=?",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, model_id);
  sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
  if (sqlite3_step(statement) != SQLITE_ROW) {
    sqlite3_finalize(statement);
    return false;
  }
  model = CleaningModel();
  model.id = model_id;
  model.version = version;
  model.name = column_text(statement, 0);
  model.status = column_text(statement, 1);
  model.recognition_confirmed = sqlite3_column_int(statement, 2) != 0;
  model.preview_confirmed = sqlite3_column_int(statement, 3) != 0;
  model.origin_lat = sqlite3_column_double(statement, 4);
  model.origin_lon = sqlite3_column_double(statement, 5);
  model.origin_valid = sqlite3_column_int(statement, 6) != 0;
  sqlite3_finalize(statement);
  return load_snapshot(model_id, version, model);
}

// 从多张关系表重建完整清扫模型对象。
bool SqliteModelRepository::load_snapshot(
    const std::string& model_id,
    const std::uint64_t version,
    CleaningModel& model) const {
  sqlite3_stmt* statement = nullptr;
  std::string error;
  if (!prepare(
          database_,
          "SELECT group_id,name,area_number,sweep_mode,sweep_angle_deg,"
          "recognition_status,recognition_message,recognition_confidence "
          "FROM model_groups WHERE model_id=? AND version=? ORDER BY group_order",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, model_id);
  sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
  model.groups.clear();
  while (sqlite3_step(statement) == SQLITE_ROW) {
    ModelGroup group;
    group.id = column_text(statement, 0);
    group.name = column_text(statement, 1);
    group.area_number =
        static_cast<std::uint32_t>(sqlite3_column_int64(statement, 2));
    group.sweep_mode = column_text(statement, 3);
    group.sweep_angle_deg = sqlite3_column_double(statement, 4);
    group.recognition_status = column_text(statement, 5);
    group.recognition_message = column_text(statement, 6);
    group.recognition_confidence = sqlite3_column_double(statement, 7);
    model.groups.push_back(group);
  }
  sqlite3_finalize(statement);

  for (auto& group : model.groups) {
    if (!prepare(
            database_,
            "SELECT point_id,sequence,lat,lon,x_cm,y_cm,heading_deg,"
            "heading_valid,capture_type,role,roles,sample_count,"
            "sample_radius_m,fix_quality,gga_age_sec,source FROM model_points "
            "WHERE model_id=? AND version=? AND group_id=? ORDER BY sequence",
            &statement,
            error)) {
      return false;
    }
    bind_text(statement, 1, model_id);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
    bind_text(statement, 3, group.id);
    while (sqlite3_step(statement) == SQLITE_ROW) {
      ModelPoint point;
      point.id = column_text(statement, 0);
      point.sequence =
          static_cast<std::size_t>(sqlite3_column_int64(statement, 1));
      point.lat = sqlite3_column_double(statement, 2);
      point.lon = sqlite3_column_double(statement, 3);
      point.x_cm = sqlite3_column_double(statement, 4);
      point.y_cm = sqlite3_column_double(statement, 5);
      point.heading_deg = sqlite3_column_double(statement, 6);
      point.heading_valid = sqlite3_column_int(statement, 7) != 0;
      point.role = column_text(statement, 9);
      point.capture_type = column_text(statement, 8);
      if (point.capture_type.empty()) {
        point.capture_type = point.role == "connection_point"
            ? "connection"
            : "boundary";
      }
      point.roles = split_roles(column_text(statement, 10));
      point.sample_count =
          static_cast<std::size_t>(sqlite3_column_int64(statement, 11));
      point.sample_radius_m = sqlite3_column_double(statement, 12);
      point.fix_quality =
          static_cast<std::uint8_t>(sqlite3_column_int(statement, 13));
      point.gga_age_sec = sqlite3_column_double(statement, 14);
      point.source = column_text(statement, 15);
      group.points.push_back(point);
    }
    sqlite3_finalize(statement);

    if (!prepare(
            database_,
            "SELECT sub_area_id,name,confirmed FROM model_sub_areas "
            "WHERE model_id=? AND version=? AND group_id=? ORDER BY area_order",
            &statement,
            error)) {
      return false;
    }
    bind_text(statement, 1, model_id);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
    bind_text(statement, 3, group.id);
    while (sqlite3_step(statement) == SQLITE_ROW) {
      ModelSubArea area;
      area.id = column_text(statement, 0);
      area.name = column_text(statement, 1);
      area.confirmed = sqlite3_column_int(statement, 2) != 0;
      group.sub_areas.push_back(area);
    }
    sqlite3_finalize(statement);

    for (auto& area : group.sub_areas) {
      if (!prepare(
              database_,
              "SELECT point_id FROM model_sub_area_points "
              "WHERE model_id=? AND version=? AND group_id=? AND sub_area_id=? "
              "ORDER BY point_order",
              &statement,
              error)) {
        return false;
      }
      bind_text(statement, 1, model_id);
      sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
      bind_text(statement, 3, group.id);
      bind_text(statement, 4, area.id);
      while (sqlite3_step(statement) == SQLITE_ROW) {
        area.point_ids.push_back(column_text(statement, 0));
      }
      sqlite3_finalize(statement);
    }

    if (!prepare(
            database_,
            "SELECT connector_id,type,start_point_id,end_point_id,"
            "from_sub_area_id,to_sub_area_id,length_cm,"
            "confirmed FROM model_connectors "
            "WHERE model_id=? AND version=? AND group_id=? "
            "ORDER BY connector_order",
            &statement,
            error)) {
      return false;
    }
    bind_text(statement, 1, model_id);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(version));
    bind_text(statement, 3, group.id);
    while (sqlite3_step(statement) == SQLITE_ROW) {
      ModelConnector connector;
      connector.id = column_text(statement, 0);
      connector.type = column_text(statement, 1);
      connector.start_point_id = column_text(statement, 2);
      connector.end_point_id = column_text(statement, 3);
      connector.from_sub_area_id = column_text(statement, 4);
      connector.to_sub_area_id = column_text(statement, 5);
      connector.length_cm = sqlite3_column_double(statement, 6);
      connector.confirmed = sqlite3_column_int(statement, 7) != 0;
      group.connectors.push_back(connector);
    }
    sqlite3_finalize(statement);
  }
  return true;
}

// 在事务中保存清扫计划元数据及全部有序任务段。
ModelWriteResult SqliteModelRepository::save_plan(
    const CleaningPlan& plan) {
  if (database_ == nullptr) {
    return write_failure(
        "MODELING_DATABASE_NOT_OPEN",
        "modeling database is not open");
  }
  if (plan.id.empty() || plan.model_id.empty() ||
      plan.model_version == 0u || plan.plan_hash.empty()) {
    return write_failure(
        "PLAN_IDENTITY_INVALID",
        "plan identity, model version and hash are required");
  }

  std::string error;
  if (!execute("BEGIN IMMEDIATE", error)) {
    return write_failure("PLAN_TRANSACTION_FAILED", error);
  }
  sqlite3_stmt* statement = nullptr;
  bool success = prepare(
      database_,
      "INSERT OR REPLACE INTO cleaning_plans("
      "plan_id,model_id,model_version,plan_hash,generated_at,brush_width_cm,"
      "minimum_overlap_cm,actual_overlap_cm,cleaning_lane_count,"
      "transfer_segment_count,total_length_cm,preview_confirmed)"
      "VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
      &statement,
      error);
  if (success) {
    bind_text(statement, 1, plan.id);
    bind_text(statement, 2, plan.model_id);
    sqlite3_bind_int64(
        statement, 3, static_cast<sqlite3_int64>(plan.model_version));
    bind_text(statement, 4, plan.plan_hash);
    sqlite3_bind_int64(
        statement, 5, static_cast<sqlite3_int64>(plan.generated_at));
    sqlite3_bind_double(statement, 6, plan.brush_width_cm);
    sqlite3_bind_double(statement, 7, plan.minimum_overlap_cm);
    sqlite3_bind_double(statement, 8, plan.actual_overlap_cm);
    sqlite3_bind_int64(
        statement, 9, static_cast<sqlite3_int64>(plan.cleaning_lane_count));
    sqlite3_bind_int64(
        statement, 10,
        static_cast<sqlite3_int64>(plan.transfer_segment_count));
    sqlite3_bind_double(statement, 11, plan.total_length_cm);
    sqlite3_bind_int(statement, 12, plan.preview_confirmed ? 1 : 0);
    success = step_done(database_, statement, error);
  }
  sqlite3_finalize(statement);

  if (success) {
    statement = nullptr;
    success = prepare(
        database_,
        "DELETE FROM plan_segments WHERE plan_id=?",
        &statement,
        error);
    if (success) {
      bind_text(statement, 1, plan.id);
      success = step_done(database_, statement, error);
    }
    sqlite3_finalize(statement);
  }

  for (const auto& segment : plan.segments) {
    if (!success) {
      break;
    }
    statement = nullptr;
    success = prepare(
        database_,
        "INSERT INTO plan_segments("
        "plan_id,segment_index,segment_id,segment_type,group_id,sub_area_id,"
        "source_lane_id,start_x_cm,start_y_cm,end_x_cm,end_y_cm,start_lat,"
        "start_lon,end_lat,end_lon,heading_deg,turn_angle_deg,speed,mode,"
        "brush_enabled) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
        &statement,
        error);
    if (success) {
      bind_text(statement, 1, plan.id);
      sqlite3_bind_int64(
          statement, 2, static_cast<sqlite3_int64>(segment.index));
      bind_text(statement, 3, segment.id);
      sqlite3_bind_int(statement, 4, segment.segment_type);
      bind_text(statement, 5, segment.group_id);
      bind_text(statement, 6, segment.sub_area_id);
      bind_text(statement, 7, segment.source_lane_id);
      sqlite3_bind_double(statement, 8, segment.start.x_cm);
      sqlite3_bind_double(statement, 9, segment.start.y_cm);
      sqlite3_bind_double(statement, 10, segment.end.x_cm);
      sqlite3_bind_double(statement, 11, segment.end.y_cm);
      sqlite3_bind_double(statement, 12, segment.start_lat);
      sqlite3_bind_double(statement, 13, segment.start_lon);
      sqlite3_bind_double(statement, 14, segment.end_lat);
      sqlite3_bind_double(statement, 15, segment.end_lon);
      sqlite3_bind_double(statement, 16, segment.heading_deg);
      sqlite3_bind_double(statement, 17, segment.turn_angle_deg);
      sqlite3_bind_int(statement, 18, segment.speed);
      sqlite3_bind_int(statement, 19, segment.mode);
      sqlite3_bind_int(statement, 20, segment.brush_enabled ? 1 : 0);
      success = step_done(database_, statement, error);
    }
    sqlite3_finalize(statement);
  }

  if (success) {
    success = execute("COMMIT", error);
  }
  if (!success) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
    return write_failure("PLAN_WRITE_FAILED", error);
  }

  ModelWriteResult result;
  result.success = true;
  result.code = "OK";
  result.message = "cleaning plan saved";
  result.version = plan.model_version;
  return result;
}

// 按计划标识读取计划元数据和有序任务段。
bool SqliteModelRepository::load_plan(
    const std::string& plan_id,
    CleaningPlan& plan) const {
  if (database_ == nullptr) {
    return false;
  }
  sqlite3_stmt* statement = nullptr;
  std::string error;
  if (!prepare(
          database_,
          "SELECT model_id,model_version,plan_hash,generated_at,brush_width_cm,"
          "minimum_overlap_cm,actual_overlap_cm,cleaning_lane_count,"
          "transfer_segment_count,total_length_cm,preview_confirmed "
          "FROM cleaning_plans WHERE plan_id=?",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, plan_id);
  if (sqlite3_step(statement) != SQLITE_ROW) {
    sqlite3_finalize(statement);
    return false;
  }
  plan = CleaningPlan();
  plan.id = plan_id;
  plan.model_id = column_text(statement, 0);
  plan.model_version =
      static_cast<std::uint64_t>(sqlite3_column_int64(statement, 1));
  plan.plan_hash = column_text(statement, 2);
  plan.generated_at =
      static_cast<std::uint64_t>(sqlite3_column_int64(statement, 3));
  plan.brush_width_cm = sqlite3_column_double(statement, 4);
  plan.minimum_overlap_cm = sqlite3_column_double(statement, 5);
  plan.actual_overlap_cm = sqlite3_column_double(statement, 6);
  plan.cleaning_lane_count =
      static_cast<std::size_t>(sqlite3_column_int64(statement, 7));
  plan.transfer_segment_count =
      static_cast<std::size_t>(sqlite3_column_int64(statement, 8));
  plan.total_length_cm = sqlite3_column_double(statement, 9);
  plan.preview_confirmed = sqlite3_column_int(statement, 10) != 0;
  sqlite3_finalize(statement);

  if (!prepare(
          database_,
          "SELECT segment_index,segment_id,segment_type,group_id,sub_area_id,"
          "source_lane_id,start_x_cm,start_y_cm,end_x_cm,end_y_cm,start_lat,"
          "start_lon,end_lat,end_lon,heading_deg,turn_angle_deg,speed,mode,"
          "brush_enabled FROM plan_segments WHERE plan_id=? "
          "ORDER BY segment_index",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, plan_id);
  while (sqlite3_step(statement) == SQLITE_ROW) {
    PlanSegment segment;
    segment.index =
        static_cast<std::size_t>(sqlite3_column_int64(statement, 0));
    segment.id = column_text(statement, 1);
    segment.segment_type =
        static_cast<std::uint8_t>(sqlite3_column_int(statement, 2));
    segment.group_id = column_text(statement, 3);
    segment.sub_area_id = column_text(statement, 4);
    segment.source_lane_id = column_text(statement, 5);
    segment.start.x_cm = sqlite3_column_double(statement, 6);
    segment.start.y_cm = sqlite3_column_double(statement, 7);
    segment.end.x_cm = sqlite3_column_double(statement, 8);
    segment.end.y_cm = sqlite3_column_double(statement, 9);
    segment.start_lat = sqlite3_column_double(statement, 10);
    segment.start_lon = sqlite3_column_double(statement, 11);
    segment.end_lat = sqlite3_column_double(statement, 12);
    segment.end_lon = sqlite3_column_double(statement, 13);
    segment.heading_deg = sqlite3_column_double(statement, 14);
    segment.turn_angle_deg = sqlite3_column_double(statement, 15);
    segment.speed = sqlite3_column_int(statement, 16);
    segment.mode =
        static_cast<std::uint8_t>(sqlite3_column_int(statement, 17));
    segment.brush_enabled = sqlite3_column_int(statement, 18) != 0;
    plan.segments.push_back(segment);
  }
  sqlite3_finalize(statement);
  return true;
}

// 查询模型当前最大版本并计算下一可用版本号。
bool SqliteModelRepository::next_version(
    const std::string& model_id,
    std::uint64_t& version,
    std::string& error) const {
  sqlite3_stmt* statement = nullptr;
  if (!prepare(
          database_,
          "SELECT COALESCE(MAX(version),0)+1 FROM model_versions "
          "WHERE model_id=?",
          &statement,
          error)) {
    return false;
  }
  bind_text(statement, 1, model_id);
  if (sqlite3_step(statement) != SQLITE_ROW) {
    error = modeling_sqlite_error(database_);
    sqlite3_finalize(statement);
    return false;
  }
  version =
      static_cast<std::uint64_t>(sqlite3_column_int64(statement, 0));
  sqlite3_finalize(statement);
  return version > 0u;
}

// 按稳定顺序列出数据库中的全部模型标识。
std::vector<std::string> SqliteModelRepository::list_model_ids() const {
  std::vector<std::string> ids;
  if (database_ == nullptr) {
    return ids;
  }
  sqlite3_stmt* statement = nullptr;
  std::string error;
  if (!prepare(
          database_,
          "SELECT model_id FROM cleaning_models ORDER BY updated_at DESC,model_id",
          &statement,
          error)) {
    return ids;
  }
  while (sqlite3_step(statement) == SQLITE_ROW) {
    ids.push_back(column_text(statement, 0));
  }
  sqlite3_finalize(statement);
  return ids;
}

// 在事务中删除模型草稿、版本、子结构和关联计划。
bool SqliteModelRepository::delete_model(const std::string& model_id) {
  if (database_ == nullptr || model_id.empty()) {
    return false;
  }
  std::string error;
  if (!execute("BEGIN IMMEDIATE", error)) {
    return false;
  }
  const char* tables[] = {
      "plan_segments",
      "cleaning_plans",
      "model_sub_area_points",
      "model_connectors",
      "model_sub_areas",
      "model_points",
      "model_groups",
      "model_versions",
      "model_drafts",
      "cleaning_models",
  };
  bool success = true;
  for (const auto* table : tables) {
    std::string sql;
    if (std::string(table) == "plan_segments") {
      sql =
          "DELETE FROM plan_segments WHERE plan_id IN "
          "(SELECT plan_id FROM cleaning_plans WHERE model_id=?)";
    } else {
      sql = "DELETE FROM " + std::string(table) +
          (std::string(table) == "cleaning_plans"
               ? " WHERE model_id=?"
               : " WHERE model_id=?");
    }
    sqlite3_stmt* statement = nullptr;
    if (!prepare(database_, sql.c_str(), &statement, error)) {
      success = false;
      break;
    }
    bind_text(statement, 1, model_id);
    success = step_done(database_, statement, error);
    sqlite3_finalize(statement);
    if (!success) {
      break;
    }
  }
  if (success) {
    success = execute("COMMIT", error);
  }
  if (!success) {
    std::string rollback_error;
    execute("ROLLBACK", rollback_error);
  }
  return success;
}

// 执行无需返回结果集的 SQL，并输出失败原因。
bool SqliteModelRepository::execute(
    const std::string& sql,
    std::string& error) const {
  char* sqlite_message = nullptr;
  const int result = sqlite3_exec(
      database_, sql.c_str(), nullptr, nullptr, &sqlite_message);
  if (result == SQLITE_OK) {
    return true;
  }
  error = sqlite_message == nullptr
      ? modeling_sqlite_error(database_)
      : sqlite_message;
  sqlite3_free(sqlite_message);
  return false;
}

// 关闭 SQLite 数据库并清空连接指针。
void SqliteModelRepository::close() {
  if (database_ != nullptr) {
    sqlite3_close(database_);
    database_ = nullptr;
  }
}

}  // namespace modeling
}  // namespace cleanbot
