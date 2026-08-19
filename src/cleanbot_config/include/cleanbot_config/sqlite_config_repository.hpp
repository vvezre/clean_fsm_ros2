// 文件作用：声明SQLite配置仓库，实现配置的事务化读写和完整性校验。
#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_config/config_repository.hpp"

struct sqlite3;

namespace cleanbot {
namespace config {

class SqliteConfigRepository : public ConfigRepository {
 public:
  // 保存数据库路径和最小磁盘可用空间阈值，但不立即打开数据库。
  explicit SqliteConfigRepository(
      std::string database_path,
      std::uint64_t minimum_free_space_bytes = 64u * 1024u * 1024u);
  // 关闭数据库连接并释放SQLite资源。
  ~SqliteConfigRepository() override;

  // 打开数据库、创建或升级表结构并同步默认配置。
  RepositoryStatus open_and_initialize(const ConfigRegistry& registry) override;
  // 从数据库读取全部配置键值。
  std::map<std::string, std::string> read_all() const override;
  // 在事务中校验并写入配置，返回修订号和变更键。
  WriteResult write_values(
      const std::map<std::string, std::string>& values,
      const std::string& updated_by,
      const ConfigRegistry& registry) override;
  // 显式关闭数据库，允许节点按生命周期提前释放文件句柄。
  void close() override;

 private:
  // 执行不返回结果集的SQL语句，并把失败原因写入error。
  bool execute(const std::string& sql, std::string& error) const;
  // 创建初始表结构和必要索引。
  bool create_schema(const ConfigRegistry& registry, std::string& error);
  // 向空仓库插入单个配置键的默认值。
  bool insert_default(
      const ConfigDefinition& definition,
      std::string& error);
  // 读取当前配置修订号。
  bool read_revision(std::uint64_t& revision, std::string& error) const;
  // 运行SQLite快速完整性检查，确认数据库可安全使用。
  bool quick_check(std::string& error) const;

  std::string database_path_;
  std::uint64_t minimum_free_space_bytes_{0u};
  sqlite3* database_{nullptr};
  std::uint64_t revision_{0u};
};

}  // namespace config
}  // namespace cleanbot
