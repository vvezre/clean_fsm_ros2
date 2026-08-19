#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_repository.hpp"

struct sqlite3;

// 文件作用：声明使用 SQLite 保存清扫模型草稿、版本和计划的仓库实现。
namespace cleanbot {
namespace modeling {

class SqliteModelRepository final : public ModelRepository {
 public:
  // 使用指定 SQLite 数据库路径创建仓库。
  explicit SqliteModelRepository(std::string database_path);
  // 析构时关闭可能仍打开的数据库连接。
  ~SqliteModelRepository() override;

  // 打开数据库、创建或迁移表结构。
  ModelRepositoryStatus open_and_initialize() override;
  // 保存一份可覆盖更新的模型草稿。
  ModelWriteResult save_draft(const CleaningModel& model) override;
  // 读取指定模型草稿。
  bool load_draft(
      const std::string& model_id,
      CleaningModel& model) const override;
  // 保存一个自动分配版本号的正式模型快照。
  ModelWriteResult save_formal_version(
      const CleaningModel& model) override;
  // 读取指定模型的指定正式版本。
  bool load_version(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const override;
  // 保存一份由模型版本生成的计划。
  ModelWriteResult save_plan(const CleaningPlan& plan) override;
  // 读取指定计划。
  bool load_plan(
      const std::string& plan_id,
      CleaningPlan& plan) const override;
  // 列出数据库中所有模型标识。
  std::vector<std::string> list_model_ids() const override;
  // 删除模型、版本和相关计划记录。
  bool delete_model(const std::string& model_id) override;
  // 关闭数据库连接。
  void close() override;

 private:
  // 执行不返回行集的 SQL 并输出错误信息。
  bool execute(const std::string& sql, std::string& error) const;
  // 创建最新数据库表和索引。
  bool create_schema(std::string& error);
  // 将旧架构逐版本迁移到最新版本。
  bool migrate_schema(std::uint32_t from_version, std::string& error);
  // 执行 v1 到 v2 的数据库迁移。
  bool migrate_v1_to_v2(std::string& error);
  // 执行 v2 到 v3 的数据库迁移。
  bool migrate_v2_to_v3(std::string& error);
  // 将模型序列化并写入指定版本快照。
  bool save_snapshot(
      const CleaningModel& model,
      std::uint64_t version,
      std::string& error);
  // 从数据库读取模型版本并反序列化。
  bool load_snapshot(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const;
  // 查询指定模型下一可用的正式版本号。
  bool next_version(
      const std::string& model_id,
      std::uint64_t& version,
      std::string& error) const;

  std::string database_path_;
  sqlite3* database_{nullptr};
};

}  // namespace modeling
}  // namespace cleanbot
