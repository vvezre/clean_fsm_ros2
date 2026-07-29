#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_repository.hpp"

struct sqlite3;

namespace cleanbot {
namespace modeling {

class SqliteModelRepository final : public ModelRepository {
 public:
  explicit SqliteModelRepository(std::string database_path);
  ~SqliteModelRepository() override;

  ModelRepositoryStatus open_and_initialize() override;
  ModelWriteResult save_draft(const CleaningModel& model) override;
  bool load_draft(
      const std::string& model_id,
      CleaningModel& model) const override;
  ModelWriteResult save_formal_version(
      const CleaningModel& model) override;
  bool load_version(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const override;
  ModelWriteResult save_plan(const CleaningPlan& plan) override;
  bool load_plan(
      const std::string& plan_id,
      CleaningPlan& plan) const override;
  std::vector<std::string> list_model_ids() const override;
  bool delete_model(const std::string& model_id) override;
  void close() override;

 private:
  bool execute(const std::string& sql, std::string& error) const;
  bool create_schema(std::string& error);
  bool migrate_schema(std::uint32_t from_version, std::string& error);
  bool migrate_v1_to_v2(std::string& error);
  bool migrate_v2_to_v3(std::string& error);
  bool save_snapshot(
      const CleaningModel& model,
      std::uint64_t version,
      std::string& error);
  bool load_snapshot(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const;
  bool next_version(
      const std::string& model_id,
      std::uint64_t& version,
      std::string& error) const;

  std::string database_path_;
  sqlite3* database_{nullptr};
};

}  // namespace modeling
}  // namespace cleanbot
