#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

struct ModelRepositoryStatus {
  bool healthy{false};
  bool writable{false};
  std::string code;
  std::string message;
  std::uint32_t schema_version{0u};
};

struct ModelWriteResult {
  bool success{false};
  std::string code;
  std::string message;
  std::uint64_t version{0u};
};

class ModelRepository {
 public:
  virtual ~ModelRepository() = default;

  virtual ModelRepositoryStatus open_and_initialize() = 0;
  virtual ModelWriteResult save_draft(const CleaningModel& model) = 0;
  virtual bool load_draft(
      const std::string& model_id,
      CleaningModel& model) const = 0;
  virtual ModelWriteResult save_formal_version(
      const CleaningModel& model) = 0;
  virtual bool load_version(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const = 0;
  virtual ModelWriteResult save_plan(const CleaningPlan& plan) = 0;
  virtual bool load_plan(
      const std::string& plan_id,
      CleaningPlan& plan) const = 0;
  virtual std::vector<std::string> list_model_ids() const = 0;
  virtual bool delete_model(const std::string& model_id) = 0;
  virtual void close() = 0;
};

}  // namespace modeling
}  // namespace cleanbot
