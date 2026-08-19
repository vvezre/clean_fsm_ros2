#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明清扫模型、版本和计划持久化仓库的抽象接口。
namespace cleanbot {
namespace modeling {

// 仓库初始化后的可用性、可写性和数据库架构状态。
struct ModelRepositoryStatus {
  bool healthy{false};
  bool writable{false};
  std::string code;
  std::string message;
  std::uint32_t schema_version{0u};
};

// 单次模型或计划写操作的成败、说明和版本号。
struct ModelWriteResult {
  bool success{false};
  std::string code;
  std::string message;
  std::uint64_t version{0u};
};

class ModelRepository {
 public:
  // 允许派生仓库通过基类指针正确析构。
  virtual ~ModelRepository() = default;

  // 打开存储并初始化或迁移数据库架构。
  virtual ModelRepositoryStatus open_and_initialize() = 0;
  // 保存可继续编辑的模型草稿。
  virtual ModelWriteResult save_draft(const CleaningModel& model) = 0;
  // 按模型标识读取草稿。
  virtual bool load_draft(
      const std::string& model_id,
      CleaningModel& model) const = 0;
  // 保存不可变的正式模型版本。
  virtual ModelWriteResult save_formal_version(
      const CleaningModel& model) = 0;
  // 按模型标识和版本号读取正式模型。
  virtual bool load_version(
      const std::string& model_id,
      std::uint64_t version,
      CleaningModel& model) const = 0;
  // 保存由模型生成的清扫计划。
  virtual ModelWriteResult save_plan(const CleaningPlan& plan) = 0;
  // 按计划标识读取清扫计划。
  virtual bool load_plan(
      const std::string& plan_id,
      CleaningPlan& plan) const = 0;
  // 列出已保存模型的标识。
  virtual std::vector<std::string> list_model_ids() const = 0;
  // 删除一个模型及其关联数据。
  virtual bool delete_model(const std::string& model_id) = 0;
  // 关闭仓库连接并释放资源。
  virtual void close() = 0;
};

}  // namespace modeling
}  // namespace cleanbot
