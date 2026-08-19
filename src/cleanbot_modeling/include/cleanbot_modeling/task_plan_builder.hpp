#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明将已确认清扫模型转化为可执行任务计划的构建接口。
namespace cleanbot {
namespace modeling {

// 任务计划构建的成败、错误说明和生成计划。
struct TaskPlanResult {
  bool success{false};
  std::string code;
  std::string message;
  CleaningPlan plan;
};

// 基于模型、刷盘宽度、重叠量和速度生成完整清扫任务计划。
TaskPlanResult build_task_plan(
    const CleaningModel& model,
    double brush_width_cm,
    double minimum_overlap_cm,
    std::int32_t speed);

}  // namespace modeling
}  // namespace cleanbot
