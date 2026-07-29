#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

struct TaskPlanResult {
  bool success{false};
  std::string code;
  std::string message;
  CleaningPlan plan;
};

TaskPlanResult build_task_plan(
    const CleaningModel& model,
    double brush_width_cm,
    double minimum_overlap_cm,
    std::int32_t speed);

}  // namespace modeling
}  // namespace cleanbot
