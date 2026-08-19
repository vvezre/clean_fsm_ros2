/*
 * 文件作用：直边保护实现：根据边缘和传感器证据限制任务运动。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_mission/straight_edge_guard.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace mission {

// 保存边沿信号去抖时间，并将目标距离容差限制为非负值。
StraightEdgeGuard::StraightEdgeGuard(
    const std::uint64_t debounce_ms,
    const double target_tolerance_m)
    : debounce_ms_(debounce_ms),
      target_tolerance_m_(std::max(0.0, target_tolerance_m)) {}

// 对边沿信号去抖，并结合目标距离判断到达或需要人工确认。
StraightEdgeDecision StraightEdgeGuard::evaluate(
    const bool edge_clear,
    const std::uint64_t now_ms,
    const double distance_to_target_m,
    const double signed_remaining_m) {
  if (edge_clear) {
    reset();
    return StraightEdgeDecision::kNone;
  }

  if (!assertion_active_) {
    assertion_active_ = true;
    asserted_since_ms_ = now_ms;
    return StraightEdgeDecision::kNone;
  }
  if (decision_emitted_ || now_ms - asserted_since_ms_ < debounce_ms_) {
    return StraightEdgeDecision::kNone;
  }

  decision_emitted_ = true;
  const bool near_target = std::isfinite(distance_to_target_m) &&
      distance_to_target_m <= target_tolerance_m_;
  const bool passed_target = std::isfinite(signed_remaining_m) &&
      signed_remaining_m <= 0.0;
  return near_target || passed_target
      ? StraightEdgeDecision::kTargetReached
      : StraightEdgeDecision::kConfirmRequired;
}

// 清除边沿断言起始时间和已发出决策状态。
void StraightEdgeGuard::reset() {
  asserted_since_ms_ = 0u;
  assertion_active_ = false;
  decision_emitted_ = false;
}

}  // namespace mission
}  // namespace cleanbot
