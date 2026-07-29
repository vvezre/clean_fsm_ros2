#include "cleanbot_mission/straight_edge_guard.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace mission {

StraightEdgeGuard::StraightEdgeGuard(
    const std::uint64_t debounce_ms,
    const double target_tolerance_m)
    : debounce_ms_(debounce_ms),
      target_tolerance_m_(std::max(0.0, target_tolerance_m)) {}

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

void StraightEdgeGuard::reset() {
  asserted_since_ms_ = 0u;
  assertion_active_ = false;
  decision_emitted_ = false;
}

}  // namespace mission
}  // namespace cleanbot
