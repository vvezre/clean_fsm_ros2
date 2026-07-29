#ifndef CLEANBOT_MISSION__STRAIGHT_EDGE_GUARD_HPP_
#define CLEANBOT_MISSION__STRAIGHT_EDGE_GUARD_HPP_

#include <cstdint>

namespace cleanbot {
namespace mission {

enum class StraightEdgeDecision {
  kNone,
  kTargetReached,
  kConfirmRequired,
};

// Classifies a debounced edge signal while a straight segment is active.
class StraightEdgeGuard {
 public:
  StraightEdgeGuard(std::uint64_t debounce_ms, double target_tolerance_m);

  StraightEdgeDecision evaluate(
      bool edge_clear,
      std::uint64_t now_ms,
      double distance_to_target_m,
      double signed_remaining_m);
  void reset();

 private:
  std::uint64_t debounce_ms_{200u};
  double target_tolerance_m_{0.10};
  std::uint64_t asserted_since_ms_{0u};
  bool assertion_active_{false};
  bool decision_emitted_{false};
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__STRAIGHT_EDGE_GUARD_HPP_
