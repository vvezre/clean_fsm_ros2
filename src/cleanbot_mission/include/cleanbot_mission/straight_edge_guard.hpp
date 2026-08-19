#ifndef CLEANBOT_MISSION__STRAIGHT_EDGE_GUARD_HPP_
#define CLEANBOT_MISSION__STRAIGHT_EDGE_GUARD_HPP_

#include <cstdint>

// 文件作用：声明直线段边沿信号的去抖、目标到达和人工确认判定器。
namespace cleanbot {
namespace mission {

// 直线边沿信号评估产生的决策。
enum class StraightEdgeDecision {
  kNone,
  kTargetReached,
  kConfirmRequired,
};

// 在直线段执行期间对去抖后的边沿信号进行分类。
class StraightEdgeGuard {
 public:
  // 使用边沿去抖时间和目标距离容差创建判定器。
  StraightEdgeGuard(std::uint64_t debounce_ms, double target_tolerance_m);

  // 根据边沿状态、时间和目标距离产生继续、到达或确认需求决策。
  StraightEdgeDecision evaluate(
      bool edge_clear,
      std::uint64_t now_ms,
      double distance_to_target_m,
      double signed_remaining_m);
  // 清空当前边沿断言和已发出决策状态。
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
