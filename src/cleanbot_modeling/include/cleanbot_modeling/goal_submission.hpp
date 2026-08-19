#pragma once

#include <chrono>
#include <string>

// 文件作用：声明模型计划 Action 目标响应的等待时限和结果分类规则。
namespace cleanbot {
namespace modeling {

// 等待下游 Action 接受或拒绝目标的最大时间。
inline constexpr std::chrono::milliseconds kGoalResponseDeadline{1000};

// 下游 Action 对目标的响应状态。
enum class GoalResponseState {
  kAccepted,
  kRejected,
  kTimeout,
};

// 对外返回的目标提交接受结果和业务说明。
struct GoalSubmissionDecision {
  bool accepted{false};
  std::string code;
  std::string message;
};

// 将下游 Action 响应状态转换为稳定业务结果。
GoalSubmissionDecision classify_goal_response(GoalResponseState state);

}  // namespace modeling
}  // namespace cleanbot
