#pragma once

#include <chrono>
#include <string>

namespace cleanbot {
namespace modeling {

inline constexpr std::chrono::milliseconds kGoalResponseDeadline{1000};

enum class GoalResponseState {
  kAccepted,
  kRejected,
  kTimeout,
};

struct GoalSubmissionDecision {
  bool accepted{false};
  std::string code;
  std::string message;
};

GoalSubmissionDecision classify_goal_response(GoalResponseState state);

}  // namespace modeling
}  // namespace cleanbot
