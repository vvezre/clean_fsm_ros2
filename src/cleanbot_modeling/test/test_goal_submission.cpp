// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include "cleanbot_modeling/goal_submission.hpp"

#include <gtest/gtest.h>

using cleanbot::modeling::GoalResponseState;
using cleanbot::modeling::classify_goal_response;

// 测试目的：验证 GoalSubmission.ReportsAcceptedGoal 场景的行为、状态变化和边界条件。
TEST(GoalSubmission, ReportsAcceptedGoal) {
  const auto decision =
      classify_goal_response(GoalResponseState::kAccepted);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_ACCEPTED");
}

// 测试目的：验证 GoalSubmission.ReportsRejectedGoal 场景的行为、状态变化和边界条件。
TEST(GoalSubmission, ReportsRejectedGoal) {
  const auto decision =
      classify_goal_response(GoalResponseState::kRejected);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_REJECTED");
}

// 测试目的：验证 GoalSubmission.ReportsGoalResponseTimeout 场景的行为、状态变化和边界条件。
TEST(GoalSubmission, ReportsGoalResponseTimeout) {
  const auto decision =
      classify_goal_response(GoalResponseState::kTimeout);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_RESPONSE_TIMEOUT");
  EXPECT_EQ(cleanbot::modeling::kGoalResponseDeadline.count(), 1000);
}
