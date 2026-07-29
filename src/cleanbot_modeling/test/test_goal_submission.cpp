#include "cleanbot_modeling/goal_submission.hpp"

#include <gtest/gtest.h>

using cleanbot::modeling::GoalResponseState;
using cleanbot::modeling::classify_goal_response;

TEST(GoalSubmission, ReportsAcceptedGoal) {
  const auto decision =
      classify_goal_response(GoalResponseState::kAccepted);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_ACCEPTED");
}

TEST(GoalSubmission, ReportsRejectedGoal) {
  const auto decision =
      classify_goal_response(GoalResponseState::kRejected);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_REJECTED");
}

TEST(GoalSubmission, ReportsGoalResponseTimeout) {
  const auto decision =
      classify_goal_response(GoalResponseState::kTimeout);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, "MISSION_GOAL_RESPONSE_TIMEOUT");
  EXPECT_EQ(cleanbot::modeling::kGoalResponseDeadline.count(), 1000);
}
