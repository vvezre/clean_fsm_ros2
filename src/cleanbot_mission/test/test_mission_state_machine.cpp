#include <gtest/gtest.h>

#include "cleanbot_mission/mission_state_machine.hpp"

using cleanbot::mission::MissionState;
using cleanbot::mission::MissionStateMachine;

TEST(MissionStateMachine, MatchesTurnAndTrackingBeforeAdvancing) {
  MissionStateMachine machine;
  ASSERT_TRUE(machine.start(1u));
  ASSERT_TRUE(machine.begin_turn(10u));
  EXPECT_FALSE(machine.complete_turn(9u));
  ASSERT_TRUE(machine.complete_turn(10u));
  ASSERT_TRUE(machine.begin_tracking(20u));
  EXPECT_FALSE(machine.complete_tracking(19u));
  ASSERT_TRUE(machine.complete_tracking(20u));
  EXPECT_EQ(machine.state(), MissionState::kCompleted);
}

TEST(MissionStateMachine, ResumeAndRtkRecoveryRestartCurrentSegment) {
  MissionStateMachine machine;
  ASSERT_TRUE(machine.start(1u));
  ASSERT_TRUE(machine.begin_tracking(20u));
  ASSERT_TRUE(machine.pause());
  ASSERT_TRUE(machine.resume());
  EXPECT_EQ(machine.state(), MissionState::kPreparing);
  EXPECT_FALSE(machine.complete_tracking(20u));

  ASSERT_TRUE(machine.begin_tracking(21u));
  ASSERT_TRUE(machine.begin_rtk_recovery());
  ASSERT_TRUE(machine.recover_rtk());
  EXPECT_EQ(machine.state(), MissionState::kPreparing);
  EXPECT_EQ(machine.current_segment(), 0u);
}
