// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_mission/mission_state_machine.hpp"

using cleanbot::mission::MissionState;
using cleanbot::mission::MissionStateMachine;

// 测试目的：验证 MissionStateMachine.MatchesTurnAndTrackingBeforeAdvancing 场景的行为、状态变化和边界条件。
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

// 测试目的：验证 MissionStateMachine.ResumeAndRtkRecoveryRestartCurrentSegment 场景的行为、状态变化和边界条件。
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
