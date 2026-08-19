// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include "cleanbot_http/user_control_request.hpp"

#include <gtest/gtest.h>

using cleanbot::http::build_emergency_command;
using cleanbot::http::build_manual_command;

// 测试目的：验证 UserControlRequest.RejectsEmptyManualSource 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, RejectsEmptyManualSource) {
  const auto result = build_manual_command("", 0, 0, true, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_SOURCE_REQUIRED");
}

// 测试目的：验证 UserControlRequest.RejectsManualLinearSpeedOutsideConfiguredLimit 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, RejectsManualLinearSpeedOutsideConfiguredLimit) {
  const auto result = build_manual_command(
      "service", 351, 0, false, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_X_SPEED_OUT_OF_RANGE");
}

// 测试目的：验证 UserControlRequest.RejectsManualAngularSpeedOutsideConfiguredLimit 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, RejectsManualAngularSpeedOutsideConfiguredLimit) {
  const auto result = build_manual_command(
      "service", 0, -15001, false, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_Z_SPEED_OUT_OF_RANGE");
}

// 测试目的：验证 UserControlRequest.BrakingManualCommandZerosMotion 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, BrakingManualCommandZerosMotion) {
  const auto result = build_manual_command(
      "service", 100, 200, true, 350, 15000);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.command.x_speed, 0);
  EXPECT_EQ(result.command.z_speed, 0);
  EXPECT_TRUE(result.command.brake);
  EXPECT_TRUE(result.command.active);
  EXPECT_TRUE(result.command.operator_intent);
}

// 测试目的：验证 UserControlRequest.MovingManualCommandPreservesValidatedMotion 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, MovingManualCommandPreservesValidatedMotion) {
  const auto result = build_manual_command(
      "service", -120, 450, false, 350, 15000);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.command.source, "service");
  EXPECT_EQ(result.command.x_speed, -120);
  EXPECT_EQ(result.command.z_speed, 450);
  EXPECT_FALSE(result.command.brake);
}

// 测试目的：验证 UserControlRequest.RejectsEmergencyWithoutReason 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, RejectsEmergencyWithoutReason) {
  const auto result = build_emergency_command("operator", "");

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "EMERGENCY_REASON_REQUIRED");
}

// 测试目的：验证 UserControlRequest.EmergencyCommandIsActiveBrake 场景的行为、状态变化和边界条件。
TEST(UserControlRequest, EmergencyCommandIsActiveBrake) {
  const auto result = build_emergency_command("operator", "obstacle");

  ASSERT_TRUE(result.accepted);
  EXPECT_TRUE(result.command.active);
  EXPECT_TRUE(result.command.operator_intent);
  EXPECT_TRUE(result.command.brake);
  EXPECT_EQ(result.command.x_speed, 0);
  EXPECT_EQ(result.command.z_speed, 0);
  EXPECT_EQ(result.command.source, "operator");
}
