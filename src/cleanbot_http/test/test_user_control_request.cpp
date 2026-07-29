#include "cleanbot_http/user_control_request.hpp"

#include <gtest/gtest.h>

using cleanbot::http::build_emergency_command;
using cleanbot::http::build_manual_command;

TEST(UserControlRequest, RejectsEmptyManualSource) {
  const auto result = build_manual_command("", 0, 0, true, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_SOURCE_REQUIRED");
}

TEST(UserControlRequest, RejectsManualLinearSpeedOutsideConfiguredLimit) {
  const auto result = build_manual_command(
      "service", 351, 0, false, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_X_SPEED_OUT_OF_RANGE");
}

TEST(UserControlRequest, RejectsManualAngularSpeedOutsideConfiguredLimit) {
  const auto result = build_manual_command(
      "service", 0, -15001, false, 350, 15000);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "MANUAL_Z_SPEED_OUT_OF_RANGE");
}

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

TEST(UserControlRequest, MovingManualCommandPreservesValidatedMotion) {
  const auto result = build_manual_command(
      "service", -120, 450, false, 350, 15000);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.command.source, "service");
  EXPECT_EQ(result.command.x_speed, -120);
  EXPECT_EQ(result.command.z_speed, 450);
  EXPECT_FALSE(result.command.brake);
}

TEST(UserControlRequest, RejectsEmergencyWithoutReason) {
  const auto result = build_emergency_command("operator", "");

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "EMERGENCY_REASON_REQUIRED");
}

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
