// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_gateway/cloud_command.hpp"

namespace {

using cleanbot::gateway::CloudCommandInput;
using cleanbot::gateway::CloudCommandKind;
using cleanbot::gateway::CloudCommandParameters;
using cleanbot::gateway::CloudCommandTranslator;

// 辅助函数作用：为测试场景提供 joystick 所需的准备、执行或清理逻辑。
CloudCommandInput joystick(const std::int64_t timestamp) {
  CloudCommandInput input;
  input.command_id = "cmd_1";
  input.command = "joystick_move";
  input.timestamp_sec = timestamp;
  input.distance = 50.0;
  input.dir_y = 1.0;
  input.has_joystick_params = true;
  return input;
}

// 测试目的：验证 CloudCommandTranslator.MapsJoystickWithConfiguredMaximumSpeed 场景的行为、状态变化和边界条件。
TEST(CloudCommandTranslator, MapsJoystickWithConfiguredMaximumSpeed) {
  CloudCommandParameters parameters;
  parameters.joystick.max_linear_speed = 600;
  CloudCommandTranslator translator(parameters);

  const auto result = translator.translate(joystick(100), 100);

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.kind, CloudCommandKind::kManualJoystick);
  EXPECT_EQ(result.x_speed, 600);
  EXPECT_FALSE(result.brake);
}

// 测试目的：验证 CloudCommandTranslator.ReleaseProducesManualBrake 场景的行为、状态变化和边界条件。
TEST(CloudCommandTranslator, ReleaseProducesManualBrake) {
  CloudCommandTranslator translator;
  auto input = joystick(100);
  input.distance = 0.0;
  input.dir_y = 0.0;

  const auto result = translator.translate(input, 100);

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.kind, CloudCommandKind::kManualJoystick);
  EXPECT_TRUE(result.brake);
  EXPECT_EQ(result.x_speed, 0);
}

// 测试目的：验证 CloudCommandTranslator.RejectsRetainedAndExpiredJoystickCommands 场景的行为、状态变化和边界条件。
TEST(CloudCommandTranslator, RejectsRetainedAndExpiredJoystickCommands) {
  CloudCommandTranslator translator;
  auto retained = joystick(100);
  retained.retained = true;
  EXPECT_EQ(
      translator.translate(retained, 100).code,
      "JOYSTICK_RETAINED_REJECTED");

  const auto expired = translator.translate(joystick(90), 100);
  EXPECT_FALSE(expired.accepted);
  EXPECT_EQ(expired.code, "JOYSTICK_COMMAND_EXPIRED");
}

// 测试目的：验证 CloudCommandTranslator.RejectsMissingJoystickParameters 场景的行为、状态变化和边界条件。
TEST(CloudCommandTranslator, RejectsMissingJoystickParameters) {
  CloudCommandTranslator translator;
  auto input = joystick(100);
  input.has_joystick_params = false;

  const auto result = translator.translate(input, 100);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "JOYSTICK_PARAMS_INVALID");
}

// 测试目的：验证 CloudCommandTranslator.ParkingAndStopAreSoftwareEmergencyStops 场景的行为、状态变化和边界条件。
TEST(CloudCommandTranslator, ParkingAndStopAreSoftwareEmergencyStops) {
  CloudCommandTranslator translator;
  CloudCommandInput input;
  input.command = "parking";

  const auto parking = translator.translate(input, 100);
  EXPECT_TRUE(parking.accepted);
  EXPECT_EQ(parking.kind, CloudCommandKind::kEmergencyStop);
  EXPECT_TRUE(parking.brake);

  input.command = "stop";
  EXPECT_EQ(
      translator.translate(input, 100).kind,
      CloudCommandKind::kEmergencyStop);
}

}  // namespace
