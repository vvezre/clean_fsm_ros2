#include <gtest/gtest.h>

#include "cleanbot_gateway/cloud_command.hpp"

namespace {

using cleanbot::gateway::CloudCommandInput;
using cleanbot::gateway::CloudCommandKind;
using cleanbot::gateway::CloudCommandParameters;
using cleanbot::gateway::CloudCommandTranslator;

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

TEST(CloudCommandTranslator, RejectsMissingJoystickParameters) {
  CloudCommandTranslator translator;
  auto input = joystick(100);
  input.has_joystick_params = false;

  const auto result = translator.translate(input, 100);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, "JOYSTICK_PARAMS_INVALID");
}

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
