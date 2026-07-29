#include <gtest/gtest.h>

#include "cleanbot_hardware/command_encoder.hpp"

namespace {

using cleanbot::hardware::CommandEncoder;
using cleanbot::hardware::CommandFields;

TEST(CommandEncoder, EncodesSignedSpeedsAndLegacyFields) {
  CommandFields fields;
  fields.status = 1u;
  fields.x_speed = 300;
  fields.z_speed = -100;
  fields.brush_speed = 50;
  fields.target_distance = 90;
  fields.target_rotation = 1800;
  fields.heading_deg = 359.99;
  fields.sequence = 0x1234u;

  const auto frame = CommandEncoder().encode(fields);

  const std::vector<std::uint8_t> expected{
      0x7b, 0x01, 0x01, 0x00, 0x01, 0x2c, 0xff, 0x9c, 0x00, 0x5a,
      0x32, 0x00, 0x00, 0x07, 0x08, 0x8c, 0x9f, 0x12, 0x34, 0x67, 0x7d};
  EXPECT_EQ(frame, expected);
}

TEST(CommandEncoder, BrakeClearsMotionFields) {
  CommandFields fields;
  fields.status = 3u;
  fields.x_speed = 500;
  fields.z_speed = -500;
  fields.brush_speed = 80;
  fields.target_distance = 1000;
  fields.target_rotation = 900;
  fields.brake = true;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[1], 0u);
  for (std::size_t index = 4u; index <= 14u; ++index) {
    EXPECT_EQ(frame[index], 0u);
  }
}

TEST(CommandEncoder, EncodesNegativeRotationAsSignedTwosComplement) {
  CommandFields fields;
  fields.target_rotation = -900;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[13], 0xfcu);
  EXPECT_EQ(frame[14], 0x7cu);
}

TEST(CommandEncoder, EncodesManualSteeringAsSignedTwosComplement) {
  CommandFields fields;
  fields.steering_offset = -1000;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[11], 0xfcu);
  EXPECT_EQ(frame[12], 0x18u);

  fields.brake = true;
  const auto stopped = CommandEncoder().encode(fields);
  EXPECT_EQ(stopped[11], 0x00u);
  EXPECT_EQ(stopped[12], 0x00u);
}

}  // namespace
