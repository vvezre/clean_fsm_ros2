#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/status_frame_parser.hpp"

namespace {

using cleanbot::hardware::StatusFrameParser;

TEST(StatusFrameParser, ParsesFullStatusUsingLegacyBigEndianFields) {
  const std::vector<std::uint8_t> frame = {
      0x7b, 0x01, 0x01, 0x02, 0x01, 0x2c, 0xff, 0x9c, 0x32, 0x00, 0x64,
      0x01, 0xbb, 0x00, 0x14, 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x12, 0x34, 0x7d};

  const auto result = StatusFrameParser().parse(frame);

  ASSERT_TRUE(result.parsed);
  EXPECT_TRUE(result.status.full_frame);
  EXPECT_EQ(result.status.status, 1);
  EXPECT_EQ(result.status.power_on, 1);
  EXPECT_EQ(result.status.hardware_state, 2);
  EXPECT_EQ(result.status.x_speed, 300);
  EXPECT_EQ(result.status.z_speed, -100);
  EXPECT_EQ(result.status.brush_speed, 50);
  EXPECT_TRUE(result.status.edge_clear);
  EXPECT_DOUBLE_EQ(result.status.battery_percent, 100.0);
  EXPECT_EQ(result.status.air_state, 1);
  EXPECT_TRUE(result.status.move_finished);
  EXPECT_FALSE(result.status.rotate_finished);
  EXPECT_DOUBLE_EQ(result.status.pack_voltage, 51.5);
  EXPECT_EQ(result.status.angle, 90);
  EXPECT_EQ(result.status.odometer, 1234);
  EXPECT_EQ(result.status.command_sequence, 0x1234);
}

TEST(StatusFrameParser, RejectsFormerShortFinishFrame) {
  const std::vector<std::uint8_t> frame = {
      0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0x00};

  const auto result = StatusFrameParser().parse(frame);

  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "frame_length_invalid");
}

TEST(StatusFrameParser, RejectsUnsynchronizedAndTooShortFrames) {
  const auto unsynchronized = StatusFrameParser().parse({0x00, 0x01, 0x02, 0x03});
  const auto too_short = StatusFrameParser().parse({0x7b, 0x01, 0x02});

  EXPECT_FALSE(unsynchronized.parsed);
  EXPECT_EQ(unsynchronized.error, "frame_start_missing");
  EXPECT_FALSE(too_short.parsed);
  EXPECT_EQ(too_short.error, "frame_length_invalid");
}

TEST(StatusFrameParser, RejectsAFrameWhoseFixedTailIsInvalid) {
  std::vector<std::uint8_t> frame = {
      0x7b, 0x01, 0x01, 0x02, 0x01, 0x2c, 0xff, 0x9c, 0x32, 0x00, 0x64,
      0x01, 0xbb, 0x00, 0x14, 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x00, 0x00, 0x00};

  const auto result = StatusFrameParser().parse(frame);

  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "frame_end_missing");
}

}  // namespace
