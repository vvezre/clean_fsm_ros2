#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/frame_buffer.hpp"

namespace {

using cleanbot::hardware::FrameBuffer;

TEST(FrameBuffer, ReassemblesAFrameSplitAcrossSerialReads) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;

  buffer.append({0x7b, 0x01, 0x01, 0x01, 0x01, 0x2c, 0x00,
                 0x64, 0x32, 0x00, 0x64, 0x01, 0xbb, 0x00});
  EXPECT_FALSE(buffer.pop(frame));

  buffer.append({0x14, 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x12, 0x34, 0x7d});
  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 23u);
  EXPECT_EQ(frame.front(), 0x7b);
  EXPECT_EQ(frame.back(), 0x7d);
}

TEST(FrameBuffer, DiscardsNoiseBeforeTheNextFrameStart) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;

  buffer.append({0x00, 0xff, 0x55, 0x7b, 0x01, 0x01, 0x01, 0x00, 0x01,
                 0x00, 0x02, 0x10, 0x00, 0x64, 0x01, 0x00, 0xbb, 0x14,
                 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x00, 0x00, 0x7d});

  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame.front(), 0x7b);
  EXPECT_EQ(frame.back(), 0x7d);
  EXPECT_EQ(buffer.size(), 0u);
}

TEST(FrameBuffer, DoesNotTreatFrameEndValueInsidePayloadAsTheTail) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;
  buffer.append({0x7b, 0x01, 0x01, 0x01, 0x00, 0x7d, 0x00, 0x01,
                 0x20, 0x00, 0x64, 0x01, 0x00, 0x00, 0x14, 0x1e,
                 0x00, 0x5a, 0x00, 0x01, 0x00, 0x00, 0x7d});

  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame.size(), 23u);
  EXPECT_EQ(frame[5], 0x7d);
}

TEST(FrameBuffer, ExtractsBackToBackFixedLengthFrames) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;
  const std::vector<std::uint8_t> one_frame = {
      0x7b, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x02,
      0x10, 0x00, 0x64, 0x01, 0x00, 0x00, 0x14, 0x1e,
      0x00, 0x5a, 0x00, 0x01, 0x00, 0x00, 0x7d};
  std::vector<std::uint8_t> input = one_frame;
  input.insert(input.end(), one_frame.begin(), one_frame.end());
  buffer.append(input);

  EXPECT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame.size(), 23u);
  EXPECT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame.size(), 23u);
  EXPECT_FALSE(buffer.pop(frame));
}

TEST(FrameBuffer, ExtractsAckAndStatusFramesFromOneRead) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;
  buffer.append({
      0x55, 0x7c, 0xa1, 0x00, 0x19, 0x03, 0x00, 0xc7, 0x7e,
      0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7d});

  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 8u);
  EXPECT_EQ(frame[0], 0x7c);
  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 23u);
  EXPECT_EQ(frame[0], 0x7b);
}

TEST(FrameBuffer, KeepsMemoryBoundedWhenNoValidFrameArrives) {
  FrameBuffer buffer(32u);
  buffer.append(std::vector<std::uint8_t>(100u, 0x55));
  EXPECT_LE(buffer.size(), 32u);
}

}  // namespace
