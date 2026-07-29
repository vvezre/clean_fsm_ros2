#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/ack_codec.hpp"

namespace {

using cleanbot::hardware::AckCodec;

TEST(AckCodec, ParsesCommandAck) {
  const std::vector<std::uint8_t> frame{0x7c, 0xa1, 0x00, 0x19, 0x03, 0x00, 0xc7, 0x7e};
  const auto result = AckCodec().parse(frame);

  ASSERT_TRUE(result.parsed);
  EXPECT_EQ(result.ack.sequence, 25u);
  EXPECT_EQ(result.ack.subject_type, 3u);
  EXPECT_EQ(result.ack.result, 0u);
}

TEST(AckCodec, EncodesCompletionAck) {
  const auto frame = AckCodec().encode_completion_ack(25u, 2u);
  const std::vector<std::uint8_t> expected{0x7c, 0xa2, 0x00, 0x19, 0x02, 0x00, 0xc5, 0x7e};
  EXPECT_EQ(frame, expected);
}

TEST(AckCodec, RejectsInvalidChecksum) {
  const auto result = AckCodec().parse({0x7c, 0xa1, 0x00, 0x19, 0x03, 0x00, 0x00, 0x7e});
  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "ack_checksum_invalid");
}

}  // namespace
