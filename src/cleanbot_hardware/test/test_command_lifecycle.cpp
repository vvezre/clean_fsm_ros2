#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/command_lifecycle.hpp"

namespace {

using cleanbot::hardware::AckCodec;
using cleanbot::hardware::AckDisposition;
using cleanbot::hardware::AckFrame;
using cleanbot::hardware::CommandLifecycle;
using cleanbot::hardware::CommandDeliveryPolicy;
using cleanbot::hardware::CompletionDisposition;

TEST(CommandLifecycle, RetriesSameBytesThenStopsAfterMatchingAck) {
  CommandLifecycle lifecycle(200u, 3u, 64u);
  const std::vector<std::uint8_t> command{0x7b, 0x01, 0x7d};
  lifecycle.track(25u, 1u, command, 1000u);

  const auto retry = lifecycle.collect_due_retries(1200u);
  ASSERT_EQ(retry.frames.size(), 1u);
  EXPECT_EQ(retry.frames.front(), command);

  AckFrame ack;
  ack.frame_type = AckCodec::kCommandAck;
  ack.sequence = 25u;
  ack.subject_type = 1u;
  ack.result = 0u;
  EXPECT_EQ(lifecycle.handle_command_ack(ack), AckDisposition::kAccepted);
  EXPECT_TRUE(lifecycle.collect_due_retries(1400u).frames.empty());
}

TEST(CommandLifecycle, DeduplicatesCompletionBySequenceAndEvent) {
  CommandLifecycle lifecycle(200u, 3u, 64u);
  lifecycle.track(25u, 3u, {0x7b, 0x03, 0x7d}, 1000u);

  EXPECT_EQ(lifecycle.observe_completion(25u, 2u), CompletionDisposition::kFirstSeen);
  EXPECT_EQ(lifecycle.observe_completion(25u, 2u), CompletionDisposition::kDuplicate);
  EXPECT_EQ(lifecycle.observe_completion(99u, 2u), CompletionDisposition::kUnknownCommand);
}

TEST(CommandLifecycle, LatestOnlyCommandSupersedesOlderPendingCommand) {
  CommandLifecycle lifecycle(200u, 3u, 64u);
  lifecycle.track(
      10u, 1u, {0x7b, 0x01, 0x01, 0x7d}, 0u,
      CommandDeliveryPolicy::kLatestOnly);
  lifecycle.track(
      11u, 1u, {0x7b, 0x01, 0x02, 0x7d}, 100u,
      CommandDeliveryPolicy::kLatestOnly);

  const auto retry = lifecycle.collect_due_retries(300u);
  ASSERT_EQ(retry.frames.size(), 1u);
  EXPECT_EQ(retry.frames.front(), (std::vector<std::uint8_t>{0x7b, 0x01, 0x02, 0x7d}));

  AckFrame old_ack;
  old_ack.frame_type = AckCodec::kCommandAck;
  old_ack.sequence = 10u;
  old_ack.subject_type = 1u;
  old_ack.result = 0u;
  EXPECT_EQ(lifecycle.handle_command_ack(old_ack), AckDisposition::kUnmatched);
}

TEST(CommandLifecycle, TimeoutIncludesOriginalCommandType) {
  CommandLifecycle lifecycle(100u, 0u, 64u);
  lifecycle.track(9u, 3u, {0x7b, 0x03, 0x7d}, 0u);

  const auto timed_out = lifecycle.collect_due_retries(100u);
  ASSERT_EQ(timed_out.timed_out_commands.size(), 1u);
  EXPECT_EQ(timed_out.timed_out_commands.front().sequence, 9u);
  EXPECT_EQ(timed_out.timed_out_commands.front().command_type, 3u);
}

TEST(CommandLifecycle, ClearPendingReturnsInterruptedCommands) {
  CommandLifecycle lifecycle(100u, 3u, 64u);
  lifecycle.track(15u, 2u, {0x7b, 0x02, 0x7d}, 0u);

  const auto interrupted = lifecycle.clear_pending();

  ASSERT_EQ(interrupted.size(), 1u);
  EXPECT_EQ(interrupted.front().sequence, 15u);
  EXPECT_EQ(interrupted.front().command_type, 2u);
  EXPECT_TRUE(lifecycle.collect_due_retries(100u).frames.empty());
}

TEST(CommandLifecycle, CancelPendingStopsRetry) {
  CommandLifecycle lifecycle(100u, 3u, 64u);
  lifecycle.track(16u, 1u, {0x7b, 0x01, 0x7d}, 0u);

  EXPECT_TRUE(lifecycle.cancel_pending(16u));
  EXPECT_FALSE(lifecycle.cancel_pending(16u));
  EXPECT_TRUE(lifecycle.collect_due_retries(100u).frames.empty());
}

}  // namespace
