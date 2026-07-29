#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/write_queue.hpp"

namespace {

using cleanbot::hardware::PriorityWriteQueue;
using cleanbot::hardware::WritePriority;

TEST(PriorityWriteQueue, CoalescesContinuousFrames) {
  PriorityWriteQueue queue(8u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kContinuous, "motion", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kContinuous, "motion", false));
  ASSERT_EQ(queue.size(), 1u);
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{2u}));
}

TEST(PriorityWriteQueue, PreservesActiveFrameAndPrioritizesAckAndSafety) {
  PriorityWriteQueue queue(8u);
  queue.enqueue({1u}, WritePriority::kContinuous, "motion", false);
  queue.enqueue({2u}, WritePriority::kFinite, "", true);
  queue.enqueue({3u}, WritePriority::kSafety, "brake", true);
  queue.enqueue({4u}, WritePriority::kProtocolAck, "ack-4", true);

  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{1u}));
  queue.pop_front();
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{4u}));
  queue.pop_front();
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{3u}));
}

TEST(PriorityWriteQueue, ActiveFrameStorageRemainsStableDuringInsertion) {
  PriorityWriteQueue queue(8u);
  queue.enqueue({1u, 2u, 3u}, WritePriority::kContinuous, "motion", false);
  const auto* active_data = queue.front().data();
  queue.enqueue({4u}, WritePriority::kFinite, "", true);
  queue.enqueue({5u}, WritePriority::kSafety, "brake", true);
  queue.enqueue({6u}, WritePriority::kProtocolAck, "ack", true);
  EXPECT_EQ(queue.front().data(), active_data);
}

TEST(PriorityWriteQueue, FullHighPriorityQueueRejectsWithoutGrowing) {
  PriorityWriteQueue queue(2u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kSafety, "brake-1", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kProtocolAck, "ack-2", false));

  EXPECT_FALSE(queue.enqueue({3u}, WritePriority::kProtocolAck, "ack-3", false));
  EXPECT_EQ(queue.size(), 2u);
}

TEST(PriorityWriteQueue, LowerPriorityFrameCannotEvictFiniteCommand) {
  PriorityWriteQueue queue(2u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kFinite, "", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kFinite, "", false));

  EXPECT_FALSE(queue.enqueue({3u}, WritePriority::kContinuous, "", false));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{1u}));
}

TEST(PriorityWriteQueue, CoalescingNotifiesSupersededFrame) {
  PriorityWriteQueue queue(2u);
  bool superseded = false;
  EXPECT_TRUE(queue.enqueue(
      {1u},
      WritePriority::kContinuous,
      "motion",
      false,
      [&superseded](const cleanbot::hardware::WriteQueueEvent event) {
        superseded = event == cleanbot::hardware::WriteQueueEvent::kSuperseded;
      }));

  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kContinuous, "motion", false));
  EXPECT_TRUE(superseded);
}

TEST(PriorityWriteQueue, ReplacesUnsentBrakeWithoutTouchingActiveFrame) {
  PriorityWriteQueue queue(3u);
  queue.enqueue({1u}, WritePriority::kContinuous, "motion", false);
  queue.enqueue({2u}, WritePriority::kSafety, "brake", true);
  queue.enqueue({3u}, WritePriority::kSafety, "brake", true);

  ASSERT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{1u}));
  queue.pop_front();
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{3u}));
}

}  // namespace
