/*
 * 文件作用：验证异步串口发送队列的合并、优先级、容量和内存稳定性。
 *
 * 队列不解释下位机ACK，只保护本机待写批次：高频运动只保留最新值，正在写入的
 * 队首不可移动，安全/关键命令不能被低优先级命令挤出。
 */
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/write_queue.hpp"

namespace {

using cleanbot::hardware::PriorityWriteQueue;
using cleanbot::hardware::WritePriority;

// 相同motion合并键只保留尚未发送的最新连续速度命令。
TEST(PriorityWriteQueue, CoalescesContinuousFrames) {
  PriorityWriteQueue queue(8u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kContinuous, "motion", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kContinuous, "motion", false));
  ASSERT_EQ(queue.size(), 1u);
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{2u}));
}

// 历史测试名中的Ack指有限高优先级项；队列本身不处理旧协议中不存在的ACK。
// preserve_front保护正在写入的队首，写完后按Critical、Safety顺序发送。
TEST(PriorityWriteQueue, PreservesActiveFrameAndPrioritizesAckAndSafety) {
  PriorityWriteQueue queue(8u);
  queue.enqueue({1u}, WritePriority::kContinuous, "motion", false);
  queue.enqueue({2u}, WritePriority::kFinite, "", true);
  queue.enqueue({3u}, WritePriority::kSafety, "brake", true);
  queue.enqueue({4u}, WritePriority::kCritical, "critical-4", true);

  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{1u}));
  queue.pop_front();
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{4u}));
  queue.pop_front();
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{3u}));
}

// 插入新项不得移动正在被Boost.Asio引用的队首字节内存。
TEST(PriorityWriteQueue, ActiveFrameStorageRemainsStableDuringInsertion) {
  PriorityWriteQueue queue(8u);
  queue.enqueue({1u, 2u, 3u}, WritePriority::kContinuous, "motion", false);
  const auto* active_data = queue.front().data();
  queue.enqueue({4u}, WritePriority::kFinite, "", true);
  queue.enqueue({5u}, WritePriority::kSafety, "brake", true);
  queue.enqueue({6u}, WritePriority::kCritical, "critical", true);
  EXPECT_EQ(queue.front().data(), active_data);
}

// 队列已被Safety/Critical占满时，新Critical不能越过硬容量上限。
TEST(PriorityWriteQueue, FullHighPriorityQueueRejectsWithoutGrowing) {
  PriorityWriteQueue queue(2u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kSafety, "brake-1", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kCritical, "critical-2", false));

  EXPECT_FALSE(queue.enqueue({3u}, WritePriority::kCritical, "critical-3", false));
  EXPECT_EQ(queue.size(), 2u);
}

// 连续速度命令不得淘汰已排队的定距、转向等有限动作。
TEST(PriorityWriteQueue, LowerPriorityFrameCannotEvictFiniteCommand) {
  PriorityWriteQueue queue(2u);
  EXPECT_TRUE(queue.enqueue({1u}, WritePriority::kFinite, "", false));
  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kFinite, "", false));

  EXPECT_FALSE(queue.enqueue({3u}, WritePriority::kContinuous, "", false));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.front(), (std::vector<std::uint8_t>{1u}));
}

// motion被新值替换时必须通知旧项，供节点上报命令被覆盖。
TEST(PriorityWriteQueue, CoalescingNotifiesSupersededFrame) {
  PriorityWriteQueue queue(2u);
  bool superseded = false;
  EXPECT_TRUE(queue.enqueue(
      {1u},
      WritePriority::kContinuous,
      "motion",
      false,
      // 测试回调作用：统计被新命令替换的队列事件，验证合并规则。
      [&superseded](const cleanbot::hardware::WriteQueueEvent event) {
        superseded = event == cleanbot::hardware::WriteQueueEvent::kSuperseded;
      }));

  EXPECT_TRUE(queue.enqueue({2u}, WritePriority::kContinuous, "motion", false));
  EXPECT_TRUE(superseded);
}

// 新刹车可以替换尚未发送的旧刹车，但不能改动当前正在写入的队首。
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
