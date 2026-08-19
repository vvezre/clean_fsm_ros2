/*
 * 文件作用：验证23字节旧状态帧在真实串口字节流中的重组和重新同步能力。
 *
 * 覆盖分包、粘包、前导噪声、载荷内0x7D、类似ACK的无关字节和容量上限，
 * 防止串口读取边界变化导致丢帧、错帧或缓存无限增长。
 */
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/frame_buffer.hpp"

namespace {

using cleanbot::hardware::FrameBuffer;

// 一帧被分成两次async_read_some返回时，第一段等待，第二段到达后完整输出。
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

// 帧前噪声必须丢弃，但从下一处0x7B开始的有效23字节需要保留。
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

// 载荷内部出现0x7D不能提前截断，帧尾只认固定第22字节。
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

// 一次读取包含两帧时应连续弹出两次，且不留下多余字节。
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

// 类似新版ACK的噪声序列不属于旧协议，只从其后的0x7B提取23字节状态。
TEST(FrameBuffer, IgnoresAckLikeBytesAndExtractsOnlyLegacyStatusFrames) {
  FrameBuffer buffer;
  std::vector<std::uint8_t> frame;
  buffer.append({
      0x55, 0x7c, 0xa1, 0x00, 0x19, 0x03, 0x00, 0xc7, 0x7e,
      0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7d});

  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 23u);
  EXPECT_EQ(frame[0], 0x7b);
  EXPECT_FALSE(buffer.pop(frame));
}

// 长时间只有无效数据时缓存仍不得超过构造容量。
TEST(FrameBuffer, KeepsMemoryBoundedWhenNoValidFrameArrives) {
  FrameBuffer buffer(32u);
  buffer.append(std::vector<std::uint8_t>(100u, 0x55));
  EXPECT_LE(buffer.size(), 32u);
}

}  // namespace
