// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_rtk/rtcm3_frame_buffer.hpp"

namespace {

using cleanbot::rtk::Rtcm3FrameBuffer;

// 辅助函数作用：为测试场景提供 crc24q 所需的准备、执行或清理逻辑。
std::uint32_t crc24q(const std::vector<std::uint8_t>& bytes) {
  std::uint32_t crc = 0u;
  for (const auto value : bytes) {
    crc ^= static_cast<std::uint32_t>(value) << 16u;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1u;
      if ((crc & 0x1000000u) != 0u) {
        crc ^= 0x1864cfbu;
      }
    }
  }
  return crc & 0x00ffffffu;
}

// 辅助函数作用：通过 makeFrame 构造当前测试所需的输入数据。
std::vector<std::uint8_t> makeFrame(const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> frame{
      0xd3u,
      static_cast<std::uint8_t>((payload.size() >> 8u) & 0x03u),
      static_cast<std::uint8_t>(payload.size() & 0xffu)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const auto crc = crc24q(frame);
  frame.push_back(static_cast<std::uint8_t>((crc >> 16u) & 0xffu));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8u) & 0xffu));
  frame.push_back(static_cast<std::uint8_t>(crc & 0xffu));
  return frame;
}

// 测试目的：验证 Rtcm3FrameBuffer.ReassemblesSplitAndStickyFrames 场景的行为、状态变化和边界条件。
TEST(Rtcm3FrameBuffer, ReassemblesSplitAndStickyFrames) {
  Rtcm3FrameBuffer buffer;
  const auto first = makeFrame({1u, 2u, 3u});
  const auto second = makeFrame({4u, 5u});
  buffer.append(std::vector<std::uint8_t>(first.begin(), first.begin() + 4));
  std::vector<std::uint8_t> output;
  EXPECT_FALSE(buffer.pop(output));
  std::vector<std::uint8_t> tail(first.begin() + 4, first.end());
  tail.insert(tail.end(), second.begin(), second.end());
  buffer.append(tail);
  EXPECT_TRUE(buffer.pop(output));
  EXPECT_EQ(output, first);
  EXPECT_TRUE(buffer.pop(output));
  EXPECT_EQ(output, second);
}

}  // namespace
