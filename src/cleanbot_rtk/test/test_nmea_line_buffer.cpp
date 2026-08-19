// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_rtk/nmea_line_buffer.hpp"

namespace {

using cleanbot::rtk::NmeaLineBuffer;

// 测试目的：验证 NmeaLineBuffer.ReassemblesSplitLine 场景的行为、状态变化和边界条件。
TEST(NmeaLineBuffer, ReassemblesSplitLine) {
  NmeaLineBuffer buffer;
  std::string line;
  buffer.append({'$', 'G', 'N', 'G'});
  EXPECT_FALSE(buffer.pop(line));
  buffer.append({'G', 'A', ',', '1', '\r', '\n'});
  ASSERT_TRUE(buffer.pop(line));
  EXPECT_EQ(line, "$GNGGA,1");
}

// 测试目的：验证 NmeaLineBuffer.ExtractsStickyLinesAndDropsNoise 场景的行为、状态变化和边界条件。
TEST(NmeaLineBuffer, ExtractsStickyLinesAndDropsNoise) {
  NmeaLineBuffer buffer;
  std::string line;
  const std::string input = "noise$GNGGA,1\n$GPHPR,2\r\n";
  buffer.append(std::vector<std::uint8_t>(input.begin(), input.end()));
  ASSERT_TRUE(buffer.pop(line));
  EXPECT_EQ(line, "$GNGGA,1");
  ASSERT_TRUE(buffer.pop(line));
  EXPECT_EQ(line, "$GPHPR,2");
}

// 测试目的：验证 NmeaLineBuffer.KeepsMemoryBounded 场景的行为、状态变化和边界条件。
TEST(NmeaLineBuffer, KeepsMemoryBounded) {
  NmeaLineBuffer buffer(32u);
  buffer.append(std::vector<std::uint8_t>(100u, 'x'));
  EXPECT_LE(buffer.size(), 32u);
}

}  // namespace
