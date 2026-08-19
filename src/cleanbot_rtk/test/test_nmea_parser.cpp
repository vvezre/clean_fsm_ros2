// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_rtk/nmea_parser.hpp"

namespace {

using cleanbot::rtk::NmeaParser;

// 测试目的：验证 NmeaParser.ParsesFixedGga 场景的行为、状态变化和边界条件。
TEST(NmeaParser, ParsesFixedGga) {
  const auto result = NmeaParser().parse_gga(
      "$GNGGA,123519.00,1220.74073400,N,09845.92592600,E,4,12,0.9,"
      "10.0,M,0.0,M,1.0,0001*5E");
  ASSERT_TRUE(result.parsed);
  EXPECT_NEAR(result.data.lat, 12.34567890, 1e-8);
  EXPECT_NEAR(result.data.lon, 98.76543210, 1e-8);
  EXPECT_EQ(result.data.fix_quality, 4u);
}

// 测试目的：验证 NmeaParser.ParsesHpr 场景的行为、状态变化和边界条件。
TEST(NmeaParser, ParsesHpr) {
  const auto result = NmeaParser().parse_heading(
      "$GNHPR,123519.10,90.00,00.00,000.00,4,12,0.00,0001*5E");
  ASSERT_TRUE(result.parsed);
  EXPECT_DOUBLE_EQ(result.data.heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(result.data.pitch_deg, 0.0);
}

// 测试目的：验证 NmeaParser.RejectsInvalidChecksum 场景的行为、状态变化和边界条件。
TEST(NmeaParser, RejectsInvalidChecksum) {
  const auto result = NmeaParser().parse_gga(
      "$GNGGA,123519.00,1220.7,N,09845.9,E,4,07,0.5,0,M,0,M,,*00");
  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "nmea_checksum_invalid");
}

}  // namespace
