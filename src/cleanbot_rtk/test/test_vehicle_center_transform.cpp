// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <limits>

#include <gtest/gtest.h>

#include "cleanbot_rtk/vehicle_center_transform.hpp"

namespace {

using cleanbot::rtk::VehicleCenterTransform;

// 测试目的：验证 VehicleCenterTransform.MatchesPythonGoldenPointAtNorthHeading 场景的行为、状态变化和边界条件。
TEST(VehicleCenterTransform, MatchesPythonGoldenPointAtNorthHeading) {
  const auto result = VehicleCenterTransform().compute(
      12.34567890, 98.76543210, 0.0, 0.10, 0.18);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.lat, 12.34567980, 1e-8);
  EXPECT_NEAR(result.lon, 98.76543376, 1e-8);
}

// 测试目的：验证 VehicleCenterTransform.RejectsInvalidHeading 场景的行为、状态变化和边界条件。
TEST(VehicleCenterTransform, RejectsInvalidHeading) {
  const auto result = VehicleCenterTransform().compute(
      12.0, 98.0, std::numeric_limits<double>::quiet_NaN(), 0.10, 0.18);
  EXPECT_FALSE(result.valid);
}

}  // namespace
