// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_control/tracking_core.hpp"

// 测试目的：验证 TrackingCore.WrapsHeadingAndFinishesAfterOvershootNearLine 场景的行为、状态变化和边界条件。
TEST(TrackingCore, WrapsHeadingAndFinishesAfterOvershootNearLine) {
  EXPECT_DOUBLE_EQ(cleanbot::control::normalize_heading_delta(1.0, 359.0), 2.0);
  EXPECT_TRUE(cleanbot::control::should_finish_point_to_point(
      1.0, -0.1, 0.2, 0.03, 0.30));
}

// 测试目的：验证 TrackingCore.KalmanInitializesThenFilters 场景的行为、状态变化和边界条件。
TEST(TrackingCore, KalmanInitializesThenFilters) {
  cleanbot::control::RtkKalmanFilter2D filter;
  EXPECT_FALSE(filter.update(32.0, 118.0, 1.0).filtered);
  EXPECT_TRUE(filter.update(32.0, 118.00001, 1.1).filtered);
}
