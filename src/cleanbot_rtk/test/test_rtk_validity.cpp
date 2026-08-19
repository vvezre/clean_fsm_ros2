// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_rtk/rtk_validity.hpp"

namespace {

// 测试目的：验证 RtkValidity.RequiresConnectedFreshFixedVehicleCenter 场景的行为、状态变化和边界条件。
TEST(RtkValidity, RequiresConnectedFreshFixedVehicleCenter) {
  cleanbot::rtk::RtkValidityInput input;
  input.serial_connected = true;
  input.coordinate_valid = true;
  input.center_valid = true;
  input.heading_valid = true;
  input.fix_quality = 4u;
  input.gga_age_sec = 0.2;
  input.max_gga_age_sec = 2.0;

  EXPECT_TRUE(cleanbot::rtk::is_fixed_valid(input));

  input.gga_age_sec = 2.1;
  EXPECT_FALSE(cleanbot::rtk::is_fixed_valid(input));
}

// 测试目的：验证 RtkValidity.HeartbeatOnlyPublishesMissingDisconnectedOrStaleState 场景的行为、状态变化和边界条件。
TEST(RtkValidity, HeartbeatOnlyPublishesMissingDisconnectedOrStaleState) {
  using cleanbot::rtk::should_publish_freshness_heartbeat;

  EXPECT_TRUE(should_publish_freshness_heartbeat(false, true, -1.0, 2.0));
  EXPECT_TRUE(should_publish_freshness_heartbeat(true, false, 0.1, 2.0));
  EXPECT_FALSE(should_publish_freshness_heartbeat(true, true, 0.1, 2.0));
  EXPECT_TRUE(should_publish_freshness_heartbeat(true, true, 2.1, 2.0));
}

}  // namespace
