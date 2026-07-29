#include <gtest/gtest.h>

#include "cleanbot_hardware/gateway_readiness.hpp"

namespace {

using cleanbot::hardware::GatewayReadiness;
using cleanbot::hardware::GatewayReadinessState;

TEST(GatewayReadiness, RequiresMatchingBrakeAckAndValidStatus) {
  GatewayReadiness readiness;
  readiness.on_connected(41u);
  EXPECT_FALSE(readiness.ready());
  EXPECT_FALSE(readiness.observe_command_ack(40u, 0u, true));
  EXPECT_TRUE(readiness.observe_command_ack(41u, 0u, true));
  EXPECT_FALSE(readiness.ready());
  EXPECT_TRUE(readiness.observe_valid_status());
  EXPECT_TRUE(readiness.ready());
}

TEST(GatewayReadiness, DisconnectResetsReadyState) {
  GatewayReadiness readiness;
  readiness.on_connected(7u);
  readiness.observe_valid_status();
  readiness.observe_command_ack(7u, 0u, true);
  ASSERT_TRUE(readiness.ready());
  readiness.on_disconnected();
  EXPECT_EQ(readiness.state(), GatewayReadinessState::kDisconnected);
}

}  // namespace
