// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <limits>

#include "cleanbot_mission/straight_edge_guard.hpp"
#include "gtest/gtest.h"

namespace {

using cleanbot::mission::StraightEdgeDecision;
using cleanbot::mission::StraightEdgeGuard;

// 测试目的：验证 StraightEdgeGuard.IgnoresPulseShorterThanDebounce 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, IgnoresPulseShorterThanDebounce) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1199u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(true, 1200u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
}

// 测试目的：验证 StraightEdgeGuard.AcceptsEdgeInsideTargetTolerance 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, AcceptsEdgeInsideTargetTolerance) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.10, 0.05),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, 0.10, 0.05),
      StraightEdgeDecision::kTargetReached);
}

// 测试目的：验证 StraightEdgeGuard.AcceptsEdgeAfterPassingTargetProjection 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, AcceptsEdgeAfterPassingTargetProjection) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.40, -0.01),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, 0.40, -0.01),
      StraightEdgeDecision::kTargetReached);
}

// 测试目的：验证 StraightEdgeGuard.RequestsConfirmationBeforeTarget 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, RequestsConfirmationBeforeTarget) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, 0.50, 0.40),
      StraightEdgeDecision::kConfirmRequired);
  EXPECT_EQ(
      guard.evaluate(false, 1300u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
}

// 测试目的：验证 StraightEdgeGuard.ClearSignalRearmsAfterAConfirmedEdge 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, ClearSignalRearmsAfterAConfirmedEdge) {
  StraightEdgeGuard guard(200u, 0.10);

  guard.evaluate(false, 1000u, 0.50, 0.40);
  ASSERT_EQ(
      guard.evaluate(false, 1200u, 0.50, 0.40),
      StraightEdgeDecision::kConfirmRequired);
  EXPECT_EQ(
      guard.evaluate(true, 1250u, 0.50, 0.40),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1300u, 0.05, 0.03),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1500u, 0.05, 0.03),
      StraightEdgeDecision::kTargetReached);
}

// 测试目的：验证 StraightEdgeGuard.MissingTrackingMetricsRequireConfirmation 场景的行为、状态变化和边界条件。
TEST(StraightEdgeGuard, MissingTrackingMetricsRequireConfirmation) {
  StraightEdgeGuard guard(200u, 0.10);
  const double missing = std::numeric_limits<double>::quiet_NaN();

  guard.evaluate(false, 1000u, missing, missing);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, missing, missing),
      StraightEdgeDecision::kConfirmRequired);
}

}  // namespace
