#include <limits>

#include "cleanbot_mission/straight_edge_guard.hpp"
#include "gtest/gtest.h"

namespace {

using cleanbot::mission::StraightEdgeDecision;
using cleanbot::mission::StraightEdgeGuard;

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

TEST(StraightEdgeGuard, AcceptsEdgeInsideTargetTolerance) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.10, 0.05),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, 0.10, 0.05),
      StraightEdgeDecision::kTargetReached);
}

TEST(StraightEdgeGuard, AcceptsEdgeAfterPassingTargetProjection) {
  StraightEdgeGuard guard(200u, 0.10);

  EXPECT_EQ(
      guard.evaluate(false, 1000u, 0.40, -0.01),
      StraightEdgeDecision::kNone);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, 0.40, -0.01),
      StraightEdgeDecision::kTargetReached);
}

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

TEST(StraightEdgeGuard, MissingTrackingMetricsRequireConfirmation) {
  StraightEdgeGuard guard(200u, 0.10);
  const double missing = std::numeric_limits<double>::quiet_NaN();

  guard.evaluate(false, 1000u, missing, missing);
  EXPECT_EQ(
      guard.evaluate(false, 1200u, missing, missing),
      StraightEdgeDecision::kConfirmRequired);
}

}  // namespace
