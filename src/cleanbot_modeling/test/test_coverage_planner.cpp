#include <gtest/gtest.h>

#include "cleanbot_modeling/coverage_planner.hpp"

namespace {

std::vector<cleanbot::modeling::Point2d> rectangle(
    const double width,
    const double height) {
  return {
      {0.0, 0.0},
      {width, 0.0},
      {width, height},
      {0.0, height},
  };
}

TEST(CoveragePlanner, AddsALaneAndRecomputesSpacingWhenNaturalCountIsOdd) {
  const auto result =
      cleanbot::modeling::plan_coverage(
          rectangle(500.0, 1000.0), 0.0, 116.0, 10.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.natural_lane_count, 5u);
  EXPECT_EQ(result.lanes.size(), 6u);
  EXPECT_DOUBLE_EQ(result.lane_spacing_cm, 76.8);
  EXPECT_DOUBLE_EQ(result.actual_overlap_cm, 39.2);
}

TEST(CoveragePlanner, ProducesTwoDistinctLanesForNarrowArea) {
  const auto result =
      cleanbot::modeling::plan_coverage(
          rectangle(80.0, 500.0), 0.0, 116.0, 10.0);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.lanes.size(), 2u);
  EXPECT_NE(result.lanes[0].offset_cm, result.lanes[1].offset_cm);
}

}  // namespace
