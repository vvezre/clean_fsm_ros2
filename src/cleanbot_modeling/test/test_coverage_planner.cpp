// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_modeling/coverage_planner.hpp"

namespace {

// 辅助函数作用：为测试场景提供 rectangle 所需的准备、执行或清理逻辑。
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

// 测试目的：验证 CoveragePlanner.AddsALaneAndRecomputesSpacingWhenNaturalCountIsOdd 场景的行为、状态变化和边界条件。
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

// 测试目的：验证 CoveragePlanner.ProducesTwoDistinctLanesForNarrowArea 场景的行为、状态变化和边界条件。
TEST(CoveragePlanner, ProducesTwoDistinctLanesForNarrowArea) {
  const auto result =
      cleanbot::modeling::plan_coverage(
          rectangle(80.0, 500.0), 0.0, 116.0, 10.0);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.lanes.size(), 2u);
  EXPECT_NE(result.lanes[0].offset_cm, result.lanes[1].offset_cm);
}

}  // namespace
