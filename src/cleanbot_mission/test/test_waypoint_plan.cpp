#include <vector>

#include "cleanbot_mission/waypoint_plan.hpp"
#include "gtest/gtest.h"

namespace cleanbot {
namespace mission {

TEST(WaypointPlanTest, RejectsCoordinatesOutsideLatitudeLongitudeRange) {
  EXPECT_FALSE(validate_waypoint(91.0, 118.0));
  EXPECT_FALSE(validate_waypoint(32.0, -181.0));
  EXPECT_TRUE(validate_waypoint(32.0, 118.0));
}

TEST(WaypointPlanTest, SinglePassVisitsEveryWaypointOnce) {
  WaypointSequence sequence(3u, false, 0u);
  std::vector<std::size_t> indexes;
  WaypointTarget target;

  while (sequence.next(target)) {
    indexes.push_back(target.waypoint_index);
  }

  EXPECT_EQ(indexes, (std::vector<std::size_t>{0u, 1u, 2u}));
}

TEST(WaypointPlanTest, FiniteLoopApproachesFirstPointThenClosesEveryLoop) {
  WaypointSequence sequence(3u, true, 2u);
  std::vector<std::size_t> indexes;
  std::vector<std::uint32_t> completed_loops;
  WaypointTarget target;

  while (sequence.next(target)) {
    indexes.push_back(target.waypoint_index);
    completed_loops.push_back(target.completed_loop);
  }

  EXPECT_EQ(
      indexes,
      (std::vector<std::size_t>{0u, 1u, 2u, 0u, 1u, 2u, 0u}));
  EXPECT_EQ(
      completed_loops,
      (std::vector<std::uint32_t>{0u, 0u, 0u, 1u, 1u, 1u, 2u}));
}

TEST(WaypointPlanTest, ContinuousLoopDoesNotTerminateAtLoopBoundary) {
  WaypointSequence sequence(2u, true, 0u);
  std::vector<std::size_t> indexes;
  WaypointTarget target;

  for (int count = 0; count < 5; ++count) {
    ASSERT_TRUE(sequence.next(target));
    indexes.push_back(target.waypoint_index);
  }

  EXPECT_EQ(indexes, (std::vector<std::size_t>{0u, 1u, 0u, 1u, 0u}));
  EXPECT_EQ(target.completed_loop, 2u);
}

TEST(WaypointPlanTest, CalculatesCompassHeadingAndShortestTurn) {
  EXPECT_NEAR(calculate_heading_deg(32.0, 118.0, 32.0, 118.001), 90.0, 0.1);
  EXPECT_NEAR(calculate_heading_deg(32.0, 118.0, 32.001, 118.0), 0.0, 0.1);
  EXPECT_NEAR(normalize_turn_angle_deg(10.0, 350.0), 20.0, 1e-9);
  EXPECT_NEAR(normalize_turn_angle_deg(350.0, 10.0), -20.0, 1e-9);
}

TEST(WaypointPlanTest, BuildsSegmentFromCurrentPoseToTarget) {
  const auto segment = build_waypoint_segment(
      32.0, 118.0, 0.0, 32.0, 118.001);

  EXPECT_DOUBLE_EQ(segment.start_lat, 32.0);
  EXPECT_DOUBLE_EQ(segment.start_lon, 118.0);
  EXPECT_DOUBLE_EQ(segment.end_lat, 32.0);
  EXPECT_DOUBLE_EQ(segment.end_lon, 118.001);
  EXPECT_NEAR(segment.heading_deg, 90.0, 0.1);
  EXPECT_NEAR(segment.turn_angle_deg, 90.0, 0.1);
}

}  // namespace mission
}  // namespace cleanbot
