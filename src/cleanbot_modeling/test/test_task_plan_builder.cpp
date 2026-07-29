#include <gtest/gtest.h>

#include "cleanbot_modeling/task_plan_builder.hpp"

namespace {

cleanbot::modeling::CleaningModel rectangle_model() {
  cleanbot::modeling::CleaningModel model;
  model.id = "model-1";
  model.name = "rectangle";
  model.version = 3u;
  model.origin_lat = 31.2;
  model.origin_lon = 121.5;
  model.origin_valid = true;
  model.recognition_confirmed = true;

  cleanbot::modeling::ModelGroup group;
  group.id = "group-1";
  group.sweep_mode = "manual";
  group.sweep_angle_deg = 0.0;
  const double coordinates[][2] = {
      {0.0, 0.0}, {500.0, 0.0}, {500.0, 1000.0}, {0.0, 1000.0}};
  cleanbot::modeling::ModelSubArea area;
  area.id = "sub-1";
  area.confirmed = true;
  for (std::size_t index = 0u; index < 4u; ++index) {
    cleanbot::modeling::ModelPoint point;
    point.id = "p" + std::to_string(index + 1u);
    point.sequence = index + 1u;
    point.x_cm = coordinates[index][0];
    point.y_cm = coordinates[index][1];
    point.lat = 31.2;
    point.lon = 121.5;
    group.points.push_back(point);
    area.point_ids.push_back(point.id);
  }
  group.sub_areas.push_back(area);
  model.groups.push_back(group);
  return model;
}

TEST(TaskPlanBuilder, BuildsEvenSnakePlan) {
  const auto result =
      cleanbot::modeling::build_task_plan(
          rectangle_model(), 116.0, 10.0, 350);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.plan.cleaning_lane_count, 6u);
  EXPECT_EQ(result.plan.transfer_segment_count, 5u);
  EXPECT_EQ(result.plan.segments.size(), 11u);
}

TEST(TaskPlanBuilder, RejectsUnconfirmedModel) {
  auto model = rectangle_model();
  model.recognition_confirmed = false;

  const auto result =
      cleanbot::modeling::build_task_plan(model, 116.0, 10.0, 350);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "MODEL_NOT_CONFIRMED");
}

}  // namespace
