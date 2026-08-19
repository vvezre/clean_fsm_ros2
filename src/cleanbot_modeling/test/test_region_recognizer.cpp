// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_modeling/region_recognizer.hpp"

namespace {

// 辅助函数作用：为测试场景提供 rectangle 所需的准备、执行或清理逻辑。
cleanbot::modeling::ModelGroup rectangle() {
  cleanbot::modeling::ModelGroup group;
  group.id = "g1";
  const double coordinates[][2] = {
      {0.0, 0.0}, {1000.0, 0.0}, {1000.0, 600.0}, {0.0, 600.0}};
  for (std::size_t index = 0u; index < 4u; ++index) {
    cleanbot::modeling::ModelPoint point;
    point.id = "p" + std::to_string(index + 1u);
    point.sequence = index + 1u;
    point.x_cm = coordinates[index][0];
    point.y_cm = coordinates[index][1];
    group.points.push_back(point);
  }
  return group;
}

// 测试目的：验证 RegionRecognizer.RecognizesRectangleAsSingleSubArea 场景的行为、状态变化和边界条件。
TEST(RegionRecognizer, RecognizesRectangleAsSingleSubArea) {
  const auto result = cleanbot::modeling::recognize_group(rectangle());

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, "recognized");
  ASSERT_EQ(result.group.sub_areas.size(), 1u);
  EXPECT_TRUE(result.group.sub_areas.front().confirmed);
}

// 测试目的：验证 RegionRecognizer.RecoversUnorderedConvexBoundary 场景的行为、状态变化和边界条件。
TEST(RegionRecognizer, RecoversUnorderedConvexBoundary) {
  auto group = rectangle();
  std::swap(group.points[1], group.points[2]);

  const auto result = cleanbot::modeling::recognize_group(group);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.status, "recognized");
}

}  // namespace
