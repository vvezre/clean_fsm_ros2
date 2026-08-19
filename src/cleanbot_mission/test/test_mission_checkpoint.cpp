// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "cleanbot_interfaces/msg/task_segment.hpp"
#include "cleanbot_mission/mission_checkpoint.hpp"
#include "cleanbot_mission/mission_state_machine.hpp"

using cleanbot::mission::MissionCheckpointRecord;
using cleanbot::mission::MissionCheckpointStore;
using cleanbot::mission::MissionState;
using cleanbot::mission::MissionStateMachine;

// 测试目的：验证 MissionStateMachine.StartFromCheckpointSegment 场景的行为、状态变化和边界条件。
TEST(MissionStateMachine, StartFromCheckpointSegment) {
  MissionStateMachine machine;
  ASSERT_TRUE(machine.start(5u, 2u));
  EXPECT_EQ(machine.current_segment(), 2u);
  EXPECT_EQ(machine.state(), MissionState::kPreparing);
  ASSERT_TRUE(machine.begin_tracking(11u));
  ASSERT_TRUE(machine.complete_tracking(11u));
  EXPECT_EQ(machine.current_segment(), 3u);
}

// 测试目的：验证 MissionCheckpointStore.SavesAndLoadsCheckpoint 场景的行为、状态变化和边界条件。
TEST(MissionCheckpointStore, SavesAndLoadsCheckpoint) {
  const auto path = std::filesystem::path(::testing::TempDir()) /
      "mission_checkpoint.json";
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::create_directories(path.parent_path(), ec);

  MissionCheckpointRecord record;
  record.run_id = "run-001";
  record.mission_kind = "cleaning";
  record.plan_id = "plan-001";
  record.plan_hash = "sha256:test";
  record.model_id = "model-001";
  record.model_version = 0x100000001ULL;
  record.loop = true;
  record.loop_count = 3u;
  record.next_segment = 0u;
  record.total_segments = 1u;
  record.completed_segments = 0u;
  record.current_loop = 1u;
  record.total_loops = 3u;
  record.brush_speed = 350;
  record.target_lat = 32.1;
  record.target_lon = 118.9;
  record.target_valid = true;

  cleanbot_interfaces::msg::TaskSegment segment;
  segment.index = 1u;
  segment.id = "segment-1";
  segment.segment_type = cleanbot_interfaces::msg::TaskSegment::SEGMENT_CLEANING;
  segment.group_id = "group-a";
  segment.sub_area_id = "sub-area-a";
  segment.source_lane_id = "lane-1";
  segment.start_lat = 32.0;
  segment.start_lon = 118.0;
  segment.end_lat = 32.01;
  segment.end_lon = 118.01;
  segment.heading_deg = 90.0;
  segment.turn_angle_deg = 15.0;
  segment.speed = 350;
  segment.mode = 1u;
  segment.turn_back_length = 0.0;
  segment.back_length = 0.0;
  record.segments.push_back(segment);

  MissionCheckpointStore store(path);
  ASSERT_TRUE(store.save(record));

  const auto loaded = store.load_latest();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->run_id, record.run_id);
  EXPECT_EQ(loaded->mission_kind, record.mission_kind);
  EXPECT_EQ(loaded->plan_id, record.plan_id);
  EXPECT_EQ(loaded->plan_hash, record.plan_hash);
  EXPECT_EQ(loaded->model_id, record.model_id);
  EXPECT_EQ(loaded->model_version, record.model_version);
  EXPECT_EQ(loaded->next_segment, record.next_segment);
  EXPECT_EQ(loaded->completed_segments, record.completed_segments);
  EXPECT_EQ(loaded->current_loop, record.current_loop);
  EXPECT_EQ(loaded->total_loops, record.total_loops);
  EXPECT_EQ(loaded->brush_speed, record.brush_speed);
  ASSERT_EQ(loaded->segments.size(), 1u);
  EXPECT_EQ(loaded->segments.front().id, segment.id);
  EXPECT_DOUBLE_EQ(loaded->segments.front().start_lat, segment.start_lat);
  EXPECT_DOUBLE_EQ(loaded->segments.front().end_lon, segment.end_lon);
}

// 测试目的：验证 MissionCheckpointStore.RejectsMalformedCheckpointWithoutThrowing 场景的行为、状态变化和边界条件。
TEST(MissionCheckpointStore, RejectsMalformedCheckpointWithoutThrowing) {
  const auto path = std::filesystem::path(::testing::TempDir()) /
      "malformed_mission_checkpoint.json";
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::create_directories(path.parent_path(), ec);

  {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << R"({"schemaVersion":1,"checkpoint":{"runId":123}})";
  }

  MissionCheckpointStore store(path);
  std::string error;
  EXPECT_NO_THROW({
    const auto loaded = store.load_latest(&error);
    EXPECT_FALSE(loaded.has_value());
  });
  EXPECT_FALSE(error.empty());
}

// 测试目的：验证 MissionCheckpointStore.RejectsProgressBeyondPlanBounds 场景的行为、状态变化和边界条件。
TEST(MissionCheckpointStore, RejectsProgressBeyondPlanBounds) {
  const auto path = std::filesystem::path(::testing::TempDir()) /
      "invalid_progress_checkpoint.json";
  MissionCheckpointRecord record;
  record.run_id = "run-invalid-progress";
  record.mission_kind = "cleaning";
  record.total_segments = 2u;
  record.next_segment = 3u;
  record.completed_segments = 2u;

  cleanbot_interfaces::msg::TaskSegment first;
  first.index = 0u;
  first.id = "segment-0";
  first.start_lat = 32.0;
  first.start_lon = 118.0;
  first.end_lat = 32.01;
  first.end_lon = 118.01;
  record.segments.push_back(first);

  cleanbot_interfaces::msg::TaskSegment second = first;
  second.index = 1u;
  second.id = "segment-1";
  record.segments.push_back(second);

  MissionCheckpointStore store(path);
  std::string error;
  EXPECT_FALSE(store.save(record, &error));
  EXPECT_FALSE(error.empty());
}

// 测试目的：验证 MissionCheckpointStore.PreservesUnboundedLoopProgress 场景的行为、状态变化和边界条件。
TEST(MissionCheckpointStore, PreservesUnboundedLoopProgress) {
  const auto path = std::filesystem::path(::testing::TempDir()) /
      "continuous_mission_checkpoint.json";
  MissionCheckpointRecord record;
  record.run_id = "run-continuous";
  record.mission_kind = "cleaning";
  record.loop = true;
  record.loop_count = 0u;
  record.current_loop = 37u;
  record.total_loops = 0u;
  record.total_segments = 1u;

  cleanbot_interfaces::msg::TaskSegment segment;
  segment.id = "segment-continuous";
  segment.start_lat = 32.0;
  segment.start_lon = 118.0;
  segment.end_lat = 32.01;
  segment.end_lon = 118.01;
  record.segments.push_back(segment);

  MissionCheckpointStore store(path);
  ASSERT_TRUE(store.save(record));
  const auto loaded = store.load_latest();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->current_loop, 37u);
  EXPECT_EQ(loaded->total_loops, 0u);
}
