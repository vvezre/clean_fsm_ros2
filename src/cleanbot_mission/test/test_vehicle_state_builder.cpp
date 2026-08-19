// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <limits>

#include "cleanbot_mission/vehicle_state_builder.hpp"

#include <gtest/gtest.h>

using cleanbot::mission::VehicleStateInput;
using cleanbot::mission::build_vehicle_state;

// 测试目的：验证 VehicleStateBuilder.ReportsReadyIdleVehicle 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, ReportsReadyIdleVehicle) {
  VehicleStateInput input;
  input.configured = true;
  input.hardware_ready = true;
  input.rtk_ready = true;
  input.has_final_command = true;
  input.final_command_active = true;
  input.final_command_brake = true;
  input.battery_percent = 82.5;

  const auto state = build_vehicle_state(input);

  EXPECT_EQ(state.control_state, "idle");
  EXPECT_EQ(state.health_state, "ready");
  EXPECT_EQ(state.fault_state, "NONE");
  EXPECT_EQ(state.current_action, "idle");
  EXPECT_TRUE(state.start_ready);
  EXPECT_TRUE(state.parking);
  EXPECT_DOUBLE_EQ(state.battery_percent, 82.5);
}

// 测试目的：验证 VehicleStateBuilder.FaultOverridesPauseAndLifecycleMessages 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, FaultOverridesPauseAndLifecycleMessages) {
  VehicleStateInput input;
  input.configured = true;
  input.hardware_ready = true;
  input.rtk_ready = true;
  input.mission_active = true;
  input.current_action = "cleaning";
  input.lifecycle_message = "executing segment";
  input.pause_reason = "user pause";
  input.fault_code = "HARDWARE_DISCONNECTED";
  input.fault_message = "lower machine disconnected";

  const auto state = build_vehicle_state(input);

  EXPECT_EQ(state.health_state, "fault");
  EXPECT_EQ(state.fault_state, "HARDWARE_DISCONNECTED");
  EXPECT_EQ(state.message, "lower machine disconnected");
  EXPECT_FALSE(state.start_ready);
  EXPECT_TRUE(state.cleaning);
}

// 测试目的：验证 VehicleStateBuilder.InactiveMissionDoesNotPublishStaleFault 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, InactiveMissionDoesNotPublishStaleFault) {
  VehicleStateInput input;
  input.configured = true;
  input.hardware_ready = true;
  input.rtk_ready = true;
  input.mission_active = false;
  input.lifecycle_message = "mission failed: turn timeout";
  input.fault_code = "TURN_TIMEOUT";
  input.fault_message = "turn command timed out";

  const auto state = build_vehicle_state(input);

  EXPECT_EQ(state.health_state, "ready");
  EXPECT_EQ(state.fault_state, "NONE");
  EXPECT_EQ(state.message, "mission failed: turn timeout");
  EXPECT_TRUE(state.start_ready);
}

// 测试目的：验证 VehicleStateBuilder.MapsFinalCommandPriorityAndUnknownBattery 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, MapsFinalCommandPriorityAndUnknownBattery) {
  VehicleStateInput input;
  input.configured = true;
  input.has_final_command = true;
  input.final_command_active = true;
  input.final_command_priority = VehicleStateInput::PRIORITY_EMERGENCY;
  input.final_command_brake = true;
  input.battery_percent = std::numeric_limits<double>::quiet_NaN();

  const auto state = build_vehicle_state(input);

  EXPECT_EQ(state.control_state, "emergency");
  EXPECT_TRUE(state.parking);
  EXPECT_DOUBLE_EQ(state.battery_percent, -1.0);
}

// 测试目的：验证 VehicleStateBuilder.MapsAllArbitratedCommandPriorities 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, MapsAllArbitratedCommandPriorities) {
  VehicleStateInput input;
  input.configured = true;
  input.has_final_command = true;
  input.final_command_active = true;
  input.final_command_brake = false;

  input.final_command_priority = VehicleStateInput::PRIORITY_SAFETY;
  EXPECT_EQ(build_vehicle_state(input).control_state, "safety");
  input.final_command_priority = VehicleStateInput::PRIORITY_MANUAL;
  EXPECT_EQ(build_vehicle_state(input).control_state, "manual");
  input.final_command_priority = VehicleStateInput::PRIORITY_MISSION;
  EXPECT_EQ(build_vehicle_state(input).control_state, "mission");
  input.final_command_priority = VehicleStateInput::PRIORITY_VISION;
  EXPECT_EQ(build_vehicle_state(input).control_state, "vision");
}

// 测试目的：验证 VehicleStateBuilder.ReportsDegradedWhenRuntimeInputsAreNotReady 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, ReportsDegradedWhenRuntimeInputsAreNotReady) {
  VehicleStateInput input;
  input.configured = true;
  input.hardware_ready = true;
  input.rtk_ready = false;
  input.lifecycle_message = "waiting for RTK";

  const auto state = build_vehicle_state(input);

  EXPECT_EQ(state.health_state, "degraded");
  EXPECT_FALSE(state.rtk_fixed);
  EXPECT_FALSE(state.start_ready);
  EXPECT_EQ(state.message, "waiting for RTK");
}

// 测试目的：验证 VehicleStateBuilder.ExposesActiveMissionProgress 场景的行为、状态变化和边界条件。
TEST(VehicleStateBuilder, ExposesActiveMissionProgress) {
  VehicleStateInput input;
  input.configured = true;
  input.hardware_ready = true;
  input.rtk_ready = true;
  input.mission_active = true;
  input.current_action = "navigate_waypoints";
  input.lifecycle_message = "driving to waypoint";
  input.current_segment = 2u;
  input.total_segments = 7u;

  const auto state = build_vehicle_state(input);

  EXPECT_FALSE(state.start_ready);
  EXPECT_EQ(state.current_action, "navigate_waypoints");
  EXPECT_EQ(state.current_segment, 2u);
  EXPECT_EQ(state.total_segments, 7u);
  EXPECT_FALSE(state.cleaning);
}
