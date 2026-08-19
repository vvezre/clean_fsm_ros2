// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_control/command_arbiter_core.hpp"

namespace {

// 辅助函数作用：通过 command 构造当前测试所需的输入数据。
cleanbot::control::ControlCommand command(
    const char* source,
    const std::uint64_t command_id,
    const std::int32_t x_speed,
    const bool operator_intent = false,
    const bool brake = false) {
  cleanbot::control::ControlCommand result;
  result.source = source;
  result.command_id = command_id;
  result.request_id = command_id;
  result.stamp_ms = command_id;
  result.active = true;
  result.operator_intent = operator_intent;
  result.status = 1u;
  result.x_speed = x_speed;
  result.z_speed = 17;
  result.steering_offset = 23;
  result.target_distance = 29;
  result.target_rotation = 31;
  result.heading_deg = 45.0;
  result.brake = brake;
  result.charge = true;
  return result;
}

// 辅助函数作用：为测试场景提供 publisherIdentity 所需的准备、执行或清理逻辑。
cleanbot::common::PublisherIdentity publisherIdentity(
    std::string implementation_identifier,
    std::initializer_list<std::uint8_t> gid) {
  return cleanbot::common::PublisherIdentity{
      std::move(implementation_identifier),
      std::vector<std::uint8_t>(gid)};
}

// 辅助函数作用：封装 expectMaintenanceOutput 对应的测试断言，统一核对预期结果。
void expectMaintenanceOutput(
    const cleanbot::control::ControlCommand& output,
    const std::uint64_t generation) {
  EXPECT_EQ(output.request_id, generation);
  EXPECT_EQ(output.source, "maintenance_gate");
  EXPECT_EQ(output.priority, 80u);
  EXPECT_TRUE(output.active);
  EXPECT_FALSE(output.operator_intent);
  EXPECT_EQ(output.status, 0u);
  EXPECT_EQ(output.x_speed, 0);
  EXPECT_EQ(output.z_speed, 0);
  EXPECT_EQ(output.steering_offset, 0);
  EXPECT_EQ(output.brush_speed, 0);
  EXPECT_EQ(output.target_distance, 0);
  EXPECT_EQ(output.target_rotation, 0);
  EXPECT_DOUBLE_EQ(output.heading_deg, 0.0);
  EXPECT_TRUE(output.brake);
  EXPECT_FALSE(output.charge);
}

}  // namespace

// 测试目的：验证 CommandArbiterCore.EmergencyClearsBrushAndRequiresFreshIntent 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, EmergencyClearsBrushAndRequiresFreshIntent) {
  cleanbot::control::CommandArbiterCore arbiter;
  arbiter.set_brush(true, 60, true);
  cleanbot::control::ControlCommand emergency;
  emergency.active = true;
  emergency.operator_intent = true;
  emergency.command_id = 10u;
  emergency.brake = true;
  arbiter.update(cleanbot::control::CommandSource::kEmergency, emergency, 0u);
  EXPECT_TRUE(arbiter.software_stopped());
  EXPECT_EQ(arbiter.output(0u).brush_speed, 0);
}

// 测试目的：验证 CommandArbiterCore.MaintenanceClearsEverySlotAndBrushAndEmitsOnlyBrake 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, MaintenanceClearsEverySlotAndBrushAndEmitsOnlyBrake) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kEmergency,
      command("emergency-monitor", 1u, 101), 1u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kSafety,
      command("safety", 2u, 102), 2u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kMission,
      command("mission", 3u, 103), 3u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kManual,
      command("manual", 4u, 104), 4u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kVision,
      command("vision", 5u, 105), 5u));
  arbiter.set_brush(true, 75, true);

  EXPECT_TRUE(arbiter.set_maintenance(true, 41u));
  EXPECT_TRUE(arbiter.maintenance_active());
  EXPECT_EQ(arbiter.maintenance_generation(), 41u);
  expectMaintenanceOutput(arbiter.output(6u), 41u);

  EXPECT_TRUE(arbiter.set_maintenance(false, 41u));
  const auto released = arbiter.output(7u);
  EXPECT_EQ(released.source, "idle_brake");
  EXPECT_TRUE(released.brake);
  EXPECT_EQ(released.brush_speed, 0);
}

// 测试目的：验证 CommandArbiterCore.ActiveGenerationSwitchStaysClampedAndResetsOperatorMode 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, ActiveGenerationSwitchStaysClampedAndResetsOperatorMode) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kManual,
      command("manual-operator-mode", 1u, 103, true), 1u));
  arbiter.set_brush(true, 85, true);

  EXPECT_TRUE(arbiter.set_maintenance(true, 50u));
  expectMaintenanceOutput(arbiter.output(2u), 50u);
  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kMission,
      command("mission-during-generation-50", 2u, 102), 2u));
  arbiter.set_brush(true, 90, true);

  EXPECT_TRUE(arbiter.set_maintenance(true, 51u));
  EXPECT_TRUE(arbiter.maintenance_active());
  EXPECT_EQ(arbiter.maintenance_generation(), 51u);
  expectMaintenanceOutput(arbiter.output(3u), 51u);
  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kVision,
      command("vision-during-generation-51", 3u, 101), 3u));
  arbiter.set_brush(true, 95, true);
  expectMaintenanceOutput(arbiter.output(4u), 51u);

  EXPECT_TRUE(arbiter.set_maintenance(false, 51u));
  const auto released = arbiter.output(5u);
  EXPECT_EQ(released.source, "idle_brake");
  EXPECT_EQ(released.brush_speed, 0);

  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kMission,
      command("mission-after-maintenance", 4u, 100), 6u));
  const auto resumed = arbiter.output(6u);
  EXPECT_EQ(resumed.source, "mission-after-maintenance");
  EXPECT_EQ(resumed.x_speed, 100);
  EXPECT_EQ(resumed.brush_speed, 0);
}

// 测试目的：验证 CommandArbiterCore.MaintenanceRejectsOrdinaryMotionAndBrushUntilRelease 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, MaintenanceRejectsOrdinaryMotionAndBrushUntilRelease) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.set_maintenance(true, 7u));

  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kSafety,
      command("safety", 1u, 300), 1u));
  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kManual,
      command("manual", 2u, 200, true), 2u));
  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kMission,
      command("mission", 3u, 100, true), 3u));
  EXPECT_FALSE(arbiter.update(
      cleanbot::control::CommandSource::kVision,
      command("vision", 4u, 50, true), 4u));
  arbiter.set_brush(true, 90, true);
  expectMaintenanceOutput(arbiter.output(5u), 7u);

  EXPECT_TRUE(arbiter.set_maintenance(false, 7u));
  const auto released = arbiter.output(6u);
  EXPECT_EQ(released.source, "idle_brake");
  EXPECT_TRUE(released.brake);
  EXPECT_EQ(released.brush_speed, 0);
}

// 测试目的：验证 CommandArbiterCore.EmergencyReceivedDuringMaintenanceSurvivesRelease 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, EmergencyReceivedDuringMaintenanceSurvivesRelease) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.set_maintenance(true, 9u));

  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kEmergency,
      command("hardware-emergency", 100u, 0, true, true), 10u));
  EXPECT_TRUE(arbiter.software_stopped());
  expectMaintenanceOutput(arbiter.output(11u), 9u);

  EXPECT_TRUE(arbiter.set_maintenance(false, 9u));
  const auto released = arbiter.output(12u);
  EXPECT_EQ(released.source, "software_emergency_stop");
  EXPECT_TRUE(released.brake);
  EXPECT_EQ(released.priority, 100u);
}

// 测试目的：验证 CommandArbiterCore.MaintenanceReleasePreservesExistingSoftwareStop 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, MaintenanceReleasePreservesExistingSoftwareStop) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kEmergency,
      command("hardware-emergency", 100u, 0, true, true), 1u));
  EXPECT_TRUE(arbiter.software_stopped());

  EXPECT_TRUE(arbiter.set_maintenance(true, 11u));
  expectMaintenanceOutput(arbiter.output(2u), 11u);
  EXPECT_TRUE(arbiter.set_maintenance(false, 11u));

  EXPECT_TRUE(arbiter.software_stopped());
  EXPECT_EQ(arbiter.output(3u).source, "software_emergency_stop");
}

// 测试目的：验证 CommandArbiterCore.MaintenanceGenerationIsStrictAndCannotBeReused 场景的行为、状态变化和边界条件。
TEST(CommandArbiterCore, MaintenanceGenerationIsStrictAndCannotBeReused) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_FALSE(arbiter.set_maintenance(true, 0u));
  EXPECT_FALSE(arbiter.set_maintenance(false, 0u));

  EXPECT_TRUE(arbiter.set_maintenance(true, 5u));
  EXPECT_TRUE(arbiter.set_maintenance(true, 5u));
  EXPECT_FALSE(arbiter.set_maintenance(false, 4u));
  EXPECT_FALSE(arbiter.set_maintenance(false, 6u));
  EXPECT_TRUE(arbiter.maintenance_active());

  EXPECT_TRUE(arbiter.set_maintenance(true, 6u));
  expectMaintenanceOutput(arbiter.output(0u), 6u);
  EXPECT_TRUE(arbiter.set_maintenance(false, 6u));
  EXPECT_FALSE(arbiter.maintenance_active());
  EXPECT_EQ(arbiter.maintenance_generation(), 6u);
  EXPECT_FALSE(arbiter.set_maintenance(false, 6u));
  EXPECT_FALSE(arbiter.set_maintenance(true, 5u));
  EXPECT_FALSE(arbiter.set_maintenance(true, 6u));
  EXPECT_TRUE(arbiter.set_maintenance(true, 7u));

  cleanbot::control::CommandArbiterCore max_arbiter;
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  EXPECT_TRUE(max_arbiter.set_maintenance(true, maximum));
  expectMaintenanceOutput(max_arbiter.output(0u), maximum);
  EXPECT_TRUE(max_arbiter.set_maintenance(false, maximum));
  EXPECT_FALSE(max_arbiter.set_maintenance(true, maximum));
  EXPECT_FALSE(max_arbiter.set_maintenance(true, maximum - 1u));
}

// 测试目的：验证 MaintenanceGateCache.MismatchedInactiveCannotReplaceActiveGeneration 场景的行为、状态变化和边界条件。
TEST(MaintenanceGateCache, MismatchedInactiveCannotReplaceActiveGeneration) {
  cleanbot::control::MaintenanceGateCache cache;
  EXPECT_TRUE(cache.update(true, 10u));
  EXPECT_TRUE(cache.has_state());
  EXPECT_TRUE(cache.active());
  EXPECT_EQ(cache.generation(), 10u);

  EXPECT_FALSE(cache.update(false, 11u));
  EXPECT_TRUE(cache.active());
  EXPECT_EQ(cache.generation(), 10u);
  EXPECT_TRUE(cache.update(false, 10u));
  EXPECT_FALSE(cache.active());
  EXPECT_EQ(cache.generation(), 10u);
}

// 测试目的：验证 MaintenanceGateCache.ReleasedAndInvalidGenerationsCannotBeReused 场景的行为、状态变化和边界条件。
TEST(MaintenanceGateCache, ReleasedAndInvalidGenerationsCannotBeReused) {
  cleanbot::control::MaintenanceGateCache cache;
  EXPECT_FALSE(cache.update(true, 0u));
  EXPECT_FALSE(cache.update(false, 0u));
  EXPECT_FALSE(cache.update(false, 5u));
  EXPECT_FALSE(cache.has_state());

  EXPECT_TRUE(cache.update(true, 5u));
  EXPECT_TRUE(cache.update(true, 5u));
  EXPECT_TRUE(cache.update(false, 5u));
  EXPECT_FALSE(cache.update(false, 5u));
  EXPECT_FALSE(cache.update(true, 5u));
  EXPECT_FALSE(cache.update(true, 4u));
  EXPECT_FALSE(cache.update(false, 6u));
  EXPECT_EQ(cache.generation(), 5u);
  EXPECT_FALSE(cache.active());

  EXPECT_TRUE(cache.update(true, 6u));
  EXPECT_TRUE(cache.active());
  EXPECT_EQ(cache.generation(), 6u);
}

// 测试目的：验证 MaintenancePublisherCoordinator.PublisherRestartForcesExactlyOneReissueAndRetiresOldPublisher 场景的行为、状态变化和边界条件。
TEST(
    MaintenancePublisherCoordinator,
    PublisherRestartForcesExactlyOneReissueAndRetiresOldPublisher) {
  cleanbot::control::MaintenancePublisherCoordinator coordinator;
  const auto publisher_a = publisherIdentity("rmw-a", {1u, 2u, 3u});
  const auto publisher_b = publisherIdentity("rmw-a", {4u, 5u, 6u});

  const auto first = coordinator.observe(true, 10u, publisher_a);
  EXPECT_TRUE(first.accepted);
  EXPECT_TRUE(first.gate_active);
  EXPECT_EQ(first.generation, 10u);
  EXPECT_TRUE(first.session_changed);
  EXPECT_TRUE(first.force_republish);

  const auto repeated_a = coordinator.observe(true, 10u, publisher_a);
  EXPECT_TRUE(repeated_a.accepted);
  EXPECT_FALSE(repeated_a.session_changed);
  EXPECT_FALSE(repeated_a.force_republish);

  const auto first_b = coordinator.observe(true, 10u, publisher_b);
  EXPECT_TRUE(first_b.accepted);
  EXPECT_TRUE(first_b.gate_active);
  EXPECT_EQ(first_b.generation, 10u);
  EXPECT_TRUE(first_b.session_changed);
  EXPECT_TRUE(first_b.force_republish);

  const auto repeated_b = coordinator.observe(true, 10u, publisher_b);
  EXPECT_TRUE(repeated_b.accepted);
  EXPECT_FALSE(repeated_b.session_changed);
  EXPECT_FALSE(repeated_b.force_republish);

  const auto retired_a = coordinator.observe(true, 11u, publisher_a);
  EXPECT_FALSE(retired_a.accepted);
  EXPECT_TRUE(retired_a.gate_active);
  EXPECT_EQ(retired_a.generation, 10u);
  EXPECT_FALSE(retired_a.session_changed);
  EXPECT_FALSE(retired_a.force_republish);
}

// 测试目的：验证 MaintenancePublisherCoordinator.InvalidUnknownStateDoesNotPoisonCurrentPublisherOrGate 场景的行为、状态变化和边界条件。
TEST(
    MaintenancePublisherCoordinator,
    InvalidUnknownStateDoesNotPoisonCurrentPublisherOrGate) {
  cleanbot::control::MaintenancePublisherCoordinator coordinator;
  const auto publisher_a = publisherIdentity("rmw-a", {1u});
  const auto publisher_b = publisherIdentity("rmw-a", {2u});
  const auto publisher_c = publisherIdentity("rmw-a", {3u});

  ASSERT_TRUE(coordinator.observe(true, 10u, publisher_a).accepted);
  ASSERT_TRUE(coordinator.observe(true, 10u, publisher_b).accepted);

  const auto old_active = coordinator.observe(true, 9u, publisher_c);
  EXPECT_FALSE(old_active.accepted);
  EXPECT_TRUE(old_active.gate_active);
  EXPECT_EQ(old_active.generation, 10u);

  const auto mismatched_release =
      coordinator.observe(false, 11u, publisher_c);
  EXPECT_FALSE(mismatched_release.accepted);
  EXPECT_TRUE(mismatched_release.gate_active);
  EXPECT_EQ(mismatched_release.generation, 10u);

  const auto current_b = coordinator.observe(true, 10u, publisher_b);
  EXPECT_TRUE(current_b.accepted);
  EXPECT_FALSE(current_b.session_changed);
  EXPECT_FALSE(current_b.force_republish);

  const auto higher_active = coordinator.observe(true, 11u, publisher_c);
  EXPECT_TRUE(higher_active.accepted);
  EXPECT_TRUE(higher_active.gate_active);
  EXPECT_EQ(higher_active.generation, 11u);
  EXPECT_TRUE(higher_active.session_changed);
  EXPECT_TRUE(higher_active.force_republish);

  const auto retired_b = coordinator.observe(true, 12u, publisher_b);
  EXPECT_FALSE(retired_b.accepted);
  EXPECT_EQ(retired_b.generation, 11u);
}

// 测试目的：验证 MaintenancePublisherCoordinator.NewPublisherCanCommitExactReleaseWithoutForcingReissue 场景的行为、状态变化和边界条件。
TEST(
    MaintenancePublisherCoordinator,
    NewPublisherCanCommitExactReleaseWithoutForcingReissue) {
  cleanbot::control::MaintenancePublisherCoordinator coordinator;
  const auto publisher_a = publisherIdentity("rmw-a", {1u});
  const auto publisher_b = publisherIdentity("rmw-a", {2u});

  ASSERT_TRUE(coordinator.observe(true, 20u, publisher_a).accepted);
  const auto released = coordinator.observe(false, 20u, publisher_b);

  EXPECT_TRUE(released.accepted);
  EXPECT_FALSE(released.gate_active);
  EXPECT_EQ(released.generation, 20u);
  EXPECT_TRUE(released.session_changed);
  EXPECT_FALSE(released.force_republish);
  const auto repeated_release =
      coordinator.observe(false, 20u, publisher_b);
  EXPECT_TRUE(repeated_release.accepted);
  EXPECT_FALSE(repeated_release.session_changed);
  EXPECT_FALSE(repeated_release.force_republish);
}

// 测试目的：验证 MaintenancePublisherCoordinator.UntrackableIdentityOnlyEstablishesInitialActiveClamp 场景的行为、状态变化和边界条件。
TEST(
    MaintenancePublisherCoordinator,
    UntrackableIdentityOnlyEstablishesInitialActiveClamp) {
  cleanbot::control::MaintenancePublisherCoordinator coordinator;
  const auto invalid_publisher = publisherIdentity("rmw-a", {0u, 0u, 0u});
  const auto publisher_a = publisherIdentity("rmw-a", {1u, 2u, 3u});

  const auto initial_active =
      coordinator.observe(true, 30u, invalid_publisher);
  EXPECT_TRUE(initial_active.accepted);
  EXPECT_TRUE(initial_active.gate_active);
  EXPECT_EQ(initial_active.generation, 30u);
  EXPECT_FALSE(initial_active.session_changed);
  EXPECT_FALSE(initial_active.force_republish);

  const auto invalid_release =
      coordinator.observe(false, 30u, invalid_publisher);
  EXPECT_FALSE(invalid_release.accepted);
  EXPECT_TRUE(invalid_release.gate_active);
  EXPECT_EQ(invalid_release.generation, 30u);

  const auto premature_valid_release =
      coordinator.observe(false, 30u, publisher_a);
  EXPECT_FALSE(premature_valid_release.accepted);
  EXPECT_TRUE(premature_valid_release.gate_active);
  EXPECT_EQ(premature_valid_release.generation, 30u);
  EXPECT_FALSE(premature_valid_release.session_changed);

  const auto first_tracked = coordinator.observe(true, 30u, publisher_a);
  EXPECT_TRUE(first_tracked.accepted);
  EXPECT_TRUE(first_tracked.session_changed);
  EXPECT_TRUE(first_tracked.force_republish);

  const auto invalid_higher =
      coordinator.observe(true, 31u, invalid_publisher);
  EXPECT_FALSE(invalid_higher.accepted);
  EXPECT_EQ(invalid_higher.generation, 30u);
  const auto invalid_after_current_release =
      coordinator.observe(false, 30u, invalid_publisher);
  EXPECT_FALSE(invalid_after_current_release.accepted);
  EXPECT_TRUE(invalid_after_current_release.gate_active);

  const auto current = coordinator.observe(true, 30u, publisher_a);
  EXPECT_TRUE(current.accepted);
  EXPECT_FALSE(current.session_changed);
  EXPECT_FALSE(current.force_republish);

  const auto tracked_release =
      coordinator.observe(false, 30u, publisher_a);
  EXPECT_TRUE(tracked_release.accepted);
  EXPECT_FALSE(tracked_release.gate_active);
  EXPECT_EQ(tracked_release.generation, 30u);
  EXPECT_FALSE(tracked_release.session_changed);
  EXPECT_FALSE(tracked_release.force_republish);

  cleanbot::control::MaintenancePublisherCoordinator inactive_coordinator;
  EXPECT_FALSE(
      inactive_coordinator.observe(false, 1u, invalid_publisher).accepted);
  EXPECT_FALSE(inactive_coordinator.has_state());
}

// 测试目的：验证 MaintenancePublisherCoordinator.TrackerCapacityExhaustionDoesNotMutateGateOrCurrentPublisher 场景的行为、状态变化和边界条件。
TEST(
    MaintenancePublisherCoordinator,
    TrackerCapacityExhaustionDoesNotMutateGateOrCurrentPublisher) {
  cleanbot::control::MaintenancePublisherCoordinator coordinator(0u);
  const auto publisher_a = publisherIdentity("rmw-a", {1u});
  const auto publisher_b = publisherIdentity("rmw-a", {2u});

  ASSERT_TRUE(coordinator.observe(true, 40u, publisher_a).accepted);
  const auto exhausted_higher =
      coordinator.observe(true, 41u, publisher_b);
  EXPECT_FALSE(exhausted_higher.accepted);
  EXPECT_TRUE(exhausted_higher.gate_active);
  EXPECT_EQ(exhausted_higher.generation, 40u);

  const auto exhausted_release =
      coordinator.observe(false, 40u, publisher_b);
  EXPECT_FALSE(exhausted_release.accepted);
  EXPECT_TRUE(exhausted_release.gate_active);
  EXPECT_EQ(exhausted_release.generation, 40u);

  const auto current = coordinator.observe(true, 40u, publisher_a);
  EXPECT_TRUE(current.accepted);
  EXPECT_FALSE(current.session_changed);
  EXPECT_FALSE(current.force_republish);
}
