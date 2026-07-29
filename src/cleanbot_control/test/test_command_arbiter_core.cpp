#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "cleanbot_control/command_arbiter_core.hpp"

namespace {

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

TEST(CommandArbiterCore, NewMaintenanceGenerationClearsStateCreatedAfterRelease) {
  cleanbot::control::CommandArbiterCore arbiter;
  EXPECT_TRUE(arbiter.set_maintenance(true, 50u));
  EXPECT_TRUE(arbiter.set_maintenance(false, 50u));

  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kEmergency,
      command("emergency-monitor", 1u, 101), 1u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kSafety,
      command("safety", 2u, 102), 2u));
  EXPECT_TRUE(arbiter.update(
      cleanbot::control::CommandSource::kManual,
      command("manual-operator-mode", 3u, 103, true), 3u));
  arbiter.set_brush(true, 85, true);

  EXPECT_TRUE(arbiter.set_maintenance(true, 51u));
  expectMaintenanceOutput(arbiter.output(4u), 51u);
  EXPECT_TRUE(arbiter.set_maintenance(false, 51u));
  const auto released = arbiter.output(5u);
  EXPECT_EQ(released.source, "idle_brake");
  EXPECT_EQ(released.brush_speed, 0);
}

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
