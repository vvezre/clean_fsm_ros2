#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "cleanbot_mission/maintenance_gate.hpp"

namespace {

using cleanbot::mission::CommandStatusEvidence;
using cleanbot::mission::FinalCommandEvidence;
using cleanbot::mission::HardwareEvidence;
using cleanbot::mission::MaintenanceGate;

constexpr std::uint64_t kGeneration = 41u;
constexpr std::uint64_t kCommandId = 7001u;
constexpr std::uint64_t kPublisherEpoch = 1u;

FinalCommandEvidence brakeCommand(
    const std::uint64_t generation = kGeneration,
    const std::uint64_t request_id = kGeneration,
    const std::uint64_t command_id = kCommandId) {
  FinalCommandEvidence command;
  command.generation = generation;
  command.request_id = request_id;
  command.command_id = command_id;
  command.source = "maintenance_gate";
  command.active = true;
  command.brake = true;
  return command;
}

CommandStatusEvidence commandStatus(
    const std::uint8_t state,
    const std::uint64_t generation = kGeneration,
    const std::uint64_t request_id = kGeneration,
    const std::uint64_t command_id = kCommandId) {
  CommandStatusEvidence status;
  status.generation = generation;
  status.request_id = request_id;
  status.command_id = command_id;
  status.source = "maintenance_gate";
  status.state = state;
  return status;
}

HardwareEvidence hardware(
    const std::uint64_t frame_sequence,
    const bool fresh = true,
    const bool connected = true,
    const std::int32_t x_speed = 0,
    const std::int32_t z_speed = 0,
    const std::int32_t brush_speed = 0,
    const std::uint64_t publisher_epoch = kPublisherEpoch) {
  HardwareEvidence sample;
  sample.publisher_epoch = publisher_epoch;
  sample.frame_sequence = frame_sequence;
  sample.fresh = fresh;
  sample.connected = connected;
  sample.x_speed = x_speed;
  sample.z_speed = z_speed;
  sample.brush_speed = brush_speed;
  return sample;
}

void applyBrakeAck(MaintenanceGate* gate) {
  gate->observeFinalCommand(brakeCommand());
  gate->observeCommandStatus(commandStatus(2u));
}

HardwareEvidence hardwareInEpoch(
    const std::uint64_t frame_sequence,
    const std::uint64_t publisher_epoch,
    const bool fresh = true) {
  return hardware(
      frame_sequence,
      fresh,
      true,
      0,
      0,
      0,
      publisher_epoch);
}

}  // namespace

TEST(MaintenanceGate, RejectsZeroGenerationAndResetsForNewGeneration) {
  MaintenanceGate gate;
  EXPECT_FALSE(gate.request(0u, true));
  EXPECT_FALSE(gate.snapshot().gate_active);

  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  ASSERT_TRUE(gate.snapshot().ready);

  ASSERT_TRUE(gate.request(kGeneration + 1u, true));
  const auto snapshot = gate.snapshot();
  EXPECT_TRUE(snapshot.gate_active);
  EXPECT_FALSE(snapshot.command_gate_applied);
  EXPECT_FALSE(snapshot.brake_acknowledged);
  EXPECT_FALSE(snapshot.hardware_fresh);
  EXPECT_FALSE(snapshot.ready);
  EXPECT_EQ(snapshot.phase, "WAITING_FOR_COMMAND_GATE");
}

TEST(MaintenanceGate, RequiresCorrelatedFinalBrakeBeforeCorrelatedAck) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));

  gate.observeFinalCommand(brakeCommand(kGeneration, kGeneration + 1u));
  EXPECT_FALSE(gate.snapshot().command_gate_applied);
  gate.observeFinalCommand(brakeCommand(kGeneration, kGeneration, 0u));
  EXPECT_FALSE(gate.snapshot().command_gate_applied);
  gate.observeFinalCommand(brakeCommand());
  EXPECT_TRUE(gate.snapshot().command_gate_applied);

  gate.observeCommandStatus(commandStatus(2u, kGeneration + 1u));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);
  gate.observeCommandStatus(commandStatus(2u));
  EXPECT_TRUE(gate.snapshot().brake_acknowledged);
}

TEST(MaintenanceGate, IgnoresPreAckZerosAndDuplicateFrameSequence) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(3u));
  gate.observeHardware(hardware(3u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(4u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, RejectedBrakeStaysFailedClosed) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeFinalCommand(brakeCommand());
  gate.observeCommandStatus(commandStatus(3u));
  EXPECT_EQ(gate.snapshot().phase, "BRAKE_REJECTED");

  gate.observeCommandStatus(commandStatus(2u));
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  EXPECT_FALSE(gate.snapshot().ready);
}

TEST(MaintenanceGate, TerminalStatusBeforeFinalBrakeStaysFailedClosed) {
  const std::pair<std::uint8_t, std::string> terminal_states[] = {
    {3u, "BRAKE_REJECTED"},
    {4u, "BRAKE_TIMED_OUT"},
    {7u, "TRANSPORT_LOST"},
    {8u, "BRAKE_SUPERSEDED"},
  };

  for (const auto& [state, phase] : terminal_states) {
    MaintenanceGate gate;
    ASSERT_TRUE(gate.request(kGeneration, true));
    gate.observeCommandStatus(commandStatus(2u));
    gate.observeCommandStatus(commandStatus(state));
    EXPECT_EQ(gate.snapshot().phase, "WAITING_FOR_COMMAND_GATE");

    gate.observeFinalCommand(brakeCommand());
    gate.observeHardware(hardware(1u));
    gate.observeHardware(hardware(2u));
    EXPECT_EQ(gate.snapshot().phase, phase);
    EXPECT_FALSE(gate.snapshot().ready);
  }
}

TEST(MaintenanceGate, MismatchedTerminalBeforeFinalBrakeIsIgnored) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeCommandStatus(commandStatus(3u, kGeneration, kGeneration + 1u));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, AckBeforeFinalIsCorrelatedAndDuplicateAckIsIdempotent) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeCommandStatus(commandStatus(2u));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);

  gate.observeFinalCommand(brakeCommand());
  EXPECT_TRUE(gate.snapshot().brake_acknowledged);
  gate.observeHardware(hardware(1u));
  gate.observeCommandStatus(commandStatus(2u));
  gate.observeHardware(hardware(2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, WrongCommandStatusDoesNotConfirmCurrentFinal) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeFinalCommand(brakeCommand());
  gate.observeCommandStatus(
      commandStatus(2u, kGeneration, kGeneration, 0u));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);
  gate.observeCommandStatus(
      commandStatus(2u, kGeneration, kGeneration, kCommandId + 1u));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);

  gate.observeCommandStatus(commandStatus(2u));
  EXPECT_TRUE(gate.snapshot().brake_acknowledged);
}

TEST(MaintenanceGate, NewFinalCommandClearsOldAckAndHardwareEvidence) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  ASSERT_TRUE(gate.snapshot().ready);

  const std::uint64_t next_command_id = kCommandId + 1u;
  gate.observeFinalCommand(
      brakeCommand(kGeneration, kGeneration, next_command_id));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeCommandStatus(commandStatus(2u));
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);

  gate.observeCommandStatus(
      commandStatus(2u, kGeneration, kGeneration, next_command_id));
  gate.observeHardware(hardware(3u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(4u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, TerminalStatusUsesCommandIdAndOverridesAck) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  const std::uint64_t wrong_command_id = kCommandId + 1u;
  gate.observeCommandStatus(
      commandStatus(3u, kGeneration, kGeneration, wrong_command_id));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  ASSERT_TRUE(gate.snapshot().ready);

  gate.observeCommandStatus(commandStatus(3u));
  EXPECT_EQ(gate.snapshot().phase, "BRAKE_REJECTED");
  EXPECT_FALSE(gate.snapshot().ready);

  MaintenanceGate other;
  ASSERT_TRUE(other.request(kGeneration, true));
  other.observeCommandStatus(
      commandStatus(3u, kGeneration, kGeneration, wrong_command_id));
  other.observeFinalCommand(
      brakeCommand(kGeneration, kGeneration, wrong_command_id));
  EXPECT_EQ(other.snapshot().phase, "BRAKE_REJECTED");
  EXPECT_FALSE(other.snapshot().ready);
}

TEST(MaintenanceGate, PendingCommandStatusOverflowFailsClosed) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  const std::size_t capacity = gate.pendingStatusCapacity();
  ASSERT_GT(capacity, 0u);
  for (std::size_t index = 0u; index < capacity; ++index) {
    gate.observeCommandStatus(commandStatus(
        2u,
        kGeneration,
        kGeneration,
        kCommandId + index));
  }

  gate.observeCommandStatus(commandStatus(
      3u,
      kGeneration,
      kGeneration,
      kCommandId + capacity));
  EXPECT_EQ(gate.snapshot().phase, "CORRELATION_OVERFLOW");
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  EXPECT_FALSE(gate.snapshot().ready);
}

TEST(MaintenanceGate, ReplayedPreAckSequenceDoesNotCountAfterAck) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  gate.observeHardware(hardware(100u));
  applyBrakeAck(&gate);

  gate.observeHardware(hardware(100u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(101u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(102u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, PublisherEpochChangeRequiresTwoNewZeroFrames) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardwareInEpoch(100u, 1u));
  gate.observeHardware(hardwareInEpoch(101u, 1u));
  ASSERT_TRUE(gate.snapshot().ready);

  gate.observeHardware(hardwareInEpoch(1u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(2u, 2u));
  EXPECT_TRUE(gate.snapshot().ready);

  gate.observeHardware(hardwareInEpoch(3u, 2u, false));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(4u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(5u, 2u));
  EXPECT_TRUE(gate.snapshot().ready);

  gate.observeHardware(hardwareInEpoch(0u, 0u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(5u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(6u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(7u, 2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, RetiredPublisherEpochCannotOverrideCurrentBlocker) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(
      1u,
      true,
      true,
      1,
      0,
      0,
      2u));
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_STOPPED");

  gate.observeHardware(hardwareInEpoch(900u, 1u));
  gate.observeHardware(hardwareInEpoch(901u, 1u));
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_STOPPED");

  gate.observeHardware(hardwareInEpoch(2u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(3u, 2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, EpochZeroAndRetiredEpochCannotRestoreReadiness) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardwareInEpoch(10u, 2u));
  gate.observeHardware(hardwareInEpoch(11u, 2u));
  ASSERT_TRUE(gate.snapshot().ready);

  gate.observeHardware(hardwareInEpoch(0u, 0u));
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_FRESH");

  gate.observeHardware(hardwareInEpoch(1000u, 1u));
  gate.observeHardware(hardwareInEpoch(1001u, 1u));
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_FRESH");

  gate.observeHardware(hardwareInEpoch(12u, 2u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(13u, 2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, PublisherEpochMaximumDoesNotWrapToOlderSession) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  constexpr std::uint64_t maximum =
      std::numeric_limits<std::uint64_t>::max();
  gate.observeHardware(hardware(
      1u,
      true,
      true,
      1,
      0,
      0,
      maximum));
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_STOPPED");

  gate.observeHardware(hardwareInEpoch(900u, maximum - 1u));
  gate.observeHardware(hardwareInEpoch(901u, maximum - 1u));
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().blocker_code, "HARDWARE_NOT_STOPPED");

  gate.observeHardware(hardwareInEpoch(2u, maximum));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardwareInEpoch(3u, maximum));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, BadLatestHardwareRequiresTwoNewZeroFrames) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(10u));
  gate.observeHardware(hardware(11u, false));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(12u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(13u));
  EXPECT_TRUE(gate.snapshot().ready);

  gate.observeHardware(hardware(14u, true, false));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(15u));
  gate.observeHardware(hardware(16u, true, true, 0, 1, 0));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(17u));
  EXPECT_FALSE(gate.snapshot().ready);
  gate.observeHardware(hardware(18u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, MissionBusyAndMismatchedReleaseFailClosed) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, false));
  EXPECT_EQ(gate.snapshot().phase, "MISSION_BUSY");
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  EXPECT_FALSE(gate.snapshot().ready);

  gate.setMissionIdle(true);
  EXPECT_TRUE(gate.snapshot().ready);
  EXPECT_FALSE(gate.release(kGeneration + 1u));
  EXPECT_TRUE(gate.snapshot().gate_active);
  EXPECT_TRUE(gate.release(kGeneration));
  EXPECT_EQ(gate.snapshot().phase, "INACTIVE");
}

TEST(MaintenanceGate, GenerationMustIncreaseAndOldEvidenceIsIgnored) {
  MaintenanceGate gate;
  ASSERT_TRUE(gate.request(kGeneration, true));
  EXPECT_EQ(gate.lastGeneration(), kGeneration);
  applyBrakeAck(&gate);
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  ASSERT_TRUE(gate.snapshot().ready);
  EXPECT_FALSE(gate.request(kGeneration, false));
  EXPECT_FALSE(gate.request(kGeneration - 1u, false));
  EXPECT_TRUE(gate.snapshot().ready);
  EXPECT_EQ(gate.snapshot().generation, kGeneration);

  ASSERT_TRUE(gate.release(kGeneration));
  EXPECT_FALSE(gate.request(kGeneration, true));
  EXPECT_FALSE(gate.request(kGeneration - 1u, true));
  EXPECT_FALSE(gate.snapshot().gate_active);
  EXPECT_EQ(gate.lastGeneration(), kGeneration);

  const std::uint64_t next_generation = kGeneration + 1u;
  ASSERT_TRUE(gate.request(next_generation, true));
  gate.observeFinalCommand(brakeCommand());
  gate.observeCommandStatus(commandStatus(2u));
  EXPECT_FALSE(gate.snapshot().command_gate_applied);
  EXPECT_FALSE(gate.snapshot().brake_acknowledged);

  gate.observeFinalCommand(
      brakeCommand(next_generation, next_generation));
  gate.observeCommandStatus(
      commandStatus(2u, next_generation, next_generation));
  gate.observeHardware(hardware(1u));
  gate.observeHardware(hardware(2u));
  EXPECT_TRUE(gate.snapshot().ready);
}

TEST(MaintenanceGate, MaximumGenerationIsNaturalFailClosedBoundary) {
  MaintenanceGate gate;
  constexpr std::uint64_t maximum =
      std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(gate.request(maximum, true));
  EXPECT_EQ(gate.lastGeneration(), maximum);
  ASSERT_TRUE(gate.release(maximum));
  EXPECT_FALSE(gate.request(maximum, true));
  EXPECT_FALSE(gate.request(maximum - 1u, true));
  EXPECT_FALSE(gate.snapshot().gate_active);
}
