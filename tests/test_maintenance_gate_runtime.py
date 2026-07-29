import ctypes
import os
import sysconfig
import unittest
from pathlib import Path


CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
    os.add_dll_directory(str(CPPYY_BACKEND_BIN))
    os.environ["PATH"] = str(CPPYY_BACKEND_BIN) + os.pathsep + os.environ.get("PATH", "")
    for dll_name in (
        "msvcp140.dll",
        "msvcp140_1.dll",
        "msvcp140_2.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
    ):
        ctypes.WinDLL(str(CPPYY_BACKEND_BIN / dll_name))

import cppyy


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"
HEADER = PACKAGE / "include" / "cleanbot_mission" / "maintenance_gate.hpp"
SOURCE = PACKAGE / "src" / "maintenance_gate.cpp"
CORE_LOADED = False
GENERATION = 41
COMMAND_ID = 7001
PUBLISHER_EPOCH = 1
MAX_UINT64 = (1 << 64) - 1


def load_core(test_case):
    global CORE_LOADED
    test_case.assertTrue(HEADER.is_file(), "maintenance gate header is missing")
    test_case.assertTrue(SOURCE.is_file(), "maintenance gate source is missing")
    if CORE_LOADED:
        return
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    cppyy.cppdef('#include "maintenance_gate.cpp"')
    CORE_LOADED = True


def final_command(
    generation=GENERATION,
    request_id=GENERATION,
    command_id=COMMAND_ID,
):
    command = cppyy.gbl.cleanbot.mission.FinalCommandEvidence()
    command.generation = generation
    command.request_id = request_id
    command.command_id = command_id
    command.source = "maintenance_gate"
    command.active = True
    command.brake = True
    command.x_speed = 0
    command.z_speed = 0
    command.brush_speed = 0
    return command


def command_status(
    state,
    generation=GENERATION,
    request_id=GENERATION,
    command_id=COMMAND_ID,
):
    status = cppyy.gbl.cleanbot.mission.CommandStatusEvidence()
    status.generation = generation
    status.request_id = request_id
    status.command_id = command_id
    status.source = "maintenance_gate"
    status.state = state
    return status


def hardware_sample(
    sequence,
    *,
    fresh=True,
    connected=True,
    x_speed=0,
    z_speed=0,
    brush_speed=0,
    publisher_epoch=PUBLISHER_EPOCH,
):
    sample = cppyy.gbl.cleanbot.mission.HardwareEvidence()
    sample.publisher_epoch = publisher_epoch
    sample.frame_sequence = sequence
    sample.fresh = fresh
    sample.connected = connected
    sample.x_speed = x_speed
    sample.z_speed = z_speed
    sample.brush_speed = brush_speed
    return sample


class MaintenanceGateRuntimeTest(unittest.TestCase):
    def setUp(self):
        load_core(self)
        self.gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(self.gate.request(GENERATION, True))

    def apply_brake_ack(self):
        self.gate.observeFinalCommand(final_command())
        self.gate.observeCommandStatus(command_status(2))

    def test_rejects_zero_generation_and_new_request_resets_all_evidence(self):
        other = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertFalse(other.request(0, True))
        self.assertFalse(other.snapshot().gate_active)

        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(10))
        self.gate.observeHardware(hardware_sample(11))
        self.assertTrue(self.gate.snapshot().ready)

        self.assertTrue(self.gate.request(GENERATION + 1, True))
        snapshot = self.gate.snapshot()
        self.assertTrue(snapshot.gate_active)
        self.assertFalse(snapshot.command_gate_applied)
        self.assertFalse(snapshot.brake_acknowledged)
        self.assertFalse(snapshot.hardware_fresh)
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.phase), "WAITING_FOR_COMMAND_GATE")

    def test_pre_ack_zero_frames_never_count(self):
        self.gate.observeHardware(hardware_sample(1))
        self.gate.observeHardware(hardware_sample(2))
        self.apply_brake_ack()
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(3))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(4))
        self.assertTrue(self.gate.snapshot().ready)

    def test_final_command_requires_matching_ids_source_and_safe_brake(self):
        mismatched = final_command(request_id=GENERATION + 1)
        self.gate.observeFinalCommand(mismatched)
        self.assertFalse(self.gate.snapshot().command_gate_applied)

        wrong_source = final_command()
        wrong_source.source = "mission"
        self.gate.observeFinalCommand(wrong_source)
        self.assertFalse(self.gate.snapshot().command_gate_applied)

        moving = final_command()
        moving.x_speed = 1
        self.gate.observeFinalCommand(moving)
        self.assertFalse(self.gate.snapshot().command_gate_applied)

        self.gate.observeFinalCommand(final_command(command_id=0))
        self.assertFalse(self.gate.snapshot().command_gate_applied)

        self.gate.observeFinalCommand(final_command())
        self.assertTrue(self.gate.snapshot().command_gate_applied)
        self.assertEqual(str(self.gate.snapshot().phase), "WAITING_FOR_BRAKE_ACK")

    def test_status_mismatch_is_ignored_and_rejection_is_fail_closed(self):
        self.gate.observeFinalCommand(final_command())
        self.gate.observeCommandStatus(command_status(2, request_id=999))
        self.assertFalse(self.gate.snapshot().brake_acknowledged)

        self.gate.observeCommandStatus(command_status(3))
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.phase), "BRAKE_REJECTED")
        self.assertEqual(str(snapshot.blocker_code), "BRAKE_REJECTED")

        self.gate.observeCommandStatus(command_status(2))
        self.gate.observeHardware(hardware_sample(1))
        self.gate.observeHardware(hardware_sample(2))
        self.assertFalse(self.gate.snapshot().ready)

    def test_terminal_transport_failures_are_fail_closed(self):
        for state, phase in ((4, "BRAKE_TIMED_OUT"), (7, "TRANSPORT_LOST"), (8, "BRAKE_SUPERSEDED")):
            with self.subTest(state=state):
                gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
                self.assertTrue(gate.request(GENERATION, True))
                gate.observeFinalCommand(final_command())
                gate.observeCommandStatus(command_status(state))
                self.assertEqual(str(gate.snapshot().phase), phase)
                gate.observeHardware(hardware_sample(1))
                gate.observeHardware(hardware_sample(2))
                self.assertFalse(gate.snapshot().ready)

    def test_terminal_status_before_final_command_is_latched(self):
        terminal_states = (
            (3, "BRAKE_REJECTED"),
            (4, "BRAKE_TIMED_OUT"),
            (7, "TRANSPORT_LOST"),
            (8, "BRAKE_SUPERSEDED"),
        )
        for state, phase in terminal_states:
            with self.subTest(state=state):
                gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
                self.assertTrue(gate.request(GENERATION, True))
                gate.observeCommandStatus(command_status(2))
                gate.observeCommandStatus(command_status(state))
                self.assertEqual(
                    str(gate.snapshot().phase),
                    "WAITING_FOR_COMMAND_GATE",
                )

                gate.observeFinalCommand(final_command())
                gate.observeHardware(hardware_sample(1))
                gate.observeHardware(hardware_sample(2))
                self.assertEqual(str(gate.snapshot().phase), phase)
                self.assertFalse(gate.snapshot().ready)

    def test_mismatched_terminal_status_before_final_is_ignored(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        gate.observeCommandStatus(command_status(3, request_id=GENERATION + 1))
        gate.observeFinalCommand(final_command())
        gate.observeCommandStatus(command_status(2))
        gate.observeHardware(hardware_sample(1))
        gate.observeHardware(hardware_sample(2))
        self.assertTrue(gate.snapshot().ready)

    def test_ack_before_final_is_correlated_and_duplicate_ack_is_idempotent(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        gate.observeCommandStatus(command_status(2))
        self.assertFalse(gate.snapshot().brake_acknowledged)

        gate.observeFinalCommand(final_command())
        self.assertTrue(gate.snapshot().brake_acknowledged)
        gate.observeHardware(hardware_sample(1))
        gate.observeCommandStatus(command_status(2))
        gate.observeHardware(hardware_sample(2))
        self.assertTrue(gate.snapshot().ready)

    def test_wrong_command_id_status_does_not_confirm_current_final(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        gate.observeFinalCommand(final_command())
        gate.observeCommandStatus(command_status(2, command_id=0))
        self.assertFalse(gate.snapshot().brake_acknowledged)
        gate.observeCommandStatus(command_status(2, command_id=COMMAND_ID + 1))
        self.assertFalse(gate.snapshot().brake_acknowledged)

        gate.observeCommandStatus(command_status(2))
        self.assertTrue(gate.snapshot().brake_acknowledged)

    def test_new_final_command_id_clears_old_ack_and_hardware_evidence(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        gate.observeFinalCommand(final_command())
        gate.observeCommandStatus(command_status(2))
        gate.observeHardware(hardware_sample(1))
        gate.observeHardware(hardware_sample(2))
        self.assertTrue(gate.snapshot().ready)

        next_command_id = COMMAND_ID + 1
        gate.observeFinalCommand(final_command(command_id=next_command_id))
        self.assertFalse(gate.snapshot().brake_acknowledged)
        self.assertFalse(gate.snapshot().ready)
        gate.observeCommandStatus(command_status(2))
        self.assertFalse(gate.snapshot().brake_acknowledged)

        gate.observeCommandStatus(command_status(2, command_id=next_command_id))
        gate.observeHardware(hardware_sample(3))
        self.assertFalse(gate.snapshot().ready)
        gate.observeHardware(hardware_sample(4))
        self.assertTrue(gate.snapshot().ready)

    def test_terminal_command_id_correlation_and_ack_then_terminal(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        wrong_command_id = COMMAND_ID + 1
        gate.observeCommandStatus(command_status(3, command_id=wrong_command_id))
        gate.observeFinalCommand(final_command())
        gate.observeCommandStatus(command_status(2))
        gate.observeHardware(hardware_sample(1))
        gate.observeHardware(hardware_sample(2))
        self.assertTrue(gate.snapshot().ready)

        gate.observeCommandStatus(command_status(3))
        self.assertEqual(str(gate.snapshot().phase), "BRAKE_REJECTED")
        self.assertFalse(gate.snapshot().ready)

        other = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(other.request(GENERATION, True))
        other.observeCommandStatus(command_status(3, command_id=wrong_command_id))
        other.observeFinalCommand(final_command(command_id=wrong_command_id))
        self.assertEqual(str(other.snapshot().phase), "BRAKE_REJECTED")
        self.assertFalse(other.snapshot().ready)

    def test_pending_command_status_overflow_fails_closed(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(GENERATION, True))
        capacity = int(gate.pendingStatusCapacity())
        self.assertGreater(capacity, 0)
        for offset in range(capacity):
            gate.observeCommandStatus(
                command_status(2, command_id=COMMAND_ID + offset),
            )

        gate.observeCommandStatus(
            command_status(3, command_id=COMMAND_ID + capacity),
        )
        self.assertEqual(str(gate.snapshot().phase), "CORRELATION_OVERFLOW")
        gate.observeFinalCommand(final_command())
        gate.observeCommandStatus(command_status(2))
        gate.observeHardware(hardware_sample(1))
        gate.observeHardware(hardware_sample(2))
        self.assertFalse(gate.snapshot().ready)

    def test_requires_two_distinct_strictly_increasing_post_ack_frames(self):
        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(100))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(100))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(101))
        self.assertTrue(self.gate.snapshot().ready)
        self.assertEqual(str(self.gate.snapshot().phase), "READY")

    def test_replayed_pre_ack_sequence_does_not_count_after_ack(self):
        self.gate.observeHardware(hardware_sample(100))
        self.apply_brake_ack()

        self.gate.observeHardware(hardware_sample(100))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(101))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(102))
        self.assertTrue(self.gate.snapshot().ready)

    def test_bad_latest_hardware_resets_zero_evidence(self):
        bad_samples = (
            hardware_sample(11, fresh=False),
            hardware_sample(11, connected=False),
            hardware_sample(11, x_speed=1),
            hardware_sample(11, z_speed=-1),
            hardware_sample(11, brush_speed=1),
        )
        for sample in bad_samples:
            with self.subTest(sample=sample):
                gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
                self.assertTrue(gate.request(GENERATION, True))
                gate.observeFinalCommand(final_command())
                gate.observeCommandStatus(command_status(2))
                gate.observeHardware(hardware_sample(10))
                gate.observeHardware(sample)
                self.assertFalse(gate.snapshot().ready)
                gate.observeHardware(hardware_sample(12))
                self.assertFalse(gate.snapshot().ready)
                gate.observeHardware(hardware_sample(13))
                self.assertTrue(gate.snapshot().ready)

    def test_publisher_epoch_change_requires_two_new_zero_frames(self):
        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(100, publisher_epoch=1))
        self.gate.observeHardware(hardware_sample(101, publisher_epoch=1))
        self.assertTrue(self.gate.snapshot().ready)

        self.gate.observeHardware(hardware_sample(1, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(2, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

        self.gate.observeHardware(
            hardware_sample(3, publisher_epoch=2, fresh=False),
        )
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(4, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(5, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

        self.gate.observeHardware(hardware_sample(0, publisher_epoch=0))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(5, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(6, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(7, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

    def test_retired_publisher_epoch_cannot_override_current_blocker(self):
        self.apply_brake_ack()
        self.gate.observeHardware(
            hardware_sample(1, publisher_epoch=2, x_speed=1),
        )
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.blocker_code), "HARDWARE_NOT_STOPPED")

        self.gate.observeHardware(hardware_sample(900, publisher_epoch=1))
        self.gate.observeHardware(hardware_sample(901, publisher_epoch=1))
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.blocker_code), "HARDWARE_NOT_STOPPED")

        self.gate.observeHardware(hardware_sample(2, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(3, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

    def test_epoch_zero_and_retired_epoch_cannot_restore_readiness(self):
        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(10, publisher_epoch=2))
        self.gate.observeHardware(hardware_sample(11, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

        self.gate.observeHardware(hardware_sample(0, publisher_epoch=0))
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.blocker_code), "HARDWARE_NOT_FRESH")

        self.gate.observeHardware(hardware_sample(1000, publisher_epoch=1))
        self.gate.observeHardware(hardware_sample(1001, publisher_epoch=1))
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.blocker_code), "HARDWARE_NOT_FRESH")

        self.gate.observeHardware(hardware_sample(12, publisher_epoch=2))
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(hardware_sample(13, publisher_epoch=2))
        self.assertTrue(self.gate.snapshot().ready)

    def test_publisher_epoch_maximum_does_not_wrap_to_older_session(self):
        self.apply_brake_ack()
        self.gate.observeHardware(
            hardware_sample(
                1,
                publisher_epoch=MAX_UINT64,
                x_speed=1,
            ),
        )
        self.assertEqual(
            str(self.gate.snapshot().blocker_code),
            "HARDWARE_NOT_STOPPED",
        )

        self.gate.observeHardware(
            hardware_sample(900, publisher_epoch=MAX_UINT64 - 1),
        )
        self.gate.observeHardware(
            hardware_sample(901, publisher_epoch=MAX_UINT64 - 1),
        )
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.blocker_code), "HARDWARE_NOT_STOPPED")

        self.gate.observeHardware(
            hardware_sample(2, publisher_epoch=MAX_UINT64),
        )
        self.assertFalse(self.gate.snapshot().ready)
        self.gate.observeHardware(
            hardware_sample(3, publisher_epoch=MAX_UINT64),
        )
        self.assertTrue(self.gate.snapshot().ready)

    def test_mission_busy_blocks_ready_without_discarding_other_evidence(self):
        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(1))
        self.gate.observeHardware(hardware_sample(2))
        self.assertTrue(self.gate.snapshot().ready)

        self.gate.setMissionIdle(False)
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.phase), "MISSION_BUSY")
        self.assertEqual(str(snapshot.blocker_code), "MISSION_BUSY")

        self.gate.setMissionIdle(True)
        self.assertTrue(self.gate.snapshot().ready)

    def test_release_requires_matching_generation_and_clears_state(self):
        self.assertFalse(self.gate.release(GENERATION + 1))
        self.assertTrue(self.gate.snapshot().gate_active)
        self.assertTrue(self.gate.release(GENERATION))
        snapshot = self.gate.snapshot()
        self.assertFalse(snapshot.gate_active)
        self.assertFalse(snapshot.ready)
        self.assertEqual(str(snapshot.phase), "INACTIVE")

    def test_generation_must_increase_and_old_generation_evidence_is_ignored(self):
        self.assertEqual(int(self.gate.lastGeneration()), GENERATION)
        self.apply_brake_ack()
        self.gate.observeHardware(hardware_sample(1))
        self.gate.observeHardware(hardware_sample(2))
        self.assertTrue(self.gate.snapshot().ready)
        self.assertFalse(self.gate.request(GENERATION, False))
        self.assertFalse(self.gate.request(GENERATION - 1, False))
        self.assertTrue(self.gate.snapshot().ready)
        self.assertEqual(int(self.gate.snapshot().generation), GENERATION)

        self.assertTrue(self.gate.release(GENERATION))
        self.assertFalse(self.gate.request(GENERATION, True))
        self.assertFalse(self.gate.request(GENERATION - 1, True))
        self.assertFalse(self.gate.snapshot().gate_active)
        self.assertEqual(int(self.gate.lastGeneration()), GENERATION)

        next_generation = GENERATION + 1
        self.assertTrue(self.gate.request(next_generation, True))
        self.assertEqual(int(self.gate.lastGeneration()), next_generation)
        self.gate.observeFinalCommand(final_command())
        self.gate.observeCommandStatus(command_status(2))
        self.assertFalse(self.gate.snapshot().command_gate_applied)
        self.assertFalse(self.gate.snapshot().brake_acknowledged)

        self.gate.observeFinalCommand(
            final_command(next_generation, next_generation),
        )
        self.gate.observeCommandStatus(
            command_status(2, next_generation, next_generation),
        )
        self.gate.observeHardware(hardware_sample(1))
        self.gate.observeHardware(hardware_sample(2))
        self.assertTrue(self.gate.snapshot().ready)

    def test_max_generation_is_a_natural_fail_closed_boundary(self):
        gate = cppyy.gbl.cleanbot.mission.MaintenanceGate()
        self.assertTrue(gate.request(MAX_UINT64, True))
        self.assertEqual(int(gate.lastGeneration()), MAX_UINT64)
        self.assertTrue(gate.release(MAX_UINT64))
        self.assertFalse(gate.request(MAX_UINT64, True))
        self.assertFalse(gate.request(MAX_UINT64 - 1, True))
        self.assertFalse(gate.snapshot().gate_active)


if __name__ == "__main__":
    unittest.main()
