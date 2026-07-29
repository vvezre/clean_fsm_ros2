import ctypes
import math
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
PACKAGE = WORKSPACE / "src" / "cleanbot_control"
COMMON_PACKAGE = WORKSPACE / "src" / "cleanbot_common"
cppyy.add_include_path(str(PACKAGE / "include"))
cppyy.add_include_path(str(PACKAGE / "src"))
cppyy.add_include_path(str(COMMON_PACKAGE / "include"))
cppyy.cppdef(
    '#include "tracking_core.cpp"\n'
    '#include "command_arbiter_core.cpp"\n'
    '#ifndef CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
    '#define CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
    '#include "joystick_mapper.cpp"\n'
    '#endif\n'
)


class TrackingCoreRuntimeTest(unittest.TestCase):
    def test_heading_wrap_and_finish_rules(self):
        normalize = cppyy.gbl.cleanbot.control.normalize_heading_delta
        finish = cppyy.gbl.cleanbot.control.should_finish_point_to_point

        self.assertAlmostEqual(normalize(1.0, 359.0), 2.0)
        self.assertAlmostEqual(normalize(359.0, 1.0), -2.0)
        self.assertTrue(finish(0.02, 1.0, 2.0, 0.03, 0.30))
        self.assertTrue(finish(1.0, -0.01, 0.20, 0.03, 0.30))
        self.assertFalse(finish(1.0, -0.01, 0.40, 0.03, 0.30))

    def test_kalman_initializes_then_smooths_measurements(self):
        params = cppyy.gbl.cleanbot.control.TrackingParameters()
        params.process_noise = 0.2
        params.measurement_noise = 1.0
        filter_ = cppyy.gbl.cleanbot.control.RtkKalmanFilter2D(params)

        first = filter_.update(32.0, 118.0, 1.0)
        second = filter_.update(32.0, 118.000010, 1.1)

        self.assertFalse(first.filtered)
        self.assertTrue(second.filtered)
        self.assertGreater(second.lon, 118.0)
        self.assertLess(second.lon, 118.000010)

    def test_controller_calculates_cte_and_limits_output(self):
        params = cppyy.gbl.cleanbot.control.TrackingParameters()
        params.heading_gain = 10.0
        params.cte_gain = 1000.0
        params.max_z_speed = 500
        controller = cppyy.gbl.cleanbot.control.StraightLinePController(params)

        command = controller.compute(
            32.0, 118.0,
            32.0001, 118.0,
            32.00005, 118.00002,
            0.0, 0.0,
            32.00005, 118.00002,
        )

        self.assertGreater(command.cte_m, 0.0)
        self.assertEqual(command.z_speed, -500)
        self.assertGreater(command.distance_to_target_m, 0.0)


class CommandArbiterRuntimeTest(unittest.TestCase):
    def command(
        self, source, command_id, speed, operator_intent=False, brake=False, stamp_ms=0
    ):
        command = cppyy.gbl.cleanbot.control.ControlCommand()
        command.source = source
        command.command_id = command_id
        command.stamp_ms = stamp_ms
        command.active = True
        command.operator_intent = operator_intent
        command.status = 1
        command.x_speed = speed
        command.brake = brake
        return command

    def publisher_identity(self, implementation_identifier, gid):
        identity = cppyy.gbl.cleanbot.common.PublisherIdentity()
        identity.implementation_identifier = implementation_identifier
        for value in gid:
            identity.gid.push_back(value)
        return identity

    def test_manual_replaces_mission_then_expires_to_brake(self):
        params = cppyy.gbl.cleanbot.control.ArbiterParameters()
        params.manual_lease_ms = 800
        params.mission_lease_ms = 1000
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore(params)
        source = cppyy.gbl.cleanbot.control.CommandSource

        arbiter.update(source.kMission, self.command("mission", 1, 200), 0)
        arbiter.update(source.kManual, self.command("manual", 2, 100, True), 100)
        self.assertEqual(arbiter.output(200).source, "manual")
        expired = arbiter.output(950)
        self.assertEqual(expired.source, "idle_brake")
        self.assertTrue(expired.brake)

    def test_old_mission_heartbeat_cannot_return_after_manual_mode_switch(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        arbiter.update(source.kMission, self.command("mission-start", 1, 200, True), 0)
        arbiter.update(source.kManual, self.command("manual", 2, 100, True), 100)

        accepted = arbiter.update(
            source.kMission,
            self.command("old-mission-heartbeat", 3, 200, False),
            200,
        )

        self.assertFalse(accepted)
        expired = arbiter.output(950)
        self.assertEqual(expired.source, "idle_brake")
        self.assertTrue(expired.brake)

        self.assertTrue(
            arbiter.update(
                source.kMission,
                self.command("new-mission", 4, 200, True),
                1000,
            )
        )
        self.assertEqual(arbiter.output(1000).source, "new-mission")

    def test_emergency_requires_newer_operator_command_and_clears_brush(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        arbiter.set_brush(True, 60, True)

        emergency = self.command("emergency", 100, 0, True, True)
        arbiter.update(source.kEmergency, emergency, 10)
        stopped = arbiter.output(20)
        self.assertTrue(stopped.brake)
        self.assertEqual(stopped.priority, 100)
        self.assertEqual(stopped.brush_speed, 0)
        self.assertTrue(arbiter.software_stopped())

        arbiter.update(source.kMission, self.command("background", 101, 200), 30)
        arbiter.update(source.kManual, self.command("stale", 99, 100, True), 40)
        self.assertTrue(arbiter.software_stopped())

        arbiter.update(source.kManual, self.command("fresh", 101, 100, True), 50)
        resumed = arbiter.output(60)
        self.assertFalse(arbiter.software_stopped())
        self.assertEqual(resumed.source, "fresh")
        self.assertEqual(resumed.brush_speed, 0)

    def test_brush_is_merged_without_starting_vehicle_motion(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        arbiter.set_brush(True, 70, True)

        output = arbiter.output(0)

        self.assertFalse(output.brake)
        self.assertEqual(output.x_speed, 0)
        self.assertEqual(output.brush_speed, 70)

    def test_timestamp_orders_operations_from_different_publishers(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        arbiter.update(
            source.kEmergency,
            self.command("emergency", 500, 0, True, True, 1000),
            1000,
        )

        arbiter.update(
            source.kManual,
            self.command("stale", 999, 100, True, False, 999),
            1100,
        )
        self.assertTrue(arbiter.software_stopped())

        arbiter.update(
            source.kMission,
            self.command("fresh-task", 1, 200, True, False, 1001),
            1200,
        )
        self.assertFalse(arbiter.software_stopped())
        self.assertEqual(arbiter.output(1200).source, "fresh-task")

    def assert_maintenance_output(self, output, generation):
        self.assertEqual(output.request_id, generation)
        self.assertEqual(output.source, "maintenance_gate")
        self.assertEqual(output.priority, 80)
        self.assertTrue(output.active)
        self.assertFalse(output.operator_intent)
        self.assertEqual(output.status, 0)
        self.assertEqual(output.x_speed, 0)
        self.assertEqual(output.z_speed, 0)
        self.assertEqual(output.steering_offset, 0)
        self.assertEqual(output.brush_speed, 0)
        self.assertEqual(output.target_distance, 0)
        self.assertEqual(output.target_rotation, 0)
        self.assertEqual(output.heading_deg, 0.0)
        self.assertTrue(output.brake)
        self.assertFalse(output.charge)

    def test_maintenance_clears_existing_and_rejects_new_motion_and_brush(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        arbiter.update(
            source.kEmergency, self.command("emergency-monitor", 1, 250), 0
        )
        arbiter.update(source.kMission, self.command("mission", 2, 200), 0)
        arbiter.update(source.kSafety, self.command("safety", 3, 100), 1)
        arbiter.update(source.kManual, self.command("manual", 4, 150), 1)
        arbiter.update(source.kVision, self.command("vision", 5, 125), 1)
        arbiter.set_brush(True, 70, True)

        self.assertTrue(arbiter.set_maintenance(True, 21))
        self.assertFalse(
            arbiter.update(
                source.kManual,
                self.command("manual-during-maintenance", 6, 300, True),
                2,
            )
        )
        arbiter.set_brush(True, 90, True)
        self.assert_maintenance_output(arbiter.output(3), 21)

        self.assertTrue(arbiter.set_maintenance(False, 21))
        released = arbiter.output(4)
        self.assertEqual(released.source, "idle_brake")
        self.assertTrue(released.brake)
        self.assertEqual(released.brush_speed, 0)

    def test_emergency_during_maintenance_remains_latched_after_release(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        self.assertTrue(arbiter.set_maintenance(True, 22))

        emergency = self.command("emergency", 100, 0, True, True, 1000)
        self.assertTrue(arbiter.update(source.kEmergency, emergency, 1))
        self.assertTrue(arbiter.software_stopped())
        self.assert_maintenance_output(arbiter.output(2), 22)

        self.assertTrue(arbiter.set_maintenance(False, 22))
        self.assertEqual(arbiter.output(3).source, "software_emergency_stop")

    def test_maintenance_generation_is_correlated_and_monotonic(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        self.assertFalse(arbiter.set_maintenance(True, 0))
        self.assertTrue(arbiter.set_maintenance(True, 30))
        self.assertTrue(arbiter.set_maintenance(True, 30))
        self.assertFalse(arbiter.set_maintenance(False, 29))
        self.assertTrue(arbiter.set_maintenance(True, 31))
        self.assert_maintenance_output(arbiter.output(0), 31)
        self.assertTrue(arbiter.set_maintenance(False, 31))
        self.assertEqual(arbiter.maintenance_generation(), 31)
        self.assertFalse(arbiter.set_maintenance(False, 31))
        self.assertFalse(arbiter.set_maintenance(True, 30))
        self.assertFalse(arbiter.set_maintenance(True, 31))
        self.assertTrue(arbiter.set_maintenance(True, 32))

        maximum = (1 << 64) - 1
        max_arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        self.assertTrue(max_arbiter.set_maintenance(True, maximum))
        self.assertTrue(max_arbiter.set_maintenance(False, maximum))
        self.assertFalse(max_arbiter.set_maintenance(True, maximum))

    def test_active_generation_switch_stays_clamped_and_resets_operator_mode(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        self.assertTrue(
            arbiter.update(
                source.kManual,
                self.command("manual-operator-mode", 1, 150, True),
                1,
            )
        )
        arbiter.set_brush(True, 80, True)

        self.assertTrue(arbiter.set_maintenance(True, 50))
        self.assert_maintenance_output(arbiter.output(2), 50)
        self.assertFalse(
            arbiter.update(
                source.kMission,
                self.command("mission-during-generation-50", 2, 200),
                2,
            )
        )
        arbiter.set_brush(True, 90, True)

        self.assertTrue(arbiter.set_maintenance(True, 51))
        self.assertTrue(arbiter.maintenance_active())
        self.assertEqual(arbiter.maintenance_generation(), 51)
        self.assert_maintenance_output(arbiter.output(3), 51)
        self.assertFalse(
            arbiter.update(
                source.kVision,
                self.command("vision-during-generation-51", 3, 100),
                3,
            )
        )
        arbiter.set_brush(True, 95, True)
        self.assert_maintenance_output(arbiter.output(4), 51)

        self.assertTrue(arbiter.set_maintenance(False, 51))
        released = arbiter.output(5)
        self.assertEqual(released.source, "idle_brake")
        self.assertEqual(released.brush_speed, 0)

        self.assertTrue(
            arbiter.update(
                source.kMission,
                self.command("mission-after-maintenance", 4, 175),
                6,
            )
        )
        resumed = arbiter.output(6)
        self.assertEqual(resumed.source, "mission-after-maintenance")
        self.assertEqual(resumed.x_speed, 175)
        self.assertEqual(resumed.brush_speed, 0)

    def test_existing_software_stop_survives_maintenance(self):
        arbiter = cppyy.gbl.cleanbot.control.CommandArbiterCore()
        source = cppyy.gbl.cleanbot.control.CommandSource
        emergency = self.command("emergency", 100, 0, True, True, 1000)
        self.assertTrue(arbiter.update(source.kEmergency, emergency, 1))

        self.assertTrue(arbiter.set_maintenance(True, 40))
        self.assert_maintenance_output(arbiter.output(2), 40)
        self.assertTrue(arbiter.set_maintenance(False, 40))

        self.assertTrue(arbiter.software_stopped())
        self.assertEqual(arbiter.output(3).source, "software_emergency_stop")

    def test_maintenance_cache_rejects_mismatched_release_without_mutation(self):
        cache = cppyy.gbl.cleanbot.control.MaintenanceGateCache()
        self.assertTrue(cache.update(True, 10))
        self.assertTrue(cache.active())
        self.assertEqual(cache.generation(), 10)

        self.assertFalse(cache.update(False, 11))
        self.assertTrue(cache.active())
        self.assertEqual(cache.generation(), 10)
        self.assertTrue(cache.update(False, 10))
        self.assertFalse(cache.active())
        self.assertEqual(cache.generation(), 10)

    def test_maintenance_cache_rejects_zero_old_and_released_generations(self):
        cache = cppyy.gbl.cleanbot.control.MaintenanceGateCache()
        self.assertFalse(cache.update(True, 0))
        self.assertFalse(cache.update(False, 0))
        self.assertFalse(cache.update(False, 5))
        self.assertFalse(cache.has_state())

        self.assertTrue(cache.update(True, 5))
        self.assertTrue(cache.update(True, 5))
        self.assertTrue(cache.update(False, 5))
        self.assertFalse(cache.update(False, 5))
        self.assertFalse(cache.update(True, 5))
        self.assertFalse(cache.update(True, 4))
        self.assertFalse(cache.update(False, 6))
        self.assertEqual(cache.generation(), 5)
        self.assertFalse(cache.active())

        self.assertTrue(cache.update(True, 6))
        self.assertTrue(cache.active())
        self.assertEqual(cache.generation(), 6)

    def test_publisher_restart_forces_one_reissue_and_retires_old_session(self):
        coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator()
        )
        publisher_a = self.publisher_identity("rmw-a", [1, 2, 3])
        publisher_b = self.publisher_identity("rmw-a", [4, 5, 6])

        first = coordinator.observe(True, 10, publisher_a)
        self.assertTrue(first.accepted)
        self.assertTrue(first.gate_active)
        self.assertEqual(first.generation, 10)
        self.assertTrue(first.session_changed)
        self.assertTrue(first.force_republish)

        repeated_a = coordinator.observe(True, 10, publisher_a)
        self.assertTrue(repeated_a.accepted)
        self.assertFalse(repeated_a.session_changed)
        self.assertFalse(repeated_a.force_republish)

        first_b = coordinator.observe(True, 10, publisher_b)
        self.assertTrue(first_b.accepted)
        self.assertTrue(first_b.session_changed)
        self.assertTrue(first_b.force_republish)

        repeated_b = coordinator.observe(True, 10, publisher_b)
        self.assertTrue(repeated_b.accepted)
        self.assertFalse(repeated_b.session_changed)
        self.assertFalse(repeated_b.force_republish)

        retired_a = coordinator.observe(True, 11, publisher_a)
        self.assertFalse(retired_a.accepted)
        self.assertTrue(retired_a.gate_active)
        self.assertEqual(retired_a.generation, 10)

    def test_invalid_unknown_state_does_not_poison_current_publisher(self):
        coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator()
        )
        publisher_a = self.publisher_identity("rmw-a", [1])
        publisher_b = self.publisher_identity("rmw-a", [2])
        publisher_c = self.publisher_identity("rmw-a", [3])
        coordinator.observe(True, 10, publisher_a)
        coordinator.observe(True, 10, publisher_b)

        old_active = coordinator.observe(True, 9, publisher_c)
        mismatched_release = coordinator.observe(False, 11, publisher_c)
        self.assertFalse(old_active.accepted)
        self.assertFalse(mismatched_release.accepted)
        self.assertTrue(mismatched_release.gate_active)
        self.assertEqual(mismatched_release.generation, 10)

        current_b = coordinator.observe(True, 10, publisher_b)
        self.assertTrue(current_b.accepted)
        self.assertFalse(current_b.session_changed)
        self.assertFalse(current_b.force_republish)

        higher_active = coordinator.observe(True, 11, publisher_c)
        self.assertTrue(higher_active.accepted)
        self.assertTrue(higher_active.session_changed)
        self.assertTrue(higher_active.force_republish)
        self.assertEqual(higher_active.generation, 11)
        self.assertFalse(coordinator.observe(True, 12, publisher_b).accepted)

    def test_new_publisher_can_commit_exact_release_without_reissue(self):
        coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator()
        )
        publisher_a = self.publisher_identity("rmw-a", [1])
        publisher_b = self.publisher_identity("rmw-a", [2])
        coordinator.observe(True, 20, publisher_a)

        released = coordinator.observe(False, 20, publisher_b)

        self.assertTrue(released.accepted)
        self.assertFalse(released.gate_active)
        self.assertEqual(released.generation, 20)
        self.assertTrue(released.session_changed)
        self.assertFalse(released.force_republish)
        repeated_release = coordinator.observe(False, 20, publisher_b)
        self.assertTrue(repeated_release.accepted)
        self.assertFalse(repeated_release.session_changed)
        self.assertFalse(repeated_release.force_republish)

    def test_untrackable_identity_cannot_release_or_poison_tracked_state(self):
        coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator()
        )
        invalid = self.publisher_identity("rmw-a", [0, 0, 0])
        publisher_a = self.publisher_identity("rmw-a", [1, 2, 3])

        initial_active = coordinator.observe(True, 30, invalid)
        self.assertTrue(initial_active.accepted)
        self.assertFalse(initial_active.session_changed)
        self.assertFalse(initial_active.force_republish)
        self.assertFalse(coordinator.observe(False, 30, invalid).accepted)
        self.assertTrue(coordinator.active())
        self.assertEqual(coordinator.generation(), 30)

        first_tracked = coordinator.observe(True, 30, publisher_a)
        self.assertTrue(first_tracked.accepted)
        self.assertTrue(first_tracked.session_changed)
        self.assertTrue(first_tracked.force_republish)
        self.assertFalse(coordinator.observe(True, 31, invalid).accepted)
        self.assertFalse(coordinator.observe(False, 30, invalid).accepted)
        self.assertTrue(coordinator.active())
        self.assertEqual(coordinator.generation(), 30)

        current = coordinator.observe(True, 30, publisher_a)
        self.assertTrue(current.accepted)
        self.assertFalse(current.session_changed)

        inactive_coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator()
        )
        self.assertFalse(inactive_coordinator.observe(False, 1, invalid).accepted)
        self.assertFalse(inactive_coordinator.has_state())

    def test_tracker_capacity_exhaustion_is_atomic_and_fails_closed(self):
        coordinator = (
            cppyy.gbl.cleanbot.control.MaintenancePublisherCoordinator(0)
        )
        publisher_a = self.publisher_identity("rmw-a", [1])
        publisher_b = self.publisher_identity("rmw-a", [2])
        coordinator.observe(True, 40, publisher_a)

        higher = coordinator.observe(True, 41, publisher_b)
        release = coordinator.observe(False, 40, publisher_b)
        self.assertFalse(higher.accepted)
        self.assertFalse(release.accepted)
        self.assertTrue(coordinator.active())
        self.assertEqual(coordinator.generation(), 40)

        current = coordinator.observe(True, 40, publisher_a)
        self.assertTrue(current.accepted)
        self.assertFalse(current.session_changed)
        self.assertFalse(current.force_republish)


class JoystickMapperRuntimeTest(unittest.TestCase):
    def test_full_scale_axes_and_dead_zone(self):
        mapper = cppyy.gbl.cleanbot.control.JoystickMapper()
        self.assertEqual(mapper.map(0.0, 1.0).x_speed, 350)
        self.assertEqual(mapper.map(0.0, -1.0).x_speed, -350)
        self.assertEqual(mapper.map(-1.0, 0.0).steering_offset, 1000)
        self.assertEqual(mapper.map(1.0, 0.0).steering_offset, -1000)
        self.assertTrue(mapper.map(0.01, 0.01).brake)


if __name__ == "__main__":
    unittest.main()
