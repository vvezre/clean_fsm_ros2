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
cppyy.add_include_path(str(PACKAGE / "include"))
cppyy.add_include_path(str(PACKAGE / "src"))
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
