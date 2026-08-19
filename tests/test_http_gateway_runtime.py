# 文件作用：验证 http gateway runtime 相关契约、运行逻辑和边界条件。
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
CONTROL = WORKSPACE / "src" / "cleanbot_control"
PACKAGE = WORKSPACE / "src" / "cleanbot_http"
HEADER = PACKAGE / "include" / "cleanbot_http" / "http_control_router.hpp"
SOURCE = PACKAGE / "src" / "http_control_router.cpp"
RUNTIME_AVAILABLE = HEADER.is_file() and SOURCE.is_file()

if RUNTIME_AVAILABLE:
    cppyy.add_include_path(str(CONTROL / "include"))
    cppyy.add_include_path(str(CONTROL / "src"))
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    cppyy.cppdef(
        '#ifndef CLEANBOT_CPPYY_HTTP_ROUTER_LOADED\n'
        '#define CLEANBOT_CPPYY_HTTP_ROUTER_LOADED\n'
        '#ifndef CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
        '#define CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
        '#include "joystick_mapper.cpp"\n'
        '#endif\n'
        '#include "joystick_sequence_guard.cpp"\n'
        '#include "http_control_router.cpp"\n'
        '#endif\n'
    )


@unittest.skipUnless(RUNTIME_AVAILABLE, "HTTP control router is not implemented")
class HttpGatewayRuntimeTest(unittest.TestCase):
    # 测试初始化：为每个用例创建相互隔离的初始状态和输入。
    def setUp(self):
        parameters = cppyy.gbl.cleanbot.control.JoystickParameters()
        parameters.max_linear_speed = 600
        self.router = cppyy.gbl.cleanbot.http.HttpControlRouter(parameters)

    # 测试作用：验证“old_joystick_url_maps_to_manual_command”场景的契约、输出结果和边界行为。
    def test_old_joystick_url_maps_to_manual_command(self):
        result = self.router.route(
            "GET", "/vehicle/joystickMove/65/0/1"
        )

        self.assertEqual(result.status_code, 200)
        self.assertEqual(str(result.body), "1")
        self.assertEqual(str(result.content_type), "text/plain; charset=utf-8")
        self.assertEqual(
            result.action,
            cppyy.gbl.cleanbot.http.HttpControlAction.kManual,
        )
        self.assertEqual(result.x_speed, 600)
        self.assertEqual(result.steering_offset, 0)
        self.assertFalse(result.brake)

    # 测试作用：验证“release_and_dead_zone_request_brake”场景的契约、输出结果和边界行为。
    def test_release_and_dead_zone_request_brake(self):
        released = self.router.route(
            "GET", "/vehicle/joystickMove/0/0/0"
        )
        dead_zone = self.router.route(
            "GET", "/vehicle/joystickMove/20/0.01/0.01"
        )

        self.assertTrue(released.brake)
        self.assertTrue(dead_zone.brake)
        self.assertEqual(released.action, cppyy.gbl.cleanbot.http.HttpControlAction.kManual)

    # 测试作用：验证“parking_maps_to_emergency_stop”场景的契约、输出结果和边界行为。
    def test_parking_maps_to_emergency_stop(self):
        result = self.router.route("GET", "/vehicle/parking")

        self.assertEqual(result.status_code, 200)
        self.assertEqual(str(result.body), "1")
        self.assertEqual(
            result.action,
            cppyy.gbl.cleanbot.http.HttpControlAction.kEmergencyStop,
        )
        self.assertTrue(result.brake)

    # 测试作用：验证“invalid_joystick_values_are_rejected_without_action”场景的契约、输出结果和边界行为。
    def test_invalid_joystick_values_are_rejected_without_action(self):
        invalid_targets = (
            "/vehicle/joystickMove",
            "/vehicle/joystickMove/20/0",
            "/vehicle/joystickMove/not-a-number/0/1",
            "/vehicle/joystickMove/nan/0/1",
            "/vehicle/joystickMove/101/0/1",
            "/vehicle/joystickMove/20/1.01/0",
            "/vehicle/joystickMove/20/0/-1.01",
        )

        for target in invalid_targets:
            result = self.router.route("GET", target)
            self.assertEqual(result.status_code, 400, target)
            self.assertEqual(
                result.action,
                cppyy.gbl.cleanbot.http.HttpControlAction.kNone,
                target,
            )
            self.assertIn("JOYSTICK_PARAMS_INVALID", str(result.body))

    # 测试作用：验证“unknown_path_wrong_method_and_options_are_explicit”场景的契约、输出结果和边界行为。
    def test_unknown_path_wrong_method_and_options_are_explicit(self):
        missing = self.router.route("GET", "/vehicle/not-found")
        wrong_method = self.router.route("POST", "/vehicle/parking")
        options = self.router.route("OPTIONS", "/vehicle/parking")

        self.assertEqual(missing.status_code, 404)
        self.assertEqual(wrong_method.status_code, 405)
        self.assertEqual(options.status_code, 204)
        self.assertEqual(options.action, cppyy.gbl.cleanbot.http.HttpControlAction.kNone)


class HttpGatewayRuntimeSourceTest(unittest.TestCase):
    # 测试作用：验证“router_source_exists”场景的契约、输出结果和边界行为。
    def test_router_source_exists(self):
        self.assertTrue(HEADER.is_file())
        self.assertTrue(SOURCE.is_file())


if __name__ == "__main__":
    unittest.main()
