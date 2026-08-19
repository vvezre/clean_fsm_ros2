# 文件作用：验证 gateway runtime 相关契约、运行逻辑和边界条件。
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
GATEWAY = WORKSPACE / "src" / "cleanbot_gateway"
cppyy.add_include_path(str(CONTROL / "include"))
cppyy.add_include_path(str(CONTROL / "src"))
cppyy.add_include_path(str(GATEWAY / "include"))
cppyy.add_include_path(str(GATEWAY / "src"))
cppyy.cppdef(
    '#ifndef CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
    '#define CLEANBOT_CPPYY_JOYSTICK_MAPPER_LOADED\n'
    '#include "joystick_mapper.cpp"\n'
    '#endif\n'
    '#include "cloud_command.cpp"\n'
)


class GatewayRuntimeTest(unittest.TestCase):
    # 辅助方法：为 command 测试场景准备输入、执行操作或整理结果。
    def command(self, name="joystick_move", timestamp=100):
        command = cppyy.gbl.cleanbot.gateway.CloudCommandInput()
        command.command_id = "cmd_1"
        command.command = name
        command.timestamp_sec = timestamp
        command.distance = 50.0
        command.dir_x = 0.0
        command.dir_y = 1.0
        command.has_joystick_params = True
        return command

    # 测试作用：验证“joystick_uses_immediate_configured_speed”场景的契约、输出结果和边界行为。
    def test_joystick_uses_immediate_configured_speed(self):
        parameters = cppyy.gbl.cleanbot.gateway.CloudCommandParameters()
        parameters.joystick.max_linear_speed = 600
        translator = cppyy.gbl.cleanbot.gateway.CloudCommandTranslator(parameters)

        result = translator.translate(self.command(), 100)

        self.assertTrue(result.accepted)
        self.assertEqual(result.x_speed, 600)
        self.assertFalse(result.brake)

    # 测试作用：验证“release_brakes_and_stale_message_is_rejected”场景的契约、输出结果和边界行为。
    def test_release_brakes_and_stale_message_is_rejected(self):
        translator = cppyy.gbl.cleanbot.gateway.CloudCommandTranslator()
        released = self.command()
        released.distance = 0.0
        released.dir_y = 0.0
        self.assertTrue(translator.translate(released, 100).brake)

        stale = translator.translate(self.command(timestamp=90), 100)
        self.assertFalse(stale.accepted)
        self.assertEqual(stale.code, "JOYSTICK_COMMAND_EXPIRED")

    # 测试作用：验证“parking_maps_to_emergency_stop”场景的契约、输出结果和边界行为。
    def test_parking_maps_to_emergency_stop(self):
        translator = cppyy.gbl.cleanbot.gateway.CloudCommandTranslator()
        result = translator.translate(self.command(name="parking"), 100)

        self.assertTrue(result.accepted)
        self.assertTrue(result.brake)
        self.assertEqual(
            result.kind,
            cppyy.gbl.cleanbot.gateway.CloudCommandKind.kEmergencyStop,
        )


if __name__ == "__main__":
    unittest.main()
