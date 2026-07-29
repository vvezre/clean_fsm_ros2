import ctypes
import os
import sysconfig
import unittest
from pathlib import Path


CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
    os.add_dll_directory(str(CPPYY_BACKEND_BIN))
    os.environ["PATH"] = (
        str(CPPYY_BACKEND_BIN) + os.pathsep + os.environ.get("PATH", "")
    )
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
HTTP = WORKSPACE / "src" / "cleanbot_http"
cppyy.add_include_path(str(CONTROL / "include"))
cppyy.add_include_path(str(CONTROL / "src"))
cppyy.add_include_path(str(HTTP / "include"))
cppyy.add_include_path(str(HTTP / "src"))
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


class HttpControlRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.router = cppyy.gbl.cleanbot.http.HttpControlRouter()

    def route(self, target):
        return self.router.route("GET", target)

    def test_stale_sequence_does_not_create_manual_action(self):
        first = self.route(
            "/vehicle/joystickMove/50/0/1?sessionId=session-a&sequence=10"
        )
        newer = self.route(
            "/vehicle/joystickMove/50/0/1?sessionId=session-a&sequence=12"
        )
        stale = self.route(
            "/vehicle/joystickMove/50/0/1?sessionId=session-a&sequence=11"
        )

        self.assertEqual(first.status_code, 200)
        self.assertEqual(newer.status_code, 200)
        self.assertEqual(stale.status_code, 409)
        self.assertEqual(
            stale.action,
            cppyy.gbl.cleanbot.http.HttpControlAction.kNone,
        )

    def test_release_prevents_delayed_movement_from_overwriting_it(self):
        released = self.route(
            "/vehicle/joystickMove/0/0/0?sessionId=session-b&sequence=20"
        )
        delayed = self.route(
            "/vehicle/joystickMove/80/0/1?sessionId=session-b&sequence=19"
        )

        self.assertEqual(released.status_code, 200)
        self.assertTrue(released.brake)
        self.assertEqual(delayed.status_code, 409)

    def test_legacy_and_partial_sequence_parameters(self):
        legacy = self.route("/vehicle/joystickMove/50/0/1")
        missing_sequence = self.route(
            "/vehicle/joystickMove/50/0/1?sessionId=session-c"
        )
        missing_session = self.route(
            "/vehicle/joystickMove/50/0/1?sequence=1"
        )

        self.assertEqual(legacy.status_code, 200)
        self.assertEqual(missing_sequence.status_code, 400)
        self.assertEqual(missing_session.status_code, 400)


if __name__ == "__main__":
    unittest.main()
