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
PACKAGE = WORKSPACE / "src" / "cleanbot_hardware"
cppyy.add_include_path(str(PACKAGE / "include"))
cppyy.add_include_path(str(PACKAGE / "src"))
cppyy.cppdef('#include "gateway_readiness.cpp"\n')


class HardwareGatewayRuntimeTest(unittest.TestCase):
    def test_reconnect_requires_brake_ack_and_valid_status(self):
        handshake = cppyy.gbl.cleanbot.hardware.GatewayReadiness()
        handshake.on_connected(41)

        self.assertFalse(handshake.ready())
        self.assertFalse(handshake.observe_command_ack(40, 0, True))
        self.assertFalse(handshake.ready())
        self.assertTrue(handshake.observe_command_ack(41, 0, True))
        self.assertFalse(handshake.ready())

        self.assertTrue(handshake.observe_valid_status())
        self.assertTrue(handshake.ready())

    def test_status_before_brake_ack_still_requires_matching_ack(self):
        handshake = cppyy.gbl.cleanbot.hardware.GatewayReadiness()
        handshake.on_connected(12)

        self.assertTrue(handshake.observe_valid_status())
        self.assertFalse(handshake.ready())
        self.assertFalse(handshake.observe_command_ack(12, 3, True))
        self.assertFalse(handshake.ready())
        self.assertTrue(handshake.observe_command_ack(12, 0, True))
        self.assertTrue(handshake.ready())

    def test_disconnect_resets_ready_state(self):
        handshake = cppyy.gbl.cleanbot.hardware.GatewayReadiness()
        handshake.on_connected(7)
        handshake.observe_command_ack(7, 0, True)
        handshake.observe_valid_status()
        self.assertTrue(handshake.ready())

        handshake.on_disconnected()

        self.assertFalse(handshake.ready())
        self.assertEqual(
            int(handshake.state()),
            int(cppyy.gbl.cleanbot.hardware.GatewayReadinessState.kDisconnected),
        )


if __name__ == "__main__":
    unittest.main()
