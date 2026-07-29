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
HEADER = PACKAGE / "include" / "cleanbot_mission" / "mission_state_machine.hpp"
SOURCE = PACKAGE / "src" / "mission_state_machine.cpp"
CORE_LOADED = False


def load_core(test_case):
    global CORE_LOADED
    test_case.assertTrue(HEADER.is_file(), "mission state-machine header is missing")
    test_case.assertTrue(SOURCE.is_file(), "mission state-machine source is missing")
    if CORE_LOADED:
        return
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    cppyy.cppdef('#include "mission_state_machine.cpp"')
    CORE_LOADED = True


class MissionStateMachineRuntimeTest(unittest.TestCase):
    def setUp(self):
        load_core(self)
        self.state = cppyy.gbl.cleanbot.mission.MissionState
        self.machine = cppyy.gbl.cleanbot.mission.MissionStateMachine()

    def test_rejects_empty_plan_and_executes_matching_turn_and_tracking_events(self):
        self.assertFalse(self.machine.start(0))
        self.assertTrue(self.machine.start(2))
        self.assertEqual(self.machine.state(), self.state.kPreparing)
        self.assertEqual(self.machine.current_segment(), 0)

        self.assertTrue(self.machine.begin_turn(101))
        self.assertEqual(self.machine.state(), self.state.kTurning)
        self.assertFalse(self.machine.complete_turn(999))
        self.assertTrue(self.machine.complete_turn(101))
        self.assertEqual(self.machine.state(), self.state.kPreparing)

        self.assertTrue(self.machine.begin_tracking(501))
        self.assertFalse(self.machine.complete_tracking(500))
        self.assertTrue(self.machine.complete_tracking(501))
        self.assertEqual(self.machine.current_segment(), 1)
        self.assertEqual(self.machine.state(), self.state.kPreparing)

        self.assertTrue(self.machine.begin_tracking(502))
        self.assertTrue(self.machine.complete_tracking(502))
        self.assertEqual(self.machine.current_segment(), 2)
        self.assertEqual(self.machine.state(), self.state.kCompleted)
        self.assertTrue(self.machine.terminal())

    def test_pause_and_resume_restart_the_current_segment(self):
        self.assertTrue(self.machine.start(1))
        self.assertTrue(self.machine.begin_tracking(700))
        self.assertTrue(self.machine.pause())
        self.assertEqual(self.machine.state(), self.state.kPaused)
        self.assertTrue(self.machine.resume())
        self.assertEqual(self.machine.state(), self.state.kPreparing)
        self.assertEqual(self.machine.current_segment(), 0)
        self.assertFalse(self.machine.complete_tracking(700))

    def test_rtk_recovery_restarts_the_current_segment(self):
        self.assertTrue(self.machine.start(1))
        self.assertTrue(self.machine.begin_tracking(800))
        self.assertTrue(self.machine.begin_rtk_recovery())
        self.assertEqual(self.machine.state(), self.state.kRtkRecovering)
        self.assertTrue(self.machine.recover_rtk())
        self.assertEqual(self.machine.state(), self.state.kPreparing)
        self.assertEqual(self.machine.current_segment(), 0)
        self.assertFalse(self.machine.complete_tracking(800))

    def test_cancel_and_fail_are_terminal(self):
        self.assertTrue(self.machine.start(1))
        self.assertTrue(self.machine.cancel())
        self.assertEqual(self.machine.state(), self.state.kCancelled)
        self.assertTrue(self.machine.terminal())

        failed = cppyy.gbl.cleanbot.mission.MissionStateMachine()
        self.assertTrue(failed.start(1))
        self.assertTrue(failed.fail())
        self.assertEqual(failed.state(), self.state.kFailed)
        self.assertTrue(failed.terminal())


if __name__ == "__main__":
    unittest.main()
