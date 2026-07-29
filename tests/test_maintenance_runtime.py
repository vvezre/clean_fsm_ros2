import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"
HEADER = PACKAGE / "include" / "cleanbot_mission" / "maintenance_runtime.hpp"
SOURCE = PACKAGE / "src" / "maintenance_runtime.cpp"
GATE_SOURCE = PACKAGE / "src" / "maintenance_gate.cpp"
HARNESS = PACKAGE / "test" / "maintenance_runtime_windows_harness.cpp"
VSWHERE = (
    Path("C:/Program Files (x86)")
    / "Microsoft Visual Studio"
    / "Installer"
    / "vswhere.exe"
)


@unittest.skipUnless(
    shutil.which("cmd.exe") and VSWHERE.is_file(),
    "Windows MSVC runtime test requires Visual Studio",
)
class MaintenanceRuntimeWindowsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for path in (HEADER, SOURCE, GATE_SOURCE, HARNESS):
            if not path.is_file():
                raise AssertionError(f"maintenance runtime source is missing: {path}")

        installation = subprocess.run(
            [
                str(VSWHERE),
                "-latest",
                "-products",
                "*",
                "-requires",
                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                "-property",
                "installationPath",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        if not installation:
            raise unittest.SkipTest("Visual Studio C++ tools are unavailable")

        cls._temporary = tempfile.TemporaryDirectory()
        build_dir = Path(cls._temporary.name)
        cls._executable = build_dir / "maintenance_runtime_test.exe"
        vcvars = Path(installation) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
        include = PACKAGE / "include"
        compile_command = (
            "@echo off\n"
            f'call "{vcvars}" >nul && '
            f'cl /nologo /std:c++17 /EHsc /W4 '
            f'/I"{include}" '
            f'"{HARNESS}" "{GATE_SOURCE}" "{SOURCE}" '
            f'/Fe:"{cls._executable}"\n'
        )
        compile_script = build_dir / "compile-maintenance-runtime.bat"
        compile_script.write_text(compile_command, encoding="utf-8")
        compiled = subprocess.run(
            ["cmd.exe", "/d", "/c", str(compile_script)],
            cwd=build_dir,
            capture_output=True,
            text=True,
        )
        if compiled.returncode != 0:
            raise AssertionError(
                "maintenance runtime harness failed to compile\n"
                + compiled.stdout
                + compiled.stderr
            )

    @classmethod
    def tearDownClass(cls):
        temporary = getattr(cls, "_temporary", None)
        if temporary is not None:
            temporary.cleanup()

    def assert_case(self, name):
        completed = subprocess.run(
            [str(self._executable), name],
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stdout + completed.stderr,
        )

    def test_initial_state_is_fail_closed(self):
        self.assert_case("initial")

    def test_inactive_restore_opens_admission(self):
        self.assert_case("inactive")

    def test_active_restore_preserves_owner_and_has_no_evidence(self):
        self.assert_case("active")

    def test_missing_load_latches_store_fault(self):
        self.assert_case("missing")

    def test_invalid_load_latches_store_fault(self):
        self.assert_case("invalid")

    def test_unsupported_load_latches_store_fault(self):
        self.assert_case("unsupported")

    def test_io_load_latches_store_fault(self):
        self.assert_case("io")

    def test_malformed_ok_record_latches_store_fault(self):
        self.assert_case("malformed")

    def test_second_initialize_is_idempotent_without_reread(self):
        self.assert_case("duplicate")

    def test_mission_idle_forwards_after_restore(self):
        self.assert_case("mission_idle")

    def test_gate_restore_failure_latches_store_fault(self):
        self.assert_case("restore_failure")


if __name__ == "__main__":
    unittest.main()
