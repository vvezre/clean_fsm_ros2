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
PACKAGE = WORKSPACE / "src" / "cleanbot_config"
REGISTRY_HEADER = PACKAGE / "include" / "cleanbot_config" / "config_registry.hpp"
REGISTRY_SOURCE = PACKAGE / "src" / "config_registry.cpp"
RUNTIME_AVAILABLE = REGISTRY_HEADER.is_file() and REGISTRY_SOURCE.is_file()
if RUNTIME_AVAILABLE:
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    cppyy.cppdef('#include "config_registry.cpp"\n')


class ConfigRegistryRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.registry = (
            cppyy.gbl.cleanbot.config.ConfigRegistry() if RUNTIME_AVAILABLE else None
        )

    def test_registry_source_exists(self):
        self.assertTrue(REGISTRY_HEADER.is_file())
        self.assertTrue(REGISTRY_SOURCE.is_file())

    @unittest.skipUnless(RUNTIME_AVAILABLE, "configuration registry implementation is not present")
    def test_base_forward_speed_has_approved_default_range_and_apply_policy(self):
        definition = self.registry.find("motion.base_forward_speed")

        self.assertTrue(bool(definition))
        self.assertEqual(definition.default_value, "350")
        self.assertEqual(definition.minimum, 50.0)
        self.assertEqual(definition.maximum, 600.0)
        self.assertEqual(
            int(definition.apply_policy),
            int(cppyy.gbl.cleanbot.config.ApplyPolicy.kImmediate),
        )
        self.assertTrue(self.registry.validate_value("motion.base_forward_speed", "600").valid)
        self.assertFalse(self.registry.validate_value("motion.base_forward_speed", "601").valid)

    @unittest.skipUnless(RUNTIME_AVAILABLE, "configuration registry implementation is not present")
    def test_http_gateway_preserves_old_service_listener_defaults(self):
        defaults = self.registry.default_values()

        self.assertEqual(defaults["http.enabled"], "true")
        self.assertEqual(defaults["http.listen_address"], "0.0.0.0")
        self.assertEqual(defaults["http.port"], "7899")
        self.assertTrue(self.registry.validate_value("http.port", "1").valid)
        self.assertTrue(self.registry.validate_value("http.port", "65535").valid)
        self.assertFalse(self.registry.validate_value("http.port", "0").valid)
        self.assertFalse(self.registry.validate_value("http.port", "65536").valid)

    @unittest.skipUnless(RUNTIME_AVAILABLE, "configuration registry implementation is not present")
    def test_required_hardware_and_rtk_values_do_not_receive_fake_defaults(self):
        defaults = self.registry.default_values()

        self.assertEqual(defaults.count("hardware.lower_machine_port"), 0)
        self.assertEqual(defaults.count("rtk.port"), 0)
        self.assertEqual(defaults.count("rtk.center_offset_along_heading_m"), 0)
        self.assertEqual(defaults.count("rtk.center_offset_right_m"), 0)

        missing = list(self.registry.missing_required(defaults))
        self.assertIn("hardware.lower_machine_port", missing)
        self.assertIn("rtk.port", missing)
        self.assertIn("rtk.center_offset_along_heading_m", missing)
        self.assertIn("rtk.center_offset_right_m", missing)

    @unittest.skipUnless(RUNTIME_AVAILABLE, "configuration registry implementation is not present")
    def test_ntrip_credentials_are_required_only_when_ntrip_is_enabled(self):
        values = self.registry.default_values()
        for key in (
            "ntrip.host",
            "ntrip.mountpoint",
            "ntrip.username",
            "ntrip.password",
        ):
            self.assertEqual(values.count(key), 1)
            self.assertEqual(values[key], "")
        values["hardware.lower_machine_port"] = "/dev/ttyTHS1"
        values["rtk.port"] = "/dev/ttyUSB0"
        values["rtk.center_offset_along_heading_m"] = "0.10"
        values["rtk.center_offset_right_m"] = "0.18"

        self.assertEqual(list(self.registry.missing_required(values)), [])

        values["ntrip.enabled"] = "true"
        missing = list(self.registry.missing_required(values))
        for key in (
            "ntrip.host",
            "ntrip.mountpoint",
            "ntrip.username",
            "ntrip.password",
        ):
            self.assertIn(key, missing)

    @unittest.skipUnless(RUNTIME_AVAILABLE, "configuration registry implementation is not present")
    def test_unknown_key_and_invalid_value_are_rejected(self):
        unknown = self.registry.validate_value("unknown.key", "1")
        self.assertFalse(unknown.valid)
        self.assertEqual(unknown.code, "CONFIG_KEY_UNKNOWN")

        invalid_boolean = self.registry.validate_value("ntrip.enabled", "maybe")
        self.assertFalse(invalid_boolean.valid)
        self.assertEqual(invalid_boolean.code, "CONFIG_VALUE_INVALID")


if __name__ == "__main__":
    unittest.main()
