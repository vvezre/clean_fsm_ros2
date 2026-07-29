import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"
PACKAGE = SRC / "cleanbot_control"


class ControlPackageContractTest(unittest.TestCase):
    def test_package_contains_control_cores_nodes_and_tests(self):
        required = (
            "package.xml",
            "CMakeLists.txt",
            "include/cleanbot_control/tracking_core.hpp",
            "include/cleanbot_control/command_arbiter_core.hpp",
            "include/cleanbot_control/joystick_mapper.hpp",
            "src/tracking_core.cpp",
            "src/command_arbiter_core.cpp",
            "src/joystick_mapper.cpp",
            "src/tracking_node.cpp",
            "src/command_arbiter_node.cpp",
            "test/test_tracking_core.cpp",
            "test/test_command_arbiter_core.cpp",
            "test/test_joystick_mapper.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    def test_package_targets_humble_cpp17(self):
        root = ET.parse(str(PACKAGE / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_control")
        dependencies = {item.text for item in root.findall("depend")}
        self.assertTrue({"rclcpp", "cleanbot_interfaces"}.issubset(dependencies))
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("CXX_STANDARD 17", cmake)
        self.assertIn("add_executable(tracking_node", cmake)
        self.assertIn("add_executable(command_arbiter_node", cmake)

    def test_topics_match_stage_four_ownership(self):
        tracking = (PACKAGE / "src/tracking_node.cpp").read_text(encoding="utf-8")
        arbiter = (PACKAGE / "src/command_arbiter_node.cpp").read_text(encoding="utf-8")
        hardware = (SRC / "cleanbot_hardware/src/lower_machine_node.cpp").read_text(
            encoding="utf-8"
        )

        for token in ('"/tracking/target"', '"/rtk/fix"', '"/control/mission_cmd"'):
            self.assertIn(token, tracking)
        for token in (
            '"/control/emergency_cmd"',
            '"/control/safety_cmd"',
            '"/control/manual_cmd"',
            '"/control/mission_cmd"',
            '"/control/vision_cmd"',
            '"/control/brush_cmd"',
            '"/control/final_cmd"',
        ):
            self.assertIn(token, arbiter)
        self.assertNotIn("create_publisher<cleanbot_interfaces::msg::VehicleCommand>(\n        \"/control/final_cmd\"", tracking)
        self.assertIn('"/control/final_cmd"', hardware)

    def test_tracking_interfaces_exist_and_are_generated(self):
        interfaces = SRC / "cleanbot_interfaces"
        for item in ("msg/TrackingTarget.msg", "msg/TrackingStatus.msg"):
            self.assertTrue((interfaces / item).is_file(), item)
        cmake = (interfaces / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('"msg/TrackingTarget.msg"', cmake)
        self.assertIn('"msg/TrackingStatus.msg"', cmake)

    def test_tracking_status_publishes_signed_remaining_distance(self):
        status = (
            SRC / "cleanbot_interfaces/msg/TrackingStatus.msg"
        ).read_text(encoding="utf-8")
        tracking = (PACKAGE / "src/tracking_node.cpp").read_text(encoding="utf-8")

        self.assertIn("float64 signed_remaining_m", status)
        self.assertIn("status.signed_remaining_m = signed_remaining", tracking)

    def test_command_leases_are_validated_by_the_config_registry(self):
        source = (PACKAGE / "src/command_arbiter_node.cpp").read_text(encoding="utf-8")
        registry = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")
        for key in (
            "control.manual_command_lease_ms",
            "control.mission_command_lease_ms",
            "control.vision_command_lease_ms",
        ):
            self.assertIn(key, source)
            self.assertIn(key, registry)
        self.assertIn(
            'integer_default("control.manual_command_lease_ms", "500", 1, 60000)',
            registry,
        )

    def test_integer_configuration_is_read_with_int64_snapshot_api(self):
        tracking = (PACKAGE / "src/tracking_node.cpp").read_text(encoding="utf-8")
        self.assertIn('snapshot.get_integer("tracking.max_z_speed")', tracking)

    def test_arbiter_preserves_producer_request_id_before_allocating_final_id(self):
        core = (
            PACKAGE / "include/cleanbot_control/command_arbiter_core.hpp"
        ).read_text(encoding="utf-8")
        source = (PACKAGE / "src/command_arbiter_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("std::uint64_t request_id", core)
        self.assertIn("command.request_id =", source)
        self.assertIn("message.request_id = command.request_id", source)
        self.assertIn("left.request_id == right.request_id", source)
        self.assertIn("message.command_id = next_output_command_id_++", source)

    def test_arbiter_subscribes_to_latched_maintenance_state_and_caches_it(self):
        source = (PACKAGE / "src/command_arbiter_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn(
            '#include "cleanbot_interfaces/msg/maintenance_state.hpp"', source
        )
        self.assertIn('"/system/maintenance_state"', source)
        self.assertIn("common::latched_status_qos()", source)
        self.assertIn("onMaintenanceState", source)
        self.assertIn("cached_maintenance_state_", source)
        self.assertIn("has_cached_maintenance_state_", source)
        self.assertIn("cacheMaintenanceState", source)

    def test_arbiter_applies_cached_maintenance_before_first_configured_output(self):
        source = (PACKAGE / "src/command_arbiter_node.cpp").read_text(
            encoding="utf-8"
        )
        apply_cache = source.index("applyCachedMaintenanceState();")
        mark_configured = source.index("configured_ = true;", apply_cache)
        publish = source.index("publishIfChanged();", mark_configured)

        self.assertLess(apply_cache, mark_configured)
        self.assertLess(mark_configured, publish)
        self.assertIn(
            "arbiter_->set_maintenance(\n"
            "        cached_maintenance_state_.gate_active,\n"
            "        cached_maintenance_state_.generation)",
            source,
        )


if __name__ == "__main__":
    unittest.main()
