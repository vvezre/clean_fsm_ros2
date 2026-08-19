# 文件作用：验证 gateway package contract 相关契约、运行逻辑和边界条件。
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"
PACKAGE = SRC / "cleanbot_gateway"


class GatewayPackageContractTest(unittest.TestCase):
    # 测试作用：验证“package_contains_mqtt_gateway_core_node_and_tests”场景的契约、输出结果和边界行为。
    def test_package_contains_mqtt_gateway_core_node_and_tests(self):
        required = (
            "package.xml",
            "CMakeLists.txt",
            "include/cleanbot_gateway/cloud_command.hpp",
            "include/cleanbot_gateway/cloud_message_codec.hpp",
            "src/cloud_command.cpp",
            "src/cloud_message_codec.cpp",
            "src/cloud_gateway_node.cpp",
            "test/test_cloud_command.cpp",
            "test/test_cloud_message_codec.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    # 测试作用：验证“package_uses_ros_config_paho_and_structured_json”场景的契约、输出结果和边界行为。
    def test_package_uses_ros_config_paho_and_structured_json(self):
        root = ET.parse(str(PACKAGE / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_gateway")
        dependencies = {item.text for item in root.findall("depend")}
        self.assertTrue(
            {
                "rclcpp",
                "cleanbot_common",
                "cleanbot_config",
                "cleanbot_control",
                "cleanbot_interfaces",
                "nlohmann-json3-dev",
                "libpaho-mqttpp-dev",
            }.issubset(dependencies)
        )

        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for token in (
            "CXX_STANDARD 17",
            "find_package(PahoMqttCpp REQUIRED)",
            "find_package(nlohmann_json REQUIRED)",
            "add_library(gateway_core",
            "add_executable(cloud_gateway_node",
            "ament_add_gtest(test_cloud_command",
            "ament_add_gtest(test_cloud_message_codec",
        ):
            self.assertIn(token, cmake)

    # 测试作用：验证“gateway_preserves_cloud_topics_and_command_results”场景的契约、输出结果和边界行为。
    def test_gateway_preserves_cloud_topics_and_command_results(self):
        source = (PACKAGE / "src/cloud_gateway_node.cpp").read_text(encoding="utf-8")
        codec = (PACKAGE / "src/cloud_message_codec.cpp").read_text(encoding="utf-8")
        for token in (
            '"RAILCAR/S/"',
            '"RAILCAR/R/"',
            '"/control/manual_cmd"',
            '"/control/emergency_cmd"',
            "common::latest_command_qos()",
            "set_clean_session(true)",
        ):
            self.assertIn(token, source)
        for token in (
            '"company_code"',
            '"product_model"',
            '"product_id"',
            '"command_id"',
            '"trace_id"',
            '"command_result"',
            '"ack"',
        ):
            self.assertIn(token, codec)

    # 测试作用：验证“gateway_has_stale_and_retained_joystick_guards”场景的契约、输出结果和边界行为。
    def test_gateway_has_stale_and_retained_joystick_guards(self):
        core = (PACKAGE / "src/cloud_command.cpp").read_text(encoding="utf-8")
        for token in (
            "JOYSTICK_RETAINED_REJECTED",
            "JOYSTICK_COMMAND_EXPIRED",
            "JOYSTICK_PARAMS_INVALID",
            'input.command != "joystick_move"',
            'input.command == "parking"',
            'input.command == "stop"',
        ):
            self.assertIn(token, core)

    # 测试作用：验证“gateway_configuration_remains_available_but_is_not_started_by_default”场景的契约、输出结果和边界行为。
    def test_gateway_configuration_remains_available_but_is_not_started_by_default(self):
        registry = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")
        launch = (
            SRC / "cleanbot_bringup/launch/cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        bringup_package = (
            SRC / "cleanbot_bringup/package.xml"
        ).read_text(encoding="utf-8")

        for key in (
            "cloud.mqtt.enabled",
            "cloud.mqtt.host",
            "cloud.mqtt.port",
            "cloud.mqtt.username",
            "cloud.mqtt.password",
            "cloud.mqtt.keepalive_sec",
            "cloud.mqtt.qos",
            "cloud.command_max_age_sec",
            "device.company_code",
            "device.product_model",
            "device.product_id",
        ):
            self.assertIn(key, registry)

        self.assertNotIn('package="cleanbot_gateway"', launch)
        self.assertNotIn('executable="cloud_gateway_node"', launch)
        self.assertNotIn("<exec_depend>cleanbot_gateway</exec_depend>", bringup_package)

    # 测试作用：验证“manual_speed_is_immediate_and_sqlite_is_not_accessed_directly”场景的契约、输出结果和边界行为。
    def test_manual_speed_is_immediate_and_sqlite_is_not_accessed_directly(self):
        registry = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")
        node = (PACKAGE / "src/cloud_gateway_node.cpp").read_text(encoding="utf-8")
        package_text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in PACKAGE.rglob("*")
            if path.is_file() and path.suffix in {".cpp", ".hpp", ".xml", ".txt"}
        ).lower()

        self.assertIn(
            '"motion.manual_max_speed", "350", 50, 600,\n      ApplyPolicy::kImmediate',
            registry,
        )
        self.assertIn('"motion.manual_max_speed"', node)
        self.assertIn("ConfigClient", node)
        self.assertNotIn("sqlite3", package_text)


if __name__ == "__main__":
    unittest.main()
