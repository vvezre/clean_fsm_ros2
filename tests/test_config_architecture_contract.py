import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"


class ConfigArchitectureContractTest(unittest.TestCase):
    def test_config_manager_package_and_ros_interfaces_exist(self):
        required = (
            SRC / "cleanbot_config" / "package.xml",
            SRC / "cleanbot_config" / "CMakeLists.txt",
            SRC / "cleanbot_config" / "src" / "config_manager_node.cpp",
            SRC / "cleanbot_interfaces" / "msg" / "ConfigEntry.msg",
            SRC / "cleanbot_interfaces" / "msg" / "ConfigStatus.msg",
            SRC / "cleanbot_interfaces" / "msg" / "ConfigChanged.msg",
            SRC / "cleanbot_interfaces" / "srv" / "GetConfig.srv",
            SRC / "cleanbot_interfaces" / "srv" / "SetConfig.srv",
        )

        missing = [str(path.relative_to(WORKSPACE)) for path in required if not path.is_file()]
        self.assertEqual(missing, [], "missing configuration architecture files: {}".format(missing))

    def test_config_interfaces_are_generated_and_legacy_speed_interface_is_retired(self):
        interfaces_root = SRC / "cleanbot_interfaces"
        cmake = (interfaces_root / "CMakeLists.txt").read_text(encoding="utf-8")

        for interface_path in (
            "msg/ConfigEntry.msg",
            "msg/ConfigStatus.msg",
            "msg/ConfigChanged.msg",
            "srv/GetConfig.srv",
            "srv/SetConfig.srv",
        ):
            self.assertIn('"{}"'.format(interface_path), cmake)

        self.assertNotIn('"srv/SetSpeed.srv"', cmake)
        self.assertNotIn(
            "int32 speed",
            (interfaces_root / "msg" / "Waypoint.msg").read_text(encoding="utf-8"),
        )
        self.assertNotIn(
            "int32 speed",
            (interfaces_root / "action" / "ReturnHome.action").read_text(encoding="utf-8"),
        )

        config_status = (
            interfaces_root / "msg" / "ConfigStatus.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "bool config_ready",
            "bool database_writable",
            "uint64 revision",
            "string state",
            "string error_code",
            "string[] missing_required_keys",
        ):
            self.assertIn(field, config_status)

    def test_common_package_owns_explicit_qos_profiles(self):
        common_root = SRC / "cleanbot_common"
        header_path = common_root / "include" / "cleanbot_common" / "qos_profiles.hpp"
        self.assertTrue(header_path.is_file())

        header = header_path.read_text(encoding="utf-8")
        for function_name in (
            "latest_command_qos",
            "latched_status_qos",
            "hardware_status_qos",
            "command_status_qos",
            "rtk_fix_qos",
            "debug_qos",
        ):
            self.assertIn(function_name, header)
        self.assertIn("inline rclcpp::QoS status_qos()", header)
        self.assertIn("KeepLast(1)", header)
        self.assertIn("transient_local", header)
        self.assertIn("best_effort", header)

        cmake = (common_root / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (common_root / "package.xml").read_text(encoding="utf-8")
        self.assertIn("find_package(rclcpp REQUIRED)", cmake)
        self.assertIn("ament_export_dependencies(rclcpp)", cmake)
        self.assertIn("<depend>rclcpp</depend>", package)

    def test_config_manager_owns_storage_state_and_services(self):
        config_root = SRC / "cleanbot_config"
        source = (config_root / "src" / "config_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        for token in (
            "SqliteConfigRepository",
            '"/config/status"',
            '"/config/changed"',
            '"/config/get"',
            '"/config/set"',
            '"STARTING"',
            '"CHECKING"',
            '"READY"',
            '"ERROR"',
            '"DEGRADED"',
            "missing_required_keys",
            "include_sensitive",
        ):
            self.assertIn(token, source)

        self.assertIn('declare_parameter<std::string>(', source)
        self.assertIn('"database_path"', source)
        self.assertIn('declare_parameter<std::string>("profile"', source)
        self.assertNotIn('declare_parameter<std::string>("rtk.port"', source)
        self.assertNotIn('declare_parameter<std::int64_t>("motion.base_forward_speed"', source)
        self.assertIn("repository_status_.writable = false", source)

    def test_config_client_is_the_only_business_node_configuration_api(self):
        config_root = SRC / "cleanbot_config"
        header = config_root / "include" / "cleanbot_config" / "config_client.hpp"
        source = config_root / "src" / "config_client.cpp"
        self.assertTrue(header.is_file())
        self.assertTrue(source.is_file())

        header_text = header.read_text(encoding="utf-8")
        source_text = source.read_text(encoding="utf-8")
        for token in (
            "class ConfigSnapshot",
            "class ConfigClient",
            "get_string",
            "get_integer",
            "get_double",
            "get_boolean",
            "ready_callback",
        ):
            self.assertIn(token, header_text)
        self.assertIn('"/config/status"', source_text)
        self.assertIn('"/config/changed"', source_text)
        self.assertIn('"/config/get"', source_text)

    def test_bringup_yaml_contains_only_database_bootstrap_configuration(self):
        bringup_root = SRC / "cleanbot_bringup"
        launch = (bringup_root / "launch" / "cleanbot.launch.py").read_text(
            encoding="utf-8"
        )
        yaml = (bringup_root / "config" / "system.yaml").read_text(encoding="utf-8")
        package = (bringup_root / "package.xml").read_text(encoding="utf-8")

        self.assertIn('package="cleanbot_config"', launch)
        self.assertIn('executable="config_manager_node"', launch)
        self.assertIn("http_gateway", launch)
        self.assertNotIn("cloud_gateway", launch)
        self.assertEqual(launch.count("parameters=[config_file]"), 1)
        self.assertIn("config_manager_node:", yaml)
        self.assertIn("database_path: /var/lib/cleanbot/cleanbot.db", yaml)
        self.assertIn("profile: default", yaml)
        for forbidden in (
            "lower_machine_node:",
            "rtk_node:",
            "tracking_node:",
            "command_arbiter_node:",
            "mission_manager_node:",
            "base_forward_speed",
            "ntrip_enabled",
        ):
            self.assertNotIn(forbidden, yaml)
        self.assertIn("<exec_depend>cleanbot_config</exec_depend>", package)

    def test_command_arbiter_waits_for_config_and_uses_shared_qos(self):
        source = (
            SRC / "cleanbot_control" / "src" / "command_arbiter_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("cleanbot_config/config_client.hpp", source)
        self.assertIn("ConfigClient", source)
        self.assertIn("config_not_ready", source)
        self.assertIn("common::latest_command_qos()", source)
        self.assertNotIn("declare_parameter", source)

    def test_tracking_uses_global_forward_speed_from_config(self):
        source = (
            SRC / "cleanbot_control" / "src" / "tracking_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("cleanbot_config/config_client.hpp", source)
        self.assertIn('"motion.base_forward_speed"', source)
        self.assertIn("base_forward_speed_", source)
        self.assertNotIn("command.x_speed = segment.speed", source)
        self.assertIn("common::rtk_fix_qos()", source)
        self.assertIn("common::debug_qos()", source)
        self.assertNotIn("declare_parameter", source)

    def test_hardware_and_rtk_do_not_open_serial_before_config_ready(self):
        hardware = (
            SRC / "cleanbot_hardware" / "src" / "lower_machine_node.cpp"
        ).read_text(encoding="utf-8")
        rtk = (SRC / "cleanbot_rtk" / "src" / "rtk_node.cpp").read_text(
            encoding="utf-8"
        )
        for source in (hardware, rtk):
            self.assertIn("cleanbot_config/config_client.hpp", source)
            self.assertIn("ConfigClient", source)
            self.assertIn("configure", source)
            self.assertNotIn("declare_parameter", source)
        self.assertIn("config_not_ready", hardware)
        self.assertIn("common::hardware_status_qos()", hardware)
        self.assertIn("common::command_status_qos()", hardware)
        self.assertIn("common::rtk_fix_qos()", rtk)

    def test_mission_rejects_tasks_until_configuration_is_ready(self):
        source = (
            SRC / "cleanbot_mission" / "src" / "mission_manager_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("cleanbot_config/config_client.hpp", source)
        self.assertIn("ConfigClient", source)
        self.assertIn("configured_", source)
        self.assertIn("CONFIG_NOT_READY", source)
        self.assertIn("common::latest_command_qos()", source)
        self.assertIn("common::rtk_fix_qos()", source)
        self.assertNotIn("declare_parameter", source)

    def test_new_configuration_sources_include_their_direct_standard_dependencies(self):
        expected = {
            SRC / "cleanbot_config/src/config_client.cpp": ("#include <chrono>", "#include <cctype>"),
            SRC / "cleanbot_config/src/config_manager_node.cpp": ("#include <chrono>",),
            SRC / "cleanbot_control/src/command_arbiter_node.cpp": ("#include <vector>",),
        }
        for path, includes in expected.items():
            source = path.read_text(encoding="utf-8")
            for include in includes:
                self.assertIn(include, source, str(path.relative_to(WORKSPACE)))

    def test_repository_hygiene_excludes_transient_workspace_outputs(self):
        gitignore_path = WORKSPACE / ".gitignore"
        self.assertTrue(gitignore_path.is_file())
        gitignore = gitignore_path.read_text(encoding="utf-8")
        for pattern in (".vs/", ".tmp/", "build/", "install/", "log/", "*.obj"):
            self.assertIn(pattern, gitignore)


if __name__ == "__main__":
    unittest.main()
