# 文件作用：验证 workspace contract 相关契约、运行逻辑和边界条件。
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"


class WorkspaceContractTest(unittest.TestCase):
    # 测试作用：验证“bringup_starts_lower_machine_gateway_with_registry_config”场景的契约、输出结果和边界行为。
    def test_bringup_starts_lower_machine_gateway_with_registry_config(self):
        launch_source = (
            SRC / "cleanbot_bringup" / "launch" / "cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        config_source = (
            SRC / "cleanbot_config" / "src" / "config_registry.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("cleanbot_hardware", launch_source)
        self.assertIn("lower_machine_node", launch_source)
        self.assertIn("system.yaml", launch_source)
        self.assertIn("hardware.lower_machine_port", config_source)
        self.assertIn("hardware.lower_machine_baudrate", config_source)
        self.assertIn('"hardware.command_repeat_count", "5"', config_source)

    # 测试作用：验证“bringup_starts_rtk_gateway_with_center_offset_and_ntrip_config”场景的契约、输出结果和边界行为。
    def test_bringup_starts_rtk_gateway_with_center_offset_and_ntrip_config(self):
        launch_source = (
            SRC / "cleanbot_bringup" / "launch" / "cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        config_source = (
            SRC / "cleanbot_config" / "src" / "config_registry.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn('package="cleanbot_rtk"', launch_source)
        self.assertIn('executable="rtk_node"', launch_source)
        self.assertIn('name="rtk_node"', launch_source)
        self.assertNotIn("node_executable=", launch_source)
        self.assertNotIn("node_name=", launch_source)
        self.assertIn("http_gateway", launch_source)
        self.assertNotIn("cloud_gateway", launch_source)
        for parameter in (
            "rtk.port",
            '"rtk.baudrate", "115200"',
            "rtk.center_offset_along_heading_m",
            "rtk.center_offset_right_m",
            '"ntrip.enabled", "false"',
            '"ntrip.gga_interval_sec", "5.0"',
            '"ntrip.connect_timeout_sec", "5.0"',
            '"ntrip.reconnect_interval_sec", "5.0"',
            '"ntrip.rtcm_timeout_sec", "15.0"',
        ):
            self.assertIn(parameter, config_source)

    # 测试作用：验证“bringup_starts_tracking_and_command_arbiter”场景的契约、输出结果和边界行为。
    def test_bringup_starts_tracking_and_command_arbiter(self):
        launch_source = (
            SRC / "cleanbot_bringup/launch/cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        config_source = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")

        for token in (
            'package="cleanbot_control"',
            'executable="tracking_node"',
            'executable="command_arbiter_node"',
        ):
            self.assertIn(token, launch_source)
        for parameter in (
            '"tracking.process_noise", "0.2"',
            '"tracking.measurement_noise", "1.0"',
            '"tracking.heading_gain", "10.0"',
            '"tracking.cte_gain", "1000.0"',
            '"control.manual_command_lease_ms", "500"',
            '"control.mission_command_lease_ms", "1000"',
            '"control.vision_command_lease_ms", "500"',
        ):
            self.assertIn(parameter, config_source)

    # 测试作用：验证“bringup_starts_cpp_http_control_gateway”场景的契约、输出结果和边界行为。
    def test_bringup_starts_cpp_http_control_gateway(self):
        launch_source = (
            SRC / "cleanbot_bringup/launch/cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        config_source = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")
        package_source = (
            SRC / "cleanbot_bringup/package.xml"
        ).read_text(encoding="utf-8")

        for token in (
            'package="cleanbot_http"',
            'executable="http_gateway_node"',
            'name="http_gateway_node"',
        ):
            self.assertIn(token, launch_source)
        self.assertNotIn('package="cleanbot_gateway"', launch_source)
        self.assertIn('"http.enabled", "true"', config_source)
        self.assertIn('"http.listen_address"', config_source)
        self.assertIn('"http.port", "7899"', config_source)
        self.assertIn("<exec_depend>cleanbot_http</exec_depend>", package_source)
        self.assertNotIn("<exec_depend>cleanbot_gateway</exec_depend>", package_source)

    # 测试作用：验证“bringup_starts_stage_five_mission_manager”场景的契约、输出结果和边界行为。
    def test_bringup_starts_stage_five_mission_manager(self):
        launch_source = (
            SRC / "cleanbot_bringup/launch/cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        config_source = (
            SRC / "cleanbot_config/src/config_registry.cpp"
        ).read_text(encoding="utf-8")
        package_source = (
            SRC / "cleanbot_bringup/package.xml"
        ).read_text(encoding="utf-8")

        for token in (
            'package="cleanbot_mission"',
            'executable="mission_manager_node"',
            'name="mission_manager_node"',
        ):
            self.assertIn(token, launch_source)
        for parameter in (
            '"mission.rtk_recovery_timeout_sec", "30.0"',
            '"mission.command_completion_timeout_sec", "20.0"',
            '"mission.turn_heartbeat_ms", "250"',
            '"mission.turn_z_speed", "0"',
            '"mission.turn_angle_tolerance_deg", "0.5"',
            '"mission.low_battery_threshold_percent", "0.0"',
            '"mission.rtk_freshness_timeout_sec", "2.0"',
            '"mission.hardware_freshness_timeout_sec", "1.0"',
        ):
            self.assertIn(parameter, config_source)
        self.assertIn("<exec_depend>cleanbot_mission</exec_depend>", package_source)

    # 测试作用：验证“stage_one_packages_and_build_scripts_exist”场景的契约、输出结果和边界行为。
    def test_stage_one_packages_and_build_scripts_exist(self):
        required = [
            WORKSPACE / "scripts" / "build_humble.sh",
            SRC / "cleanbot_interfaces" / "package.xml",
            SRC / "cleanbot_interfaces" / "CMakeLists.txt",
            SRC / "cleanbot_common" / "package.xml",
            SRC / "cleanbot_common" / "CMakeLists.txt",
            SRC / "cleanbot_config" / "package.xml",
            SRC / "cleanbot_config" / "CMakeLists.txt",
            SRC / "cleanbot_bringup" / "package.xml",
            SRC / "cleanbot_bringup" / "CMakeLists.txt",
        ]

        missing = [str(path.relative_to(WORKSPACE)) for path in required if not path.is_file()]
        self.assertEqual(missing, [], "missing stage-one files: {}".format(missing))

    # 测试作用：验证“humble_setup_is_sourced_without_nounset”场景的契约、输出结果和边界行为。
    def test_humble_setup_is_sourced_without_nounset(self):
        script = (WORKSPACE / "scripts" / "build_humble.sh").read_text(encoding="utf-8")
        disable_nounset = script.index("set +u")
        source_humble = script.index("source /opt/ros/humble/setup.bash")
        enable_nounset = script.index("set -u", source_humble)
        self.assertLess(disable_nounset, source_humble)
        self.assertLess(source_humble, enable_nounset)

    # 测试作用：验证“package_metadata_targets_ament_cmake”场景的契约、输出结果和边界行为。
    def test_package_metadata_targets_ament_cmake(self):
        package_names = (
            "cleanbot_interfaces",
            "cleanbot_common",
            "cleanbot_config",
            "cleanbot_bringup",
        )
        for package_name in package_names:
            package_xml = SRC / package_name / "package.xml"
            root = ET.parse(str(package_xml)).getroot()
            self.assertEqual(root.attrib.get("format"), "3")
            self.assertEqual(root.findtext("name"), package_name)
            build_type = root.find("export/build_type")
            self.assertIsNotNone(build_type)
            self.assertEqual(build_type.text, "ament_cmake")

    # 测试作用：验证“interface_contract_contains_required_files”场景的契约、输出结果和边界行为。
    def test_interface_contract_contains_required_files(self):
        required = {
            "msg/BrushCommand.msg",
            "msg/CommandExecutionStatus.msg",
            "msg/VehicleCommand.msg",
            "msg/HardwareStatus.msg",
            "msg/RtkFix.msg",
            "msg/VehicleState.msg",
            "msg/TrackingDebug.msg",
            "msg/TrackingTarget.msg",
            "msg/TrackingStatus.msg",
            "msg/ConfigEntry.msg",
            "msg/ConfigStatus.msg",
            "msg/ConfigChanged.msg",
            "srv/EmergencyStop.srv",
            "srv/ManualControl.srv",
            "srv/SetMissionPause.srv",
            "srv/GetConfig.srv",
            "srv/SetConfig.srv",
            "action/ExecuteCleaning.action",
            "action/NavigateWaypoints.action",
            "action/Dock.action",
            "action/ReturnHome.action",
        }
        package_root = SRC / "cleanbot_interfaces"
        missing = sorted(item for item in required if not (package_root / item).is_file())
        self.assertEqual(missing, [])

        rtk_fix = (package_root / "msg/RtkFix.msg").read_text(encoding="utf-8")
        self.assertIn("float64 raw_lat", rtk_fix)
        self.assertIn("bool center_valid", rtk_fix)
        self.assertIn("bool serial_connected", rtk_fix)
        self.assertIn("bool coordinate_valid", rtk_fix)
        self.assertIn("bool fixed_valid", rtk_fix)

        vehicle_command = (
            package_root / "msg/VehicleCommand.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "uint64 command_id",
            "uint64 request_id",
            "bool active",
            "bool operator_intent",
            "int32 steering_offset",
        ):
            self.assertIn(field, vehicle_command)

        manual_control = (
            package_root / "srv/ManualControl.srv"
        ).read_text(encoding="utf-8")
        self.assertNotIn("brush_speed", manual_control)

        command_status = (
            package_root / "msg/CommandExecutionStatus.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "uint8 STATE_SENT=1",
            "uint8 STATE_ACKNOWLEDGED=2",
            "uint8 STATE_REJECTED=3",
            "uint8 STATE_TIMED_OUT=4",
            "uint8 STATE_COMPLETED=5",
            "uint64 command_id",
            "uint64 request_id",
            "string source",
            "uint16 sequence",
            "uint8 state",
        ):
            self.assertIn(field, command_status)

        cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('"msg/BrushCommand.msg"', cmake)
        self.assertIn('"msg/CommandExecutionStatus.msg"', cmake)
        self.assertIn('"srv/SetMissionPause.srv"', cmake)
        self.assertIn('"srv/GetConfig.srv"', cmake)
        self.assertIn('"srv/SetConfig.srv"', cmake)
        self.assertNotIn('"srv/SetSpeed.srv"', cmake)

        task_segment = (
            package_root / "msg/TaskSegment.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "uint8 SEGMENT_CLEANING=1",
            "uint8 SEGMENT_TRANSFER=2",
            "uint8 SEGMENT_RETURN=3",
            "uint8 segment_type",
            "string group_id",
            "string sub_area_id",
            "string source_lane_id",
        ):
            self.assertIn(field, task_segment)

        execute_cleaning = (
            package_root / "action/ExecuteCleaning.action"
        ).read_text(encoding="utf-8")
        for field in (
            "string model_id",
            "uint64 model_version",
            "string plan_id",
            "string plan_hash",
            "int32 brush_speed",
        ):
            self.assertIn(field, execute_cleaning)

        pause_service = (
            package_root / "srv/SetMissionPause.srv"
        ).read_text(encoding="utf-8")
        for field in ("bool pause", "bool success", "string code", "string message"):
            self.assertIn(field, pause_service)

        for action in sorted(required):
            if not action.startswith("action/"):
                continue
            sections = (package_root / action).read_text(encoding="utf-8").split("---")
            self.assertEqual(len(sections), 3, "{} must define goal/result/feedback".format(action))

    # 测试作用：验证“common_package_enforces_cpp17_and_gtest”场景的契约、输出结果和边界行为。
    def test_common_package_enforces_cpp17_and_gtest(self):
        cmake_text = (SRC / "cleanbot_common" / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("CXX_STANDARD 17", cmake_text)
        self.assertIn("ament_add_gtest", cmake_text)
        self.assertIn("BUILD_TESTING", cmake_text)

    # 测试作用：验证“hardware_status_exposes_frame_and_signed_speed_diagnostics”场景的契约、输出结果和边界行为。
    def test_hardware_status_exposes_frame_and_signed_speed_diagnostics(self):
        message = (
            SRC / "cleanbot_interfaces" / "msg" / "HardwareStatus.msg"
        ).read_text(encoding="utf-8")
        required_fields = (
            "uint64 frame_sequence",
            "uint16 x_speed_raw",
            "uint16 z_speed_raw",
            "int32 x_speed",
            "int32 z_speed",
            "uint16 command_sequence",
            "uint8 completion_event",
            "bool completion_new",
        )
        for field in required_fields:
            self.assertIn(field, message)

    # 测试作用：验证“no_redis_dependency_is_introduced”场景的契约、输出结果和边界行为。
    def test_no_redis_dependency_is_introduced(self):
        forbidden = []
        for path in SRC.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix not in {".xml", ".txt", ".cmake", ".cpp", ".hpp", ".h", ".msg", ".srv", ".action"}:
                continue
            if "redis" in path.read_text(encoding="utf-8").lower():
                forbidden.append(str(path.relative_to(WORKSPACE)))
        self.assertEqual(forbidden, [])


if __name__ == "__main__":
    unittest.main()
