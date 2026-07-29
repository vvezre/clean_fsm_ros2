import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"
INTERFACES = SRC / "cleanbot_interfaces"
PACKAGE = SRC / "cleanbot_modeling"


class ModelingPackageContractTest(unittest.TestCase):
    def test_typed_modeling_interfaces_exist(self):
        required = {
            "msg/ModelPoint.msg": (
                "uint32 sequence",
                "float64 lat",
                "float64 lon",
                "float64 x_cm",
                "float64 y_cm",
                "string capture_type",
                "string role",
                "uint32 sample_count",
                "float64 sample_radius_m",
            ),
            "msg/ModelSubArea.msg": (
                "string id",
                "string[] point_ids",
                "bool confirmed",
            ),
            "msg/ModelConnector.msg": (
                "string start_point_id",
                "string end_point_id",
                "string from_sub_area_id",
                "string to_sub_area_id",
                "float64 length_cm",
            ),
            "msg/ModelGroup.msg": (
                "uint32 area_number",
                "string sweep_mode",
                "float64 sweep_angle_deg",
                "string recognition_status",
                "string recognition_message",
                "float64 recognition_confidence",
                "cleanbot_interfaces/ModelPoint[] points",
                "cleanbot_interfaces/ModelSubArea[] sub_areas",
            ),
            "msg/CleaningModel.msg": (
                "uint64 version",
                "float64 origin_lat",
                "float64 origin_lon",
                "bool origin_valid",
                "bool recognition_confirmed",
                "bool preview_confirmed",
                "cleanbot_interfaces/ModelGroup[] groups",
            ),
            "msg/CleaningPlan.msg": (
                "string model_id",
                "uint64 model_version",
                "string plan_hash",
                "float64 actual_overlap_cm",
                "cleanbot_interfaces/TaskSegment[] segments",
            ),
            "srv/ManageCleaningModel.srv": (
                "uint8 OP_CREATE=1",
                "uint8 OP_CREATE_GROUP=5",
                "uint8 OP_RECOGNIZE_GROUP=8",
                "uint8 operation",
                "cleanbot_interfaces/CleaningModel model",
            ),
            "srv/SampleModelPoint.srv": (
                "string model_id",
                "string group_id",
                "string capture_type",
                "cleanbot_interfaces/ModelPoint point",
            ),
            "srv/GenerateCleaningPlan.srv": (
                "string model_id",
                "bool confirm_preview",
                "cleanbot_interfaces/CleaningPlan plan",
            ),
            "srv/ExecuteModelPlan.srv": (
                "string plan_id",
                "int32 brush_speed",
                "bool accepted",
            ),
        }
        missing = []
        for relative, fields in required.items():
            path = INTERFACES / relative
            if not path.is_file():
                missing.append(relative)
                continue
            content = path.read_text(encoding="utf-8")
            for field in fields:
                self.assertIn(field, content, "{} missing {}".format(relative, field))
        self.assertEqual(missing, [])

        cmake = (INTERFACES / "CMakeLists.txt").read_text(encoding="utf-8")
        for relative in required:
            self.assertIn('"{}"'.format(relative), cmake)

    def test_modeling_package_declares_required_dependencies(self):
        package_xml = PACKAGE / "package.xml"
        cmake_path = PACKAGE / "CMakeLists.txt"
        self.assertTrue(package_xml.is_file())
        self.assertTrue(cmake_path.is_file())

        root = ET.parse(str(package_xml)).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_modeling")
        dependencies = {node.text for node in root.findall("depend")}
        for dependency in (
            "cleanbot_common",
            "cleanbot_config",
            "cleanbot_interfaces",
            "rclcpp",
            "rclcpp_action",
        ):
            self.assertIn(dependency, dependencies)

        cmake = cmake_path.read_text(encoding="utf-8")
        for token in (
            "CMAKE_CXX_STANDARD 17",
            "find_package(SQLite3 REQUIRED)",
            "add_library(modeling_core",
            "add_executable(modeling_manager_node",
            "ament_add_gtest",
        ):
            self.assertIn(token, cmake)

    def test_modeling_node_is_configured_and_launched(self):
        source_path = PACKAGE / "src" / "modeling_manager_node.cpp"
        self.assertTrue(source_path.is_file())
        source = source_path.read_text(encoding="utf-8")
        for token in (
            "ConfigClient",
            '"/rtk/fix"',
            '"/hardware/status"',
            '"/modeling/manage"',
            '"/modeling/sample_point"',
            '"/modeling/generate_plan"',
            '"/modeling/execute_plan"',
            '"/mission/execute_cleaning"',
            "SqliteModelRepository",
            "build_task_plan",
        ):
            self.assertIn(token, source)
        self.assertNotIn('"/control/final_cmd"', source)
        self.assertNotIn("serial_port", source)

        config = (
            SRC / "cleanbot_config" / "src" / "config_registry.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            '"modeling.database_path"',
            '"/var/lib/cleanbot/modeling.db"',
            '"modeling.brush_width_cm", "116.0"',
            '"modeling.minimum_overlap_cm", "10.0"',
            '"modeling.sample_count", "10"',
            '"modeling.maximum_sample_radius_m", "0.05"',
        ):
            self.assertIn(token, config)

        launch = (
            SRC / "cleanbot_bringup" / "launch" / "cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        for token in (
            'package="cleanbot_modeling"',
            'executable="modeling_manager_node"',
            'name="modeling_manager_node"',
        ):
            self.assertIn(token, launch)

        bringup_package = (
            SRC / "cleanbot_bringup" / "package.xml"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "<exec_depend>cleanbot_modeling</exec_depend>",
            bringup_package,
        )


if __name__ == "__main__":
    unittest.main()
