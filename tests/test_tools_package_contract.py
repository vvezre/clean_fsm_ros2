# 文件作用：验证 tools package contract 相关契约、运行逻辑和边界条件。
import ast
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"


class ToolsPackageContractTest(unittest.TestCase):
    # 测试作用：验证“visualization_package_scaffold_and_entry_point”场景的契约、输出结果和边界行为。
    def test_visualization_package_scaffold_and_entry_point(self):
        package = SRC / "cleanbot_visualization"
        for relative in (
            "package.xml",
            "setup.py",
            "setup.cfg",
            "resource/cleanbot_visualization",
            "cleanbot_visualization/__init__.py",
            "cleanbot_visualization/geo.py",
            "cleanbot_visualization/visualization_rules.py",
            "cleanbot_visualization/visualization_node.py",
        ):
            self.assertTrue((package / relative).is_file(), relative)
        setup = (package / "setup.py").read_text(encoding="utf-8")
        self.assertIn("cleanbot_visualization_node", setup)

    # 测试作用：验证“diagnostics_package_scaffold_and_entry_point”场景的契约、输出结果和边界行为。
    def test_diagnostics_package_scaffold_and_entry_point(self):
        package = SRC / "cleanbot_diagnostics"
        for relative in (
            "package.xml",
            "setup.py",
            "setup.cfg",
            "resource/cleanbot_diagnostics",
            "cleanbot_diagnostics/__init__.py",
            "cleanbot_diagnostics/rules.py",
            "cleanbot_diagnostics/diagnostics_node.py",
        ):
            self.assertTrue((package / relative).is_file(), relative)
        setup = (package / "setup.py").read_text(encoding="utf-8")
        self.assertIn("cleanbot_diagnostics_node", setup)

    # 测试作用：验证“optional_tool_launch_files_exist”场景的契约、输出结果和边界行为。
    def test_optional_tool_launch_files_exist(self):
        launch = SRC / "cleanbot_bringup" / "launch"
        for name in ("tools.launch.py", "diagnostics.launch.py", "record.launch.py"):
            self.assertTrue((launch / name).is_file(), name)

    # 测试作用：验证“foxglove_bridge_is_read_only_and_topic_limited”场景的契约、输出结果和边界行为。
    def test_foxglove_bridge_is_read_only_and_topic_limited(self):
        source = (
            SRC / "cleanbot_bringup" / "launch" / "tools.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('package="foxglove_bridge"', source)
        self.assertIn('"client_topic_whitelist": ["(?!)"]', source)
        self.assertIn('"service_whitelist": ["(?!)"]', source)
        self.assertIn('"param_whitelist": ["(?!)"]', source)
        self.assertIn('"topic_whitelist": FOXGLOVE_TOPIC_WHITELIST', source)
        self.assertNotIn("clientPublish", source)

    # 测试作用：验证“core_launch_does_not_require_tools”场景的契约、输出结果和边界行为。
    def test_core_launch_does_not_require_tools(self):
        source = (
            SRC / "cleanbot_bringup" / "launch" / "cleanbot.launch.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("cleanbot_visualization", source)
        self.assertNotIn("cleanbot_diagnostics", source)
        self.assertNotIn("foxglove_bridge", source)

    # 测试作用：验证“visualization_has_no_control_publishers”场景的契约、输出结果和边界行为。
    def test_visualization_has_no_control_publishers(self):
        source = (
            SRC / "cleanbot_visualization"
            / "cleanbot_visualization"
            / "visualization_node.py"
        )
        if source.is_file():
            text = source.read_text(encoding="utf-8")
            for forbidden in (
                '"/control/final_cmd"',
                '"/control/mission_cmd"',
                '"/control/safety_cmd"',
                '"/control/manual_cmd"',
            ):
                self.assertNotIn(forbidden, text)

    # 测试作用：验证“visualization_keeps_last_pose_and_valid_marker_orientation”场景的契约、输出结果和边界行为。
    def test_visualization_keeps_last_pose_and_valid_marker_orientation(self):
        source = (
            SRC / "cleanbot_visualization"
            / "cleanbot_visualization"
            / "visualization_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("self._last_pose", source)
        self.assertGreaterEqual(
            source.count("marker.pose.orientation.w = 1.0"),
            2,
        )
        self.assertIn("Marker.TEXT_VIEW_FACING", source)
        self.assertIn("def _status_text", source)

    # 测试作用：验证“model_subscription_is_created_during_node_initialization”场景的契约、输出结果和边界行为。
    def test_model_subscription_is_created_during_node_initialization(self):
        source = (
            SRC / "cleanbot_visualization"
            / "cleanbot_visualization"
            / "visualization_node.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        node_class = next(
            item
            for item in tree.body
            if isinstance(item, ast.ClassDef)
            and item.name == "VisualizationNode"
        )
        methods = {
            item.name: item
            for item in node_class.body
            if isinstance(item, ast.FunctionDef)
        }

        # 辅助方法：为 string_constants 测试场景准备输入、执行操作或整理结果。
        def string_constants(method):
            return {
                item.value
                for item in ast.walk(method)
                if isinstance(item, ast.Constant)
                and isinstance(item.value, str)
            }

        self.assertIn("/modeling/model", string_constants(methods["__init__"]))
        for name, method in methods.items():
            if name != "__init__":
                self.assertNotIn("/modeling/model", string_constants(method))

    # 测试作用：验证“diagnostics_has_no_control_clients_or_publishers”场景的契约、输出结果和边界行为。
    def test_diagnostics_has_no_control_clients_or_publishers(self):
        source = (
            SRC / "cleanbot_diagnostics"
            / "cleanbot_diagnostics"
            / "diagnostics_node.py"
        )
        if source.is_file():
            text = source.read_text(encoding="utf-8")
            for forbidden in (
                "create_client",
                "create_action_client",
                "final_cmd",
                "mission_cmd",
                "safety_cmd",
                "manual_cmd",
            ):
                self.assertNotIn(forbidden, text)

    # 测试作用：验证“diagnostics_callbacks_return_status_and_use_one_timer”场景的契约、输出结果和边界行为。
    def test_diagnostics_callbacks_return_status_and_use_one_timer(self):
        source = (
            SRC / "cleanbot_diagnostics"
            / "cleanbot_diagnostics"
            / "diagnostics_node.py"
        ).read_text(encoding="utf-8")
        self.assertGreaterEqual(source.count("return stat"), 12)
        self.assertNotIn("self.create_timer(", source)
        self.assertIn("Updater(self, publish_period)", source)

    # 测试作用：验证“hardware_diagnostics_include_latest_command_result”场景的契约、输出结果和边界行为。
    def test_hardware_diagnostics_include_latest_command_result(self):
        source = (
            SRC / "cleanbot_diagnostics"
            / "cleanbot_diagnostics"
            / "diagnostics_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('stat.add("command_id"', source)
        self.assertIn('stat.add("command_state"', source)
        self.assertIn('stat.add("command_detail"', source)

    # 测试作用：验证“modeling_exposes_latched_model_topic”场景的契约、输出结果和边界行为。
    def test_modeling_exposes_latched_model_topic(self):
        source = (
            SRC / "cleanbot_modeling" / "src" / "modeling_manager_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("model_publisher_", source)
        self.assertIn('"/modeling/model"', source)
        self.assertIn("response->model", source)

    # 测试作用：验证“recording_whitelist_excludes_control_topics”场景的契约、输出结果和边界行为。
    def test_recording_whitelist_excludes_control_topics(self):
        source = (
            SRC / "cleanbot_bringup" / "launch" / "record.launch.py"
        )
        if source.is_file():
            text = source.read_text(encoding="utf-8")
            self.assertIn("/rtk/fix", text)
            self.assertIn("/tracking/debug", text)
            for forbidden in (
                "/control/final_cmd",
                "/control/emergency_cmd",
                "/control/manual_cmd",
                "/control/mission_cmd",
                "/control/safety_cmd",
                "/control/brush_cmd",
            ):
                self.assertNotIn(forbidden, text)

    # 测试作用：验证“record_launch_uses_humble_compatible_error_handling”场景的契约、输出结果和边界行为。
    def test_record_launch_uses_humble_compatible_error_handling(self):
        source = (
            SRC / "cleanbot_bringup" / "launch" / "record.launch.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("LogError", source)
        self.assertIn("raise RuntimeError", source)


if __name__ == "__main__":
    unittest.main()
