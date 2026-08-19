# 文件作用：验证 mission package contract 相关契约、运行逻辑和边界条件。
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"


class MissionPackageContractTest(unittest.TestCase):
    # 测试作用：验证“package_contains_core_node_and_tests”场景的契约、输出结果和边界行为。
    def test_package_contains_core_node_and_tests(self):
        required = (
            "package.xml",
            "CMakeLists.txt",
            "include/cleanbot_mission/mission_state_machine.hpp",
            "include/cleanbot_mission/straight_edge_guard.hpp",
            "include/cleanbot_mission/waypoint_plan.hpp",
            "src/mission_state_machine.cpp",
            "src/straight_edge_guard.cpp",
            "src/waypoint_plan.cpp",
            "src/mission_manager_node.cpp",
            "test/test_mission_state_machine.cpp",
            "test/test_straight_edge_guard.cpp",
            "test/test_waypoint_plan.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    # 测试作用：验证“package_targets_humble_cpp17_and_actions”场景的契约、输出结果和边界行为。
    def test_package_targets_humble_cpp17_and_actions(self):
        root = ET.parse(str(PACKAGE / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_mission")
        dependencies = {item.text for item in root.findall("depend")}
        self.assertTrue(
            {"rclcpp", "rclcpp_action", "cleanbot_interfaces"}.issubset(dependencies)
        )

        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for token in (
            "CXX_STANDARD 17",
            "add_library(mission_core",
            "add_executable(mission_manager_node",
            "ament_add_gtest(test_mission_state_machine",
            "ament_add_gtest(test_waypoint_plan",
            "rclcpp_action",
        ):
            self.assertIn(token, cmake)

    # 测试作用：验证“node_owns_mission_action_and_runtime_topics”场景的契约、输出结果和边界行为。
    def test_node_owns_mission_action_and_runtime_topics(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )

        for token in (
            "rclcpp_action::create_server<ExecuteCleaning>",
            '"/mission/execute_cleaning"',
            '"/mission/set_pause"',
            '"/tracking/target"',
            '"/control/mission_cmd"',
            '"/control/brush_cmd"',
            '"/control/safety_cmd"',
            '"/tracking/status"',
            '"/rtk/fix"',
            '"/hardware/status"',
            '"/hardware/command_status"',
            '"/control/final_cmd"',
            "rclcpp_action::create_server<NavigateWaypoints>",
            '"/mission/navigate_waypoints"',
        ):
            self.assertIn(token, source)

    # 测试作用：验证“waypoint_action_reuses_navigation_chain_and_preserves_brush_normally”场景的契约、输出结果和边界行为。
    def test_waypoint_action_reuses_navigation_chain_and_preserves_brush_normally(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )

        for token in (
            "executeWaypoints",
            "WaypointSequence",
            "build_waypoint_segment",
            "executeTurn(",
            "executeTracking(",
            "finishWaypointAction",
            "publishWaypointFeedback",
        ):
            self.assertIn(token, source)

        waypoint_start = source.index("void executeWaypoints(")
        waypoint_end = source.index(
            "template <typename GoalHandleT>\n  StepResult repositionToPoint(",
            waypoint_start,
        )
        waypoint_body = source[waypoint_start:waypoint_end]
        self.assertNotIn("publishBrushForSegment", waypoint_body)
        self.assertNotIn("publishBrush(", waypoint_body)

        finish_start = source.index("void finishWaypointAction(")
        finish_end = source.index("void finishReturnHomeAction(", finish_start)
        finish_body = source[finish_start:finish_end]
        self.assertIn("if (step == StepResult::kFailed)", finish_body)
        self.assertIn("publishBrush(false, 0, false);", finish_body)
        self.assertNotIn("publishStoppedOutputs", finish_body)

    # 测试作用：验证“waypoint_action_uses_current_rtk_and_global_tracking_speed”场景的契约、输出结果和边界行为。
    def test_waypoint_action_uses_current_rtk_and_global_tracking_speed(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        tracking_source = (
            WORKSPACE / "src/cleanbot_control/src/tracking_node.cpp"
        ).read_text(encoding="utf-8")

        waypoint_start = source.index("void executeWaypoints(")
        waypoint_end = source.index(
            "template <typename GoalHandleT>\n  StepResult repositionToPoint(",
            waypoint_start,
        )
        waypoint_body = source[waypoint_start:waypoint_end]
        self.assertIn("current_fix = latest_rtk_", waypoint_body)
        self.assertIn("build_waypoint_segment(", waypoint_body)
        self.assertIn("segment.speed = 0", waypoint_body)
        self.assertIn("base_forward_speed_", tracking_source)
        self.assertIn("command.x_speed = base_forward_speed_", tracking_source)

    # 测试作用：验证“waypoint_restart_rebuilds_leg_from_latest_rtk_pose”场景的契约、输出结果和边界行为。
    def test_waypoint_restart_rebuilds_leg_from_latest_rtk_pose(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        tracking_start = source.index("StepResult executeTracking(")
        tracking_end = source.index("void finishAction(", tracking_start)
        tracking_body = source[tracking_start:tracking_end]

        self.assertGreaterEqual(
            tracking_body.count("if (!manage_brush)"),
            3,
        )
        self.assertGreaterEqual(
            tracking_body.count("return StepResult::kRestart;"),
            3,
        )

    # 测试作用：验证“node_matches_turn_request_and_tracking_generation”场景的契约、输出结果和边界行为。
    def test_node_matches_turn_request_and_tracking_generation(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        for token in (
            "status->request_id",
            "status->source",
            "tracking->generation",
            "STATE_COMPLETED",
            "STATE_REJECTED",
            "STATE_TIMED_OUT",
            "STATE_TRANSPORT_LOST",
            "TURN_COMMAND_TRANSPORT_LOST",
            "begin_rtk_recovery",
            "recover_rtk",
            "is_canceling",
        ):
            self.assertIn(token, source)

    # 测试作用：验证“turn_heartbeat_refreshes_lease_without_reissuing_finite_command”场景的契约、输出结果和边界行为。
    def test_turn_heartbeat_refreshes_lease_without_reissuing_finite_command(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("publishTurnCommand(segment, request_id, true);", source)
        self.assertNotIn("publishTurnCommand(segment, request_id, false);", source)

    # 测试作用：验证“preflight_can_wait_for_hardware_to_become_ready”场景的契约、输出结果和边界行为。
    def test_preflight_can_wait_for_hardware_to_become_ready(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("preflight_complete_", source)
        self.assertIn("mission_active_ && preflight_complete_", source)

    # 测试作用：验证“edge_confirmation_is_scoped_to_straight_tracking”场景的契约、输出结果和边界行为。
    def test_edge_confirmation_is_scoped_to_straight_tracking(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        tracking_start = source.index("StepResult executeTracking(")
        tracking_end = source.index("void finishAction(", tracking_start)
        tracking_body = source[tracking_start:tracking_end]
        turn_start = source.index("StepResult executeTurn(")
        turn_body = source[turn_start:tracking_start]

        self.assertIn("straight_edge_guard_->evaluate", tracking_body)
        self.assertIn("StraightEdgeDecision::kTargetReached", tracking_body)
        self.assertIn("StraightEdgeDecision::kConfirmRequired", tracking_body)
        self.assertIn('"EDGE_CONFIRM_REQUIRED"', source)
        self.assertNotIn("straight_edge_guard_->evaluate", turn_body)
        self.assertNotIn('setFaultLocked("EDGE_TRIGGERED"', source)
        self.assertNotIn("!latest_hardware_.edge_clear", source)

    # 测试作用：验证“edge_confirmation_requires_manual_resume_without_auto_reverse”场景的契约、输出结果和边界行为。
    def test_edge_confirmation_requires_manual_resume_without_auto_reverse(self):
        source = (PACKAGE / "src/mission_manager_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn('"EDGE_CONTINUE_CONFIRMED"', source)
        self.assertIn("edge_confirmation_required_", source)
        self.assertNotIn("edge_recovery_back", source)
        self.assertNotIn("publishEdgeReverse", source)

    # 测试作用：验证“mission_package_has_no_redis_dependency”场景的契约、输出结果和边界行为。
    def test_mission_package_has_no_redis_dependency(self):
        forbidden = []
        for path in PACKAGE.rglob("*"):
            if path.is_file() and path.suffix in {".xml", ".txt", ".cpp", ".hpp"}:
                if "redis" in path.read_text(encoding="utf-8").lower():
                    forbidden.append(str(path.relative_to(PACKAGE)))
        self.assertEqual(forbidden, [])


if __name__ == "__main__":
    unittest.main()
