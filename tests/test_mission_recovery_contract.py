# 文件作用：验证 mission recovery contract 相关契约、运行逻辑和边界条件。
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MISSION_SOURCE = (
    ROOT / "src" / "cleanbot_mission" / "src" / "mission_manager_node.cpp"
).read_text(encoding="utf-8")


class MissionRecoveryContractTests(unittest.TestCase):
    # 辅助方法：为 _function_body 测试场景准备输入、执行操作或整理结果。
    def _function_body(self, signature, next_signature=None):
        start = MISSION_SOURCE.index(signature)
        if next_signature is None:
            return MISSION_SOURCE[start:]
        end = MISSION_SOURCE.index(next_signature, start + len(signature))
        return MISSION_SOURCE[start:end]

    # 测试作用：验证“recovery_repositions_before_restoring_segment”场景的契约、输出结果和边界行为。
    def test_recovery_repositions_before_restoring_segment(self):
        body = self._function_body(
            "void executeRecoveredSegments(",
            "template <typename GoalHandleT>\n  StepResult waitForPreflight(",
        )
        self.assertIn("build_waypoint_segment(", body)
        self.assertLess(
            body.index("build_waypoint_segment("),
            body.index("publishBrushForSegment(segment"),
        )

    # 测试作用：验证“cleaning_and_return_home_have_independent_checkpoint_owners”场景的契约、输出结果和边界行为。
    def test_cleaning_and_return_home_have_independent_checkpoint_owners(self):
        self.assertIn("cleaning_checkpoint_store_", MISSION_SOURCE)
        self.assertIn("return_home_checkpoint_store_", MISSION_SOURCE)

    # 测试作用：验证“waypoint_completion_does_not_clear_cleaning_checkpoint”场景的契约、输出结果和边界行为。
    def test_waypoint_completion_does_not_clear_cleaning_checkpoint(self):
        body = self._function_body(
            "void finishWaypointAction(",
            "void finishReturnHomeAction(",
        )
        self.assertNotIn("clearCheckpoint(", body)

    # 测试作用：验证“return_home_restarts_from_fresh_pose”场景的契约、输出结果和边界行为。
    def test_return_home_restarts_from_fresh_pose(self):
        body = self._function_body(
            "void executeReturnHome(",
            "void executeRecoverMission(",
        )
        self.assertIn("StepResult::kRestart", body)
        self.assertIn("rtkReadyLocked()", body)


if __name__ == "__main__":
    unittest.main()
