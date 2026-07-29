from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MISSION_SOURCE = (
    ROOT / "src" / "cleanbot_mission" / "src" / "mission_manager_node.cpp"
).read_text(encoding="utf-8")


class MissionRecoveryContractTests(unittest.TestCase):
    def _function_body(self, signature, next_signature=None):
        start = MISSION_SOURCE.index(signature)
        if next_signature is None:
            return MISSION_SOURCE[start:]
        end = MISSION_SOURCE.index(next_signature, start + len(signature))
        return MISSION_SOURCE[start:end]

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

    def test_cleaning_and_return_home_have_independent_checkpoint_owners(self):
        self.assertIn("cleaning_checkpoint_store_", MISSION_SOURCE)
        self.assertIn("return_home_checkpoint_store_", MISSION_SOURCE)

    def test_waypoint_completion_does_not_clear_cleaning_checkpoint(self):
        body = self._function_body(
            "void finishWaypointAction(",
            "void finishReturnHomeAction(",
        )
        self.assertNotIn("clearCheckpoint(", body)

    def test_return_home_restarts_from_fresh_pose(self):
        body = self._function_body(
            "void executeReturnHome(",
            "void executeRecoverMission(",
        )
        self.assertIn("StepResult::kRestart", body)
        self.assertIn("rtkReadyLocked()", body)


if __name__ == "__main__":
    unittest.main()
