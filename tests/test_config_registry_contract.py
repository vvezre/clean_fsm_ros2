# 文件作用：验证配置注册表相关契约、运行逻辑和边界条件。
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"


class ModelingRecognitionConfigContractTest(unittest.TestCase):
    # 测试作用：验证“recognition_thresholds_are_owned_by_config_registry”场景的契约、输出结果和边界行为。
    def test_recognition_thresholds_are_owned_by_config_registry(self):
        source = (
            SRC / "cleanbot_config" / "src" / "config_registry.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            '"modeling.recognition.duplicate_tolerance_cm"',
            '"modeling.recognition.minimum_area_cm2"',
            '"modeling.recognition.assist_turn_threshold_deg"',
            '"modeling.recognition.minimum_connector_length_cm"',
            '"modeling.recognition.maximum_connector_endpoint_distance_cm"',
            '"modeling.recognition.unordered_auto_confirm_confidence"',
        ):
            self.assertIn(token, source)

    # 测试作用：验证“modeling_node_requests_recognition_thresholds”场景的契约、输出结果和边界行为。
    def test_modeling_node_requests_recognition_thresholds(self):
        source = (
            SRC / "cleanbot_modeling" / "src" / "modeling_manager_node.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            '"modeling.recognition.duplicate_tolerance_cm"',
            '"modeling.recognition.minimum_area_cm2"',
            '"modeling.recognition.assist_turn_threshold_deg"',
            '"modeling.recognition.minimum_connector_length_cm"',
            '"modeling.recognition.maximum_connector_endpoint_distance_cm"',
            '"modeling.recognition.unordered_auto_confirm_confidence"',
            "RecognitionOptions",
        ):
            self.assertIn(token, source)


if __name__ == "__main__":
    unittest.main()
