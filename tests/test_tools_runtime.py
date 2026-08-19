# 文件作用：验证 tools runtime 相关契约、运行逻辑和边界条件。
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "src" / "cleanbot_visualization"))
sys.path.insert(0, str(WORKSPACE / "src" / "cleanbot_diagnostics"))

from cleanbot_diagnostics.rules import freshness_level, status_level
from cleanbot_visualization.geo import heading_to_quaternion, lat_lon_to_local
from cleanbot_visualization.visualization_rules import (
    Origin,
    is_rtk_fresh,
    model_group_geometry,
    select_origin,
)


class ToolsRuntimeTest(unittest.TestCase):
    # 测试作用：验证“same_coordinate_is_local_origin”场景的契约、输出结果和边界行为。
    def test_same_coordinate_is_local_origin(self):
        self.assertEqual(
            lat_lon_to_local(31.0, 121.0, 31.0, 121.0),
            (0.0, 0.0),
        )

    # 测试作用：验证“north_and_east_offsets_are_positive”场景的契约、输出结果和边界行为。
    def test_north_and_east_offsets_are_positive(self):
        east, north = lat_lon_to_local(31.0, 121.0, 31.00001, 121.00001)
        self.assertGreater(east, 0.0)
        self.assertGreater(north, 0.0)

    # 测试作用：验证“invalid_coordinates_are_rejected”场景的契约、输出结果和边界行为。
    def test_invalid_coordinates_are_rejected(self):
        with self.assertRaises(ValueError):
            lat_lon_to_local(91.0, 121.0, 31.0, 121.0)
        with self.assertRaises(ValueError):
            lat_lon_to_local(31.0, 121.0, math.nan, 121.0)

    # 测试作用：验证“heading_wraps_to_ros_yaw”场景的契约、输出结果和边界行为。
    def test_heading_wraps_to_ros_yaw(self):
        x, y, z, w = heading_to_quaternion(450.0)
        self.assertAlmostEqual(x, 0.0, places=7)
        self.assertAlmostEqual(y, 0.0, places=7)
        self.assertAlmostEqual(z, 0.0, places=7)
        self.assertAlmostEqual(w, 1.0, places=7)

    # 测试作用：验证“freshness_levels”场景的契约、输出结果和边界行为。
    def test_freshness_levels(self):
        self.assertEqual(freshness_level(0.2, 1.0), "OK")
        self.assertEqual(freshness_level(0.6, 1.0), "WARN")
        self.assertEqual(freshness_level(1.1, 1.0), "ERROR")
        with self.assertRaises(ValueError):
            freshness_level(-0.1, 1.0)

    # 测试作用：验证“status_level”场景的契约、输出结果和边界行为。
    def test_status_level(self):
        self.assertEqual(status_level(True, "OK", "ready")[0], "OK")
        self.assertEqual(status_level(True, "WARN", "aging")[0], "WARN")
        self.assertEqual(status_level(True, "ERROR", "stale")[0], "WARN")
        self.assertEqual(
            status_level(True, "ERROR", "stale", stale_is_error=True)[0],
            "ERROR",
        )
        self.assertEqual(status_level(False, "OK", "fault")[0], "ERROR")

    # 测试作用：验证“invalid_model_origin_falls_back_to_configured_origin”场景的契约、输出结果和边界行为。
    def test_invalid_model_origin_falls_back_to_configured_origin(self):
        origin = select_origin(
            91.0,
            121.0,
            True,
            31.0,
            121.0,
            True,
        )
        self.assertEqual(origin, Origin(31.0, 121.0))

    # 测试作用：验证“rtk_freshness_checks_message_and_subscription_age”场景的契约、输出结果和边界行为。
    def test_rtk_freshness_checks_message_and_subscription_age(self):
        self.assertTrue(is_rtk_fresh(0.2, 0.3, 1.0))
        self.assertFalse(is_rtk_fresh(1.1, 0.1, 1.0))
        self.assertFalse(is_rtk_fresh(0.1, 1.1, 1.0))
        self.assertFalse(is_rtk_fresh(-1.0, 0.1, 1.0))
        with self.assertRaises(ValueError):
            is_rtk_fresh(0.1, 0.1, 0.0)

    # 测试作用：验证“model_geometry_keeps_subareas_and_connectors”场景的契约、输出结果和边界行为。
    def test_model_geometry_keeps_subareas_and_connectors(self):
        points = [
            SimpleNamespace(id="a", x_cm=0.0, y_cm=0.0),
            SimpleNamespace(id="b", x_cm=100.0, y_cm=0.0),
            SimpleNamespace(id="c", x_cm=100.0, y_cm=100.0),
        ]
        group = SimpleNamespace(
            points=points,
            sub_areas=[SimpleNamespace(point_ids=["a", "b", "c"])],
            connectors=[
                SimpleNamespace(start_point_id="a", end_point_id="c")
            ],
        )
        geometry = model_group_geometry(group)
        self.assertEqual(geometry["points"][1], (1.0, 0.0))
        self.assertEqual(
            geometry["areas"][0],
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)],
        )
        self.assertEqual(
            geometry["connectors"][0],
            ((0.0, 0.0), (1.0, 1.0)),
        )


if __name__ == "__main__":
    unittest.main()
