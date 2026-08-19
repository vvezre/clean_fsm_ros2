# 文件作用：验证 modeling task plan runtime 相关契约、运行逻辑和边界条件。
import ctypes
import os
import sysconfig
import unittest
from pathlib import Path


CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
    os.add_dll_directory(str(CPPYY_BACKEND_BIN))
    os.environ["PATH"] = str(CPPYY_BACKEND_BIN) + os.pathsep + os.environ.get("PATH", "")
    for dll_name in (
        "msvcp140.dll",
        "msvcp140_1.dll",
        "msvcp140_2.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
    ):
        ctypes.WinDLL(str(CPPYY_BACKEND_BIN / dll_name))

import cppyy


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_modeling"
SOURCES = (
    PACKAGE / "src" / "geometry.cpp",
    PACKAGE / "src" / "coverage_planner.cpp",
    PACKAGE / "src" / "task_plan_builder.cpp",
)
CORE_LOADED = False


# 辅助方法：读取或加载 load_core 所需的测试数据并返回解析结果。
def load_core(test_case):
    global CORE_LOADED
    for source in SOURCES:
        test_case.assertTrue(source.is_file(), "{} is missing".format(source.name))
    if CORE_LOADED:
        return
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    try:
        api = cppyy.gbl.cleanbot.modeling
    except AttributeError:
        api = None
    if api is None or not hasattr(api, "lat_lon_to_local_cm"):
        cppyy.cppdef('#include "geometry.cpp"')
    if api is None or not hasattr(cppyy.gbl.cleanbot.modeling, "plan_coverage"):
        cppyy.cppdef('#include "coverage_planner.cpp"')
    if api is None or not hasattr(cppyy.gbl.cleanbot.modeling, "build_task_plan"):
        cppyy.cppdef('#include "task_plan_builder.cpp"')
    CORE_LOADED = True


# 辅助方法：为 rectangle_model 测试场景准备输入、执行操作或整理结果。
def rectangle_model(confirmed=True):
    api = cppyy.gbl.cleanbot.modeling
    model = api.CleaningModel()
    model.id = "model-1"
    model.name = "panel area"
    model.version = 3
    model.origin_lat = 31.2
    model.origin_lon = 121.5
    model.origin_valid = True
    model.recognition_confirmed = confirmed

    group = api.ModelGroup()
    group.id = "group-1"
    group.name = "group"
    group.area_number = 1
    group.sweep_mode = "manual"
    group.sweep_angle_deg = 0.0
    coordinates = ((0, 0), (500, 0), (500, 1000), (0, 1000))
    for index, (x, y) in enumerate(coordinates, start=1):
        point = api.ModelPoint()
        point.id = "p{}".format(index)
        point.sequence = index
        point.x_cm = x
        point.y_cm = y
        point.lat = 31.2
        point.lon = 121.5
        group.points.push_back(point)

    sub_area = api.ModelSubArea()
    sub_area.id = "sub-1"
    sub_area.name = "sub"
    sub_area.confirmed = confirmed
    for index in range(1, 5):
        sub_area.point_ids.push_back("p{}".format(index))
    group.sub_areas.push_back(sub_area)
    model.groups.push_back(group)
    return model


# 辅助方法：为 two_area_model 测试场景准备输入、执行操作或整理结果。
def two_area_model(with_connector=True):
    api = cppyy.gbl.cleanbot.modeling
    model = rectangle_model()
    group = model.groups[0]
    group.id = "group-1"
    group.points.clear()
    group.sub_areas.clear()
    group.connectors.clear()

    coordinates = (
        (0, 0), (500, 0), (500, 1000), (0, 1000),
        (500, 500), (1000, 500),
        (1000, 0), (1500, 0), (1500, 1000), (1000, 1000),
    )
    for index, (x, y) in enumerate(coordinates, start=1):
        point = api.ModelPoint()
        point.id = "p{}".format(index)
        point.sequence = index
        point.x_cm = x
        point.y_cm = y
        point.lat = 31.2
        point.lon = 121.5
        point.capture_type = "connection" if index in (5, 6) else "boundary"
        group.points.push_back(point)

    for area_id, ids in (("sub-1", (1, 2, 3, 4)), ("sub-2", (7, 8, 9, 10))):
        area = api.ModelSubArea()
        area.id = area_id
        area.confirmed = True
        for point_id in ids:
            area.point_ids.push_back("p{}".format(point_id))
        group.sub_areas.push_back(area)

    if with_connector:
        connector = api.ModelConnector()
        connector.id = "connector-1"
        connector.start_point_id = "p5"
        connector.end_point_id = "p6"
        connector.from_sub_area_id = "sub-1"
        connector.to_sub_area_id = "sub-2"
        connector.length_cm = 500
        connector.confirmed = True
        group.connectors.push_back(connector)
    return model


class ModelingTaskPlanRuntimeTest(unittest.TestCase):
    # 测试初始化：为每个用例创建相互隔离的初始状态和输入。
    def setUp(self):
        load_core(self)
        self.api = cppyy.gbl.cleanbot.modeling

    # 测试作用：验证“cleaning_model_declares_one_shared_coordinate_origin”场景的契约、输出结果和边界行为。
    def test_cleaning_model_declares_one_shared_coordinate_origin(self):
        model = self.api.CleaningModel()

        self.assertTrue(hasattr(model, "origin_valid"))
        self.assertTrue(hasattr(model, "origin_lat"))
        self.assertTrue(hasattr(model, "origin_lon"))

    # 测试作用：验证“points_from_different_groups_share_the_model_origin”场景的契约、输出结果和边界行为。
    def test_points_from_different_groups_share_the_model_origin(self):
        self.assertTrue(
            hasattr(self.api, "set_model_point_local_coordinates"),
            "shared model coordinate helper is missing",
        )
        if not hasattr(self.api, "set_model_point_local_coordinates"):
            return

        model = self.api.CleaningModel()
        first = self.api.ModelPoint()
        first.lat = 31.2
        first.lon = 121.5
        self.assertTrue(self.api.set_model_point_local_coordinates(model, first))
        self.assertTrue(model.origin_valid)
        self.assertAlmostEqual(first.x_cm, 0.0, places=6)
        self.assertAlmostEqual(first.y_cm, 0.0, places=6)

        second_group_first = self.api.ModelPoint()
        second_group_first.lat = 31.2
        second_group_first.lon = 121.5005
        self.assertTrue(
            self.api.set_model_point_local_coordinates(model, second_group_first)
        )
        self.assertGreater(second_group_first.x_cm, 4000.0)
        self.assertAlmostEqual(second_group_first.y_cm, 0.0, places=3)

    # 测试作用：验证“builds_even_snake_cleaning_lanes_and_transfer_segments”场景的契约、输出结果和边界行为。
    def test_builds_even_snake_cleaning_lanes_and_transfer_segments(self):
        result = self.api.build_task_plan(
            rectangle_model(), 116.0, 10.0, 350
        )

        self.assertTrue(result.success)
        self.assertEqual(result.plan.cleaning_lane_count, 6)
        self.assertEqual(result.plan.transfer_segment_count, 5)
        self.assertEqual(len(result.plan.segments), 11)
        self.assertAlmostEqual(result.plan.actual_overlap_cm, 39.2, places=6)

        cleaning = [
            segment for segment in result.plan.segments if segment.segment_type == 1
        ]
        self.assertEqual(len(cleaning), 6)
        self.assertEqual(len({segment.source_lane_id for segment in cleaning}), 6)
        for index in range(1, len(cleaning)):
            previous_delta = cleaning[index - 1].end.y_cm - cleaning[index - 1].start.y_cm
            current_delta = cleaning[index].end.y_cm - cleaning[index].start.y_cm
            self.assertLess(previous_delta * current_delta, 0.0)

    # 测试作用：验证“plan_hash_is_stable_for_same_model”场景的契约、输出结果和边界行为。
    def test_plan_hash_is_stable_for_same_model(self):
        first = self.api.build_task_plan(rectangle_model(), 116.0, 10.0, 350)
        second = self.api.build_task_plan(rectangle_model(), 116.0, 10.0, 350)

        self.assertTrue(first.success)
        self.assertEqual(first.plan.plan_hash, second.plan.plan_hash)
        self.assertNotEqual(first.plan.plan_hash, "")

    # 测试作用：验证“unconfirmed_model_is_rejected”场景的契约、输出结果和边界行为。
    def test_unconfirmed_model_is_rejected(self):
        result = self.api.build_task_plan(
            rectangle_model(confirmed=False), 116.0, 10.0, 350
        )

        self.assertFalse(result.success)
        self.assertEqual(result.code, "MODEL_NOT_CONFIRMED")

    # 测试作用：验证“model_without_shared_origin_is_rejected”场景的契约、输出结果和边界行为。
    def test_model_without_shared_origin_is_rejected(self):
        model = rectangle_model()
        model.origin_valid = False

        result = self.api.build_task_plan(model, 116.0, 10.0, 350)

        self.assertFalse(result.success)
        self.assertEqual(result.code, "MODEL_ORIGIN_MISSING")

    # 测试作用：验证“multi_area_plan_uses_confirmed_connector_path”场景的契约、输出结果和边界行为。
    def test_multi_area_plan_uses_confirmed_connector_path(self):
        result = self.api.build_task_plan(
            two_area_model(with_connector=True), 116.0, 10.0, 350
        )

        self.assertTrue(result.success)
        connector_segments = [
            segment for segment in result.plan.segments
            if segment.segment_type == 2 and
            segment.start.x_cm == 500 and segment.start.y_cm == 500 and
            segment.end.x_cm == 1000 and segment.end.y_cm == 500
        ]
        self.assertEqual(len(connector_segments), 1)
        self.assertFalse(connector_segments[0].brush_enabled)

    # 测试作用：验证“multi_area_plan_rejects_missing_connector”场景的契约、输出结果和边界行为。
    def test_multi_area_plan_rejects_missing_connector(self):
        result = self.api.build_task_plan(
            two_area_model(with_connector=False), 116.0, 10.0, 350
        )

        self.assertFalse(result.success)
        self.assertEqual(result.code, "SUB_AREA_CONNECTOR_MISSING")


if __name__ == "__main__":
    unittest.main()
