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
)
CORE_LOADED = False


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
    CORE_LOADED = True


def polygon(points):
    point_type = cppyy.gbl.cleanbot.modeling.Point2d
    values = cppyy.gbl.std.vector[point_type]()
    for x, y in points:
        point = point_type()
        point.x_cm = x
        point.y_cm = y
        values.push_back(point)
    return values


class ModelingCoverageRuntimeTest(unittest.TestCase):
    def setUp(self):
        load_core(self)
        self.api = cppyy.gbl.cleanbot.modeling

    def test_odd_natural_count_is_increased_to_distinct_even_lanes(self):
        area = polygon(((0, 0), (500, 0), (500, 1000), (0, 1000)))

        result = self.api.plan_coverage(area, 0.0, 116.0, 10.0)

        self.assertTrue(result.success)
        self.assertEqual(result.natural_lane_count, 5)
        self.assertEqual(len(result.lanes), 6)
        self.assertEqual(len(result.lanes) % 2, 0)
        self.assertAlmostEqual(result.lane_spacing_cm, 76.8, places=6)
        self.assertAlmostEqual(result.actual_overlap_cm, 39.2, places=6)

        offsets = [round(lane.offset_cm, 6) for lane in result.lanes]
        self.assertEqual(len(offsets), len(set(offsets)))

    def test_even_natural_count_is_kept_and_overlap_is_at_least_ten(self):
        area = polygon(((0, 0), (400, 0), (400, 1000), (0, 1000)))

        result = self.api.plan_coverage(area, 0.0, 116.0, 10.0)

        self.assertTrue(result.success)
        self.assertEqual(result.natural_lane_count, 4)
        self.assertEqual(len(result.lanes), 4)
        self.assertGreaterEqual(result.actual_overlap_cm, 10.0)

    def test_narrow_area_produces_two_symmetric_distinct_lanes(self):
        area = polygon(((0, 0), (80, 0), (80, 500), (0, 500)))

        result = self.api.plan_coverage(area, 0.0, 116.0, 10.0)

        self.assertTrue(result.success)
        self.assertEqual(len(result.lanes), 2)
        self.assertGreater(result.lane_spacing_cm, 0.0)
        self.assertLessEqual(result.lane_spacing_cm, 106.0)
        self.assertNotEqual(result.lanes[0].offset_cm, result.lanes[1].offset_cm)
        self.assertGreaterEqual(result.actual_overlap_cm, 10.0)

    def test_invalid_overlap_and_self_intersection_are_rejected(self):
        valid = polygon(((0, 0), (500, 0), (500, 1000), (0, 1000)))
        invalid_overlap = self.api.plan_coverage(valid, 0.0, 116.0, 116.0)
        self.assertFalse(invalid_overlap.success)
        self.assertEqual(invalid_overlap.code, "LANE_SPACING_INVALID")

        crossed = polygon(((0, 0), (500, 1000), (0, 1000), (500, 0)))
        invalid_polygon = self.api.plan_coverage(crossed, 0.0, 116.0, 10.0)
        self.assertFalse(invalid_polygon.success)
        self.assertEqual(invalid_polygon.code, "POLYGON_INVALID")


if __name__ == "__main__":
    unittest.main()
