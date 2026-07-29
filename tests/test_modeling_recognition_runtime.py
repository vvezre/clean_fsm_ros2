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
    PACKAGE / "src" / "region_recognizer.cpp",
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
    if api is None or not hasattr(cppyy.gbl.cleanbot.modeling, "recognize_group"):
        cppyy.cppdef('#include "region_recognizer.cpp"')
    CORE_LOADED = True


def make_group(points):
    api = cppyy.gbl.cleanbot.modeling
    group = api.ModelGroup()
    group.id = "g1"
    group.name = "group"
    group.area_number = 1
    for index, item in enumerate(points, start=1):
        x, y = item[:2]
        capture_type = item[2] if len(item) > 2 else "boundary"
        point = api.ModelPoint()
        point.id = "p{}".format(index)
        point.sequence = index
        point.x_cm = x
        point.y_cm = y
        point.capture_type = capture_type
        group.points.push_back(point)
    return group


class ModelingRecognitionRuntimeTest(unittest.TestCase):
    def setUp(self):
        load_core(self)
        self.api = cppyy.gbl.cleanbot.modeling

    def test_rectangle_becomes_one_confirmed_sub_area(self):
        group = make_group(((0, 0), (1000, 0), (1000, 600), (0, 600)))

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "recognized")
        self.assertFalse(result.needs_confirmation)
        self.assertEqual(len(result.group.sub_areas), 1)
        self.assertEqual(len(result.group.connectors), 0)
        self.assertTrue(
            all(point.role == "boundary_corner" for point in result.group.points)
        )

    def test_boundary_geometry_is_not_guessed_as_connector(self):
        group = make_group(
            (
                (0, 0),
                (1000, 0),
                (1000, 400),
                (1800, 400),
                (1800, 0),
                (2600, 0),
                (2600, 800),
                (1800, 800),
                (1000, 800),
                (0, 800),
            )
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(len(result.group.connectors), 0)
        self.assertEqual(len(result.group.sub_areas), 1)

    def test_straight_boundary_helper_is_not_connector(self):
        group = make_group(
            ((0, 0), (1000, 0), (1000, 250), (980, 500), (1000, 750), (1000, 1000), (0, 1000))
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(len(result.group.connectors), 0)
        roles = {point.id: point.role for point in result.group.points}
        self.assertEqual(roles["p3"], "boundary_assist")
        self.assertEqual(roles["p4"], "boundary_assist")
        self.assertEqual(roles["p5"], "boundary_assist")

    def test_unordered_convex_boundary_is_recovered(self):
        group = make_group(((0, 0), (1000, 1000), (0, 1000), (1000, 0)))

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "recognized")
        self.assertEqual(len(result.group.sub_areas), 1)

    def test_unordered_concave_boundary_waits_for_confirmation(self):
        group = make_group(
            (
                (0, 0), (800, 300), (300, 300),
                (0, 800), (800, 0), (300, 800),
            )
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "needs_confirmation")
        self.assertTrue(result.needs_confirmation)
        self.assertFalse(result.group.sub_areas[0].confirmed)

    def test_duplicate_boundary_point_is_rejected(self):
        group = make_group(
            ((0, 0), (1000, 0), (1000, 600), (1000, 600), (0, 600))
        )

        result = self.api.recognize_group(group)

        self.assertFalse(result.success)
        self.assertEqual(result.code, "DUPLICATE_BOUNDARY_POINT")

    def test_l_shape_keeps_six_boundary_corners(self):
        group = make_group(
            ((0, 0), (800, 0), (800, 300), (300, 300), (300, 800), (0, 800))
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "recognized")
        self.assertEqual(len(result.group.sub_areas), 1)
        self.assertEqual(
            sum(point.role == "boundary_corner" for point in result.group.points),
            6,
        )

    def test_manual_connector_splits_two_ordered_sub_areas(self):
        group = make_group(
            (
                (0, 0), (400, 0), (400, 400), (0, 400),
                (400, 200, "connection"), (800, 200, "connection"),
                (800, 0), (1200, 0), (1200, 400), (800, 400),
            )
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "recognized")
        self.assertEqual(len(result.group.sub_areas), 2)
        self.assertEqual(len(result.group.connectors), 1)
        connector = result.group.connectors[0]
        self.assertEqual(connector.start_point_id, "p5")
        self.assertEqual(connector.end_point_id, "p6")
        self.assertEqual(connector.from_sub_area_id, "g1-sa1")
        self.assertEqual(connector.to_sub_area_id, "g1-sa2")
        self.assertTrue(connector.confirmed)

    def test_three_sub_areas_form_a_confirmed_chain(self):
        group = make_group(
            (
                (0, 0), (300, 0), (300, 300), (0, 300),
                (300, 150, "connection"), (500, 150, "connection"),
                (500, 0), (800, 0), (800, 300), (500, 300),
                (800, 150, "connection"), (1000, 150, "connection"),
                (1000, 0), (1300, 0), (1300, 300), (1000, 300),
            )
        )

        result = self.api.recognize_group(group)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "recognized")
        self.assertEqual(len(result.group.sub_areas), 3)
        self.assertEqual(len(result.group.connectors), 2)
        self.assertEqual(result.group.connectors[1].from_sub_area_id, "g1-sa2")
        self.assertEqual(result.group.connectors[1].to_sub_area_id, "g1-sa3")

    def test_unpaired_connection_point_is_rejected(self):
        group = make_group(
            (
                (0, 0), (400, 0), (400, 400), (0, 400),
                (400, 200, "connection"),
                (800, 0), (1200, 0), (1200, 400), (800, 400),
            )
        )

        result = self.api.recognize_group(group)

        self.assertFalse(result.success)
        self.assertEqual(result.code, "CONNECTION_POINTS_UNPAIRED")

    def test_local_coordinate_conversion_uses_centimetres(self):
        point = self.api.lat_lon_to_local_cm(31.2, 121.5, 31.20001, 121.50001)

        self.assertGreater(point.x_cm, 90.0)
        self.assertLess(point.x_cm, 100.0)
        self.assertGreater(point.y_cm, 110.0)
        self.assertLess(point.y_cm, 112.0)


if __name__ == "__main__":
    unittest.main()
