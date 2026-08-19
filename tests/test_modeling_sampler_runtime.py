# 文件作用：验证 modeling sampler runtime 相关契约、运行逻辑和边界条件。
import ctypes
import math
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
HEADER = PACKAGE / "include" / "cleanbot_modeling" / "point_sampler.hpp"
SOURCE = PACKAGE / "src" / "point_sampler.cpp"
CORE_LOADED = False


# 辅助方法：读取或加载 load_core 所需的测试数据并返回解析结果。
def load_core(test_case):
    global CORE_LOADED
    test_case.assertTrue(HEADER.is_file(), "point sampler header is missing")
    test_case.assertTrue(SOURCE.is_file(), "point sampler source is missing")
    if CORE_LOADED:
        return
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    try:
        already_loaded = hasattr(
            cppyy.gbl.cleanbot.modeling, "sample_point"
        )
    except AttributeError:
        already_loaded = False
    if not already_loaded:
        cppyy.cppdef('#include "point_sampler.cpp"')
    CORE_LOADED = True


# 辅助方法：构造 make_samples 所需的测试对象、参数或临时资源。
def make_samples(
    count=10,
    fixed=True,
    gga_age=0.2,
    static=True,
    lat=31.20000000,
    lon=121.50000000,
):
    sample_type = cppyy.gbl.cleanbot.modeling.RtkSample
    vector_type = cppyy.gbl.std.vector[sample_type]
    samples = vector_type()
    for index in range(count):
        sample = sample_type()
        sample.fixed_valid = fixed
        sample.center_valid = fixed
        sample.gga_age_sec = gga_age
        sample.vehicle_static = static
        sample.lat = lat + index * 0.000000001
        sample.lon = lon - index * 0.000000001
        sample.heading_deg = 359.0 if index % 2 == 0 else 1.0
        sample.heading_valid = True
        sample.fix_quality = 4
        samples.push_back(sample)
    return samples


class ModelingSamplerRuntimeTest(unittest.TestCase):
    # 测试初始化：为每个用例创建相互隔离的初始状态和输入。
    def setUp(self):
        load_core(self)
        self.api = cppyy.gbl.cleanbot.modeling

    # 测试作用：验证“stable_fixed_samples_produce_mean_point”场景的契约、输出结果和边界行为。
    def test_stable_fixed_samples_produce_mean_point(self):
        result = self.api.sample_point(make_samples(), 10, 0.05)

        self.assertTrue(result.success)
        self.assertEqual(result.code, "OK")
        self.assertEqual(result.point.sample_count, 10)
        self.assertLess(result.point.sample_radius_m, 0.05)
        self.assertAlmostEqual(result.point.lat, 31.2000000045, places=9)
        self.assertAlmostEqual(result.point.lon, 121.4999999955, places=9)
        self.assertTrue(
            result.point.heading_deg < 2.0 or result.point.heading_deg > 358.0
        )

    # 测试作用：验证“rejects_invalid_sampling_conditions”场景的契约、输出结果和边界行为。
    def test_rejects_invalid_sampling_conditions(self):
        cases = (
            (make_samples(count=9), "SAMPLE_COUNT_INSUFFICIENT"),
            (make_samples(fixed=False), "RTK_NOT_FIXED"),
            (make_samples(gga_age=2.1), "RTK_STALE"),
            (make_samples(static=False), "VEHICLE_NOT_STATIC"),
        )
        for samples, expected_code in cases:
            with self.subTest(code=expected_code):
                result = self.api.sample_point(samples, 10, 0.05)
                self.assertFalse(result.success)
                self.assertEqual(result.code, expected_code)

    # 测试作用：验证“rejects_samples_with_more_than_five_centimetres_radius”场景的契约、输出结果和边界行为。
    def test_rejects_samples_with_more_than_five_centimetres_radius(self):
        samples = make_samples()
        samples[9].lat += 0.000001

        result = self.api.sample_point(samples, 10, 0.05)

        self.assertFalse(result.success)
        self.assertEqual(result.code, "RTK_SAMPLE_UNSTABLE")


if __name__ == "__main__":
    unittest.main()
