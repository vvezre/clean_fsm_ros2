# 文件作用：验证 modeling sqlite contract 相关契约、运行逻辑和边界条件。
import ctypes
import os
import shutil
import sqlite3
import sysconfig
import tempfile
import unittest
from pathlib import Path


CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
SQLITE_BIN = Path(sysconfig.get_paths()["data"]) / "Library" / "bin"
SQLITE_INCLUDE = Path(sysconfig.get_paths()["data"]) / "Library" / "include"
if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
    os.add_dll_directory(str(CPPYY_BACKEND_BIN))
    if SQLITE_BIN.is_dir():
        os.add_dll_directory(str(SQLITE_BIN))
    os.environ["PATH"] = (
        str(CPPYY_BACKEND_BIN)
        + os.pathsep
        + str(SQLITE_BIN)
        + os.pathsep
        + os.environ.get("PATH", "")
    )
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
HEADER = PACKAGE / "include" / "cleanbot_modeling" / "sqlite_model_repository.hpp"
SOURCE = PACKAGE / "src" / "sqlite_model_repository.cpp"
CORE_LOADED = False


# 辅助方法：读取或加载 load_core 所需的测试数据并返回解析结果。
def load_core(test_case):
    global CORE_LOADED
    test_case.assertTrue(HEADER.is_file(), "SQLite model repository header is missing")
    test_case.assertTrue(SOURCE.is_file(), "SQLite model repository source is missing")
    if CORE_LOADED:
        return
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    if SQLITE_INCLUDE.is_dir():
        cppyy.add_include_path(str(SQLITE_INCLUDE))
    if os.name == "nt":
        cppyy.load_library(str(SQLITE_BIN / "sqlite3.dll"))
    try:
        already_loaded = hasattr(
            cppyy.gbl.cleanbot.modeling, "SqliteModelRepository"
        )
    except AttributeError:
        already_loaded = False
    if not already_loaded:
        sources = []
        try:
            modeling = cppyy.gbl.cleanbot.modeling
        except AttributeError:
            modeling = None
        if modeling is None or not hasattr(modeling, "lat_lon_to_local_cm"):
            sources.append('#include "geometry.cpp"')
        sources.append('#include "sqlite_model_repository.cpp"')
        cppyy.cppdef("\n".join(sources))
    CORE_LOADED = True


# 辅助方法：为 sample_model 测试场景准备输入、执行操作或整理结果。
def sample_model():
    api = cppyy.gbl.cleanbot.modeling
    model = api.CleaningModel()
    model.id = "model-1"
    model.name = "array"
    model.status = "draft"
    model.origin_lat = 31.2
    model.origin_lon = 121.5
    model.origin_valid = True
    model.recognition_confirmed = True

    group = api.ModelGroup()
    group.id = "group-1"
    group.name = "group"
    group.area_number = 1
    group.sweep_mode = "manual"
    group.sweep_angle_deg = 0.0
    group.recognition_status = "recognized"
    group.recognition_message = "ready"
    group.recognition_confidence = 0.98
    for index, (x, y) in enumerate(
        ((0, 0), (500, 0), (500, 1000), (0, 1000)), start=1
    ):
        point = api.ModelPoint()
        point.id = "p{}".format(index)
        point.sequence = index
        point.x_cm = x
        point.y_cm = y
        point.lat = 31.2
        point.lon = 121.5
        point.role = "boundary_corner"
        point.capture_type = "boundary"
        point.sample_count = 10
        point.sample_radius_m = 0.01
        point.fix_quality = 4
        group.points.push_back(point)
    area = api.ModelSubArea()
    area.id = "sub-1"
    area.name = "sub"
    area.confirmed = True
    for index in range(1, 5):
        area.point_ids.push_back("p{}".format(index))
    group.sub_areas.push_back(area)
    connector = api.ModelConnector()
    connector.id = "connector-1"
    connector.start_point_id = "p1"
    connector.end_point_id = "p2"
    connector.from_sub_area_id = "sub-1"
    connector.to_sub_area_id = "sub-1-next"
    connector.length_cm = 500.0
    connector.confirmed = False
    group.connectors.push_back(connector)
    model.groups.push_back(group)
    return model


# 辅助方法：为 sample_plan 测试场景准备输入、执行操作或整理结果。
def sample_plan(model_version):
    api = cppyy.gbl.cleanbot.modeling
    plan = api.CleaningPlan()
    plan.id = "plan-1"
    plan.model_id = "model-1"
    plan.model_version = model_version
    plan.plan_hash = "abc123"
    plan.brush_width_cm = 116.0
    plan.minimum_overlap_cm = 10.0
    plan.actual_overlap_cm = 39.2
    plan.cleaning_lane_count = 2
    plan.preview_confirmed = True
    segment = api.PlanSegment()
    segment.index = 0
    segment.id = "clean-1"
    segment.segment_type = 1
    segment.group_id = "group-1"
    segment.sub_area_id = "sub-1"
    segment.source_lane_id = "lane-1"
    segment.start.x_cm = 58.0
    segment.start.y_cm = 0.0
    segment.end.x_cm = 58.0
    segment.end.y_cm = 1000.0
    segment.heading_deg = 0.0
    segment.speed = 350
    segment.brush_enabled = True
    plan.segments.push_back(segment)
    return plan


class ModelingSqliteContractTest(unittest.TestCase):
    # 测试初始化：为每个用例创建相互隔离的初始状态和输入。
    def setUp(self):
        load_core(self)
        self.temp_dir = Path(tempfile.mkdtemp(prefix="cleanbot-modeling-"))

    # 测试清理：回收当前用例产生的临时文件、进程和状态。
    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    # 测试作用：验证“schema_and_draft_round_trip”场景的契约、输出结果和边界行为。
    def test_schema_and_draft_round_trip(self):
        database_path = self.temp_dir / "modeling.db"
        repository = cppyy.gbl.cleanbot.modeling.SqliteModelRepository(
            str(database_path)
        )
        status = repository.open_and_initialize()
        self.assertTrue(status.healthy)
        self.assertEqual(status.schema_version, 3)

        write = repository.save_draft(sample_model())
        self.assertTrue(write.success)
        loaded = cppyy.gbl.cleanbot.modeling.CleaningModel()
        self.assertTrue(repository.load_draft("model-1", loaded))
        self.assertEqual(loaded.name, "array")
        self.assertTrue(loaded.origin_valid)
        self.assertAlmostEqual(loaded.origin_lat, 31.2, places=9)
        self.assertAlmostEqual(loaded.origin_lon, 121.5, places=9)
        self.assertEqual(len(loaded.groups), 1)
        self.assertEqual(len(loaded.groups[0].points), 4)
        self.assertEqual(len(loaded.groups[0].sub_areas), 1)
        self.assertEqual(loaded.groups[0].recognition_status, "recognized")
        self.assertEqual(loaded.groups[0].recognition_message, "ready")
        self.assertAlmostEqual(loaded.groups[0].recognition_confidence, 0.98)
        self.assertEqual(loaded.groups[0].points[0].capture_type, "boundary")
        self.assertEqual(
            loaded.groups[0].connectors[0].from_sub_area_id, "sub-1"
        )
        self.assertEqual(
            loaded.groups[0].connectors[0].to_sub_area_id, "sub-1-next"
        )
        repository.close()

        with sqlite3.connect(database_path) as connection:
            tables = {
                row[0]
                for row in connection.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                )
            }
        for table in (
            "cleaning_models",
            "model_drafts",
            "model_versions",
            "model_groups",
            "model_points",
            "model_sub_areas",
            "model_connectors",
            "cleaning_plans",
            "plan_segments",
        ):
            self.assertIn(table, tables)

    # 测试作用：验证“formal_version_and_plan_survive_reopen”场景的契约、输出结果和边界行为。
    def test_formal_version_and_plan_survive_reopen(self):
        database_path = self.temp_dir / "modeling.db"
        repository = cppyy.gbl.cleanbot.modeling.SqliteModelRepository(
            str(database_path)
        )
        self.assertTrue(repository.open_and_initialize().healthy)
        model = sample_model()
        self.assertTrue(repository.save_draft(model).success)
        version_write = repository.save_formal_version(model)
        self.assertTrue(version_write.success)
        self.assertEqual(version_write.version, 1)
        self.assertTrue(repository.save_plan(sample_plan(1)).success)
        repository.close()

        reopened = cppyy.gbl.cleanbot.modeling.SqliteModelRepository(
            str(database_path)
        )
        self.assertTrue(reopened.open_and_initialize().healthy)
        loaded_model = cppyy.gbl.cleanbot.modeling.CleaningModel()
        loaded_plan = cppyy.gbl.cleanbot.modeling.CleaningPlan()
        self.assertTrue(reopened.load_version("model-1", 1, loaded_model))
        self.assertEqual(loaded_model.version, 1)
        self.assertTrue(reopened.load_plan("plan-1", loaded_plan))
        self.assertEqual(loaded_plan.plan_hash, "abc123")
        self.assertEqual(len(loaded_plan.segments), 1)

    # 测试作用：验证“schema_v1_migrates_to_one_shared_model_origin”场景的契约、输出结果和边界行为。
    def test_schema_v1_migrates_to_one_shared_model_origin(self):
        database_path = self.temp_dir / "legacy-modeling.db"
        with sqlite3.connect(database_path) as connection:
            connection.executescript(
                """
                CREATE TABLE modeling_metadata(
                    meta_key TEXT PRIMARY KEY, meta_value TEXT NOT NULL);
                INSERT INTO modeling_metadata VALUES('schema_version','1');
                CREATE TABLE cleaning_models(
                    model_id TEXT PRIMARY KEY,name TEXT NOT NULL,status TEXT NOT NULL,
                    current_version INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);
                CREATE TABLE model_points(
                    model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,
                    point_id TEXT NOT NULL,sequence INTEGER NOT NULL,
                    lat REAL NOT NULL,lon REAL NOT NULL,x_cm REAL NOT NULL,y_cm REAL NOT NULL,
                    heading_deg REAL NOT NULL,heading_valid INTEGER NOT NULL,
                    role TEXT NOT NULL,roles TEXT NOT NULL,sample_count INTEGER NOT NULL,
                    sample_radius_m REAL NOT NULL,fix_quality INTEGER NOT NULL,
                    gga_age_sec REAL NOT NULL,source TEXT NOT NULL,
                    PRIMARY KEY(model_id,version,group_id,point_id));
                INSERT INTO cleaning_models(model_id,name,status,current_version)
                    VALUES('legacy','legacy','draft',0);
                INSERT INTO model_points VALUES(
                    'legacy',0,'group-1','p1',1,31.2,121.5,0,0,
                    0,0,'unknown','',1,0,4,0,'rtk_mean');
                INSERT INTO model_points VALUES(
                    'legacy',0,'group-2','p2',1,31.2,121.5005,0,0,
                    0,0,'unknown','',1,0,4,0,'rtk_mean');
                """
            )

        repository = cppyy.gbl.cleanbot.modeling.SqliteModelRepository(
            str(database_path)
        )
        status = repository.open_and_initialize()
        self.assertTrue(status.healthy)
        self.assertEqual(status.schema_version, 3)
        repository.close()

        with sqlite3.connect(database_path) as connection:
            origin = connection.execute(
                "SELECT origin_lat,origin_lon,origin_valid "
                "FROM cleaning_models WHERE model_id='legacy'"
            ).fetchone()
            second_x = connection.execute(
                "SELECT x_cm FROM model_points "
                "WHERE model_id='legacy' AND group_id='group-2'"
            ).fetchone()[0]
        self.assertEqual(origin, (31.2, 121.5, 1))
        self.assertGreater(second_x, 4000.0)

    # 测试作用：验证“schema_v2_migrates_new_recognition_columns”场景的契约、输出结果和边界行为。
    def test_schema_v2_migrates_new_recognition_columns(self):
        database_path = self.temp_dir / "schema-v2-modeling.db"
        with sqlite3.connect(database_path) as connection:
            connection.executescript(
                """
                CREATE TABLE modeling_metadata(
                    meta_key TEXT PRIMARY KEY, meta_value TEXT NOT NULL);
                INSERT INTO modeling_metadata VALUES('schema_version','2');
                CREATE TABLE cleaning_models(
                    model_id TEXT PRIMARY KEY,name TEXT NOT NULL,status TEXT NOT NULL,
                    current_version INTEGER NOT NULL DEFAULT 0,
                    origin_lat REAL NOT NULL DEFAULT 0,origin_lon REAL NOT NULL DEFAULT 0,
                    origin_valid INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);
                CREATE TABLE model_groups(
                    model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,
                    group_order INTEGER NOT NULL,name TEXT NOT NULL,area_number INTEGER NOT NULL,
                    sweep_mode TEXT NOT NULL,sweep_angle_deg REAL NOT NULL,
                    PRIMARY KEY(model_id,version,group_id));
                CREATE TABLE model_points(
                    model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,
                    point_id TEXT NOT NULL,sequence INTEGER NOT NULL,
                    lat REAL NOT NULL,lon REAL NOT NULL,x_cm REAL NOT NULL,y_cm REAL NOT NULL,
                    heading_deg REAL NOT NULL,heading_valid INTEGER NOT NULL,
                    role TEXT NOT NULL,roles TEXT NOT NULL,sample_count INTEGER NOT NULL,
                    sample_radius_m REAL NOT NULL,fix_quality INTEGER NOT NULL,
                    gga_age_sec REAL NOT NULL,source TEXT NOT NULL,
                    PRIMARY KEY(model_id,version,group_id,point_id));
                CREATE TABLE model_connectors(
                    model_id TEXT NOT NULL,version INTEGER NOT NULL,group_id TEXT NOT NULL,
                    connector_id TEXT NOT NULL,connector_order INTEGER NOT NULL,type TEXT NOT NULL,
                    start_point_id TEXT NOT NULL,end_point_id TEXT NOT NULL,
                    length_cm REAL NOT NULL,confirmed INTEGER NOT NULL,
                    PRIMARY KEY(model_id,version,group_id,connector_id));
                """
            )

        repository = cppyy.gbl.cleanbot.modeling.SqliteModelRepository(
            str(database_path)
        )
        status = repository.open_and_initialize()
        self.assertTrue(status.healthy)
        self.assertEqual(status.schema_version, 3)
        repository.close()

        with sqlite3.connect(database_path) as connection:
            for table, column in (
                ("model_points", "capture_type"),
                ("model_groups", "recognition_status"),
                ("model_groups", "recognition_message"),
                ("model_groups", "recognition_confidence"),
                ("model_connectors", "from_sub_area_id"),
                ("model_connectors", "to_sub_area_id"),
            ):
                columns = {
                    row[1]
                    for row in connection.execute(
                        "PRAGMA table_info({})".format(table)
                    )
                }
                self.assertIn(column, columns)


if __name__ == "__main__":
    unittest.main()
