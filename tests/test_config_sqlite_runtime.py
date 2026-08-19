# 文件作用：验证 config sqlite runtime 相关契约、运行逻辑和边界条件。
import ctypes
import os
import shutil
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
PACKAGE = WORKSPACE / "src" / "cleanbot_config"
REPOSITORY_HEADER = (
    PACKAGE / "include" / "cleanbot_config" / "sqlite_config_repository.hpp"
)
REPOSITORY_SOURCE = PACKAGE / "src" / "sqlite_config_repository.cpp"
RUNTIME_AVAILABLE = REPOSITORY_HEADER.is_file() and REPOSITORY_SOURCE.is_file()
if RUNTIME_AVAILABLE:
    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.add_include_path(str(PACKAGE / "src"))
    if SQLITE_INCLUDE.is_dir():
        cppyy.add_include_path(str(SQLITE_INCLUDE))
    if os.name == "nt":
        cppyy.load_library(str(SQLITE_BIN / "sqlite3.dll"))
    if not hasattr(getattr(cppyy.gbl, "cleanbot", object()), "config") or not hasattr(
        cppyy.gbl.cleanbot.config, "ConfigRegistry"
    ):
        cppyy.cppdef('#include "config_registry.cpp"\n')
    cppyy.cppdef('#include "sqlite_config_repository.cpp"\n')


class ConfigSqliteRuntimeTest(unittest.TestCase):
    # 测试初始化：为每个用例创建相互隔离的初始状态和输入。
    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp(prefix="cleanbot-config-"))

    # 测试清理：回收当前用例产生的临时文件、进程和状态。
    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    # 测试作用：验证“repository_source_exists”场景的契约、输出结果和边界行为。
    def test_repository_source_exists(self):
        self.assertTrue(REPOSITORY_HEADER.is_file())
        self.assertTrue(REPOSITORY_SOURCE.is_file())

    # 测试作用：验证“first_open_creates_schema_and_only_approved_defaults”场景的契约、输出结果和边界行为。
    @unittest.skipUnless(RUNTIME_AVAILABLE, "SQLite repository implementation is not present")
    def test_first_open_creates_schema_and_only_approved_defaults(self):
        registry = cppyy.gbl.cleanbot.config.ConfigRegistry()
        repository = cppyy.gbl.cleanbot.config.SqliteConfigRepository(
            str(self.temp_dir / "cleanbot.db"), 0
        )

        status = repository.open_and_initialize(registry)
        values = repository.read_all()

        self.assertTrue(status.healthy)
        self.assertTrue(status.writable)
        self.assertEqual(status.schema_version, 1)
        self.assertEqual(status.revision, 0)
        self.assertEqual(values["motion.base_forward_speed"], "350")
        self.assertEqual(values.count("hardware.lower_machine_port"), 0)
        self.assertEqual(values.count("rtk.port"), 0)

    # 测试作用：验证“transaction_write_increments_revision_and_survives_reopen”场景的契约、输出结果和边界行为。
    @unittest.skipUnless(RUNTIME_AVAILABLE, "SQLite repository implementation is not present")
    def test_transaction_write_increments_revision_and_survives_reopen(self):
        registry = cppyy.gbl.cleanbot.config.ConfigRegistry()
        database_path = str(self.temp_dir / "cleanbot.db")
        repository = cppyy.gbl.cleanbot.config.SqliteConfigRepository(database_path, 0)
        self.assertTrue(repository.open_and_initialize(registry).healthy)

        updates = cppyy.gbl.std.map["std::string", "std::string"]()
        updates["hardware.lower_machine_port"] = "/dev/ttyTHS1"
        updates["motion.base_forward_speed"] = "420"
        result = repository.write_values(updates, "runtime-test", registry)

        self.assertTrue(result.success)
        self.assertEqual(result.revision, 1)
        self.assertTrue(result.restart_required)
        self.assertEqual(list(result.changed_keys), [
            "hardware.lower_machine_port",
            "motion.base_forward_speed",
        ])
        repository.close()

        reopened = cppyy.gbl.cleanbot.config.SqliteConfigRepository(database_path, 0)
        status = reopened.open_and_initialize(registry)
        values = reopened.read_all()
        self.assertEqual(status.revision, 1)
        self.assertEqual(values["hardware.lower_machine_port"], "/dev/ttyTHS1")
        self.assertEqual(values["motion.base_forward_speed"], "420")

    # 测试作用：验证“missing_parent_directory_is_reported_without_silent_creation”场景的契约、输出结果和边界行为。
    @unittest.skipUnless(RUNTIME_AVAILABLE, "SQLite repository implementation is not present")
    def test_missing_parent_directory_is_reported_without_silent_creation(self):
        registry = cppyy.gbl.cleanbot.config.ConfigRegistry()
        repository = cppyy.gbl.cleanbot.config.SqliteConfigRepository(
            str(self.temp_dir / "missing" / "cleanbot.db"), 0
        )

        status = repository.open_and_initialize(registry)

        self.assertFalse(status.healthy)
        self.assertEqual(status.code, "CONFIG_DIRECTORY_MISSING")


if __name__ == "__main__":
    unittest.main()
