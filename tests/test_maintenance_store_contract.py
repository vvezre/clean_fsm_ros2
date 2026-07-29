import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"
HEADER = PACKAGE / "include" / "cleanbot_mission" / "maintenance_store.hpp"
SOURCE = PACKAGE / "src" / "maintenance_store.cpp"
NATIVE_TEST = PACKAGE / "test" / "test_maintenance_store.cpp"


class MaintenanceStoreContractTest(unittest.TestCase):
    def test_ros_free_store_files_are_present(self):
        self.assertTrue(HEADER.is_file(), "maintenance store header is missing")
        self.assertTrue(SOURCE.is_file(), "maintenance store source is missing")
        self.assertTrue(
            NATIVE_TEST.is_file(),
            "maintenance store native test is missing",
        )

    def test_store_exposes_only_monotonic_transitions(self):
        header = HEADER.read_text(encoding="utf-8")
        for token in (
            "class MaintenanceStore",
            "MaintenanceStoreResult load() const noexcept",
            "MaintenanceStoreResult initializeGenesis() noexcept",
            "MaintenanceStoreResult activate(",
            "MaintenanceStoreResult release(",
            "kUnsupportedSchema",
            "kGenerationExhausted",
            "bool committed",
        ):
            self.assertIn(token, header)

        for forbidden in (" save(", " clear("):
            self.assertNotIn(forbidden, header)

    def test_linux_store_uses_guarded_durable_atomic_replace(self):
        source = SOURCE.read_text(encoding="utf-8")
        for token in (
            "O_DIRECTORY",
            "O_NOFOLLOW",
            "O_CREAT",
            "O_EXCL",
            "flock",
            "fsync",
            "renameat",
            "unlinkat",
        ):
            self.assertIn(token, source)

    def test_store_is_registered_with_json_linkage(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for token in (
            "src/maintenance_store.cpp",
            "test_maintenance_store",
            "test/test_maintenance_store.cpp",
            "nlohmann_json::nlohmann_json",
        ):
            self.assertIn(token, cmake)


if __name__ == "__main__":
    unittest.main()
