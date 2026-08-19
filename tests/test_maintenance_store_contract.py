# 文件作用：验证 maintenance store contract 相关契约、运行逻辑和边界条件。
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"
HEADER = PACKAGE / "include" / "cleanbot_mission" / "maintenance_store.hpp"
SOURCE = PACKAGE / "src" / "maintenance_store.cpp"
NATIVE_TEST = PACKAGE / "test" / "test_maintenance_store.cpp"


class MaintenanceStoreContractTest(unittest.TestCase):
    # 测试作用：验证“ros_free_store_files_are_present”场景的契约、输出结果和边界行为。
    def test_ros_free_store_files_are_present(self):
        self.assertTrue(HEADER.is_file(), "maintenance store header is missing")
        self.assertTrue(SOURCE.is_file(), "maintenance store source is missing")
        self.assertTrue(
            NATIVE_TEST.is_file(),
            "maintenance store native test is missing",
        )

    # 测试作用：验证“store_exposes_only_monotonic_transitions”场景的契约、输出结果和边界行为。
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

    # 测试作用：验证“linux_store_uses_guarded_durable_atomic_replace”场景的契约、输出结果和边界行为。
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

    # 测试作用：验证“windows_backend_is_macro_safe_and_anchors_local_path_chain”场景的契约、输出结果和边界行为。
    def test_windows_backend_is_macro_safe_and_anchors_local_path_chain(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertLess(
            source.index("#define NOMINMAX"),
            source.index("#include <windows.h>"),
        )
        for token in (
            "kDirectoryShareMode",
            "FILE_SHARE_READ | FILE_SHARE_WRITE",
            "anchor_directory_chain",
            "validate_parent_anchor",
            "directory_anchors_",
            "FILE_READ_ATTRIBUTES",
            "unsupported Windows maintenance state path",
            "LockedStore(const LockedStore&) = delete",
            "LockedStore& operator=(const LockedStore&) = delete",
            "LockedStore(LockedStore&&) = delete",
            "LockedStore& operator=(LockedStore&&) = delete",
        ):
            self.assertIn(token, source)

        anchor_start = source.index("anchor_directory_chain")
        anchor_end = source.index("validate_parent_anchor", anchor_start)
        anchor_body = source[anchor_start:anchor_end]
        self.assertNotIn("FILE_SHARE_DELETE", anchor_body)

    # 测试作用：验证“commit_results_are_built_before_nonthrowing_commit_point”场景的契约、输出结果和边界行为。
    def test_commit_results_are_built_before_nonthrowing_commit_point(self):
        source = SOURCE.read_text(encoding="utf-8")
        for token in (
            "is_nothrow_move_constructible<MaintenanceStoreResult>",
            "is_nothrow_move_assignable<MaintenanceStoreResult>",
            "committed_success",
            "post_commit_failure",
            "// Windows maintenance state commit point.",
            "// POSIX maintenance state commit point.",
        ):
            self.assertIn(token, source)

        windows_marker = source.index(
            "// Windows maintenance state commit point."
        )
        windows_function_end = source.index(
            "\n}\n\n#else",
            windows_marker,
        )
        windows_after_commit = source[windows_marker:windows_function_end]
        self.assertNotIn("make_result(", windows_after_commit)
        self.assertNotIn("std::string", windows_after_commit)

        posix_marker = source.index("// POSIX maintenance state commit point.")
        posix_function_end = source.index(
            "\n}\n\n#endif",
            posix_marker,
        )
        posix_after_commit = source[posix_marker:posix_function_end]
        self.assertNotIn("make_result(", posix_after_commit)
        self.assertNotIn("std::string", posix_after_commit)
        self.assertNotIn("result->committed", source)

        wrapper_start = source.index("MaintenanceStoreResult with_locked_store")
        wrapper_end = source.index(
            "\n}\n\nMaintenanceStoreResult exception_result",
            wrapper_start,
        )
        wrapper = source[wrapper_start:wrapper_end]
        self.assertLess(
            wrapper.index("finish_error_message"),
            wrapper.index("operation(store)"),
        )
        self.assertIn("return std::move(result)", wrapper)

    # 测试作用：验证“store_is_registered_with_json_linkage”场景的契约、输出结果和边界行为。
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
