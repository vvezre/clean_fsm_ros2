# 文件作用：验证 deployment files 相关契约、运行逻辑和边界条件。
import importlib.util
import hashlib
import json
import tarfile
import tempfile
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
DEPLOYMENT = WORKSPACE / "deployment"


class DeploymentFilesTest(unittest.TestCase):
    # 辅助方法：读取或加载 read 所需的测试数据并返回解析结果。
    def read(self, relative):
        path = WORKSPACE / relative
        self.assertTrue(path.is_file(), f"missing deployment file: {path}")
        return path.read_text(encoding="utf-8")

    # 测试作用：验证“updater_config_is_strict_and_defaults_to_manual_apply”场景的契约、输出结果和边界行为。
    def test_updater_config_is_strict_and_defaults_to_manual_apply(self):
        updater_path = DEPLOYMENT / "updater" / "cleanbot_updater.py"
        spec = importlib.util.spec_from_file_location(
            "deployment_config_updater", updater_path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        config_path = DEPLOYMENT / "config" / "updater.json"

        config = module.parse_config(config_path.read_bytes())

        self.assertEqual(config.repository, "vvezre/clean_fsm_ros2")
        self.assertTrue(config.auto_download)
        self.assertFalse(config.auto_apply)
        self.assertEqual(config.service_name, "cleanbot.service")

    # 测试作用：验证“systemd_runs_robot_unprivileged_and_update_timer_as_root”场景的契约、输出结果和边界行为。
    def test_systemd_runs_robot_unprivileged_and_update_timer_as_root(self):
        robot = self.read("deployment/systemd/cleanbot.service")
        updater = self.read("deployment/systemd/cleanbot-update.service")
        timer = self.read("deployment/systemd/cleanbot-update.timer")

        for expected in (
            "User=cleanbot",
            "Group=cleanbot",
            "SupplementaryGroups=dialout",
            "ExecStart=/usr/local/libexec/cleanbot-start",
            "Restart=on-failure",
            "NoNewPrivileges=true",
            "ProtectSystem=strict",
        ):
            self.assertIn(expected, robot)
        self.assertNotIn("User=cleanbot", updater)
        self.assertIn(
            "ExecStart=/usr/local/sbin/cleanbot-updater "
            "--config /etc/cleanbot/updater.json auto",
            updater,
        )
        self.assertIn(
            "Environment=ROS_HOME=/var/lib/cleanbot/runtime/updater-ros-home",
            updater,
        )
        self.assertIn(
            "Environment=ROS_LOG_DIR=/var/lib/cleanbot/runtime/updater-ros-log",
            updater,
        )
        self.assertIn(
            "ReadWritePaths=/opt/cleanbot /var/lib/cleanbot/update "
            "/var/lib/cleanbot/runtime/updater-ros-home "
            "/var/lib/cleanbot/runtime/updater-ros-log",
            updater,
        )
        self.assertIn("OnUnitActiveSec=", timer)
        self.assertIn("Persistent=true", timer)
        self.assertIn("[Install]", updater)
        self.assertIn("WantedBy=multi-user.target", updater)

    # 测试作用：验证“runtime_wrappers_source_only_fixed_ros_and_release_paths”场景的契约、输出结果和边界行为。
    def test_runtime_wrappers_source_only_fixed_ros_and_release_paths(self):
        start = self.read("deployment/bin/cleanbot-start")
        updater = self.read("deployment/bin/cleanbot-updater")
        ros_cli = self.read("deployment/bin/cleanbot-ros-cli")

        for script in (start, updater, ros_cli):
            self.assertIn("set -euo pipefail", script)
            self.assertNotIn("eval ", script)
        self.assertIn("/opt/ros/humble/setup.bash", start)
        self.assertIn("/opt/ros/humble/setup.bash", ros_cli)
        self.assertIn(
            "/opt/cleanbot/current/install/setup.bash",
            start,
        )
        self.assertIn(
            "/opt/cleanbot/current/install/setup.bash",
            ros_cli,
        )
        self.assertNotIn(
            "/opt/cleanbot/current/install/setup.bash",
            updater,
        )
        self.assertIn(
            "/usr/local/lib/cleanbot/cleanbot_updater.py",
            updater,
        )
        self.assertIn("/usr/bin/flock", updater)
        self.assertIn("/run/lock/cleanbot-updater.lock", updater)

    # 测试作用：验证“udev_rules_are_inert_templates_until_serials_are_configured”场景的契约、输出结果和边界行为。
    def test_udev_rules_are_inert_templates_until_serials_are_configured(self):
        rules = self.read("deployment/udev/99-cleanbot.rules")

        self.assertIn('ATTRS{serial}=="REPLACE_LOWER_MACHINE_SERIAL"', rules)
        self.assertIn('SYMLINK+="cleanbot/lower-machine"', rules)
        self.assertIn('ATTRS{serial}=="REPLACE_RTK_SERIAL"', rules)
        self.assertIn('SYMLINK+="cleanbot/rtk"', rules)
        self.assertIn('GROUP="dialout"', rules)
        self.assertNotIn('KERNEL=="ttyUSB*"', rules)

    # 测试作用：验证“logrotate_and_install_rollback_entrypoints_exist”场景的契约、输出结果和边界行为。
    def test_logrotate_and_install_rollback_entrypoints_exist(self):
        logrotate = self.read("deployment/logrotate/cleanbot")
        install = self.read("deployment/install/install.sh")
        rollback = self.read("deployment/install/rollback.sh")

        self.assertIn("/var/log/cleanbot/ros/*.log", logrotate)
        self.assertIn("/var/log/cleanbot/ros/*/*.log", logrotate)
        self.assertIn("rotate 14", logrotate)
        self.assertIn("set -euo pipefail", install)
        self.assertIn("systemctl daemon-reload", install)
        self.assertIn("cleanbot-update.timer", install)
        self.assertIn(
            "cleanbot.service cleanbot-update.service cleanbot-update.timer",
            install,
        )
        self.assertIn('chmod 0755 "${temporary_release}"', install)
        self.assertGreater(
            install.index('mv -Tf -- "${temporary_link}" /opt/cleanbot/current'),
            install.index("systemctl daemon-reload"),
        )
        self.assertIn("cleanbot-updater", rollback)
        self.assertIn("rollback", rollback)

    # 测试作用：验证“install_bootstraps_maintenance_store_before_activation”场景的契约、输出结果和边界行为。
    def test_install_bootstraps_maintenance_store_before_activation(self):
        mission_cmake = self.read("src/cleanbot_mission/CMakeLists.txt")
        initializer = self.read(
            "src/cleanbot_mission/src/maintenance_store_init.cpp"
        )
        install = self.read("deployment/install/install.sh")

        self.assertIn(
            "add_executable(maintenance_store_init "
            "src/maintenance_store_init.cpp)",
            mission_cmake,
        )
        self.assertIn(
            "install(TARGETS mission_core mission_manager_node "
            "maintenance_store_init",
            mission_cmake,
        )
        self.assertIn("MaintenanceStore store(argv[1]);", initializer)
        self.assertIn("store.initializeGenesis()", initializer)

        init_command = (
            'runuser -u cleanbot -- "${maintenance_initializer}" '
            '"/var/lib/cleanbot/runtime/maintenance.lock"'
        )
        self.assertIn(init_command, install)
        self.assertLess(
            install.index(init_command),
            install.index(
                'mv -Tf -- "${temporary_link}" /opt/cleanbot/current'
            ),
        )

    # 测试作用：验证“arm64_release_workflow_builds_tests_signs_and_publishes”场景的契约、输出结果和边界行为。
    def test_arm64_release_workflow_builds_tests_signs_and_publishes(self):
        workflow = self.read(".github/workflows/arm64-release.yml")

        for expected in (
            "workflow_dispatch:",
            "runs-on: ubuntu-22.04-arm",
            "python3 -m unittest tests.test_deployment_files "
            "tests.test_deployment_updater -q",
            "colcon build",
            "colcon test",
            "colcon test-result --verbose",
            "build_release_assets.py",
            "openssl pkeyutl -sign -rawin",
            "gh release create",
            "CLEANBOT_RELEASE_SIGNING_KEY_B64",
            "environment: cleanbot-release",
            "persist-credentials: false",
            "if: github.ref == 'refs/heads/main'",
        ):
            self.assertIn(expected, workflow)
        self.assertIn(
            "uses: actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1",
            workflow,
        )
        self.assertIn(
            "uses: actions/upload-artifact@043fb46d1a93c77aae656e7c1c64a875d1fc6a0a",
            workflow,
        )
        self.assertIn(
            "uses: actions/download-artifact@3e5f45b2cfb9172054b4087a40e8e0b5a5461e7c",
            workflow,
        )
        self.assertIn("contents: read", workflow)
        self.assertIn("contents: write", workflow)
        self.assertLess(
            workflow.index("upload-artifact@"),
            workflow.index("CLEANBOT_RELEASE_SIGNING_KEY_B64"),
        )
        self.assertNotIn("pull_request:", workflow)
        self.assertNotIn("push:", workflow)

    # 测试作用：验证“arm64_release_adds_ros_repository_before_ros_build_tools”场景的契约、输出结果和边界行为。
    def test_arm64_release_adds_ros_repository_before_ros_build_tools(self):
        workflow = self.read(".github/workflows/arm64-release.yml")
        bootstrap_install = (
            "sudo apt-get install -y curl gnupg lsb-release"
        )
        repository_write = (
            "| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null"
        )
        ros_install = (
            "sudo apt-get install -y ros-humble-ros-base "
            "python3-colcon-common-extensions python3-rosdep"
        )

        self.assertIn(bootstrap_install, workflow)
        self.assertIn(repository_write, workflow)
        self.assertIn(ros_install, workflow)
        repository_index = workflow.index(repository_write)
        refreshed_index = workflow.index("sudo apt-get update", repository_index)
        self.assertLess(workflow.index(bootstrap_install), repository_index)
        self.assertLess(repository_index, refreshed_index)
        self.assertLess(refreshed_index, workflow.index(ros_install))

    # 测试作用：验证“release_builder_creates_reproducible_archive_and_canonical_manifest”场景的契约、输出结果和边界行为。
    def test_release_builder_creates_reproducible_archive_and_canonical_manifest(self):
        builder_path = (
            DEPLOYMENT / "release" / "build_release_assets.py"
        )
        spec = importlib.util.spec_from_file_location(
            "cleanbot_release_builder", builder_path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            install = root / "install"
            (install / "share").mkdir(parents=True)
            (install / "setup.bash").write_text(
                "#!/bin/bash\n", encoding="utf-8"
            )
            (install / "share" / "payload.txt").write_text(
                "payload\n", encoding="utf-8"
            )
            first = root / "first"
            second = root / "second"

            first_result = module.build_release_assets(
                release="v1.2.3",
                install_directory=install,
                output_directory=first,
                created_at="2026-07-29T00:00:00Z",
            )
            second_result = module.build_release_assets(
                release="v1.2.3",
                install_directory=install,
                output_directory=second,
                created_at="2026-07-29T00:00:00Z",
            )

            first_archive = first_result.archive.read_bytes()
            self.assertEqual(
                first_archive,
                second_result.archive.read_bytes(),
            )
            with tarfile.open(first_result.archive, "r:gz") as archive:
                names = archive.getnames()
                self.assertEqual(names, sorted(names))
                self.assertIn("VERSION", names)
                self.assertIn("install/setup.bash", names)
                self.assertEqual(
                    archive.extractfile("VERSION").read(),
                    b"v1.2.3\n",
                )
            manifest = json.loads(first_result.manifest.read_text("utf-8"))
            self.assertEqual(manifest["release"], "v1.2.3")
            self.assertEqual(manifest["target"], {
                "architecture": "arm64",
                "os": "ubuntu-22.04",
                "ros": "humble",
            })
            self.assertEqual(
                manifest["archive"]["sha256"],
                hashlib.sha256(first_archive).hexdigest(),
            )
            self.assertEqual(
                manifest["archive"]["size"],
                len(first_archive),
            )
            canonical = json.dumps(
                manifest,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8") + b"\n"
            self.assertEqual(first_result.manifest.read_bytes(), canonical)
            with self.assertRaises(ValueError):
                module.build_release_assets(
                    release="release-1",
                    install_directory=install,
                    output_directory=root / "invalid",
                    created_at="2026-07-29T00:00:00Z",
                )

    # 测试作用：验证“windows_bootstrap_and_emergency_rollback_do_not_embed_credentials”场景的契约、输出结果和边界行为。
    def test_windows_bootstrap_and_emergency_rollback_do_not_embed_credentials(self):
        bootstrap = self.read(
            "deployment/windows/bootstrap-pi.ps1"
        )
        rollback = self.read(
            "deployment/windows/emergency-rollback.ps1"
        )

        for script in (bootstrap, rollback):
            self.assertIn("BatchMode=yes", script)
            self.assertIn("ConnectTimeout=10", script)
            self.assertNotIn("njzt666", script)
            self.assertNotIn("sshpass", script)
        self.assertIn("install.sh", bootstrap)
        self.assertIn("scp", bootstrap)
        self.assertIn("/usr/local/sbin/cleanbot-rollback", rollback)

    # 测试作用：验证“deployment_readme_documents_manual_release_and_auto_apply_gate”场景的契约、输出结果和边界行为。
    def test_deployment_readme_documents_manual_release_and_auto_apply_gate(self):
        documentation = self.read("deployment/README.md")

        for expected in (
            "CLEANBOT_RELEASE_SIGNING_KEY_B64",
            "workflow_dispatch",
            "autoApply",
            "bootstrap-pi.ps1",
            "systemctl status cleanbot.service",
            "emergency-rollback.ps1",
        ):
            self.assertIn(expected, documentation)


if __name__ == "__main__":
    unittest.main()
