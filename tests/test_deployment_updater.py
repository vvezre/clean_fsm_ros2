import hashlib
import importlib.util
import io
import json
import os
import shutil
import stat
import subprocess
import tarfile
import tempfile
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    WORKSPACE / "deployment" / "updater" / "cleanbot_updater.py"
)


def load_updater_module():
    if not MODULE_PATH.is_file():
        raise AssertionError(f"updater module is missing: {MODULE_PATH}")
    spec = importlib.util.spec_from_file_location(
        "cleanbot_updater", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def manifest(archive_bytes=b"archive"):
    return {
        "schemaVersion": 1,
        "release": "v1.2.3",
        "createdAt": "2026-07-29T00:00:00Z",
        "target": {
            "architecture": "arm64",
            "os": "ubuntu-22.04",
            "ros": "humble",
        },
        "archive": {
            "name": "cleanbot-ros2-humble-arm64.tar.gz",
            "sha256": hashlib.sha256(archive_bytes).hexdigest(),
            "size": len(archive_bytes),
            "unpackedSize": 1024,
        },
    }


class FakeHttpResponse:
    def __init__(self, payload):
        self._payload = io.BytesIO(payload)

    def read(self, size=-1):
        return self._payload.read(size)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        return False


class DeploymentUpdaterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.updater = load_updater_module()

    def canonical(self, value):
        return self.updater.canonical_json_bytes(value)

    def test_parses_only_canonical_strict_manifest(self):
        value = manifest()
        encoded = self.canonical(value)

        parsed = self.updater.parse_manifest(
            encoded,
            expected_architecture="arm64",
            expected_os="ubuntu-22.04",
            expected_ros="humble",
        )

        self.assertEqual(parsed.release, "v1.2.3")
        self.assertEqual(parsed.archive.name, value["archive"]["name"])
        self.assertEqual(parsed.archive.size, len(b"archive"))
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                json.dumps(value, indent=2).encode("utf-8"),
                expected_architecture="arm64",
                expected_os="ubuntu-22.04",
                expected_ros="humble",
            )

    def test_rejects_duplicate_fields_unknown_fields_and_unsafe_release(self):
        duplicate = (
            b'{"archive":{},"createdAt":"x","release":"v1",'
            b'"release":"v2","schemaVersion":1,"target":{}}'
        )
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                duplicate, "arm64", "ubuntu-22.04", "humble"
            )

        unknown = manifest()
        unknown["unexpected"] = True
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                self.canonical(unknown),
                "arm64",
                "ubuntu-22.04",
                "humble",
            )

        unsafe = manifest()
        unsafe["release"] = "../escape"
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                self.canonical(unsafe),
                "arm64",
                "ubuntu-22.04",
                "humble",
            )

    def test_rejects_wrong_target_hash_and_size_bounds(self):
        for field, value in (
            ("architecture", "x86_64"),
            ("os", "ubuntu-24.04"),
            ("ros", "jazzy"),
        ):
            candidate = manifest()
            candidate["target"][field] = value
            with self.assertRaises(self.updater.ManifestError):
                self.updater.parse_manifest(
                    self.canonical(candidate),
                    "arm64",
                    "ubuntu-22.04",
                    "humble",
                )

        invalid_hash = manifest()
        invalid_hash["archive"]["sha256"] = "00"
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                self.canonical(invalid_hash),
                "arm64",
                "ubuntu-22.04",
                "humble",
            )

        oversized = manifest()
        oversized["archive"]["size"] = (
            self.updater.MAX_ARCHIVE_BYTES + 1
        )
        with self.assertRaises(self.updater.ManifestError):
            self.updater.parse_manifest(
                self.canonical(oversized),
                "arm64",
                "ubuntu-22.04",
                "humble",
            )

    def test_selects_three_assets_from_one_stable_release(self):
        names = self.updater.REQUIRED_RELEASE_ASSETS
        payload = {
            "tag_name": "v1.2.3",
            "draft": False,
            "prerelease": False,
            "assets": [
                {
                    "name": name,
                    "browser_download_url": f"https://example.invalid/{name}",
                    "size": 100,
                }
                for name in names
            ],
        }

        selected = self.updater.select_release_assets(payload)

        self.assertEqual(set(selected), set(names))
        self.assertTrue(all(item.name in names for item in selected.values()))
        payload["prerelease"] = True
        with self.assertRaises(self.updater.ReleaseError):
            self.updater.select_release_assets(payload)

    def test_rejects_duplicate_missing_or_non_https_release_assets(self):
        names = self.updater.REQUIRED_RELEASE_ASSETS
        base = {
            "tag_name": "v1.2.3",
            "draft": False,
            "prerelease": False,
            "assets": [],
        }
        missing = dict(base)
        missing["assets"] = [
            {
                "name": names[0],
                "browser_download_url": "https://example.invalid/a",
                "size": 1,
            }
        ]
        with self.assertRaises(self.updater.ReleaseError):
            self.updater.select_release_assets(missing)

        duplicate = dict(base)
        duplicate["assets"] = [
            {
                "name": name,
                "browser_download_url": f"https://example.invalid/{index}",
                "size": 1,
            }
            for index, name in enumerate((*names, names[0]))
        ]
        with self.assertRaises(self.updater.ReleaseError):
            self.updater.select_release_assets(duplicate)

        insecure = dict(base)
        insecure["assets"] = [
            {
                "name": name,
                "browser_download_url": f"http://example.invalid/{name}",
                "size": 1,
            }
            for name in names
        ]
        with self.assertRaises(self.updater.ReleaseError):
            self.updater.select_release_assets(insecure)

        oversized_metadata = dict(base)
        oversized_metadata["assets"] = [
            {
                "name": name,
                "browser_download_url": f"https://example.invalid/{name}",
                "size": (
                    self.updater.MAX_MANIFEST_BYTES + 1
                    if name == self.updater.MANIFEST_ASSET
                    else 1
                ),
            }
            for name in names
        ]
        with self.assertRaises(self.updater.ReleaseError):
            self.updater.select_release_assets(oversized_metadata)

    def test_fetches_latest_release_once_with_fixed_github_api_request(self):
        names = self.updater.REQUIRED_RELEASE_ASSETS
        payload = {
            "tag_name": "v1.2.3",
            "draft": False,
            "prerelease": False,
            "assets": [
                {
                    "name": name,
                    "browser_download_url": f"https://example.invalid/{name}",
                    "size": 100,
                }
                for name in names
            ],
        }
        calls = []

        def opener(request, *, timeout):
            calls.append((request, timeout))
            return FakeHttpResponse(json.dumps(payload).encode("utf-8"))

        release = self.updater.fetch_latest_release(
            "vvezre/clean_fsm_ros2",
            opener=opener,
            timeout_seconds=17,
        )

        self.assertEqual(release.tag, "v1.2.3")
        self.assertEqual(set(release.assets), set(names))
        self.assertEqual(len(calls), 1)
        request, timeout = calls[0]
        self.assertEqual(
            request.full_url,
            "https://api.github.com/repos/vvezre/clean_fsm_ros2/releases/latest",
        )
        self.assertEqual(timeout, 17)
        self.assertEqual(
            request.get_header("Accept"),
            "application/vnd.github+json",
        )
        self.assertIn("cleanbot-updater/", request.get_header("User-agent"))
        self.assertIsNone(request.get_header("Authorization"))

    def test_fetch_latest_release_rejects_oversized_or_invalid_json(self):
        for payload in (
            b"x" * (self.updater.MAX_RELEASE_RESPONSE_BYTES + 1),
            b"{not-json",
        ):
            with self.subTest(payload_size=len(payload)):
                with self.assertRaises(self.updater.ReleaseError):
                    self.updater.fetch_latest_release(
                        "vvezre/clean_fsm_ros2",
                        opener=lambda request, timeout: FakeHttpResponse(payload),
                    )

    def test_downloads_asset_to_atomic_destination_with_exact_size(self):
        payload = b"signed release bytes"
        asset = self.updater.ReleaseAsset(
            self.updater.MANIFEST_ASSET,
            "https://example.invalid/manifest.json",
            len(payload),
        )
        calls = []

        def opener(request, *, timeout):
            calls.append((request, timeout))
            return FakeHttpResponse(payload)

        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / asset.name
            result = self.updater.download_release_asset(
                asset,
                destination,
                opener=opener,
                timeout_seconds=19,
            )

            self.assertEqual(result, destination)
            self.assertEqual(destination.read_bytes(), payload)
            self.assertEqual(len(calls), 1)
            request, timeout = calls[0]
            self.assertEqual(request.full_url, asset.download_url)
            self.assertEqual(timeout, 19)
            self.assertEqual(
                request.get_header("Accept"),
                "application/octet-stream",
            )
            self.assertFalse(
                any(path.name.startswith(f".{asset.name}.download-")
                    for path in destination.parent.iterdir())
            )

    def test_download_rejects_wrong_size_and_leaves_no_partial_file(self):
        asset = self.updater.ReleaseAsset(
            self.updater.MANIFEST_ASSET,
            "https://example.invalid/manifest.json",
            4,
        )
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / asset.name
            with self.assertRaises(self.updater.ReleaseError):
                self.updater.download_release_asset(
                    asset,
                    destination,
                    opener=lambda request, timeout: FakeHttpResponse(b"too long"),
                )

            self.assertFalse(destination.exists())
            self.assertEqual(list(destination.parent.iterdir()), [])

    def test_verifies_archive_size_and_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "release.tar.gz"
            archive.write_bytes(b"archive")
            parsed = self.updater.parse_manifest(
                self.canonical(manifest()),
                "arm64",
                "ubuntu-22.04",
                "humble",
            )
            self.updater.verify_archive(archive, parsed.archive)
            archive.write_bytes(b"tampered")
            with self.assertRaises(self.updater.VerificationError):
                self.updater.verify_archive(archive, parsed.archive)

    def test_ed25519_manifest_signature_round_trip_with_openssl(self):
        openssl = shutil.which("openssl")
        if openssl is None:
            self.skipTest("openssl is unavailable")
        version = subprocess.run(
            [openssl, "version"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        if version.startswith("OpenSSL 1."):
            self.skipTest("OpenSSL 3 raw Ed25519 pkeyutl is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            private_key = root / "private.pem"
            public_key = root / "public.pem"
            manifest_path = root / self.updater.MANIFEST_ASSET
            signature = root / self.updater.SIGNATURE_ASSET
            manifest_path.write_bytes(self.canonical(manifest()))
            signed = subprocess.run(
                [
                    openssl,
                    "genpkey",
                    "-algorithm",
                    "ED25519",
                    "-out",
                    str(private_key),
                ],
                check=False,
                capture_output=True,
            )
            self.assertEqual(
                signed.returncode,
                0,
                signed.stderr.decode("utf-8", errors="replace"),
            )
            subprocess.run(
                [
                    openssl,
                    "pkey",
                    "-in",
                    str(private_key),
                    "-pubout",
                    "-out",
                    str(public_key),
                ],
                check=True,
                capture_output=True,
            )
            signed = subprocess.run(
                [
                    openssl,
                    "pkeyutl",
                    "-sign",
                    "-rawin",
                    "-inkey",
                    str(private_key),
                    "-in",
                    str(manifest_path),
                    "-out",
                    str(signature),
                ],
                check=False,
                capture_output=True,
            )
            self.assertEqual(
                signed.returncode,
                0,
                signed.stderr.decode("utf-8", errors="replace"),
            )

            self.updater.verify_manifest_signature(
                manifest_path,
                signature,
                public_key,
                openssl=openssl,
            )
            manifest_path.write_bytes(b"tampered\n")
            with self.assertRaises(self.updater.VerificationError):
                self.updater.verify_manifest_signature(
                    manifest_path,
                    signature,
                    public_key,
                    openssl=openssl,
                )

    def make_tar(self, path, entries):
        with tarfile.open(path, "w:gz") as archive:
            for name, kind, data in entries:
                info = tarfile.TarInfo(name)
                if kind == "file":
                    info.size = len(data)
                    info.mode = 0o644
                    archive.addfile(info, io.BytesIO(data))
                elif kind == "dir":
                    info.type = tarfile.DIRTYPE
                    info.mode = 0o755
                    archive.addfile(info)
                elif kind == "symlink":
                    info.type = tarfile.SYMTYPE
                    info.linkname = data.decode("utf-8")
                    archive.addfile(info)

    def test_secure_extract_accepts_regular_release_tree(self):
        if not hasattr(tarfile, "data_filter"):
            self.skipTest("secure tar data_filter is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "release.tar.gz"
            destination = root / "release"
            self.make_tar(
                archive,
                [
                    ("install/", "dir", b""),
                    ("install/setup.bash", "file", b"#!/bin/bash\n"),
                    ("install/payload.txt", "file", b"original\n"),
                    ("VERSION", "file", b"v1.2.3\n"),
                ],
            )

            self.updater.secure_extract(
                archive,
                destination,
                maximum_unpacked_bytes=1024,
            )

            self.assertEqual(
                (destination / "VERSION").read_text(encoding="utf-8"),
                "v1.2.3\n",
            )
            self.assertTrue((destination / "install/setup.bash").is_file())

    def test_secure_extract_rejects_traversal_links_and_duplicates(self):
        if not hasattr(tarfile, "data_filter"):
            self.skipTest("secure tar data_filter is unavailable")
        cases = (
            [("../escape", "file", b"x")],
            [("link", "symlink", b"/etc/passwd")],
            [
                ("VERSION", "file", b"a"),
                ("VERSION", "file", b"b"),
            ],
            [("dir\\windows-path", "file", b"x")],
        )
        for entries in cases:
            with self.subTest(entries=entries):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    archive = root / "bad.tar.gz"
                    self.make_tar(archive, entries)
                    with self.assertRaises(
                        self.updater.VerificationError
                    ):
                        self.updater.secure_extract(
                            archive,
                            root / "release",
                            maximum_unpacked_bytes=1024,
                        )

    def test_strict_config_defaults_to_download_without_automatic_apply(self):
        value = {
            "schemaVersion": 1,
            "repository": "vvezre/clean_fsm_ros2",
            "stagingRoot": "/var/lib/cleanbot/update/staging",
            "releasesRoot": "/opt/cleanbot/releases",
            "currentLink": "/opt/cleanbot/current",
            "statePath": "/var/lib/cleanbot/update/state.json",
            "publicKey": "/etc/cleanbot/update-public.pem",
            "serviceName": "cleanbot.service",
            "requester": "cleanbot-updater",
            "autoDownload": True,
            "autoApply": False,
        }
        parsed = self.updater.parse_config(
            json.dumps(value).encode("utf-8")
        )
        self.assertEqual(parsed.repository, "vvezre/clean_fsm_ros2")
        self.assertTrue(parsed.auto_download)
        self.assertFalse(parsed.auto_apply)

        value["unknown"] = 1
        with self.assertRaises(self.updater.ConfigError):
            self.updater.parse_config(json.dumps(value).encode("utf-8"))
        value.pop("unknown")
        value["currentLink"] = "relative/current"
        with self.assertRaises(self.updater.ConfigError):
            self.updater.parse_config(json.dumps(value).encode("utf-8"))

    def test_stage_release_verifies_signature_archive_and_version(self):
        if not hasattr(tarfile, "data_filter"):
            self.skipTest("secure tar data_filter is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / self.updater.ARCHIVE_ASSET
            self.make_tar(
                archive,
                [
                    ("install/", "dir", b""),
                    ("install/setup.bash", "file", b"#!/bin/bash\n"),
                    ("VERSION", "file", b"v1.2.3\n"),
                ],
            )
            archive_bytes = archive.read_bytes()
            value = manifest(archive_bytes)
            manifest_path = root / self.updater.MANIFEST_ASSET
            manifest_path.write_bytes(self.canonical(value))
            signature = root / self.updater.SIGNATURE_ASSET
            signature.write_bytes(b"signature")
            public_key = root / "public.pem"
            public_key.write_bytes(b"public")
            calls = []

            staged = self.updater.stage_release(
                manifest_path=manifest_path,
                signature_path=signature,
                archive_path=archive,
                public_key_path=public_key,
                staging_root=root / "staging",
                expected_architecture="arm64",
                expected_os="ubuntu-22.04",
                expected_ros="humble",
                signature_verifier=lambda *args: calls.append(args),
            )

            self.assertEqual(staged.release, "v1.2.3")
            self.assertEqual(calls, [(manifest_path, signature, public_key)])
            self.assertEqual(
                (staged.path / "payload" / "VERSION").read_text(
                    encoding="utf-8"
                ),
                "v1.2.3\n",
            )
            self.assertTrue((staged.path / self.updater.MANIFEST_ASSET).is_file())
            self.assertTrue((staged.path / self.updater.ARCHIVE_ASSET).is_file())
            with self.assertRaises(self.updater.VerificationError):
                self.updater.stage_release(
                    manifest_path=manifest_path,
                    signature_path=signature,
                    archive_path=archive,
                    public_key_path=public_key,
                    staging_root=root / "staging",
                    expected_architecture="arm64",
                    expected_os="ubuntu-22.04",
                    expected_ros="humble",
                    signature_verifier=lambda *args: None,
                )

    def test_reloads_staged_release_and_reverifies_signed_archive(self):
        if not hasattr(tarfile, "data_filter"):
            self.skipTest("secure tar data_filter is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / self.updater.ARCHIVE_ASSET
            self.make_tar(
                archive,
                [
                    ("install/", "dir", b""),
                    ("install/setup.bash", "file", b"#!/bin/bash\n"),
                    ("install/payload.txt", "file", b"original\n"),
                    ("VERSION", "file", b"v1.2.3\n"),
                ],
            )
            manifest_path = root / self.updater.MANIFEST_ASSET
            manifest_path.write_bytes(
                self.canonical(manifest(archive.read_bytes()))
            )
            signature = root / self.updater.SIGNATURE_ASSET
            signature.write_bytes(b"signature")
            public_key = root / "public.pem"
            public_key.write_bytes(b"public")
            staged = self.updater.stage_release(
                manifest_path=manifest_path,
                signature_path=signature,
                archive_path=archive,
                public_key_path=public_key,
                staging_root=root / "staging",
                expected_architecture="arm64",
                expected_os="ubuntu-22.04",
                expected_ros="humble",
                signature_verifier=lambda *args: None,
            )
            calls = []

            reloaded = self.updater.load_staged_release(
                staged.path,
                "v1.2.3",
                public_key,
                signature_verifier=lambda *args: calls.append(args),
            )

            self.assertEqual(reloaded.release, "v1.2.3")
            self.assertEqual(len(calls), 1)
            (staged.path / "payload" / "install" / "payload.txt").write_text(
                "tampered\n", encoding="utf-8"
            )
            self.updater.load_staged_release(
                staged.path,
                "v1.2.3",
                public_key,
                signature_verifier=lambda *args: None,
            )
            self.assertEqual(
                (
                    staged.path
                    / "payload"
                    / "install"
                    / "payload.txt"
                ).read_text(encoding="utf-8"),
                "original\n",
            )
            (staged.path / self.updater.ARCHIVE_ASSET).write_bytes(
                b"tampered"
            )
            with self.assertRaises(self.updater.VerificationError):
                self.updater.load_staged_release(
                    staged.path,
                    "v1.2.3",
                    public_key,
                    signature_verifier=lambda *args: None,
                )

    def test_update_state_is_strict_and_atomically_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "state.json"
            state = self.updater.UpdateState(
                current_release="v1.2.2",
                previous_release="v1.2.1",
                staged_release="v1.2.3",
                phase="STAGED",
                last_error="",
            )

            self.updater.write_update_state(state_path, state)
            loaded = self.updater.load_update_state(state_path)

            self.assertEqual(loaded, state)
            self.assertEqual(
                state_path.read_bytes(),
                self.updater.canonical_json_bytes(
                    self.updater.update_state_object(state)
                ),
            )
            state_path.write_text('{"schemaVersion":1}', encoding="utf-8")
            with self.assertRaises(self.updater.StateError):
                self.updater.load_update_state(state_path)

    @unittest.skipIf(os.name == "nt", "POSIX symlink switching is Linux-only")
    def test_current_release_symlink_switch_is_contained_and_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            releases = root / "releases"
            releases.mkdir()
            first = releases / "v1.2.2"
            second = releases / "v1.2.3"
            first.mkdir()
            second.mkdir()
            current = root / "current"
            current.symlink_to(first)

            previous = self.updater.switch_current_release(
                current, second, releases
            )

            self.assertEqual(previous, first.resolve())
            self.assertEqual(current.resolve(), second.resolve())
            outside = root / "outside"
            outside.mkdir()
            with self.assertRaises(self.updater.VerificationError):
                self.updater.switch_current_release(
                    current, outside, releases
                )

    def test_parses_maintenance_service_and_state_output_fail_closed(self):
        accepted = self.updater.parse_maintenance_service_output(
            "response:\naccepted: true\ngate_active: true\n"
            "ready: false\ngeneration: 42\ncode: OK\n"
        )
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.generation, 42)
        self.assertFalse(accepted.ready)
        self.assertTrue(
            self.updater.parse_maintenance_ready(
                "gate_active: true\nready: true\ngeneration: 42\n",
                generation=42,
            )
        )
        for output in (
            "",
            "accepted: false\ngeneration: 42",
            "accepted: true\ngeneration: 0",
        ):
            with self.assertRaises(self.updater.VerificationError):
                self.updater.parse_maintenance_service_output(output)
        self.assertFalse(
            self.updater.parse_maintenance_ready(
                "gate_active: true\nready: true\ngeneration: 41\n",
                generation=42,
            )
        )

    def test_maintenance_service_parser_uses_response_generation_not_request(self):
        output = (
            "requester: making request: "
            "cleanbot_interfaces.srv.SetMaintenanceMode_Request("
            "enable=True, generation=0, requester='cleanbot-updater', "
            "reason='software-update')\n\n"
            "response:\n"
            "cleanbot_interfaces.srv.SetMaintenanceMode_Response("
            "accepted=True, gate_active=True, ready=False, "
            "generation=42, code='OK', message='accepted')\n"
        )

        result = self.updater.parse_maintenance_service_output(output)

        self.assertEqual(result.generation, 42)
        self.assertTrue(result.accepted)
        self.assertTrue(result.gate_active)

    def test_calls_maintenance_service_with_fixed_argv_and_exact_token(self):
        calls = []

        def runner(argv, **options):
            calls.append((argv, options))
            return subprocess.CompletedProcess(
                argv,
                0,
                stdout=(
                    "accepted: true\ngate_active: true\nready: false\n"
                    "generation: 42\ncode: OK\n"
                ),
                stderr="",
            )

        enabled = self.updater.call_maintenance_service(
            enable=True,
            generation=0,
            requester="cleanbot-updater",
            reason="software-update",
            runner=runner,
            timeout_seconds=12,
        )
        released = self.updater.call_maintenance_service(
            enable=False,
            generation=enabled.generation,
            requester="cleanbot-updater",
            reason="software-update",
            runner=runner,
            timeout_seconds=12,
        )

        self.assertEqual(released.generation, 42)
        self.assertEqual(len(calls), 2)
        enable_argv, enable_options = calls[0]
        disable_argv, _ = calls[1]
        self.assertEqual(
            enable_argv[:6],
            [
                "/usr/sbin/runuser",
                "--user",
                "cleanbot",
                "--",
                "/usr/local/libexec/cleanbot-ros-cli",
                "service",
            ],
        )
        self.assertEqual(
            enable_argv[8],
            "cleanbot_interfaces/srv/SetMaintenanceMode",
        )
        self.assertEqual(
            json.loads(enable_argv[9]),
            {
                "enable": True,
                "generation": 0,
                "requester": "cleanbot-updater",
                "reason": "software-update",
            },
        )
        self.assertEqual(json.loads(disable_argv[9])["generation"], 42)
        self.assertFalse(json.loads(disable_argv[9])["enable"])
        self.assertEqual(enable_options["timeout"], 12)
        self.assertFalse(enable_options["shell"])

    def test_maintenance_service_rejects_invalid_input_or_command_failure(self):
        for requester in ("", "../unsafe", "space value"):
            with self.subTest(requester=requester):
                with self.assertRaises(self.updater.VerificationError):
                    self.updater.call_maintenance_service(
                        enable=True,
                        generation=0,
                        requester=requester,
                        reason="software-update",
                        runner=lambda *args, **kwargs: None,
                    )

        def failed_runner(argv, **options):
            return subprocess.CompletedProcess(
                argv, 1, stdout="", stderr="service unavailable"
            )

        with self.assertRaises(self.updater.VerificationError):
            self.updater.call_maintenance_service(
                enable=True,
                generation=0,
                requester="cleanbot-updater",
                reason="software-update",
                runner=failed_runner,
            )

    def test_reads_exact_generation_maintenance_readiness_with_fixed_argv(self):
        calls = []

        def runner(argv, **options):
            calls.append((argv, options))
            return subprocess.CompletedProcess(
                argv,
                0,
                stdout=(
                    "gate_active: true\nready: true\ngeneration: 42\n"
                ),
                stderr="",
            )

        self.assertTrue(
            self.updater.read_maintenance_ready(
                generation=42,
                runner=runner,
                timeout_seconds=8,
            )
        )
        argv, options = calls[0]
        self.assertEqual(
            argv,
            [
                "/usr/sbin/runuser",
                "--user",
                "cleanbot",
                "--",
                "/usr/local/libexec/cleanbot-ros-cli",
                "topic",
                "echo",
                "--once",
                "/system/maintenance_state",
                "cleanbot_interfaces/msg/MaintenanceState",
            ],
        )
        self.assertFalse(options["shell"])

    def test_controls_only_valid_cleanbot_systemd_service_with_fixed_argv(self):
        calls = []

        def runner(argv, **options):
            calls.append((argv, options))
            return subprocess.CompletedProcess(
                argv, 0, stdout="", stderr=""
            )

        self.updater.control_systemd_service(
            "stop",
            "cleanbot.service",
            runner=runner,
            timeout_seconds=23,
        )

        self.assertEqual(calls[0][0], [
            "systemctl", "stop", "cleanbot.service"
        ])
        self.assertEqual(calls[0][1]["timeout"], 23)
        self.assertFalse(calls[0][1]["shell"])
        for action, service in (
            ("restart", "cleanbot.service"),
            ("stop", "../unsafe.service"),
        ):
            with self.assertRaises(self.updater.VerificationError):
                self.updater.control_systemd_service(
                    action, service, runner=runner
                )

    def test_health_check_requires_systemd_and_exact_maintenance_service_type(self):
        calls = []

        def runner(argv, **options):
            calls.append(argv)
            if argv[:3] == ["systemctl", "is-active", "--quiet"]:
                return subprocess.CompletedProcess(
                    argv, 0, stdout="", stderr=""
                )
            return subprocess.CompletedProcess(
                argv,
                0,
                stdout="cleanbot_interfaces/srv/SetMaintenanceMode\n",
                stderr="",
            )

        self.assertTrue(
            self.updater.check_cleanbot_health(
                "cleanbot.service", runner=runner
            )
        )
        self.assertEqual(
            calls,
            [
                [
                    "systemctl",
                    "is-active",
                    "--quiet",
                    "cleanbot.service",
                ],
                [
                    "/usr/sbin/runuser",
                    "--user",
                    "cleanbot",
                    "--",
                    "/usr/local/libexec/cleanbot-ros-cli",
                    "service",
                    "type",
                    "/system/set_maintenance",
                ],
            ],
        )

        def wrong_type(argv, **options):
            return subprocess.CompletedProcess(
                argv, 0, stdout="wrong/Service\n", stderr=""
            )

        self.assertFalse(
            self.updater.check_cleanbot_health(
                "cleanbot.service", runner=wrong_type
            )
        )

    def test_installs_staged_payload_into_release_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staged = root / "staging" / "v1.2.3"
            payload = staged / "payload"
            (payload / "install").mkdir(parents=True)
            (payload / "VERSION").write_text(
                "v1.2.3\n", encoding="utf-8"
            )
            (payload / "install" / "setup.bash").write_text(
                "#!/bin/bash\n", encoding="utf-8"
            )
            releases = root / "releases"

            installed = self.updater.install_staged_payload(
                staged,
                "v1.2.3",
                releases,
            )

            self.assertEqual(installed, releases / "v1.2.3")
            self.assertEqual(
                (installed / "VERSION").read_text(encoding="utf-8"),
                "v1.2.3\n",
            )
            self.assertTrue((installed / "install" / "setup.bash").is_file())
            self.assertFalse(
                any(path.name.startswith(".v1.2.3.install-")
                    for path in releases.iterdir())
            )

    @unittest.skipIf(os.name == "nt", "POSIX release permissions are Linux-only")
    def test_installed_release_root_is_traversable_by_service_user(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staged = root / "staging" / "v1.2.3"
            payload = staged / "payload"
            (payload / "install").mkdir(parents=True)
            (payload / "VERSION").write_text(
                "v1.2.3\n", encoding="utf-8"
            )
            (payload / "install" / "setup.bash").write_text(
                "#!/bin/bash\n", encoding="utf-8"
            )
            payload.chmod(0o700)

            installed = self.updater.install_staged_payload(
                staged, "v1.2.3", root / "releases"
            )

            self.assertEqual(stat.S_IMODE(installed.stat().st_mode), 0o755)

    def updater_config_for(self, root):
        return self.updater.UpdaterConfig(
            repository="vvezre/clean_fsm_ros2",
            staging_root=str(root / "staging"),
            releases_root=str(root / "releases"),
            current_link=str(root / "current"),
            state_path=str(root / "state.json"),
            public_key=str(root / "public.pem"),
            service_name="cleanbot.service",
            requester="cleanbot-updater",
            auto_download=True,
            auto_apply=False,
        )

    def test_apply_release_stops_switches_health_checks_and_releases_token(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            old_release = root / "releases" / "v1.2.2"
            new_release = root / "releases" / "v1.2.3"
            old_release.mkdir(parents=True)
            events = []
            states = []

            def maintenance_call(**request):
                events.append(
                    (
                        "maintenance",
                        request["enable"],
                        request["generation"],
                    )
                )
                return self.updater.MaintenanceServiceResult(
                    True,
                    request["enable"],
                    False,
                    42,
                    "OK",
                )

            def ready(generation):
                events.append(("ready", generation))
                return True

            def service(action, service_name):
                events.append(("service", action, service_name))

            def install(staged, release, releases):
                events.append(("install", release))
                new_release.mkdir(parents=True)
                return new_release

            def switch(current, target, releases):
                events.append(("switch", target.name))
                return old_release

            def health(service_name):
                events.append(("health", service_name))
                return True

            initial = self.updater.UpdateState(
                current_release="v1.2.2",
                previous_release="v1.2.1",
                staged_release="v1.2.3",
                phase="STAGED",
                last_error="",
            )
            staged = self.updater.StagedRelease(
                "v1.2.3", staged_path, None
            )

            final = self.updater.apply_staged_release(
                config,
                initial,
                staged,
                maintenance_call=maintenance_call,
                readiness_check=ready,
                service_control=service,
                installer=install,
                switcher=switch,
                health_check=health,
                state_writer=lambda path, state: states.append(state),
                readiness_attempts=1,
                health_attempts=1,
                sleeper=lambda seconds: None,
            )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("ready", 42),
                    ("service", "stop", "cleanbot.service"),
                    ("install", "v1.2.3"),
                    ("switch", "v1.2.3"),
                    ("service", "start", "cleanbot.service"),
                    ("health", "cleanbot.service"),
                    ("ready", 42),
                    ("maintenance", False, 42),
                ],
            )
            self.assertEqual(final.current_release, "v1.2.3")
            self.assertEqual(final.previous_release, "v1.2.2")
            self.assertEqual(final.staged_release, "")
            self.assertEqual(final.phase, "IDLE")
            self.assertEqual(
                [state.phase for state in states],
                ["APPLYING", "HEALTH_CHECK", "IDLE"],
            )

    def test_apply_release_rolls_back_when_new_service_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            old_release = root / "releases" / "v1.2.2"
            new_release = root / "releases" / "v1.2.3"
            old_release.mkdir(parents=True)
            events = []
            states = []
            health_results = iter((False, True))

            def maintenance_call(**request):
                events.append(
                    (
                        "maintenance",
                        request["enable"],
                        request["generation"],
                    )
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 42, "OK"
                )

            def switch(current, target, releases):
                events.append(("switch", target.name))
                if target == new_release:
                    return old_release
                return new_release

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.apply_staged_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.2", "v1.2.1", "v1.2.3", "STAGED", ""
                    ),
                    self.updater.StagedRelease(
                        "v1.2.3", staged_path, None
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    installer=lambda staged, release, releases: new_release,
                    switcher=switch,
                    health_check=lambda service: next(health_results),
                    state_writer=lambda path, state: states.append(state),
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.3"),
                    ("service", "start"),
                    ("service", "stop"),
                    ("switch", "v1.2.2"),
                    ("service", "start"),
                    ("maintenance", False, 42),
                ],
            )
            self.assertEqual(
                [state.phase for state in states],
                [
                    "APPLYING",
                    "HEALTH_CHECK",
                    "ROLLING_BACK",
                    "FAILED",
                ],
            )
            self.assertEqual(states[-1].current_release, "v1.2.2")
            self.assertEqual(states[-1].staged_release, "v1.2.3")
            self.assertNotEqual(states[-1].last_error, "")

    def test_apply_keeps_maintenance_active_when_rollback_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            old_release = root / "releases" / "v1.2.2"
            new_release = root / "releases" / "v1.2.3"
            old_release.mkdir(parents=True)
            maintenance_requests = []

            def maintenance_call(**request):
                maintenance_requests.append(
                    (request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 61, "OK"
                )

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.apply_staged_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.2", "v1.2.1", "v1.2.3", "STAGED", ""
                    ),
                    self.updater.StagedRelease(
                        "v1.2.3", staged_path, None
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: None,
                    installer=lambda staged, release, releases: new_release,
                    switcher=lambda current, target, releases: (
                        old_release if target == new_release else new_release
                    ),
                    health_check=lambda service: False,
                    state_writer=lambda path, state: None,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(maintenance_requests, [(True, 0)])

    def test_apply_stays_stopped_if_switch_has_no_rollback_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            new_release = root / "releases" / "v1.2.3"
            events = []
            states = []

            def maintenance_call(**request):
                events.append(
                    ("maintenance", request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 81, "OK"
                )

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.apply_staged_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.2", "v1.2.1", "v1.2.3", "STAGED", ""
                    ),
                    self.updater.StagedRelease(
                        "v1.2.3", staged_path, None
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    installer=lambda staged, release, releases: new_release,
                    switcher=lambda current, target, releases: (
                        events.append(("switch", target.name)) or None
                    ),
                    health_check=lambda service: self.fail(
                        "unknown switched state must not be started"
                    ),
                    state_writer=lambda path, state: states.append(state),
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.3"),
                ],
            )
            self.assertEqual(states[-1].phase, "FAILED")
            self.assertEqual(states[-1].current_release, "v1.2.3")

    def test_apply_does_not_rollback_after_maintenance_was_released(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            old_release = root / "releases" / "v1.2.2"
            new_release = root / "releases" / "v1.2.3"
            old_release.mkdir(parents=True)
            events = []

            def maintenance_call(**request):
                events.append(
                    ("maintenance", request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 91, "OK"
                )

            def state_writer(path, state):
                if state.phase == "IDLE":
                    raise self.updater.StateError("disk full")

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.apply_staged_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.2", "v1.2.1", "v1.2.3", "STAGED", ""
                    ),
                    self.updater.StagedRelease(
                        "v1.2.3", staged_path, None
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    installer=lambda staged, release, releases: new_release,
                    switcher=lambda current, target, releases: (
                        events.append(("switch", target.name)) or old_release
                    ),
                    health_check=lambda service: True,
                    state_writer=state_writer,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.3"),
                    ("service", "start"),
                    ("maintenance", False, 91),
                ],
            )

    def test_apply_release_timeout_releases_maintenance_without_stopping_service(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staged_path = root / "staging" / "v1.2.3"
            staged_path.mkdir(parents=True)
            events = []
            states = []

            def maintenance_call(**request):
                events.append(
                    (
                        "maintenance",
                        request["enable"],
                        request["generation"],
                    )
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 42, "OK"
                )

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.apply_staged_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.2", "v1.2.1", "v1.2.3", "STAGED", ""
                    ),
                    self.updater.StagedRelease(
                        "v1.2.3", staged_path, None
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: False,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    installer=lambda *args: self.fail(
                        "installer must not run"
                    ),
                    switcher=lambda *args: self.fail(
                        "switcher must not run"
                    ),
                    health_check=lambda service: self.fail(
                        "health check must not run"
                    ),
                    state_writer=lambda path, state: states.append(state),
                    readiness_attempts=2,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("maintenance", False, 42),
                ],
            )
            self.assertEqual(
                [state.phase for state in states],
                ["APPLYING", "FAILED"],
            )

    def test_stages_latest_release_from_one_github_release_response(self):
        if not hasattr(tarfile, "data_filter"):
            self.skipTest("secure tar data_filter is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            archive_path = root / self.updater.ARCHIVE_ASSET
            self.make_tar(
                archive_path,
                [
                    ("install/", "dir", b""),
                    ("install/setup.bash", "file", b"#!/bin/bash\n"),
                    ("install/payload.txt", "file", b"original\n"),
                    ("VERSION", "file", b"v1.2.3\n"),
                ],
            )
            archive_bytes = archive_path.read_bytes()
            manifest_bytes = self.canonical(manifest(archive_bytes))
            signature_bytes = b"signed"
            (root / "public.pem").write_bytes(b"public")
            asset_payloads = {
                self.updater.ARCHIVE_ASSET: archive_bytes,
                self.updater.MANIFEST_ASSET: manifest_bytes,
                self.updater.SIGNATURE_ASSET: signature_bytes,
            }
            release_payload = {
                "tag_name": "v1.2.3",
                "draft": False,
                "prerelease": False,
                "assets": [
                    {
                        "name": name,
                        "browser_download_url": (
                            f"https://example.invalid/{name}"
                        ),
                        "size": len(payload),
                    }
                    for name, payload in asset_payloads.items()
                ],
            }
            api_url = (
                "https://api.github.com/repos/"
                "vvezre/clean_fsm_ros2/releases/latest"
            )
            responses = {
                api_url: json.dumps(release_payload).encode("utf-8"),
                **{
                    f"https://example.invalid/{name}": payload
                    for name, payload in asset_payloads.items()
                },
            }
            calls = []

            def opener(request, *, timeout):
                calls.append(request.full_url)
                return FakeHttpResponse(responses[request.full_url])

            states = []
            staged = self.updater.stage_latest_release(
                config,
                self.updater.UpdateState(
                    "v1.2.2", "v1.2.1", "", "IDLE", ""
                ),
                opener=opener,
                signature_verifier=lambda *args: None,
                state_writer=lambda path, state: states.append(state),
            )

            self.assertIsNotNone(staged)
            self.assertEqual(staged.release, "v1.2.3")
            self.assertTrue(
                (staged.path / "payload" / "install" / "setup.bash").is_file()
            )
            self.assertEqual(calls[0], api_url)
            self.assertEqual(
                set(calls[1:]),
                {
                    f"https://example.invalid/{name}"
                    for name in asset_payloads
                },
            )
            self.assertEqual(
                [state.phase for state in states],
                ["DOWNLOADING", "STAGED"],
            )
            self.assertEqual(states[-1].staged_release, "v1.2.3")
            self.assertEqual(
                [path.name for path in Path(config.staging_root).iterdir()],
                ["v1.2.3"],
            )

    def test_stage_latest_release_does_nothing_when_already_current(self):
        payload = {
            "tag_name": "v1.2.3",
            "draft": False,
            "prerelease": False,
            "assets": [
                {
                    "name": name,
                    "browser_download_url": f"https://example.invalid/{name}",
                    "size": 1,
                }
                for name in self.updater.REQUIRED_RELEASE_ASSETS
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            states = []
            staged = self.updater.stage_latest_release(
                self.updater_config_for(Path(directory)),
                self.updater.UpdateState(
                    "v1.2.3", "v1.2.2", "", "IDLE", ""
                ),
                opener=lambda request, timeout: FakeHttpResponse(
                    json.dumps(payload).encode("utf-8")
                ),
                signature_verifier=lambda *args: self.fail(
                    "signature verification must not run"
                ),
                state_writer=lambda path, state: states.append(state),
            )

            self.assertIsNone(staged)
            self.assertEqual(states, [])

    def test_stage_latest_release_rejects_automatic_downgrade(self):
        payload = {
            "tag_name": "v1.2.2",
            "draft": False,
            "prerelease": False,
            "assets": [
                {
                    "name": name,
                    "browser_download_url": f"https://example.invalid/{name}",
                    "size": 1,
                }
                for name in self.updater.REQUIRED_RELEASE_ASSETS
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            states = []
            with self.assertRaises(self.updater.UpdaterError):
                self.updater.stage_latest_release(
                    self.updater_config_for(Path(directory)),
                    self.updater.UpdateState(
                        "v1.2.3", "v1.2.1", "", "IDLE", ""
                    ),
                    opener=lambda request, timeout: FakeHttpResponse(
                        json.dumps(payload).encode("utf-8")
                    ),
                    signature_verifier=lambda *args: self.fail(
                        "downgrade assets must not be downloaded"
                    ),
                    state_writer=lambda path, state: states.append(state),
                )

            self.assertEqual(states, [])

    def test_failed_download_state_can_retry_staging(self):
        payload = {
            "tag_name": "v1.2.3",
            "draft": False,
            "prerelease": False,
            "assets": [
                {
                    "name": name,
                    "browser_download_url": f"https://example.invalid/{name}",
                    "size": 1,
                }
                for name in self.updater.REQUIRED_RELEASE_ASSETS
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            states = []
            staged = self.updater.stage_latest_release(
                self.updater_config_for(Path(directory)),
                self.updater.UpdateState(
                    "v1.2.3", "v1.2.2", "", "FAILED", "network timeout"
                ),
                opener=lambda request, timeout: FakeHttpResponse(
                    json.dumps(payload).encode("utf-8")
                ),
                signature_verifier=lambda *args: self.fail(
                    "current release assets must not be downloaded"
                ),
                state_writer=lambda path, state: states.append(state),
            )

            self.assertIsNone(staged)
            self.assertEqual(len(states), 1)
            self.assertEqual(states[0].phase, "IDLE")
            self.assertEqual(states[0].last_error, "")

    def test_manual_rollback_switches_to_previous_healthy_release(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            current.mkdir(parents=True)
            previous.mkdir(parents=True)
            events = []
            states = []

            def maintenance_call(**request):
                events.append(
                    (
                        "maintenance",
                        request["enable"],
                        request["generation"],
                    )
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 51, "OK"
                )

            final = self.updater.rollback_to_previous_release(
                config,
                self.updater.UpdateState(
                    "v1.2.3", "v1.2.2", "", "IDLE", ""
                ),
                maintenance_call=maintenance_call,
                readiness_check=lambda generation: True,
                service_control=lambda action, service: events.append(
                    ("service", action)
                ),
                switcher=lambda link, target, releases: (
                    events.append(("switch", target.name)) or current
                ),
                health_check=lambda service: True,
                state_writer=lambda path, state: states.append(state),
                readiness_attempts=1,
                health_attempts=1,
                sleeper=lambda seconds: None,
            )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.2"),
                    ("service", "start"),
                    ("maintenance", False, 51),
                ],
            )
            self.assertEqual(final.current_release, "v1.2.2")
            self.assertEqual(final.previous_release, "v1.2.3")
            self.assertEqual(final.phase, "IDLE")
            self.assertEqual(
                [state.phase for state in states],
                ["ROLLING_BACK", "HEALTH_CHECK", "IDLE"],
            )

    def test_manual_rollback_keeps_maintenance_when_recovery_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            current.mkdir(parents=True)
            previous.mkdir(parents=True)
            maintenance_requests = []

            def maintenance_call(**request):
                maintenance_requests.append(
                    (request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 71, "OK"
                )

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.rollback_to_previous_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.3", "v1.2.2", "", "IDLE", ""
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: None,
                    switcher=lambda link, target, releases: (
                        current if target == previous else previous
                    ),
                    health_check=lambda service: False,
                    state_writer=lambda path, state: None,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(maintenance_requests, [(True, 0)])

    def test_manual_rollback_does_not_switch_after_maintenance_was_released(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            current.mkdir(parents=True)
            previous.mkdir(parents=True)
            events = []

            def maintenance_call(**request):
                events.append(
                    ("maintenance", request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 72, "OK"
                )

            def state_writer(path, state):
                if state.phase == "IDLE":
                    raise self.updater.StateError("disk full")

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.rollback_to_previous_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.3", "v1.2.2", "", "IDLE", ""
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    switcher=lambda link, target, releases: (
                        events.append(("switch", target.name)) or current
                    ),
                    health_check=lambda service: True,
                    state_writer=state_writer,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.2"),
                    ("service", "start"),
                    ("maintenance", False, 72),
                ],
            )

    def test_manual_rollback_recovers_if_switch_reports_wrong_previous_release(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            unrelated = root / "releases" / "v1.2.1"
            for release in (current, previous, unrelated):
                release.mkdir(parents=True)
            events = []

            def maintenance_call(**request):
                events.append(
                    ("maintenance", request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 73, "OK"
                )

            def switch(link, target, releases):
                events.append(("switch", target.name))
                if target == previous:
                    return unrelated
                return previous

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.rollback_to_previous_release(
                    config,
                    self.updater.UpdateState(
                        "v1.2.3", "v1.2.2", "", "IDLE", ""
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: events.append(
                        ("service", action)
                    ),
                    switcher=switch,
                    health_check=lambda service: True,
                    state_writer=lambda path, state: None,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.2"),
                    ("service", "stop"),
                    ("switch", "v1.2.3"),
                    ("service", "start"),
                    ("maintenance", False, 73),
                ],
            )

    def test_cli_exposes_check_stage_apply_rollback_status_and_auto(self):
        parser = self.updater.build_cli_parser()

        for command in (
            "check",
            "stage",
            "apply",
            "rollback",
            "status",
            "auto",
        ):
            with self.subTest(command=command):
                arguments = parser.parse_args(
                    ["--config", "/etc/cleanbot/updater.json", command]
                )
                self.assertEqual(arguments.command, command)
                self.assertEqual(
                    arguments.config,
                    "/etc/cleanbot/updater.json",
                )

    def test_status_payload_reports_current_previous_staged_phase_and_error(self):
        value = self.updater.status_object(
            self.updater.UpdateState(
                "v1.2.3",
                "v1.2.2",
                "v1.2.4",
                "FAILED",
                "health check failed",
            )
        )

        self.assertEqual(
            value,
            {
                "currentRelease": "v1.2.3",
                "previousRelease": "v1.2.2",
                "stagedRelease": "v1.2.4",
                "phase": "FAILED",
                "lastError": "health check failed",
            },
        )

    def test_cli_status_and_check_dispatch_without_network_side_effects(self):
        output = io.StringIO()
        state = self.updater.UpdateState(
            "v1.2.3", "v1.2.2", "", "IDLE", ""
        )
        config = self.updater_config_for(Path("C:/unused"))
        release = self.updater.LatestRelease(
            "v1.2.4",
            {},
        )

        status_code = self.updater.run_cli(
            ["--config", "/etc/cleanbot/updater.json", "status"],
            output=output,
            config_loader=lambda path: config,
            state_loader=lambda path: state,
            release_fetcher=lambda repository: self.fail(
                "status must not access GitHub"
            ),
        )

        self.assertEqual(status_code, 0)
        self.assertEqual(
            json.loads(output.getvalue()),
            self.updater.status_object(state),
        )

        output = io.StringIO()
        check_code = self.updater.run_cli(
            ["--config", "/etc/cleanbot/updater.json", "check"],
            output=output,
            config_loader=lambda path: config,
            state_loader=lambda path: state,
            release_fetcher=lambda repository: release,
        )

        self.assertEqual(check_code, 0)
        self.assertEqual(
            json.loads(output.getvalue()),
            {
                "available": True,
                "currentRelease": "v1.2.3",
                "latestRelease": "v1.2.4",
            },
        )

    def test_cli_auto_stages_but_does_not_apply_when_auto_apply_is_false(self):
        output = io.StringIO()
        root = Path("C:/unused")
        config = self.updater_config_for(root)
        state = self.updater.UpdateState(
            "v1.2.3", "v1.2.2", "", "IDLE", ""
        )
        staged = self.updater.StagedRelease(
            "v1.2.4", root / "staging" / "v1.2.4", None
        )
        calls = []

        code = self.updater.run_cli(
            ["--config", "/etc/cleanbot/updater.json", "auto"],
            output=output,
            config_loader=lambda path: config,
            state_loader=lambda path: state,
            stager=lambda loaded_config, loaded_state: (
                calls.append("stage") or staged
            ),
            applier=lambda *args: calls.append("apply"),
        )

        self.assertEqual(code, 0)
        self.assertEqual(calls, ["stage"])
        self.assertEqual(json.loads(output.getvalue())["result"], "staged")

    def test_recovers_interrupted_download_to_idle_and_removes_only_its_temps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            staging = Path(config.staging_root)
            staging.mkdir(parents=True)
            interrupted = staging / ".v1.2.4.download-abcd"
            interrupted.mkdir()
            unrelated = staging / ".v9.9.9.download-keep"
            unrelated.mkdir()
            states = []

            recovered = self.updater.recover_interrupted_download(
                config,
                self.updater.UpdateState(
                    "v1.2.3",
                    "v1.2.2",
                    "v1.2.4",
                    "DOWNLOADING",
                    "",
                ),
                state_writer=lambda path, state: states.append(state),
            )

            self.assertEqual(recovered.phase, "IDLE")
            self.assertEqual(recovered.staged_release, "")
            self.assertFalse(interrupted.exists())
            self.assertTrue(unrelated.exists())
            self.assertEqual(states, [recovered])

    def test_recovers_interrupted_health_check_to_previous_release(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            current.mkdir(parents=True)
            previous.mkdir(parents=True)
            link = Path(config.current_link)
            if os.name == "nt":
                actual_loader = lambda link, releases: current
            else:
                link.parent.mkdir(parents=True, exist_ok=True)
                link.symlink_to(current)
                actual_loader = None
            events = []
            states = []

            def maintenance_call(**request):
                events.append(
                    ("maintenance", request["enable"], request["generation"])
                )
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 84, "OK"
                )

            recovered = self.updater.recover_interrupted_transition(
                config,
                self.updater.UpdateState(
                    "v1.2.3",
                    "v1.2.2",
                    "v1.2.3",
                    "HEALTH_CHECK",
                    "",
                ),
                maintenance_call=maintenance_call,
                readiness_check=lambda generation: True,
                service_control=lambda action, service: events.append(
                    ("service", action)
                ),
                switcher=lambda link, target, releases: (
                    events.append(("switch", target.name)) or current
                ),
                health_check=lambda service: True,
                state_writer=lambda path, state: states.append(state),
                current_loader=actual_loader,
                readiness_attempts=1,
                health_attempts=1,
                sleeper=lambda seconds: None,
            )

            self.assertEqual(
                events,
                [
                    ("maintenance", True, 0),
                    ("service", "stop"),
                    ("switch", "v1.2.2"),
                    ("service", "start"),
                    ("maintenance", False, 84),
                ],
            )
            self.assertEqual(recovered.phase, "FAILED")
            self.assertEqual(recovered.current_release, "v1.2.2")
            self.assertEqual(recovered.previous_release, "v1.2.3")
            self.assertIn("interrupted", recovered.last_error)
            self.assertEqual(states, [recovered])

    def test_interrupted_transition_keeps_maintenance_if_recovery_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.updater_config_for(root)
            current = root / "releases" / "v1.2.3"
            previous = root / "releases" / "v1.2.2"
            current.mkdir(parents=True)
            previous.mkdir(parents=True)
            requests = []

            def maintenance_call(**request):
                requests.append((request["enable"], request["generation"]))
                return self.updater.MaintenanceServiceResult(
                    True, request["enable"], False, 85, "OK"
                )

            with self.assertRaises(self.updater.UpdaterError):
                self.updater.recover_interrupted_transition(
                    config,
                    self.updater.UpdateState(
                        "v1.2.3",
                        "v1.2.2",
                        "v1.2.3",
                        "HEALTH_CHECK",
                        "",
                    ),
                    maintenance_call=maintenance_call,
                    readiness_check=lambda generation: True,
                    service_control=lambda action, service: None,
                    switcher=lambda link, target, releases: current,
                    health_check=lambda service: False,
                    state_writer=lambda path, state: None,
                    current_loader=lambda link, releases: current,
                    readiness_attempts=1,
                    health_attempts=1,
                    sleeper=lambda seconds: None,
                )

            self.assertEqual(requests, [(True, 0)])


if __name__ == "__main__":
    unittest.main()
