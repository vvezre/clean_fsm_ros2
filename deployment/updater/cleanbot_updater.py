#!/usr/bin/env python3
"""Fail-closed release verification primitives for the cleanbot updater."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Dict, NamedTuple, Optional
from urllib.parse import urlparse
from urllib.request import Request, urlopen


MAX_MANIFEST_BYTES = 64 * 1024
MAX_SIGNATURE_BYTES = 16 * 1024
MAX_RELEASE_RESPONSE_BYTES = 2 * 1024 * 1024
MAX_ARCHIVE_BYTES = 2 * 1024 * 1024 * 1024
MAX_UNPACKED_BYTES = 8 * 1024 * 1024 * 1024
MAX_ARCHIVE_ENTRIES = 100_000
MINIMUM_FREE_RESERVE_BYTES = 128 * 1024 * 1024

ARCHIVE_ASSET = "cleanbot-ros2-humble-arm64.tar.gz"
MANIFEST_ASSET = "manifest.json"
SIGNATURE_ASSET = "manifest.json.sig"
REQUIRED_RELEASE_ASSETS = (
    ARCHIVE_ASSET,
    MANIFEST_ASSET,
    SIGNATURE_ASSET,
)
_ASSET_SIZE_LIMITS = {
    ARCHIVE_ASSET: MAX_ARCHIVE_BYTES,
    MANIFEST_ASSET: MAX_MANIFEST_BYTES,
    SIGNATURE_ASSET: MAX_SIGNATURE_BYTES,
}

_RELEASE_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_SEMVER_PATTERN = re.compile(
    r"^v?(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$"
)
_CREATED_AT_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$"
)
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class UpdaterError(RuntimeError):
    """Base updater failure."""


class ManifestError(UpdaterError):
    """Manifest bytes or fields are invalid."""


class ReleaseError(UpdaterError):
    """GitHub release metadata is invalid."""


class VerificationError(UpdaterError):
    """A signed asset or extracted archive failed verification."""


class ConfigError(UpdaterError):
    """Updater configuration is invalid."""


class StateError(UpdaterError):
    """Updater state is invalid or cannot be committed."""


class ArchiveSpec(NamedTuple):
    name: str
    sha256: str
    size: int
    unpacked_size: int


class Manifest(NamedTuple):
    schema_version: int
    release: str
    created_at: str
    architecture: str
    operating_system: str
    ros_distribution: str
    archive: ArchiveSpec


class ReleaseAsset(NamedTuple):
    name: str
    download_url: str
    size: int


class LatestRelease(NamedTuple):
    tag: str
    assets: Dict[str, ReleaseAsset]


class UpdaterConfig(NamedTuple):
    repository: str
    staging_root: str
    releases_root: str
    current_link: str
    state_path: str
    public_key: str
    service_name: str
    requester: str
    auto_download: bool
    auto_apply: bool


class UpdateState(NamedTuple):
    current_release: str
    previous_release: str
    staged_release: str
    phase: str
    last_error: str


class StagedRelease(NamedTuple):
    release: str
    path: Path
    manifest: Manifest


class MaintenanceServiceResult(NamedTuple):
    accepted: bool
    gate_active: bool
    ready: bool
    generation: int
    code: str


_CONFIG_FIELDS = (
    "schemaVersion",
    "repository",
    "stagingRoot",
    "releasesRoot",
    "currentLink",
    "statePath",
    "publicKey",
    "serviceName",
    "requester",
    "autoDownload",
    "autoApply",
)
_STATE_FIELDS = (
    "schemaVersion",
    "currentRelease",
    "previousRelease",
    "stagedRelease",
    "phase",
    "lastError",
)
_STATE_PHASES = {
    "IDLE",
    "DOWNLOADING",
    "STAGED",
    "APPLYING",
    "HEALTH_CHECK",
    "ROLLING_BACK",
    "FAILED",
}
_REPOSITORY_PATTERN = re.compile(
    r"^[A-Za-z0-9_.-]{1,100}/[A-Za-z0-9_.-]{1,100}$"
)
_SERVICE_PATTERN = re.compile(r"^[A-Za-z0-9_.@-]{1,128}\.service$")
_REQUESTER_PATTERN = re.compile(r"^[A-Za-z0-9_.-]{1,128}$")
_REASON_PATTERN = re.compile(r"^[A-Za-z0-9_.-]{1,128}$")
_ROS_CLI_PREFIX = (
    "/usr/sbin/runuser",
    "--user",
    "cleanbot",
    "--",
    "/usr/local/libexec/cleanbot-ros-cli",
)


def canonical_json_bytes(value: Any) -> bytes:
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exception:
        raise ManifestError(f"manifest cannot be canonicalized: {exception}") from exception
    return encoded.encode("utf-8") + b"\n"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ManifestError(f"duplicate manifest field: {key}")
        result[key] = value
    return result


def _exact_fields(value: Any, fields, context: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ManifestError(f"{context} must be an object")
    actual = set(value)
    expected = set(fields)
    if actual != expected:
        raise ManifestError(
            f"{context} fields do not match schema: "
            f"missing={sorted(expected - actual)} "
            f"unknown={sorted(actual - expected)}"
        )
    return value


def _bounded_integer(
    value: Any,
    *,
    minimum: int,
    maximum: int,
    context: str,
) -> int:
    if type(value) is not int or value < minimum or value > maximum:
        raise ManifestError(f"{context} is outside the allowed range")
    return value


def parse_manifest(
    data: bytes,
    expected_architecture: str,
    expected_os: str,
    expected_ros: str,
) -> Manifest:
    if not isinstance(data, bytes) or not data or len(data) > MAX_MANIFEST_BYTES:
        raise ManifestError("manifest byte length is invalid")
    try:
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ManifestError(f"invalid JSON constant: {token}")
            ),
        )
    except ManifestError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exception:
        raise ManifestError(f"manifest JSON is invalid: {exception}") from exception

    if canonical_json_bytes(value) != data:
        raise ManifestError("manifest is not canonical JSON")

    root = _exact_fields(
        value,
        ("schemaVersion", "release", "createdAt", "target", "archive"),
        "manifest",
    )
    if type(root["schemaVersion"]) is not int or root["schemaVersion"] != 1:
        raise ManifestError("manifest schemaVersion is unsupported")

    release = root["release"]
    created_at = root["createdAt"]
    if not isinstance(release, str) or not _RELEASE_PATTERN.fullmatch(release):
        raise ManifestError("manifest release is unsafe")
    if (
        not isinstance(created_at, str)
        or not _CREATED_AT_PATTERN.fullmatch(created_at)
    ):
        raise ManifestError("manifest createdAt is invalid")

    target = _exact_fields(
        root["target"], ("architecture", "os", "ros"), "manifest target"
    )
    for field in ("architecture", "os", "ros"):
        if not isinstance(target[field], str):
            raise ManifestError(f"manifest target {field} must be a string")
    if target["architecture"] != expected_architecture:
        raise ManifestError("release architecture does not match this device")
    if target["os"] != expected_os:
        raise ManifestError("release operating system does not match this device")
    if target["ros"] != expected_ros:
        raise ManifestError("release ROS distribution does not match this device")

    archive = _exact_fields(
        root["archive"],
        ("name", "sha256", "size", "unpackedSize"),
        "manifest archive",
    )
    if archive["name"] != ARCHIVE_ASSET:
        raise ManifestError("manifest archive name is invalid")
    if (
        not isinstance(archive["sha256"], str)
        or not _SHA256_PATTERN.fullmatch(archive["sha256"])
    ):
        raise ManifestError("manifest archive sha256 is invalid")
    archive_size = _bounded_integer(
        archive["size"],
        minimum=1,
        maximum=MAX_ARCHIVE_BYTES,
        context="manifest archive size",
    )
    unpacked_size = _bounded_integer(
        archive["unpackedSize"],
        minimum=1,
        maximum=MAX_UNPACKED_BYTES,
        context="manifest archive unpackedSize",
    )
    return Manifest(
        schema_version=1,
        release=release,
        created_at=created_at,
        architecture=target["architecture"],
        operating_system=target["os"],
        ros_distribution=target["ros"],
        archive=ArchiveSpec(
            name=archive["name"],
            sha256=archive["sha256"],
            size=archive_size,
            unpacked_size=unpacked_size,
        ),
    )


def _strict_json_object(data: bytes, context: str, maximum_bytes: int):
    if not isinstance(data, bytes) or not data or len(data) > maximum_bytes:
        raise ConfigError(f"{context} byte length is invalid")
    try:
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ConfigError(f"invalid JSON constant: {token}")
            ),
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ManifestError) as exception:
        raise ConfigError(f"{context} JSON is invalid: {exception}") from exception
    if not isinstance(value, dict):
        raise ConfigError(f"{context} must be an object")
    return value


def _absolute_posix_path(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or "\\" in value:
        raise ConfigError(f"{context} must be an absolute POSIX path")
    path = PurePosixPath(value)
    if any(part in ("", ".", "..") for part in path.parts[1:]):
        raise ConfigError(f"{context} contains an unsafe component")
    return path.as_posix()


def parse_config(data: bytes) -> UpdaterConfig:
    value = _strict_json_object(data, "updater config", MAX_MANIFEST_BYTES)
    if set(value) != set(_CONFIG_FIELDS):
        raise ConfigError("updater config fields do not match the schema")
    if type(value["schemaVersion"]) is not int or value["schemaVersion"] != 1:
        raise ConfigError("updater config schemaVersion is unsupported")
    repository = value["repository"]
    service_name = value["serviceName"]
    requester = value["requester"]
    if (
        not isinstance(repository, str)
        or not _REPOSITORY_PATTERN.fullmatch(repository)
    ):
        raise ConfigError("updater repository is invalid")
    if (
        not isinstance(service_name, str)
        or not _SERVICE_PATTERN.fullmatch(service_name)
    ):
        raise ConfigError("updater serviceName is invalid")
    if (
        not isinstance(requester, str)
        or not _REQUESTER_PATTERN.fullmatch(requester)
    ):
        raise ConfigError("updater requester is invalid")
    if type(value["autoDownload"]) is not bool or type(value["autoApply"]) is not bool:
        raise ConfigError("updater automation flags must be booleans")
    return UpdaterConfig(
        repository=repository,
        staging_root=_absolute_posix_path(value["stagingRoot"], "stagingRoot"),
        releases_root=_absolute_posix_path(value["releasesRoot"], "releasesRoot"),
        current_link=_absolute_posix_path(value["currentLink"], "currentLink"),
        state_path=_absolute_posix_path(value["statePath"], "statePath"),
        public_key=_absolute_posix_path(value["publicKey"], "publicKey"),
        service_name=service_name,
        requester=requester,
        auto_download=value["autoDownload"],
        auto_apply=value["autoApply"],
    )


def select_release_assets(payload: Any) -> Dict[str, ReleaseAsset]:
    if not isinstance(payload, dict):
        raise ReleaseError("release response must be an object")
    if payload.get("draft") is not False or payload.get("prerelease") is not False:
        raise ReleaseError("release must be stable and published")
    tag_name = payload.get("tag_name")
    if not isinstance(tag_name, str) or not _RELEASE_PATTERN.fullmatch(tag_name):
        raise ReleaseError("release tag is invalid")
    assets = payload.get("assets")
    if not isinstance(assets, list):
        raise ReleaseError("release assets must be an array")

    selected: Dict[str, ReleaseAsset] = {}
    required = set(REQUIRED_RELEASE_ASSETS)
    for item in assets:
        if not isinstance(item, dict):
            raise ReleaseError("release asset must be an object")
        name = item.get("name")
        if name not in required:
            continue
        if name in selected:
            raise ReleaseError(f"release asset is duplicated: {name}")
        url = item.get("browser_download_url")
        size = item.get("size")
        if (
            not isinstance(url, str)
            or urlparse(url).scheme.lower() != "https"
            or not urlparse(url).netloc
        ):
            raise ReleaseError(f"release asset URL is invalid: {name}")
        if (
            type(size) is not int
            or size <= 0
            or size > _ASSET_SIZE_LIMITS[name]
        ):
            raise ReleaseError(f"release asset size is invalid: {name}")
        selected[name] = ReleaseAsset(name, url, size)

    missing = required - set(selected)
    if missing:
        raise ReleaseError(f"release assets are missing: {sorted(missing)}")
    return selected


def _validate_repository(repository: Any) -> str:
    if (
        not isinstance(repository, str)
        or not _REPOSITORY_PATTERN.fullmatch(repository)
    ):
        raise ReleaseError("GitHub repository is invalid")
    return repository


def _read_bounded_response(response, maximum_bytes: int, context: str) -> bytes:
    try:
        data = response.read(maximum_bytes + 1)
    except Exception as exception:
        raise ReleaseError(f"{context} cannot be read: {exception}") from exception
    if not isinstance(data, bytes) or len(data) > maximum_bytes:
        raise ReleaseError(f"{context} exceeds the allowed size")
    return data


def fetch_latest_release(
    repository: str,
    *,
    opener: Callable = urlopen,
    timeout_seconds: int = 30,
) -> LatestRelease:
    repository = _validate_repository(repository)
    if type(timeout_seconds) is not int or timeout_seconds <= 0:
        raise ReleaseError("GitHub request timeout is invalid")
    request = Request(
        f"https://api.github.com/repos/{repository}/releases/latest",
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": "cleanbot-updater/1",
            "X-GitHub-Api-Version": "2022-11-28",
        },
        method="GET",
    )
    try:
        with opener(request, timeout=timeout_seconds) as response:
            data = _read_bounded_response(
                response,
                MAX_RELEASE_RESPONSE_BYTES,
                "GitHub release response",
            )
    except ReleaseError:
        raise
    except Exception as exception:
        raise ReleaseError(
            f"GitHub latest release request failed: {exception}"
        ) from exception
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exception:
        raise ReleaseError(
            f"GitHub release response JSON is invalid: {exception}"
        ) from exception
    assets = select_release_assets(payload)
    return LatestRelease(payload["tag_name"], assets)


def _validate_release_asset(asset: Any) -> ReleaseAsset:
    if not isinstance(asset, ReleaseAsset):
        raise ReleaseError("release asset is invalid")
    if asset.name not in REQUIRED_RELEASE_ASSETS:
        raise ReleaseError("release asset name is invalid")
    parsed_url = urlparse(asset.download_url)
    if parsed_url.scheme.lower() != "https" or not parsed_url.netloc:
        raise ReleaseError("release asset URL is invalid")
    if (
        type(asset.size) is not int
        or asset.size <= 0
        or asset.size > _ASSET_SIZE_LIMITS[asset.name]
    ):
        raise ReleaseError("release asset size is invalid")
    return asset


def download_release_asset(
    asset: ReleaseAsset,
    destination: Path,
    *,
    opener: Callable = urlopen,
    timeout_seconds: int = 120,
) -> Path:
    asset = _validate_release_asset(asset)
    destination = Path(destination)
    if destination.name != asset.name:
        raise ReleaseError("release asset destination name does not match")
    if type(timeout_seconds) is not int or timeout_seconds <= 0:
        raise ReleaseError("release asset request timeout is invalid")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        raise ReleaseError("release asset destination already exists")

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{asset.name}.download-",
        dir=destination.parent,
    )
    temporary = Path(temporary_name)
    request = Request(
        asset.download_url,
        headers={
            "Accept": "application/octet-stream",
            "User-Agent": "cleanbot-updater/1",
        },
        method="GET",
    )
    try:
        with os.fdopen(descriptor, "wb") as output:
            if hasattr(os, "fchmod"):
                os.fchmod(output.fileno(), 0o600)
            try:
                with opener(request, timeout=timeout_seconds) as response:
                    total = 0
                    while True:
                        chunk = response.read(
                            min(1024 * 1024, asset.size - total + 1)
                        )
                        if not chunk:
                            break
                        if not isinstance(chunk, bytes):
                            raise ReleaseError(
                                "release asset response returned non-bytes data"
                            )
                        total += len(chunk)
                        if total > asset.size:
                            raise ReleaseError(
                                "release asset exceeds its declared size"
                            )
                        output.write(chunk)
            except ReleaseError:
                raise
            except Exception as exception:
                raise ReleaseError(
                    f"release asset download failed: {exception}"
                ) from exception
            if total != asset.size:
                raise ReleaseError(
                    "release asset size does not match GitHub metadata"
                )
            output.flush()
            os.fsync(output.fileno())
        if destination.exists() or destination.is_symlink():
            raise ReleaseError("release asset destination appeared during download")
        os.replace(temporary, destination)
        if os.name != "nt":
            directory_descriptor = os.open(
                destination.parent,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
    except Exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise
    return destination


def _regular_file(path: Path, context: str) -> os.stat_result:
    try:
        information = path.lstat()
    except OSError as exception:
        raise VerificationError(f"{context} cannot be inspected: {exception}") from exception
    if not stat.S_ISREG(information.st_mode) or path.is_symlink():
        raise VerificationError(f"{context} must be a regular non-symlink file")
    return information


def ensure_free_space(path: Path, required_bytes: int, context: str) -> None:
    if type(required_bytes) is not int or required_bytes < 0:
        raise VerificationError(f"{context} required space is invalid")
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    try:
        free_bytes = shutil.disk_usage(path).free
    except OSError as exception:
        raise VerificationError(
            f"{context} free space cannot be inspected: {exception}"
        ) from exception
    if free_bytes < required_bytes + MINIMUM_FREE_RESERVE_BYTES:
        raise VerificationError(
            f"{context} has insufficient free space"
        )


def sha256_file(path: Path) -> str:
    _regular_file(path, "archive")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source:
            while True:
                block = source.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
    except OSError as exception:
        raise VerificationError(f"archive cannot be read: {exception}") from exception
    return digest.hexdigest()


def verify_archive(path: Path, expected: ArchiveSpec) -> None:
    information = _regular_file(path, "archive")
    if information.st_size != expected.size:
        raise VerificationError("archive size does not match the signed manifest")
    if sha256_file(path) != expected.sha256:
        raise VerificationError("archive sha256 does not match the signed manifest")


def verify_manifest_signature(
    manifest_path: Path,
    signature_path: Path,
    public_key_path: Path,
    *,
    openssl: str = "openssl",
) -> None:
    manifest_info = _regular_file(manifest_path, "manifest")
    signature_info = _regular_file(signature_path, "manifest signature")
    _regular_file(public_key_path, "update public key")
    if manifest_info.st_size <= 0 or manifest_info.st_size > MAX_MANIFEST_BYTES:
        raise VerificationError("manifest size is invalid")
    if signature_info.st_size <= 0 or signature_info.st_size > MAX_SIGNATURE_BYTES:
        raise VerificationError("manifest signature size is invalid")
    try:
        completed = subprocess.run(
            [
                openssl,
                "pkeyutl",
                "-verify",
                "-rawin",
                "-pubin",
                "-inkey",
                str(public_key_path),
                "-sigfile",
                str(signature_path),
                "-in",
                str(manifest_path),
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError) as exception:
        raise VerificationError(f"signature verification could not run: {exception}") from exception
    if completed.returncode != 0:
        detail = completed.stderr.decode("utf-8", errors="replace").strip()
        raise VerificationError(
            "manifest signature verification failed"
            + (f": {detail}" if detail else "")
        )


def _safe_member_name(name: str) -> str:
    if not name or "\x00" in name or "\\" in name:
        raise VerificationError("archive member path is invalid")
    path = PurePosixPath(name)
    if path.is_absolute():
        raise VerificationError("archive contains an absolute path")
    if any(part in ("", ".", "..") for part in path.parts):
        raise VerificationError("archive member escapes the release directory")
    normalized = path.as_posix().rstrip("/")
    if not normalized:
        raise VerificationError("archive member path is empty")
    return normalized


def secure_extract(
    archive_path: Path,
    destination: Path,
    *,
    maximum_unpacked_bytes: int,
) -> None:
    _regular_file(archive_path, "archive")
    if not hasattr(tarfile, "data_filter"):
        raise VerificationError(
            "this Python runtime lacks the secure tar data_filter"
        )
    if (
        type(maximum_unpacked_bytes) is not int
        or maximum_unpacked_bytes <= 0
        or maximum_unpacked_bytes > MAX_UNPACKED_BYTES
    ):
        raise VerificationError("maximum unpacked size is invalid")
    if destination.exists() or destination.is_symlink():
        raise VerificationError("release destination already exists")

    destination_parent = destination.parent
    destination_parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(
            prefix=f".{destination.name}.extract-",
            dir=str(destination_parent),
        )
    )
    try:
        try:
            archive = tarfile.open(archive_path, mode="r:gz")
        except (OSError, tarfile.TarError) as exception:
            raise VerificationError(f"archive cannot be opened: {exception}") from exception
        with archive:
            members = archive.getmembers()
            if not members or len(members) > MAX_ARCHIVE_ENTRIES:
                raise VerificationError("archive entry count is invalid")
            seen = set()
            unpacked = 0
            for member in members:
                normalized = _safe_member_name(member.name)
                if normalized in seen:
                    raise VerificationError(
                        f"archive member is duplicated: {normalized}"
                    )
                seen.add(normalized)
                if getattr(member, "sparse", None):
                    raise VerificationError("sparse archive members are forbidden")
                if not (member.isfile() or member.isdir()):
                    raise VerificationError(
                        f"archive member type is forbidden: {normalized}"
                    )
                if member.size < 0:
                    raise VerificationError("archive member size is invalid")
                if member.isfile():
                    unpacked += member.size
                    if unpacked > maximum_unpacked_bytes:
                        raise VerificationError(
                            "archive exceeds the signed unpacked size bound"
                        )
            try:
                archive.extractall(
                    path=temporary,
                    members=members,
                    filter="data",
                )
            except (OSError, tarfile.TarError) as exception:
                raise VerificationError(f"archive extraction failed: {exception}") from exception

        for path in temporary.rglob("*"):
            information = path.lstat()
            if path.is_symlink() or not (
                stat.S_ISREG(information.st_mode)
                or stat.S_ISDIR(information.st_mode)
            ):
                raise VerificationError("extracted tree contains an unsafe file")
            try:
                path.resolve().relative_to(temporary.resolve())
            except ValueError as exception:
                raise VerificationError("extracted path escaped destination") from exception

        if not (temporary / "VERSION").is_file():
            raise VerificationError("release VERSION file is missing")
        if not (temporary / "install" / "setup.bash").is_file():
            raise VerificationError("release install/setup.bash is missing")
        os.replace(temporary, destination)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def stage_release(
    *,
    manifest_path: Path,
    signature_path: Path,
    archive_path: Path,
    public_key_path: Path,
    staging_root: Path,
    expected_architecture: str,
    expected_os: str,
    expected_ros: str,
    signature_verifier: Callable[
        [Path, Path, Path], None
    ] = verify_manifest_signature,
) -> StagedRelease:
    _regular_file(manifest_path, "manifest")
    _regular_file(signature_path, "manifest signature")
    _regular_file(archive_path, "archive")
    _regular_file(public_key_path, "update public key")
    signature_verifier(manifest_path, signature_path, public_key_path)
    try:
        manifest_bytes = manifest_path.read_bytes()
    except OSError as exception:
        raise VerificationError(f"manifest cannot be read: {exception}") from exception
    parsed = parse_manifest(
        manifest_bytes,
        expected_architecture,
        expected_os,
        expected_ros,
    )
    verify_archive(archive_path, parsed.archive)

    staging_root.mkdir(parents=True, exist_ok=True)
    ensure_free_space(
        staging_root,
        parsed.archive.size + parsed.archive.unpacked_size,
        "release staging",
    )
    final_path = staging_root / parsed.release
    if final_path.exists() or final_path.is_symlink():
        raise VerificationError("release is already staged")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{parsed.release}.stage-", dir=staging_root)
    )
    try:
        payload = temporary / "payload"
        secure_extract(
            archive_path,
            payload,
            maximum_unpacked_bytes=parsed.archive.unpacked_size,
        )
        try:
            version = (payload / "VERSION").read_text(
                encoding="utf-8"
            ).strip()
        except (OSError, UnicodeError) as exception:
            raise VerificationError(f"release VERSION cannot be read: {exception}") from exception
        if version != parsed.release:
            raise VerificationError("release VERSION does not match manifest")
        shutil.copyfile(manifest_path, temporary / MANIFEST_ASSET)
        shutil.copyfile(signature_path, temporary / SIGNATURE_ASSET)
        shutil.copyfile(archive_path, temporary / ARCHIVE_ASSET)
        os.replace(temporary, final_path)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return StagedRelease(parsed.release, final_path, parsed)


def load_staged_release(
    staged_path: Path,
    expected_release: str,
    public_key_path: Path,
    *,
    signature_verifier: Callable[
        [Path, Path, Path], None
    ] = verify_manifest_signature,
) -> StagedRelease:
    if (
        not isinstance(expected_release, str)
        or not _RELEASE_PATTERN.fullmatch(expected_release)
    ):
        raise VerificationError("expected staged release is invalid")
    staged_path = Path(staged_path)
    if staged_path.name != expected_release:
        raise VerificationError("staged directory does not match expected release")
    _validate_regular_tree(staged_path, "staged release")
    manifest_path = staged_path / MANIFEST_ASSET
    signature_path = staged_path / SIGNATURE_ASSET
    archive_path = staged_path / ARCHIVE_ASSET
    _regular_file(manifest_path, "staged manifest")
    _regular_file(signature_path, "staged manifest signature")
    _regular_file(archive_path, "staged archive")
    signature_verifier(manifest_path, signature_path, Path(public_key_path))
    try:
        manifest_bytes = manifest_path.read_bytes()
    except OSError as exception:
        raise VerificationError(
            f"staged manifest cannot be read: {exception}"
        ) from exception
    parsed = parse_manifest(
        manifest_bytes,
        "arm64",
        "ubuntu-22.04",
        "humble",
    )
    if parsed.release != expected_release:
        raise VerificationError(
            "staged manifest release does not match expected release"
        )
    verify_archive(archive_path, parsed.archive)
    payload = staged_path / "payload"
    if payload.exists() or payload.is_symlink():
        _validate_regular_tree(payload, "staged payload")
        shutil.rmtree(payload)
    secure_extract(
        archive_path,
        payload,
        maximum_unpacked_bytes=parsed.archive.unpacked_size,
    )
    _validate_regular_tree(payload, "staged payload")
    _fsync_regular_tree(payload)
    try:
        version = (payload / "VERSION").read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError) as exception:
        raise VerificationError(
            f"staged payload VERSION cannot be read: {exception}"
        ) from exception
    if version != expected_release:
        raise VerificationError(
            "staged payload VERSION does not match expected release"
        )
    _regular_file(
        payload / "install" / "setup.bash",
        "staged payload setup",
    )
    return StagedRelease(expected_release, staged_path, parsed)


def update_state_object(state: UpdateState) -> Dict[str, Any]:
    return {
        "schemaVersion": 1,
        "currentRelease": state.current_release,
        "previousRelease": state.previous_release,
        "stagedRelease": state.staged_release,
        "phase": state.phase,
        "lastError": state.last_error,
    }


def _validate_release_or_empty(value: Any, context: str) -> str:
    if value == "":
        return ""
    if not isinstance(value, str) or not _RELEASE_PATTERN.fullmatch(value):
        raise StateError(f"{context} is invalid")
    return value


def _parse_update_state_value(value: Any) -> UpdateState:
    if not isinstance(value, dict) or set(value) != set(_STATE_FIELDS):
        raise StateError("updater state fields do not match the schema")
    if type(value["schemaVersion"]) is not int or value["schemaVersion"] != 1:
        raise StateError("updater state schemaVersion is unsupported")
    phase = value["phase"]
    last_error = value["lastError"]
    if not isinstance(phase, str) or phase not in _STATE_PHASES:
        raise StateError("updater state phase is invalid")
    if not isinstance(last_error, str) or len(last_error.encode("utf-8")) > 4096:
        raise StateError("updater state lastError is invalid")
    return UpdateState(
        current_release=_validate_release_or_empty(
            value["currentRelease"], "currentRelease"
        ),
        previous_release=_validate_release_or_empty(
            value["previousRelease"], "previousRelease"
        ),
        staged_release=_validate_release_or_empty(
            value["stagedRelease"], "stagedRelease"
        ),
        phase=phase,
        last_error=last_error,
    )


def load_update_state(path: Path) -> UpdateState:
    information = _regular_file(path, "updater state")
    if information.st_size <= 0 or information.st_size > MAX_MANIFEST_BYTES:
        raise StateError("updater state size is invalid")
    try:
        data = path.read_bytes()
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                StateError(f"invalid JSON constant: {token}")
            ),
        )
    except StateError:
        raise
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ManifestError) as exception:
        raise StateError(f"updater state is invalid: {exception}") from exception
    if canonical_json_bytes(value) != data:
        raise StateError("updater state is not canonical JSON")
    return _parse_update_state_value(value)


def write_update_state(path: Path, state: UpdateState) -> None:
    value = update_state_object(state)
    _parse_update_state_value(value)
    data = canonical_json_bytes(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = -1
    temporary_path = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.tmp-",
            dir=path.parent,
        )
        temporary_path = Path(temporary_name)
        if hasattr(os, "fchmod"):
            os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=True) as output:
            descriptor = -1
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
        temporary_path = None
        if os.name != "nt":
            directory_fd = os.open(
                path.parent,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    except (OSError, ValueError) as exception:
        raise StateError(f"updater state could not be committed: {exception}") from exception
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except OSError:
                pass


def _contained_directory(path: Path, root: Path, context: str) -> Path:
    try:
        resolved_root = root.resolve(strict=True)
        resolved = path.resolve(strict=True)
        resolved.relative_to(resolved_root)
    except (OSError, ValueError) as exception:
        raise VerificationError(f"{context} is outside the releases root") from exception
    if not resolved.is_dir() or resolved.is_symlink():
        raise VerificationError(f"{context} must be a real release directory")
    return resolved


def switch_current_release(
    current_link: Path,
    target_release: Path,
    releases_root: Path,
) -> Optional[Path]:
    target = _contained_directory(
        target_release, releases_root, "target release"
    )
    if not (current_link.exists() or current_link.is_symlink()):
        raise VerificationError("current release symlink is missing")
    if not current_link.is_symlink():
        raise VerificationError("current release path is not a symlink")
    previous = _contained_directory(
        current_link, releases_root, "current release"
    )
    current_link.parent.mkdir(parents=True, exist_ok=True)
    temporary = current_link.with_name(
        f".{current_link.name}.new-{os.getpid()}"
    )
    if temporary.exists() or temporary.is_symlink():
        raise VerificationError("temporary current symlink already exists")
    try:
        os.symlink(target, temporary, target_is_directory=True)
        os.replace(temporary, current_link)
        if os.name != "nt":
            descriptor = os.open(
                current_link.parent,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    except OSError as exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise VerificationError(f"current release switch failed: {exception}") from exception
    return previous


def _output_boolean(output: str, name: str) -> Optional[bool]:
    match = re.search(
        rf"\b{re.escape(name)}\s*[:=]\s*(true|false)",
        output,
        flags=re.IGNORECASE,
    )
    if match is None:
        return None
    return match.group(1).lower() == "true"


def _output_generation(output: str) -> Optional[int]:
    matches = re.findall(r"\bgeneration\s*[:=]\s*(\d+)", output)
    if not matches:
        return None
    return int(matches[-1])


def parse_maintenance_service_output(output: str) -> MaintenanceServiceResult:
    if not isinstance(output, str) or len(output) > 64 * 1024:
        raise VerificationError("maintenance service output is invalid")
    accepted = _output_boolean(output, "accepted")
    gate_active = _output_boolean(output, "gate_active")
    ready = _output_boolean(output, "ready")
    generation = _output_generation(output)
    if accepted is not True or generation is None or generation <= 0:
        raise VerificationError("maintenance service did not accept a valid token")
    code_match = re.search(
        r"\bcode\s*[:=]\s*['\"]?([A-Za-z0-9_.-]+)",
        output,
    )
    return MaintenanceServiceResult(
        accepted=True,
        gate_active=gate_active is True,
        ready=ready is True,
        generation=generation,
        code=code_match.group(1) if code_match else "",
    )


def parse_maintenance_ready(output: str, *, generation: int) -> bool:
    if (
        not isinstance(output, str)
        or generation <= 0
        or len(output) > 64 * 1024
    ):
        return False
    return (
        _output_boolean(output, "gate_active") is True
        and _output_boolean(output, "ready") is True
        and _output_generation(output) == generation
    )


def _run_checked_command(
    argv,
    *,
    runner: Callable,
    timeout_seconds: int,
    context: str,
):
    if type(timeout_seconds) is not int or timeout_seconds <= 0:
        raise VerificationError(f"{context} timeout is invalid")
    try:
        result = runner(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
            shell=False,
        )
    except Exception as exception:
        raise VerificationError(f"{context} failed: {exception}") from exception
    if result is None or getattr(result, "returncode", None) != 0:
        stderr = getattr(result, "stderr", "") if result is not None else ""
        raise VerificationError(
            f"{context} failed with non-zero status: {stderr}"
        )
    stdout = getattr(result, "stdout", None)
    if not isinstance(stdout, str):
        raise VerificationError(f"{context} returned invalid output")
    return stdout


def _ros_cli_command(*arguments: str):
    return [*_ROS_CLI_PREFIX, *arguments]


def call_maintenance_service(
    *,
    enable: bool,
    generation: int,
    requester: str,
    reason: str,
    runner: Callable = subprocess.run,
    timeout_seconds: int = 15,
) -> MaintenanceServiceResult:
    if type(enable) is not bool:
        raise VerificationError("maintenance enable flag is invalid")
    if (
        type(generation) is not int
        or generation < 0
        or generation > 0xFFFFFFFFFFFFFFFF
        or (enable and generation != 0)
        or (not enable and generation == 0)
    ):
        raise VerificationError("maintenance generation is invalid")
    if (
        not isinstance(requester, str)
        or not _REQUESTER_PATTERN.fullmatch(requester)
    ):
        raise VerificationError("maintenance requester is invalid")
    if not isinstance(reason, str) or not _REASON_PATTERN.fullmatch(reason):
        raise VerificationError("maintenance reason is invalid")
    request = json.dumps(
        {
            "enable": enable,
            "generation": generation,
            "requester": requester,
            "reason": reason,
        },
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
    )
    output = _run_checked_command(
        _ros_cli_command(
            "service",
            "call",
            "/system/set_maintenance",
            "cleanbot_interfaces/srv/SetMaintenanceMode",
            request,
        ),
        runner=runner,
        timeout_seconds=timeout_seconds,
        context="maintenance service call",
    )
    result = parse_maintenance_service_output(output)
    if not enable and result.generation != generation:
        raise VerificationError("maintenance release returned the wrong generation")
    return result


def read_maintenance_ready(
    *,
    generation: int,
    runner: Callable = subprocess.run,
    timeout_seconds: int = 10,
) -> bool:
    if type(generation) is not int or generation <= 0:
        raise VerificationError("maintenance generation is invalid")
    output = _run_checked_command(
        _ros_cli_command(
            "topic",
            "echo",
            "--once",
            "/system/maintenance_state",
            "cleanbot_interfaces/msg/MaintenanceState",
        ),
        runner=runner,
        timeout_seconds=timeout_seconds,
        context="maintenance state read",
    )
    return parse_maintenance_ready(output, generation=generation)


def _validate_regular_tree(root: Path, context: str) -> None:
    if not root.is_dir() or root.is_symlink():
        raise VerificationError(f"{context} must be a real directory")
    for directory, directory_names, file_names in os.walk(
        root, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        for name in directory_names:
            path = directory_path / name
            if path.is_symlink() or not path.is_dir():
                raise VerificationError(f"{context} contains an unsafe directory")
        for name in file_names:
            path = directory_path / name
            information = path.lstat()
            if path.is_symlink() or not stat.S_ISREG(information.st_mode):
                raise VerificationError(f"{context} contains an unsafe file")


def _fsync_regular_tree(root: Path) -> None:
    if os.name == "nt":
        return
    directories = []
    for directory, _, file_names in os.walk(root, topdown=False):
        directory_path = Path(directory)
        directories.append(directory_path)
        for name in file_names:
            descriptor = os.open(directory_path / name, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    for directory in directories:
        descriptor = os.open(
            directory,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
        )
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def install_staged_payload(
    staged_path: Path,
    release: str,
    releases_root: Path,
) -> Path:
    if not isinstance(release, str) or not _RELEASE_PATTERN.fullmatch(release):
        raise VerificationError("staged release name is unsafe")
    staged_path = Path(staged_path)
    payload = staged_path / "payload"
    _validate_regular_tree(staged_path, "staged release")
    _validate_regular_tree(payload, "staged payload")
    try:
        version = (payload / "VERSION").read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError) as exception:
        raise VerificationError(
            f"staged payload VERSION cannot be read: {exception}"
        ) from exception
    if version != release:
        raise VerificationError("staged payload VERSION does not match release")
    setup = payload / "install" / "setup.bash"
    _regular_file(setup, "staged payload setup")

    releases_root = Path(releases_root)
    releases_root.mkdir(parents=True, exist_ok=True)
    payload_size = sum(
        path.lstat().st_size
        for path in payload.rglob("*")
        if path.is_file() and not path.is_symlink()
    )
    ensure_free_space(
        releases_root,
        payload_size,
        "release installation",
    )
    final_path = releases_root / release
    if final_path.exists() or final_path.is_symlink():
        raise VerificationError("release is already installed")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{release}.install-", dir=releases_root)
    )
    try:
        shutil.copytree(payload, temporary, dirs_exist_ok=True, symlinks=False)
        _validate_regular_tree(temporary, "installed release")
        temporary.chmod(0o755)
        _fsync_regular_tree(temporary)
        os.replace(temporary, final_path)
        if os.name != "nt":
            descriptor = os.open(
                releases_root,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return final_path


def cleanup_successful_update(
    staged_path: Path,
    staging_root: Path,
    releases_root: Path,
    keep_releases,
) -> None:
    keep = set(keep_releases)
    if not keep or any(
        not isinstance(name, str) or not _RELEASE_PATTERN.fullmatch(name)
        for name in keep
    ):
        return
    try:
        resolved_staging = Path(staging_root).resolve(strict=True)
        resolved_staged = Path(staged_path).resolve(strict=True)
        resolved_staged.relative_to(resolved_staging)
        if (
            resolved_staged.parent == resolved_staging
            and resolved_staged.name not in keep
        ):
            return
        if (
            resolved_staged.parent == resolved_staging
            and resolved_staged.name in keep
            and resolved_staged.is_dir()
            and not resolved_staged.is_symlink()
        ):
            shutil.rmtree(resolved_staged)
    except (OSError, ValueError):
        pass

    try:
        root = Path(releases_root).resolve(strict=True)
        if not root.is_dir() or root.is_symlink():
            return
        for candidate in root.iterdir():
            if candidate.name in keep:
                continue
            if (
                not _RELEASE_PATTERN.fullmatch(candidate.name)
                or candidate.is_symlink()
                or not candidate.is_dir()
            ):
                continue
            try:
                _validate_regular_tree(candidate, "retired release")
                version = (candidate / "VERSION").read_text(
                    encoding="utf-8"
                ).strip()
                if version != candidate.name:
                    continue
                shutil.rmtree(candidate)
            except (OSError, UnicodeError, VerificationError):
                continue
    except (OSError, ValueError):
        return


def control_systemd_service(
    action: str,
    service_name: str,
    *,
    runner: Callable = subprocess.run,
    timeout_seconds: int = 60,
) -> None:
    if action not in ("start", "stop"):
        raise VerificationError("systemd service action is invalid")
    if (
        not isinstance(service_name, str)
        or not _SERVICE_PATTERN.fullmatch(service_name)
    ):
        raise VerificationError("systemd service name is invalid")
    _run_checked_command(
        ["systemctl", action, service_name],
        runner=runner,
        timeout_seconds=timeout_seconds,
        context=f"systemd {action}",
    )


def check_cleanbot_health(
    service_name: str,
    *,
    runner: Callable = subprocess.run,
    timeout_seconds: int = 15,
) -> bool:
    if (
        not isinstance(service_name, str)
        or not _SERVICE_PATTERN.fullmatch(service_name)
    ):
        raise VerificationError("systemd service name is invalid")
    try:
        _run_checked_command(
            ["systemctl", "is-active", "--quiet", service_name],
            runner=runner,
            timeout_seconds=timeout_seconds,
            context="cleanbot systemd health check",
        )
        service_type = _run_checked_command(
            _ros_cli_command(
                "service", "type", "/system/set_maintenance"
            ),
            runner=runner,
            timeout_seconds=timeout_seconds,
            context="cleanbot ROS health check",
        )
    except VerificationError:
        return False
    return (
        service_type.strip()
        == "cleanbot_interfaces/srv/SetMaintenanceMode"
    )


def _retry_boolean(
    operation: Callable[[], bool],
    *,
    attempts: int,
    sleeper: Callable[[float], None],
    interval_seconds: float,
) -> bool:
    if type(attempts) is not int or attempts <= 0:
        raise VerificationError("retry attempt count is invalid")
    for attempt in range(attempts):
        if operation():
            return True
        if attempt + 1 < attempts:
            sleeper(interval_seconds)
    return False


def apply_staged_release(
    config: UpdaterConfig,
    state: UpdateState,
    staged: StagedRelease,
    *,
    maintenance_call: Optional[Callable] = None,
    readiness_check: Optional[Callable[[int], bool]] = None,
    service_control: Optional[Callable[[str, str], None]] = None,
    installer: Callable[[Path, str, Path], Path] = install_staged_payload,
    switcher: Callable[[Path, Path, Path], Optional[Path]] = switch_current_release,
    health_check: Optional[Callable[[str], bool]] = None,
    state_writer: Callable[[Path, UpdateState], None] = write_update_state,
    readiness_attempts: int = 30,
    health_attempts: int = 30,
    sleeper: Callable[[float], None] = time.sleep,
    runner: Callable = subprocess.run,
) -> UpdateState:
    if not isinstance(config, UpdaterConfig):
        raise ConfigError("updater config is invalid")
    if not isinstance(state, UpdateState) or state.phase != "STAGED":
        raise StateError("only a staged update can be applied")
    if not isinstance(staged, StagedRelease):
        raise VerificationError("staged release descriptor is invalid")
    if (
        staged.release != state.staged_release
        or not _RELEASE_PATTERN.fullmatch(staged.release)
    ):
        raise StateError("staged release does not match update state")
    if not state.current_release:
        raise StateError("automatic update requires an installed current release")

    state_path = Path(config.state_path)
    releases_root = Path(config.releases_root)
    current_link = Path(config.current_link)
    maintenance_reason = "software-update"

    if maintenance_call is None:
        maintenance_call = lambda **request: call_maintenance_service(
            **request, runner=runner
        )
    if readiness_check is None:
        readiness_check = lambda generation: read_maintenance_ready(
            generation=generation, runner=runner
        )
    if service_control is None:
        service_control = lambda action, service: control_systemd_service(
            action, service, runner=runner
        )
    if health_check is None:
        health_check = lambda service: check_cleanbot_health(
            service, runner=runner
        )

    applying = UpdateState(
        current_release=state.current_release,
        previous_release=state.previous_release,
        staged_release=staged.release,
        phase="APPLYING",
        last_error="",
    )
    state_writer(state_path, applying)

    token = 0
    service_stopped = False
    service_disrupted = False
    switched = False
    rollback_unavailable = False
    maintenance_released = False
    previous_path: Optional[Path] = None
    installed_path: Optional[Path] = None
    try:
        enabled = maintenance_call(
            enable=True,
            generation=0,
            requester=config.requester,
            reason=maintenance_reason,
        )
        if (
            not isinstance(enabled, MaintenanceServiceResult)
            or not enabled.accepted
            or enabled.generation <= 0
        ):
            raise VerificationError(
                "maintenance enable did not return a valid generation"
            )
        token = enabled.generation
        ready = _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        )
        if not ready:
            raise VerificationError("maintenance readiness timed out")

        service_disrupted = True
        service_control("stop", config.service_name)
        service_stopped = True
        installed_path = installer(
            staged.path,
            staged.release,
            releases_root,
        )
        previous_path = switcher(
            current_link,
            installed_path,
            releases_root,
        )
        if previous_path is None:
            rollback_unavailable = True
            raise VerificationError(
                "automatic update cannot proceed without a rollback release"
            )
        switched = True
        service_control("start", config.service_name)
        service_stopped = False

        checking = UpdateState(
            current_release=staged.release,
            previous_release=previous_path.name,
            staged_release=staged.release,
            phase="HEALTH_CHECK",
            last_error="",
        )
        state_writer(state_path, checking)
        healthy = _retry_boolean(
            lambda: health_check(config.service_name),
            attempts=health_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        )
        if not healthy:
            raise VerificationError("new release failed its health check")
        if not _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError(
                "new release did not re-establish maintenance readiness"
            )

        released = maintenance_call(
            enable=False,
            generation=token,
            requester=config.requester,
            reason=maintenance_reason,
        )
        if (
            not isinstance(released, MaintenanceServiceResult)
            or not released.accepted
            or released.generation != token
        ):
            raise VerificationError(
                "maintenance release did not confirm the exact generation"
            )
        maintenance_released = True
        token = 0
        final = UpdateState(
            current_release=staged.release,
            previous_release=previous_path.name,
            staged_release="",
            phase="IDLE",
            last_error="",
        )
        state_writer(state_path, final)
        cleanup_successful_update(
            staged.path,
            Path(config.staging_root),
            releases_root,
            (staged.release, previous_path.name),
        )
        return final
    except Exception as exception:
        failure = str(exception) or exception.__class__.__name__
        if maintenance_released:
            failed = UpdateState(
                current_release=staged.release,
                previous_release=(
                    previous_path.name
                    if previous_path is not None
                    else state.current_release
                ),
                staged_release="",
                phase="FAILED",
                last_error=(
                    "update is running but final state commit failed after "
                    f"maintenance release: {failure}"
                ),
            )
            try:
                state_writer(state_path, failed)
            except Exception as state_exception:
                failure = (
                    f"{failed.last_error}; failed-state commit also failed: "
                    f"{state_exception}"
                )
            else:
                failure = failed.last_error
            raise UpdaterError(failure) from exception
        restored_release = state.current_release
        rollback_failure = ""
        safe_to_release = not service_disrupted
        if rollback_unavailable:
            restored_release = staged.release
            rollback_failure = (
                "maintenance remains active because the previous release "
                "could not be identified"
            )
        elif switched and previous_path is not None:
            rolling_back = UpdateState(
                current_release=staged.release,
                previous_release=previous_path.name,
                staged_release=staged.release,
                phase="ROLLING_BACK",
                last_error=failure,
            )
            state_writer(state_path, rolling_back)
            try:
                service_control("stop", config.service_name)
                service_stopped = True
                if installed_path is None:
                    raise VerificationError("installed release path is unavailable")
                switcher(current_link, previous_path, releases_root)
                switched = False
                service_control("start", config.service_name)
                service_stopped = False
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "rollback release failed its health check"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "rollback release did not re-establish "
                        "maintenance readiness"
                    )
                restored_release = previous_path.name
                safe_to_release = True
            except Exception as rollback_exception:
                rollback_failure = (
                    str(rollback_exception)
                    or rollback_exception.__class__.__name__
                )
        elif service_stopped:
            try:
                service_control("start", config.service_name)
                service_stopped = False
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "restored release failed its health check"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "restored release did not re-establish "
                        "maintenance readiness"
                    )
                safe_to_release = True
            except Exception as restart_exception:
                rollback_failure = (
                    str(restart_exception)
                    or restart_exception.__class__.__name__
                )
        elif service_disrupted:
            try:
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "service health is unknown after failed stop"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "service did not retain maintenance readiness "
                        "after failed stop"
                    )
                safe_to_release = True
            except Exception as health_exception:
                rollback_failure = (
                    str(health_exception)
                    or health_exception.__class__.__name__
                )

        if token and safe_to_release:
            try:
                released = maintenance_call(
                    enable=False,
                    generation=token,
                    requester=config.requester,
                    reason=maintenance_reason,
                )
                if (
                    not isinstance(released, MaintenanceServiceResult)
                    or not released.accepted
                    or released.generation != token
                ):
                    raise VerificationError(
                        "maintenance release did not confirm the exact generation"
                    )
                token = 0
            except Exception as release_exception:
                release_failure = (
                    str(release_exception)
                    or release_exception.__class__.__name__
                )
                rollback_failure = "; ".join(
                    part for part in (rollback_failure, release_failure) if part
                )
        elif token:
            rollback_failure = "; ".join(
                part
                for part in (
                    rollback_failure,
                    "maintenance remains active because runtime health is unconfirmed",
                )
                if part
            )

        combined_failure = failure
        if rollback_failure:
            combined_failure = (
                f"{failure}; rollback/release failure: {rollback_failure}"
            )
        failed = UpdateState(
            current_release=restored_release,
            previous_release=state.previous_release,
            staged_release=staged.release,
            phase="FAILED",
            last_error=combined_failure,
        )
        state_writer(state_path, failed)
        raise UpdaterError(combined_failure) from exception


def stage_latest_release(
    config: UpdaterConfig,
    state: UpdateState,
    *,
    opener: Callable = urlopen,
    signature_verifier: Callable[
        [Path, Path, Path], None
    ] = verify_manifest_signature,
    state_writer: Callable[[Path, UpdateState], None] = write_update_state,
) -> Optional[StagedRelease]:
    if not isinstance(config, UpdaterConfig):
        raise ConfigError("updater config is invalid")
    if not isinstance(state, UpdateState) or not (
        state.phase == "IDLE"
        or (state.phase == "FAILED" and not state.staged_release)
    ):
        raise StateError(
            "latest release can only be staged from IDLE or a retryable "
            "FAILED state"
        )

    release = fetch_latest_release(config.repository, opener=opener)
    if release.tag == state.current_release:
        if state.phase == "FAILED":
            state_writer(
                Path(config.state_path),
                UpdateState(
                    current_release=state.current_release,
                    previous_release=state.previous_release,
                    staged_release="",
                    phase="IDLE",
                    last_error="",
                ),
            )
        return None
    current_match = _SEMVER_PATTERN.fullmatch(state.current_release)
    release_match = _SEMVER_PATTERN.fullmatch(release.tag)
    if current_match is None or release_match is None:
        raise ReleaseError(
            "automatic updates require stable semantic release versions"
        )
    current_version = tuple(int(part) for part in current_match.groups())
    release_version = tuple(int(part) for part in release_match.groups())
    if release_version <= current_version:
        raise ReleaseError(
            "latest GitHub release is not newer than the installed release"
        )

    state_path = Path(config.state_path)
    downloading = UpdateState(
        current_release=state.current_release,
        previous_release=state.previous_release,
        staged_release=release.tag,
        phase="DOWNLOADING",
        last_error="",
    )
    state_writer(state_path, downloading)

    staging_root = Path(config.staging_root)
    staging_root.mkdir(parents=True, exist_ok=True)
    ensure_free_space(
        staging_root,
        sum(asset.size for asset in release.assets.values()),
        "release download",
    )
    download_root = Path(
        tempfile.mkdtemp(
            prefix=f".{release.tag}.download-",
            dir=staging_root,
        )
    )
    staged: Optional[StagedRelease] = None
    try:
        downloaded = {}
        for name in REQUIRED_RELEASE_ASSETS:
            downloaded[name] = download_release_asset(
                release.assets[name],
                download_root / name,
                opener=opener,
            )
        staged = stage_release(
            manifest_path=downloaded[MANIFEST_ASSET],
            signature_path=downloaded[SIGNATURE_ASSET],
            archive_path=downloaded[ARCHIVE_ASSET],
            public_key_path=Path(config.public_key),
            staging_root=staging_root,
            expected_architecture="arm64",
            expected_os="ubuntu-22.04",
            expected_ros="humble",
            signature_verifier=signature_verifier,
        )
        if staged.release != release.tag:
            shutil.rmtree(staged.path)
            staged = None
            raise VerificationError(
                "signed manifest release does not match GitHub release tag"
            )
        staged_state = UpdateState(
            current_release=state.current_release,
            previous_release=state.previous_release,
            staged_release=staged.release,
            phase="STAGED",
            last_error="",
        )
        state_writer(state_path, staged_state)
        return staged
    except Exception as exception:
        if staged is not None:
            shutil.rmtree(staged.path, ignore_errors=True)
        failed = UpdateState(
            current_release=state.current_release,
            previous_release=state.previous_release,
            staged_release="",
            phase="FAILED",
            last_error=str(exception) or exception.__class__.__name__,
        )
        state_writer(state_path, failed)
        raise UpdaterError(failed.last_error) from exception
    finally:
        shutil.rmtree(download_root, ignore_errors=True)


def rollback_to_previous_release(
    config: UpdaterConfig,
    state: UpdateState,
    *,
    maintenance_call: Optional[Callable] = None,
    readiness_check: Optional[Callable[[int], bool]] = None,
    service_control: Optional[Callable[[str, str], None]] = None,
    switcher: Callable[[Path, Path, Path], Optional[Path]] = switch_current_release,
    health_check: Optional[Callable[[str], bool]] = None,
    state_writer: Callable[[Path, UpdateState], None] = write_update_state,
    readiness_attempts: int = 30,
    health_attempts: int = 30,
    sleeper: Callable[[float], None] = time.sleep,
    runner: Callable = subprocess.run,
) -> UpdateState:
    if not isinstance(config, UpdaterConfig):
        raise ConfigError("updater config is invalid")
    if (
        not isinstance(state, UpdateState)
        or state.phase not in ("IDLE", "FAILED")
    ):
        raise StateError("rollback requires an idle or failed update state")
    if (
        not state.current_release
        or not state.previous_release
        or state.current_release == state.previous_release
    ):
        raise StateError("rollback requires distinct current and previous releases")

    state_path = Path(config.state_path)
    releases_root = Path(config.releases_root)
    current_link = Path(config.current_link)
    target = _contained_directory(
        releases_root / state.previous_release,
        releases_root,
        "rollback release",
    )
    original = _contained_directory(
        releases_root / state.current_release,
        releases_root,
        "current release",
    )
    if maintenance_call is None:
        maintenance_call = lambda **request: call_maintenance_service(
            **request, runner=runner
        )
    if readiness_check is None:
        readiness_check = lambda generation: read_maintenance_ready(
            generation=generation, runner=runner
        )
    if service_control is None:
        service_control = lambda action, service: control_systemd_service(
            action, service, runner=runner
        )
    if health_check is None:
        health_check = lambda service: check_cleanbot_health(
            service, runner=runner
        )

    state_writer(
        state_path,
        UpdateState(
            state.current_release,
            state.previous_release,
            state.staged_release,
            "ROLLING_BACK",
            "",
        ),
    )
    token = 0
    service_stopped = False
    service_disrupted = False
    switched = False
    maintenance_released = False
    try:
        enabled = maintenance_call(
            enable=True,
            generation=0,
            requester=config.requester,
            reason="software-rollback",
        )
        if (
            not isinstance(enabled, MaintenanceServiceResult)
            or not enabled.accepted
            or enabled.generation <= 0
        ):
            raise VerificationError(
                "maintenance enable did not return a valid generation"
            )
        token = enabled.generation
        if not _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError("maintenance readiness timed out")
        service_disrupted = True
        service_control("stop", config.service_name)
        service_stopped = True
        replaced = switcher(current_link, target, releases_root)
        switched = True
        if replaced is None or replaced.name != original.name:
            raise VerificationError(
                "rollback switch did not identify the current release"
            )
        service_control("start", config.service_name)
        service_stopped = False
        state_writer(
            state_path,
            UpdateState(
                state.previous_release,
                state.current_release,
                state.staged_release,
                "HEALTH_CHECK",
                "",
            ),
        )
        if not _retry_boolean(
            lambda: health_check(config.service_name),
            attempts=health_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError(
                "rollback release failed its health check"
            )
        if not _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError(
                "rollback release did not re-establish maintenance readiness"
            )
        released = maintenance_call(
            enable=False,
            generation=token,
            requester=config.requester,
            reason="software-rollback",
        )
        if (
            not isinstance(released, MaintenanceServiceResult)
            or not released.accepted
            or released.generation != token
        ):
            raise VerificationError(
                "maintenance release did not confirm the exact generation"
            )
        maintenance_released = True
        token = 0
        final = UpdateState(
            current_release=state.previous_release,
            previous_release=state.current_release,
            staged_release=state.staged_release,
            phase="IDLE",
            last_error="",
        )
        state_writer(state_path, final)
        return final
    except Exception as exception:
        failure = str(exception) or exception.__class__.__name__
        if maintenance_released:
            failed = UpdateState(
                current_release=state.previous_release,
                previous_release=state.current_release,
                staged_release=state.staged_release,
                phase="FAILED",
                last_error=(
                    "rollback is running but final state commit failed after "
                    f"maintenance release: {failure}"
                ),
            )
            try:
                state_writer(state_path, failed)
            except Exception as state_exception:
                failure = (
                    f"{failed.last_error}; failed-state commit also failed: "
                    f"{state_exception}"
                )
            else:
                failure = failed.last_error
            raise UpdaterError(failure) from exception
        recovery_failures = []
        safe_to_release = not service_disrupted
        if switched:
            try:
                service_control("stop", config.service_name)
                service_stopped = True
                switcher(current_link, original, releases_root)
                service_control("start", config.service_name)
                service_stopped = False
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "original release failed after rollback recovery"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "original release did not re-establish "
                        "maintenance readiness"
                    )
                safe_to_release = True
            except Exception as recovery_exception:
                recovery_failures.append(
                    str(recovery_exception)
                    or recovery_exception.__class__.__name__
                )
        elif service_stopped:
            try:
                service_control("start", config.service_name)
                service_stopped = False
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "original release failed after restart"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "original release did not re-establish "
                        "maintenance readiness"
                    )
                safe_to_release = True
            except Exception as recovery_exception:
                recovery_failures.append(
                    str(recovery_exception)
                    or recovery_exception.__class__.__name__
                )
        elif service_disrupted:
            try:
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "service health is unknown after failed stop"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "service did not retain maintenance readiness "
                        "after failed stop"
                    )
                safe_to_release = True
            except Exception as recovery_exception:
                recovery_failures.append(
                    str(recovery_exception)
                    or recovery_exception.__class__.__name__
                )
        if token and safe_to_release:
            try:
                released = maintenance_call(
                    enable=False,
                    generation=token,
                    requester=config.requester,
                    reason="software-rollback",
                )
                if (
                    not isinstance(released, MaintenanceServiceResult)
                    or not released.accepted
                    or released.generation != token
                ):
                    raise VerificationError(
                        "maintenance release did not confirm the exact generation"
                    )
            except Exception as release_exception:
                recovery_failures.append(
                    str(release_exception)
                    or release_exception.__class__.__name__
                )
        elif token:
            recovery_failures.append(
                "maintenance remains active because runtime health is unconfirmed"
            )
        if recovery_failures:
            failure += "; recovery failure: " + "; ".join(recovery_failures)
        state_writer(
            state_path,
            UpdateState(
                state.current_release,
                state.previous_release,
                state.staged_release,
                "FAILED",
                failure,
            ),
        )
        raise UpdaterError(failure) from exception


def recover_interrupted_transition(
    config: UpdaterConfig,
    state: UpdateState,
    *,
    maintenance_call: Optional[Callable] = None,
    readiness_check: Optional[Callable[[int], bool]] = None,
    service_control: Optional[Callable[[str, str], None]] = None,
    switcher: Callable[[Path, Path, Path], Optional[Path]] = (
        switch_current_release
    ),
    health_check: Optional[Callable[[str], bool]] = None,
    state_writer: Callable[[Path, UpdateState], None] = write_update_state,
    current_loader: Optional[Callable[[Path, Path], Path]] = None,
    readiness_attempts: int = 30,
    health_attempts: int = 30,
    sleeper: Callable[[float], None] = time.sleep,
    runner: Callable = subprocess.run,
) -> UpdateState:
    if not isinstance(config, UpdaterConfig):
        raise ConfigError("updater config is invalid")
    if not isinstance(state, UpdateState):
        raise StateError("updater state is invalid")
    if state.phase not in ("APPLYING", "HEALTH_CHECK", "ROLLING_BACK"):
        return state

    releases_root = Path(config.releases_root)
    current_link = Path(config.current_link)
    desired_release = (
        state.current_release if state.phase == "APPLYING"
        else state.previous_release
    )
    if not desired_release:
        raise StateError(
            "interrupted update has no known safe recovery release"
        )
    desired_path = _contained_directory(
        releases_root / desired_release,
        releases_root,
        "interrupted update recovery release",
    )
    if current_loader is None:
        actual_path = _contained_directory(
            current_link,
            releases_root,
            "current release",
        )
    else:
        actual_path = _contained_directory(
            current_loader(current_link, releases_root),
            releases_root,
            "current release",
        )

    if maintenance_call is None:
        maintenance_call = lambda **request: call_maintenance_service(
            **request, runner=runner
        )
    if readiness_check is None:
        readiness_check = lambda generation: read_maintenance_ready(
            generation=generation, runner=runner
        )
    if service_control is None:
        service_control = lambda action, service: control_systemd_service(
            action, service, runner=runner
        )
    if health_check is None:
        health_check = lambda service: check_cleanbot_health(
            service, runner=runner
        )

    token = 0
    service_disrupted = False
    service_stopped = False
    maintenance_released = False
    active_release = actual_path.name
    try:
        enabled = maintenance_call(
            enable=True,
            generation=0,
            requester=config.requester,
            reason="software-update-recovery",
        )
        if (
            not isinstance(enabled, MaintenanceServiceResult)
            or not enabled.accepted
            or enabled.generation <= 0
        ):
            raise VerificationError(
                "maintenance enable did not return a valid generation"
            )
        token = enabled.generation
        if not _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError("maintenance readiness timed out")

        service_disrupted = True
        service_control("stop", config.service_name)
        service_stopped = True
        if actual_path != desired_path:
            replaced = switcher(
                current_link,
                desired_path,
                releases_root,
            )
            active_release = desired_release
            if replaced is None or replaced.name != actual_path.name:
                raise VerificationError(
                    "interrupted update recovery switch did not identify "
                    "the active release"
                )
        service_control("start", config.service_name)
        service_stopped = False
        if not _retry_boolean(
            lambda: health_check(config.service_name),
            attempts=health_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError(
                "interrupted update recovery release is unhealthy"
            )
        if not _retry_boolean(
            lambda: readiness_check(token),
            attempts=readiness_attempts,
            sleeper=sleeper,
            interval_seconds=1.0,
        ):
            raise VerificationError(
                "recovered release did not re-establish maintenance readiness"
            )
        released = maintenance_call(
            enable=False,
            generation=token,
            requester=config.requester,
            reason="software-update-recovery",
        )
        if (
            not isinstance(released, MaintenanceServiceResult)
            or not released.accepted
            or released.generation != token
        ):
            raise VerificationError(
                "maintenance release did not confirm the exact generation"
            )
        maintenance_released = True
        token = 0
        recovered = UpdateState(
            current_release=active_release,
            previous_release=actual_path.name,
            staged_release=state.staged_release,
            phase="FAILED",
            last_error=(
                f"interrupted {state.phase} was recovered to "
                f"{active_release}; operator confirmation is required"
            ),
        )
        state_writer(Path(config.state_path), recovered)
        return recovered
    except Exception as exception:
        failure = str(exception) or exception.__class__.__name__
        if maintenance_released:
            raise UpdaterError(
                "interrupted update is safely running but recovery state "
                f"could not be committed: {failure}"
            ) from exception

        safe_to_release = not service_disrupted
        recovery_failure = ""
        if service_stopped:
            try:
                service_control("start", config.service_name)
                service_stopped = False
                if not _retry_boolean(
                    lambda: health_check(config.service_name),
                    attempts=health_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "recovery release failed its health check"
                    )
                if not _retry_boolean(
                    lambda: readiness_check(token),
                    attempts=readiness_attempts,
                    sleeper=sleeper,
                    interval_seconds=1.0,
                ):
                    raise VerificationError(
                        "recovery release did not retain maintenance readiness"
                    )
                safe_to_release = True
            except Exception as recovery_exception:
                recovery_failure = (
                    str(recovery_exception)
                    or recovery_exception.__class__.__name__
                )
        if token and safe_to_release:
            try:
                released = maintenance_call(
                    enable=False,
                    generation=token,
                    requester=config.requester,
                    reason="software-update-recovery",
                )
                if (
                    not isinstance(released, MaintenanceServiceResult)
                    or not released.accepted
                    or released.generation != token
                ):
                    raise VerificationError(
                        "maintenance release did not confirm the exact generation"
                    )
                token = 0
            except Exception as release_exception:
                recovery_failure = "; ".join(
                    part
                    for part in (
                        recovery_failure,
                        str(release_exception)
                        or release_exception.__class__.__name__,
                    )
                    if part
                )
        elif token:
            recovery_failure = "; ".join(
                part
                for part in (
                    recovery_failure,
                    "maintenance remains active because runtime health "
                    "is unconfirmed",
                )
                if part
            )
        if recovery_failure:
            failure = (
                f"{failure}; interrupted recovery failure: {recovery_failure}"
            )
        failed = UpdateState(
            current_release=active_release,
            previous_release=actual_path.name,
            staged_release=state.staged_release,
            phase="FAILED",
            last_error=failure,
        )
        state_writer(Path(config.state_path), failed)
        raise UpdaterError(failure) from exception


def recover_interrupted_download(
    config: UpdaterConfig,
    state: UpdateState,
    *,
    staged_loader: Callable[[Path, str, Path], StagedRelease] = (
        load_staged_release
    ),
    state_writer: Callable[[Path, UpdateState], None] = write_update_state,
) -> UpdateState:
    if not isinstance(config, UpdaterConfig):
        raise ConfigError("updater config is invalid")
    if not isinstance(state, UpdateState):
        raise StateError("updater state is invalid")
    if state.phase != "DOWNLOADING":
        return state
    if not state.staged_release:
        raise StateError("interrupted download has no release identifier")

    staging_root = Path(config.staging_root)
    staged_path = staging_root / state.staged_release
    if staged_path.exists() or staged_path.is_symlink():
        staged = staged_loader(
            staged_path,
            state.staged_release,
            Path(config.public_key),
        )
        recovered = UpdateState(
            current_release=state.current_release,
            previous_release=state.previous_release,
            staged_release=staged.release,
            phase="STAGED",
            last_error="",
        )
        state_writer(Path(config.state_path), recovered)
        return recovered

    for prefix in (
        f".{state.staged_release}.download-",
        f".{state.staged_release}.stage-",
    ):
        if not staging_root.is_dir() or staging_root.is_symlink():
            break
        for candidate in staging_root.iterdir():
            if not candidate.name.startswith(prefix):
                continue
            if candidate.is_symlink() or not candidate.is_dir():
                raise VerificationError(
                    "interrupted update temporary path is unsafe"
                )
            shutil.rmtree(candidate)
    recovered = UpdateState(
        current_release=state.current_release,
        previous_release=state.previous_release,
        staged_release="",
        phase="IDLE",
        last_error="",
    )
    state_writer(Path(config.state_path), recovered)
    return recovered


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="cleanbot-updater",
        description="Safely update a Cleanbot ROS2 installation.",
    )
    parser.add_argument(
        "--config",
        default="/etc/cleanbot/updater.json",
        help="strict updater configuration file",
    )
    subcommands = parser.add_subparsers(dest="command", required=True)
    for command in ("check", "stage", "apply", "rollback", "status", "auto"):
        subcommands.add_parser(command)
    return parser


def status_object(state: UpdateState) -> Dict[str, str]:
    if not isinstance(state, UpdateState):
        raise StateError("updater state is invalid")
    return {
        "currentRelease": state.current_release,
        "previousRelease": state.previous_release,
        "stagedRelease": state.staged_release,
        "phase": state.phase,
        "lastError": state.last_error,
    }


def load_updater_config(path: Path) -> UpdaterConfig:
    path = Path(path)
    information = _regular_file(path, "updater config")
    if information.st_size <= 0 or information.st_size > MAX_MANIFEST_BYTES:
        raise ConfigError("updater config size is invalid")
    try:
        data = path.read_bytes()
    except OSError as exception:
        raise ConfigError(
            f"updater config cannot be read: {exception}"
        ) from exception
    return parse_config(data)


def _write_cli_json(output, value: Any) -> None:
    output.write(
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    )


def run_cli(
    argv=None,
    *,
    output=None,
    error_output=None,
    config_loader: Callable[[Path], UpdaterConfig] = load_updater_config,
    state_loader: Callable[[Path], UpdateState] = load_update_state,
    release_fetcher: Callable[[str], LatestRelease] = fetch_latest_release,
    stager: Callable[[UpdaterConfig, UpdateState], Optional[StagedRelease]] = (
        stage_latest_release
    ),
    staged_loader: Callable[[Path, str, Path], StagedRelease] = (
        load_staged_release
    ),
    applier: Callable[
        [UpdaterConfig, UpdateState, StagedRelease], UpdateState
    ] = apply_staged_release,
    rollbacker: Callable[
        [UpdaterConfig, UpdateState], UpdateState
    ] = rollback_to_previous_release,
    transition_recoverer: Callable[
        [UpdaterConfig, UpdateState], UpdateState
    ] = recover_interrupted_transition,
) -> int:
    if output is None:
        output = sys.stdout
    if error_output is None:
        error_output = sys.stderr
    arguments = build_cli_parser().parse_args(argv)
    try:
        config = config_loader(Path(arguments.config))
        state = state_loader(Path(config.state_path))
        if arguments.command == "auto" and state.phase == "DOWNLOADING":
            state = recover_interrupted_download(
                config,
                state,
                staged_loader=staged_loader,
            )
        if arguments.command == "auto" and state.phase in (
            "APPLYING",
            "HEALTH_CHECK",
            "ROLLING_BACK",
        ):
            state = transition_recoverer(config, state)
            _write_cli_json(
                output,
                {"result": "recovered-interrupted", **status_object(state)},
            )
            return 0

        if arguments.command == "status":
            _write_cli_json(output, status_object(state))
            return 0
        if arguments.command == "check":
            release = release_fetcher(config.repository)
            _write_cli_json(
                output,
                {
                    "available": release.tag != state.current_release,
                    "currentRelease": state.current_release,
                    "latestRelease": release.tag,
                },
            )
            return 0
        if arguments.command == "stage":
            staged = stager(config, state)
            _write_cli_json(
                output,
                {
                    "result": "staged" if staged is not None else "up-to-date",
                    "release": (
                        staged.release
                        if staged is not None
                        else state.current_release
                    ),
                },
            )
            return 0
        if arguments.command == "apply":
            if state.phase != "STAGED" or not state.staged_release:
                raise StateError("there is no staged release to apply")
            staged = staged_loader(
                Path(config.staging_root) / state.staged_release,
                state.staged_release,
                Path(config.public_key),
            )
            final = applier(config, state, staged)
            _write_cli_json(
                output,
                {"result": "applied", **status_object(final)},
            )
            return 0
        if arguments.command == "rollback":
            final = rollbacker(config, state)
            _write_cli_json(
                output,
                {"result": "rolled-back", **status_object(final)},
            )
            return 0
        if arguments.command == "auto":
            if state.phase == "IDLE" or (
                state.phase == "FAILED" and not state.staged_release
            ):
                if not config.auto_download:
                    _write_cli_json(output, {"result": "automatic-download-disabled"})
                    return 0
                staged = stager(config, state)
                if staged is None:
                    _write_cli_json(output, {"result": "up-to-date"})
                    return 0
                staged_state = UpdateState(
                    current_release=state.current_release,
                    previous_release=state.previous_release,
                    staged_release=staged.release,
                    phase="STAGED",
                    last_error="",
                )
                if not config.auto_apply:
                    _write_cli_json(
                        output,
                        {"result": "staged", "release": staged.release},
                    )
                    return 0
                final = applier(config, staged_state, staged)
                _write_cli_json(
                    output,
                    {"result": "applied", **status_object(final)},
                )
                return 0
            if state.phase == "STAGED" and config.auto_apply:
                staged = staged_loader(
                    Path(config.staging_root) / state.staged_release,
                    state.staged_release,
                    Path(config.public_key),
                )
                final = applier(config, state, staged)
                _write_cli_json(
                    output,
                    {"result": "applied", **status_object(final)},
                )
                return 0
            _write_cli_json(
                output,
                {
                    "result": "no-action",
                    "phase": state.phase,
                    "autoApply": config.auto_apply,
                },
            )
            return 0
        raise StateError("unsupported updater command")
    except (UpdaterError, OSError) as exception:
        _write_cli_json(
            error_output,
            {
                "error": str(exception) or exception.__class__.__name__,
                "result": "failed",
            },
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(run_cli())
