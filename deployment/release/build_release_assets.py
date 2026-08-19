#!/usr/bin/env python3
# 文件作用：生成内容可复现的 Cleanbot ARM64 发布压缩包和校验清单。

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import re
import stat
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from typing import NamedTuple, Optional


ARCHIVE_NAME = "cleanbot-ros2-humble-arm64.tar.gz"
MANIFEST_NAME = "manifest.json"
_RELEASE_PATTERN = re.compile(
    r"^v?(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$"
)
_CREATED_AT_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$"
)


class ReleaseAssets(NamedTuple):
    archive: Path
    manifest: Path


# 方法作用：把对象编码为字段有序、无多余空白且以换行结尾的规范 JSON 字节。
def _canonical_json_bytes(value) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        + b"\n"
    )


# 方法作用：校验安装目录结构，并按稳定顺序列出需要写入发布包的目录和文件。
def _release_entries(install_directory: Path):
    if not install_directory.is_dir() or install_directory.is_symlink():
        raise ValueError("install directory must be a real directory")
    setup = install_directory / "setup.bash"
    if not setup.is_file() or setup.is_symlink():
        raise ValueError("install/setup.bash is missing")
    entries = [("install", install_directory)]
    for path in sorted(
        install_directory.rglob("*"),
        key=lambda item: item.relative_to(install_directory).as_posix(),
    ):
        information = path.lstat()
        if path.is_symlink() or not (
            stat.S_ISDIR(information.st_mode)
            or stat.S_ISREG(information.st_mode)
        ):
            raise ValueError(f"unsupported install entry: {path}")
        name = (
            Path("install") / path.relative_to(install_directory)
        ).as_posix()
        entries.append((name, path))
    return entries


# 方法作用：为归档成员生成固定属主、时间戳和权限的元数据，保证构建结果可复现。
def _normalized_info(name: str, source: Optional[Path]) -> tarfile.TarInfo:
    information = tarfile.TarInfo(name)
    information.uid = 0
    information.gid = 0
    information.uname = "root"
    information.gname = "root"
    information.mtime = 0
    if source is None:
        information.type = tarfile.REGTYPE
        information.mode = 0o644
        return information
    source_mode = source.lstat().st_mode
    if stat.S_ISDIR(source_mode):
        information.type = tarfile.DIRTYPE
        information.mode = 0o755
        information.size = 0
    else:
        information.type = tarfile.REGTYPE
        information.mode = 0o755 if source_mode & 0o111 else 0o644
        information.size = source.lstat().st_size
    return information


# 方法作用：分块读取文件并返回其 SHA-256 十六进制摘要。
def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


# 方法作用：校验版本参数，构建确定性压缩包并生成包含大小和摘要的发布清单。
def build_release_assets(
    *,
    release: str,
    install_directory: Path,
    output_directory: Path,
    created_at: Optional[str] = None,
) -> ReleaseAssets:
    if not isinstance(release, str) or not _RELEASE_PATTERN.fullmatch(release):
        raise ValueError("unsafe release name")
    if created_at is None:
        created_at = datetime.now(timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"
        )
    if (
        not isinstance(created_at, str)
        or not _CREATED_AT_PATTERN.fullmatch(created_at)
    ):
        raise ValueError("createdAt must be UTC with whole-second precision")

    install_directory = Path(install_directory)
    entries = _release_entries(install_directory)
    output_directory = Path(output_directory)
    if output_directory.exists():
        if not output_directory.is_dir() or any(output_directory.iterdir()):
            raise ValueError("output directory must be absent or empty")
    else:
        output_directory.mkdir(parents=True)

    archive_path = output_directory / ARCHIVE_NAME
    version_bytes = release.encode("utf-8") + b"\n"
    unpacked_size = len(version_bytes)
    with archive_path.open("wb") as raw_archive:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            fileobj=raw_archive,
            mtime=0,
        ) as compressed:
            with tarfile.open(
                fileobj=compressed,
                mode="w",
                format=tarfile.PAX_FORMAT,
            ) as archive:
                version_info = _normalized_info("VERSION", None)
                version_info.size = len(version_bytes)
                import io

                archive.addfile(version_info, io.BytesIO(version_bytes))
                for name, source in entries:
                    member = _normalized_info(name, source)
                    if member.isfile():
                        unpacked_size += member.size
                        with source.open("rb") as data:
                            archive.addfile(member, data)
                    else:
                        archive.addfile(member)
        raw_archive.flush()
        os.fsync(raw_archive.fileno())

    manifest_path = output_directory / MANIFEST_NAME
    manifest = {
        "schemaVersion": 1,
        "release": release,
        "createdAt": created_at,
        "target": {
            "architecture": "arm64",
            "os": "ubuntu-22.04",
            "ros": "humble",
        },
        "archive": {
            "name": ARCHIVE_NAME,
            "sha256": _sha256(archive_path),
            "size": archive_path.stat().st_size,
            "unpackedSize": unpacked_size,
        },
    }
    with manifest_path.open("xb") as manifest_file:
        manifest_file.write(_canonical_json_bytes(manifest))
        manifest_file.flush()
        os.fsync(manifest_file.fileno())
    return ReleaseAssets(archive_path, manifest_path)


# 方法作用：创建发布资产构建命令的参数解析器。
def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--release", required=True)
    parser.add_argument("--install", required=True)
    parser.add_argument("--output", required=True)
    return parser


# 程序入口：解析命令行参数，生成发布资产并输出生成文件路径。
def main(argv=None) -> int:
    arguments = _parser().parse_args(argv)
    result = build_release_assets(
        release=arguments.release,
        install_directory=Path(arguments.install),
        output_directory=Path(arguments.output),
    )
    print(result.archive)
    print(result.manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
