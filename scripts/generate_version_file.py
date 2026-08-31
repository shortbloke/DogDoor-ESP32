from __future__ import annotations

import subprocess
from datetime import datetime, timezone
from pathlib import Path

from SCons.Script import Import  # type: ignore

Import("env")  # type: ignore # noqa: F821 pylint: disable=undefined-variable

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))  # type: ignore # noqa: F821 pylint: disable=undefined-variable
SRC_DIR = Path(env.subst("$PROJECT_SRC_DIR"))  # type: ignore # noqa: F821 pylint: disable=undefined-variable
OUTPUT_PATH = SRC_DIR / "Version.h"
FALLBACK_VERSION = "1.0.0"
FALLBACK_HASH = "unknown"


def _run_git_command(args: list[str]) -> str | None:
    """Run a git command relative to the project; return stripped output or None."""
    git_exe = env.GetProjectOption("custom_git_executable", "git")  # type: ignore # noqa: F821 pylint: disable=undefined-variable
    try:
        completed = subprocess.run(
            [git_exe, "-C", str(PROJECT_DIR), *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    output = completed.stdout.strip()
    return output or None


def _probe_version() -> tuple[str, str, bool]:
    """Return the configured version and current Git metadata."""
    version = env.GetProjectOption("custom_firmware_version", FALLBACK_VERSION)  # type: ignore # noqa: F821 pylint: disable=undefined-variable

    describe = _run_git_command(["describe", "--dirty", "--always"])
    git_hash = _run_git_command(["rev-parse", "--short", "HEAD"]) or FALLBACK_HASH
    is_dirty = bool(describe and describe.endswith("-dirty"))

    return version, git_hash, is_dirty


def _build_header(version: str, build_date: str, git_hash: str, is_dirty: bool) -> str:
    string_fields = {
        "kVersion": version,
        "kBuildDate": build_date,
        "kVersionWithDate": f"{version} ({build_date})",
        "kGitHash": git_hash,
    }

    lines = ["#pragma once", "", "namespace AppVersion {",]
    for key, value in string_fields.items():
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        lines.append(f'constexpr const char {key}[] = "{escaped}";')
    lines.append(f"constexpr const bool kDirty = {'true' if is_dirty else 'false'};")
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def _write_if_changed(path: Path, contents: str) -> None:
    if path.exists():
        existing = path.read_text(encoding="utf-8")
        if existing == contents:
            return
    path.write_text(contents, encoding="utf-8")


def generate_version_header() -> None:
    version, git_hash, is_dirty = _probe_version()
    build_date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    header_contents = _build_header(version, build_date, git_hash, is_dirty)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    _write_if_changed(OUTPUT_PATH, header_contents)


# Run immediately so the header exists before PlatformIO scans sources.
generate_version_header()
