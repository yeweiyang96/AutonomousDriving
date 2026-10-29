"""Find an installed, compatible Windows CMake / Visual Studio toolchain.

Only discovery commands are run. Nothing is downloaded, installed, or changed.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import NamedTuple


class _Installation(NamedTuple):
    path: str
    version: tuple[int, ...]
    prerelease: bool


def _run_json(command: list[str], label: str) -> object:
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"{label} 超时（30 秒），请稍后重试。") from exc
    except OSError as exc:
        raise RuntimeError(f"无法运行{label}，请检查工具是否可用。") from exc
    if result.returncode:
        raise RuntimeError(f"{label} 执行失败（退出码 {result.returncode}）。")
    try:
        return json.loads(result.stdout.lstrip("\ufeff"))
    except (json.JSONDecodeError, TypeError) as exc:
        raise RuntimeError(f"{label} 返回了无法识别的结果。") from exc


def _find_vswhere() -> str:
    for variable in ("ProgramFiles(x86)", "ProgramFiles"):
        base = os.environ.get(variable)
        if base:
            candidate = Path(base) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
            if candidate.is_file():
                return str(candidate)
    candidate = shutil.which("vswhere.exe") or shutil.which("vswhere")
    if candidate:
        return candidate
    raise RuntimeError("未找到 vswhere.exe；请确认 Visual Studio C++ Build Tools 已安装。")


def _find_installations(vswhere: str) -> list[_Installation]:
    records = _run_json(
        [
            vswhere, "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
            "-format", "json", "-utf8", "-prerelease",
        ],
        "Visual Studio 安装查询",
    )
    if not isinstance(records, list):
        raise RuntimeError("Visual Studio 安装信息格式不正确。")
    installations = []
    seen = set()
    for record in records:
        if not isinstance(record, dict):
            continue
        path = record.get("installationPath")
        version = record.get("installationVersion")
        if (not isinstance(path, str) or not path.strip()
                or not isinstance(version, str)
                or re.fullmatch(r"\d+(?:\.\d+)*", version) is None
                or record.get("isComplete") is False
                or record.get("isLaunchable") is False):
            continue
        path_key = os.path.normcase(os.path.normpath(path))
        if path_key in seen:
            continue
        seen.add(path_key)
        installations.append(_Installation(
            path=path,
            version=tuple(int(part) for part in version.split(".")),
            prerelease=record.get("isPrerelease") is True,
        ))
    if not installations:
        raise RuntimeError("未找到可用的 Visual Studio x64 C++ 工具链；请检查 C++ 桌面开发组件。")
    # Stable installations first, then newest versions within each channel.
    installations.sort(key=lambda item: (not item.prerelease, item.version), reverse=True)
    return installations


def _cmake_candidates(installations: list[_Installation]) -> list[str]:
    candidates = []
    on_path = shutil.which("cmake") or shutil.which("cmake.exe")
    if on_path:
        candidates.append(on_path)
    for installation in installations:
        bundled = (Path(installation.path) / "Common7" / "IDE" / "CommonExtensions"
                   / "Microsoft" / "CMake" / "CMake" / "bin" / "cmake.exe")
        if bundled.is_file():
            candidates.append(str(bundled))
    unique = []
    seen = set()
    for candidate in candidates:
        key = os.path.normcase(os.path.normpath(candidate))
        if key not in seen:
            seen.add(key)
            unique.append(candidate)
    if not unique:
        raise RuntimeError("未找到 CMake；请检查 PATH 或 Visual Studio 的 CMake 工具组件。")
    return unique


def _capabilities(cmake: str) -> dict:
    capabilities = _run_json([cmake, "-E", "capabilities"], "CMake 工具查询")
    if not isinstance(capabilities, dict) or not isinstance(capabilities.get("generators"), list):
        raise RuntimeError("CMake 生成器信息格式不正确。")
    version = capabilities.get("version")
    if isinstance(version, dict):
        major, minor = version.get("major"), version.get("minor")
        if isinstance(major, int) and isinstance(minor, int) and (major, minor) < (3, 20):
            raise RuntimeError("此项目需要 CMake 3.20 或更新版本。")
    return capabilities


def _generator_for(capabilities: dict, major: int) -> str | None:
    for generator in capabilities["generators"]:
        if not isinstance(generator, dict):
            continue
        name = generator.get("name")
        if not isinstance(name, str) or generator.get("platformSupport") is False:
            continue
        match = re.match(r"^Visual Studio (\d+)(?:\s|$)", name)
        if match is None or int(match.group(1)) != major:
            continue
        # Legacy platform-suffixed aliases should not be combined with -A x64.
        if name.endswith((" Win64", " ARM", " ARM64")):
            continue
        platforms = generator.get("supportedPlatforms")
        if isinstance(platforms, list) and platforms and "x64" not in platforms:
            continue
        return name
    return None


def find_windows_build_tools() -> dict[str, str]:
    """Return ``cmake``, ``generator`` and ``instance`` for a later x64 build.

    The caller should pass the returned generator with ``-G``, the instance as
    ``CMAKE_GENERATOR_INSTANCE``, and explicitly pass ``-A x64``. A compatible
    stable VS installation is preferred over every preview installation.
    """
    if sys.platform != "win32":
        raise RuntimeError("此 DLL 更新工具只能在 Windows 上运行。")
    installations = _find_installations(_find_vswhere())
    candidates = _cmake_candidates(installations)
    checked: dict[str, dict | None] = {}
    failures = []
    # Prefer the latest stable VS even when it needs its bundled CMake instead
    # of an older CMake found on PATH. Reuse capability results across instances.
    for installation in installations:
        for cmake in candidates:
            if cmake not in checked:
                try:
                    checked[cmake] = _capabilities(cmake)
                except RuntimeError as exc:
                    checked[cmake] = None
                    failures.append(str(exc))
            capabilities = checked[cmake]
            if capabilities is None:
                continue
            generator = _generator_for(capabilities, installation.version[0])
            if generator:
                return {"cmake": cmake, "generator": generator, "instance": installation.path}
    if not any(value is not None for value in checked.values()):
        raise RuntimeError(f"没有可用的 CMake。{failures[0] if failures else ''}")
    raise RuntimeError("现有 CMake 不支持已安装的 Visual Studio；请更新 CMake 后重试。")
