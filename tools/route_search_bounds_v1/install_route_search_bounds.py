#!/usr/bin/env python3
"""Build, verify, and install bounded native road search on Windows.

The native bridge and two Python planner modules are updated together.
Tile bytes, the tile index, navigation identity and existing diagnostics stay
unchanged. A new DLL gets a new path; the manifest is switched last. Use
--build-only to keep all candidate work under runtime without activating it.
Only the already-installed Python/CMake/Visual Studio tools are used.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import struct
import subprocess
import sys
import tempfile
import time
import uuid


PACKAGE = Path(__file__).resolve().parent
VERSION = "route-search-bounds-v1"
DEFAULT_ROOT = r"D:\wangzekun\Documents\AutonomousDriving"
BRIDGE = "native/navmesh/bridge.cpp"
CMAKE = "native/navmesh/CMakeLists.txt"
MANIFEST = "data/navigation/manifest.json"
FIXTURE = "native/navmesh/fixture.cpp"
SURFACE_HEADER = "native/navmesh/surface_diagnostics_v2.h"
SURFACE_INCLUDE = "native/navmesh/surface_diagnostics_v2.inc"
DIAGNOSTIC_WRAPPER = "adrive/nav_diagnostics.py"
BASELINE = {
    BRIDGE: "231a50cd0229d2ea7b8f53fdbd59cbcc21f8d54adaf812c99c66d5a8af7bfa03",
    CMAKE: "65db5461b4acfa44ca3c7446c7120ff7f453f4d646666696e570f2babf225342",
    FIXTURE: "1f5d26522bae88e1491cf9ad8cd6654eb0664bb8111cdfd229bcc6e6f37391ae",
    SURFACE_HEADER: "ea33ef4d772cc2a10ae1a0456fd1cfd803bd8e9dc3e89c830c14701c7e233f6e",
    SURFACE_INCLUDE: "72662ba9abea88e18d1d426ffbc62b82a3d74e3cb3e282523a3442c581367392",
    DIAGNOSTIC_WRAPPER: "213b7d40702231657253228d7c51499ea57d528e85ff8660ab19dd8c41d4600d",
    "adrive/navmesh.py": "54f75aea3a68e43935931cc951472a9935c1725579d12f7a40775e3d4581a54e",
    "adrive/road_corridor.py": "ebb294704bddf569ca2035553676920934a7d83e600af3d7c1f7a32b370301b5",
}
PAYLOAD_TARGETS = {
    "path_search_bounds_v3.inc": "native/navmesh/path_search_bounds_v3.inc",
    "road_corridor.py": "adrive/road_corridor.py",
    "navmesh.py": "adrive/navmesh.py",
}
PAYLOAD_NAMES = set(PAYLOAD_TARGETS) | {
    "diagnostic_fixture.cpp", "nav_diagnostics.py",
    "reference-last-plan.json", "regression-assets.json",
}
SUPPORT_NAMES = {
    "install_route_search_bounds.py", "verify_surface_diagnostics.py",
    "verify_route_search_bounds.py", "windows_build.py",
}
FILE_LIMIT = 32 * 1024 * 1024
ANY = object()


def say(message):
    # ASCII status text remains readable in Windows PowerShell 5 and redirected
    # logs regardless of the active console code page.
    print(str(message).encode("ascii", "backslashreplace").decode("ascii"), flush=True)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, ensure_ascii=True, indent=2) + "\n").encode("utf-8")


def normalized_source_sha(data):
    source = data.decode("utf-8-sig").replace("\r\n", "\n").replace("\r", "\n")
    return sha(source.encode("utf-8"))


def beneath(root, relative):
    """Resolve a known project-relative path, rejecting traversal or redirection."""
    relative = str(relative).replace("\\", "/")
    pure = PurePosixPath(relative)
    if (not relative or pure.is_absolute() or ".." in pure.parts
            or any(":" in part for part in pure.parts)):
        raise RuntimeError("Invalid relative path: " + relative)
    path = root.joinpath(*pure.parts)
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        raise RuntimeError("Path escapes its declared root: " + relative)
    return path


def stable_read(path, limit=FILE_LIMIT):
    path = Path(path)
    resolved_before = path.resolve(strict=True)
    path_before = path.stat()
    with path.open("rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
            raise RuntimeError("Not a regular bounded input: " + str(path))
        raw = stream.read(limit + 1)
        after = os.fstat(stream.fileno())
    resolved_after = path.resolve(strict=True)
    path_after = path.stat()
    key = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
    # Windows path-stat and handle-fstat may report different identities/times
    # for the same unchanged file. Compare each API only with its own snapshot.
    if (len(raw) > limit or len(raw) != after.st_size
            or resolved_before != resolved_after
            or key(before) != key(after) or key(path_before) != key(path_after)):
        raise RuntimeError("Input changed during read: " + str(path))
    return raw


def read_json(path):
    result = json.loads(stable_read(path).decode("utf-8-sig"))
    if not isinstance(result, dict):
        raise RuntimeError("Expected a JSON object: " + str(path))
    return result


def atomic_write(path, data, expected=ANY, root=None):
    """Replace one file with an immediate expected-byte comparison."""
    path = Path(path)
    if root is not None:
        path = beneath(root, path.relative_to(root))
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if root is not None:
            beneath(root, path.relative_to(root))
        if expected is None:
            if path.exists() or path.is_symlink():
                raise RuntimeError("Destination appeared before write: " + str(path))
        elif expected is not ANY and stable_read(path) != expected:
            raise RuntimeError("Destination changed before write: " + str(path))
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def create_exact(path, data, root):
    path = beneath(root, path.relative_to(root))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    if stable_read(path) != data:
        raise RuntimeError("Written file did not verify: " + str(path))


def payload_files():
    config = read_json(PACKAGE / "package.json")
    if config.get("schema") != 1 or config.get("version") != VERSION:
        raise RuntimeError("Unsupported package version")
    payload_hashes = config.get("payload_sha256")
    support_hashes = config.get("support_sha256")
    if not isinstance(payload_hashes, dict) or set(payload_hashes) != PAYLOAD_NAMES:
        raise RuntimeError("Package payload inventory is incomplete")
    if not isinstance(support_hashes, dict) or set(support_hashes) != SUPPORT_NAMES:
        raise RuntimeError("Package support inventory is incomplete")
    result = {}
    for name, expected in payload_hashes.items():
        data = stable_read(PACKAGE / "payload" / name)
        if sha(data) != expected:
            raise RuntimeError("Package payload hash mismatch: " + name)
        result[name] = data
    for name, expected in support_hashes.items():
        if sha(stable_read(PACKAGE / name)) != expected:
            raise RuntimeError("Package support hash mismatch: " + name)
    return result, config


def asset(nav, entry):
    if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
        raise RuntimeError("Invalid manifest library entry")
    path = beneath(nav, entry["path"])
    data = stable_read(path)
    if (type(entry.get("bytes")) is not int or len(data) != entry["bytes"]
            or sha(data) != entry.get("sha256")):
        raise RuntimeError("Manifest library bytes/SHA mismatch: " + str(path))
    return path, data


def tile_identity(raw):
    return sha(json.dumps(raw["tiles"], sort_keys=True).encode())[:16]


def preflight(root, payload):
    if not root.is_dir():
        raise RuntimeError("Project directory does not exist: " + str(root))
    snapshots = {}
    for name in [*BASELINE, MANIFEST]:
        snapshots[name] = stable_read(beneath(root, name))
    for name, expected in BASELINE.items():
        if normalized_source_sha(snapshots[name]) != expected:
            raise RuntimeError("Source baseline differs; file was not overwritten: " + name)
    new_include = PAYLOAD_TARGETS["path_search_bounds_v3.inc"]
    destination = beneath(root, new_include)
    if destination.exists() or destination.is_symlink():
        raise RuntimeError("Bounds include already exists; package was not reapplied: " + new_include)
    snapshots[new_include] = None
    # Snapshot the whole small Python source package so the verification child
    # imports a coherent candidate, with only the two approved modules changed.
    python_files = {}
    for path in sorted(beneath(root, "adrive").rglob("*.py")):
        if path.is_symlink():
            raise RuntimeError("Symlink in planner source tree: " + str(path))
        relative = path.relative_to(root).as_posix()
        data = stable_read(beneath(root, relative))
        if relative in snapshots and snapshots[relative] != data:
            raise RuntimeError("Planner source changed during snapshot: " + relative)
        snapshots[relative] = data
        python_files[relative] = data
    if "adrive/__init__.py" not in python_files or "adrive/models.py" not in python_files:
        raise RuntimeError("Planner Python source package is incomplete")
    raw = json.loads(snapshots[MANIFEST].decode("utf-8-sig"))
    if (not isinstance(raw, dict) or raw.get("schema") != 1
            or raw.get("kind") != "adrive-ground-navmesh" or raw.get("mmap_version") != 16):
        raise RuntimeError("Unsupported navigation manifest")
    if not isinstance(raw.get("tiles"), list) or not raw["tiles"]:
        raise RuntimeError("Manifest tile inventory is missing")
    if tile_identity(raw) != raw.get("identity"):
        raise RuntimeError("Manifest identity does not match its indexed tiles")
    nav = beneath(root, "data/navigation").resolve()
    dll, dll_data = asset(nav, raw.get("library"))
    snapshots[dll.relative_to(root).as_posix()] = dll_data

    reference = json.loads(payload["reference-last-plan.json"].decode("utf-8-sig"))
    regression = json.loads(payload["regression-assets.json"].decode("utf-8-sig"))
    if (not isinstance(reference, dict) or reference.get("schema") != 1
            or reference.get("result", {}).get("status") != "error"):
        raise RuntimeError("Packaged reference must be the recorded failed route")
    if (not isinstance(regression, dict) or regression.get("schema") != 1
            or regression.get("reference_plan_sha256") != sha(payload["reference-last-plan.json"])):
        raise RuntimeError("Packaged regression case identity is invalid")
    if reference.get("navigation_identity") != raw["identity"] or regression.get("navigation_identity") != raw["identity"]:
        raise RuntimeError("Regression navigation identity differs from the installed data")
    tile_index = {(entry["c"], entry["row"], entry["col"]): entry for entry in raw["tiles"]}
    map_index = {entry["c"]: entry for entry in raw["maps"]}
    maps, tiles = regression.get("maps"), regression.get("tiles")
    if (not isinstance(maps, list) or not maps or not isinstance(tiles, list) or not tiles
            or len(tiles) > 128):
        raise RuntimeError("Packaged regression asset inventory is incomplete")
    seen = set()
    regression_assets = []
    for is_map, entries in [(True, maps), (False, tiles)]:
        for entry in entries:
            key = (entry["c"],) if is_map else (entry["c"], entry["row"], entry["col"])
            token = (is_map, key)
            if token in seen:
                raise RuntimeError("Duplicate regression asset key")
            seen.add(token)
            actual = map_index.get(key[0]) if is_map else tile_index.get(key)
            if actual != entry:
                raise RuntimeError("Regression asset no longer matches the manifest: " + str(key))
            path, data = asset(nav, entry)
            snapshots[path.relative_to(root).as_posix()] = data
            regression_assets.append(entry)
    third_party = beneath(root, "third_party/detour")
    if not third_party.is_dir():
        raise RuntimeError("Project Detour source directory is missing")
    detour_files = {}
    for path in sorted(third_party.rglob("*")):
        if path.is_symlink():
            raise RuntimeError("Symlink in Detour source tree: " + str(path))
        if path.is_file():
            relative = path.relative_to(root).as_posix()
            data = stable_read(beneath(root, relative))
            snapshots[relative] = data
            detour_files[relative] = data
    if ("third_party/detour/Include/DetourNavMesh.h" not in detour_files
            or not any(name.startswith("third_party/detour/Source/") and name.endswith(".cpp") for name in detour_files)):
        raise RuntimeError("Project Detour source inventory is incomplete")
    original = snapshots[BRIDGE]
    newline = b"\r\n" if b"\r\n" in original else b"\n"
    bridge_after = original.rstrip(b"\r\n") + newline * 2 + b'#include "path_search_bounds_v3.inc"' + newline
    # Included native code first, dependent planner module second, manifest last.
    # Existing diagnostic source files are checked and copied to the candidate,
    # but are never installed again or altered by this release.
    updates = {new_include: payload["path_search_bounds_v3.inc"], BRIDGE: bridge_after,
               "adrive/road_corridor.py": payload["road_corridor.py"],
               "adrive/navmesh.py": payload["navmesh.py"]}
    return {"raw": raw, "snapshots": snapshots, "dll": dll, "dll_data": dll_data,
            "detour_files": detour_files, "python_files": python_files,
            "updates": updates, "nav": nav, "regression_assets": regression_assets,
            "reference_files": {name: payload[name] for name in
                                ("reference-last-plan.json", "regression-assets.json")}}


def recheck(root, snapshots):
    for relative, expected in snapshots.items():
        path = beneath(root, relative)
        if expected is None:
            if path.exists() or path.is_symlink():
                raise RuntimeError("Destination appeared during operation: " + relative)
        elif stable_read(path) != expected:
            raise RuntimeError("Input changed during operation: " + relative)


def recheck_candidate(work, inputs):
    """Bind tested candidate modules and native sources to installed bytes."""
    expected = {**inputs["detour_files"], **inputs["python_files"],
                **{name: inputs["snapshots"][name] for name in
                   (SURFACE_HEADER, SURFACE_INCLUDE, FIXTURE)},
                **inputs["updates"]}
    for relative, content in expected.items():
        if stable_read(work / "candidate" / relative) != content:
            raise RuntimeError("Candidate source changed: " + relative)
    for name, content in inputs["reference_files"].items():
        if stable_read(work / "reference" / name) != content:
            raise RuntimeError("Packaged reference changed after staging: " + name)


def snapshot_verified_assets(root, inputs, verification):
    """Include expanded-radius assets actually used by the verification worker."""
    entries = verification["worker_report"].get("verified_assets")
    if not isinstance(entries, list) or not entries:
        raise RuntimeError("Route worker omitted its verified asset inventory")
    indexed = {entry["path"]: entry for entry in
               inputs["raw"]["maps"] + inputs["raw"]["tiles"]}
    for entry in entries:
        original = indexed.get(entry.get("path")) if isinstance(entry, dict) else None
        if original is None or any(entry.get(key) != original[key] for key in ("path", "bytes", "sha256")):
            raise RuntimeError("Route worker asset is not bound to the original manifest")
        path, data = asset(inputs["nav"], original)
        relative = path.relative_to(root).as_posix()
        if relative in inputs["snapshots"] and inputs["snapshots"][relative] != data:
            raise RuntimeError("Route asset changed since its starting snapshot: " + relative)
        inputs["snapshots"][relative] = data


def run_logged(command, log, cwd, label, timeout):
    say(label + ": starting; log=" + str(log))
    with log.open("ab") as output:
        output.write(("\nCOMMAND " + json.dumps(command, ensure_ascii=True) + "\n").encode())
        output.flush()
        started = time.monotonic()
        with subprocess.Popen(command, cwd=str(cwd), stdout=output, stderr=subprocess.STDOUT) as process:
            while True:
                try:
                    rc = process.wait(timeout=10)
                    break
                except subprocess.TimeoutExpired:
                    elapsed = time.monotonic() - started
                    if elapsed > timeout:
                        # This is our own build/verification child, never a game
                        # or an already-running navigation process.
                        process.kill()
                        process.wait()
                        raise RuntimeError(label + " timed out; see " + str(log))
                    say(label + ": still running (" + str(int(elapsed)) + " seconds)")
    if rc:
        tail = "\n".join(log.read_text(encoding="utf-8", errors="replace").splitlines()[-20:])
        say(tail)
        raise RuntimeError(label + " failed with exit code " + str(rc) + "; see " + str(log))
    say(label + ": passed")


def stage_and_build(root, work, inputs, payload, build_tools, report):
    stage = work / "candidate"
    native = stage / "native/navmesh"
    native.mkdir(parents=True)
    for relative, data in {**inputs["detour_files"], **inputs["python_files"]}.items():
        target = stage / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    for relative in (SURFACE_HEADER, SURFACE_INCLUDE):
        target = stage / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(inputs["snapshots"][relative])
    for relative, data in inputs["updates"].items():
        target = stage / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    (native / "fixture.cpp").write_bytes(inputs["snapshots"][FIXTURE])
    (native / "diagnostic_fixture.cpp").write_bytes(payload["diagnostic_fixture.cpp"])
    reference = work / "reference"
    reference.mkdir()
    for name in ("reference-last-plan.json", "regression-assets.json"):
        create_exact(reference / name, payload[name], root)
    current_plan = beneath(root, "runtime/route-quality/last-plan.json")
    if current_plan.exists():
        try:
            current_plan_data = stable_read(current_plan)
            create_exact(reference / "current-last-plan-at-install.json", current_plan_data, root)
            report["current_plan_snapshot"] = {"sha256": sha(current_plan_data),
                "same_as_packaged_reference": current_plan_data == payload["reference-last-plan.json"],
                "used_as_install_gate": False}
        except (OSError, RuntimeError) as error:
            report["current_plan_snapshot"] = {"unavailable": str(error), "used_as_install_gate": False}
    cmake = inputs["snapshots"][CMAKE].decode("utf-8-sig")
    cmake += """
# Isolated verification target; the production CMake file is unchanged.
target_compile_definitions(adrive_navmesh PRIVATE DT_POLYREF64=1)
target_compile_definitions(navmesh_fixture PRIVATE DT_POLYREF64=1)
add_executable(diagnostic_fixture diagnostic_fixture.cpp ${DETOUR_SOURCES})
target_include_directories(diagnostic_fixture PRIVATE ../../third_party/detour/Include)
target_compile_definitions(diagnostic_fixture PRIVATE DT_VIRTUAL_QUERYFILTER=1 DT_POLYREF64=1)
"""
    (native / "CMakeLists.txt").write_text(cmake, encoding="utf-8")
    build = work / "build"
    artifacts = build / "artifacts"
    configure = [build_tools["cmake"], "-S", str(native), "-B", str(build),
                 "-G", build_tools["generator"], "-A", "x64",
                 "-DCMAKE_GENERATOR_INSTANCE:PATH=" + build_tools["instance"],
                 "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE:PATH=" + str(artifacts)]
    compile_command = [build_tools["cmake"], "--build", str(build), "--config", "Release",
                       "--target", "adrive_navmesh", "diagnostic_fixture",
                       "--parallel", str(min(4, os.cpu_count() or 2))]
    log = work / "build.log"
    report["build"] = {"toolchain": build_tools, "configure": configure, "compile": compile_command,
                       "production_cmake_modified": False,
                       "stage_bridge_sha256": sha(inputs["updates"][BRIDGE]),
                       "stage_cmake_sha256": sha(cmake.encode("utf-8")),
                       "candidate_python_directory": str(stage),
                       "reference_plan_sha256": sha(payload["reference-last-plan.json"]),
                       "regression_assets_sha256": sha(payload["regression-assets.json"])}
    run_logged(configure, log, root, "CONFIGURE", 180)
    run_logged(compile_command, log, root, "BUILD", 1200)
    dll = artifacts / "adrive_navmesh.dll"
    generator = artifacts / "diagnostic_fixture.exe"
    if not dll.is_file() or not generator.is_file():
        raise RuntimeError("Expected DLL or diagnostic fixture executable was not built")
    fixture_dir = work / "fixtures"
    run_logged([str(generator), str(fixture_dir)], log, root, "FIXTURE", 120)
    return dll, fixture_dir


def pe_x64(data):
    if len(data) < 64 or data[:2] != b"MZ":
        return False
    offset = struct.unpack_from("<I", data, 0x3C)[0]
    return (offset + 6 <= len(data) and data[offset:offset + 4] == b"PE\x00\x00"
            and struct.unpack_from("<H", data, offset + 4)[0] == 0x8664)


def verify(root, work, dll, fixture_dir, baseline, suffix):
    before = stable_read(dll)
    if not pe_x64(before):
        raise RuntimeError("Candidate DLL is not an AMD64 Windows PE file")
    legacy_path = work / ("verification-legacy-" + suffix + ".json")
    legacy_command = [sys.executable, str(PACKAGE / "verify_surface_diagnostics.py"),
                      "--dll", str(dll.resolve()), "--fixture-dir", str(fixture_dir.resolve()),
                      "--baseline-dll", str(baseline.resolve()), "--report", str(legacy_path)]
    run_logged(legacy_command, work / "build.log", root, "VERIFY_LEGACY_" + suffix.upper(), 300)
    legacy = read_json(legacy_path)
    if legacy.get("status") != "passed" or not isinstance(legacy.get("capabilities"), dict):
        raise RuntimeError("Legacy verification did not return a passed capability report")
    result_path = work / ("verification-bounds-" + suffix + ".json")
    command = [sys.executable, str(PACKAGE / "verify_route_search_bounds.py"),
               "--dll", str(dll.resolve()), "--baseline-dll", str(baseline.resolve()),
               "--fixture-dir", str(fixture_dir.resolve()), "--project-root", str(root),
               "--candidate-python", str((work / "candidate").resolve()),
               "--plan", str((work / "reference/reference-last-plan.json").resolve()),
               "--regression-assets", str((work / "reference/regression-assets.json").resolve()),
               "--report", str(result_path)]
    run_logged(command, work / "build.log", root, "VERIFY_BOUNDS_AND_RECORDED_ROUTE_" + suffix.upper(), 600)
    result = read_json(result_path)
    if result.get("status") != "passed":
        raise RuntimeError("Bounds and recorded-route worker did not return passed")
    if stable_read(dll) != before:
        raise RuntimeError("Candidate DLL changed during verification")
    say("VERIFIED_LEGACY_ABI = %s/%s checks passed" %
        (legacy.get("passed_check_count", "?"), legacy.get("total_check_count", "?")))
    say("VERIFIED_BOUNDED_SEARCH = %d groups passed" % len(result.get("native_checks", [])))
    say("VERIFIED_RECORDED_ROUTE = passed")
    # Keep the worker's concrete assertions and route outcome visible without
    # interpreting any changed failure code as a repaired bridge or route.
    if isinstance(result.get("summary"), dict):
        say("RECORDED_ROUTE_RESULT = " + json.dumps(result["summary"], ensure_ascii=True, sort_keys=True))
    say("VERIFICATION_REPORT = " + str(result_path))
    return {"dll_path": str(dll.resolve()), "dll_bytes": len(before),
            "dll_sha256": sha(before), "legacy_command": legacy_command,
            "legacy_worker_report": legacy, "command": command,
            "worker_report": result, "capabilities": legacy["capabilities"]}


def proposed_manifest(raw, nav, dll, dll_data):
    result = copy.deepcopy(raw)
    result["library"].update(path=dll.relative_to(nav).as_posix(), bytes=len(dll_data), sha256=sha(dll_data))
    for key, value in raw.items():
        if key != "library" and result.get(key) != value:
            raise RuntimeError("Unexpected manifest mutation outside library")
    if result["tiles"] != raw["tiles"] or result["identity"] != raw["identity"]:
        raise RuntimeError("Bounded-search update attempted to change navigation identity")
    return result


def save_report(root, path, report):
    atomic_write(path, encoded(report), root=root)


def install(root, work, inputs, dll, fixture_dir, report, report_path):
    recheck(root, inputs["snapshots"])
    recheck_candidate(work, inputs)
    dll_data = stable_read(dll)
    if sha(dll_data) != report["candidate_verification"]["dll_sha256"]:
        raise RuntimeError("Built DLL changed after candidate verification")
    # Backups are byte-exact, including a copy of the manifest-selected old DLL.
    backup = work / "backup"
    backup.mkdir()
    backup_index = {}
    for relative in [*BASELINE, MANIFEST]:
        data = inputs["snapshots"][relative]
        create_exact(backup / relative, data, root)
        backup_index[relative] = {"bytes": len(data), "sha256": sha(data)}
    create_exact(backup / "loaded-before.dll", inputs["dll_data"], root)
    backup_index["loaded-before.dll"] = {"source": str(inputs["dll"]),
                                        "bytes": len(inputs["dll_data"]), "sha256": sha(inputs["dll_data"])}
    report["backup"] = {"directory": str(backup), "files": backup_index}
    version_dir = beneath(root, "data/navigation/route-search-bounds/" + work.name)
    if version_dir.exists():
        raise RuntimeError("Immutable bounded-search version directory already exists")
    version_dir.mkdir(parents=True)
    installed_dll = version_dir / "adrive_navmesh.dll"
    create_exact(installed_dll, dll_data, root)
    report["installed_path_verification"] = verify(root, work, installed_dll, fixture_dir, inputs["dll"], "installed_path")
    snapshot_verified_assets(root, inputs, report["installed_path_verification"])
    recheck_candidate(work, inputs)
    recheck(root, inputs["snapshots"])
    if stable_read(installed_dll) != dll_data:
        raise RuntimeError("Installed candidate changed before manifest activation")
    proposed = proposed_manifest(inputs["raw"], inputs["nav"], installed_dll, dll_data)
    after_manifest = encoded(proposed)
    create_exact(work / "manifest.after.json", after_manifest, root)
    report.update(previous_library=inputs["raw"]["library"], new_library=proposed["library"],
                  identity_before=inputs["raw"]["identity"], identity_after=proposed["identity"],
                  tiles_changed=False, blocked_edge_identity_preserved=True,
                  manifest_before_sha256=sha(inputs["snapshots"][MANIFEST]),
                  manifest_after_sha256=sha(after_manifest), transaction=[])
    save_report(root, report_path, report)
    operations = [(relative, inputs["snapshots"].get(relative), content)
                  for relative, content in inputs["updates"].items()]
    # Only this final pointer update changes the DLL loaded by a new planner.
    operations.append((MANIFEST, inputs["snapshots"][MANIFEST], after_manifest))
    current = dict(inputs["snapshots"])
    attempted = []
    try:
        for relative, before, after in operations:
            recheck(root, current)
            event = {"path": relative, "before_sha256": sha(before) if before is not None else None,
                     "after_sha256": sha(after), "status": "attempting"}
            attempted.append((relative, before, after, event))
            report["transaction"].append(event)
            save_report(root, report_path, report)
            path = beneath(root, relative)
            atomic_write(path, after, expected=before, root=root)
            if stable_read(path) != after:
                raise RuntimeError("Post-write verification failed: " + relative)
            current[relative] = after
            event["status"] = "written"
            save_report(root, report_path, report)
        recheck(root, current)
        report["status"] = "installed"
        report["production_planner_sources_modified"] = True
        report["manifest_selected_dll_after_update"] = {"path": str(installed_dll.resolve()),
                                                       "bytes": len(dll_data), "sha256": sha(dll_data)}
        report["capabilities"] = report["installed_path_verification"]["capabilities"]
        save_report(root, report_path, report)
    except BaseException as error:
        report["transaction_error"] = type(error).__name__ + ": " + str(error)
        report["recovery"] = []
        for relative, before, after, event in reversed(attempted):
            item = {"path": relative}
            try:
                path = beneath(root, relative)
                observed = stable_read(path) if path.exists() else None
                if observed == before:
                    item["status"] = "already_at_starting_bytes"
                elif observed != after:
                    raise RuntimeError("File changed again; automatic rollback will not overwrite it")
                elif before is None:
                    # The comparison is repeated immediately before removing
                    # only this operation's new file.
                    if stable_read(path) != after:
                        raise RuntimeError("New file changed during rollback")
                    path.unlink()
                    item["status"] = "removed_our_new_file"
                else:
                    atomic_write(path, before, expected=after, root=root)
                    if stable_read(path) != before:
                        raise RuntimeError("Restored bytes did not verify")
                    item["status"] = "restored_starting_bytes"
            except BaseException as recovery_error:
                item.update(status="recovery_failed", error=type(recovery_error).__name__ + ": " + str(recovery_error))
            report["recovery"].append(item)
        report["status"] = "failed_partial" if any(x["status"] == "recovery_failed" for x in report["recovery"]) else "failed_restored"
        try:
            report["production_planner_sources_modified"] = any(
                stable_read(beneath(root, name)) != inputs["snapshots"][name]
                for name in ("adrive/navmesh.py", "adrive/road_corridor.py"))
        except (OSError, RuntimeError):
            report["production_planner_sources_modified"] = None
        # The new, unreferenced DLL is retained as an immutable build
        # artifact. Never delete the previous DLL or touch a running process.
        try:
            save_report(root, report_path, report)
        except Exception as report_error:
            say("REPORT_WRITE_FAILED: " + str(report_error))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(DEFAULT_ROOT))
    parser.add_argument("--build-only", action="store_true", help="Build and verify without changing active source files or manifest.")
    args = parser.parse_args(argv)
    if os.name != "nt" or struct.calcsize("P") != 8:
        raise RuntimeError("Run this installer with the project's 64-bit Windows Python, not WSL.")
    root = args.project_root.resolve()
    payload, config = payload_files()
    say("ROUTE_SEARCH_BOUNDS_V1: checking project and package")
    inputs = preflight(root, payload)
    # Import only after the bundled support-file integrity check.
    from windows_build import find_windows_build_tools
    build_tools = find_windows_build_tools()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    work = beneath(root, "runtime/route-search-bounds-v1/" + run_id)
    work.mkdir(parents=True)
    report_path = work / "report.json"
    report = {"schema": 1, "version": VERSION, "run_id": run_id, "project_root": str(root),
              "created_at_utc": datetime.now(timezone.utc).isoformat(), "build_only": args.build_only,
              "status": "preparing", "production_planner_sources_modified": False,
              "tile_bytes_modified": False, "swimming_enabled": False,
              "loaded_dll_before": {"path": str(inputs["dll"].resolve()), "bytes": len(inputs["dll_data"]),
                                    "sha256": sha(inputs["dll_data"])},
              "input_inventory": {name: {"bytes": len(data), "sha256": sha(data)} if data is not None else {"expected_absent": True}
                                  for name, data in inputs["snapshots"].items()},
              "package_inventory": config}
    save_report(root, report_path, report)
    say("REPORT = " + str(report_path))
    say("BASELINE_DLL = " + str(inputs["dll"]))
    try:
        dll, fixture_dir = stage_and_build(root, work, inputs, payload, build_tools, report)
        recheck_candidate(work, inputs)
        report["candidate_verification"] = verify(root, work, dll, fixture_dir, inputs["dll"], "candidate")
        snapshot_verified_assets(root, inputs, report["candidate_verification"])
        recheck_candidate(work, inputs)
        recheck(root, inputs["snapshots"])
        if args.build_only:
            report["status"] = "verified_build_only"
            report["capabilities"] = report["candidate_verification"]["capabilities"]
            save_report(root, report_path, report)
            say("BUILD_ONLY_VERIFIED")
            say("CANDIDATE_DLL = " + str(dll))
        else:
            say("INSTALL: verified; saving exact backups and switching manifest last")
            install(root, work, inputs, dll, fixture_dir, report, report_path)
            say("ROUTE_SEARCH_BOUNDS_INSTALLED")
            say("NEW_DLL = " + report["manifest_selected_dll_after_update"]["path"])
            say("NAVIGATION_IDENTITY_UNCHANGED = " + report["identity_after"])
            say("Restart AutonomousDriving to load the new planner and DLL. No running process was stopped.")
            say("This changes search bounds and failure handling; it does not repair tile geometry or enable swimming.")
        say("REPORT = " + str(report_path))
        return 0
    except BaseException as error:
        if report.get("status") not in {"failed_partial", "failed_restored"}:
            report["status"] = "failed_before_activation"
        report["error"] = type(error).__name__ + ": " + str(error)
        try:
            save_report(root, report_path, report)
        except Exception as report_error:
            say("REPORT_WRITE_FAILED: " + str(report_error))
        say("UPDATE_FAILED: " + str(error))
        say("REPORT = " + str(report_path))
        if report["status"] == "failed_partial":
            say("PARTIAL_UPDATE: inspect the exact backup and recovery report before restarting.")
        return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        say("UPDATE_NOT_STARTED: " + str(error))
        raise SystemExit(1)
