#!/usr/bin/env python3
"""Regression gate for the AutonomousDriving ground-navigation pack.

Default behavior:
- validates the active data/navigation/manifest.json
- verifies the complete pack can be loaded through the native DLL
- checks required bounded-search ABI support
- runs two known bridge regressions in both directions
- runs two long-route regressions
- runs the recorded road-loop regression in both directions
- runs the recorded Westfall topology/quality regression
- writes a JSON report under runtime/navigation-regressions

This tool is read-only with respect to the active navigation pack and does not
control the game. It disables route-quality snapshot writes while probing.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import traceback
import uuid


def point_dict(point):
    return {"c": point.c, "x": point.x, "y": point.y, "z": point.z}


def route_length_3d(route):
    return sum(
        math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        for a, b in zip(route.points, route.points[1:])
    )


KNOWN_CASES = (
    {
        "name": "kalimdor_bridge_A_to_B",
        "continent": 1,
        "start": (-2820.714451053201, -3396.102114364935, None),
        "goal": (-2834.214451053201, -3348.352114364935, None),
        "guided": True,
        "xy_range": (47.0, 52.0),
        "length_3d_range": (48.0, 52.0),
        "max_expanded": 500000,
    },
    {
        "name": "kalimdor_bridge_B_to_A",
        "continent": 1,
        "start": (-2834.214451053201, -3348.352114364935, None),
        "goal": (-2820.714451053201, -3396.102114364935, None),
        "guided": True,
        "xy_range": (47.0, 52.0),
        "length_3d_range": (48.0, 52.0),
        "max_expanded": 500000,
    },
    {
        "name": "kalimdor_direct_A_to_B",
        "continent": 1,
        "start": (-2820.714451053201, -3396.102114364935, None),
        "goal": (-2834.214451053201, -3348.352114364935, None),
        "guided": False,
        "xy_range": (47.0, 52.0),
        "length_3d_range": (48.0, 52.0),
        "max_expanded": 500000,
    },
    {
        "name": "kalimdor_direct_B_to_A",
        "continent": 1,
        "start": (-2834.214451053201, -3348.352114364935, None),
        "goal": (-2820.714451053201, -3396.102114364935, None),
        "guided": False,
        "xy_range": (47.0, 52.0),
        "length_3d_range": (48.0, 52.0),
        "max_expanded": 500000,
    },
    {
        "name": "ek_bridge_A_to_B",
        "continent": 0,
        "start": (-10065.478503779887, -1507.212281364077, 28.680936813354492),
        "goal": (-10079.181030588508, -1468.9011349807895, 28.680936813354492),
        "guided": True,
        "xy_range": (39.0, 43.0),
        "length_3d_range": (40.0, 45.0),
        "max_expanded": 500000,
    },
    {
        "name": "ek_bridge_B_to_A",
        "continent": 0,
        "start": (-10079.181030588508, -1468.9011349807895, 28.680936813354492),
        "goal": (-10065.478503779887, -1507.212281364077, 28.680936813354492),
        "guided": True,
        "xy_range": (39.0, 43.0),
        "length_3d_range": (40.0, 45.0),
        "max_expanded": 500000,
    },
    {
        "name": "known_long_route",
        "continent": 0,
        "start": (-9486.727746196266, 66.20478324520349, 56.24121856689453),
        "goal": (-10550.815788443762, -1196.5663646349744, 27.7370548248291),
        "guided": True,
        "xy_range": (3400.0, 3600.0),
        "length_3d_range": (3400.0, 3650.0),
        "max_expanded": 500000,
    },
    {
        "name": "known_long_reverse",
        "continent": 0,
        "start": (-10550.815788443762, -1196.5663646349744, 27.7370548248291),
        "goal": (-9486.727746196266, 66.20478324520349, 56.24121856689453),
        "guided": True,
        "xy_range": (3400.0, 3600.0),
        "length_3d_range": (3400.0, 3650.0),
        "max_expanded": 500000,
    },
)

WESTFALL = {
    "name": "westfall_recorded_route",
    "continent": 0,
    "start": (-9488.56774119409, 81.83808086998738, None),
    "goal": (-11046.225769178085, 1514.7143896288444, None),
    "max_expanded": 500000,
    "max_route_guide_ratio": 1.10,
    "max_fallback_count": 0,
    "max_route_xy": 3000.0,
}

ROAD_LOOP_CASES = (
    {
        "name": "road_loop_A_to_B",
        "continent": 0,
        "start": (-9937.311572232578, 1001.1050237878376, None),
        "goal": (-10641.868693563683, 1017.8745550378376, None),
        "allow_candidates": False,
        "max_expanded": 500000,
        "max_route_xy": 1000.0,
        "max_route_guide_ratio": 0.8,
    },
    {
        "name": "road_loop_B_to_A",
        "continent": 0,
        "start": (-10641.868693563683, 1017.8745550378376, None),
        "goal": (-9937.311572232578, 1001.1050237878376, None),
        "allow_candidates": False,
        "max_expanded": 500000,
        "max_route_xy": 1000.0,
        "max_route_guide_ratio": 0.8,
    },
    {
        "name": "road_loop_candidates_A_to_B",
        "continent": 0,
        "start": (-9937.311572232578, 1001.1050237878376, None),
        "goal": (-10641.868693563683, 1017.8745550378376, None),
        "allow_candidates": True,
        "max_expanded": 500000,
        "max_route_xy": 1000.0,
        "max_route_guide_ratio": 0.8,
    },
    {
        "name": "road_loop_candidates_B_to_A",
        "continent": 0,
        "start": (-10641.868693563683, 1017.8745550378376, None),
        "goal": (-9937.311572232578, 1001.1050237878376, None),
        "allow_candidates": True,
        "max_expanded": 500000,
        "max_route_xy": 1000.0,
        "max_route_guide_ratio": 0.8,
    },
)


def check_direct_selection(pack, route, guide, budget):
    selection = (pack.last_route_diagnostics or {}).get("route_selection", {})
    if selection.get("triggered") is not True or selection.get("selected") != "direct":
        raise AssertionError(f"shorter complete walk was not selected: {selection}")
    if selection.get("direct_status") != "complete":
        raise AssertionError("direct comparison did not establish a complete route")
    if selection.get("search_scope") != "loaded_walk_graph_with_conservative_length_domain":
        raise AssertionError("direct comparison did not use the bounded native search domain")
    limit = selection.get("search_length_limit_xy")
    guided_length = selection.get("guided_length_xy")
    if not isinstance(limit, (int, float)) or not isinstance(guided_length, (int, float)):
        raise AssertionError("direct comparison omitted its native length bound")
    if not math.isclose(limit, guided_length * 0.8, rel_tol=1e-10, abs_tol=1e-6):
        raise AssertionError("direct comparison length bound differs from 80% of guided route")
    if not math.isclose(selection.get("guide_length_xy", math.inf), guide.distance,
                        rel_tol=1e-10, abs_tol=1e-6):
        raise AssertionError("direct comparison does not describe the actual road guide")
    if not math.isclose(selection.get("direct_length_xy", math.inf), route.distance,
                        rel_tol=1e-10, abs_tol=1e-6):
        raise AssertionError("selected route length differs from the direct candidate")
    guided_expanded = selection.get("guided_expanded")
    direct_expanded = selection.get("direct_expanded")
    if not all(isinstance(value, int) and value >= 0
               for value in (guided_expanded, direct_expanded)):
        raise AssertionError("comparison omitted guided/direct expansion accounting")
    total = guided_expanded + direct_expanded
    if total != selection.get("expanded_total") or total != route.expanded or total > budget:
        raise AssertionError(f"shared search budget accounting is inconsistent: {selection}")
    if selection.get("passed_budget") != budget - guided_expanded:
        raise AssertionError("direct comparison did not receive only the remaining shared budget")
    return selection


def run_road_loop(pack, coarse, Point, case):
    start = Point(case["continent"], *case["start"])
    goal = Point(case["continent"], *case["goal"])
    guide = coarse._plan_coarse(start, goal, allow_candidates=case["allow_candidates"],
                                blocked_edges=frozenset())
    route = pack.plan(start, goal, guide=guide, blocked_edges=frozenset(),
                      max_expansions=case["max_expanded"])
    selection = check_direct_selection(pack, route, guide, case["max_expanded"])
    ratio = route.distance / guide.distance if guide.distance else math.inf
    if route.distance >= case["max_route_xy"] or ratio >= case["max_route_guide_ratio"]:
        raise AssertionError(f"road loop retained: route_xy={route.distance}, route/guide={ratio}")
    for requested, resolved in ((start, route.start), (goal, route.goal)):
        if (requested.c, requested.x, requested.y) != (resolved.c, resolved.x, resolved.y):
            raise AssertionError("road-loop route moved an endpoint horizontally")
    return {
        "name": case["name"], "status": "passed",
        "allow_candidates": case["allow_candidates"],
        "start": point_dict(start), "goal": point_dict(goal),
        "straight_xy": start.distance(goal), "guide_xy": guide.distance,
        "route_xy": route.distance, "route_guide_ratio": ratio,
        "point_count": len(route.points), "expanded": route.expanded,
        "route_selection": selection,
        "resolved_start": point_dict(route.start), "resolved_goal": point_dict(route.goal),
    }


def check_range(label, value, bounds):
    low, high = bounds
    if not low <= value <= high:
        raise AssertionError(
            f"{label}={value:.6f} outside accepted range [{low}, {high}]"
        )


def run_known_case(pack, coarse, Point, case):
    start = Point(case["continent"], *case["start"])
    goal = Point(case["continent"], *case["goal"])
    guide = (
        coarse._plan_coarse(
            start,
            goal,
            allow_candidates=False,
            blocked_edges=frozenset(),
        )
        if case["guided"]
        else None
    )

    route = pack.plan(
        start,
        goal,
        guide=guide,
        blocked_edges=frozenset(),
        max_expansions=case["max_expanded"],
    )
    length_3d = route_length_3d(route)

    check_range("route_xy", route.distance, case["xy_range"])
    check_range("route_3d", length_3d, case["length_3d_range"])
    if not 0 <= route.expanded <= case["max_expanded"]:
        raise AssertionError(f"expanded={route.expanded} exceeds gate")

    result = {
        "name": case["name"],
        "status": "passed",
        "guided": case["guided"],
        "route_xy": route.distance,
        "route_3d": length_3d,
        "point_count": len(route.points),
        "expanded": route.expanded,
        "resolved_start": point_dict(route.start),
        "resolved_goal": point_dict(route.goal),
    }
    return result


def run_westfall(pack, coarse, Point, detour_metrics):
    start = Point(WESTFALL["continent"], *WESTFALL["start"])
    goal = Point(WESTFALL["continent"], *WESTFALL["goal"])

    guide = coarse._plan_coarse(
        start,
        goal,
        allow_candidates=False,
        blocked_edges=frozenset(),
    )
    route = pack.plan(
        start,
        goal,
        guide=guide,
        blocked_edges=frozenset(),
        max_expansions=WESTFALL["max_expanded"],
    )

    metrics = detour_metrics(route.points, guide.points)
    selection = check_direct_selection(pack, route, guide, WESTFALL["max_expanded"])
    ratio = route.distance / guide.distance if guide.distance else math.inf
    trace = (pack.last_route_diagnostics or {}).get("road_trace", [])
    fallback_modes = {
        "remaining_native_fallback",
        "bounded_remaining_fallback",
        "remaining_native_fallback_quality",
    }
    fallback = [row for row in trace if row.get("mode") in fallback_modes]

    if metrics.get("excessive_detour"):
        raise AssertionError("Westfall route was classified as excessive_detour")
    if route.distance >= WESTFALL["max_route_xy"]:
        raise AssertionError(f"Westfall route_xy={route.distance} did not improve below 3000 yards")
    if ratio > WESTFALL["max_route_guide_ratio"]:
        raise AssertionError(
            f"Westfall route/guide ratio {ratio:.6f} exceeds "
            f"{WESTFALL['max_route_guide_ratio']:.6f}"
        )
    if len(fallback) > WESTFALL["max_fallback_count"]:
        raise AssertionError(
            f"Westfall fallback_count={len(fallback)} exceeds "
            f"{WESTFALL['max_fallback_count']}"
        )
    if not 0 <= route.expanded <= WESTFALL["max_expanded"]:
        raise AssertionError(f"Westfall expanded={route.expanded} exceeds gate")

    return {
        "name": WESTFALL["name"],
        "status": "passed",
        "straight_xy": start.distance(goal),
        "guide_xy": guide.distance,
        "route_xy": route.distance,
        "route_guide_ratio": ratio,
        "point_count": len(route.points),
        "expanded": route.expanded,
        "detour_metrics": metrics,
        "route_selection": selection,
        "trace_count": len(trace),
        "trace_modes": dict(Counter(row.get("mode") for row in trace)),
        "trace_statuses": dict(
            Counter(row.get("status") for row in trace if row.get("status"))
        ),
        "fallback_count": len(fallback),
        "resolved_start": point_dict(route.start),
        "resolved_goal": point_dict(route.goal),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--project-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="AutonomousDriving project root",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        help="Navigation manifest to test; default: data/navigation/manifest.json",
    )
    parser.add_argument(
        "--skip-full-load",
        action="store_true",
        help="Skip the complete SHA/native-load pass and run route regressions only",
    )
    parser.add_argument(
        "--allow-non-monotone",
        action="store_true",
        help="Do not require manifest.settings.partition_method == 'monotone'",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="Explicit JSON report path; default: runtime/navigation-regressions/<timestamp>.json",
    )
    parser.add_argument('--roads-manifest',type=Path,help='Unified road manifest; defaults to active roads for the active navigation pack')
    args = parser.parse_args(argv)

    root = args.project_root.resolve()
    if not (root / "adrive").is_dir():
        parser.error("project root must contain adrive/")

    manifest = (
        args.manifest.resolve()
        if args.manifest
        else (root / "data" / "navigation" / "manifest.json").resolve()
    )
    if not manifest.is_file():
        parser.error("manifest does not exist: " + str(manifest))

    sys.path.insert(0, str(root))

    from adrive.data import DataBundle
    from adrive.models import PlanningError, Point
    from adrive.navmesh import NavMeshPack
    from adrive.planner import Planner
    from adrive.road_corridor import detour_metrics

    raw = json.loads(manifest.read_text(encoding="utf-8"))
    report = {
        "schema": 1,
        "tool": "check-navigation-regressions-v1",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "project_root": str(root),
        "manifest": str(manifest),
        "build": raw.get("client", {}).get("version"),
        "navigation_identity": raw.get("identity"),
        "tile_count": raw.get("tile_count"),
        "settings": raw.get("settings"),
        "full_load": None,
        "native": {},
        "cases": [],
        "road_loops": [],
        "westfall": None,
        "status": "running",
    }

    report_dir = root / "runtime" / "navigation-regressions"
    if args.report:
        report_path = args.report.resolve()
    else:
        report_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        report_path = report_dir / f"{stamp}-{uuid.uuid4().hex[:8]}.json"

    failures = []

    try:
        if (
            not args.allow_non_monotone
            and raw.get("settings", {}).get("partition_method") != "monotone"
        ):
            raise AssertionError(
                "manifest does not declare partition_method='monotone'"
            )

        if not args.skip_full_load:
            from tools.check_navigation import check
            report["full_load"] = check(manifest.parent, raw)
            expected_tiles = raw.get("tile_count")
            if (
                not isinstance(expected_tiles, int)
                or report["full_load"]["tiles_verified"] != expected_tiles
            ):
                raise AssertionError(
                    "complete native-load verification did not cover all manifest tiles"
                )

        pack = NavMeshPack(manifest)
        pack.record_diagnostics = False

        report["native"] = {
            "library_path": str((manifest.parent / raw["library"]["path"]).resolve()),
            "library_sha256": raw["library"]["sha256"],
            "ad_budget_reason": pack.native.budget_reason is not None,
            "ad_path_bounded_v3": pack.native.bounded_path is not None,
        }
        if pack.native.budget_reason is None:
            raise AssertionError("native DLL is missing ad_budget_reason")
        if pack.native.bounded_path is None:
            raise AssertionError("native DLL is missing ad_path_bounded_v3")

        bundle_path = root / "data" / "bundle.json"
        bundle = DataBundle.load(bundle_path)
        coarse = Planner(bundle)

        print("=== KNOWN REGRESSIONS ===", flush=True)
        for case in KNOWN_CASES:
            try:
                result = run_known_case(pack, coarse, Point, case)
                report["cases"].append(result)
                print(
                    f"PASS {result['name']}: "
                    f"xy={result['route_xy']:.4f} "
                    f"3d={result['route_3d']:.4f} "
                    f"points={result['point_count']} "
                    f"expanded={result['expanded']}",
                    flush=True,
                )
            except Exception as exc:
                item = {
                    "name": case["name"],
                    "status": "failed",
                    "error_code": getattr(exc, "code", type(exc).__name__),
                    "error": str(exc),
                }
                report["cases"].append(item)
                failures.append(item)
                print(
                    f"FAIL {case['name']}: "
                    f"{item['error_code']}: {item['error']}",
                    flush=True,
                )

        print("\n=== ROAD LOOP QUALITY ===", flush=True)
        for case in ROAD_LOOP_CASES:
            try:
                result = run_road_loop(pack, coarse, Point, case)
                report["road_loops"].append(result)
                print(f"PASS {result['name']}: straight={result['straight_xy']:.4f} "
                      f"guide={result['guide_xy']:.4f} route={result['route_xy']:.4f} "
                      f"expanded={result['expanded']} selected=direct", flush=True)
            except Exception as exc:
                item = {"name": case["name"], "status": "failed",
                        "error_code": getattr(exc, "code", type(exc).__name__), "error": str(exc)}
                report["road_loops"].append(item)
                failures.append(item)
                print(f"FAIL {case['name']}: {item['error_code']}: {item['error']}", flush=True)

        print("\n=== WESTFALL QUALITY ===", flush=True)
        try:
            westfall = run_westfall(pack, coarse, Point, detour_metrics)
            report["westfall"] = westfall
            print(f"PASS westfall_recorded_route", flush=True)
            print(f"straight_xy = {westfall['straight_xy']:.4f}", flush=True)
            print(f"guide_xy    = {westfall['guide_xy']:.4f}", flush=True)
            print(f"route_xy    = {westfall['route_xy']:.4f}", flush=True)
            print(
                f"route/guide = {westfall['route_guide_ratio']:.4f}",
                flush=True,
            )
            print(f"expanded    = {westfall['expanded']}", flush=True)
            print(f"fallbacks   = {westfall['fallback_count']}", flush=True)
            print(f"selected    = {westfall['route_selection']['selected']}", flush=True)
        except Exception as exc:
            item = {
                "name": WESTFALL["name"],
                "status": "failed",
                "error_code": getattr(exc, "code", type(exc).__name__),
                "error": str(exc),
            }
            report["westfall"] = item
            failures.append(item)
            print(
                f"FAIL westfall_recorded_route: "
                f"{item['error_code']}: {item['error']}",
                flush=True,
            )

        road_manifest=args.roads_manifest
        if road_manifest is None and manifest==(root/'data/navigation/manifest.json').resolve():
            road_manifest=root/'data/roads/manifest.json'
        if road_manifest is not None:
            if not road_manifest.exists():raise ValueError('Unified road execution gate requires its road manifest')
            print('\n=== UNIFIED ROAD EXECUTION GATE ===',flush=True)
            from tools.check_unified_roads import check as check_roads
            unified=check_roads(manifest,road_manifest.resolve(),report_path.with_suffix('.roads.json'),
                                full=not args.skip_full_load)
            report['unified_roads']=unified
            if unified['status']!='passed':failures.append(dict(name='unified_roads',error='Unified road gate failed'))

    except Exception as exc:
        failures.append(
            {
                "name": "preflight",
                "status": "failed",
                "error_code": getattr(exc, "code", type(exc).__name__),
                "error": str(exc),
            }
        )
        report["preflight_error"] = failures[-1]
        report["traceback"] = traceback.format_exc()

    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    report["failure_count"] = len(failures)
    report["status"] = "passed" if not failures else "failed"

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    print("\n=== SUMMARY ===", flush=True)
    print("STATUS =", report["status"], flush=True)
    print("FAILURES =", len(failures), flush=True)
    print("REPORT =", report_path, flush=True)
    if not failures:
        print("NAVIGATION_REGRESSIONS_PASSED", flush=True)
        return 0
    print("NAVIGATION_REGRESSIONS_FAILED", flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
