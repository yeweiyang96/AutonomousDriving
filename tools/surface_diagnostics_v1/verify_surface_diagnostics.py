#!/usr/bin/env python3
"""Verify additive diagnostics and legacy behavior on synthetic Detour geometry.

This worker is launched in a child process so loaded DLLs are released before
installation. No project manifest, production tiles or controls are modified.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes as C
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import traceback


def load_wrapper():
    source = Path(__file__).resolve().parent / 'payload' / 'nav_diagnostics.py'
    spec = importlib.util.spec_from_file_location('surface_diag_validation_wrapper', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Legacy:
    def __init__(self, path, callback_type):
        self.library_path, self.callback_type = Path(path).resolve(strict=True), callback_type
        self.dll = C.CDLL(str(self.library_path))
        ptr, ints, floats, refs = C.c_void_p, C.POINTER(C.c_int), C.POINTER(C.c_float), C.POINTER(C.c_uint64)
        specs = {
            'ad_create': ([ptr, C.c_int, C.POINTER(ptr)], C.c_int),
            'ad_destroy': ([ptr], None), 'ad_add': ([ptr, ptr, C.c_int], C.c_int),
            'ad_surfaces': ([ptr, floats, floats, C.c_int, ints], C.c_int),
            'ad_poly_key': ([ptr, C.c_uint64, ints], C.c_int),
            'ad_block': ([ptr, ints, ints], C.c_int),
            'ad_path': ([ptr, floats, floats, C.c_int, callback_type, floats, refs, C.c_int, ints, ints], C.c_int),
            'ad_budget_reason': ([ptr, C.POINTER(C.c_uint32)], C.c_int),
        }
        for name, (args, result) in specs.items():
            function = getattr(self.dll, name)
            function.argtypes, function.restype = args, result

    @contextmanager
    def context(self, directory, info, *, all_layers=False):
        handle = C.c_void_p()
        raw = (directory / info['map']).read_bytes()
        assert self.dll.ad_create(raw, len(raw), C.byref(handle)) == 0
        assert handle.value
        try:
            for name in info['tiles'] if all_layers else info['tiles'][:1]:
                raw = (directory / name).read_bytes()
                assert self.dll.ad_add(handle, raw, len(raw)) == 0, name
            yield handle
        finally:
            self.dll.ad_destroy(handle)

    @staticmethod
    def point(world):
        return (C.c_float * 3)(*(math.nan if value is None else value for value in world))

    def reason(self, handle):
        status = C.c_uint32()
        reason = self.dll.ad_budget_reason(handle, C.byref(status))
        return {'reason': reason, 'status': status.value}

    def path(self, handle, start, goal, budget=10000):
        capacity = 4096
        output, refs = (C.c_float * (capacity * 3))(), (C.c_uint64 * capacity)()
        count, expanded = C.c_int(), C.c_int()
        rc = self.dll.ad_path(handle, self.point(start), self.point(goal), budget,
                              self.callback_type(), output, refs, capacity,
                              C.byref(count), C.byref(expanded))
        assert 0 <= count.value <= capacity
        return {'return_code': rc, 'expanded': expanded.value, 'count': count.value,
                'points': [[output[i * 3 + j] for j in range(3)] for i in range(count.value)],
                'refs': list(refs[:count.value]), 'budget_reason': self.reason(handle)}

    def heights(self, handle, point):
        output, count = (C.c_float * 32)(), C.c_int()
        rc = self.dll.ad_surfaces(handle, self.point(point), output, 32, C.byref(count))
        return rc, list(output[:max(0, min(count.value, 32))])


def run(args):
    if not __debug__:
        raise RuntimeError('Validation requires assertions; run Python without -O or PYTHONOPTIMIZE.')
    module = load_wrapper()
    candidate = Legacy(args.dll, module.CANCEL)
    baseline = Legacy(args.baseline_dll, module.CANCEL)
    diagnostic = module.Diagnostics(candidate.dll)
    fixture_dir = Path(args.fixture_dir).resolve(strict=True)
    info = json.loads((fixture_dir / 'fixture-info.json').read_text(encoding='utf-8'))
    assert info['synthetic'] is True and info['coordinate_order'] == 'north,west,height'
    assert info['tiles'][0] == 'base.mmtile' and len(info['tiles']) == 34
    points = info['points']
    checks = []

    def check(name, function):
        try:
            details = function()
            checks.append({'name': name, 'passed': True, 'details': details})
        except Exception as error:
            checks.append({'name': name, 'passed': False,
                           'error': '%s: %s' % (type(error).__name__, str(error)),
                           'traceback': traceback.format_exc()})

    def candidates(handle, name, capacity=256):
        result = diagnostic.surfaces(handle, points[name], height_tolerance=.01, capacity=capacity)
        assert result['return_code'] == 0 and result['complete_for_loaded_graph']
        assert result['global_coverage_known'] is False
        assert all(item['physical_kind'] == 'UNKNOWN' and not item['swimming_authorized']
                   for item in result['surfaces'])
        return result['surfaces']

    def ref(handle, name):
        found = candidates(handle, name)
        assert len(found) == 1, (name, found)
        return found[0]['poly_ref']

    def capability_check():
        caps = diagnostic.capabilities
        assert caps['abi_version'] == 2 and caps['poly_ref_bytes'] == 8
        assert caps['walk_include_flags'] == 0x11 and caps['walk_exclude_flags'] == 0x0e
        assert caps['path_node_capacity'] == 65535
        assert [C.sizeof(t) for t in (module.Capabilities, module.Surface, module.SurfaceSummary,
                                       module.ReachabilityRequest, module.ReachabilityReport)] == [48, 72, 48, 40, 80]
        return caps

    def multilevel_check():
        with candidate.context(fixture_dir, info) as handle:
            found = candidates(handle, 'multilevel')
            assert len(found) == 2 and sorted(s['height'] for s in found) == [0.0, 5.0]
            assert len({s['poly_ref'] for s in found}) == 2
            outcome = diagnostic.reachability(handle, ref(handle, 'lower'), ref(handle, 'upper'))
            assert outcome['result'] == 2 and outcome['traversal_complete'] == 1
            assert outcome['global_connectivity_known'] == 0
            return {'heights': [s['height'] for s in found], 'reachability': outcome}

    def excluded_tags_check():
        with candidate.context(fixture_dir, info) as handle:
            water = candidates(handle, 'water')
            assert len(water) == 1 and (water[0]['area'], water[0]['flags']) == (9, 4)
            assert water[0]['walk_filter_pass'] == 0
            assert candidate.heights(handle, points['water'])[0] == 3
            zero = candidates(handle, 'zero_flags')
            assert len(zero) == 1 and zero[0]['flags'] == 0 and zero[0]['walk_filter_pass'] == 0
            assert candidate.heights(handle, points['zero_flags'])[0] == 3
            same = candidates(handle, 'equal_height')
            assert len(same) == 2 and len({s['poly_ref'] for s in same}) == 2
            assert {s['height'] for s in same} == {0.0} and {s['area'] for s in same} == {9, 11}
            return {'water': water, 'zero_flags': zero, 'equal_height': same}

    def hole_check():
        with candidate.context(fixture_dir, info) as handle:
            result = diagnostic.surfaces(handle, points['hole'])
            assert result['return_code'] == 0 and result['complete_for_loaded_graph']
            assert result['surfaces'] == [] and result['summary']['total_surfaces'] == 0
            return result

    def layer_check():
        with candidate.context(fixture_dir, info, all_layers=True) as handle:
            result = diagnostic.surfaces(handle, points['many_layers'])
            assert result['return_code'] == 0 and result['complete_for_loaded_graph']
            assert len(result['surfaces']) == 33
            assert {s['tile_layer'] for s in result['surfaces']} == set(range(1, 34))
            assert result['summary']['loaded_tiles'] == 34
            return {'count': 33, 'layers': sorted(s['tile_layer'] for s in result['surfaces'])}

    def capacity_check():
        with candidate.context(fixture_dir, info) as handle:
            one = diagnostic.surfaces(handle, points['multilevel'], capacity=1)
            empty = diagnostic.surfaces(handle, points['multilevel'], capacity=0)
            for result, capacity in ((one, 1), (empty, 0)):
                assert result['return_code'] == 6 and not result['complete_for_loaded_graph']
                assert result['summary']['total_surfaces'] == 2
                assert result['summary']['written_surfaces'] == capacity
                assert result['summary']['truncated'] == 1
            return {'capacity_one': one['summary'], 'capacity_zero': empty['summary']}

    def water_chain_check():
        with candidate.context(fixture_dir, info) as handle:
            start, goal = ref(handle, 'water_chain_start'), ref(handle, 'water_chain_goal')
            walk = diagnostic.reachability(handle, start, goal)
            topology = diagnostic.reachability(handle, start, goal, profile='tag_agnostic')
            assert walk['result'] == 2 and walk['traversal_complete'] == 1
            assert topology['result'] == 1 and topology['swimming_authorized'] is False
            assert topology['meaning'] == 'tag_agnostic_topology_witness_only'
            excluded = diagnostic.reachability(handle, ref(handle, 'water'), ref(handle, 'water'))
            assert excluded['result'] == 5
            return {'walk': walk, 'tag_agnostic': topology, 'water_endpoint': excluded}

    def offmesh_check():
        with candidate.context(fixture_dir, info) as handle:
            start, goal = ref(handle, 'offmesh_start'), ref(handle, 'offmesh_goal')
            outcomes = [diagnostic.reachability(handle, start, goal, profile=profile)
                        for profile in ('walk', 'tag_agnostic')]
            assert all(value['result'] == 2 for value in outcomes)
            assert candidate.path(handle, points['offmesh_start'], points['offmesh_goal'])['return_code'] == 5
            return outcomes

    def limits_check():
        with candidate.context(fixture_dir, info) as handle:
            start, goal = ref(handle, 'ground_chain_start'), ref(handle, 'ground_chain_goal')
            visits = diagnostic.reachability(handle, start, goal, max_visits=1)
            links = diagnostic.reachability(handle, start, goal, max_link_checks=1)
            cancelled = diagnostic.reachability(handle, start, goal, cancel=lambda: True)
            assert visits['result'] == 3 and visits['traversal_complete'] == 0
            assert links['result'] == 3 and links['link_limit_reached'] == 1
            assert cancelled['result'] == 4
            assert all(value['global_connectivity_known'] == 0 for value in (visits, links, cancelled))
            return {'visits': visits, 'links': links, 'cancelled': cancelled}

    def blocks_check():
        with candidate.context(fixture_dir, info) as handle:
            start = candidates(handle, 'ground_chain_start')[0]
            middle = candidates(handle, 'ground_chain_middle')[0]
            goal = candidates(handle, 'ground_chain_goal')[0]
            key = lambda s: (C.c_int * 3)(s['tile_x'], s['tile_y'], s['poly_index'])
            assert diagnostic.reachability(handle, start['poly_ref'], goal['poly_ref'])['result'] == 1
            assert candidate.dll.ad_block(handle, key(start), key(middle)) == 0
            forward = diagnostic.reachability(handle, start['poly_ref'], goal['poly_ref'])
            reverse = diagnostic.reachability(handle, goal['poly_ref'], start['poly_ref'])
            assert forward['result'] == reverse['result'] == 2
            assert candidate.dll.ad_block(handle, key(goal), key(goal)) == 0
            blocked_self = diagnostic.reachability(handle, goal['poly_ref'], goal['poly_ref'])
            assert blocked_self['result'] == 5
            assert candidates(handle, 'ground_chain_goal')[0]['blocked_self'] == 1
            return {'forward': forward, 'reverse': reverse, 'blocked_self': blocked_self}

    def invalid_check():
        dll = candidate.dll
        with candidate.context(fixture_dir, info) as handle:
            start = ref(handle, 'lower')
            invalid = diagnostic.reachability(handle, 0, start)
            assert invalid['result'] == 6
            cap = module.initialized(module.Capabilities)
            cap.struct_size -= 4
            assert dll.ad_capabilities_v2(C.byref(cap)) == 1
            assert dll.ad_capabilities_v2(None) == 1
            summary = module.initialized(module.SurfaceSummary)
            output = (module.Surface * 1)()
            point = (C.c_float * 3)(math.inf, 5, 0)
            assert dll.ad_query_surfaces_v2(handle, point, 2.5, output, 1, C.byref(summary)) == 1
            point = (C.c_float * 3)(5, 5, 0)
            summary = module.initialized(module.SurfaceSummary)
            assert dll.ad_query_surfaces_v2(None, point, 2.5, output, 1, C.byref(summary)) == 1
            summary = module.initialized(module.SurfaceSummary)
            assert dll.ad_query_surfaces_v2(handle, point, -1, output, 1, C.byref(summary)) == 1
            request = module.initialized(module.ReachabilityRequest)
            request.start_ref = request.goal_ref = start
            request.profile = 1; request.max_visits = 10; request.max_link_checks = 100
            request.abi_version = 99
            report = module.initialized(module.ReachabilityReport)
            assert dll.ad_query_reachability_v2(handle, C.byref(request), module.CANCEL(), C.byref(report)) == 1
            return {'invalid_reference': invalid, 'invalid_layout_null_and_nonfinite_rejected': True}

    def legacy_check():
        route_cases = [
            ('ring_forward', points['ring_start'], points['ring_goal'], 10000),
            ('ring_reverse', points['ring_goal'], points['ring_start'], 10000),
            ('ground_chain', points['ground_chain_start'], points['ground_chain_goal'], 10000),
            ('water_endpoint', points['water'], points['ring_goal'], 10000),
            ('ambiguous_floor', points['multilevel'], points['ring_goal'], 10000),
            ('hole_endpoint', points['hole'], points['ring_goal'], 10000),
            ('budget_zero', points['ring_start'], points['ring_goal'], 0),
            ('offmesh_disallowed', points['offmesh_start'], points['offmesh_goal'], 10000),
        ]
        results = {}
        with baseline.context(fixture_dir, info) as old, candidate.context(fixture_dir, info) as new:
            for name, start, goal, budget in route_cases:
                expected = baseline.path(old, start, goal, budget)
                actual = candidate.path(new, start, goal, budget)
                assert actual == expected, (name, expected, actual)
                results[name] = actual
        assert results['ring_forward']['return_code'] == results['ring_reverse']['return_code'] == 0
        assert results['water_endpoint']['return_code'] == results['hole_endpoint']['return_code'] == 3
        assert results['ambiguous_floor']['return_code'] == 4
        assert results['budget_zero']['return_code'] == 6
        return results

    def readonly_check():
        with candidate.context(fixture_dir, info) as handle:
            expected = candidate.path(handle, points['ring_start'], points['ring_goal'])
            limited = candidate.path(handle, points['ring_start'], points['ring_goal'], 0)
            assert limited['return_code'] == 6
            prior_reason = candidate.reason(handle)
            assert prior_reason['reason'] == 1
            lower = candidates(handle, 'lower')[0]['poly_ref']
            upper = candidates(handle, 'upper')[0]['poly_ref']
            diagnostic.surfaces(handle, points['multilevel'], capacity=1)
            diagnostic.reachability(handle, lower, upper)
            diagnostic.reachability(handle, lower, upper, max_visits=1)
            assert candidate.reason(handle) == prior_reason
            actual = candidate.path(handle, points['ring_start'], points['ring_goal'])
            assert actual == expected
            return {'path_and_expansions_unchanged': True, 'budget_reason_preserved': prior_reason}

    for name, function in (
        ('capability_layout_and_original_filter', capability_check),
        ('multilevel_surfaces_and_loaded_only_disconnection', multilevel_check),
        ('water_zero_flags_and_equal_height_identities', excluded_tags_check),
        ('hole_does_not_snap_to_nearby_ground', hole_check),
        ('all_33_loaded_layers_are_enumerated', layer_check),
        ('surface_capacity_is_explicitly_incomplete', capacity_check),
        ('water_chain_is_topology_only_not_swimming', water_chain_check),
        ('offmesh_is_excluded_in_both_profiles', offmesh_check),
        ('visit_link_limits_and_cancellation_are_unknown', limits_check),
        ('blocked_edges_are_bidirectional_and_blocked_nodes_rejected', blocks_check),
        ('invalid_layout_handle_coordinate_and_reference', invalid_check),
        ('legacy_abi_matches_current_baseline_dll', legacy_check),
        ('diagnostics_preserve_search_result_and_budget_reason', readonly_check),
    ):
        check(name, function)
    passed = all(item['passed'] for item in checks)
    return {
        'schema': 1, 'status': 'passed' if passed else 'failed',
        'scope': 'Synthetic actual Detour fixtures; not a production route or whole-map validation.',
        'dll': str(candidate.library_path), 'dll_sha256': hashlib.sha256(candidate.library_path.read_bytes()).hexdigest(),
        'baseline_dll': str(baseline.library_path),
        'baseline_dll_sha256': hashlib.sha256(baseline.library_path.read_bytes()).hexdigest(),
        'capabilities': diagnostic.capabilities, 'checks': checks,
        'passed_check_count': sum(item['passed'] for item in checks), 'total_check_count': len(checks),
        'production_changes_performed': False, 'swimming_enabled': False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll', required=True)
    parser.add_argument('--fixture-dir', required=True)
    parser.add_argument('--baseline-dll', required=True)
    parser.add_argument('--report', required=True)
    args = parser.parse_args(argv)
    try:
        report = run(args)
    except Exception as error:
        report = {'schema': 1, 'status': 'failed',
                  'error': '%s: %s' % (type(error).__name__, str(error)),
                  'traceback': traceback.format_exc(), 'production_changes_performed': False}
    destination = Path(args.report)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('DIAGNOSTIC_VALIDATION_' + report['status'].upper())
    print('REPORT = ' + str(destination.resolve()))
    if report['status'] != 'passed':
        for check in report.get('checks', []):
            if not check['passed']:
                print('FAILED: ' + check['name'] + ': ' + check['error'])
        if 'error' in report:
            print(report['error'])
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
