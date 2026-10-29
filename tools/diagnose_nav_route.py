#!/usr/bin/env python3
"""Inspect a recorded navigation failure with the installed diagnostic ABI.

Run from any working directory after saving this file in PROJECT/tools.
Reads the current manifest and runtime/route-quality/last-plan.json; writes
only a new runtime/route-surface-diagnostics directory. It never plans a new
route, changes movement policy, modifies the manifest, or controls the game.
Graph answers concern the loaded tiles and explicit candidate polygon refs.
Navigation tags are not physical-water evidence or permission to swim.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid
import zipfile

VERSION = 'route-surface-diagnostics-v1'
MAX_JSON = 16 * 1024 * 1024
MAX_ASSET = 32 * 1024 * 1024
ARCHIVE_TILE_BYTES = 64 * 1024 * 1024
MAX_SURFACES = 256
MAX_PAIR_TESTS = 4
MAX_VISITS = 200000
MAX_LINK_CHECKS = 4000000
FORWARD = {'road', 'road_tolerance', 'local_detour'}
SIGNIFICANT = {'excessive_detour', 'budget', 'no_route', 'navmesh_outside',
               'navmesh_height', 'blocked_navmesh', 'metric_limit'}
SUPPORTED_SOURCES = {
    'road-corridor-v1': (
        '64efa0d0f8f069632644c2db11bff6dd3a72a0a437f545f23c38bd28fbbd325d',
        'f42e3cfd5b8d6053a6452afc043278512189b559913c884cdbbcd0dd1bedf7b3'),
    'road-corridor-v2': (
        '54f75aea3a68e43935931cc951472a9935c1725579d12f7a40775e3d4581a54e',
        'ebb294704bddf569ca2035553676920934a7d83e600af3d7c1f7a32b370301b5'),
}
WRAPPER_SHA = '213b7d40702231657253228d7c51499ea57d528e85ff8660ab19dd8c41d4600d'


def say(message):
    print(message, flush=True)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def signature(stat):
    # Compare path stat with path stat, descriptor stat with descriptor stat.
    # Windows may give different inode representations for the two APIs.
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


def stable_read(path, maximum=MAX_JSON):
    path = Path(path)
    before = signature(path.stat())
    if before[2] > maximum:
        raise ValueError('Input exceeds size limit: ' + str(path))
    with path.open('rb') as stream:
        opened = signature(os.fstat(stream.fileno()))
        data = stream.read(maximum + 1)
        closed = signature(os.fstat(stream.fileno()))
    after = signature(path.stat())
    if before != after or opened != closed or len(data) != before[2] or len(data) != opened[2]:
        raise ValueError('Input changed during its read: ' + str(path))
    if len(data) > maximum:
        raise ValueError('Input exceeds size limit: ' + str(path))
    return data


def json_bytes(data):
    def invalid_constant(text):
        raise ValueError('Non-finite JSON number: ' + text)
    return json.loads(data.decode('utf-8-sig'), parse_constant=invalid_constant)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def contained(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Path escapes project/data directory: ' + str(relative))
    return path


def xyz(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('Expected three coordinates')
    result = []
    for index, number in enumerate(value):
        if number is None and index == 2:
            result.append(None)
        elif type(number) in (int, float) and math.isfinite(number) and abs(number) <= 20000:
            result.append(float(number))
        else:
            raise ValueError('Invalid navigation coordinate')
    return result


def point_record(value):
    if not isinstance(value, dict) or type(value.get('c')) is not int or value['c'] not in (0, 1):
        raise ValueError('Invalid continent/point record')
    return value['c'], xyz([value.get('x'), value.get('y'), value.get('z')])


def native_debug(document):
    message = document.get('result', {}).get('message', '')
    if not isinstance(message, str) or 'NAV_DEBUG ' not in message:
        return None
    result, _ = json.JSONDecoder().raw_decode(message.split('NAV_DEBUG ', 1)[1])
    if not isinstance(result, dict):
        raise ValueError('NAV_DEBUG is not an object')
    return result


def final_window(trace):
    """A recorded control-flow episode, not a claim about physical cause."""
    active, frontier, transition = [], None, None
    for index, row in enumerate(trace):
        mode = row.get('mode')
        if mode == 'rollback_soft_anchor':
            if frontier is None:
                frontier = row.get('abandoned_anchor')
            active.append(row)
        elif mode in FORWARD and row.get('accepted') is True:
            target = row.get('to_anchor')
            if type(frontier) is int and type(target) is int and target > frontier:
                active, frontier = [], None
        elif mode == 'remaining_native_fallback':
            transition = (index, row, list(active))
    if transition is None:
        return None
    end, row, active = transition
    bounds = [row.get('remaining_reference_range', {}).get('anchor_from')]
    bounds += [r.get('resume_anchor') for r in active]
    bounds = [v for v in bounds if type(v) is int]
    if not bounds:
        return None
    lower = min(bounds)
    candidates = [i for i, r in enumerate(trace[:end]) if r.get('mode') in FORWARD
                  and type(r.get('from_anchor')) is int and r['from_anchor'] >= lower]
    return {'trace_from': min(candidates, default=end), 'trace_to_exclusive': end,
            'earliest_resume_anchor': lower, 'method': 'final_active_rollback_episode'}


def choose_local(document, trace):
    window = final_window(trace)
    if not window:
        # No final fallback episode means that its location is not established.
        return None, None
    known = {0: point_record(document['start'])[1]}
    options = []
    for index, row in enumerate(trace):
        if row.get('mode') == 'native_reconnect' and row.get('accepted') is True:
            lower, upper = row.get('from_anchor'), row.get('to_anchor')
            # Reconnect may move retained intermediate rollback checkpoints to
            # actual polygon boundaries; their new XYZ is not in the old log.
            if type(lower) is int and type(upper) is int:
                known = {key: value for key, value in known.items()
                         if type(key) is int and not lower < key < upper}
            else:
                known.clear()
        if row.get('mode') not in FORWARD:
            continue
        source = known.get(row.get('from_anchor'))
        target = xyz(row['target']) if 'target' in row else None
        if (window['trace_from'] <= index < window['trace_to_exclusive'] and
                row.get('accepted') is False and source is not None and target is not None):
            options.append({'name': 'local_road_candidate', 'start': list(source), 'goal': target,
                            'trace_index': index, 'recorded_candidate': row,
                            'coordinate_evidence': 'accepted_anchor_target_or_original_start'})
        if row.get('accepted') is True and target is not None:
            known[row.get('to_anchor')] = target
    significant = [option for option in options if option['recorded_candidate'].get('status') in SIGNIFICANT]
    return (significant or options or [None])[0], window


def build_reference(document, Point):
    continent, a = point_record(document['start'])
    gc, b = point_record(document['goal'])
    if gc != continent:
        raise ValueError('Cross-continent input cannot be diagnosed in one native context')
    start, goal = Point(continent, *a), Point(continent, *b)
    reference, segments = [start], []
    guide = document.get('guide')
    if guide is not None:
        if not isinstance(guide, dict) or guide.get('truncated') or not isinstance(guide.get('segments'), list):
            raise ValueError('A complete guide segment list is required to reconstruct tile loading')
        if len(guide['segments']) > 10000:
            raise ValueError('Guide exceeds the recorded protocol bound')
        for segment in guide['segments']:
            points = []
            for value in segment.get('points', []):
                c, p = point_record(value)
                if c != continent:
                    raise ValueError('Guide mixes continents')
                points.append(Point(c, *p))
            if not points:
                raise ValueError('Empty guide segment')
            segments.append((segment.get('reliability'), points))
            if segment.get('reliability') in ('reviewed', 'candidate'):
                for point in points:
                    if reference[-1].distance(point) > 1e-5:
                        reference.append(point)
    if reference[-1].distance(goal) > 1e-5 or len(reference) == 1:
        reference.append(goal)
    else:
        reference[-1] = goal
    return continent, start, goal, reference, segments


def reconstruct_anchors(start, goal, segments, Point):
    """Pinned v1/v2 XY anchor construction; Z is never used as measured evidence."""
    anchors = [start]
    for reliability, points in segments:
        if reliability not in ('reviewed', 'candidate'):
            continue
        for index, vertex in enumerate(points):
            before = anchors[-1]
            distance = before.distance(vertex)
            if distance <= 1e-7:
                continue
            steps = max(1, math.ceil(distance / 8.0)) if index > 0 else 1
            if len(anchors) + steps > 100000:
                raise ValueError('Anchor reconstruction exceeds diagnostic bound')
            for step in range(1, steps + 1):
                t = step / steps
                anchors.append(Point(vertex.c, before.x + t * (vertex.x-before.x),
                                     before.y + t * (vertex.y-before.y), None))
    if anchors[-1].distance(goal) > 1e-7:
        anchors.append(goal)
    else:
        anchors[-1] = goal
    return anchors


def reference_samples(line, spacing=2.0, capacity=65):
    lengths = [math.dist(a[:2], b[:2]) for a, b in zip(line, line[1:])]
    total = sum(lengths)
    count = min(capacity, max(2, math.ceil(total / spacing) + 1))
    samples, segment, before = [], 0, 0.0
    for index in range(count):
        distance = total * index / (count-1)
        while segment < len(lengths)-1 and before + lengths[segment] < distance:
            before += lengths[segment]
            segment += 1
        t = (distance-before) / lengths[segment] if lengths[segment] else 0.0
        a, b = line[segment], line[segment+1]
        samples.append({'distance_along_xy': distance,
                        'coordinate': [a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1]), None]})
    return {'reference_xy': [p[:2] for p in line], 'length_xy': total,
            'nominal_spacing_yards': spacing, 'actual_spacing_yards': total/(count-1),
            'coarsened_to_point_limit': total/(count-1) > spacing + 1e-6,
            'sampling_is_not_continuous_collision_validation': True, 'samples': samples}


def endpoint_candidates(observation, requested):
    surfaces = observation['surfaces']
    if requested[2] is None:
        candidates = list(surfaces)
        rule = 'height_unspecified_all_observed_candidates_no_floor_selected'
    else:
        candidates = [s for s in surfaces if abs(s['height']-requested[2]) <= 2.5]
        rule = 'within_2.5_yards_of_recorded_z_no_original_ref_reconstruction'
    candidates.sort(key=lambda s: (abs(s['height']-requested[2]) if requested[2] is not None else s['height'],
                                   s['tile_x'], s['tile_y'], s['tile_layer'], s['poly_index']))
    return candidates, rule


def check_pairs(diag, mesh, pair, observations, deadline):
    a, arule = endpoint_candidates(observations[pair['start_key']], pair['start'])
    b, brule = endpoint_candidates(observations[pair['goal_key']], pair['goal'])
    aw = [s for s in a if s['walk_filter_pass'] and not s['blocked_self']]
    bw = [s for s in b if s['walk_filter_pass'] and not s['blocked_self']]
    # Prioritize walk-eligible candidates; retain excluded-tag candidates only
    # if no walk pair is available. No chosen pair becomes a movement endpoint.
    aa, bb = (aw, bw) if aw and bw else (a, b)
    total = len(aa) * len(bb)
    result = {'start_candidate_rule': arule, 'goal_candidate_rule': brule,
              'start_matching_candidates': len(a), 'goal_matching_candidates': len(b),
              'start_walk_candidates': len(aw), 'goal_walk_candidates': len(bw),
              'candidate_pair_count': total, 'candidate_pair_limit': MAX_PAIR_TESTS,
              'candidate_pairs_truncated': total > MAX_PAIR_TESTS, 'tests': [],
              'original_native_refs_recorded': False, 'physical_root_cause_known': False,
              'complete_route_verified': False, 'swimming_authorized': False}
    tested = 0
    for left in aa:
        for right in bb:
            if tested >= MAX_PAIR_TESTS:
                break
            if time.monotonic() >= deadline:
                result['stopped_at_time_budget'] = True
                result['candidate_pairs_truncated'] = True
                return result
            tested += 1
            for direction, p, q in [('forward', left, right), ('reverse', right, left)]:
                item = {'direction': direction, 'start_ref_hex': p['poly_ref_hex'],
                        'goal_ref_hex': q['poly_ref_hex'], 'start_height': p['height'], 'goal_height': q['height']}
                say('  GRAPH %s %s pair %d/%d' % (pair['name'], direction, tested, min(total, MAX_PAIR_TESTS)))
                item['walk'] = diag.reachability(mesh.handle, p['poly_ref'], q['poly_ref'],
                    max_visits=MAX_VISITS, max_link_checks=MAX_LINK_CHECKS,
                    cancel=lambda: time.monotonic() >= deadline)
                if item['walk']['result_name'] != 'reachable_in_loaded_graph' and time.monotonic() < deadline:
                    item['tag_agnostic'] = diag.reachability(mesh.handle, p['poly_ref'], q['poly_ref'],
                        profile='tag_agnostic', max_visits=MAX_VISITS, max_link_checks=MAX_LINK_CHECKS,
                        cancel=lambda: time.monotonic() >= deadline)
                    if (item['walk']['result_name'] in ('disconnected_in_loaded_graph', 'endpoint_excluded') and
                            item['tag_agnostic']['result_name'] == 'reachable_in_loaded_graph'):
                        item['interpretation'] = 'navigation_filter_changes_connectivity_for_this_candidate_pair'
                result['tests'].append(item)
                if time.monotonic() >= deadline:
                    result['stopped_at_time_budget'] = True
                    result['candidate_pairs_truncated'] = True
                    return result
        if tested >= MAX_PAIR_TESTS:
            break
    return result


def observed_findings(document, debug, local, observations, scan):
    findings = []
    if debug:
        if debug.get('native_reason') == 'node_pool_exhausted':
            findings.append({'code': 'recorded_path_node_pool_exhausted', 'evidence': 'input_plan.result.message.NAV_DEBUG',
                             'text': 'The recorded native path call exhausted its node pool; this is not a proof of missing terrain or global disconnection.'})
        if (type(debug.get('loaded_bytes')) is int and type(debug.get('max_bytes')) is int and
                debug['loaded_bytes'] < debug['max_bytes']):
            findings.append({'code': 'recorded_loaded_bytes_below_limit',
                             'loaded_bytes': debug['loaded_bytes'], 'max_bytes': debug['max_bytes']})
    if local and local['recorded_candidate'].get('status') == 'excessive_detour':
        row = local['recorded_candidate']
        findings.append({'code': 'recorded_complete_local_path_rejected_for_length',
                         'trace_index': local['trace_index'], 'route_length_xy': row.get('route_length_xy'),
                         'reference_length_xy': row.get('reference_length_xy'),
                         'length_limit_xy': row.get('length_limit_xy')})
    if scan:
        absent, water, unknown = [], [], []
        for index, sample in enumerate(scan['samples']):
            obs = observations[sample['observation_key']]
            if not obs['complete_for_loaded_graph']:
                unknown.append(index)
            elif not obs['surfaces']:
                absent.append(index)
            if any(s['area'] == 9 or s['flags'] & 4 for s in obs['surfaces']):
                water.append(index)
        findings.append({'code': 'reference_line_all_height_observations',
                         'sample_count': len(scan['samples']), 'no_surface_sample_indices': absent,
                         'water_tag_at_any_height_sample_indices': water, 'incomplete_sample_indices': unknown,
                         'physical_water_or_mislabeling_proven': False,
                         'note': 'A water tag below a bridge may be legitimate. All heights are retained; no intended floor is inferred from the 2D road.'})
    return findings


def run(args, output, report):
    root = args.project_root.resolve()
    plan_path = (args.plan or root/'runtime/route-quality/last-plan.json').resolve()
    manifest_path = contained(root, 'data/navigation/manifest.json')
    manifest_data, plan_data = stable_read(manifest_path), stable_read(plan_path)
    raw, document = json_bytes(manifest_data), json_bytes(plan_data)
    if not isinstance(document, dict) or document.get('schema') != 1:
        raise ValueError('Expected a schema 1 route-quality last-plan.json')
    if document.get('result', {}).get('status') != 'error':
        raise ValueError('The selected last-plan is not a failed plan; use --plan with the failed report')
    if document.get('navigation_identity') != raw.get('identity'):
        raise ValueError('NAVIGATION_IDENTITY_MISMATCH: original failure and current tiles differ; no historical blocks or connectivity were applied')
    tile_identity = hashlib.sha256(json.dumps(raw['tiles'], sort_keys=True).encode()).hexdigest()[:16]
    if tile_identity != raw['identity']:
        raise ValueError('Manifest tile identity does not match its tile index')
    trace = document.get('road_trace', [])
    if not isinstance(trace, list) or any(not isinstance(row, dict) for row in trace):
        raise ValueError('Expected an object list for road_trace')
    if len(trace) > 2000 or document.get('road_trace_count', len(trace)) > len(trace) or document.get('complete') is False:
        raise ValueError('The selected plan is truncated; its original loading/anchor sequence cannot be reconstructed reliably')
    source_data = {}
    report['source_files'] = {}
    for relative in ['adrive/navmesh.py', 'adrive/road_corridor.py', 'adrive/nav_diagnostics.py',
                     'adrive/models.py', 'adrive/data.py', 'adrive/__init__.py']:
        path = contained(root, relative)
        content = stable_read(path)
        source_data[relative] = content
        normalized = content.decode('utf-8-sig').replace('\r\n', '\n').encode()
        report['source_files'][relative] = {'bytes': len(content), 'sha256': digest(content),
                                           'normalized_sha256': digest(normalized)}
    expected = SUPPORTED_SOURCES.get(document.get('planner_version'))
    actual = tuple(report['source_files'][s]['normalized_sha256'] for s in ['adrive/navmesh.py', 'adrive/road_corridor.py'])
    if actual != expected:
        raise ValueError('PLANNER_SOURCE_NOT_RECOGNIZED: loading/anchor reconstruction is limited to the verified v1/v2 source pair')
    if report['source_files']['adrive/nav_diagnostics.py']['normalized_sha256'] != WRAPPER_SHA:
        raise ValueError('DIAGNOSTIC_WRAPPER_NOT_RECOGNIZED: expected the installed surface_diagnostics_v1 wrapper')
    (output/'input-last-plan.json').write_bytes(plan_data)
    (output/'input-manifest.json').write_bytes(manifest_data)
    report['input'] = {'plan_path': str(plan_path), 'plan_sha256': digest(plan_data),
                       'manifest_path': str(manifest_path), 'manifest_sha256': digest(manifest_data),
                       'navigation_identity': raw['identity'], 'recorded_dll_sha256': document.get('native_library_sha256'),
                       'current_dll_sha256': raw['library']['sha256'],
                       'same_dll_as_original_plan': document.get('native_library_sha256') == raw['library']['sha256'],
                       'dll_change_is_expected_after_diagnostic_install': True,
                       'planner_version': document.get('planner_version')}
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(root))
    nav = importlib.import_module('adrive.navmesh')
    wrapper = importlib.import_module('adrive.nav_diagnostics')
    for module in (nav, wrapper, sys.modules['adrive.road_corridor']):
        if not Path(module.__file__).resolve().is_relative_to(root):
            raise ValueError('Imported module does not belong to the selected project')
    pack = nav.NavMeshPack(manifest_path)
    pack.record_diagnostics = False
    if stable_read(manifest_path) != manifest_data:
        raise ValueError('Manifest changed while initializing the independent native context')
    diag = wrapper.Diagnostics(pack.native.dll)
    report['capabilities'] = diag.capabilities
    report['current_dll_path'] = str(contained(pack.root, raw['library']['path']))
    say('DLL = ' + report['current_dll_path'])
    c, start, goal, reference, segments = build_reference(document, nav.Point)
    debug = native_debug(document)
    report['recorded_native_debug'] = debug
    if debug and debug.get('continent', c) != c:
        raise ValueError('NAV_DEBUG continent disagrees with the route')
    keys = set()
    for a, b in zip(reference, reference[1:]):
        steps = max(1, math.ceil(a.distance(b)/(nav.TILE_SIZE/2)))
        if steps > 512:
            raise ValueError('Reference segment exceeds diagnostic reconstruction bound')
        for index in range(steps+1):
            t = index/steps
            keys.update(pack.around(nav.Point(c, a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t)))
    keys = keys or pack.around(start)
    radius = debug.get('radius', 1) if debug else 1
    if radius not in (1, 2, 4):
        raise ValueError('Unsupported recorded fallback radius')
    if radius > 1:
        keys |= {(r+dr, col+dc) for r, col in tuple(keys) for dr in range(-radius, radius+1)
                 for dc in range(-radius, radius+1) if 0 <= r+dr < 64 and 0 <= col+dc < 64}
    local, window = choose_local(document, trace)
    report['final_recovery_window'] = window
    pairs = []
    if local:
        pairs.append(local)
    if debug and 'start' in debug and 'goal' in debug:
        pairs.append({'name': 'terminal_native_call', 'start': xyz(debug['start']), 'goal': xyz(debug['goal']),
                      'coordinate_evidence': 'literal_NAV_DEBUG', 'recorded_stage': debug.get('stage')})
    report['selected_pairs'] = pairs
    samples = None
    if local:
        anchors = reconstruct_anchors(start, goal, segments, nav.Point)
        for row in trace:
            if row.get('mode') in FORWARD and 'target' in row:
                index = row.get('to_anchor')
                if type(index) is not int or not 0 <= index < len(anchors):
                    raise ValueError('Recorded anchor index disagrees with reconstructed guide')
                if math.dist([anchors[index].x, anchors[index].y], xyz(row['target'])[:2]) > .002:
                    raise ValueError('Recorded target XY disagrees with reconstructed guide')
        row = local['recorded_candidate']
        lower, upper = row['from_anchor'], row['to_anchor']
        if not (type(lower) is int and type(upper) is int and 0 <= lower < upper < len(anchors)):
            raise ValueError('Invalid selected anchor range')
        line = [local['start'], *[[p.x, p.y, None] for p in anchors[lower+1:upper]], local['goal']]
        samples = reference_samples(line)
        samples['reference_evidence'] = 'reconstructed_v1_v2_guide_XY_validated_against_all_recorded_targets'
    report['reference_scan'] = samples
    blocks = document.get('blocked_edges', [])
    if not isinstance(blocks, list) or len(blocks) > 100000 or any(not isinstance(edge, str) for edge in blocks):
        raise ValueError('Invalid blocked-edge list')
    prefix = 'nav:%s:%d:' % (pack.identity, c)
    report['blocked_edges'] = {'recorded_count': len(blocks),
                               'matching_identity_and_continent': sum(e.startswith(prefix) for e in blocks),
                               'policy': 'same_version_records_submitted_to_legacy_block_API_in_independent_context',
                               'actual_bound_record_count_known': False}
    known_keys = sorted(key for key in keys if (c, *key) in pack.tiles)
    missing_keys = sorted(keys-set(known_keys))
    report['loading'] = {'method': 'reconstructed_from_verified_v1_v2_loader_and_recorded_guide',
                         'original_tile_keyset_was_not_recorded': True, 'continent': c,
                         'radius': radius, 'requested_keys': [list(k) for k in sorted(keys)],
                         'not_indexed_keys': [list(k) for k in missing_keys], 'global_coverage_known': False,
                         'loaded_tiles': [], 'unavailable_tiles': [], 'max_bytes': pack.max_bytes}
    deadline = time.monotonic() + args.max_seconds
    observations = {}
    report['surface_observations'] = observations
    coordinate_cache = {}
    with pack.mesh(c) as mesh:
        say('LOAD: verifying %d indexed tiles; %d requested keys are not indexed' % (len(known_keys), len(missing_keys)))
        for index, key in enumerate(known_keys):
            if time.monotonic() >= deadline:
                report['loading']['unavailable_tiles'].extend({'key': list(k), 'reason': 'diagnostic_time_limit'} for k in known_keys[index:])
                break
            try:
                mesh.load({key})
                entry = pack.tiles[(c, *key)]
                report['loading']['loaded_tiles'].append(dict(entry))
            except Exception as error:
                report['loading']['unavailable_tiles'].append({'key': list(key), 'reason': str(error)})
            if (index+1) % 8 == 0 or index+1 == len(known_keys):
                say('  TILES %d/%d; loaded %.1f MiB' % (index+1, len(known_keys), mesh.bytes/(1024*1024)))
        mesh.block(blocks)
        report['loading']['loaded_count'] = len(mesh.loaded)
        report['loading']['loaded_bytes'] = mesh.bytes
        report['loading']['count_matches_recorded'] = None if not debug else debug.get('loaded_tiles') == len(mesh.loaded)
        report['loading']['bytes_match_recorded'] = None if not debug else debug.get('loaded_bytes') == mesh.bytes
        report['loading']['all_indexed_requested_tiles_loaded'] = not report['loading']['unavailable_tiles']
        tile_by_mesh = {}
        for entry in report['loading']['loaded_tiles']:
            tile_by_mesh.setdefault((entry.get('mesh_x'), entry.get('mesh_y')), []).append(entry)

        def observe(label, p):
            key = (p[0], p[1])
            if key in coordinate_cache:
                return coordinate_cache[key]
            if time.monotonic() >= deadline:
                value = {'complete_for_loaded_graph': False, 'surfaces': [], 'status': 'not_run_time_limit',
                         'requested_xy': list(key), 'global_coverage_known': False}
            else:
                value = diag.surfaces(mesh.handle, (p[0], p[1], None), capacity=MAX_SURFACES)
                value['requested_xy'] = list(key)
                value['height_query'] = 'all_heights_no_floor_selection'
                for surface in value['surfaces']:
                    candidates = tile_by_mesh.get((surface['tile_x'], surface['tile_y']), [])
                    surface['source_tile_candidates'] = [dict(path=e['path'], sha256=e['sha256'], c=e['c'],
                                                             row=e['row'], col=e['col']) for e in candidates]
            observations[label] = value
            coordinate_cache[key] = label
            return label

        say('SURFACES: original endpoints, selected failure intervals and reference-line samples')
        report['original_endpoint_observations'] = {
            'start': observe('original_start', [start.x, start.y, start.z]),
            'goal': observe('original_goal', [goal.x, goal.y, goal.z])}
        for pair in pairs:
            pair['start_key'] = observe(pair['name']+'_start', pair['start'])
            pair['goal_key'] = observe(pair['name']+'_goal', pair['goal'])
        if samples:
            for index, sample in enumerate(samples['samples']):
                sample['observation_key'] = observe('reference_%03d' % index, sample['coordinate'])
        for pair in pairs:
            pair['connectivity'] = check_pairs(diag, mesh, pair, observations, deadline)
    report['findings'] = observed_findings(document, debug, local, observations, samples)
    report['time_budget_reached'] = time.monotonic() >= deadline
    report['input_recheck'] = {}
    for label, path, content in [('manifest', manifest_path, manifest_data), ('input_plan', plan_path, plan_data)]:
        try:
            report['input_recheck'][label] = digest(stable_read(path)) == digest(content)
        except (OSError, ValueError):
            report['input_recheck'][label] = False
    report['input_recheck']['sources'] = all(stable_read(contained(root, path)) == content for path, content in source_data.items())
    report['input_recheck']['dll'] = digest(stable_read(contained(pack.root, raw['library']['path']), MAX_ASSET)) == raw['library']['sha256']
    report['artifact_inventory'] = []
    say('PACKAGE: saving the recorded inputs, source snapshots and bounded verified tile evidence')
    evidence = output/'evidence'
    evidence.mkdir()
    for relative, content in source_data.items():
        destination = evidence/'sources'/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
    used = 0
    entries = [pack.maps[c], *report['loading']['loaded_tiles']]
    for entry in entries:
        record = {'path': entry['path'], 'bytes': entry['bytes'], 'sha256': entry['sha256']}
        report['artifact_inventory'].append(record)
        if used + entry['bytes'] > ARCHIVE_TILE_BYTES:
            record['status'] = 'omitted_archive_byte_limit'
            continue
        try:
            data = stable_read(contained(pack.root, entry['path']), MAX_ASSET)
            if len(data) != entry['bytes'] or digest(data) != entry['sha256']:
                raise ValueError('Asset no longer matches the loaded manifest snapshot')
            destination = contained(evidence, 'navigation/'+entry['path'])
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
            used += len(data)
            record['status'] = 'captured_and_hash_verified'
        except (OSError, ValueError) as error:
            record['status'], record['error'] = 'unavailable', str(error)
    report['archive_asset_bytes'] = used
    report['limits'] = {'surface_capacity_per_xy': MAX_SURFACES, 'pair_combinations_per_interval': MAX_PAIR_TESTS,
                         'visits_per_graph_query': MAX_VISITS, 'link_checks_per_graph_query': MAX_LINK_CHECKS,
                         'load_and_query_seconds': args.max_seconds, 'archive_asset_bytes': ARCHIVE_TILE_BYTES}
    incomplete = (report['time_budget_reached'] or bool(report['loading']['unavailable_tiles']) or
                  any(not value['complete_for_loaded_graph'] for value in observations.values()) or
                  not all(report['input_recheck'].values()) or
                  any(item['status'] != 'captured_and_hash_verified' for item in report['artifact_inventory']))
    for pair in pairs:
        connectivity = pair['connectivity']
        incomplete |= connectivity.get('candidate_pairs_truncated', False)
        for test in connectivity['tests']:
            for profile in ('walk', 'tag_agnostic'):
                if profile in test and test[profile]['result_name'] in ('unknown_resource_limit', 'cancelled', 'invalid_reference', 'invalid_graph_link'):
                    incomplete = True
    report['status'] = 'partial' if incomplete else 'completed_bounded_probe'
    report['status_meaning'] = 'Diagnostic work status only; does not mean route success, correct physical labels, globally complete data, or a repaired route.'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--plan', type=Path, help='An explicit failed route-quality JSON; default: project/runtime/route-quality/last-plan.json')
    parser.add_argument('--max-seconds', type=float, default=60.0, help='Total tile loading and native query time budget (default: 60 seconds)')
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= 300:
        parser.error('--max-seconds must be between 1 and 300')
    root = args.project_root.resolve()
    if not (root/'adrive').is_dir():
        parser.error('Project root must contain adrive; save this script in PROJECT/tools or pass --project-root')
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    output = contained(root, 'runtime/route-surface-diagnostics/'+run_id)
    output.mkdir(parents=True, exist_ok=False)
    report = {'schema': 1, 'tool': VERSION, 'started_at_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'independent_native_context_loaded_graph_only', 'physical_root_cause_known': False,
              'global_connectivity_known': False, 'swimming_authorized': False,
              'production_policy_changed': False, 'status': 'running', 'output_directory': str(output)}
    say('ROUTE_SURFACE_DIAGNOSTICS_V1: starting read-only route evidence probe')
    say('REPORT = ' + str(output/'report.json'))
    exit_code = 0
    try:
        run(args, output, report)
    except (Exception, KeyboardInterrupt) as error:
        report['status'] = 'failed' if isinstance(error, Exception) else 'cancelled'
        report['error'] = type(error).__name__ + ': ' + str(error)
        say('DIAGNOSTIC_ERROR = ' + report['error'])
        exit_code = 1
    report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    write_json(output/'report.json', report)
    (output/'README.txt').write_text(
        'Read report.json first. This is a bounded read-only observation, not a route fix.\n'
        'Input plans and manifests are byte snapshots. A changed DLL after diagnostic installation is expected.\n'
        'Loaded tile keys were reconstructed; old reports did not record the complete original keyset.\n'
        'Native polygon refs are local to one context. Use tile SHA and tile/layer/poly key for comparisons.\n'
        'Graph disconnection applies only to the loaded graph. A limit or cancellation means UNKNOWN.\n'
        'A water navigation tag is not proof of physical water or permission to swim.\n'
        'A long completed route rejected by road quality is different from no route.\n'
        'No route execution, movement, tile repair or production policy change was performed.\n', encoding='utf-8')
    archive = output/'route-surface-diagnostics.zip'
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
        for path in sorted(output.rglob('*')):
            if path.is_file() and path != archive:
                bundle.write(path, path.relative_to(output).as_posix())
    say('STATUS = ' + report['status'])
    for item in report.get('findings', []):
        say('FINDING = ' + item['code'])
    say('ZIP = ' + str(archive))
    say('Upload this ZIP. A partial/failed diagnostic is retained with its exact reason.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
