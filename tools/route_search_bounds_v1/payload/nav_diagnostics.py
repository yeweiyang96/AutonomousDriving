"""Read-only native surface and loaded-graph diagnostics.

This module never changes navigation filters, blocks, tile data or movement
modes. A water tag is not proof of physical water or permission to swim.
All refs are valid only inside the supplied native context. Calls on one
context must be serialized with tile loading and path planning.
"""
from __future__ import annotations

import ctypes as C
import math
from pathlib import Path

ABI_VERSION = 2
U32, I32, U64 = C.c_uint32, C.c_int32, C.c_uint64
CANCEL = C.CFUNCTYPE(C.c_int)


class Capabilities(C.Structure):
    _fields_ = [(name, U32) for name in (
        'struct_size', 'abi_version', 'features', 'poly_ref_bytes',
        'surface_record_size', 'surface_summary_size', 'reach_request_size',
        'reach_report_size', 'walk_include_flags', 'walk_exclude_flags',
        'path_node_capacity', 'reserved')]


class Surface(C.Structure):
    _fields_ = [
        ('struct_size', U32), ('abi_version', U32), ('poly_ref', U64),
        ('tile_x', I32), ('tile_y', I32), ('tile_layer', I32), ('poly_index', I32),
        ('north', C.c_float), ('west', C.c_float), ('height', C.c_float),
        ('area', U32), ('flags', U32), ('poly_type', U32),
        ('walk_filter_pass', U32), ('blocked_self', U32),
        ('semantic_evidence', U32), ('reserved', U32),
    ]


class SurfaceSummary(C.Structure):
    _fields_ = [(name, U32) for name in (
        'struct_size', 'abi_version', 'total_surfaces', 'written_surfaces',
        'loaded_tiles', 'tiles_considered', 'polygons_examined', 'height_failures',
        'truncated', 'scope', 'complete_for_loaded_graph', 'reserved')]


class ReachabilityRequest(C.Structure):
    _fields_ = [
        ('struct_size', U32), ('abi_version', U32),
        ('start_ref', U64), ('goal_ref', U64),
        ('profile', U32), ('max_visits', U32), ('max_link_checks', U32), ('reserved', U32),
    ]


class ReachabilityReport(C.Structure):
    _fields_ = [(name, U32) for name in (
        'struct_size', 'abi_version', 'result', 'profile', 'visited_polygons',
        'discovered_polygons', 'link_checks', 'loaded_tiles',
        'unresolved_external_edges', 'filtered_links', 'blocked_links', 'scope')]
    _fields_ += [('start_ref', U64), ('goal_ref', U64)]
    _fields_ += [(name, U32) for name in (
        'traversal_complete', 'global_connectivity_known', 'link_limit_reached', 'reserved')]


RESULTS = {
    1: 'reachable_in_loaded_graph',
    2: 'disconnected_in_loaded_graph',
    3: 'unknown_resource_limit',
    4: 'cancelled',
    5: 'endpoint_excluded',
    6: 'invalid_reference',
    7: 'invalid_graph_link',
}
PROFILES = {'walk': 1, 'tag_agnostic': 2}
NAV_AREAS = {11: 'ground_tag', 10: 'steep_tag', 9: 'water_tag',
             8: 'magma_or_slime_tag', 7: 'road_tag'}


class DiagnosticError(RuntimeError):
    def __init__(self, operation, code):
        self.operation, self.code = operation, code
        super().__init__('%s failed with diagnostic return code %d' % (operation, code))


def initialized(record_type):
    record = record_type()
    record.struct_size = C.sizeof(record_type)
    record.abi_version = ABI_VERSION
    return record


def record_dict(record):
    return {name: getattr(record, name) for name, _ in record._fields_}


def _handle(value):
    if isinstance(value, C.c_void_p):
        if value.value is None:
            raise ValueError('A live native context handle is required')
        return value
    if type(value) is int and value > 0:
        return C.c_void_p(value)
    raise ValueError('A live native context handle is required')


def _unsigned(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('%s must be an integer in [%d, %d]' % (name, minimum, maximum))
    return value


class Diagnostics:
    """Bind only the additive diagnostic exports on a CDLL or explicit path."""

    def __init__(self, library):
        self.dll = library if isinstance(library, C.CDLL) else C.CDLL(str(Path(library).resolve(strict=True)))
        specs = {
            'ad_capabilities_v2': ([C.POINTER(Capabilities)], C.c_int),
            'ad_query_surfaces_v2': ([C.c_void_p, C.POINTER(C.c_float), C.c_float,
                                      C.POINTER(Surface), U32, C.POINTER(SurfaceSummary)], C.c_int),
            'ad_query_reachability_v2': ([C.c_void_p, C.POINTER(ReachabilityRequest), CANCEL,
                                          C.POINTER(ReachabilityReport)], C.c_int),
        }
        for name, (args, result) in specs.items():
            try:
                function = getattr(self.dll, name)
            except AttributeError as error:
                raise RuntimeError('Loaded DLL has no %s; diagnostic ABI v2 is unavailable' % name) from error
            function.argtypes, function.restype = args, result
        caps = initialized(Capabilities)
        rc = self.dll.ad_capabilities_v2(C.byref(caps))
        if rc:
            raise DiagnosticError('ad_capabilities_v2', rc)
        expected = {
            'struct_size': C.sizeof(Capabilities), 'abi_version': ABI_VERSION,
            'poly_ref_bytes': C.sizeof(U64), 'surface_record_size': C.sizeof(Surface),
            'surface_summary_size': C.sizeof(SurfaceSummary),
            'reach_request_size': C.sizeof(ReachabilityRequest),
            'reach_report_size': C.sizeof(ReachabilityReport),
        }
        for name, value in expected.items():
            if getattr(caps, name) != value:
                raise RuntimeError('Diagnostic ABI mismatch for %s: DLL=%d Python=%d' %
                                   (name, getattr(caps, name), value))
        if caps.features & 3 != 3:
            raise RuntimeError('Loaded diagnostic DLL lacks required read-only capabilities')
        self.capabilities = record_dict(caps)

    def surfaces(self, handle, world, *, height_tolerance=2.5, capacity=256):
        """Return each actual overlying ordinary polygon, including excluded tags.

        No height deduplication or nearest-point relocation is performed. A
        nonzero diagnostic rc is retained; truncated output is never complete.
        """
        handle = _handle(handle)
        if len(world) != 3:
            raise ValueError('world must be (north, west, height_or_None)')
        north, west = float(world[0]), float(world[1])
        height = math.nan if world[2] is None else float(world[2])
        if not math.isfinite(north) or not math.isfinite(west) or math.isinf(height):
            raise ValueError('XY must be finite; height may be finite, None or NaN')
        tolerance = float(height_tolerance)
        if not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError('height_tolerance must be finite and nonnegative')
        capacity = _unsigned(capacity, 'capacity', 0, 65535)
        point = (C.c_float * 3)(north, west, height)
        output = (Surface * capacity)() if capacity else None
        summary = initialized(SurfaceSummary)
        rc = self.dll.ad_query_surfaces_v2(handle, point, tolerance, output, capacity, C.byref(summary))
        if rc not in (0, 6, 9):
            raise DiagnosticError('ad_query_surfaces_v2', rc)
        if summary.written_surfaces > capacity:
            raise RuntimeError('DLL returned an invalid surface count')
        records = []
        for item in list(output or ())[:summary.written_surfaces]:
            if item.struct_size != C.sizeof(Surface) or item.abi_version != ABI_VERSION:
                raise RuntimeError('DLL returned an incompatible surface record')
            record = record_dict(item)
            record.update({
                'poly_ref_hex': '0x%016x' % item.poly_ref,
                'native_ref_context_only': True,
                'nav_area_label': NAV_AREAS.get(item.area, 'unknown_nav_tag'),
                'physical_kind': 'UNKNOWN',
                'classification_evidence': 'nav_tag_only',
                'swimming_authorized': False,
            })
            records.append(record)
        return {
            'return_code': rc,
            'complete_for_loaded_graph': bool(rc == 0 and summary.complete_for_loaded_graph
                                               and not summary.truncated and not summary.height_failures),
            'scope': 'loaded_graph_only',
            'global_coverage_known': False,
            'summary': record_dict(summary), 'surfaces': records,
        }

    def reachability(self, handle, start_ref, goal_ref, *, profile='walk',
                     max_visits=100000, max_link_checks=2000000, cancel=None):
        """Check graph reachability, never movement execution or shortest distance."""
        handle = _handle(handle)
        if profile not in PROFILES:
            raise ValueError('profile must be walk or tag_agnostic')
        start_ref = _unsigned(start_ref, 'start_ref', 0, 2**64 - 1)
        goal_ref = _unsigned(goal_ref, 'goal_ref', 0, 2**64 - 1)
        max_visits = _unsigned(max_visits, 'max_visits', 1, 1000000)
        max_link_checks = _unsigned(max_link_checks, 'max_link_checks', 1, 32000000)
        request = initialized(ReachabilityRequest)
        request.start_ref, request.goal_ref = start_ref, goal_ref
        request.profile = PROFILES[profile]
        request.max_visits, request.max_link_checks = max_visits, max_link_checks
        report = initialized(ReachabilityReport)
        callback_errors = []

        def should_cancel():
            try:
                return int(bool(cancel and cancel()))
            except BaseException as error:
                callback_errors.append(error)
                return 1

        callback = CANCEL(should_cancel) if cancel is not None else CANCEL()
        rc = self.dll.ad_query_reachability_v2(handle, C.byref(request), callback, C.byref(report))
        if callback_errors:
            raise callback_errors[0]
        if rc:
            raise DiagnosticError('ad_query_reachability_v2', rc)
        value = record_dict(report)
        value.update({
            'return_code': rc, 'result_name': RESULTS.get(report.result, 'unknown_result'),
            'scope_name': 'loaded_graph_only', 'profile_name': profile,
            'native_ref_context_only': True,
            'meaning': ('tag_agnostic_topology_witness_only' if profile == 'tag_agnostic'
                        else 'walk_filter_graph_reachability_only'),
            'complete_route_verified': False, 'swimming_authorized': False,
        })
        return value
