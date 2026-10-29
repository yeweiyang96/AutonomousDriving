#pragma once
// Additive diagnostic ABI. No movement mode or geometry is authorized here.
#include <cstdint>

enum : uint32_t {
    AD_DIAG_ABI_V2 = 2,
    AD_DIAG_FEATURE_SURFACES = 1,
    AD_DIAG_FEATURE_REACHABILITY = 2,
    AD_DIAG_PROFILE_WALK = 1,
    AD_DIAG_PROFILE_TAG_AGNOSTIC = 2,
    AD_DIAG_SCOPE_LOADED_GRAPH = 1,
    AD_DIAG_EVIDENCE_NAV_TAG_ONLY = 1,
    AD_DIAG_REACHABLE = 1,
    AD_DIAG_DISCONNECTED_IN_LOADED_GRAPH = 2,
    AD_DIAG_VISIT_LIMIT = 3,
    AD_DIAG_CANCELLED = 4,
    AD_DIAG_ENDPOINT_EXCLUDED = 5,
    AD_DIAG_INVALID_REF = 6,
    AD_DIAG_CORRUPT_LINK = 7
};

struct AdCapabilitiesV2 {
    uint32_t struct_size, abi_version, features, poly_ref_bytes;
    uint32_t surface_record_size, surface_summary_size;
    uint32_t reach_request_size, reach_report_size;
    uint32_t walk_include_flags, walk_exclude_flags, path_node_capacity, reserved;
};

struct AdSurfaceV2 {
    uint32_t struct_size, abi_version;
    uint64_t poly_ref;
    int32_t tile_x, tile_y, tile_layer, poly_index;
    float north, west, height;
    uint32_t area, flags, poly_type, walk_filter_pass, blocked_self;
    uint32_t semantic_evidence, reserved;
};

struct AdSurfaceSummaryV2 {
    uint32_t struct_size, abi_version, total_surfaces, written_surfaces;
    uint32_t loaded_tiles, tiles_considered, polygons_examined, height_failures;
    uint32_t truncated, scope, complete_for_loaded_graph, reserved;
};

struct AdReachabilityRequestV2 {
    uint32_t struct_size, abi_version;
    uint64_t start_ref, goal_ref;
    uint32_t profile, max_visits, max_link_checks, reserved;
};

struct AdReachabilityReportV2 {
    uint32_t struct_size, abi_version, result, profile;
    uint32_t visited_polygons, discovered_polygons, link_checks, loaded_tiles;
    uint32_t unresolved_external_edges, filtered_links, blocked_links, scope;
    uint64_t start_ref, goal_ref;
    uint32_t traversal_complete, global_connectivity_known, link_limit_reached, reserved;
};

// Return codes for these additive APIs only:
// 0 = valid result (inspect reachability.result), 1 = invalid arguments/layout,
// 2 = allocation failure, 6 = surface output truncated, 9 = surface height failure.
// The exported declarations live at the end of bridge.cpp via the .inc file.
// Callers must verify the capability sizes before allocating result arrays.
