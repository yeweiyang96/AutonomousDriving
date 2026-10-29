// Included after the legacy bridge implementation. Diagnostic functions never
// change GroundFilter, legacy query state, budgetReason or budgetStatus.
#include "surface_diagnostics_v2.h"
#include <new>
#include <unordered_set>

static_assert(sizeof(dtPolyRef) == 8, "Diagnostics require the pinned 64-bit refs");
static_assert(sizeof(AdCapabilitiesV2) == 48, "Unexpected capability ABI layout");
static_assert(sizeof(AdSurfaceV2) == 72, "Unexpected surface ABI layout");
static_assert(sizeof(AdSurfaceSummaryV2) == 48, "Unexpected surface summary layout");
static_assert(sizeof(AdReachabilityRequestV2) == 40, "Unexpected request layout");
static_assert(sizeof(AdReachabilityReportV2) == 80, "Unexpected reachability layout");

namespace ad_diag_v2 {
constexpr uint32_t MAX_VISITS = 1000000;
constexpr uint32_t MAX_LINK_CHECKS = 32000000;
constexpr uint32_t MAX_SURFACE_OUTPUT = 65535;

template<class T> bool validLayout(const T* value) {
    return value && value->struct_size == sizeof(T) && value->abi_version == AD_DIAG_ABI_V2;
}
template<class T> T blank() {
    T value{};
    value.struct_size = sizeof(T);
    value.abi_version = AD_DIAG_ABI_V2;
    return value;
}
static uint32_t loadedTiles(const Context* ctx) {
    const dtNavMesh& mesh = ctx->mesh;
    uint32_t count = 0;
    for (int i = 0; i < mesh.getMaxTiles(); ++i) {
        const dtMeshTile* tile = mesh.getTile(i);
        if (tile && tile->header) ++count;
    }
    return count;
}
static bool blockedSelf(const Context* ctx, dtPolyRef ref) {
    return ctx->filter.blocked.count({ref, ref}) != 0;
}
static bool blockedEdge(const Context* ctx, dtPolyRef a, dtPolyRef b) {
    return ctx->filter.blocked.count({a, b}) || ctx->filter.blocked.count({b, a});
}
static bool allowed(const Context* ctx, dtPolyRef ref, const dtMeshTile* tile,
                    const dtPoly* poly, uint32_t profile) {
    if (poly->getType() != DT_POLYTYPE_GROUND) return false;
    return profile == AD_DIAG_PROFILE_TAG_AGNOSTIC || ctx->filter.passFilter(ref, tile, poly);
}
} // namespace ad_diag_v2

API int ad_capabilities_v2(AdCapabilitiesV2* output) {
    using namespace ad_diag_v2;
    if (!validLayout(output)) return 1;
    auto value = blank<AdCapabilitiesV2>();
    value.features = AD_DIAG_FEATURE_SURFACES | AD_DIAG_FEATURE_REACHABILITY;
    value.poly_ref_bytes = sizeof(dtPolyRef);
    value.surface_record_size = sizeof(AdSurfaceV2);
    value.surface_summary_size = sizeof(AdSurfaceSummaryV2);
    value.reach_request_size = sizeof(AdReachabilityRequestV2);
    value.reach_report_size = sizeof(AdReachabilityReportV2);
    value.walk_include_flags = 0x11;
    value.walk_exclude_flags = 0x0e;
    value.path_node_capacity = 65535;
    *output = value;
    return 0;
}

API int ad_query_surfaces_v2(void* handle, const float* world, float heightTolerance,
                            AdSurfaceV2* output, uint32_t capacity,
                            AdSurfaceSummaryV2* summary) {
    using namespace ad_diag_v2;
    if (!handle || !world || !validLayout(summary) || (capacity && !output)
        || capacity > MAX_SURFACE_OUTPUT || !std::isfinite(world[0])
        || !std::isfinite(world[1]) || std::abs(world[0]) > 20000.f
        || std::abs(world[1]) > 20000.f || std::isinf(world[2])
        || (std::isfinite(world[2]) && std::abs(world[2]) > 20000.f)
        || !std::isfinite(heightTolerance) || heightTolerance < 0.f
        || heightTolerance > 20000.f) return 1;
    auto result = blank<AdSurfaceSummaryV2>();
    result.scope = AD_DIAG_SCOPE_LOADED_GRAPH;
    const auto ctx = static_cast<Context*>(handle);
    const dtNavMesh& mesh = ctx->mesh;
    const bool knownHeight = std::isfinite(world[2]);
    const float p[3] = {world[1], knownHeight ? world[2] : 0.f, world[0]};
    try {
        // Direct enumeration deliberately avoids both the ordinary flag filter
        // (which hides flags=0) and queryPolygons' hidden 32-layer tile limit.
        std::vector<AdSurfaceV2> found;
        found.reserve(std::min<uint32_t>(capacity, 128));
        for (int ti = 0; ti < mesh.getMaxTiles(); ++ti) {
            const dtMeshTile* tile = mesh.getTile(ti);
            if (!tile || !tile->header) continue;
            ++result.loaded_tiles;
            const auto h = tile->header;
            if (p[0] < h->bmin[0] || p[0] > h->bmax[0]
                || p[2] < h->bmin[2] || p[2] > h->bmax[2]) continue;
            ++result.tiles_considered;
            const dtPolyRef base = mesh.getPolyRefBase(tile);
            for (int pi = 0; pi < h->polyCount; ++pi) {
                const dtPoly* poly = &tile->polys[pi];
                if (poly->getType() != DT_POLYTYPE_GROUND) continue;
                ++result.polygons_examined;
                if (poly->vertCount < 3 || poly->vertCount > DT_VERTS_PER_POLYGON) {
                    ++result.height_failures;
                    continue;
                }
                float xmin = std::numeric_limits<float>::infinity();
                float xmax = -xmin, zmin = xmin, zmax = -xmin;
                bool validVertices = true;
                for (unsigned int vi = 0; vi < poly->vertCount; ++vi) {
                    if (poly->verts[vi] >= h->vertCount) { validVertices = false; break; }
                    const float* v = &tile->verts[poly->verts[vi] * 3];
                    xmin = std::min(xmin, v[0]); xmax = std::max(xmax, v[0]);
                    zmin = std::min(zmin, v[2]); zmax = std::max(zmax, v[2]);
                }
                if (!validVertices) { ++result.height_failures; continue; }
                if (p[0] < xmin || p[0] > xmax || p[2] < zmin || p[2] > zmax) continue;
                const dtPolyRef ref = base | dtPolyRef(pi);
                float nearest[3]{}; bool over = false;
                if (dtStatusFailed(ctx->query.closestPointOnPoly(ref, p, nearest, &over))
                    || !std::isfinite(nearest[1])) {
                    ++result.height_failures;
                    continue;
                }
                // Never relocate XY to the nearest surface. A polygon is only
                // a candidate if the exact requested XY lies above it.
                if (!over || (knownHeight && std::abs(nearest[1] - p[1]) > heightTolerance)) continue;
                auto row = blank<AdSurfaceV2>();
                row.poly_ref = ref;
                row.tile_x = h->x; row.tile_y = h->y; row.tile_layer = h->layer; row.poly_index = pi;
                row.north = world[0]; row.west = world[1]; row.height = nearest[1];
                row.area = poly->getArea(); row.flags = poly->flags; row.poly_type = poly->getType();
                row.walk_filter_pass = ctx->filter.passFilter(ref, tile, poly) ? 1u : 0u;
                row.blocked_self = blockedSelf(ctx, ref) ? 1u : 0u;
                row.semantic_evidence = AD_DIAG_EVIDENCE_NAV_TAG_ONLY;
                if (result.total_surfaces == std::numeric_limits<uint32_t>::max()) {
                    *summary = result;
                    return 2;
                }
                ++result.total_surfaces;
                // Count every match, but keep at most the caller's bounded
                // capacity. With capacity=0 this is an allocation-free count.
                // A truncated subset is not a complete or lowest-height list.
                if (found.size() < capacity) found.push_back(row);
            }
        }
        std::sort(found.begin(), found.end(), [](const AdSurfaceV2& a, const AdSurfaceV2& b) {
            if (a.height != b.height) return a.height < b.height;
            if (a.tile_x != b.tile_x) return a.tile_x < b.tile_x;
            if (a.tile_y != b.tile_y) return a.tile_y < b.tile_y;
            if (a.tile_layer != b.tile_layer) return a.tile_layer < b.tile_layer;
            return a.poly_index < b.poly_index;
        });
        result.written_surfaces = static_cast<uint32_t>(found.size());
        for (uint32_t i = 0; i < result.written_surfaces; ++i) output[i] = found[i];
        result.truncated = result.total_surfaces > capacity ? 1u : 0u;
        result.complete_for_loaded_graph = !result.truncated && !result.height_failures ? 1u : 0u;
        *summary = result;
        if (result.truncated) return 6;
        if (result.height_failures) return 9;
        return 0;
    } catch (const std::bad_alloc&) {
        *summary = result;
        return 2;
    } catch (...) {
        *summary = result;
        return 2;
    }
}

API int ad_query_reachability_v2(void* handle, const AdReachabilityRequestV2* request,
                                Cancel cancel, AdReachabilityReportV2* output) {
    using namespace ad_diag_v2;
    if (!handle || !validLayout(request) || !validLayout(output)
        || (request->profile != AD_DIAG_PROFILE_WALK && request->profile != AD_DIAG_PROFILE_TAG_AGNOSTIC)
        || !request->max_visits || request->max_visits > MAX_VISITS
        || !request->max_link_checks || request->max_link_checks > MAX_LINK_CHECKS
        || request->reserved) return 1;
    auto report = blank<AdReachabilityReportV2>();
    report.profile = request->profile;
    report.scope = AD_DIAG_SCOPE_LOADED_GRAPH;
    report.start_ref = request->start_ref; report.goal_ref = request->goal_ref;
    const auto ctx = static_cast<Context*>(handle);
    const dtNavMesh& mesh = ctx->mesh;
    report.loaded_tiles = loadedTiles(ctx);
    auto finish = [&](uint32_t state) {
        report.result = state;
        // Even graph exhaustion says nothing about unloaded manifest assets.
        report.global_connectivity_known = 0;
        *output = report;
        return 0;
    };
    const dtMeshTile *startTile = nullptr, *goalTile = nullptr;
    const dtPoly *startPoly = nullptr, *goalPoly = nullptr;
    if (dtStatusFailed(mesh.getTileAndPolyByRef(dtPolyRef(request->start_ref), &startTile, &startPoly))
        || dtStatusFailed(mesh.getTileAndPolyByRef(dtPolyRef(request->goal_ref), &goalTile, &goalPoly)))
        return finish(AD_DIAG_INVALID_REF);
    if (!allowed(ctx, request->start_ref, startTile, startPoly, request->profile)
        || !allowed(ctx, request->goal_ref, goalTile, goalPoly, request->profile)
        || blockedSelf(ctx, request->start_ref) || blockedSelf(ctx, request->goal_ref))
        return finish(AD_DIAG_ENDPOINT_EXCLUDED);
    try {
        // Separate bounded scratch state; the Detour path node pool is untouched.
        std::unordered_set<dtPolyRef> discovered;
        std::vector<dtPolyRef> queue;
        const auto reserve = std::min<uint32_t>(request->max_visits, 65536);
        discovered.reserve(reserve); queue.reserve(reserve);
        queue.push_back(dtPolyRef(request->start_ref));
        discovered.insert(dtPolyRef(request->start_ref));
        report.discovered_polygons = 1;
        for (size_t head = 0; head < queue.size(); ++head) {
            if (cancel && cancel()) return finish(AD_DIAG_CANCELLED);
            const dtPolyRef ref = queue[head];
            ++report.visited_polygons;
            if (ref == dtPolyRef(request->goal_ref)) return finish(AD_DIAG_REACHABLE);
            const dtMeshTile* tile = nullptr; const dtPoly* poly = nullptr;
            if (dtStatusFailed(mesh.getTileAndPolyByRef(ref, &tile, &poly)))
                return finish(AD_DIAG_CORRUPT_LINK);
            if (poly->vertCount > DT_VERTS_PER_POLYGON) return finish(AD_DIAG_CORRUPT_LINK);
            bool edgeLinked[DT_VERTS_PER_POLYGON]{};
            uint32_t linksInChain = 0;
            for (unsigned int li = poly->firstLink; li != DT_NULL_LINK;) {
                if (li >= static_cast<unsigned int>(tile->header->maxLinkCount)
                    || ++linksInChain > static_cast<uint32_t>(tile->header->maxLinkCount))
                    return finish(AD_DIAG_CORRUPT_LINK);
                if (report.link_checks >= request->max_link_checks) {
                    report.link_limit_reached = 1;
                    return finish(AD_DIAG_VISIT_LIMIT);
                }
                ++report.link_checks;
                const dtLink& link = tile->links[li];
                li = link.next;
                if (!link.ref) continue;
                if (link.edge < poly->vertCount) edgeLinked[link.edge] = true;
                const dtMeshTile* nextTile = nullptr; const dtPoly* nextPoly = nullptr;
                if (dtStatusFailed(mesh.getTileAndPolyByRef(link.ref, &nextTile, &nextPoly)))
                    return finish(AD_DIAG_CORRUPT_LINK);
                if (!allowed(ctx, link.ref, nextTile, nextPoly, request->profile)) {
                    ++report.filtered_links; continue;
                }
                if (blockedSelf(ctx, link.ref) || blockedEdge(ctx, ref, link.ref)) {
                    ++report.blocked_links; continue;
                }
                if (discovered.count(link.ref)) continue;
                if (discovered.size() >= request->max_visits) return finish(AD_DIAG_VISIT_LIMIT);
                discovered.insert(link.ref); queue.push_back(link.ref);
                ++report.discovered_polygons;
                // The valid directed link already proves reachability; do not
                // lose that witness if a later unrelated neighbor hits a limit.
                if (link.ref == dtPolyRef(request->goal_ref)) return finish(AD_DIAG_REACHABLE);
            }
            for (unsigned int i = 0; i < poly->vertCount; ++i)
                if ((poly->neis[i] & DT_EXT_LINK) && !edgeLinked[i]) ++report.unresolved_external_edges;
        }
        report.traversal_complete = 1;
        return finish(AD_DIAG_DISCONNECTED_IN_LOADED_GRAPH);
    } catch (const std::bad_alloc&) {
        *output = report;
        return 2;
    } catch (...) {
        *output = report;
        return 2;
    }
}
