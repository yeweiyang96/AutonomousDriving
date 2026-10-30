// Additive bounded-domain path query. Include after the existing bridge.cpp
// and surface_diagnostics_v2.inc; the legacy ad_path implementation is unchanged.
//
// World order is [north, west, height]. max_length_xy is a SEARCH DOMAIN
// parameter, not a guarantee about returned path length. The caller must still
// apply its length, ordered-road-distance and backtracking quality checks.
//
// If a continuous XY curve from A to B has length <= L, every point P on it
// satisfies |A-P| + |P-B| <= L. For a polygon's XY AABB Q, d(A,Q)+d(B,Q) is
// no greater than |A-P|+|P-B| for any P in that polygon. Rejecting Q only when
// this lower bound exceeds L plus a conservative numerical margin therefore
// retains a superset of the polygons a qualifying curve could traverse.
// AABB distances may have different closest points; the domain is deliberately
// loose. This is not an exact continuous shortest-path or physical-collision
// proof. Result 10 specifically means that a completed search finalized a
// partial/non-goal corridor in the loaded filtered graph and search domain.
// Result 5 remains an operation/geometry failure, not disconnection evidence.
// Node/expansion/output limits retain result 6 and ad_budget_reason details.
//
// ABI: int ad_path_bounded_v3(void*, const float*, const float*, int, float,
//                            Cancel, float*, uint64_t*, int, int*, int*);
// All non-callback pointers must be valid. A handle must be a live Context.
// Calls on a Context are serialized with loading and all other queries.
// Capacity is 1..65535, budget >= 0, L is finite and nonnegative; XY and finite
// height are within +/-20000. NaN height retains the legacy unknown-height rule.
// L below straight XY distance by more than the margin is invalid (result 1).
// A nonzero return leaves output count zero; expansions still report work done.

#include <new>

namespace adrive_path_search_bounds_v3 {

// 0.05 yards is conservative relative to float32 rounding at the supported
// +/-20000 world coordinates. It also prevents boundary equality being lost
// to closest-point/portal arithmetic. Final route quality keeps its own limit.
constexpr double kMarginXY = 0.05;

static bool validPoint(const float* p) {
    if (!p || !std::isfinite(p[0]) || !std::isfinite(p[1]) || std::isinf(p[2]))
        return false;
    return std::abs(p[0]) <= 20000.f && std::abs(p[1]) <= 20000.f &&
           (!std::isfinite(p[2]) || std::abs(p[2]) <= 20000.f);
}

struct LengthDomainFilter final : GroundFilter {
    double startXY[2], goalXY[2], limit;

    LengthDomainFilter(const GroundFilter& original, const float* start,
                       const float* goal, float maxLengthXY)
        : GroundFilter(original), limit(double(maxLengthXY) + kMarginXY) {
        startXY[0] = start[0]; startXY[1] = start[2];
        goalXY[0] = goal[0]; goalXY[1] = goal[2];
    }

    bool passFilter(dtPolyRef ref, const dtMeshTile* tile,
                    const dtPoly* poly) const override {
        // Copies and preserves the complete original filter, including its
        // area costs and block set. No water/offmesh permission is introduced.
        if (!GroundFilter::passFilter(ref, tile, poly)) return false;
        double low[2] = {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity()};
        double high[2] = {-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity()};
        for (int vertex = 0; vertex < poly->vertCount; ++vertex) {
            const float* p = &tile->verts[3 * poly->verts[vertex]];
            low[0] = std::min(low[0], double(p[0]));
            high[0] = std::max(high[0], double(p[0]));
            low[1] = std::min(low[1], double(p[2]));
            high[1] = std::max(high[1], double(p[2]));
        }
        const auto distance = [&](const double* p) {
            const double dx = std::max(std::max(low[0] - p[0], p[0] - high[0]), 0.0);
            const double dy = std::max(std::max(low[1] - p[1], p[1] - high[1]), 0.0);
            return std::sqrt(dx * dx + dy * dy);
        };
        return distance(startXY) + distance(goalXY) <= limit;
    }
};

static int query(Context* ctx, const float* start, const float* goal,
                 int budget, float maxLengthXY, Cancel cancel,
                 float* output, uint64_t* refs, int capacity,
                 int* count, int* expanded, const GroundFilter* domain = nullptr) {
    Surface a, b;
    int rc = resolve(ctx, start, a); if (rc) return rc;
    rc = resolve(ctx, goal, b); if (rc) return rc;
    // Endpoint resolution is exactly the old strict over-polygon resolution.
    // A two-dimensional road cannot silently select a different height layer.
    LengthDomainFilter filter(ctx->filter, a.p, b.p, maxLengthXY);
    auto status = ctx->query.initSlicedFindPath(a.ref, b.ref, a.p, b.p, domain ? domain : &filter);
    while (dtStatusInProgress(status)) {
        if (cancel && cancel()) return 7;
        if (*expanded >= budget) return budgetResult(ctx, EXPANSION_LIMIT, status);
        int done = 0;
        status = ctx->query.updateSlicedFindPath(std::min(256, budget - *expanded), &done);
        *expanded += done;
    }
    if (dtStatusFailed(status)) return 5;
    if (dtStatusDetail(status, DT_OUT_OF_NODES))
        return budgetResult(ctx, NODE_POOL_EXHAUSTED, status);

    std::vector<dtPolyRef> corridor(65535); int length = 0;
    status = ctx->query.finalizeSlicedFindPath(corridor.data(), &length, int(corridor.size()));
    if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
        return budgetResult(ctx, CORRIDOR_CAPACITY, status);
    if (dtStatusFailed(status)) return 5;
    if (dtStatusDetail(status, DT_PARTIAL_RESULT)) return 10;
    if (!length) return 5;
    if (corridor[length - 1] != b.ref) return 10;
    for (int i = 0; i < length; ++i) {
        if (ctx->filter.blocked.count({corridor[i], corridor[i]})) return 8;
        if (i && (ctx->filter.blocked.count({corridor[i - 1], corridor[i]}) ||
                  ctx->filter.blocked.count({corridor[i], corridor[i - 1]}))) return 8;
    }

    std::vector<float> straight(size_t(capacity) * 3);
    std::vector<unsigned char> flags(capacity);
    status = ctx->query.findStraightPath(a.p, b.p, corridor.data(), length,
        straight.data(), flags.data(), refs, count, capacity, DT_STRAIGHTPATH_ALL_CROSSINGS);
    if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
        return budgetResult(ctx, PATH_OUTPUT_CAPACITY, status);
    if (dtStatusFailed(status) || *count < 1 || !(flags[*count - 1] & DT_STRAIGHTPATH_END)) return 5;
    for (int i = 0; i < *count; ++i) {
        if (flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) return 8;
        if (!refs[i]) refs[i] = b.ref;
        auto p = &straight[i * 3]; float height = p[1];
        if (dtStatusFailed(ctx->query.getPolyHeight(refs[i], p, &height))) {
            float nearest[3]; bool over = false;
            if (dtStatusFailed(ctx->query.closestPointOnPoly(refs[i], p, nearest, &over)) ||
                dtVdist2DSqr(p, nearest) > .0001f) return 5;
            height = nearest[1];
        }
        output[i * 3] = p[2];
        output[i * 3 + 1] = p[0];
        output[i * 3 + 2] = height;
    }
    return 0;
}

} // namespace adrive_path_search_bounds_v3

API int ad_path_bounded_v3(void* handle, const float* start, const float* goal,
                          int budget, float max_length_xy, Cancel cancel,
                          float* output, uint64_t* refs, int capacity,
                          int* count, int* expanded) {
    if (count) *count = 0;
    if (expanded) *expanded = 0;
    if (!handle || !count || !expanded) return 1;
    auto ctx = static_cast<Context*>(handle);
    ctx->budgetReason = 0; ctx->budgetStatus = 0;
    using namespace adrive_path_search_bounds_v3;
    if (!output || !refs || capacity < 1 || capacity > 65535 || budget < 0 ||
        !validPoint(start) || !validPoint(goal) ||
        !std::isfinite(max_length_xy) || max_length_xy < 0.f) return 1;
    const double dx = double(start[0]) - goal[0];
    const double dy = double(start[1]) - goal[1];
    if (std::sqrt(dx * dx + dy * dy) > double(max_length_xy) + kMarginXY) return 1;
    try {
        const int result = query(ctx, start, goal, budget, max_length_xy, cancel,
                                 output, refs, capacity, count, expanded);
        if (result) *count = 0;
        return result;
    } catch (const std::bad_alloc&) {
        *count = 0;
        return 2;
    }
}
