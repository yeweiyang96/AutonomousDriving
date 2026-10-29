// Diagnostic build: exposes the exact native budget reason; search behavior is unchanged.
// Autonomous Driving's small C ABI over the pinned, unmodified Detour library.
// World coordinates: north, west, height -> Detour: west, height, north.
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif

// Result codes: 0 success, 1 format, 2 allocation, 3 outside mesh,
// 4 ambiguous height, 5 unreachable/partial, 6 budget, 7 cancelled, 8 blocked.
constexpr uint32_t MMAGIC = 0x4d4d4150, MVERSION = 16;
struct MapHeader { uint32_t magic, version; dtNavMeshParams params; uint32_t links; };
struct TileHeader { uint32_t magic, dtVersion, version, size; uint8_t liquids, padding[3]; };
static_assert(sizeof(MapHeader) == 40 && sizeof(TileHeader) == 20);

struct GroundFilter : dtQueryFilter {
    std::set<std::pair<dtPolyRef, dtPolyRef>> blocked;
    GroundFilter() { setIncludeFlags(0x11); setExcludeFlags(0x0e); }
    bool passFilter(dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) const override {
        return poly->getType() == DT_POLYTYPE_GROUND && dtQueryFilter::passFilter(ref, tile, poly);
    }
    float getCost(const float* a, const float* b, dtPolyRef pr, const dtMeshTile* pt, const dtPoly* pp,
                  dtPolyRef cr, const dtMeshTile* ct, const dtPoly* cp,
                  dtPolyRef nr, const dtMeshTile* nt, const dtPoly* np) const override {
        if (blocked.count({cr, nr}) || blocked.count({nr, cr}) || blocked.count({cr, cr}))
            return std::numeric_limits<float>::infinity();
        return dtQueryFilter::getCost(a,b,pr,pt,pp,cr,ct,cp,nr,nt,np);
    }
};
struct Context {
    dtNavMesh mesh;
    dtNavMeshQuery query;
    GroundFilter filter;
    int budgetReason = 0;
    dtStatus budgetStatus = 0;
};
// Diagnostic reasons only. Existing ABI result codes (including 6) are unchanged.
enum BudgetReason {
    EXPANSION_LIMIT = 1, NODE_POOL_EXHAUSTED = 2,
    CORRIDOR_CAPACITY = 3, PATH_OUTPUT_CAPACITY = 4,
    SURFACE_QUERY_FAILED = 5, SURFACE_POLYGON_CAPACITY = 6,
    SURFACE_HEIGHT_CAPACITY = 7
};
static int budgetResult(Context* ctx, int reason, dtStatus status = 0) {
    ctx->budgetReason = reason;
    ctx->budgetStatus = status;
    return 6;
}
// Optional read-only diagnostics for the most recent operation on this context.
API int ad_budget_reason(void* handle, uint32_t* status) {
    const auto ctx = static_cast<Context*>(handle);
    if (status) *status = ctx->budgetStatus;
    return ctx->budgetReason;
}
struct Surface { dtPolyRef ref; float p[3]; };

static int surfaces(Context* ctx, const float* world, std::vector<Surface>& out) {
    const bool known = std::isfinite(world[2]);
    float p[3] = {world[1], known ? world[2] : 0.f, world[0]};
    const float extent[3] = {.05f, known ? 2.5f : 20000.f, .05f};
    dtPolyRef refs[512]; int count = 0;
    auto status = ctx->query.queryPolygons(p, extent, &ctx->filter, refs, &count, 512);
    if (dtStatusFailed(status)) return budgetResult(ctx, SURFACE_QUERY_FAILED, status);
    if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL)) return budgetResult(ctx, SURFACE_POLYGON_CAPACITY, status);
    for (int i=0; i<count; ++i) {
        Surface s{}; s.ref = refs[i]; bool over = false;
        if (dtStatusFailed(ctx->query.closestPointOnPoly(s.ref, p, s.p, &over))) continue;
        // Endpoints must actually be over walkable ground. No nearest-free-cell
        // relocation or unchecked connector from the requested XY coordinate.
        if (!over || (known && std::abs(s.p[1]-p[1]) > 2.5f)) continue;
        if (std::none_of(out.begin(), out.end(), [&](const Surface& q){return std::abs(q.p[1]-s.p[1]) < .35f;}))
            out.push_back(s);
    }
    std::sort(out.begin(), out.end(), [](const Surface& a, const Surface& b){return a.p[1]<b.p[1];});
    return out.empty() ? 3 : 0;
}
static int resolve(Context* ctx, const float* p, Surface& out) {
    std::vector<Surface> found;
    int rc = surfaces(ctx,p,found); if (rc) return rc;
    if (std::isfinite(p[2])) {
        std::sort(found.begin(),found.end(),[&](const Surface& a,const Surface& b){
            return std::abs(a.p[1]-p[2])<std::abs(b.p[1]-p[2]);});
        if (found.size()>1 && std::abs(std::abs(found[0].p[1]-p[2])-std::abs(found[1].p[1]-p[2])) < .35f)
            return 4;
    } else if (found.size()!=1) return 4;
    out=found[0]; return 0;
}

API int ad_create(const unsigned char* data, int size, void** output) {
    *output=nullptr;
    if(size<int(sizeof(MapHeader))) return 1;
    MapHeader h; std::memcpy(&h,data,sizeof h);
    if(h.magic!=MMAGIC || h.version!=MVERSION || h.params.maxTiles<1 || h.params.maxTiles>8192
       // The 64-bit-reference Forever generator stores 1u<<31 in this signed
       // field. Detour ignores maxPolys with DT_POLYREF64; accept that exact
       // representation as well as small fixture values, not arbitrary negatives.
       || (h.params.maxPolys!=std::numeric_limits<int>::min() && (h.params.maxPolys<1 || h.params.maxPolys>1048576))
       || !std::isfinite(h.params.tileWidth) || !std::isfinite(h.params.tileHeight)
       || h.params.tileWidth<=0 || h.params.tileHeight<=0) return 1;
    for(float v:h.params.orig) if(!std::isfinite(v)) return 1;
    auto ctx=std::make_unique<Context>();
    if(dtStatusFailed(ctx->mesh.init(&h.params)) || dtStatusFailed(ctx->query.init(&ctx->mesh,65535))) return 2;
    *output=ctx.release(); return 0;
}
API void ad_destroy(void* handle) { delete static_cast<Context*>(handle); }
API int ad_add(void* handle,const unsigned char* data,int size) {
    if(size<int(sizeof(TileHeader)+sizeof(dtMeshHeader))) return 1;
    TileHeader h; dtMeshHeader m;
    std::memcpy(&h,data,sizeof h); std::memcpy(&m,data+sizeof h,sizeof m);
    if(h.magic!=MMAGIC || h.version!=MVERSION || h.dtVersion!=DT_NAVMESH_VERSION
       || h.size!=size-sizeof h || m.magic!=DT_NAVMESH_MAGIC || m.version!=DT_NAVMESH_VERSION) return 1;
    const int counts[]={m.vertCount,m.polyCount,m.maxLinkCount,m.detailMeshCount,m.detailVertCount,
                        m.detailTriCount,m.bvNodeCount,m.offMeshConCount};
    const size_t strides[]={sizeof(float)*3,sizeof(dtPoly),sizeof(dtLink),sizeof(dtPolyDetail),
                            sizeof(float)*3,4,sizeof(dtBVNode),sizeof(dtOffMeshConnection)};
    size_t expected=dtAlign4(sizeof(dtMeshHeader));
    for(int i=0;i<8;++i) {
        if(counts[i]<0 || counts[i]>4000000) return 1;
        expected+=(size_t(counts[i])*strides[i]+3)&~size_t(3);
    }
    if(expected!=h.size || !m.vertCount || !m.polyCount || !m.maxLinkCount) return 1;
    // Reject NPC-style tall-step meshes: the client never jumps over fences.
    if(!std::isfinite(m.walkableClimb) || m.walkableClimb>.8f || m.walkableRadius<.5f) return 1;
    auto copy=static_cast<unsigned char*>(dtAlloc(h.size,DT_ALLOC_PERM)); if(!copy) return 2;
    std::memcpy(copy,data+sizeof h,h.size);
    auto rc=static_cast<Context*>(handle)->mesh.addTile(copy,h.size,DT_TILE_FREE_DATA,0,nullptr);
    if(dtStatusFailed(rc)) {dtFree(copy);return 1;} return 0;
}
API int ad_surfaces(void* handle,const float* point,float* heights,int capacity,int* count) {
    auto ctx=static_cast<Context*>(handle);
    ctx->budgetReason=0; ctx->budgetStatus=0;
    std::vector<Surface> out; int rc=surfaces(ctx,point,out);
    *count=int(out.size()); if(rc) return rc;
    if(*count>capacity) return budgetResult(ctx, SURFACE_HEIGHT_CAPACITY);
    for(int i=0;i<*count;++i) heights[i]=out[i].p[1]; return 0;
}
API int ad_poly_key(void* handle,uint64_t ref,int* key) {
    const dtMeshTile* tile=nullptr; const dtPoly* poly=nullptr;
    if(dtStatusFailed(static_cast<Context*>(handle)->mesh.getTileAndPolyByRef(ref,&tile,&poly))) return 1;
    key[0]=tile->header->x; key[1]=tile->header->y; key[2]=int(poly-tile->polys); return 0;
}
API int ad_block(void* handle,const int* a,const int* b) {
    auto ctx=static_cast<Context*>(handle);
    auto ref=[&](const int* key)->dtPolyRef {
        const auto tile=ctx->mesh.getTileAt(key[0],key[1],0);
        if(!tile || key[2]<0 || key[2]>=tile->header->polyCount) return 0;
        return ctx->mesh.getPolyRefBase(tile)|dtPolyRef(key[2]);
    };
    auto ar=ref(a),br=ref(b); if(ar && br) ctx->filter.blocked.insert({ar,br}); return 0;
}
using Cancel=int(*)();
API int ad_path(void* handle,const float* start,const float* goal,int budget,Cancel cancel,
                float* output,uint64_t* refs,int capacity,int* count,int* expanded) {
    *count=0; *expanded=0; auto ctx=static_cast<Context*>(handle); Surface a,b;
    ctx->budgetReason=0; ctx->budgetStatus=0;
    int rc=resolve(ctx,start,a); if(rc) return rc;
    rc=resolve(ctx,goal,b); if(rc) return rc;
    auto status=ctx->query.initSlicedFindPath(a.ref,b.ref,a.p,b.p,&ctx->filter);
    while(dtStatusInProgress(status)) {
        if(cancel && cancel()) return 7;
        if(*expanded>=budget) return budgetResult(ctx, EXPANSION_LIMIT, status);
        int done=0; status=ctx->query.updateSlicedFindPath(std::min(256,budget-*expanded),&done); *expanded+=done;
    }
    if(dtStatusFailed(status)) return 5;
    if(dtStatusDetail(status,DT_OUT_OF_NODES)) return budgetResult(ctx, NODE_POOL_EXHAUSTED, status);
    std::vector<dtPolyRef> corridor(65535); int length=0;
    status=ctx->query.finalizeSlicedFindPath(corridor.data(),&length,int(corridor.size()));
    if(dtStatusDetail(status,DT_BUFFER_TOO_SMALL)) return budgetResult(ctx, CORRIDOR_CAPACITY, status);
    if(dtStatusFailed(status) || dtStatusDetail(status,DT_PARTIAL_RESULT) || !length || corridor[length-1]!=b.ref) return 5;
    for(int i=0;i<length;++i) {
        if(ctx->filter.blocked.count({corridor[i],corridor[i]})) return 8;
        if(i && (ctx->filter.blocked.count({corridor[i-1],corridor[i]}) || ctx->filter.blocked.count({corridor[i],corridor[i-1]}))) return 8;
    }
    std::vector<float> straight(size_t(capacity)*3); std::vector<unsigned char> flags(capacity);
    status=ctx->query.findStraightPath(a.p,b.p,corridor.data(),length,straight.data(),flags.data(),refs,count,capacity,DT_STRAIGHTPATH_ALL_CROSSINGS);
    if(dtStatusDetail(status,DT_BUFFER_TOO_SMALL)) return budgetResult(ctx, PATH_OUTPUT_CAPACITY, status);
    if(dtStatusFailed(status) || *count<1 || !(flags[*count-1]&DT_STRAIGHTPATH_END)) return 5;
    for(int i=0;i<*count;++i) {
        if(flags[i]&DT_STRAIGHTPATH_OFFMESH_CONNECTION) return 8;
        if(!refs[i]) refs[i]=b.ref;
        auto p=&straight[i*3]; float height=p[1];
        if(dtStatusFailed(ctx->query.getPolyHeight(refs[i],p,&height))) {
            // Funnel corners lie exactly on polygon boundaries. Detour's
            // point-in-triangle height test may reject the shared edge.
            float nearest[3]; bool over=false;
            if(dtStatusFailed(ctx->query.closestPointOnPoly(refs[i],p,nearest,&over))
               || dtVdist2DSqr(p,nearest)>.0001f) return 5;
            height=nearest[1];
        }
        output[i*3]=p[2]; output[i*3+1]=p[0]; output[i*3+2]=height;
    }
    return 0;
}

#include "surface_diagnostics_v2.inc"

#include "path_search_bounds_v3.inc"
