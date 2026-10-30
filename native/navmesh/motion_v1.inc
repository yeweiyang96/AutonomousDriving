// Ground-only motion certificates. Navmesh erosion already accounts for the
// baked agent body; margin is EXTRA tracking/quantization/curve error.
// A clearance certificate uses distance-to-wall's 1-Lipschitz bound, not a
// claim that a finite set of unconnected sample points proves a curve safe.
struct MotionFilter : GroundFilter {
    std::set<dtPolyRef> allowed;
    bool passFilter(dtPolyRef ref,const dtMeshTile* tile,const dtPoly* poly) const override {
        return GroundFilter::passFilter(ref,tile,poly) && !blocked.count({ref,ref}) &&
            (allowed.empty() || allowed.count(ref));
    }
};

static int motionRay(Context* ctx,MotionFilter& filter,dtPolyRef start,const float* a,const float* b,
                     std::vector<dtPolyRef>& visited) {
    std::array<dtPolyRef,2048> refs{}; int n=0; float t=0,normal[3];
    dtStatus status=ctx->query.raycast(start,a,b,&filter,&t,normal,refs.data(),&n,int(refs.size()));
    if (dtStatusDetail(status,DT_BUFFER_TOO_SMALL) || dtStatusDetail(status,DT_OUT_OF_NODES)) return 6;
    if (dtStatusFailed(status) || t<1.f || !n) return 8;
    for (int i=1;i<n;++i) if (filter.blocked.count({refs[i-1],refs[i]}) || filter.blocked.count({refs[i],refs[i-1]})) return 8;
    visited.assign(refs.begin(),refs.begin()+n); return 0;
}

// A portal endpoint belongs to either adjacent polygon. Keep the identity
// actually reached by the ray and check its height, instead of replacing it
// with an arbitrary same-XY nearest polygon (possibly on another floor).
static bool motionEndpoint(Context* ctx,dtPolyRef ref,Surface& endpoint) {
    float p[3]; bool over=false;
    if(dtStatusFailed(ctx->query.closestPointOnPoly(ref,endpoint.p,p,&over)) ||
       dtVdist2DSqr(p,endpoint.p)>.0001f || std::abs(p[1]-endpoint.p[1])>.15f) return false;
    endpoint.ref=ref; endpoint.p[1]=p[1]; return true;
}

// Funnel output can lie one float ULP outside a shared portal. This resolver
// is ONLY for collecting source topology, never for authorizing movement.
// Actual traces still use strict resolve() and positive swept clearance.
static int motionSource(Context* ctx,const float* world,Surface& out) {
    int rc=resolve(ctx,world,out); if(rc!=3)return rc;
    float p[]={world[1],world[2],world[0]},extent[]={.05f,.15f,.05f};
    if(!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))return 1;
    dtPolyRef refs[512];int count=0;
    auto status=ctx->query.queryPolygons(p,extent,&ctx->filter,refs,&count,512);
    if(dtStatusFailed(status)||dtStatusDetail(status,DT_BUFFER_TOO_SMALL))return 6;
    float best=.0025f*.0025f;
    for(int i=0;i<count;++i) {
        Surface candidate{};candidate.ref=refs[i];bool over=false;
        if(dtStatusFailed(ctx->query.closestPointOnPoly(refs[i],p,candidate.p,&over)))continue;
        float distance=dtVdist2DSqr(p,candidate.p);
        if(distance<=best && std::abs(candidate.p[1]-p[1])<.15f) {best=distance;out=candidate;rc=0;}
    }
    return rc;
}

// Collect a connected, ground-only local corridor. Expansion is bounded at
// each original route point and retains Detour identities (never XY aliases).
API int ad_motion_corridor_v1(void* handle,const float* points,int n,float width,
                             uint64_t* output,int capacity,int* count) {
    if (!handle || !points || !output || !count || n<1 || n>8192 || width<0 || width>8 || capacity<1) return 1;
    *count=0; auto ctx=static_cast<Context*>(handle); MotionFilter filter; filter.blocked=ctx->filter.blocked;
    std::set<dtPolyRef> result; Surface previous{};
    for (int i=0;i<n;++i) {
        Surface s; int rc=motionSource(ctx,points+i*3,s); if(rc) return rc;
        if (i) {
            // Source funnel vertices are topology anchors, not motion traces.
            // A ray starting exactly on a shared portal can select the incoming
            // side at t=0. Resolve connectivity with a strictly bounded Detour
            // search; this does not itself authorize any physical shortcut.
            auto status=ctx->query.initSlicedFindPath(previous.ref,s.ref,previous.p,s.p,&filter,0);
            int expanded=0;
            if(dtStatusInProgress(status))status=ctx->query.updateSlicedFindPath(256,&expanded);
            if(dtStatusInProgress(status)||dtStatusDetail(status,DT_OUT_OF_NODES))return 6;
            if(dtStatusFailed(status))return 8;
            std::array<dtPolyRef,256> refs{};int count=0;
            status=ctx->query.finalizeSlicedFindPath(refs.data(),&count,int(refs.size()));
            if(dtStatusDetail(status,DT_BUFFER_TOO_SMALL))return 6;
            if(dtStatusFailed(status)||!count||refs[count-1]!=s.ref)return 8;
            for(int j=0;j<count;++j) {
                if(filter.blocked.count({refs[j],refs[j]}) || (j &&
                    (filter.blocked.count({refs[j-1],refs[j]})||filter.blocked.count({refs[j],refs[j-1]}))))return 8;
                result.insert(refs[j]);
            }
        }
        std::array<dtPolyRef,2048> around{}; int found=0;
        auto status=ctx->query.findPolysAroundCircle(s.ref,s.p,width,&filter,around.data(),nullptr,nullptr,&found,int(around.size()));
        if (dtStatusDetail(status,DT_BUFFER_TOO_SMALL) || dtStatusDetail(status,DT_OUT_OF_NODES)) return 6;
        if (dtStatusFailed(status)) return 8;
        result.insert(around.begin(),around.begin()+found); result.insert(s.ref); previous=s;
        if (result.size()>size_t(capacity)) return 6;
    }
    for(auto ref:result) output[(*count)++]=ref;
    return 0;
}

API int ad_motion_validate_v1(void* handle,const float* points,int n,float margin,
                             const uint64_t* allowed,int allowed_count,int* failed,float* clearance) {
    if (!handle || !points || !failed || !clearance || n<1 || n>8192 ||
        !std::isfinite(margin) || margin<0 || margin>4 || allowed_count<0 || (allowed_count && !allowed)) return 1;
    *failed=-1; *clearance=std::numeric_limits<float>::infinity();
    auto ctx=static_cast<Context*>(handle); MotionFilter filter; filter.blocked=ctx->filter.blocked;
    if(allowed_count) filter.allowed.insert(allowed,allowed+allowed_count);
    Surface a; int rc=resolve(ctx,points,a); if(rc)return rc;
    const dtMeshTile* tile=nullptr; const dtPoly* poly=nullptr;
    if(dtStatusFailed(ctx->mesh.getTileAndPolyByRef(a.ref,&tile,&poly)) || !filter.passFilter(a.ref,tile,poly))return 8;
    int work=0;
    for(int i=0;i<n;++i) {
        *failed=i;
        if(!std::isfinite(points[i*3]) || !std::isfinite(points[i*3+1]) || !std::isfinite(points[i*3+2]))return 1;
        Surface b; rc=resolve(ctx,points+i*3,b); if(rc)return rc;
        std::vector<dtPolyRef> refs;
        rc=motionRay(ctx,filter,a.ref,a.p,b.p,refs); if(rc)return rc;
        if(!motionEndpoint(ctx,refs.back(),b) || std::abs(b.p[1]-points[i*3+2])>.15f) return 8;
        float length=dtVdist2D(a.p,b.p);
        int pieces=std::max(1,int(std::ceil(length/.20f)));
        if((work+=pieces)>16384)return 6;
        // On every sub-chord, endpoint wall distances minus half its length
        // bound all interior points. Recursive shortening is unnecessary at
        // this fixed certificate granularity: uncertain clearance is rejected.
        dtPolyRef ref=a.ref;
        for(int j=0;j<=pieces;++j) {
            float u=float(j)/pieces,p[3]; dtVlerp(p,a.p,b.p,u);
            std::vector<dtPolyRef> crossed; rc=motionRay(ctx,filter,a.ref,a.p,p,crossed);
            // Always cast from the original start identity; raycast is planar
            // but each height/last reference below preserves the selected floor.
            if(rc)return rc;
            ref=crossed.back();
            float h=0;
            if(dtStatusFailed(ctx->query.getPolyHeight(ref,p,&h))) {
                float closest[3]; bool over=false;
                if(dtStatusFailed(ctx->query.closestPointOnPoly(ref,p,closest,&over)) || dtVdist2DSqr(p,closest)>.0001f)return 8;
                h=closest[1];
            }
            if(std::abs(h-p[1])>.15f) return 8;
            p[1]=h; float distance=0,hit[3],normal[3];
            float required=margin+length/(2*pieces)+.001f;
            auto status=ctx->query.findDistanceToWall(ref,p,required+.01f,&filter,&distance,hit,normal);
            if(dtStatusDetail(status,DT_OUT_OF_NODES))return 6;
            if(dtStatusFailed(status))return 8;
            *clearance=std::min(*clearance,distance-length/(2*pieces));
            if(distance<required)return 8;
        }
        a=b;
    }
    *failed=-1;return 0;
}
