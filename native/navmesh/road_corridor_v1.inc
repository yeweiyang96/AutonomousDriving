// Additive road-only ABI. Existing queries and filters retain their behavior.
namespace adrive_roads {
static double distance(const float* p, const float* line, int n, double* dz=nullptr) {
    double best=1e100, height=1e100;
    for(int i=0;i<n-1;++i) {
        const auto a=line+i*3,b=a+3;
        const double dx=b[0]-a[0],dy=b[1]-a[1],norm=dx*dx+dy*dy;
        const double t=norm>0?std::clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm,0.,1.):0.;
        const double d=std::hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
        const double z=std::isfinite(a[2])&&std::isfinite(b[2])?std::abs(p[2]-a[2]-t*(b[2]-a[2])):0.;
        // A nearby different floor does not hide the road on this floor.
        if(z<=2.5 && d<best) {best=d;height=z;}
    }
    if(dz)*dz=height;
    return best;
}
struct Filter final : GroundFilter {
    std::set<dtPolyRef> allowed;
    explicit Filter(const GroundFilter& f):GroundFilter(f){}
    bool passFilter(dtPolyRef ref,const dtMeshTile* tile,const dtPoly* poly) const override {
        return allowed.count(ref)&&GroundFilter::passFilter(ref,tile,poly)&&!blocked.count({ref,ref});
    }
};
}

// Source geometry projection only. Never used to relocate requested endpoints.
API int ad_road_project_v1(void* handle,const float* world,float radius,float* output,int capacity,int* count) {
    if(!handle||!world||!output||!count||capacity<1||capacity>512||!std::isfinite(radius)||radius<0||radius>24
       ||!adrive_path_search_bounds_v3::validPoint(world))return 1;
    *count=0;
    try {
        auto ctx=static_cast<Context*>(handle);
        const bool known=std::isfinite(world[2]);
        float p[]={world[1],known?world[2]:0.f,world[0]},ext[]={radius+.05f,known?2.5f:20000.f,radius+.05f};
        dtPolyRef refs[4096];int num=0;
        auto status=ctx->query.queryPolygons(p,ext,&ctx->filter,refs,&num,4096);
        if(dtStatusFailed(status)||dtStatusDetail(status,DT_BUFFER_TOO_SMALL))return 6;
        std::vector<std::pair<double,std::array<float,3>>> candidates;
        for(int i=0;i<num;++i) {
            float q[3];bool over=false;
            if(dtStatusFailed(ctx->query.closestPointOnPoly(refs[i],p,q,&over)))continue;
            if(!over) {
                const dtMeshTile* tile=nullptr;const dtPoly* poly=nullptr;
                if(dtStatusFailed(ctx->mesh.getTileAndPolyByRef(refs[i],&tile,&poly)))continue;
                float center[3]={};
                for(int v=0;v<poly->vertCount;++v)for(int k=0;k<3;++k)center[k]+=tile->verts[3*poly->verts[v]+k]/poly->vertCount;
                const double toward=std::hypot(center[0]-q[0],center[2]-q[2]);
                if(toward>0) {
                    const float fraction=float(std::min(.02/toward,.5));
                    q[0]+=(center[0]-q[0])*fraction;q[2]+=(center[2]-q[2])*fraction;
                    if(dtStatusFailed(ctx->query.getPolyHeight(refs[i],q,&q[1])))continue;
                }
            }
            double d=std::hypot(q[0]-p[0],q[2]-p[2]);
            if(d>radius+.002|| (known&&std::abs(q[1]-p[1])>2.5))continue;
            candidates.push_back({d,{q[2],q[0],q[1]}});
        }
        std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.first<b.first;});
        std::vector<std::array<float,3>> chosen;
        for(const auto& item:candidates) {
            bool same=false;
            for(const auto& q:chosen)if(std::abs(q[2]-item.second[2])<.35){same=true;break;}
            if(!same)chosen.push_back(item.second);
        }
        if(int(chosen.size())>capacity)return 6;
        *count=int(chosen.size());
        for(int i=0;i<*count;++i)std::copy(chosen[i].begin(),chosen[i].end(),output+i*3);
        return 0;
    }catch(const std::bad_alloc&){return 2;}
}

// World-order polyline. Build an explicit allowed-poly set, intersect it with
// GroundFilter, then check the complete returned geometry remains in the ribbon.
API int ad_path_corridor_v1(void* handle,const float* start,const float* goal,int budget,
    const float* line,int n,float width,Cancel cancel,float* output,uint64_t* refs,
    int capacity,int* count,int* expanded) {
    if(!count||!expanded)return 1;
    *count=0;*expanded=0;
    if(!handle||!line||!output||!refs||n<2||n>20000||capacity<1||capacity>65535||budget<0
       ||!std::isfinite(width)||width<=0||width>24
       ||!adrive_path_search_bounds_v3::validPoint(start)||!adrive_path_search_bounds_v3::validPoint(goal))return 1;
    for(int i=0;i<n;++i)if(!adrive_path_search_bounds_v3::validPoint(line+i*3))return 1;
    try {
        auto ctx=static_cast<Context*>(handle);adrive_roads::Filter filter(ctx->filter);
        for(int i=0;i<n-1;++i) {
            if(cancel&&cancel())return 7;
            const auto a=line+i*3,b=a+3;
            const bool known=std::isfinite(a[2])&&std::isfinite(b[2]);
            float p[]={(a[1]+b[1])/2,known?(a[2]+b[2])/2:0.f,(a[0]+b[0])/2};
            float ext[]={std::abs(a[1]-b[1])/2+width+.05f,known?std::abs(a[2]-b[2])/2+3:20000.f,std::abs(a[0]-b[0])/2+width+.05f};
            dtPolyRef found[16384];int num=0;
            auto status=ctx->query.queryPolygons(p,ext,&ctx->filter,found,&num,16384);
            if(dtStatusFailed(status)||dtStatusDetail(status,DT_BUFFER_TOO_SMALL))return budgetResult(ctx,SURFACE_POLYGON_CAPACITY,status);
            for(int k=0;k<num;++k)filter.allowed.insert(found[k]);
        }
        // The set above is a conservative AABB superset. The mandatory full
        // geometry check below prevents its corners/large polygons authorizing
        // a shortcut outside the actual polyline corridor.
        int rc=adrive_path_search_bounds_v3::query(ctx,start,goal,budget,1e8f,cancel,output,refs,capacity,count,expanded,&filter);
        if(rc){*count=0;return rc;}
        for(int i=0;i<*count-1;++i) {
            const auto a=output+i*3,b=a+3;const double len=std::hypot(b[0]-a[0],b[1]-a[1]);
            const int steps=std::max(1,int(std::ceil(len/.25)));
            for(int j=0;j<=steps;++j) {
                if(cancel&&cancel()){*count=0;return 7;}
                const float t=float(j)/steps;float p[]={a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2])};
                // Half-sample Lipschitz allowance makes this continuous, not
                // merely a vertex-only ribbon test.
                if(adrive_roads::distance(p,line,n)+len/steps/2>width+.002){*count=0;return 10;}
            }
        }
        return 0;
    }catch(const std::bad_alloc&){*count=0;return 2;}
}
