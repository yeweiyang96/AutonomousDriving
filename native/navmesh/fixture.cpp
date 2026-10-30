// Deterministic offline geometry: a hole, a water polygon, an unconnected
// upper floor and an off-mesh shortcut that the walking filter must reject.
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <limits>
#include <vector>
#include <cstring>
int main(int argc,char** argv) {
    if(argc!=2) return 1;
    std::filesystem::path dir=argv[1]; std::filesystem::create_directories(dir);
    std::vector<unsigned short> vertices,polys,flags;
    std::vector<unsigned char> areas;
    std::map<std::array<int,3>,unsigned short> index;
    auto vertex=[&](int x,int y,int z){
        std::array<int,3> key{x,y,z}; auto it=index.find(key); if(it!=index.end()) return it->second;
        auto id=static_cast<unsigned short>(vertices.size()/3); index[key]=id;
        vertices.insert(vertices.end(),{(unsigned short)x,(unsigned short)y,(unsigned short)z}); return id;
    };
    auto quad=[&](int x,int z,int y,int area,int flag){
        std::array<unsigned short,12> p; p.fill(0xffff);
        p[0]=vertex(x,y,z); p[1]=vertex(x,y,z+10); p[2]=vertex(x+10,y,z+10); p[3]=vertex(x+10,y,z);
        polys.insert(polys.end(),p.begin(),p.end()); areas.push_back(area); flags.push_back(flag);
    };
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) if(r!=1 || c!=1)
        quad(c*10,r*10,0, r==0 && c==1 ? 9:11, r==0 && c==1 ? 4:1);
    quad(0,0,20,11,1); // 20 height voxels * .25 = 5 yards.
    int count=int(areas.size());
    for(int i=0;i<count;++i) for(int j=0;j<count;++j) if(i!=j)
        for(int a=0;a<4;++a) for(int b=0;b<4;++b)
            if(polys[i*12+a]==polys[j*12+(b+1)%4] && polys[i*12+(a+1)%4]==polys[j*12+b])
                polys[i*12+6+a]=(unsigned short)j;
    float connection[]={5,0,15,25,0,15}; float radius=2;
    unsigned short connectionFlag=1; unsigned char connectionArea=11,direction=1;
    dtNavMeshCreateParams p{};
    p.verts=vertices.data();p.vertCount=int(vertices.size()/3);p.polys=polys.data();p.polyFlags=flags.data();
    p.polyAreas=areas.data();p.polyCount=count;p.nvp=6;p.walkableHeight=1.6f;p.walkableRadius=.533333f;p.walkableClimb=.533333f;
    p.cs=1;p.ch=.25f;p.bmax[0]=30;p.bmax[1]=10;p.bmax[2]=30;p.buildBvTree=true;
    p.offMeshConVerts=connection;p.offMeshConRad=&radius;p.offMeshConFlags=&connectionFlag;
    p.offMeshConAreas=&connectionArea;p.offMeshConDir=&direction;p.offMeshConCount=1;
    unsigned char* data=nullptr;int size=0;if(!dtCreateNavMeshData(&p,&data,&size))return 2;
    struct MH {uint32_t magic,version;dtNavMeshParams params;uint32_t links;};
    MH mh{};mh.magic=0x4d4d4150;mh.version=16;mh.params.tileWidth=mh.params.tileHeight=30;mh.params.maxTiles=4;mh.params.maxPolys=std::numeric_limits<int>::min();
    struct TH {uint32_t magic,dtVersion,version,size;uint8_t liquids,pad[3];};
    TH th{0x4d4d4150,7,16,(uint32_t)size,1,{0,0,0}};
    for(int c=0;c<2;++c){
        std::ofstream map(dir/(c==0?"0000.mmap":"0001.mmap"),std::ios::binary);map.write((char*)&mh,sizeof mh);
        std::ofstream tile(dir/(c==0?"0000_31_31.mmtile":"0001_31_31.mmtile"),std::ios::binary);
        tile.write((char*)&th,sizeof th);tile.write((char*)data,size);
    }
    dtFree(data);
    // Continent 1 additionally has a real external tile portal at west=30.
    // Keep continent 0's original hole/water/stacked-floor fixture unchanged.
    for(int i=0;i<count;++i) for(int a=0;a<4;++a) {
        int va=polys[i*12+a],vb=polys[i*12+(a+1)%4];
        if(polys[i*12+6+a]!=0xffff)continue;
        if(vertices[va*3]==0 && vertices[vb*3]==0)polys[i*12+6+a]=0x8000;
        if(vertices[va*3]==30 && vertices[vb*3]==30)polys[i*12+6+a]=0x8002;
    }
    p.offMeshConCount=0;
    for(int tileIndex=0;tileIndex<2;++tileIndex) {
        p.tileX=tileIndex;p.bmin[0]=float(tileIndex*30);p.bmax[0]=float((tileIndex+1)*30);
        if(!dtCreateNavMeshData(&p,&data,&size))return 3;
        th.size=uint32_t(size);
        std::ofstream tile(dir/(tileIndex==0?"0001_31_31.mmtile":"0001_31_30.mmtile"),std::ios::binary);
        tile.write((char*)&th,sizeof th);tile.write((char*)data,size);dtFree(data);
    }
    return 0;
}
