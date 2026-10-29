// Deterministic diagnostic geometry, independent from game data.
// Builds real Detour polygons/links, including excluded tags and 33 layers.
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

static_assert(sizeof(dtPolyRef) == 8, "The project requires 64-bit polygon references");

struct Geometry {
    std::vector<unsigned short> vertices, polys, flags;
    std::vector<unsigned char> areas;
    std::map<std::array<int, 3>, unsigned short> index;

    unsigned short vertex(int x, int y, int z) {
        std::array<int, 3> key{x, y, z};
        const auto found = index.find(key);
        if (found != index.end()) return found->second;
        const auto id = static_cast<unsigned short>(vertices.size() / 3);
        index[key] = id;
        vertices.insert(vertices.end(), {static_cast<unsigned short>(x),
                         static_cast<unsigned short>(y), static_cast<unsigned short>(z)});
        return id;
    }
    void quad(int west, int north, int heightVoxels, int area, int flag) {
        std::array<unsigned short, 12> p; p.fill(0xffff);
        p[0] = vertex(west, heightVoxels, north);
        p[1] = vertex(west, heightVoxels, north + 10);
        p[2] = vertex(west + 10, heightVoxels, north + 10);
        p[3] = vertex(west + 10, heightVoxels, north);
        polys.insert(polys.end(), p.begin(), p.end());
        areas.push_back(static_cast<unsigned char>(area));
        flags.push_back(static_cast<unsigned short>(flag));
    }
    void adjacency() {
        const int count = static_cast<int>(areas.size());
        for (int i = 0; i < count; ++i) for (int j = 0; j < count; ++j) if (i != j)
            for (int a = 0; a < 4; ++a) for (int b = 0; b < 4; ++b)
                if (polys[i * 12 + a] == polys[j * 12 + (b + 1) % 4] &&
                    polys[i * 12 + (a + 1) % 4] == polys[j * 12 + b])
                    polys[i * 12 + 6 + a] = static_cast<unsigned short>(j);
    }
};

static void writeTile(const std::filesystem::path& path, Geometry& g, int layer, bool offmesh) {
    g.adjacency();
    const float connections[] = {5, 0, 15, 25, 0, 15, 45, 0, 45, 65, 0, 45};
    const float radii[] = {2, 2};
    const unsigned short connectionFlags[] = {1, 1};
    const unsigned char connectionAreas[] = {11, 11}, directions[] = {1, 1};
    dtNavMeshCreateParams p{};
    p.verts = g.vertices.data(); p.vertCount = static_cast<int>(g.vertices.size() / 3);
    p.polys = g.polys.data(); p.polyCount = static_cast<int>(g.areas.size()); p.nvp = 6;
    p.polyFlags = g.flags.data(); p.polyAreas = g.areas.data();
    p.walkableHeight = 1.6f; p.walkableRadius = .533333f; p.walkableClimb = .533333f;
    p.cs = 1; p.ch = .25f; p.bmax[0] = 100; p.bmax[1] = 64; p.bmax[2] = 100;
    p.tileLayer = layer; p.buildBvTree = true;
    if (offmesh) {
        p.offMeshConVerts = connections; p.offMeshConRad = radii;
        p.offMeshConFlags = connectionFlags; p.offMeshConAreas = connectionAreas;
        p.offMeshConDir = directions; p.offMeshConCount = 2;
    }
    unsigned char* data = nullptr; int size = 0;
    if (!dtCreateNavMeshData(&p, &data, &size)) throw std::runtime_error("fixture mesh build failed");
    struct TileHeader { uint32_t magic, dtVersion, version, size; uint8_t liquids, pad[3]; };
    static_assert(sizeof(TileHeader) == 20);
    TileHeader header{0x4d4d4150, DT_NAVMESH_VERSION, 16, static_cast<uint32_t>(size), 1, {0, 0, 0}};
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(&header), sizeof header);
    output.write(reinterpret_cast<const char*>(data), size);
    const bool good = output.good();
    dtFree(data);
    if (!good) throw std::runtime_error("fixture tile write failed");
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    try {
        const std::filesystem::path dir = argv[1];
        std::filesystem::create_directories(dir);
        struct MapHeader { uint32_t magic, version; dtNavMeshParams params; uint32_t links; };
        static_assert(sizeof(MapHeader) == 40);
        MapHeader map{}; map.magic = 0x4d4d4150; map.version = 16;
        map.params.tileWidth = map.params.tileHeight = 100;
        map.params.maxTiles = 64; map.params.maxPolys = std::numeric_limits<int>::min();
        std::ofstream mapFile(dir / "0000.mmap", std::ios::binary);
        mapFile.write(reinterpret_cast<const char*>(&map), sizeof map);
        if (!mapFile.good()) return 2;
        mapFile.close();
        Geometry base;
        for (int north = 0; north < 3; ++north) for (int west = 0; west < 3; ++west)
            if (north != 1 || west != 1)
                base.quad(west * 10, north * 10, 0,
                    north == 0 && west == 1 ? 9 : 11, north == 0 && west == 1 ? 4 : 1);
        base.quad(0, 0, 20, 11, 1);  // over the lower ring, height 5
        base.quad(40, 0, 0, 11, 0);  // flags=0, never authorized by GroundFilter
        base.quad(40, 20, 0, 11, 1); // equal-height, equal-XY independent identities
        base.quad(40, 20, 0, 9, 4);
        base.quad(0, 40, 0, 11, 1); base.quad(10, 40, 0, 9, 4); base.quad(20, 40, 0, 11, 1);
        base.quad(40, 40, 0, 11, 1); base.quad(60, 40, 0, 11, 1); // offmesh-only connection
        base.quad(0, 60, 0, 11, 1); base.quad(10, 60, 0, 11, 1); base.quad(20, 60, 0, 11, 1);
        writeTile(dir / "base.mmtile", base, 0, true);
        std::vector<std::string> tileNames{"base.mmtile"};
        for (int layer = 1; layer <= 33; ++layer) {
            Geometry stacked; stacked.quad(70, 70, 20 + layer * 2, 11, 1);
            const auto name = "layer-" + std::to_string(layer) + ".mmtile";
            writeTile(dir / name, stacked, layer, false);
            tileNames.push_back(name);
        }
        std::ofstream info(dir / "fixture-info.json", std::ios::binary);
        info << "{\n  \"schema\": 1,\n  \"synthetic\": true,\n  \"coordinate_order\": \"north,west,height\",\n"
             << "  \"map\": \"0000.mmap\",\n  \"tiles\": [";
        for (size_t i = 0; i < tileNames.size(); ++i)
            info << (i ? "," : "") << "\"" << tileNames[i] << "\"";
        info << "],\n  \"points\": {\n"
             << "    \"lower\": [5,5,0], \"upper\": [5,5,5], \"multilevel\": [5,5,null],\n"
             << "    \"water\": [5,15,0], \"hole\": [15,15,null], \"zero_flags\": [5,45,0],\n"
             << "    \"equal_height\": [25,45,0], \"many_layers\": [75,75,null],\n"
             << "    \"water_chain_start\": [45,5,0], \"water_chain_goal\": [45,25,0],\n"
             << "    \"offmesh_start\": [45,45,0], \"offmesh_goal\": [45,65,0],\n"
             << "    \"ground_chain_start\": [65,5,0], \"ground_chain_middle\": [65,15,0], \"ground_chain_goal\": [65,25,0],\n"
             << "    \"ring_start\": [15,5,0], \"ring_goal\": [15,25,0]\n  }\n}\n";
        return info.good() ? 0 : 3;
    } catch (...) { return 4; }
}
