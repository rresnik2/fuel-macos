// FUEL's ground mesh: the decor quadtree around the camera (12288 m roots, split while
// the camera is within 2.0625 x the chunk size of its centre, 24 m leaves; FUN_007dfdc0,
// notes p15 D2) with one 25 x 25 point patch per chunk (Paysage_FillPoints 0x7f92c0,
// Paysage_BuildVertices 0x7f9b50; notes p21 M1-M3). Vertices carry a morph target on
// the half-resolution grid; the vertex shader blends to it between 2.375 and 2.875 chunk
// sizes from the camera, so a chunk has the parent's shape where it hands over: odd points
// collapse onto the even point (i & ~1, j & ~1), normals morph too (notes p31 E6).
#pragma once

#include "terrain.hpp"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuel {

struct TerrainVertex {
    float p0[3];
    float tweenStart;  // horizontal camera distance where the morph starts (2.375 C)
    float p1[3];       // morph target (coarser grid)
    float tweenInv;    // 1 / morph range (2 / C)
    float n[3];        // ground normal
    float size;        // chunk size C (for the shader's texture bands)
    float n1[3];       // morph target normal (the parent's coarse normal)
    float pad;
};
static_assert(sizeof(TerrainVertex) == 64, "TerrainVertex layout is shared with the Metal shader");

constexpr int kChunkSide = 25;  // points per side (24 quads)

struct TerrainChunk {
    uint64_t key = 0;
    double cx = 0, cz = 0, size = 0;
    std::vector<TerrainVertex> verts;  // 25 x 25 grid, then 96 skirt vertices
};

// The vertices of one chunk (thread-safe; about 7 ms for a near chunk).
std::vector<TerrainVertex> buildTerrainChunk(const Heightfield& hf, double cx, double cz, double size);
// Triangle list shared by every chunk (grid + skirts).
const std::vector<uint32_t>& terrainChunkIndices();

class TerrainLod {
public:
    explicit TerrainLod(const Heightfield& hf, int threads = 3);
    ~TerrainLod();
    TerrainLod(const TerrainLod&) = delete;
    TerrainLod& operator=(const TerrainLod&) = delete;

    // The chunks to draw for a camera at (ex, ez): the quadtree's leaves, where a chunk
    // whose four children are not built yet stands in for them (as FUEL keeps the parent
    // until its children are generated). Missing chunks are queued for the workers.
    std::vector<std::shared_ptr<const TerrainChunk>> update(double ex, double ez);
    // Blocks until nothing is queued or building.
    void settle();

private:
    struct Job {
        uint64_t key;
        double cx, cz, size, priority;
    };
    void worker();
    bool cover(double cx, double cz, double size, int level, double ex, double ez, std::vector<std::shared_ptr<const TerrainChunk>>& out,
               std::vector<Job>& want);

    const Heightfield& hf_;
    std::mutex mu_;
    std::condition_variable cv_, idle_;
    std::deque<Job> queue_;
    std::unordered_set<uint64_t> pending_;
    std::unordered_map<uint64_t, std::shared_ptr<const TerrainChunk>> built_;
    std::unordered_map<uint64_t, uint64_t> lastUsed_;
    uint64_t frame_ = 0;
    int busy_ = 0;
    bool stop_ = false;
    std::vector<std::thread> threads_;
};

}  // namespace fuel
