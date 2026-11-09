// fuelterrain: decodes the world heightfield and prints its layout.
//
//   fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...] [--seat]
//
// --pgm compares every sample with a 16-bit PGM export (raw * 65535 / 4095).
// --seat measures how many objects the sub-worlds next to USA1.DPC place on the
// ground, with the raw and the levelled (flat surfaces) heightfield.
// --float adds geometry classes (floating / on another object / elevated origin / buried, p35);
// FUEL_FLOAT_CLASSES=FILE lists them. --float-dump OUT.tsv: one row per placement for classifying.
// --file-flats X Z: the authored flat surfaces holding a point; --emu-flat-cmd FILE: EmuRun commands
// for the emulated flat-distance vectors; --resident: USA1.DPC's own placements.
// --decal-check [X Z ...]: road decal uv continuity between owned neighbour vertices on FUEL's 1/2/4/8 m
// lattices (p71); default spots yell_tarmac, yell_start, the tarmac at (-39743.5, -28195.5), a dirt road (-39465, -27606).

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/events.hpp"
#include "core/ground.hpp"
#include "core/ground_point.hpp"
#include "core/road_types.hpp"
#include "core/static_collision.hpp"
#include "core/roads.hpp"
#include "core/terrain.hpp"
#include "core/terrain_height.hpp"
#include "core/automesh.hpp"
#include "core/terrain_mesh.hpp"
#include "core/vegetation.hpp"
#include "core/road_graph.hpp"
#include "core/races.hpp"
#include "core/world.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "core/road_segments.hpp"
#include "core/water.hpp"

namespace {

// Emulated flat-surface distance vectors (p35 F1): file record index and the point as fractions of
// the file X and Z axes from the centre (FUEL's half extent is 0.5 of each axis).
struct EmuFlatPoint {
    uint32_t index;
    float fx, fz;
};
const EmuFlatPoint kEmuFlatPoints[] = {{510, -0.45f, 0.40f}, {510, 0.30f, -0.47f}, {510, -0.48f, -0.46f}, {510, 0.60f, 0.10f},
                                       {505, -0.47f, 0.20f}, {505, 0.10f, -0.49f}, {526, 0.30f, 0.30f},   {526, -0.20f, 0.45f},
                                       {526, 0.70f, 0.00f}};

// --float-dump OUT.tsv (p35): one row per sub-world mesh placement with what classifying a
// floating object needs: the file, node, its parent and the parent's object class, the
// prototype name (from USA1's prototype GameObj_Z groups), UserDefine text, scale, the
// offset from FUEL's ground, the height terms there, and the placement it rests on, if any.
void dumpPlacements(const fuel::DpcFile& usa1, const std::vector<std::unique_ptr<fuel::DpcFile>>& subs, const fuel::Heightfield& ground,
                    const char* out) {
    constexpr uint32_t kNode = 0x85D02528, kLod = 0x73DC6A13, kUserDefine = 0x52F79F96;
    // Prototype names: USA1's GameObj_Z groups list library nodes; their object is the prototype.
    std::unordered_map<uint32_t, std::string> protoName;
    std::unordered_map<uint32_t, size_t> protoGroupSize;  // the smallest group names the prototype
    for (uint32_t g : usa1.ids(fuel::cls::GameObj)) try {
            for (const fuel::NodeGroup& grp : fuel::parseGameObjBody(*usa1.body(*usa1.find(g))))
                for (uint32_t n : grp.nodes)
                    if (const auto* e = usa1.find(n); e && e->classCrc == kNode)
                        if (const auto b = usa1.body(*e)) {
                            const uint32_t obj = fuel::parseNodeBody(*b).object;
                            auto [it, added] = protoGroupSize.try_emplace(obj, grp.nodes.size());
                            if (added || grp.nodes.size() < it->second) it->second = grp.nodes.size(), protoName[obj] = grp.name;
                        }
        } catch (const std::exception&) {
        }
    auto findIn = [&](const fuel::DpcFile& sub, uint32_t id) -> std::pair<const fuel::DpcFile*, const fuel::DpcFile::Entry*> {
        for (const fuel::DpcFile* f : {&sub, &usa1})
            if (const auto* e = f->find(id); e && e->header) return {f, e};
        return {nullptr, nullptr};
    };
    struct Row {
        std::string file, name, ud, parentClass;
        uint32_t node = 0, parent = 0, object = 0, mesh = 0;
        bool fit = false;
        float x = 0, y = 0, z = 0, scale = 1, top = 0, radius = 0;
    };
    std::vector<Row> rows;
    std::unordered_map<uint32_t, std::pair<float, float>> meshTopRadius;  // local max y, xz radius
    for (const auto& sub : subs) {
        const std::string file = std::filesystem::path(sub->path()).filename().string();
        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, {sub.get(), &usa1})) {
            Row r;
            r.file = file, r.node = pm.nodeId, r.mesh = pm.meshId, r.fit = pm.fitToGround;
            r.x = pm.world[12], r.y = pm.world[13], r.z = pm.world[14];
            if (std::fabs(r.x) < 50 && std::fabs(r.z) < 50) continue;
            r.scale = std::sqrt(pm.world[4] * pm.world[4] + pm.world[5] * pm.world[5] + pm.world[6] * pm.world[6]);
            try {
                const fuel::Node n = fuel::parseNodeBody(*sub->body(*sub->find(pm.nodeId)));
                r.parent = n.parent, r.object = n.object;
                if (auto it = protoName.find(n.object); it != protoName.end()) r.name = it->second;
                if (auto [f, e] = findIn(*sub, n.userDefine); e && e->classCrc == kUserDefine)
                    for (const std::string& l : fuel::parseUserDefineBody(*f->body(*e))) r.ud += (r.ud.empty() ? "" : "|") + l.substr(0, 40);
                if (auto [f, e] = findIn(*sub, n.parent); e && e->classCrc == kNode) {
                    const fuel::Node p = fuel::parseNodeBody(*f->body(*e));
                    auto [pf, pe] = findIn(*sub, p.object);
                    char buf[16];
                    std::snprintf(buf, sizeof buf, "%08x", pe ? pe->classCrc : 0u);
                    r.parentClass = p.object == 0 ? "group" : pe && pe->classCrc == kLod ? "Lod" : pe && pe->classCrc == fuel::cls::Mesh ? "Mesh" : buf;
                }
            } catch (const std::exception&) {
            }
            auto mt = meshTopRadius.find(r.mesh);
            if (mt == meshTopRadius.end()) {
                float top = 0, rad = 0;
                try {
                    const fuel::CollisionMesh cm = fuel::loadCollisionMesh({sub.get(), &usa1}, r.mesh);
                    for (size_t i = 0; i + 2 < cm.positions.size(); i += 3)
                        top = std::max(top, cm.positions[i + 1]), rad = std::max(rad, std::hypot(cm.positions[i], cm.positions[i + 2]));
                } catch (const std::exception&) {
                }
                mt = meshTopRadius.emplace(r.mesh, std::make_pair(top, rad)).first;
            }
            r.top = mt->second.first * r.scale, r.radius = mt->second.second * r.scale;
            rows.push_back(std::move(r));
        }
    }
    // Spatial hash (16 m) for "rests on": another placement whose xz footprint covers this
    // one's origin and whose top is within 1.5 m of this one's base.
    std::unordered_map<int64_t, std::vector<uint32_t>> grid;
    auto key = [](float x, float z) { return (int64_t(std::floor(x / 16)) << 32) ^ int64_t(uint32_t(int32_t(std::floor(z / 16)))); };
    for (uint32_t i = 0; i < rows.size(); ++i) grid[key(rows[i].x, rows[i].z)].push_back(i);
    std::FILE* f = std::fopen(out, "w");
    if (!f) throw std::runtime_error(std::string("cannot write ") + out);
    std::fprintf(f, "file\tnode\tparent\tparentClass\tobject\tmesh\tname\tfit\tud\tx\ty\tz\tscale\ttop\tradius\tground\td\th0\tdetail\tgulch\tflatA\troad\tflatB\trock\tny\trestsOn\trestsOnD\n");
    for (const Row& r : rows) {
        const fuel::GroundSample g = ground.detail->sample(r.x, r.z);
        const float gr = ground.smoothHeightAt(r.x, r.z);
        int best = -1;
        float bestD = 1e9f;
        for (int dz = -4; dz <= 4; ++dz)
            for (int dx = -4; dx <= 4; ++dx) {
                auto it = grid.find(key(r.x + dx * 16, r.z + dz * 16));
                if (it == grid.end()) continue;
                for (uint32_t j : it->second) {
                    const Row& q = rows[j];
                    if (&q == &r || q.node == r.node || q.y > r.y - 0.3f) continue;
                    if (std::hypot(q.x - r.x, q.z - r.z) > q.radius) continue;
                    const float d = std::fabs(q.y + q.top - r.y);
                    if (d < 1.5f && d < bestD) bestD = d, best = int(j);
                }
            }
        std::fprintf(f, "%s\t%08x\t%08x\t%s\t%08x\t%08x\t%s\t%d\t%s\t%.2f\t%.2f\t%.2f\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%s\t%.2f\n",
                     r.file.c_str(), r.node, r.parent, r.parentClass.c_str(), r.object, r.mesh, r.name.empty() ? "-" : r.name.c_str(), r.fit ? 1 : 0,
                     r.ud.empty() ? "-" : r.ud.c_str(), r.x, r.y, r.z, r.scale, r.top, r.radius, gr, r.y - gr, g.h0, g.detail, g.gulch, g.flatA, g.road,
                     g.flatB, g.rock, g.ny, best < 0 ? "-" : (rows[size_t(best)].name.empty() ? "?" : rows[size_t(best)].name.c_str()), best < 0 ? 0.0f : bestD);
    }
    std::fclose(f);
    std::printf("float-dump: %zu placements -> %s\n", rows.size(), out);
}

// --float (p35): classify every sub-world mesh placement by its geometry, not only its origin.
// The origin offset (authored y - FUEL's ground) cannot tell a floating prop from one that stands on a
// bridge, a building or another prop, or a big mesh whose origin is not at its base. Here each
// placement's mesh vertices (the lowest 64 in its own space plus 32 spread over it and the top one)
// are taken to world space; a vertex is supported when it is at most 0.5 m above the ground or above
// the top of another placement's triangles under it (vertical ray). Classes:
//   floating   no sampled vertex within 0.5 m of a supporting surface (the object hangs in the air)
//   on object  origin > 0.5 m above the ground, supported by another placement (a surface under a vertex
//              within 0.5 m, or a vertex within 0.3 m of its triangles: hung on or leaning on it)
//   elevated   origin > 0.5 m above the ground, but the mesh itself reaches the ground
//   buried     every sampled vertex (and the top) more than 0.3 m under the ground
// Distance from p to triangle abc (closest point on the triangle, Ericson's region test).
float pointTriangleDistance(const float* p, const float* a, const float* b, const float* c) {
    auto sub = [](const float* u, const float* v, float* o) { o[0] = u[0] - v[0], o[1] = u[1] - v[1], o[2] = u[2] - v[2]; };
    auto dot = [](const float* u, const float* v) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };
    float ab[3], ac[3], ap[3], bp[3], cp[3], q[3];
    sub(b, a, ab), sub(c, a, ac), sub(p, a, ap);
    auto at = [&](float s, float t) {
        for (int k = 0; k < 3; ++k) q[k] = a[k] + s * ab[k] + t * ac[k];
        const float d[3] = {p[0] - q[0], p[1] - q[1], p[2] - q[2]};
        return std::sqrt(dot(d, d));
    };
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return at(0, 0);
    sub(p, b, bp);
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return at(1, 0);
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return at(d1 / (d1 - d3), 0);
    sub(p, c, cp);
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return at(0, 1);
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return at(0, d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return at(1 - w, w);
    }
    const float den = 1 / (va + vb + vc);
    return at(vb * den, vc * den);
}

struct FloatClasses {
    size_t total = 0, onGround = 0, floating = 0, floating2 = 0, onObject = 0, elevated = 0, buried = 0, sunkOrigin = 0;
};

FloatClasses classifyPlacements(const fuel::DpcFile& usa1, const std::vector<std::unique_ptr<fuel::DpcFile>>& subs, const fuel::Heightfield& ground,
                                bool includeFit, std::FILE* list) {
    struct MeshGeo {
        std::vector<float> pos;            // local xyz
        std::vector<uint32_t> idx;         // triangles
        std::vector<uint32_t> sample;      // vertex indices tested for support
        float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    };
    struct Pl {
        const MeshGeo* geo;
        std::array<float, 16> m;
        float lo[3], hi[3];  // world AABB
        uint32_t mesh;
        std::string file;
    };
    std::unordered_map<uint32_t, MeshGeo> geos;
    std::vector<Pl> pls;
    auto xf = [](const std::array<float, 16>& m, const float* v, float* o) {
        for (int c = 0; c < 3; ++c) o[c] = v[0] * m[c] + v[1] * m[4 + c] + v[2] * m[8 + c] + m[12 + c];
    };
    for (const auto& sub : subs)
        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, {sub.get(), &usa1})) {
            if (std::fabs(pm.world[12]) < 50 && std::fabs(pm.world[14]) < 50) continue;
            if (pm.fitToGround && !includeFit) continue;
            auto it = geos.find(pm.meshId);
            if (it == geos.end()) {
                MeshGeo g;
                try {
                    fuel::CollisionMesh cm = fuel::loadCollisionMesh({sub.get(), &usa1}, pm.meshId);
                    g.pos = std::move(cm.positions), g.idx = std::move(cm.indices);
                } catch (const std::exception&) {
                }
                const size_t n = g.pos.size() / 3;
                std::vector<uint32_t> order(n);
                for (uint32_t k = 0; k < n; ++k) order[k] = k;
                std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return g.pos[3 * a + 1] < g.pos[3 * b + 1]; });
                for (size_t k = 0; k < std::min<size_t>(64, n); ++k) g.sample.push_back(order[k]);
                for (size_t k = 0; n > 0 && k < 32; ++k) g.sample.push_back(order[k * n / 32]);
                if (n > 0) g.sample.push_back(order[n - 1]);
                for (size_t k = 0; k < n; ++k)
                    for (int c = 0; c < 3; ++c) g.lo[c] = std::min(g.lo[c], g.pos[3 * k + c]), g.hi[c] = std::max(g.hi[c], g.pos[3 * k + c]);
                if (const char* dbg = std::getenv("FUEL_MESH_DEBUG"); dbg && std::strtoul(dbg, nullptr, 16) == pm.meshId) {
                    std::printf("mesh %08x: %zu vertices, %zu triangles, y %.2f..%.2f; lowest:", pm.meshId, n, g.idx.size() / 3, g.lo[1], g.hi[1]);
                    for (size_t k = 0; k < std::min<size_t>(8, n); ++k) std::printf(" (%.2f %.2f %.2f)", g.pos[3 * order[k]], g.pos[3 * order[k] + 1], g.pos[3 * order[k] + 2]);
                    std::printf("\n");
                }
                it = geos.emplace(pm.meshId, std::move(g)).first;
            }
            if (it->second.sample.empty()) continue;
            Pl p{&it->second, pm.world, {1e9f, 1e9f, 1e9f}, {-1e9f, -1e9f, -1e9f}, pm.meshId, std::filesystem::path(sub->path()).filename().string()};
            for (int k = 0; k < 8; ++k) {
                const float v[3] = {(k & 1) ? p.geo->hi[0] : p.geo->lo[0], (k & 2) ? p.geo->hi[1] : p.geo->lo[1], (k & 4) ? p.geo->hi[2] : p.geo->lo[2]};
                float w[3];
                xf(p.m, v, w);
                for (int c = 0; c < 3; ++c) p.lo[c] = std::min(p.lo[c], w[c]), p.hi[c] = std::max(p.hi[c], w[c]);
            }
            pls.push_back(std::move(p));
        }
    // 32 m spatial hash of the world AABBs.
    std::unordered_map<int64_t, std::vector<uint32_t>> grid;
    auto key = [](int64_t cx, int64_t cz) { return (cx << 32) ^ int64_t(uint32_t(int32_t(cz))); };
    for (uint32_t i = 0; i < pls.size(); ++i)
        for (int64_t cz = int64_t(std::floor(pls[i].lo[2] / 32)); cz <= int64_t(std::floor(pls[i].hi[2] / 32)); ++cz)
            for (int64_t cx = int64_t(std::floor(pls[i].lo[0] / 32)); cx <= int64_t(std::floor(pls[i].hi[0] / 32)); ++cx) grid[key(cx, cz)].push_back(i);
    std::unordered_map<uint32_t, std::vector<float>> worldTris;  // placement -> world vertex positions, on demand
    auto surfaceBelow = [&](uint32_t self, float x, float y, float z) {
        float best = -1e9f;
        auto it = grid.find(key(int64_t(std::floor(x / 32)), int64_t(std::floor(z / 32))));
        if (it == grid.end()) return best;
        for (uint32_t q : it->second) {
            const Pl& Q = pls[q];
            if (q == self || x < Q.lo[0] || x > Q.hi[0] || z < Q.lo[2] || z > Q.hi[2] || Q.lo[1] > y + 0.5f || Q.hi[1] < best) continue;
            auto [wt, added] = worldTris.try_emplace(q);
            if (added) {
                wt->second.resize(Q.geo->pos.size());
                for (size_t k = 0; k < Q.geo->pos.size(); k += 3) xf(Q.m, &Q.geo->pos[k], &wt->second[k]);
            }
            const std::vector<float>& P = wt->second;
            const std::vector<uint32_t>& I = Q.geo->idx;
            for (size_t t = 0; t + 2 < I.size(); t += 3) {
                const float *a = &P[3 * I[t]], *b = &P[3 * I[t + 1]], *c = &P[3 * I[t + 2]];
                const float d = (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
                if (std::fabs(d) < 1e-9f) continue;
                const float u = ((x - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (z - a[2])) / d, v = ((b[0] - a[0]) * (z - a[2]) - (x - a[0]) * (b[2] - a[2])) / d;
                if (u < 0 || v < 0 || u + v > 1) continue;
                const float h = a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1]);
                if (h <= y + 0.5f && h > best) best = h;
            }
        }
        return best;
    };
    // Another placement's triangles within 0.3 m of a point (an object hung on, or leaning on, another).
    auto touches = [&](uint32_t self, const float* v) {
        auto it = grid.find(key(int64_t(std::floor(v[0] / 32)), int64_t(std::floor(v[2] / 32))));
        if (it == grid.end()) return false;
        constexpr float r = 0.3f;
        for (uint32_t q : it->second) {
            const Pl& Q = pls[q];
            if (q == self || v[0] < Q.lo[0] - r || v[0] > Q.hi[0] + r || v[1] < Q.lo[1] - r || v[1] > Q.hi[1] + r || v[2] < Q.lo[2] - r ||
                v[2] > Q.hi[2] + r)
                continue;
            auto [wt, added] = worldTris.try_emplace(q);
            if (added) {
                wt->second.resize(Q.geo->pos.size());
                for (size_t k = 0; k < Q.geo->pos.size(); k += 3) xf(Q.m, &Q.geo->pos[k], &wt->second[k]);
            }
            const std::vector<float>& P = wt->second;
            const std::vector<uint32_t>& I = Q.geo->idx;
            for (size_t t = 0; t + 2 < I.size(); t += 3)
                if (pointTriangleDistance(v, &P[3 * I[t]], &P[3 * I[t + 1]], &P[3 * I[t + 2]]) < r) return true;
        }
        return false;
    };
    FloatClasses fc;
    for (uint32_t i = 0; i < pls.size(); ++i) {
        const Pl& p = pls[i];
        ++fc.total;
        const float origin = p.m[13] - ground.smoothHeightAt(p.m[12], p.m[14]);
        float minGap = 1e9f, maxAbove = -1e9f;
        bool byObject = false;
        for (uint32_t k : p.geo->sample) {
            float w[3];
            xf(p.m, &p.geo->pos[3 * k], w);
            const float g = ground.smoothHeightAt(w[0], w[2]);
            maxAbove = std::max(maxAbove, w[1] - g);
            float gap = w[1] - g;
            if (gap > 0.5f && minGap > 0.5f) {
                const float s = surfaceBelow(i, w[0], w[1], w[2]);
                if (w[1] - s < gap) gap = w[1] - s, byObject = byObject || gap <= 0.5f;
                if (gap > 0.5f && touches(i, w)) gap = 0, byObject = true;
            }
            minGap = std::min(minGap, gap);
        }
        const char* cls;
        if (maxAbove < -0.3f) cls = "buried", ++fc.buried;
        else if (minGap > 0.5f) cls = "floating", ++fc.floating, fc.floating2 += minGap > 2.0f;
        else if (origin > 0.5f) cls = byObject ? "on-object" : "elevated", ++(byObject ? fc.onObject : fc.elevated);
        else cls = "ground", ++fc.onGround;
        fc.sunkOrigin += origin < -0.5f;
        if (list && (std::strcmp(cls, "ground") != 0 || std::fabs(origin) > 0.5f))
            std::fprintf(list, "%s\t%s\t%08x\t%.2f\t%.2f\t%.2f\t%+.2f\t%.2f\t%.2f\n", cls, p.file.c_str(), p.mesh, p.m[12], p.m[13], p.m[14], origin,
                         minGap > 1e8f ? -1.0f : minGap, p.geo->hi[1] - p.geo->lo[1]);
    }
    return fc;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...] [--ground OUT.ppm SIZE]\n");
        return 2;
    }
    try {
        const auto t0 = std::chrono::steady_clock::now();
        fuel::DpcFile dpc(argv[1]);
        const fuel::Heightfield hf = fuel::loadWorldHeightfield(dpc);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        uint16_t lo = 0xFFFF, hi = 0;
        for (uint16_t v : hf.raw) lo = std::min(lo, v), hi = std::max(hi, v);
        std::printf("%u x %u samples, %.3f m spacing, origin (%.1f, %.1f), %.4f m per unit, heights %.1f..%.1f m (%.2f s)\n", hf.width,
                    hf.height, hf.spacing, hf.originX, hf.originZ, hf.metresPerUnit, lo * hf.metresPerUnit, hi * hf.metresPerUnit, secs);
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            if (a == "--levelled") {  // FUEL's ground (procedural detail) for later --at queries
                const_cast<fuel::Heightfield&>(hf) = fuel::loadLevelledHeightfield(dpc);
                std::printf("procedural ground attached\n");
            } else if (a == "--file-flats" && i + 2 < argc) {  // --file-flats X Z: file flat surfaces holding (X, Z) and the height terms
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr);
                i += 2;
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                const fuel::GroundSample g = ground.detail->sample(X, Z);
                std::printf("(%.1f, %.1f): h0 %.2f detail %.2f gulch %.2f flatA %.2f road %.2f flatB %.2f ground %.2f\n", X, Z, g.h0, g.detail, g.gulch,
                            g.flatA, g.road, g.flatB, g.ground);
                for (const auto& f : ground.detail->fileFlatsAt(X, Z)) {
                    std::printf("  flat %u %s%s prio %3d c (%.1f, %.2f, %.1f) len %.1f x %.1f d %+.1f P %.2f str %.2f margin %.1f raw40:", f.index,
                                f.listB ? "B" : "A", f.ellipse ? " ellipse" : "", f.priority, f.cx, f.cy, f.cz, f.lenX, f.lenZ, f.d, f.P, f.strength,
                                f.margin);
                    for (size_t k = 0x40; k < f.raw.size(); ++k) std::printf("%s%02x", (k & 3) ? "" : " ", f.raw[k]);
                    std::printf("\n");
                }
            } else if (a == "--subworlds") {  // p35: USA1's WorldRef_Z table (kind, file, centre, mission)
                for (const fuel::SubWorldRef& r : fuel::loadSubWorldRefs(dpc))
                    std::printf("subworld %08x %-8s flags %08x %-40s centre (%.0f, %.0f) %s\n", r.id,
                                r.kind == fuel::SubWorldRef::Region ? "region" : r.kind == fuel::SubWorldRef::Course ? "course" : "resident", r.flags,
                                r.file.c_str(), r.cx, r.cz, r.mission.c_str());
            } else if (a == "--resident") {  // p35: USA1.DPC's own mesh placements away from the origin library
                size_t n = 0, far = 0;
                std::unordered_map<uint32_t, size_t> perMesh;
                for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(dpc, {&dpc})) {
                    ++n;
                    if (std::fabs(pm.world[12]) < 50 && std::fabs(pm.world[14]) < 50) continue;
                    ++far, ++perMesh[pm.meshId];
                    if (std::getenv("FUEL_RESIDENT_LIST")) std::printf("resident %08x %08x %.1f %.1f %.1f\n", pm.nodeId, pm.meshId, pm.world[12], pm.world[13], pm.world[14]);
                }
                std::printf("USA1 resident mesh placements: %zu, away from the origin %zu (%zu meshes)\n", n, far, perMesh.size());
            } else if (a == "--emu-flat-cmd" && i + 1 < argc) {
                // Writes an EmuRun.java command file (p35 F1): for file flat surfaces next to the
                // regression points, the shape FUEL's loader 0x77d480 builds (SetShape 0x79c330 with
                // ECX = X / (0.5 |X|^2), EDX = Z / (0.5 |Z|^2), float32) and the inside distance
                // 0x799020 (jitter 0) at points spread over the shape; prints the port's distances.
                const std::string out = argv[++i];
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                std::ofstream f(out);
                f.precision(9);
                const uint32_t obj = 0x23000000, C = 0x23001000, AX = 0x23001010, AZ = 0x23001020, P = 0x23001030;
                for (const auto& [index, fa, fb] : kEmuFlatPoints) {
                    const fuel::TerrainDetail::FileFlatInfo info = ground.detail->fileFlat(index, 0, 0);
                    float v[16];
                    std::memcpy(v, info.raw.data() + 4, 64);  // X, N, Z, centre (rows of 4)
                    auto axis = [&](const float* r, uint32_t at) {
                        const float k = float(1.0 / (0.5 * double(float(std::sqrt(double(r[0]) * r[0] + double(r[1]) * r[1] + double(r[2]) * r[2]))) *
                                                     double(float(std::sqrt(double(r[0]) * r[0] + double(r[1]) * r[1] + double(r[2]) * r[2])))));
                        for (int c = 0; c < 3; ++c) f << "wf " << at + 4 * c << " " << float(r[c] * k) << "\n";
                        f << "wf " << at + 12 << " 0\n";
                    };
                    f << "reset\nzero " << obj << " 0xb0\n";
                    for (int c = 0; c < 3; ++c) f << "wf " << C + 4 * c << " " << v[12 + c] << "\n";
                    f << "wf " << C + 12 << " 1\n";
                    axis(v, AX), axis(v + 8, AZ);
                    f << "pushf " << *reinterpret_cast<const float*>(info.raw.data() + 0x4c) << "\nreg ESI " << obj << "\nreg EAX " << C << "\nreg ECX " << AX
                      << "\nreg EDX " << AZ << "\ncall 0x79c330\n";
                    f << "w32 " << obj + 0xa4 << " " << (info.ellipse ? 0x100 : 0) << "\n";  // +0xa5 shape (file +0x82), set by the loader
                    const float x = v[12] + fa * v[0] + fb * v[8], z = v[14] + fa * v[2] + fb * v[10];
                    f << "wf " << P << " " << x << "\nwf " << P + 4 << " " << v[13] << "\nwf " << P + 8 << " " << z << "\nwf " << P + 12 << " 1\n";
                    f << "pushf 0\nreg ESI " << obj << "\nreg EDI " << P << "\ncall 0x799020\n";
                    std::printf("flat %u (%s) at (%.3f, %.3f): port d %.6f\n", index, info.ellipse ? "ellipse" : "rect", x, z,
                                ground.detail->fileFlat(index, x, z).d);
                }
                std::printf("wrote %s\n", out.c_str());
            } else if (a == "--float-dump" && i + 1 < argc) {
                const char* out = argv[++i];
                std::vector<std::unique_ptr<fuel::DpcFile>> subs;
                for (const auto& e : std::filesystem::directory_iterator(std::filesystem::path(argv[1]).parent_path())) {
                    if (e.path().extension() != ".DPC" || e.path().filename() == "USA1.DPC") continue;
                    try {
                        auto d = std::make_unique<fuel::DpcFile>(e.path().string());
                        if (d->isCurrentVersion()) subs.push_back(std::move(d));
                    } catch (const std::exception&) {
                    }
                }
                dumpPlacements(dpc, subs, fuel::loadLevelledHeightfield(dpc), out);
            } else if (a == "--seat" || a == "--float") {
                // Placed objects against the map B-spline (old port ground) and FUEL's ground.
                // --seat: fixed placements within 0.5 / 2 m. --float: every non-FITTOGROUND
                // object whose base is more than 0.5 m (and 4 m) above or below the ground,
                // excluding those on decks (more than 4 m above both).
                std::vector<std::unique_ptr<fuel::DpcFile>> subs;
                for (const auto& e : std::filesystem::directory_iterator(std::filesystem::path(argv[1]).parent_path())) {
                    if (e.path().extension() != ".DPC" || e.path().filename() == "USA1.DPC") continue;
                    try {
                        auto d = std::make_unique<fuel::DpcFile>(e.path().string());
                        if (d->isCurrentVersion()) subs.push_back(std::move(d));
                    } catch (const std::exception&) {
                    }
                }
                const auto l0 = std::chrono::steady_clock::now();
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                std::printf("procedural ground: %zu flat surfaces, %zu gulch lines (%.2f s)\n", ground.detail->flatSurfaceCount(),
                            ground.detail->gulchLineCount(), std::chrono::duration<double>(std::chrono::steady_clock::now() - l0).count());
                size_t total = 0, fit = 0, onRaw = 0, onNew = 0, nearRaw = 0, nearNew = 0;
                size_t offRaw05 = 0, offNew05 = 0, offRaw4 = 0, offNew4 = 0, aboveNew = 0, belowNew = 0;
                double sumAbsNew = 0, sumAbsRaw = 0;
                std::vector<float> devNew;
                const auto q0 = std::chrono::steady_clock::now();
                for (const auto& sub : subs)
                    for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, {sub.get(), &dpc})) {
                        const float x = pm.world[12], y = pm.world[13], z = pm.world[14];
                        if (std::fabs(x) < 50 && std::fabs(z) < 50) continue;
                        if (pm.fitToGround) {
                            ++fit;
                            continue;
                        }
                        ++total;
                        const float dr = y - hf.baseHeightAt(x, z), dn = y - ground.smoothHeightAt(x, z);
                        const float r = std::fabs(dr), l = std::fabs(dn);
                        onRaw += r < 0.5f, onNew += l < 0.5f, nearRaw += r < 2.0f, nearNew += l < 2.0f;
                        offRaw05 += r > 0.5f, offNew05 += l > 0.5f, offRaw4 += r > 4.0f, offNew4 += l > 4.0f;
                        if (l > 0.5f) (dn > 0 ? aboveNew : belowNew)++;
                        if (l < 50) sumAbsNew += l, sumAbsRaw += std::min(r, 50.0f);
                        devNew.push_back(dn);
                        if (std::getenv("FUEL_FLOAT_LIST") && l > 0.5f)
                            std::printf("off %08x %.1f %.2f %.1f %+.2f map %+.2f\n", pm.meshId, x, y, z, dn, dr);
                    }
                const double qs = std::chrono::duration<double>(std::chrono::steady_clock::now() - q0).count();
                std::printf("placements: %zu fixed + %zu FITTOGROUND (%.1f us per height)\n", total, fit, qs * 1e6 / double(std::max<size_t>(total, 1)));
                std::printf("procedural flat surfaces built: %zu, work bumps: %zu\n", ground.detail->proceduralFlatCount(), ground.detail->workBumpCount());
                std::printf("within 0.5 m: map %zu, FUEL ground %zu; within 2 m: map %zu, FUEL ground %zu\n", onRaw, onNew, nearRaw, nearNew);
                if (a == "--float") {
                    // p35: what the origin offsets do not show (see classifyPlacements). FUEL never seats static
                    // FITTOGROUND placements (only AutoMesh spawns 0x778650 read the keyword), so "all" counts them too.
                    std::FILE* list = std::getenv("FUEL_FLOAT_CLASSES") ? std::fopen(std::getenv("FUEL_FLOAT_CLASSES"), "w") : nullptr;
                    const auto c0 = std::chrono::steady_clock::now();
                    for (bool all : {false, true}) {
                        const FloatClasses fc = classifyPlacements(dpc, subs, ground, all, all ? list : nullptr);
                        std::printf("float classes (%s, %zu): on the ground %zu, floating %zu (%zu > 2 m), on another object %zu, elevated origin "
                                    "(mesh reaches the ground) %zu, buried %zu; origin > 0.5 m under the ground %zu\n",
                                    all ? "all incl. FITTOGROUND" : "fixed", fc.total, fc.onGround, fc.floating, fc.floating2, fc.onObject, fc.elevated, fc.buried,
                                    fc.sunkOrigin);
                    }
                    if (list) std::fclose(list);
                    std::printf("float classes: %.1f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - c0).count());
                    std::printf("float: off by > 0.5 m: map %zu, FUEL ground %zu (%zu above, %zu below); > 4 m: map %zu, FUEL ground %zu\n",
                                offRaw05, offNew05, aboveNew, belowNew, offRaw4, offNew4);
                    std::sort(devNew.begin(), devNew.end());
                    if (!devNew.empty())
                        std::printf("float: FUEL ground offset percentiles p10 %.2f p25 %.2f p50 %.2f p75 %.2f p90 %.2f m; mean |d| (< 50 m) map %.2f, ground %.2f\n",
                                    devNew[devNew.size() / 10], devNew[devNew.size() / 4], devNew[devNew.size() / 2], devNew[devNew.size() * 3 / 4],
                                    devNew[devNew.size() * 9 / 10], sumAbsRaw / double(total), sumAbsNew / double(total));
                }
            } else if (a == "--rivers" && i + 3 < argc) {  // --rivers X Z R: river water (0x7a07d0) on an 8 m grid
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr), R = std::strtod(argv[i + 3], nullptr);
                i += 3;
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                size_t riverPts = 0, wetPts = 0;
                double bx = 0, bz = 0, bd = 0;
                for (double z = Z - R; z <= Z + R; z += 8)
                    for (double x = X - R; x <= X + R; x += 8) {
                        if (!ground.detail->river(x, z, 0).flag) continue;
                        ++riverPts;
                        const double h = ground.smoothHeightAt(x, z);
                        const fuel::WaterPoint w = fuel::waterAt(nullptr, ground.detail.get(), x, z, h);
                        if (w.under()) {
                            ++wetPts;
                            if (w.depth - w.base > bd) bd = w.depth - w.base, bx = x, bz = z;
                        }
                    }
                std::printf("rivers within %.0f m of (%.0f, %.0f): %zu carve points, %zu under water; deepest %.2f m at (%.0f, %.0f)\n", R, X, Z, riverPts,
                            wetPts, bd, bx, bz);
            } else if (a == "--sand" && i + 3 < argc) {  // --sand X Z R: Dune / SandTexture / zone on a 4 m grid (p34)
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr), R = std::strtod(argv[i + 3], nullptr);
                i += 3;
                const fuel::Heightfield levelled = fuel::loadLevelledHeightfield(dpc);
                fuel::GroundGenerator gen(dpc, levelled, nullptr);
                if (levelled.roads) gen.setRoadSampler(levelled.roads.get());
                fuel::Lakes lakes(dpc, [&](double x, double z) { return levelled.baseHeightAt(x, z); });
                gen.setLakes(&lakes);
                size_t n = 0, dune = 0, sand = 0, zone = 0, wet = 0, types[40] = {};
                double sumDune = 0;
                for (double z = Z - R; z <= Z + R; z += 4)
                    for (double x = X - R; x <= X + R; x += 4) {
                        const fuel::GroundPoint p = gen.at(x, z);
                        ++n, dune += p.dune > 0.01f, sand += p.sand > 0.01f, zone += p.zone != 0, wet += p.underWater(), sumDune += p.dune;
                        for (int k = 0; k < 4; ++k)
                            if (p.surfaceType[k] < 40 && p.surfaceWeight[k] > 0) types[p.surfaceType[k]]++;
                    }
                std::printf("sand around (%.0f, %.0f) r %.0f: %zu points, Dune > 0.01 at %zu (mean %.3f), SandTexture > 0.01 at %zu, zone at %zu, under water %zu\n",
                            X, Z, R, n, dune, sumDune / double(std::max<size_t>(n, 1)), sand, zone, wet);
                std::printf("  points with SAND/SAND2 %zu/%zu, WETSAND/WETSAND2 %zu/%zu, ROUGHSAND/2 %zu/%zu\n", types[6], types[7], types[36], types[37],
                            types[38], types[39]);
            } else if (a == "--wet" && i + 2 < argc) {  // --wet X Z: the ground point's water terms (P83: the spray depth)
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr);
                i += 2;
                const fuel::Heightfield levelled = fuel::loadLevelledHeightfield(dpc);
                fuel::GroundGenerator gen(dpc, levelled, nullptr);
                if (levelled.roads) gen.setRoadSampler(levelled.roads.get());
                fuel::Lakes lakes(dpc, [&](double x, double z) { return levelled.baseHeightAt(x, z); });
                gen.setLakes(&lakes);
                if (const char* line = std::getenv("FUEL_WET_LINE")) {  // "X2,Z2": 1 m steps from (X, Z), cells with depth > 0
                    double X2 = X, Z2 = Z;
                    std::sscanf(line, "%lf,%lf", &X2, &Z2);
                    const double L = std::hypot(X2 - X, Z2 - Z);
                    int wetCells = 0, n = 0;
                    std::string row;
                    for (double t = 0; t <= L; t += 1, ++n) {
                        float fr[fuel::GroundGenerator::kGroundClasses] = {}, d = 0;
                        const double x = X + (X2 - X) * t / std::max(L, 1e-9), z = Z + (Z2 - Z) * t / std::max(L, 1e-9);
                        gen.classFractions(x, std::nan(""), z, fr, d);
                        wetCells += d > 0;
                        row += d > 0 ? '#' : '.';
                    }
                    std::printf("wet line (%.0f, %.0f) -> (%.0f, %.0f): %d of %d 1 m steps with depth > 0\n%s\n", X, Z, X2, Z2, wetCells, n, row.c_str());
                }
                const fuel::GroundPoint p = gen.at(X, Z);
                float frac[fuel::GroundGenerator::kGroundClasses] = {}, depth = 0;
                gen.classFractions(X, std::nan(""), Z, frac, depth);
                std::printf("wet at (%.1f, %.1f): height %.2f water depth %.3f base %.2f puddle %.3f humidity %.3f surface humidity %.3f "
                            "normal y %.4f road %.2f -> class depth %.3f\n",
                            X, Z, p.height, p.waterDepth, p.waterBase, p.puddle, p.humidity, p.surfaceHumidity, p.normal[1], p.road, depth);
            } else if (a == "--emu-map-cmd" && i + 1 < argc) {
                // P80: EmuRun.java commands that run FUEL.exe's own map sampler set-up and samplers on this world's
                // maps 8 (population), 9 (occlusion) and 14 (stormed): InitMapSamplers 0x77bb80 over a GenWorld whose
                // slot table (G+0xb748c) holds only that map (Binary_Z body loaded from DIR/mapN.bin, header at +4:
                // w, h, f32 span, i32, f32 max; data at +24), then the inlined per-map samplers 0x458e60 (map 8),
                // 0x772110 (map 9) and 0x458780 (map 14), which all read the sampler array at P + 0x14c68 + 0x1814 k
                // (P = G + 0x498c). Points sit 0.1..0.45 of a texel past a texel centre so the emulator's
                // round-to-nearest fistp equals the game's round-down. Prints the port's mapAt for each point.
                const std::string dir = argv[++i];
                const std::vector<uint32_t> ids = fuel::genWorldMaps(dpc);
                const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                fuel::GroundGenerator gen(dpc, lv, nullptr);
                std::ofstream f(dir + "/map.cmd");
                f.precision(9);
                const uint32_t G = 0x20000000, RES = 0x2f000000, BODY = 0x30000000, PT = 0x2f100000, OUT = 0x2f200000;
                const double spots[][2] = {{-39570.0, -28068.0}, {45850.0, -6250.0}, {4900.0, 35538.0}, {-51400.0, -30100.0}, {12000.0, -51000.0}, {300.0, 700.0}};
                const double fr[][2] = {{0.13, 0.31}, {0.42, 0.11}, {0.27, 0.27}, {0.36, 0.44}, {0.19, 0.4}, {0.1, 0.22}};
                const std::pair<int, const char*> fns[] = {{8, "0x458e60"}, {9, "0x772110"}, {14, "0x458780"}, {2, "0x4dde20"}, {10, "0x7a2880"},
                                                           {13, "0x458100"}, {15, "0x457a80"}, {16, "0x4573e0"}, {18, "0x646730"}};
                // (Maps 3, 5, 6, 7 are read by composite functions 0x45d010 / 0x459530 / 0x45c270 / 0x6bd200 that also need
                // fractal banks; maps 10 and 18 are 0 at these points.)
                for (const auto& [slot, fn] : fns) {
                    const fuel::DpcFile::Entry* e = size_t(slot) < ids.size() ? dpc.find(ids[size_t(slot)]) : nullptr;
                    const auto body = e ? dpc.body(*e) : std::nullopt;
                    if (!body) continue;
                    const std::string bin = dir + "/map" + std::to_string(slot) + ".bin";
                    std::ofstream(bin, std::ios::binary).write(reinterpret_cast<const char*>(body->data()), std::streamsize(body->size()));
                    uint32_t w = 0;
                    std::memcpy(&w, body->data() + 4, 4);
                    const double sp = 140000.0 / w;
                    f << "reset\nzero " << G << " 0xb7600\nload " << BODY << " " << bin << "\nw32 " << (G + 0xb748c + 4u * uint32_t(slot)) << " " << RES
                      << "\nw32 " << (RES + 0xc) << " " << (BODY + 4) << "\npush32 " << G << "\ncall 0x77bb80 50000000\n";
                    for (size_t k = 0; k < std::size(spots); ++k) {
                        const double tx = std::floor(spots[k][0] / sp + w / 2.0) + fr[k][0], tz = std::floor(spots[k][1] / sp + w / 2.0) + fr[k][1];
                        const float x = float((tx - w / 2.0) * sp), z = float((tz - w / 2.0) * sp);
                        f << "wf " << PT << " " << x << "\nwf " << (PT + 4) << " 0\nwf " << (PT + 8) << " " << z << "\nwf " << (PT + 12) << " 0\n";
                        f << "reg EAX " << PT << "\nreg ECX " << (G + 0x498c) << "\nreg EBX " << OUT << "\nreg ESI " << PT << "\nreg EDI " << PT << "\n";
                        f << "zero " << OUT << " 0x60\n## slot " << slot << " point " << k << "\ncall " << fn << "\n";
                        std::printf("slot %d point %zu (%.9g, %.9g): port mapAt %.9g\n", slot, k, double(x), double(z), double(gen.map(slot, x, z)));
                    }
                }
                std::printf("map sampler commands written to %s/map.cmd\n", dir.c_str());
            } else if (a == "--emu-trunk-cmd" && i + 1 < argc) {
                // P52: an EmuRun.java command file that runs FUEL.exe's trunk radius code (0x785200 from 0x78531d to
                // 0x78536e, trees; 0x788ab0 from 0x788bd3 to 0x788c33, bushes: species +0x78, the noise 0x784370 /
                // 0x787c70 over this world's random table, the size lerp, x 6, min 2) for plants the port scatters
                // near four spots; prints the port's radius for each. Plants are kept only when the noise argument
                // sits 0.05..0.45 past an integer, so the emulator's round-to-nearest fstp / fistp give the game's
                // round-down index.
                const std::string out = argv[++i];
                const fuel::Vegetation veg = fuel::loadVegetation(dpc);
                const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                std::ofstream f(out);
                auto bits = [](float v) {
                    uint32_t b;
                    std::memcpy(&b, &v, 4);
                    char t[16];
                    std::snprintf(t, sizeof t, "0x%08x", b);
                    return std::string(t);
                };
                const uint32_t RB = 0x20000000, OBJ = 0x21000000, TBL = 0x22000000, OUT = 0x23000000, FR = 0x0fd00000, SP = 0x0fe00000;
                const double spots[][2] = {{46809.6, -1284.4}, {14778.0, 18650.8}, {43532.0, -49504.4}, {-39570.0, -28068.0}};
                int count = 0;
                for (const auto& q : spots) {
                    std::vector<fuel::Plant> ps;
                    fuel::scatterPlants(veg, lv, nullptr, nullptr, float(q[0]), float(q[1]), 80.0f, ps);
                    int trees = 0, bushes = 0;
                    // Each kept plant also stands in for species with other trunk values (TREES 4: 0.5, 5: 0.64,
                    // 10: 0.1; BUSHES 18: 0.1): the radius maths at that point.
                    std::vector<fuel::Plant> cases;
                    for (fuel::Plant p : ps) {
                        cases.push_back(p);
                        const uint16_t alt = p.tree ? uint16_t(4 + (cases.size() % 3 == 0 ? 6 : cases.size() % 3 == 1 ? 0 : 1)) : uint16_t(18);
                        if (alt <= (p.tree ? veg.trees : veg.bushes).size()) p.species = alt, cases.push_back(p);
                    }
                    for (const fuel::Plant& p : cases) {
                        const std::vector<fuel::Species>& list = p.tree ? veg.trees : veg.bushes;
                        const fuel::Species& sp = list[p.species - 1];
                        if ((sp.flags & 2) || (p.tree ? trees >= 4 : bushes >= 2)) continue;
                        const double h = (double(p.position[0]) + double(425.23886f)) * double(12.317513f) + double(p.position[2]) * double(10.193578f);
                        const double fr = h - std::floor(h);
                        if (fr < 0.05 || fr > 0.45) continue;
                        (p.tree ? trees : bushes)++, ++count;
                        f << "reset\n";
                        for (int k = 0; k < 64; ++k) {
                            const uint32_t v = uint32_t(veg.perm[size_t(4 * k)]) | uint32_t(veg.perm[size_t(4 * k + 1)]) << 8 |
                                               uint32_t(veg.perm[size_t(4 * k + 2)]) << 16 | uint32_t(veg.perm[size_t(4 * k + 3)]) << 24;
                            f << "w32 " << (TBL + 0x704 + 4 * k) << " " << v << "\n";
                        }
                        const uint32_t rec = RB + uint32_t(p.species - 1) * 0xf8;
                        f << "w32 " << (OBJ + 0x148) << " " << TBL << "\nw32 0xa6ae68 " << RB << "\nw32 0xa6ae70 " << RB << "\n";
                        f << "w32 " << (rec + 0x44) << " " << bits(sp.size.lo) << "\nw32 " << (rec + 0x48) << " " << bits(sp.size.hi) << "\nw32 "
                          << (rec + 0x78) << " " << bits(sp.trunk) << "\n";
                        f << "w32 " << (SP + 0x10) << " " << bits(p.position[0]) << "\nw32 " << (SP + 0x14) << " " << bits(p.position[1]) << "\nw32 "
                          << (SP + 0x18) << " " << bits(p.position[2]) << "\nw32 " << (FR + 8) << " " << OBJ << "\nw32 " << OUT << " 0\n";
                        f << "sp " << SP << "\nreg EAX " << RB << "\nreg EBX " << OUT << "\nreg EBP " << FR << "\n";
                        if (p.tree)
                            f << "reg ESI " << uint32_t(p.species) * 0xf8 << "\nreg ECX " << OBJ << "\nrun 0x78531d 0x78536e\n";
                        else
                            f << "reg ESI " << p.species << "\nreg ECX " << uint32_t(p.species) * 0xf8 << "\nrun 0x788bd3 0x788c33\n";
                        f << "rf " << OUT << "\n";
                        std::printf("{%d, %u, %.9gf, %.9gf, /* emulated */ 0}, // port r %.9g, k %.3f, size %.3f..%.3f, n %.6f\n", p.tree ? 1 : 0, p.species,
                                    p.position[0], p.position[2], fuel::trunkRadius(veg, sp, p.position[0], p.position[2]), sp.trunk, sp.size.lo, sp.size.hi,
                                    fuel::plantNoise(veg, p.position[0], p.position[2]));
                    }
                }
                std::printf("%d trunk vectors written to %s\n", count, out.c_str());
            } else if (a == "--emu-fractal-cmd" && i + 1 < argc) {
                // Writes an EmuRun.java command file (tools/ghidra, read-only emulation of FUEL.exe) that
                // builds three Fractal objects with Fractal_Init 0x69b1b0 over this world's random table
                // and evaluates them through 0x79d470 (F 0x5efc), 0x6a88d0 (F 0x11cec) and 0x79eed0
                // (E of F 0x7ea4) at sample points; prints the port's values for the same points.
                // The points sit at a small fraction (< 0.5) of a finest cell from a top-cell corner so
                // the emulator's round-to-nearest fistp equals the game's round-down (p34 V1).
                const std::string out = argv[++i];
                const fuel::Permutation perm = fuel::buildPermutation(dpc, fuel::genWorldSeed(dpc));
                std::ofstream f(out);
                f.precision(12);
                const uint32_t G = 0x20000000, bank = 0x21000000, reg = 0x22000000, arr = 0x22001000;
                f << "reset\nzero " << G << " 0x800\nzero " << bank << " 0x13000\nzero " << reg << " 0x10\nzero " << arr << " 0x400\n";
                for (int k = 0; k < 64; ++k) {
                    const uint32_t v = uint32_t(perm[size_t(4 * k)]) | uint32_t(perm[size_t(4 * k + 1)]) << 8 | uint32_t(perm[size_t(4 * k + 2)]) << 16 |
                                       uint32_t(perm[size_t(4 * k + 3)]) << 24;
                    f << "w32 " << (G + 0x704 + 4 * k) << " " << v << "\n";
                }
                // Fractal registry DAT_00a65890 pre-made (count 1, capacity 64) so 0x69d390 does not grow it.
                f << "w32 0xa65890 " << reg << "\nw32 " << reg << " 0x00010000\nw32 " << (reg + 8) << " 0x00400000\nw32 " << (reg + 12) << " " << arr << "\n";
                struct Fr {
                    uint32_t off, fn;
                    int L;
                    float S, seed, p4, p5, p6;
                    bool e;
                };
                const Fr frs[] = {{0x5efc, 0x79d470, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f, false},
                                  {0x11cec, 0x6a88d0, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f, false},
                                  {0x7ea4, 0x79eed0, 4, 3000.0f, 9.647993f, 0.025f, 0.15f, 0.5f, true}};
                for (const Fr& r : frs) {
                    const uint32_t obj = bank + r.off;
                    f << "pushf " << r.p6 << "\npushf " << r.p5 << "\npushf " << r.p4 << "\npushf " << r.seed << "\npushf " << r.S << "\npush32 " << r.L
                      << "\nreg EBX " << obj << "\nreg EAX " << G << "\ncall 0x69b1b0\n";
                    // The seed offset is a fistp (round-down in the game, nearest in the emulator).
                    f << "w32 " << (obj + 0xb44) << " " << uint32_t(int32_t(std::floor(double(r.seed) * 262897.0))) << "\n";
                    const fuel::Fractal F(perm, r.L, r.S, r.seed, r.p4, r.p5, r.p6);
                    const double fine = double(r.S) / double(1 << r.L);
                    const double pts[][2] = {{-39600.0, -28080.0}, {45900.0, -6300.0}, {0.0, 0.0}, {12000.0, -51000.0}};
                    const double fr[][2] = {{0.1, 0.3}, {0.45, 0.05}, {0.2, 0.2}, {0.33, 0.41}};
                    for (int k = 0; k < 4; ++k) {
                        const double x = std::floor(pts[k][0] / r.S) * r.S + fr[k][0] * fine, z = std::floor(pts[k][1] / r.S) * r.S + fr[k][1] * fine;
                        f << "wf 0x23000000 " << x << "\nwf 0x23000008 " << z << "\nreg ECX " << bank << "\nreg EAX 0x23000000\ncall " << r.fn << "\n";
                        double v = F.at(x, z);
                        if (r.e) {
                            auto P8 = [](double t) { return t <= 0 ? std::pow(t + 1, 8) - 1 : 1 - std::pow(1 - t, 8); };
                            v = 0.5 + 0.5 * P8(P8(2 * v * std::fabs(v) - 1));
                        }
                        std::printf("port 0x%x (%.4f, %.4f) = %.9g\n", r.fn, x, z, v);
                    }
                }
                // Canyon terraces 0x7a0c80 (object W+0x48404: +0xc / +0x10 = Fractal_Init(5, 100, 25.442329,
                // 0.3, 0.3, 2) / (5, 50, 28.442329, ...), 0x7a0b20) at scaled points, a = 10, b = 50.
                {
                    const uint32_t obj = 0x24000000, fc = 0x25000000, fd = 0x25002000;
                    f << "zero " << obj << " 0x40\nzero " << fc << " 0x4000\nw32 " << (obj + 0xc) << " " << fc << "\nw32 " << (obj + 0x10) << " " << fd << "\n";
                    for (const auto& [o, S, seed] : {std::tuple{fc, 100.0f, 25.442329f}, std::tuple{fd, 50.0f, 28.442329f}}) {
                        f << "pushf 2\npushf 0.3\npushf 0.3\npushf " << seed << "\npushf " << S << "\npush32 5\nreg EBX " << o << "\nreg EAX " << G
                          << "\ncall 0x69b1b0\n";
                        f << "w32 " << (o + 0xb44) << " " << uint32_t(int32_t(std::floor(double(seed) * 262897.0))) << "\n";
                    }
                    const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                    const double cp[][2] = {{-158399.7, -112299.8}, {183600.3, -25199.8}, {400.1, 800.3}, {48000.2, -203999.9}};
                    for (const auto& q : cp) {
                        f << "wf 0x23000000 " << q[0] << "\nwf 0x23000008 " << q[1] << "\npushf 50\npushf 10\npush32 0x23000000\npush32 " << obj
                          << "\ncall 0x7a0c80\n";
                        const double down = lv.detail->canyonTerraces(q[0], q[1], 10, 50);
                        fuel::setFloat32RoundNearest(true);
                        const double near = lv.detail->canyonTerraces(q[0], q[1], 10, 50);
                        fuel::setFloat32RoundNearest(false);
                        std::printf("port 0x7a0c80 (%.4f, %.4f) = %.9g (stores rounded to nearest: %.9g)\n", q[0], q[1], down, near);
                    }
                }
            } else if (a == "--decor-density" && i + 2 < argc) {  // --decor-density X Z: 0x6b7f10 and its two stubbed inputs
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr);
                i += 2;
                const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                fuel::GroundGenerator gen(dpc, lv, nullptr);
                float h = 0, c = 0;
                const float d = gen.decorDensity(X, Z, &h, &c);
                std::printf("decor density (%.4f, %.4f) = %.9g humidity %.9g clearing %.9g\n", X, Z, d, h, c);
            } else if (a == "--flats" && i + 3 < argc) {  // --flats X Z R: procedural flat surfaces (0x799910) near
                const double X = std::strtod(argv[i + 1], nullptr), Z = std::strtod(argv[i + 2], nullptr), R = std::strtod(argv[i + 3], nullptr);
                i += 3;
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                for (const auto& f : ground.detail->proceduralFlats(X, Z, R))
                    std::printf("flat c (%.1f, %.2f, %.1f) L2 %.1f D %.1f dir (%.3f, %.3f) n (%.3f, %.3f) margin %.1f k %.1f mode %d; ground at c %.2f\n", f.cx, f.cy,
                                f.cz, f.lenX, f.lenZ, f.ax, f.az, f.nx, f.nz, f.margin, f.strength, f.mode, ground.smoothHeightAt(f.cx, f.cz));
            } else if (a == "--near" && i + 3 < argc) {  // --near X Z R: placed objects and the ground under them
                const float X = std::strtof(argv[i + 1], nullptr), Z = std::strtof(argv[i + 2], nullptr), R = std::strtof(argv[i + 3], nullptr);
                i += 3;
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                // FUEL_P22_CARVE=1: the ground with the roads work's 0x6fcaa0 (RoadSampler::carve)
                // instead of terrain_height's own road decode, to compare the two against placed objects.
                std::unique_ptr<fuel::RoadSampler> p22;
                if (std::getenv("FUEL_P22_CARVE") && ground.detail) {
                    p22 = std::make_unique<fuel::RoadSampler>(dpc, fuel::loadRoadTypeTable(dpc),
                                                              [&](double px, double pz) { return ground.detail->detailHeight(px, pz); });
                    const_cast<fuel::TerrainDetail*>(ground.detail.get())
                        ->setRoadCarver([&](double px, double pz, double& y, double c, double w) { return p22->carve(px, pz, y, c, w); });
                }
                for (const auto& e : std::filesystem::directory_iterator(std::filesystem::path(argv[1]).parent_path())) {
                    if (e.path().extension() != ".DPC" || e.path().filename() == "USA1.DPC") continue;
                    try {
                        fuel::DpcFile sub(e.path().string());
                        if (!sub.isCurrentVersion()) continue;
                        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(sub, {&sub, &dpc})) {
                            const float x = pm.world[12], y = pm.world[13], z = pm.world[14];
                            if ((x - X) * (x - X) + (z - Z) * (z - Z) > R * R) continue;
                            const fuel::GroundSample g = ground.detail->sample(x, z);
                            std::printf("  %08x %s at (%.1f, %.2f, %.1f): ground %.2f (y - ground %+.2f) map %.2f detail %+.2f rock %.2f w18 %.2f%s\n", pm.meshId,
                                        e.path().filename().string().c_str(), x, y, z, g.ground, y - g.ground, g.h0, g.h2 - g.h0, g.rock, g.w18,
                                        pm.fitToGround ? " FIT" : "");
                        }
                    } catch (const std::exception&) {
                    }
                }
            } else if (a == "--chunks" && i + 2 < argc) {  // --chunks X Z: time ground chunk builds of every size there
                const float X = std::strtof(argv[i + 1], nullptr), Z = std::strtof(argv[i + 2], nullptr);
                i += 2;
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                for (double c = 24; c <= 12288; c *= 2) {
                    const auto c0 = std::chrono::steady_clock::now();
                    const auto v = fuel::buildTerrainChunk(ground, X + 3 * c, Z, c);
                    std::printf("chunk %5.0f m: %zu vertices in %.1f ms\n", c, v.size(),
                                std::chrono::duration<double>(std::chrono::steady_clock::now() - c0).count() * 1e3);
                }
            } else if (a == "--vectors") {
                // Terrain checks (notes p31 "Vectors"). (1) Morph hand-over, 0x7f9b50 (p31 E6): every vertex of a
                // chunk of size C morphs to the even point (i & ~1, j & ~1) as the parent chunk (2C) builds it, so
                // at full tween the chunk is exactly its parent: target height and normal must equal the parent's
                // own vertex there, for every level from 24 m to 6144 m. (2) Heights at fixed points against the
                // values recorded in notes p31 (port regression; FUEL.exe's 0x64a2a0 itself cannot be emulated
                // without the runtime GenWorld state).
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                int bad = 0, checked = 0;
                const double px = -39570.0, pz = -28068.0;  // CR2-Yell-CP start
                double cx = std::round(px / 12288.0) * 12288.0, cz = std::round(pz / 12288.0) * 12288.0, C = 12288.0;
                std::vector<std::array<double, 3>> path;  // centre x, z, size from the root down to a 24 m leaf
                path.push_back({cx, cz, C});
                while (C > 24.01) {
                    const double q = C * 0.25;
                    cx += px >= cx ? q : -q, cz += pz >= cz ? q : -q, C *= 0.5;
                    path.push_back({cx, cz, C});
                }
                std::vector<fuel::TerrainVertex> parent = fuel::buildTerrainChunk(ground, path[0][0], path[0][1], path[0][2]);
                for (size_t k = 1; k < path.size(); ++k) {
                    const auto child = fuel::buildTerrainChunk(ground, path[k][0], path[k][1], path[k][2]);
                    const double Cc = path[k][2], s = Cc / 24.0;
                    const int oi = int(std::lround((path[k][0] - path[k - 1][0]) / (Cc / 2) + 1)) * 6;  // 0 or 12
                    const int oj = int(std::lround((path[k][1] - path[k - 1][1]) / (Cc / 2) + 1)) * 6;
                    float worstY = 0, worstN = 0, worstXZ = 0;
                    for (int j = 0; j < fuel::kChunkSide; ++j)
                        for (int i = 0; i < fuel::kChunkSide; ++i) {
                            const fuel::TerrainVertex& v = child[size_t(j) * fuel::kChunkSide + size_t(i)];
                            const fuel::TerrainVertex& p = parent[size_t(oj + (j >> 1)) * fuel::kChunkSide + size_t(oi + (i >> 1))];
                            worstXZ = std::max({worstXZ, std::fabs(v.p1[0] - p.p0[0]), std::fabs(v.p1[2] - p.p0[2])});
                            worstY = std::max(worstY, std::fabs(v.p1[1] - p.p0[1]));
                            for (int c = 0; c < 3; ++c) worstN = std::max(worstN, std::fabs(v.n1[c] - p.n[c]));
                        }
                    (void)s;
                    const bool ok = worstXZ < 1e-3f && worstY < 1e-3f && worstN < 2e-3f;
                    ++checked, bad += !ok;
                    std::printf("morph %5.0f m -> parent %5.0f m: target xz %.4f, y %.4f, normal %.5f %s\n", Cc, path[k - 1][2], worstXZ, worstY, worstN,
                                ok ? "match" : "MISMATCH");
                    parent = child;
                }
                struct Pt {
                    double x, z, h;
                };
                // Port values (notes p31 "Vectors"), recorded after the morph/LOD port (heights unchanged by it).
                // (45850, -6250) re-recorded by p35 (file flat surfaces half the size, F1; was 1153.995).
                const Pt pts[] = {{-39570.0, -28068.0, 863.728}, {45850.0, -6250.0, 1154.503}, {62652.4, 12699.0, 896.835},
                                  {-39700.0, -28300.0, 898.577}, {0.0, 0.0, 1750.413}};
                for (const Pt& q : pts) {
                    const double h = ground.smoothHeightAt(q.x, q.z);
                    const bool ok = std::fabs(h - q.h) < 2e-3;
                    ++checked, bad += !ok;
                    std::printf("height (%.1f, %.1f) = %.3f (recorded %.3f) %s\n", q.x, q.z, h, q.h, ok ? "match" : "MISMATCH");
                }
                // (3) Emulated vectors (p34 V1/V2): FUEL.exe's Fractal_Init 0x69b1b0 + the evaluators 0x79d470 (bank
                // F 0x5efc), 0x6a88d0 (F 0x11cec), 0x79eed0 (E of F 0x7ea4) and the canyon terraces 0x7a0c80, run
                // under Ghidra's EmulatorHelper (tools/ghidra/EmuRun.java, read only) over this world's random table
                // (command file from --emu-fractal-cmd). Points sit < 0.5 finest cell past a cell corner so the
                // emulator's round-to-nearest fistp equals the game's round-down; the canyon is compared with the
                // port's float32 stores rounded to nearest like the emulator (setFloat32RoundNearest).
                {
                    const fuel::Permutation perm = fuel::buildPermutation(dpc, fuel::genWorldSeed(dpc));
                    const fuel::Fractal FN(perm, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f), FJ(perm, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f),
                        FE(perm, 4, 3000.0f, 9.647993f, 0.025f, 0.15f, 0.5f);
                    auto P8 = [](double t) { return t <= 0 ? std::pow(t + 1, 8) - 1 : 1 - std::pow(1 - t, 8); };
                    struct EV {
                        int f;  // 0 N, 1 J, 2 E, 3 canyon
                        double x, z, emu, tol;
                    };
                    const EV ev[] = {{0, -39599.25, -28077.75, 0.184621947, 2e-5},   {0, 45903.375, -6299.625, 0.635593992, 2e-5},
                                     {0, 1.5, 1.5, 0.0281049758, 2e-5},             {0, 12002.475, -50996.925, 0.663505909, 2e-5},
                                     {1, -39599.75, -28079.25, 0.312729538, 2e-5},   {1, 45901.125, -6299.875, 0.751545586, 2e-5},
                                     {1, 0.5, 0.5, 0.332733206, 2e-5},               {1, 12000.825, -50998.975, 0.636388680, 2e-5},
                                     {2, -41981.25, -29943.75, 2.25107422e-13, 1e-6}, {2, 45084.375, -8990.625, 0.0, 1e-6},
                                     {2, 37.5, 37.5, 1.0, 1e-6},                     {2, 12061.875, -50923.125, 1.0, 1e-6},
                                     {3, -158399.7, -112299.8, 32.8392295, 1e-4},    {3, 183600.3, -25199.8, 40.8465797, 1e-4},
                                     {3, 400.1, 800.3, 52.5867159, 1e-4},            {3, 48000.2, -203999.9, 27.2667041, 5e-4}};
                    const char* names[] = {"0x79d470 F5efc", "0x6a88d0 F11cec", "0x79eed0 E(F7ea4)", "0x7a0c80 canyon"};
                    for (const EV& e : ev) {
                        const double x = double(float(e.x)), z = double(float(e.z));  // the engine reads a float3
                        double v;
                        if (e.f == 0) v = FN.at(x, z);
                        else if (e.f == 1) v = FJ.at(x, z);
                        else if (e.f == 2) {
                            const double u = FE.at(x, z);
                            v = 0.5 + 0.5 * P8(P8(2 * u * std::fabs(u) - 1));
                        } else {
                            fuel::setFloat32RoundNearest(true);
                            v = ground.detail->canyonTerraces(x, z, 10, 50);
                            fuel::setFloat32RoundNearest(false);
                        }
                        const bool ok = std::fabs(v - e.emu) <= e.tol;
                        ++checked, bad += !ok;
                        std::printf("emulated %-18s (%.3f, %.3f) = %.9g, port %.9g %s\n", names[e.f], e.x, e.z, e.emu, v, ok ? "match" : "MISMATCH");
                    }
                }
                // (4) Port regression for p34's pieces (recorded values; no engine reference exists for them).
                {
                    const auto fl = ground.detail->proceduralFlats(-39600, -28080, 300);
                    int mode1 = -1;
                    for (size_t k = 0; k < fl.size(); ++k)
                        if (fl[k].mode == 1) mode1 = int(k);
                    const bool okF = fl.size() == 5 && mode1 >= 0 && std::fabs(fl[size_t(mode1)].cx - -39638.7) < 0.1 &&
                                     std::fabs(fl[size_t(mode1)].cz - -28071.6) < 0.1 && std::fabs(fl[size_t(mode1)].lenZ - 59.3) < 0.1;
                    ++checked, bad += !okF;
                    std::printf("procedural flats within 300 m of the CR2 start: %zu (recorded 5, one mode 1 at (-39638.7, -28071.6), D 59.3) %s\n", fl.size(),
                                okF ? "match" : "MISMATCH");
                    const double hf = ground.smoothHeightAt(-39611.1, -28089.3);
                    const bool okH = std::fabs(hf - 866.607) < 2e-3;
                    ++checked, bad += !okH;
                    std::printf("height on that flat (-39611.1, -28089.3) = %.3f (recorded 866.607) %s\n", hf, okH ? "match" : "MISMATCH");
                    const double hr = ground.smoothHeightAt(-37367, -27769);
                    const fuel::WaterPoint w = fuel::waterAt(nullptr, ground.detail.get(), -37367, -27769, hr);
                    const bool okR = std::fabs((w.depth - w.base) - 6.871) < 2e-3 && w.riverShare > 0.99f;
                    ++checked, bad += !okR;
                    std::printf("river water depth at (-37367, -27769) = %.3f m, river share %.2f (recorded %.3f) %s\n", w.depth - w.base, w.riverShare, 6.871,
                                okR ? "match" : "MISMATCH");
                    fuel::GroundGenerator gen(dpc, ground, nullptr);
                    const fuel::GroundPoint gp = gen.at(4900, 35538);
                    const bool okD = std::fabs(gp.dune - 1.0f) < 1e-4f && gp.sand > 0.5f;
                    ++checked, bad += !okD;
                    std::printf("Dune at the Sand Dunes (4900, 35538) = %.3f, SandTexture %.3f (recorded 1, > 0.5) %s\n", gp.dune, gp.sand, okD ? "match" : "MISMATCH");
                }
                // (5) Emulated file flat-surface distances (p35 F1): SetShape 0x79c330 fed the axes FUEL's loader
                // 0x77d480 builds (ECX = X / (0.5 |X|^2), EDX = Z / (0.5 |Z|^2)), then 0x799020 with jitter 0, under
                // EmuRun.java (command file from --emu-flat-cmd); the points are kEmuFlatPoints in order.
                {
                    const double emu[] = {-1.22354739, 1.60087672, -9.73431509, 0.0, 3.21735287, 1.16814735, 6.48721729, 0.645581101, -17.1307408};
                    size_t k = 0;
                    for (const EmuFlatPoint& q : kEmuFlatPoints) {
                        const auto info = ground.detail->fileFlat(q.index, 0, 0);
                        float v[16];
                        std::memcpy(v, info.raw.data() + 4, 64);
                        const float x = v[12] + q.fx * v[0] + q.fz * v[8], z = v[14] + q.fx * v[2] + q.fz * v[10];
                        const double d = ground.detail->fileFlat(q.index, x, z).d;
                        const bool ok = std::fabs(d - emu[k]) < 1e-3;
                        ++checked, bad += !ok;
                        std::printf("emulated 0x799020 flat %u %s (%.3f, %.3f) = %.6f, port %.6f %s\n", q.index, info.ellipse ? "ellipse" : "rect", x, z,
                                    emu[k], d, ok ? "match" : "MISMATCH");
                        ++k;
                    }
                }
                // (6) Emulated trunk radii (P52, notes p52-trunks.md): FUEL.exe's 0x785200 (0x78531d..0x78536e, trees) and
                // 0x788ab0 (0x788bd3..0x788c33, bushes) under EmuRun.java (command file from --emu-trunk-cmd): species
                // +0x78 x the size lerp (+0x44 / +0x48) at the noise 0x784370 / 0x787c70 x 6, min 2, over this world's
                // random table and species records. The emulator stores floats to nearest, the game (and the port)
                // round down: one float ulp apart at most. Points: plants the port scatters near Boysen Canyon_VP_1,
                // Sand Dunes_VP_4, Salt Lake_VP_2 and the CR2 start, some with another species' record.
                {
                    const fuel::Vegetation veg = fuel::loadVegetation(dpc);
                    struct TV {
                        int tree;
                        unsigned species;  // 1-based record index
                        float x, z, emu;
                    };
                    const TV tv[] = {
                        {1, 59, 46790.7969f, -1305.6001f, 1.1796293f},
                        {1, 10, 46790.7969f, -1305.6001f, 0.4718517f},
                        {0, 5, 46789.5977f, -1318.80005f, 1.2062058f},
                        {0, 18, 46789.5977f, -1318.80005f, 0.48248228f},
                        {1, 55, 46804.7969f, -1308.0f, 1.3597587f},
                        {1, 4, 46804.7969f, -1308.0f, 2.0f},
                        {1, 68, 14738.0f, 18596.7988f, 1.1500999f},
                        {1, 10, 14738.0f, 18596.7988f, 0.46003994f},
                        {0, 18, 14776.7998f, 18592.0f, 0.44114113f},
                        {1, 31, 14776.7998f, 18607.1992f, 1.0172175f},
                        {1, 5, 14776.7998f, 18607.1992f, 2.0f},
                        {0, 12, 14798.0f, 18607.1992f, 1.2150646f},
                        {0, 1, 43518.0f, -49568.0f, 1.093994f},
                        {0, 18, 43518.0f, -49568.0f, 0.43759757f},
                        {1, 2, 43548.3984f, -49534.0f, 1.4660645f},
                        {1, 5, 43548.3984f, -49534.0f, 2.0f},
                        {1, 62, 43561.1992f, -49475.2031f, 1.2150646f},
                        {1, 10, 43561.1992f, -49475.2031f, 0.4860258f},
                        {0, 18, -39542.0f, -28142.0f, 0.5769764f},
                        {1, 1, -39588.0f, -28138.8008f, 0.9168175f},
                        {1, 10, -39588.0f, -28138.8008f, 0.366727f},
                        {1, 51, -39560.0f, -28126.8008f, 1.4188175f},
                        {1, 5, -39560.0f, -28126.8008f, 2.0f},
                        {0, 16, -39550.4023f, -28132.8008f, 0.88728815f}};
                    for (const TV& t : tv) {
                        const std::vector<fuel::Species>& list = t.tree ? veg.trees : veg.bushes;
                        const bool have = t.species >= 1 && t.species <= list.size();
                        const float r = have ? fuel::trunkRadius(veg, list[t.species - 1], t.x, t.z) : -1.0f;
                        const bool ok = have && std::fabs(r - t.emu) <= 2.5e-7f;
                        ++checked, bad += !ok;
                        std::printf("emulated %s trunk %s %2u (%.4f, %.4f) = %.9g, port %.9g %s\n", t.tree ? "0x785200" : "0x788ab0", t.tree ? "tree" : "bush", t.species,
                                    t.x, t.z, t.emu, r, ok ? "match" : "MISMATCH");
                    }
                }
                // (7) Emulated GenWorld_BuildPermutation 0x74e600 (P70b): RT[256] built at G+0x704 from this world's seed under
                // EmuRun (run 0x74e600..0x74e6fb, EDI = G); compared by FNV-1a hash of the 256 bytes (no table in the repo).
                {
                    const fuel::Permutation perm = fuel::buildPermutation(dpc, fuel::genWorldSeed(dpc));
                    uint32_t h = 2166136261u;
                    for (uint8_t c : perm) h = (h ^ c) * 16777619u;
                    const bool ok = h == 0x47f79187u;
                    ++checked, bad += !ok;
                    std::printf("emulated 0x74e600 permutation hash 0x%08x (emulated 0x47f79187) %s\n", h, ok ? "match" : "MISMATCH");
                }
                // (8) GenWorldCore ctor 0x74da50 -> settings 0x74d5b0 (P70b): the decor/ground quadtree split factor G+0xa88 is
                // the float at [0x9dbff0] of the player's FUEL.exe (read at run time) = the port's kChunkSplit.
                {
                    const auto f = fuel::readEngineFloats(dpc, 0x9dbff0, 1);
                    const bool ok = !f.empty() && double(f[0]) == fuel::kChunkSplit;
                    ++checked, bad += !ok;
                    std::printf("0x74d5b0 split G+0xa88 = %g (port %g) %s\n", f.empty() ? -1.0 : double(f[0]), fuel::kChunkSplit, ok ? "match" : "MISMATCH");
                }
                // (9) Shipped-data conditions that keep two FUEL functions from contributing (P70b):
                // - 0x6b51d0 MapSampler_SampleSlot runs only from 0x79b870 when a flat surface's height-map index fs+0xa0
                //   (file record +0x78, u16; loader 0x77d480) is not 0xffff: every USA1 file flat must have 0xffff.
                // - 0x7006f0 (map slot 11 sampler) feeds the rock evaluator's lake term: slot 11 must be 0 at every texel
                //   (the B-spline of an all-zero raster is 0 everywhere).
                {
                    size_t n = 0, other = 0;
                    for (uint32_t k = 0; k < ground.detail->fileFlatCount(); ++k, ++n) {
                        const auto info = ground.detail->fileFlat(k, 0, 0);
                        if (info.raw.size() < 0x7a || (info.raw[0x78] | info.raw[0x79] << 8) != 0xffff) ++other;
                    }
                    const bool okF = n > 0 && other == 0;
                    ++checked, bad += !okF;
                    std::printf("0x6b51d0: file flats with a height map (+0x78 != 0xffff): %zu of %zu %s\n", other, n, okF ? "match (never runs)" : "MISMATCH");
                    const auto& m11 = ground.detail->slot11();
                    size_t nz = 0;
                    for (uint16_t v : m11.raw) nz += v != 0;
                    const size_t nt = m11.raw.size();
                    const bool ok11 = nt > 0 && nz == 0;
                    ++checked, bad += !ok11;
                    std::printf("0x7006f0: map slot 11 non-zero texels %zu of %zu %s\n", nz, nt, ok11 ? "match (no effect)" : "MISMATCH");
                }
                // (10) Emulated Polyline_Offset 0x6f7360 (P70b; EmuRun, d = 3 on a 5-point polyline) vs fuel::offsetPolyline.
                {
                    const std::vector<std::array<double, 2>> poly = {{0, 0}, {10, 0}, {20, 5}, {25, 15}, {25, 30}};
                    const double emu[10] = {0.0, 3.0, 9.272393, 2.9104276, 17.87868, 7.1213207, 22.058258, 15.588348, 22.0, 30.0};
                    const auto o = fuel::offsetPolyline(poly, 3.0);
                    double worst = 0;
                    for (int k = 0; k < 5; ++k) worst = std::max({worst, std::fabs(o[size_t(k)][0] - emu[2 * k]), std::fabs(o[size_t(k)][1] - emu[2 * k + 1])});
                    const bool ok = worst < 1e-4;
                    ++checked, bad += !ok;
                    std::printf("emulated 0x6f7360 Polyline_Offset d=3: worst %.6f %s\n", worst, ok ? "match" : "MISMATCH");
                }
                // (11) Emulated Noise_DecorDensity 0x6b7f10 (P70b): the whole function under EmuRun with the F_dd9c bank set up by
                // Fractal_Init 0x69b1b0 (L5 S20000 seed 44.68557) over this world's random table; its two callees stubbed with
                // the port's values at the point (0x4dde20 humidity, 0x4debf0 clamp01 F_cdc8). Points 62.5 m past a 20 km cell
                // corner so the emulator's round-to-nearest fistp equals the game's round-down.
                {
                    fuel::GroundGenerator gen(dpc, ground, nullptr);
                    const double dv[][3] = {{-39937.5, -19937.5, 0.229577796}, {-19937.5, -39937.5, 0.164893514}, {-59937.5, 20062.5, 0.208735088},
                                            {20062.5, 40062.5, 0.646074593}};
                    for (const auto& q : dv) {
                        const float d = gen.decorDensity(q[0], q[1]);
                        const bool ok = std::fabs(d - q[2]) < 1e-6;
                        ++checked, bad += !ok;
                        std::printf("emulated 0x6b7f10 decor density (%.1f, %.1f) = %.9g, port %.9g %s\n", q[0], q[1], q[2], d, ok ? "match" : "MISMATCH");
                    }
                }
                // (12) Emulated GenWorld map samplers (P80): InitMapSamplers 0x77bb80 (GwMapSampler::Init 0x69b620: format 6,
                // value scale 1 / max for all 20 slots, uniform cubic B-spline basis, scale trunc(span w) / 140000, origin w/2)
                // on this world's map bodies, then the inlined samplers 0x458e60 (map 8), 0x772110 (map 9, Map9_Sample) and
                // 0x458780 (map 14), 0x4dde20 (2), 0x458100 (13), 0x457a80 (15), 0x4573e0 (16) with SampleBicubic 0x69c560 and Fetch4x4_Mode6 0x69bfc0, under EmuRun (commands from
                // --emu-map-cmd) vs GroundGenerator::mapAt. Proves the sampler every map consumer shares (away from the
                // map border, where FUEL mirrors texels and the port clamps: never within the +-65.5 km world).
                {
                    fuel::GroundGenerator gen(dpc, ground, nullptr);
                    const float px[6][2] = {{-39612.8906f, -28079.2969f}, {45778.9062f, -6258.98438f}, {4722.26562f, 35347.2656f},
                                            {-51307.8125f, -30231.25f},   {11809.7656f, -51023.4375f}, {300.78125f, 607.03125f}};
                    const int slots[7] = {8, 9, 14, 2, 13, 15, 16};
                    const double emu[7][6] = {{1.00000012, 0.897882961, 0.176362305, 1.00000012, 1.00000012, 1.00000012},
                                              {0.635746240, 0.467859003, 0.876620169, 0.582726506, 0.577462546, 0.643455258},
                                              {0.0, 0.0, 0.291906952, 0.292316250, 0.391560220, 0.0},
                                              {0.853376052, 0.662336707, 0.0, 1.0, 0.736202446, 1.0},
                                              {0.0, 0.0853316857, 0.0, 0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.222440372},
                                              {0.0, 0.0158704487, 0.0, 0.0, 0.742091173, 0.0}};
                    const char* fn[7] = {"0x458e60", "0x772110", "0x458780", "0x4dde20", "0x458100", "0x457a80", "0x4573e0"};
                    for (int s = 0; s < 7; ++s) {
                        double worst = 0;
                        for (int k = 0; k < 6; ++k) worst = std::max(worst, std::fabs(gen.map(slots[s], px[k][0], px[k][1]) - emu[s][k]));
                        const bool ok = worst < 5e-6;
                        ++checked, bad += !ok;
                        std::printf("emulated %s map %d sampler, 6 points: worst %.2g %s\n", fn[s], slots[s], worst, ok ? "match" : "MISMATCH");
                    }
                }
                {
                    // (13) Vehicles and the AutoMesh / RoadMesh bushes (P84, notes p84-bushes.md). FUEL's Broussaille*
                    // templates (clone.tsc: IT_DECORMESH_G, a thin decor agent: vf00 0x516c80 only sets agent flags
                    // 0x44000 and sleeps it, vf24/25 0x516cb0/0x516db0 register a sound bank, none of them) carry no
                    // collision: no Lod/Mesh boxes or spheres and no collision triangle tree (0x884dc0 / 0x885110 /
                    // 0x884f50, P46), so 0x71dd80 gathers nothing for them and the contact hook 0x51edb0 (breakables,
                    // IT_DECORDYNMESH_G +0x60 0x800000 only) never sees them: vehicles drive through, drawn intersecting.
                    // The RoadMesh rules put Broussaille01Petite in the middle of forest tracks (MiddleDist 2 / 0 / -3).
                    // Checked: every Broussaille template is IT_DECORMESH_G with empty FUEL collision (port: no solid);
                    // control: the IT_DECORDYNMESH_G templates are not all empty (the loader finds collision at all).
                    const std::filesystem::path common = std::filesystem::path(argv[1]).parent_path().parent_path();
                    const fuel::AutoMesh am(common.string(), dpc, {&dpc});
                    const auto classes = fuel::loadCloneClasses((common / "clone.tsc").string());
                    auto classOf = [&](uint32_t crc) {
                        const auto it = classes.find(crc);
                        return it == classes.end() ? fuel::GameClass::None : it->second;
                    };
                    auto solid = [&](uint32_t crc) {
                        for (const fuel::PlacedMesh& m : am.templateParts(crc))
                            if (m.collisionFirst && !fuel::loadStaticCollision({&dpc}, m.objectId).empty()) return true;
                        return false;
                    };
                    int bushes = 0, solidBushes = 0, wrongClass = 0, middle = 0, dyn = 0, dynSolid = 0;
                    std::unordered_map<uint32_t, bool> seen;
                    for (int l = 0; l < 4; ++l)
                        for (const fuel::AutoMeshRule& r : am.rules(l)) {
                            if (l == 3 && r.name.rfind("Broussaille", 0) == 0 && r.middleDist <= 1000.0f) ++middle;
                            if (seen.count(r.crc) || am.templateParts(r.crc).empty()) continue;
                            seen[r.crc] = true;
                            const fuel::GameClass c = classOf(r.crc);
                            if (r.name.rfind("Broussaille", 0) == 0) {
                                ++bushes, wrongClass += c != fuel::GameClass::DecorMesh;
                                const bool s = solid(r.crc);
                                solidBushes += s;
                                std::printf("  %-22s %s, FUEL collision %s", r.name.c_str(), fuel::gameClassName(c), s ? "solid:" : "none");
                                for (const fuel::PlacedMesh& m : am.templateParts(r.crc)) {
                                    if (!m.collisionFirst) continue;
                                    const fuel::StaticCollision sc = fuel::loadStaticCollision({&dpc}, m.objectId);
                                    if (sc.empty()) continue;
                                    std::printf(" [%08x: %zu boxes, %zu spheres, %zu triangles, r %.2f m]", m.objectId, sc.boxes.size(), sc.spheres.size(),
                                                sc.tris.size(), sc.radius);
                                }
                                std::printf("\n");
                            } else if (c == fuel::GameClass::DecorDynMesh) {
                                ++dyn, dynSolid += solid(r.crc);
                            }
                        }
                    const bool ok = bushes >= 6 && solidBushes == 4 && wrongClass == 0 && middle == 3 && dynSolid > 0;
                    ++checked, bad += !ok;
                    std::printf("AutoMesh bushes: %d Broussaille templates, %d with collision, %d not IT_DECORMESH_G, %d road-middle rules; "
                                "control: %d of %d IT_DECORDYNMESH_G templates solid %s\n",
                                bushes, solidBushes, wrongClass, middle, dynSolid, dyn, ok ? "match" : "MISMATCH");
                }
                {
                    // (PK05) Decor_FreeOfObjects 0x557650 = !0x6404a0 && 0x4f8a80 && 0x4ed9e0 (notes/pk05.md). (a) the logic at
                    // hand-computed points; (b) trees / bushes standing inside start, checkpoint and finish boxes with and
                    // without the clearance (CR2-Yell-CP and every mission of the level).
                    const std::string data = std::filesystem::path(argv[1]).parent_path().parent_path().parent_path().string();
                    fuel::PlantClearance C;
                    C.gates.push_back({1000, 2000, 30, 20});  // a start box of half size 30 ((AI 4 + 1) / 2 + 1 = 3, x 10)
                    C.gates.push_back({3000, 2000, 25, 20});  // a checkpoint on a 20 m road
                    C.gates.push_back({5000, 2000, 25, 7.5f});  // a checkpoint on a 7.5 m road
                    C.pois.push_back({-1000, 0});
                    fuel::PlantClearance::Box bx;  // 4 x 2 x 6 box turned 90 degrees about y at (-2000, 10, -2000), grid radius 8
                    bx.m = {0, 0, -1, -2000, 0, 1, 0, 10, 1, 0, 0, -2000};
                    bx.ext = {2, 1, 3};
                    bx.radius = 8;
                    C.boxes.push_back(bx);
                    C.index();
                    const float r = fuel::kPlantSphereRadius;
                    struct Case {
                        const char* what;
                        bool free;
                        bool want;
                    };
                    const Case cases[] = {
                        {"start box centre", C.free(1000, 0, 2000, r), false},
                        {"start: 34 m out on an axis (grid box 30 + r 5 = 35, circle 55)", C.free(1034, 0, 2000, r), false},
                        {"start: 36 m out (outside the grid box)", C.free(1036, 0, 2000, r), true},
                        {"start: box corner (34, 34), distance 48 < 55", C.free(1034, 0, 2034, r), false},
                        {"checkpoint, 20 m road: 29 m out", C.free(3029, 0, 2000, r), false},
                        {"checkpoint, 20 m road: 31 m out (outside the box)", C.free(3031, 0, 2000, r), true},
                        {"checkpoint, 7.5 m road: corner (29, 29), distance 41 > 37.5", C.free(5029, 0, 2029, r), true},
                        {"checkpoint, 7.5 m road: 29 m out on an axis", C.free(5029, 0, 2000, r), false},
                        {"POI: 29.9 m (< r + 25)", C.free(-1000 + 29.9f, 0, 0, r), false},
                        {"POI: 30.1 m", C.free(-1000 + 30.1f, 0, 0, r), true},
                        {"GenWorld box: centre", C.free(-2000, 10, -2000, r), false},
                        {"GenWorld box: 4.9 m off the 3 m half-length face", C.free(-2000 + 3 + 4.9f, 10, -2000, r), false},
                        {"GenWorld box: 5.1 m off the 3 m half-length face", C.free(-2000 + 3 + 5.1f, 10, -2000, r), true},
                        {"GenWorld box: 4.9 m off the 2 m half-width face", C.free(-2000, 10, -2000 + 2 + 4.9f, r), false},
                        {"GenWorld box: 4.9 m above the 1 m half-height", C.free(-2000, 10 + 1 + 4.9f, -2000, r), false},
                        {"GenWorld box: 6 m above the 1 m half-height", C.free(-2000, 10 + 1 + 6.0f, -2000, r), true},
                    };
                    for (const Case& c : cases) {
                        const bool ok = c.free == c.want;
                        ++checked, bad += !ok;
                        if (!ok) std::printf("Decor_FreeOfObjects %s: %s, expected %s: MISMATCH\n", c.what, c.free ? "free" : "blocked", c.want ? "free" : "blocked");
                    }
                    std::printf("Decor_FreeOfObjects: %zu logic cases checked\n", sizeof cases / sizeof cases[0]);
                    try {
                        fuel::Vegetation veg = fuel::loadVegetation(dpc);
                        const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                        const auto roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(dpc));
                        std::unique_ptr<fuel::RoadGraph> graph;
                        try { graph = std::make_unique<fuel::RoadGraph>(dpc); } catch (const std::exception&) {}
                        const auto types = fuel::loadRoadTypes(dpc);
                        const size_t gates = fuel::loadPlantClearance(veg, dpc, data, graph.get(), types ? types->data() : nullptr, types ? types->size() : 0);
                        std::printf("Decor_FreeOfObjects: %zu GenWorld boxes, %zu mission gates, %zu POIs\n", veg.clearance.boxes.size(), gates, veg.clearance.pois.size());
                        fuel::Vegetation base = veg;  // before: no gates, no POIs, no GenWorld boxes
                        base.clearance.gates.clear(), base.clearance.pois.clear(), base.clearance.boxes.clear(), base.clearance.index();
                        // Count plants inside each gate's own box (|dx|, |dz| <= half) and inside the circle road + half.
                        auto count = [&](const fuel::Vegetation& V, float x, float z, float half, float road, bool circle) {
                            std::vector<fuel::Plant> ps;
                            fuel::scatterPlants(V, lv, roads.get(), nullptr, x, z, (half + std::min(road, 40.0f) + 6) * 1.5f, ps, false, 0, nullptr);
                            size_t n = 0;
                            for (const fuel::Plant& p : ps)
                                n += (circle || (std::fabs(p.position[0] - x) <= half && std::fabs(p.position[2] - z) <= half)) &&
                                     std::hypot(p.position[0] - x, p.position[2] - z) < half + road;  // the box corners beyond road + half stay open
                            return n;
                        };
                        size_t startBefore = 0, startAfter = 0, allBefore = 0, allAfter = 0, startsChecked = 0;
                        for (const fuel::PlantClearance::Gate& g : veg.clearance.gates) {
                            if (g.x == 0 && g.z == 0) continue;
                            const size_t b = count(base, g.x, g.z, g.box, g.road, false), a2 = count(veg, g.x, g.z, g.box, g.road, false);
                            allBefore += b, allAfter += a2;
                            if (g.box > 25) startBefore += b, startAfter += a2, ++startsChecked;
                        }
                        std::printf("Decor_FreeOfObjects: plants inside every gate box and its road + half circle (start / finish +-((AI + 1) / 2 + 1) x 10 m, checkpoints +-25 m): %zu before, %zu after (%zu "
                                    "start / finish boxes: %zu before, %zu after)\n",
                                    allBefore, allAfter, startsChecked, startBefore, startAfter);
                        for (const fuel::Event& e : fuel::loadEvents(data)) {
                            if (e.id != "CR2-Yell-CP") continue;
                            const float box = float((e.race.aiCount + 1) / 2 + 1) * 10.0f;
                            const auto& st = e.race.start;
                            const size_t b = count(base, st[0], st[2], box, 1e9f, false), a2 = count(veg, st[0], st[2], box, 1e9f, false);
                            std::printf("Decor_FreeOfObjects: CR2-Yell-CP start (%.1f, %.1f) AI %d box +-%.0f m: %zu plants before, %zu after\n", st[0], st[2], e.race.aiCount, box, b, a2);
                            const bool ok = a2 == 0 && b > 0;
                            ++checked, bad += !ok;
                            if (!ok) std::printf("  CR2-Yell-CP start box: MISMATCH (expected plants before and none after)\n");
                            size_t cb = 0, ca = 0;
                            for (const auto& cp : e.race.checkpoints) cb += count(base, cp[0], cp[2], 25, 1e9f, false), ca += count(veg, cp[0], cp[2], 25, 1e9f, false);
                            std::printf("Decor_FreeOfObjects: CR2-Yell-CP %zu checkpoints +-25 m: %zu plants before, %zu after\n", e.race.checkpoints.size(), cb, ca);
                            const bool ok2 = ca == 0;
                            ++checked, bad += !ok2;
                        }
                        const bool okAll = allAfter == 0 && allBefore > 0;
                        ++checked, bad += !okAll;
                        if (!okAll) std::printf("  gate boxes: MISMATCH (expected plants before and none after)\n");
                    } catch (const std::exception& e) {
                        std::printf("Decor_FreeOfObjects: %s\n", e.what());
                    }
                }
                {
                    // (PK32) Ground occlusion curvature term 0x7f90d0 (notes pk32.md): the whole function under EmuRun
                    // (args h, h(x+s), h(x-s), h(z+s), h(z-s), s pushed as the asm does; result ST0) vs groundCurvatureFactor.
                    struct Cv {
                        float a[6];
                        double emu;
                    };
                    const Cv cv[] = {
                        {{100, 100, 100, 100, 100, 1}, 1.00000000},
                        {{100, 100.05f, 100.05f, 100.05f, 100.05f, 1}, 0.721743676},
                        {{100, 99.97f, 99.95f, 99.98f, 99.96f, 1}, 1.19921030},
                        {{250.3f, 251.1f, 249.2f, 250.9f, 249.8f, 2}, 1.12724301},
                        {{12.5f, 13.4f, 12.9f, 12.1f, 13.6f, 4}, 0.388846158},
                        {{-3.25f, -2.0f, -2.5f, -1.75f, -3.0f, 1}, -0.496337805},
                        {{840.125f, 840.5f, 840.0f, 841.25f, 839.0f, 1}, 0.768922947},
                        {{500, 501, 499, 500.5f, 499.5f, 0.5f}, 1.00000000},
                    };
                    double worst = 0;
                    for (const Cv& c : cv)
                        worst = std::max(worst, std::fabs(double(fuel::groundCurvatureFactor(c.a[0], c.a[1], c.a[2], c.a[3], c.a[4], c.a[5])) - c.emu));
                    const bool ok = worst < 2e-6;
                    ++checked, bad += !ok;
                    std::printf("emulated 0x7f90d0 ground curvature factor, %zu points: worst %.2g %s\n", sizeof cv / sizeof cv[0], worst, ok ? "match" : "MISMATCH");
                    // The point byte +0x89 after the term (asm 0x7f9850..0x7f98b3, RC_DOWN fistp): hand-computed.
                    const bool okB = fuel::curvedOcclusion(1.0f, 0.721743676f) == 46.0f / 64 && fuel::curvedOcclusion(0.8f, 1.2f) == 61.0f / 64 &&
                                     fuel::curvedOcclusion(0.9f, 1.5f) == 1.0f && fuel::curvedOcclusion(0.5f, -0.4963f) == 0.0f;
                    ++checked, bad += !okB;
                    std::printf("0x7f92c0 occlusion byte x curvature (4 cases) %s\n", okB ? "match" : "MISMATCH");
                    // On USA1: the factor over a 1 m lattice (tile level 0 spacing) at an arbitrary spot (information only).
                    try {
                        const fuel::Heightfield lv = fuel::loadLevelledHeightfield(dpc);
                        if (lv.detail) {
                            const auto f = fuel::groundCurvatureGrid(*lv.detail, -2000.5, 1000.5, 1.0, 64);
                            double lo = 1e9, hi = -1e9, mean = 0;
                            size_t dark = 0;
                            for (float v : f) lo = std::min(lo, double(v)), hi = std::max(hi, double(v)), mean += v, dark += v < 0.95f;
                            std::printf("0x7f90d0 on USA1, 64 x 64 points at 1 m from (-2000.5, 1000.5): f %.3f..%.3f, mean %.3f, %zu darkened by > 5%%\n", lo, hi,
                                        mean / double(f.size()), dark);
                        }
                    } catch (const std::exception& e) {
                        std::printf("0x7f90d0 on USA1: %s\n", e.what());
                    }
                }
                {
                    // (PK32) RoadMesh ShortCut search 0x78e030 under EmuRun (EDI = a 50-point hairpin: up x = 0 to z = 400, a
                    // 60 m half circle, down x = 120; EAX = the sample, ECX = start, args out, count, step) vs roadShortCut.
                    std::vector<std::array<double, 2>> P;
                    for (int z = 0; z <= 400; z += 20) P.push_back({0.0, double(z)});
                    for (int k = 1; k < 9; ++k) {
                        const double a = 3.141592653589793 * (1 - k / 9.0);
                        P.push_back({std::round((60 + 60 * std::cos(a)) * 1e4) / 1e4, std::round((400 + 60 * std::sin(a)) * 1e4) / 1e4});
                    }
                    for (int z = 400; z >= 0; z -= 20) P.push_back({120.0, double(z)});
                    for (auto& q : P) q = {double(float(q[0])), double(float(q[1]))};
                    struct Sc {
                        double x, z;
                        int start, step;
                        bool found;
                        double qx, qz;
                    };
                    const Sc sc[] = {
                        {0, 205, 10, -1, false, 0, 0}, {0, 205, 11, 1, true, 120, 200},  {0, 345, 17, -1, false, 0, 0}, {0, 345, 18, 1, true, 120, 340},
                        {120, 300, 34, -1, true, 0, 300}, {120, 300, 35, 1, false, 0, 0}, {500, 500, 5, -1, false, 0, 0}, {500, 500, 6, 1, false, 0, 0},
                    };
                    int okN = 0;
                    for (const Sc& c : sc) {
                        double qx = 0, qz = 0;
                        const bool f = fuel::roadShortCut(P, c.start, int(P.size()), c.step, c.x, c.z, qx, qz);
                        const bool ok = f == c.found && (!f || (qx == c.qx && qz == c.qz));
                        okN += ok;
                        if (!ok) std::printf("0x78e030 (%.0f, %.0f) start %d step %d: port %d (%.1f, %.1f), emulated %d (%.1f, %.1f) MISMATCH\n", c.x, c.z, c.start, c.step, f, qx, qz,
                                             c.found, c.qx, c.qz);
                    }
                    const bool ok = okN == int(sizeof sc / sizeof sc[0]);
                    ++checked, bad += !ok;
                    std::printf("emulated 0x78e030 RoadMesh ShortCut search: %d of %zu calls %s\n", okN, sizeof sc / sizeof sc[0], ok ? "match" : "MISMATCH");
                }
                {
                    // (PK32) ProGroundMgr_G (0x4fa5a0 Fill, 0x4fa680 Update, 0x4f9ec0 / 0x4f9e50 flat spots) is dead in the shipped
                    // data: its prefabs PROCEDURAL_1..3 (CRC as 0x4fa5a0 computes it) are named by no DPC (name ids are stored raw).
                    const std::string data = std::filesystem::path(argv[1]).parent_path().parent_path().parent_path().string();
                    const uint32_t crc[3] = {fuel::asoboCrc("PROCEDURAL_1"), fuel::asoboCrc("PROCEDURAL_2"), fuel::asoboCrc("PROCEDURAL_3")};
                    size_t files = 0, hits = 0;
                    for (const auto& e : std::filesystem::recursive_directory_iterator(data)) {
                        std::string ext = e.path().extension().string();
                        for (char& ch : ext) ch = char(std::toupper(static_cast<unsigned char>(ch)));
                        if (!e.is_regular_file() || ext != ".DPC") continue;
                        ++files;
                        std::ifstream in(e.path(), std::ios::binary);
                        const std::string b((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
                        for (uint32_t c : crc) {
                            const char pat[4] = {char(c & 0xff), char(c >> 8 & 0xff), char(c >> 16 & 0xff), char(c >> 24)};
                            for (size_t k = b.find(pat, 0, 4); k != std::string::npos; k = b.find(pat, k + 1, 4)) ++hits;
                        }
                    }
                    const bool ok = crc[0] == 0x43d0f0d3u && crc[1] == 0x4e93d60au && crc[2] == 0x4a52cbbdu && files > 0 && hits == 0;
                    ++checked, bad += !ok;
                    std::printf("ProGround: PROCEDURAL_1..3 (%08x %08x %08x) named in %zu of %zu DPCs %s (pool always empty: dead)\n", crc[0], crc[1], crc[2], hits, files,
                                ok ? "match" : "MISMATCH");
                }
                std::printf("terrain vectors: %d of %d match\n", checked - bad, checked);
                if (bad) return 1;
            } else if (a == "--road-types") {  // the AGW_TerraRoad table replayed from FUEL.exe
                std::string err;
                const auto t = fuel::loadRoadTypes(dpc, &err);
                if (!t) {
                    std::printf("road types: %s\n", err.c_str());
                    return 1;
                }
                for (size_t k = 0; k < t->size(); ++k) {
                    const fuel::RoadTypeParams& q = (*t)[k];
                    std::printf("type %2zu: T0 %g T24 %g T28 %g T34 %g T4c %g T50 %g T54 %g T5c %g T74 %g T78 %g T7c %g T80 %g Tbc %g Tc4 %g Tc8 %g Tcc %g M %g Mmin %g Tb4 %g\n", k,
                                q.reach, q.crown, q.verge, q.noise, q.fade, q.ditch, q.ditchDepth, q.stretch, q.bumpAmp, q.bumpLen, q.waveAmp,
                                q.waveLen, q.band, q.core, q.cc8, q.flat, q.smooth, q.smoothMin, q.aiSpeedBlend);
                }
            } else if (a == "--events") {
                // Race starts and checkpoints (authored on FUEL's ground) against the ground.
                const std::string data = std::filesystem::path(argv[1]).parent_path().parent_path().parent_path().string();
                const fuel::Heightfield ground = fuel::loadLevelledHeightfield(dpc);
                std::vector<float> dn, dr;
                for (const fuel::Event& e : fuel::loadEvents(data)) {
                    std::vector<std::array<float, 3>> pts = e.race.checkpoints;
                    pts.push_back(e.race.start);
                    for (const auto& p : pts) {
                        if (p[0] == 0 && p[2] == 0) continue;
                        dn.push_back(p[1] - ground.smoothHeightAt(p[0], p[2]));
                        dr.push_back(p[1] - hf.baseHeightAt(p[0], p[2]));
                    }
                    if (e.id == "CR2-Yell-CP") {
                        const auto& st = e.race.start;
                        const fuel::GroundSample g = ground.detail->sample(st[0], st[2]);
                        std::printf("CR2-Yell-CP start (%.1f, %.1f, %.1f): map %.2f, ground %.2f (h2 %.2f rock %.2f floor %.2f w18 %.2f)\n", st[0],
                                    st[1], st[2], g.h0, g.ground, g.h2, g.rock, g.floor, g.w18);
                    }
                }
                auto stats = [](const char* name, std::vector<float> v) {
                    std::sort(v.begin(), v.end());
                    size_t in1 = 0;
                    for (float d : v) in1 += std::fabs(d) < 1.0f;
                    if (!v.empty())
                        std::printf("events %s: %zu points, p10 %.2f p50 %.2f p90 %.2f m, within 1 m %zu\n", name, v.size(), v[v.size() / 10],
                                    v[v.size() / 2], v[v.size() * 9 / 10], in1);
                };
                stats("map", dr);
                stats("ground", dn);
            } else if (a == "--grass" && i + 3 < argc) {
                // --grass X Z R: the decoded ground generator (grass, rock, fields) around (X, Z),
                // with timing and statistics; also a map of the area as grass.ppm (R/G/B = grass/rock/field).
                const float x = std::strtof(argv[i + 1], nullptr), z = std::strtof(argv[i + 2], nullptr);
                const int r = std::atoi(argv[i + 3]);
                i += 3;
                const fuel::Heightfield levelled = fuel::loadLevelledHeightfield(dpc);
                std::unique_ptr<fuel::RoadNetwork> roads;
                try {
                    roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(dpc));
                } catch (const std::exception&) {
                }
                const fuel::GroundGenerator gen(dpc, levelled, roads.get());
                if (std::getenv("FUEL_FIELDSCAN")) {  // the whole world, one point per 100 m
                    long counts[3] = {}, popOk = 0, flatLow = 0;
                    for (double pz = -69950; pz < 70000; pz += 100)
                        for (double px = -69950; px < 70000; px += 100) {
                            const fuel::GroundPoint p = gen.at(px, pz);
                            counts[std::clamp(p.field, 0, 2)]++;
                            flatLow += p.normal[1] > 0.8707f && p.height < 1500;
                        }
                    std::printf("fieldscan: none %ld ploughed %ld crops %ld; flat and low %ld\n", counts[0], counts[1], counts[2], flatLow);
                    (void)popOk;
                }
                if (std::getenv("FUEL_GPOINT")) std::printf("mean T0 over 1 km: %.3f\n", gen.meanT0(x, z, 1000, 40));
                if (std::getenv("FUEL_GPOINT")) {  // a line of points eastwards, one per 5 m
                    for (int k = 0; k < r; k += 5) {
                        const fuel::GroundPoint p = gen.at(x + k, z);
                        std::printf("  %+5d: y %7.1f n.y %.3f hum %.2f/%.2f road %.2f cover %.2f high %.2f rock %.2f field %d grass %.2f/%.2f size %.2f\n", k,
                                    p.height, p.normal[1], p.humidity, p.surfaceHumidity, p.road, p.cover, p.highGround, p.rock, p.field, p.grass,
                                    p.groundGrass, p.grassSize);
                        const fuel::GroundColours c = gen.colours(x + k, z, p);
                        std::printf("         colour %.3f %.3f %.3f far %.3f %.3f %.3f road %.3f %.3f %.3f occ %.2f forest %.2f rockSlope %.2f tarmac %.0f\n",
                                    c.base[0], c.base[1], c.base[2], c.far[0], c.far[1], c.far[2], c.road[0], c.road[1], c.road[2],
                                    c.occlusion, c.forest, p.rockSlope, p.tarmac);
                    }
                }
                const auto g0 = std::chrono::steady_clock::now();
                std::vector<fuel::GrassTuft> tufts;
                gen.grass(int32_t(x) - r, int32_t(z) - r, int32_t(x) + r, int32_t(z) + r, tufts);
                const double gs = std::chrono::duration<double>(std::chrono::steady_clock::now() - g0).count();
                std::printf("grass: %zu tufts over %d x %d m in %.1f ms (%.2f us per point)\n", tufts.size(), 2 * r, 2 * r, gs * 1e3,
                            gs * 1e6 / (4.0 * r * r));
                double sumSize = 0;
                int tiles[16] = {};
                for (const auto& t : tufts) sumSize += t.size, tiles[t.tile & 15]++;
                if (!tufts.empty()) std::printf("  mean size %.2f m; tiles", sumSize / double(tufts.size()));
                for (int k = 0; k < 9; ++k) std::printf(" %d", tiles[k]);
                std::printf("\n");
                // Coarse map: one point per 4 m.
                const int n = r / 2;
                std::ofstream out("grass.ppm", std::ios::binary);
                out << "P6\n" << n << " " << n << "\n255\n";
                double sum[6] = {};
                int fields[3] = {}, excluded = 0;
                for (int j = 0; j < n; ++j)
                    for (int k = 0; k < n; ++k) {
                        const double px = x - r + k * 4.0, pz = z - r + j * 4.0;
                        const fuel::GroundPoint p = gen.at(px, pz);
                        sum[0] += p.groundGrass, sum[1] += p.rock, sum[2] += p.road, sum[3] += p.surfaceHumidity, sum[4] += p.grassSize,
                            sum[5] += p.cover;
                        fields[std::clamp(p.field, 0, 2)]++;
                        excluded += gen.excludesPlants(px, pz);
                        const unsigned char px3[3] = {(unsigned char)(p.groundGrass * 255), (unsigned char)(p.rock * 255),
                                                      (unsigned char)(p.field * 120)};
                        out.write(reinterpret_cast<const char*>(px3), 3);
                    }
                const double cnt = double(n) * n;
                std::printf("  means: groundGrass %.3f rock %.3f road %.3f surfHumidity %.3f size %.3f cover %.3f\n", sum[0] / cnt,
                            sum[1] / cnt, sum[2] / cnt, sum[3] / cnt, sum[4] / cnt, sum[5] / cnt);
                std::printf("  fields: none %d ploughed %d crops %d; in no-plant flat surfaces: %d (of %d)\n", fields[0], fields[1], fields[2],
                            excluded, n * n);
                try {  // trees and bushes with and without the ground tests
                    const fuel::Vegetation veg = fuel::loadVegetation(dpc);
                    std::vector<fuel::Plant> before, after;
                    fuel::scatterPlants(veg, levelled, roads.get(), nullptr, x, z, float(r), before);
                    fuel::scatterPlants(veg, levelled, roads.get(), nullptr, x, z, float(r), after, false, 0, &gen);
                    std::printf("  plants: %zu without the ground tests, %zu with\n", before.size(), after.size());
                } catch (const std::exception& e) {
                    std::printf("  plants: %s\n", e.what());
                }
            } else if (a == "--lakes") {
                // --lakes: the 19 lakes the engine adds from code (read from FUEL.exe), flood-filled.
                const auto l0 = std::chrono::steady_clock::now();
                const fuel::Lakes lakes(dpc, [&](double px, double pz) { return double(hf.smoothHeightAt(px, pz)); });
                std::printf("lakes: %zu (%.2f s)\n", lakes.lakes().size(), std::chrono::duration<double>(std::chrono::steady_clock::now() - l0).count());
                for (const fuel::Lake& L : lakes.lakes()) {
                    size_t cells = 0;
                    for (uint32_t w : L.mask) cells += size_t(__builtin_popcount(w));
                    // Area under water: 25 m samples inside the mask with B-spline ground below level - 2.
                    size_t wet = 0;
                    for (double z = L.minZ; z < L.maxZ; z += 25)
                        for (double x = L.minX; x < L.maxX; x += 25)
                            if (L.contains(x, z) && hf.smoothHeightAt(x, z) < L.level - 2) ++wet;
                    std::printf("  %s x %.2f z %.2f level %.2f R %.0f: %d x %d cells of %.2f m, %zu set, %.1f km2 under water\n",
                                L.flood ? "basin" : "pond ", L.x, L.z, L.level, L.radius, L.w, L.h, L.cell, cells, double(wet) * 625e-6);
                }
            } else if (a == "--decal-check") {
                // Road decal uv (0x6fbab0 ctx+0x1f8 / 0x1fc) between neighbouring ground vertices that both carry the
                // decal (weight byte > 0.01, same material) on the chunk lattices 1, 2, 4, 8 m. Pairs differing by more
                // than 0.5 in u or v lose the decal by the seam rule (0x7f9b50); the rest are interpolated across a
                // triangle, so a step there shows as a squeezed band of texture. Rates are per metre.
                std::vector<std::array<double, 2>> spots;
                while (i + 2 < argc && argv[i + 1][0] != '-' ) {
                    spots.push_back({std::strtod(argv[i + 1], nullptr), std::strtod(argv[i + 2], nullptr)});
                    i += 2;
                }
                if (spots.empty()) spots = {{-39570, -28136}, {-39570, -28068}, {-39743.5, -28195.5}, {-39465, -27606}};
                const fuel::Heightfield levelled = fuel::loadLevelledHeightfield(dpc);
                if (!levelled.roads) {
                    std::printf("decal check: FAIL no road sampler\n");
                    return 1;
                }
                const fuel::RoadSampler& rs = *levelled.roads;
                bool ok = true;
                for (const auto& sp : spots) {
                    for (const double L : {1.0, 2.0, 4.0, 8.0}) {
                        const int R = 64, n = int(2 * R / L) + 1;
                        const double x0 = std::round((sp[0] - R) / L) * L, z0 = std::round((sp[1] - R) / L) * L;
                        std::vector<fuel::RoadFields> F(size_t(n) * size_t(n));
                        for (int j = 0; j < n; ++j)
                            for (int k = 0; k < n; ++k) F[size_t(j) * size_t(n) + size_t(k)] = rs.sample(x0 + k * L, z0 + j * L);
                        size_t pairs = 0, seams = 0, steps = 0;
                        double maxU = 0, maxV = 0;
                        std::vector<double> rates;
                        for (int j = 0; j < n; ++j)
                            for (int k = 0; k < n; ++k) {
                                const fuel::RoadFields& A = F[size_t(j) * size_t(n) + size_t(k)];
                                if (!(std::floor(A.decal * 64 + 0.5) / 64 > 0.01) || A.material == 0xfff) continue;
                                for (int d = 0; d < 2; ++d) {
                                    const int kk = k + (d == 0), jj = j + (d == 1);
                                    if (kk >= n || jj >= n) continue;
                                    const fuel::RoadFields& B = F[size_t(jj) * size_t(n) + size_t(kk)];
                                    if (!(std::floor(B.decal * 64 + 0.5) / 64 > 0.01) || B.material != A.material) continue;
                                    const double du = std::fabs(A.across - B.across), dv = std::fabs(A.along - B.along);
                                    if (du > 0.5 || dv > 0.5) {
                                        ++seams;
                                        continue;
                                    }
                                    ++pairs;
                                    maxU = std::max(maxU, du / L), maxV = std::max(maxV, dv / L);
                                    rates.push_back(std::max(du, dv) / L);
                                }
                            }
                        double med = 0;
                        if (!rates.empty()) {
                            std::nth_element(rates.begin(), rates.begin() + long(rates.size() / 2), rates.end());
                            med = rates[rates.size() / 2];
                            for (double r : rates) steps += r > 2 * med + 0.02;
                        }
                        // The uv of one road changes by at most 0.5 / (T[cc] - 0.5) (~0.05-0.15) per metre; a pair
                        // above 0.5 per metre would be a broken sample (the old blocks came from the renderer).
                        const bool good = maxU <= 0.5 && maxV <= 0.5;
                        ok = ok && good;
                        std::printf("  (%.1f, %.1f) %g m lattice: %zu owned pairs, %zu seam-rule cuts, median %.3f /m, max |du| %.3f /m "
                                    "|dv| %.3f /m, %zu joint steps (> 2x median)%s\n",
                                    sp[0], sp[1], L, pairs, seams, med, maxU, maxV, steps, good ? "" : "  FAIL");
                    }
                }
                std::printf("decal check: %s\n", ok ? "ok" : "FAIL");
                if (!ok) return 1;
            } else if (a == "--roads" && i + 3 < argc) {
                // --roads X Z R: FUEL's road segments, point sampler and height carve around (X, Z)
                // (AGW_TerraRoad, notes p22). Writes roads.ppm (R weight, G decal, B carve depth).
                const double x = std::strtod(argv[i + 1], nullptr), z = std::strtod(argv[i + 2], nullptr);
                const int r = std::atoi(argv[i + 3]);
                i += 3;
                const auto r0 = std::chrono::steady_clock::now();
                const fuel::Heightfield levelled = fuel::loadLevelledHeightfield(dpc);
                fuel::RoadTypeTable types = fuel::loadRoadTypeTable(dpc);
                std::printf("road types: %s\n", types.valid ? "rebuilt from FUEL.exe" : "NOT AVAILABLE");
                if (!types.valid) return 1;
                for (int t = 4; t <= 20; ++t) {
                    const fuel::RoadType& T = types.types[size_t(t)];
                    std::printf("  %2d %-24s C %5.2f half %5.2f bc %6.2f cc %6.2f d4 %6.2f decal 0x%03x tarmac %.0f\n", t, T.name.c_str(), T.at(0x1f0),
                                T.at(0x00), T.at(0xbc), T.at(0xcc), T.at(0xd4), T.u32(0xa8), T.at(0x38));
                }
                fuel::RoadSampler rs(dpc, std::move(types), [&](double px, double pz) {  // flags 3: ground without roads
                    return levelled.detail ? levelled.detail->detailHeight(px, pz) : double(levelled.smoothHeightAt(px, pz));
                });
                std::printf("  %zu records (%.2f s)\n", rs.recordCount(), std::chrono::duration<double>(std::chrono::steady_clock::now() - r0).count());
                const auto s0 = std::chrono::steady_clock::now();
                const int n = 2 * r + 1;
                std::vector<uint8_t> img(size_t(n) * size_t(n) * 3);
                double maxCarve = 0, sumCarve = 0;
                size_t onRoad = 0, decal = 0;
                for (int jz = 0; jz < n; ++jz)
                    for (int jx = 0; jx < n; ++jx) {
                        const double px = x - r + jx, pz = z - r + jz;
                        const fuel::RoadFields f = rs.sample(px, pz);
                        double y = levelled.smoothHeightAt(px, pz);
                        const double y0 = y;
                        rs.carve(px, pz, y, 1.0, 1.0);
                        const double dy = y - y0;
                        maxCarve = std::max(maxCarve, std::fabs(dy));
                        sumCarve += std::fabs(dy);
                        onRoad += f.weight > 0.5f;
                        decal += f.decal > 0.01f;
                        uint8_t* o = &img[(size_t(jz) * size_t(n) + size_t(jx)) * 3];
                        o[0] = uint8_t(std::lround(f.weight * 255));
                        o[1] = uint8_t(std::lround(f.decal * 255));
                        o[2] = uint8_t(std::clamp(128.0 + dy * 32.0, 0.0, 255.0));
                    }
                const double ss = std::chrono::duration<double>(std::chrono::steady_clock::now() - s0).count();
                std::printf("  %d x %d points in %.2f s (%.1f us each incl. building %zu segments): road %zu, decal %zu, |carve| max %.2f m mean %.3f m\n", n,
                            n, ss, ss * 1e6 / double(n * n), rs.builtSegments(), onRoad, decal, maxCarve, sumCarve / double(n * n));
                std::ofstream out("roads.ppm", std::ios::binary);
                out << "P6\n" << n << " " << n << "\n255\n";
                out.write(reinterpret_cast<const char*>(img.data()), std::streamsize(img.size()));
                // Cross-check against the terrain's own decode of 0x6fcaa0 (p21 H6): the ground with
                // P21's road carve vs with this sampler's carve through the RoadCarver hook.
                if (levelled.detail) {
                    std::vector<double> a;
                    for (int k = 0; k <= 400; ++k) a.push_back(levelled.detail->height(x - r + k * (2.0 * r / 400), z));
                    auto* d = const_cast<fuel::TerrainDetail*>(levelled.detail.get());
                    d->setRoadCarver([&rs](double px, double pz, double& y, double c, double w) { return rs.carve(px, pz, y, c, w); });
                    double maxd = 0, sumd = 0;
                    for (int k = 0; k <= 400; ++k) {
                        const double b = levelled.detail->height(x - r + k * (2.0 * r / 400), z), dd = std::fabs(b - a[size_t(k)]);
                        maxd = std::max(maxd, dd), sumd += dd;
                    }
                    d->setRoadCarver(nullptr);
                    std::printf("  carve vs terrain_height's own road decode, 401 points along x: max |diff| %.4f m, mean %.5f m\n", maxd, sumd / 401);
                }
                if (std::getenv("FUEL_ROAD_DEBUG")) {
                    std::vector<const fuel::RoadSampler::Segment*> segs;
                    rs.segmentsNear(x, z, segs);
                    for (const auto* sg : segs) {
                        const float t = (sg->tx * float(x) + sg->tz * float(z)) - sg->c34;
                        std::printf("  seg rec %u type %u len %.1f t %.2f H(t) %.2f base %d wid %u/%u curv %d/%d\n", sg->record, sg->type, sg->len, t,
                                    sg->heightAt(t, rs.types().types[sg->type]), sg->base, sg->wid0, sg->wid1, sg->curv0, sg->curv1);
                    }
                    if (levelled.detail)
                        std::printf("  flags-3 height %.3f\n", levelled.detail->detailHeight(x, z));
                }
                // A transect along x through (X, Z).
                for (int k = -20; k <= 20; k += 2) {
                    const fuel::RoadFields f = rs.sample(x + k, z);
                    double y = levelled.smoothHeightAt(x + k, z);
                    const double y0 = y;
                    const double rr = rs.carve(x + k, z, y, 1.0, 1.0);
                    std::printf("  x%+3d: w %.2f tarmac %.2f rough %.2f decal %.2f mat 0x%03x uv (%.2f, %.2f) h %.2f -> %.2f (r %.2f)\n", k, f.weight, f.tarmac,
                                f.roughness, f.decal, f.material, f.across, f.along, y0, y, rr);
                }
            } else if (a == "--at" && i + 2 < argc) {
                const float x = std::strtof(argv[i + 1], nullptr), z = std::strtof(argv[i + 2], nullptr);
                if (hf.detail) {
                    const fuel::GroundSample g = hf.detail->sample(x, z);
                    std::printf("height at (%.1f, %.1f) = %.3f m (map %.3f, n.y %.3f, detail %.3f, rock %.3f, floor %.3f, w18 %.3f, before finish %.3f)\n",
                                x, z, g.ground, g.h0, g.ny, g.h2 - g.h0, g.rock, g.floor, g.w18, g.detail);
                    std::printf("    after gulch %.3f, flats A %.3f, roads %.3f, flats B %.3f\n", g.gulch, g.flatA, g.road, g.flatB);
                } else {
                    std::printf("height at (%.1f, %.1f) = %.3f m (map B-spline)\n", x, z, hf.baseHeightAt(x, z));
                }
                i += 2;
            } else if (a == "--ground" && i + 2 < argc) {  // --ground FILE.ppm SIZE: the colour map
                const auto g0 = std::chrono::steady_clock::now();
                const fuel::GroundMaps g = fuel::buildGroundMaps(dpc, hf, uint32_t(std::atoi(argv[i + 2])));
                std::printf("ground maps %u x %u in %.2f s\n", g.size, g.size,
                            std::chrono::duration<double>(std::chrono::steady_clock::now() - g0).count());
                std::ofstream out(argv[i + 1], std::ios::binary);
                out << "P6\n" << g.size << " " << g.size << "\n255\n";
                for (size_t k = 0; k < size_t(g.size) * g.size; ++k) out.write(reinterpret_cast<const char*>(&g.colour[k * 4]), 3);
                i += 2;
            } else if (a == "--pgm" && i + 1 < argc) {
                std::ifstream f(argv[++i], std::ios::binary);
                std::string magic;
                unsigned w = 0, h = 0, maxv = 0;
                f >> magic >> w >> h >> maxv;
                f.get();
                if (magic != "P5" || w != hf.width || h != hf.height || maxv != 65535) {
                    std::printf("PGM header does not match\n");
                    return 1;
                }
                size_t mismatches = 0;
                for (size_t k = 0; k < hf.raw.size(); ++k) {
                    unsigned char px[2];
                    f.read(reinterpret_cast<char*>(px), 2);
                    const unsigned ref = (unsigned(px[0]) << 8) | px[1];
                    const unsigned mine = hf.raw[k] <= 4095 ? unsigned(hf.raw[k] * 65535.0 / 4095.0 + 0.5) : 65535u;
                    mismatches += ref != mine;
                }
                std::printf("PGM comparison: %zu of %zu samples differ\n", mismatches, hf.raw.size());
                if (mismatches) return 1;
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelterrain: %s\n", e.what());
        return 1;
    }
    return 0;
}
