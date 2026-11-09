// fuelsurface (P73): checks the Surface_Z Bezier patch port (core/surface_patch.hpp).
//
//   fuelsurface --data DIR       every Surface_Z of WORLD/*.DPC (and MENU.DPC): patches, draw / collision triangles,
//                                materials, front faces, and the placements that use them (world position, node flags)
//   fuelsurface --vectors        a synthetic patch against FUEL.exe outputs recorded under the emulator
//                                (EmuRun.java of 0x846f80 draw N = 3 and 0x7cf0a0 collision 2 x 3); exit 1 on mismatch
//
// No game data is stored here; --vectors uses invented control points.

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/static_collision.hpp"
#include "core/surface_patch.hpp"
#include "core/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

// The synthetic surface of --vectors: one patch, 12 control points, edges with one reversed (flags 0x2c: edges 1
// and 2 reversed, like the shipped data), four corner normals / uvs, one colour.
fuel::SurfaceBody syntheticSurface() {
    fuel::SurfaceBody s;
    s.points = {{0, 0, 0},     {10, 1, 0},   {10, 2, 8},   {0, 0.5f, 8}, {3, 1.5f, -0.5f}, {7, 2.5f, 0.5f},
                {10.5f, 3, 3}, {9.5f, 0, 5}, {7, -1, 8.5f}, {3, 1, 7.5f}, {-0.5f, 2, 5},   {0.5f, -1, 3}};
    // edge = corner a, corner b, handle near a, handle near b
    s.edges = {{0, 1, 4, 5}, {2, 1, 7, 6}, {2, 3, 8, 9}, {3, 0, 10, 11}};
    s.normals = {{0, 1, 0}, {0.6f, 0.8f, 0}, {0, 0.8f, 0.6f}, {-0.36f, 0.48f, -0.8f}};
    s.colours = {{1, 1, 1}};
    s.uvs = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.25f, 0.5f}, {2, 0.125f}};
    fuel::SurfacePatch p;
    p.flags = 0x2c;  // bit 2: edge 1 reversed, bit 3: edge 2 reversed
    p.edges = {0, 1, 2, 3};
    p.colour = {0, 0, 0, 0};
    p.normal = {0, 1, 2, 3};
    p.uv0 = {0, 1, 2, 3};
    p.uv1 = {4, 5, 3, 1};
    s.patches.push_back(p);
    return s;
}

// Recorded with EmuRun.java (notes p73-mesh-material.md, "vectors"): the same surface laid out at the Surface_Z
// offsets (+0x70 points, +0x90 patches, +0x98 edges, +0xa0 normals, +0xa8 colours, +0xb0 uvs), the patch record
// with flags 0x2c and 0x12; 0x846f80 at N = 3 and 8 (PackColorBGRA stubbed), 0x7cf0a0 at 2 x 3 and 4 x 1.
struct DrawVec {
    uint16_t flags;
    int n, idx;
    float x, y, z, u0, v0, u1, v1;
    uint8_t nb, ng, nr;  // bytes +0x10 / +0x11 / +0x12
};
struct GridVec {
    uint16_t flags;
    int nu, nv, idx;
    float x, y, z;
};
#include "fuelsurface_vectors.inc"

int vectors() {
    fuel::SurfaceBody s = syntheticSurface();
    int bad = 0, n = 0;
    // The emulator ignores the game's round-down control word (fstp, fistp round to nearest there): positions within
    // a few float ulps, normal bytes within 1.
    auto close = [](float a, float b) { return std::fabs(a - b) <= 4e-6f * std::max(1.0f, std::fabs(b)); };
    float worst = 0;
    for (const DrawVec& e : kDrawVectors) {
        s.patches[0].flags = e.flags;
        const std::vector<fuel::SurfaceVertex> v = fuel::tessellatePatch(s, s.patches[0], e.n);
        const fuel::SurfaceVertex& o = v[size_t(e.idx)];
        const float got[7] = {o.pos[0], o.pos[1], o.pos[2], o.uv0[0], o.uv0[1], o.uv1[0], o.uv1[1]};
        const float want[7] = {e.x, e.y, e.z, e.u0, e.v0, e.u1, e.v1};
        bool ok = std::abs(int(o.normal[0]) - e.nb) <= 1 && std::abs(int(o.normal[1]) - e.ng) <= 1 && std::abs(int(o.normal[2]) - e.nr) <= 1;
        for (int k = 0; k < 7; ++k) {
            ok = ok && close(got[k], want[k]);
            worst = std::max(worst, std::fabs(got[k] - want[k]));
        }
        ++n;
        if (!ok) {
            ++bad;
            std::printf("SD FAIL flags %x N %d vertex %d: port (%.7g %.7g %.7g) uv (%.7g %.7g) (%.7g %.7g) n %u %u %u, FUEL (%.7g %.7g %.7g) "
                        "uv (%.7g %.7g) (%.7g %.7g) n %u %u %u\n",
                        e.flags, e.n, e.idx, o.pos[0], o.pos[1], o.pos[2], o.uv0[0], o.uv0[1], o.uv1[0], o.uv1[1], o.normal[0], o.normal[1],
                        o.normal[2], e.x, e.y, e.z, e.u0, e.v0, e.u1, e.v1, e.nb, e.ng, e.nr);
        }
    }
    std::printf("draw 0x846f80: %d / %d vertices (worst %.3g)\n", n - bad, n, worst);
    int k = 0, gbad = 0;
    worst = 0;
    for (const GridVec& e : kGridVectors) {
        s.patches[0].flags = e.flags;
        const std::vector<std::array<float, 3>> g = fuel::patchCollisionGrid(s, s.patches[0], e.nu, e.nv);
        const auto& o = g[size_t(e.idx)];
        const bool ok = close(o[0], e.x) && close(o[1], e.y) && close(o[2], e.z);
        for (int c = 0; c < 3; ++c) worst = std::max(worst, std::fabs(o[size_t(c)] - (&e.x)[c]));
        ++k;
        if (!ok) {
            ++gbad;
            std::printf("SC FAIL flags %x %d x %d point %d port (%.7g %.7g %.7g) FUEL (%.7g %.7g %.7g)\n", e.flags, e.nu, e.nv, e.idx, o[0], o[1],
                        o[2], e.x, e.y, e.z);
        }
    }
    std::printf("collision grid 0x7cf0a0: %d / %d points (worst %.3g)\n", k - gbad, k, worst);
    bad += gbad;
    std::printf("surface vectors: %d / %d\n", n + k - bad, n + k);
    return bad == 0 ? 0 : 1;
}

struct Lookup {
    std::vector<std::unique_ptr<fuel::DpcFile>> own;
    std::vector<const fuel::DpcFile*> list;
};

int survey(const fs::path& data) {
    const fs::path world = data / "CommonComponent" / "WORLD";
    auto usa1 = std::make_unique<fuel::DpcFile>((world / "USA1.DPC").string());
    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(world))
        if (e.path().extension() == ".DPC") files.push_back(e.path());
    for (const auto& e : fs::recursive_directory_iterator(data))
        if (e.is_regular_file() && e.path().filename() == "MENU.DPC") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    int surfaces = 0, placements = 0;
    std::map<uint32_t, int> seen;
    for (const fs::path& f : files) {
        std::unique_ptr<fuel::DpcFile> own;
        const fuel::DpcFile* dpc = usa1.get();
        if (f.filename() != "USA1.DPC") {
            own = std::make_unique<fuel::DpcFile>(f.string());
            dpc = own.get();
        }
        const std::vector<const fuel::DpcFile*> lookup = dpc == usa1.get() ? std::vector<const fuel::DpcFile*>{dpc}
                                                                           : std::vector<const fuel::DpcFile*>{dpc, usa1.get()};
        for (uint32_t id : dpc->ids(fuel::cls::Surface)) {
            if (seen[id]++) continue;
            ++surfaces;
            fuel::SurfaceBody s;
            try {
                s = fuel::parseSurfaceBody(dpc->body(*dpc->find(id)).value());
            } catch (const std::exception& e) {
                std::printf("%s %08x: parse error %s\n", f.filename().c_str(), id, e.what());
                continue;
            }
            fuel::BakedMesh b;
            std::string err;
            const bool baked = fuel::bakeSurface(lookup, id, b, &err);
            // Front faces: the port draws clockwise front faces (FUEL's D3D order); a triangle (a, b, c) whose
            // (b - a) x (c - a) points along the vertex normal is front-facing in the right-handed sense used by
            // Mesh_Z data (see --data output "mesh agree").
            int agree = 0, tris = 0;
            for (size_t t = 0; baked && t + 3 <= b.indices.size(); t += 3) {
                const auto pa = b.position(b.indices[t]), pb = b.position(b.indices[t + 1]), pc = b.position(b.indices[t + 2]);
                const float e1[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]}, e2[3] = {pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]};
                const float cr[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
                const uint8_t* nb = &b.vertices[size_t(b.indices[t]) * fuel::BakedMesh::kStride + 16];
                const float n[3] = {nb[2] / 127.5f - 1, nb[1] / 127.5f - 1, nb[0] / 127.5f - 1};
                agree += cr[0] * n[0] + cr[1] * n[1] + cr[2] * n[2] > 0;
                ++tris;
            }
            const fuel::StaticCollision col = fuel::loadStaticCollision(lookup, id);
            std::map<std::string, int> mats;
            for (const auto& p : s.patches) {
                const uint32_t m = fuel::patchMaterial(lookup, p);
                char buf[64];
                uint32_t fl = 0, cf = 0;
                for (const fuel::DpcFile* d : lookup)
                    if (const auto* e = d->find(m); e != nullptr && e->header != nullptr && e->classCrc == fuel::cls::Material) {
                        const fuel::Material mm = fuel::parseMaterialBody(d->body(*e).value());
                        fl = mm.flags, cf = uint32_t(mm.field2c);
                        break;
                    }
                std::snprintf(buf, sizeof buf, "%08x code %u col %x", m, fl & 0x1f, cf);
                ++mats[buf];
            }
            std::printf("%s %08x: %zu patches, %zu points, %u vertices, %zu triangles (%d front-facing by normal), %zu collision "
                        "triangles%s%s\n",
                        f.filename().c_str(), id, s.patches.size(), s.points.size(), b.vertexCount(), b.indices.size() / 3, agree,
                        col.tris.size(), baked ? "" : " BAKE FAILED: ", baked ? "" : err.c_str());
            for (const auto& [m, c] : mats) std::printf("    material %s x%d\n", m.c_str(), c);
        }
        // What the port places (world.cpp): Surface_Z entries among the placed meshes, with their world position.
        const std::vector<fuel::PlacedMesh> placed = dpc == usa1.get() ? fuel::collectWorldRootMeshes(*dpc, lookup)
                                                                      : fuel::collectPlacedMeshes(*dpc, lookup);
        for (const fuel::PlacedMesh& pm : placed)
            if (dpc->find(pm.meshId) != nullptr && dpc->find(pm.meshId)->classCrc == fuel::cls::Surface)
                std::printf("  port places %08x (node %08x) at (%.1f, %.1f, %.1f)%s\n", pm.meshId, pm.nodeId, pm.world[12], pm.world[13],
                            pm.world[14], pm.collisionFirst && fuel::inPhysicsScene(pm.nodeFlags) ? " solid" : "");
        for (uint32_t nid : dpc->ids(0x85D02528u)) {  // Node_Z
            try {
                const fuel::Node n = fuel::parseNodeBody(dpc->body(*dpc->find(nid)).value());
                bool isSurface = false;
                for (const fuel::DpcFile* d : lookup)
                    if (const auto* e = d->find(n.object); e != nullptr && e->header != nullptr) {
                        isSurface = e->classCrc == fuel::cls::Surface;
                        break;
                    }
                if (!isSurface) continue;
                ++placements;
                std::printf("  placement %s node %08x -> %08x at (%.1f, %.1f, %.1f) flags %08x%s\n", f.filename().c_str(), nid, n.object,
                            n.world[12], n.world[13], n.world[14], n.flags, fuel::inPhysicsScene(n.flags) ? " solid" : "");
            } catch (const std::exception&) {
            }
        }
    }
    std::printf("surfaces %d, placements %d\n", surfaces, placements);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::string data;
    bool vec = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--data" && i + 1 < argc) data = argv[++i];
        else if (a == "--vectors") vec = true;
    }
    if (vec) return vectors();
    if (!data.empty()) return survey(data);
    std::fprintf(stderr, "usage: fuelsurface --data DIR | --vectors\n");
    return 2;
}
