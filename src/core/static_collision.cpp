#include "static_collision.hpp"

#include "assembly.hpp"
#include "reader.hpp"
#include "resources.hpp"
#include "surface_patch.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace fuel {

namespace {

constexpr uint32_t kVehicleMask = 2;             // shape type / material flag for vehicles (0x71dd80 pushes 2)
constexpr uint32_t kTriangleExclude = 0x808;     // material flags that drop a triangle (0x71dd80 pushes 0x808)
constexpr uint32_t kLodMeshShapes = 0x400000;    // Lod+0x60: the Lod uses its first mesh's boxes and spheres

struct Found {
    const DpcFile* file = nullptr;
    const DpcFile::Entry* entry = nullptr;
};
Found find(const std::vector<const DpcFile*>& lookup, uint32_t id) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) return {f, e};
    return {};
}

// Named points (FUN_00809550 spheres 24 B, FUN_008095c0 boxes 72 B), kept when type & 2.
void readShapes(ByteReader& r, StaticCollision& out) {
    const uint32_t ns = r.count(100000);
    for (uint32_t i = 0; i < ns; ++i) {
        StaticCollision::Sphere s;
        for (float& c : s.c) c = r.f32();
        s.r = r.f32();
        s.type = r.u32();
        r.u32();  // name
        if (s.type & kVehicleMask) out.spheres.push_back(s);
    }
    const uint32_t nb = r.count(100000);
    for (uint32_t i = 0; i < nb; ++i) {
        StaticCollision::Box b;
        for (float& f : b.frame) f = r.f32();
        for (float& f : b.half) f = r.f32();
        b.r = r.f32();
        b.type = r.u32();
        r.u32();  // name
        if (b.type & kVehicleMask) out.boxes.push_back(b);
        const uint32_t kind = b.type & 0x1c000000u;  // 0x619d70 (P58)
        if (kind != 0x10000000u && ((b.type & 7u) != 0 || kind == 0x8000000u || kind == 0x4000000u)) out.aiBoxes.push_back(b);
    }
}

void meshShapes(const Found& m, StaticCollision& out) {  // Mesh_Z link: header, dependency ids, 3 u32, points
    const std::vector<uint8_t> link = m.file->link(*m.entry);
    if (link.size() < 98) return;
    ByteReader r(link, 98);
    const uint32_t deps = r.count(100000);
    r.raw(size_t(deps) * 4 + 12);
    readShapes(r, out);
}

uint32_t materialFlags(const std::vector<const DpcFile*>& lookup, uint32_t id) {  // Material_Z +0x38
    const Found f = find(lookup, id);
    if (f.entry == nullptr || f.entry->classCrc != cls::Material) return 0;
    const auto body = f.file->body(*f.entry);
    if (!body || body->size() < 0x30) return 0;
    uint32_t v;
    std::memcpy(&v, body->data() + 0x2c, 4);
    return v;
}

void meshTriangles(const std::vector<const DpcFile*>& lookup, const Found& m, StaticCollision& out) {  // 0x80b1d0
    const auto body = m.file->body(*m.entry);
    if (!body) return;
    const MeshLink link = parseMeshLink(m.file->link(*m.entry));
    const Mesh mesh = parseMeshBody(*body, link.flags);
    if (mesh.collisionNodes.empty() || mesh.collisionTris.empty()) return;
    std::vector<uint32_t> flags(mesh.materials.size());
    for (size_t i = 0; i < flags.size(); ++i) flags[i] = materialFlags(lookup, mesh.materials[i]);
    const size_t nv = mesh.collisionVerts.size() / 3;
    for (size_t t = 0; t + 4 <= mesh.collisionTris.size(); t += 4) {
        const uint16_t mat = mesh.collisionTris[t + 3];
        const uint32_t f = mat < flags.size() ? flags[mat] : 0;
        if (!(f & kVehicleMask) || (f & kTriangleExclude)) continue;
        std::array<float, 9> tri{};
        bool ok = true;
        for (int k = 0; k < 3; ++k) {
            const uint16_t v = mesh.collisionTris[t + size_t(k)];
            if (v >= nv) {
                ok = false;
                break;
            }
            for (int c = 0; c < 3; ++c) tri[size_t(k * 3 + c)] = float(mesh.collisionVerts[size_t(v) * 3 + size_t(c)]) * 0.0009765625f;
        }
        if (!ok) continue;
        out.tris.push_back(tri);
        out.triFlags.push_back(f);
    }
}

// Surface_Z::vf34 0x812120 -> 0x811bc0 (P73): each patch's collision grid (0x7cf0a0, nu x nv cells from the patch
// flags), two triangles per cell, kept with the vehicle masks on the patch material's flags (Material +0x38).
void surfaceTriangles(const std::vector<const DpcFile*>& lookup, const Found& s, StaticCollision& out) {
    const auto body = s.file->body(*s.entry);
    if (!body) return;
    const SurfaceBody sb = parseSurfaceBody(*body);
    for (const SurfacePatch& p : sb.patches) {
        const uint32_t f = materialFlags(lookup, patchMaterial(lookup, p));
        if (!(f & kVehicleMask) || (f & kTriangleExclude)) continue;
        const int nu = p.collisionNu(), nv = p.collisionNv();
        const std::vector<std::array<float, 3>> g = patchCollisionGrid(sb, p, nu, nv);
        const std::vector<uint32_t> idx = patchGridTriangles(nu, nv);
        for (size_t t = 0; t + 3 <= idx.size(); t += 3) {
            std::array<float, 9> tri{};
            for (size_t k = 0; k < 3; ++k)
                for (size_t c = 0; c < 3; ++c) tri[k * 3 + c] = g[idx[t + k]][c];
            out.tris.push_back(tri);
            out.triFlags.push_back(f);
        }
    }
}

// Level-0 mesh of a Lod's first object (a Mesh_Z; Skin_Z levels collide with nothing: their slots are the base
// class's).
Found firstMesh(const std::vector<const DpcFile*>& lookup, const Lod& lod) {
    for (uint32_t o : lod.objects) {
        const Found f = find(lookup, o);
        if (f.entry != nullptr) return f.entry->classCrc == cls::Mesh ? f : Found{};
    }
    return {};
}

}  // namespace

StaticCollision loadStaticCollision(const std::vector<const DpcFile*>& lookup, uint32_t objectId) {
    StaticCollision out;
    const Found obj = find(lookup, objectId);
    if (obj.entry == nullptr) return out;
    if (obj.entry->classCrc == cls::Lod) {
        const auto body = obj.file->body(*obj.entry);
        if (!body) return out;
        ByteReader r(*body);
        readShapes(r, out);
        const Lod lod = parseLodBody(*body);
        const std::vector<uint8_t> link = obj.file->link(*obj.entry);
        uint32_t lodFlags = 0;
        if (link.size() >= 0x60) std::memcpy(&lodFlags, &link[0x5c], 4);
        const Found mesh = firstMesh(lookup, lod);
        if (mesh.entry != nullptr) {
            if (lodFlags & kLodMeshShapes)
                meshShapes(mesh, out);
            else
                meshTriangles(lookup, mesh, out);
        }
    } else if (obj.entry->classCrc == cls::Mesh) {
        meshShapes(obj, out);
        meshTriangles(lookup, obj, out);
    } else if (obj.entry->classCrc == cls::Surface) {
        try {
            surfaceTriangles(lookup, obj, out);
        } catch (const std::exception&) {
            out.tris.clear(), out.triFlags.clear();
        }
    }
    for (const auto& s : out.spheres) out.radius = std::max(out.radius, std::hypot(s.c[0], s.c[1], s.c[2]) + s.r);
    for (const auto& b : out.boxes)
        out.radius = std::max(out.radius, std::hypot(b.frame[3], b.frame[7], b.frame[11]) + std::hypot(b.half[0], b.half[1], b.half[2]));
    for (const auto& t : out.tris)
        for (int k = 0; k < 3; ++k) out.radius = std::max(out.radius, std::hypot(t[size_t(3 * k)], t[size_t(3 * k + 1)], t[size_t(3 * k + 2)]));
    return out;
}

const char* gameClassName(GameClass c) {
    switch (c) {
    case GameClass::None: return "none";
    case GameClass::DecorMesh: return "IT_DECORMESH_G";
    case GameClass::DecorDynMesh: return "IT_DECORDYNMESH_G";
    case GameClass::DynObj: return "IT_DYNOBJ_G";
    case GameClass::Debris: return "IT_Debris";
    case GameClass::Collect: return "IT_DYNCOLLECT";
    case GameClass::Animated: return "IT_ANIMATED";
    case GameClass::Agent: return "Agent";
    case GameClass::Other: return "other";
    }
    return "?";
}

std::unordered_map<uint32_t, GameClass> loadCloneClasses(const std::string& path) {
    std::unordered_map<uint32_t, GameClass> out;
    std::ifstream f(path, std::ios::binary);
    if (!f) return out;
    std::unordered_map<std::string, std::string> base;  // upper-case clone name -> base name
    std::vector<std::string> order;
    std::string line;
    auto upper = [](std::string s) {
        for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
        return s;
    };
    while (std::getline(f, line)) {
        std::istringstream in(line);
        std::string cmd, name, cls;
        if (!(in >> cmd >> name >> cls) || upper(cmd) != "CLONECLASS") continue;
        if (base.emplace(upper(name), cls).second) order.push_back(name);
    }
    for (const std::string& name : order) {
        std::string b = base[upper(name)];
        for (int k = 0; k < 8; ++k) {  // clones of clones ("cit")
            auto it = base.find(upper(b));
            if (it == base.end()) break;
            b = it->second;
        }
        const std::string u = upper(b);
        GameClass c = GameClass::Other;
        if (u == "IT_DECORMESH_G") c = GameClass::DecorMesh;
        else if (u == "IT_DECORDYNMESH_G") c = GameClass::DecorDynMesh;
        else if (u == "IT_DYNOBJ_G") c = GameClass::DynObj;
        else if (u == "IT_DEBRIS" || u == "IT_SMALLDEBRIS") c = GameClass::Debris;
        else if (u.rfind("IT_DYNCOLLECT", 0) == 0 || u == "IT_COLLECT_G") c = GameClass::Collect;
        else if (u == "IT_ANIMATED") c = GameClass::Animated;
        else if (u == "AGENT_Z" || u == "BASEAGENT_G") c = GameClass::Agent;
        out.emplace(asoboCrc(name), c);
    }
    return out;
}

std::vector<float> collisionHullPoints(const StaticCollision& c) {
    std::vector<float> p;
    for (const auto& s : c.spheres)
        for (int a = 0; a < 3; ++a)
            for (int sg = -1; sg <= 1; sg += 2) {
                std::array<float, 3> q = s.c;
                q[size_t(a)] += float(sg) * s.r;
                p.insert(p.end(), q.begin(), q.end());
            }
    for (const auto& b : c.boxes)
        for (int k = 0; k < 8; ++k) {
            float q[3];
            for (int i = 0; i < 3; ++i) {
                q[i] = b.frame[size_t(4 * i + 3)];
                for (int a = 0; a < 3; ++a) q[i] += ((k >> a) & 1 ? 1.0f : -1.0f) * b.half[size_t(a)] * b.frame[size_t(4 * a + i)];
            }
            p.insert(p.end(), q, q + 3);
        }
    for (const auto& t : c.tris) p.insert(p.end(), t.begin(), t.end());
    return p;
}

}  // namespace fuel
