#include "world.hpp"

#include "assembly.hpp"
#include "reader.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <functional>
#include <unordered_map>

namespace fuel {

namespace {

constexpr uint32_t kNodeClass = 0x85D02528;
constexpr uint32_t kLodClass = 0x73DC6A13;
constexpr uint32_t kSkinClass = 0x53415807;
constexpr uint32_t kUserDefineClass = 0x52F79F96;

struct Found {
    const DpcFile* file = nullptr;
    const DpcFile::Entry* entry = nullptr;
};

Found find(const std::vector<const DpcFile*>& lookup, uint32_t id) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) return {f, e};
    return {};
}

// FITTOGROUND / GROUNDNORMAL keywords of a node's UserDefine_Z. The engine's keyword search
// 0x7b1bc0 compares whole tokens case-insensitively (_strnicmp, equal lengths), so the data's
// five misspelt FITOGROUND never match.
void groundFlags(const std::vector<const DpcFile*>& lookup, uint32_t userDefine, bool& fit, bool& normal) {
    fit = normal = false;
    if (userDefine == 0) return;
    const Found f = find(lookup, userDefine);
    if (f.entry == nullptr || f.entry->classCrc != kUserDefineClass) return;
    const auto body = f.file->body(*f.entry);
    if (!body) return;
    for (const std::string& line : parseUserDefineBody(*body)) {
        if (line == "FITTOGROUND") fit = true;
        if (line == "GROUNDNORMAL") normal = true;
    }
}

}  // namespace

namespace {

// Object link float at link offset 88 (Object_Z +0x5c): the authored view distance (p6 W8).
float linkViewDistance(const Found& f) {
    const std::vector<uint8_t> link = f.file->link(*f.entry);
    if (link.size() < 92) return 0;
    float v;
    std::memcpy(&v, &link[88], 4);
    return std::isfinite(v) ? v : 0;
}

// Meshes of one LOD-level object: a Mesh_Z, or every mesh of a Skin_Z.
std::vector<uint32_t> levelMeshes(const Found& obj) {
    if (obj.entry == nullptr) return {};
    if (obj.entry->classCrc == cls::Mesh) return {obj.entry->id};
    if (obj.entry->classCrc == kSkinClass)
        if (const auto skinBody = obj.file->body(*obj.entry)) return parseSkinBody(*skinBody).meshes;
    return {};
}

void collect(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, uint32_t nodeId, std::vector<PlacedMesh>& out) {
    const auto body = dpc.body(*dpc.find(nodeId));
    if (!body) return;
    const Node node = parseNodeBody(*body);
    const Found obj = find(lookup, node.object);
    if (obj.entry == nullptr) return;
    PlacedMesh base;
    base.nodeId = nodeId;
    base.world = node.world;
    base.lodScale = node.lodScale;
    groundFlags(lookup, node.userDefine, base.fitToGround, base.groundNormal);
    std::vector<std::vector<uint32_t>> levels;  // meshes per level
    std::vector<std::vector<float>> views;
    auto addLevel = [&](const Found& o) {
        std::vector<uint32_t> m = levelMeshes(o);
        if (m.empty()) return;
        std::vector<float> v;
        for (uint32_t id : m) {
            const Found mf = find(lookup, id);
            v.push_back(mf.entry != nullptr && mf.entry->classCrc == cls::Mesh ? linkViewDistance(mf) : 0.0f);
        }
        levels.push_back(std::move(m));
        views.push_back(std::move(v));
    };
    if (obj.entry->classCrc == kLodClass) {
        const auto lodBody = obj.file->body(*obj.entry);
        if (!lodBody) return;
        const Lod lod = parseLodBody(*lodBody);
        base.lod = true;
        base.baseDistance = lod.baseDistance, base.nearFade = lod.nearFade, base.hardCull = lod.hardCull;
        base.viewDist = linkViewDistance(obj);
        for (uint32_t o : lod.objects) {
            const size_t before = levels.size();
            addLevel(find(lookup, o));
            if (levels.size() == before && !levels.empty()) levels.push_back(levels.back()), views.push_back(views.back());
        }
    } else {
        addLevel(obj);
    }
    if (levels.empty()) return;
    for (size_t j = 0; j < levels[0].size(); ++j) {  // one entry per mesh of the first level
        PlacedMesh pm = base;
        pm.meshId = levels[0][j];
        for (size_t k = 0; k < levels.size(); ++k) {
            const size_t i = std::min(j, levels[k].size() - 1);
            pm.levels.push_back(levels[k][i]);
            pm.levelView.push_back(views[k][i]);
        }
        out.push_back(std::move(pm));
    }
}

}  // namespace

std::vector<PlacedMesh> collectPlacedMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup) {
    std::vector<PlacedMesh> out;
    for (uint32_t nodeId : dpc.ids(kNodeClass)) {
        try {
            collect(dpc, lookup, nodeId, out);
        } catch (const std::exception&) {
            // Nodes that do not parse are skipped.
        }
    }
    return out;
}

std::vector<PlacedMesh> collectNodeMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, uint32_t nodeId) {
    std::vector<PlacedMesh> out;
    try {
        if (dpc.find(nodeId) != nullptr) collect(dpc, lookup, nodeId, out);
    } catch (const std::exception&) {
    }
    return out;
}

std::vector<PlacedMesh> collectWorldRootMeshes(const DpcFile& usa1, const std::vector<const DpcFile*>& lookup) {
    constexpr uint32_t kWorldClass = 0x39B67ECB;  // World_Z
    uint32_t root = 0;
    for (uint32_t id : usa1.ids(kWorldClass))
        if (const auto b = usa1.body(*usa1.find(id)); b && b->size() >= 4) std::memcpy(&root, b->data(), 4);
    std::vector<PlacedMesh> out;
    if (root == 0) return out;
    std::unordered_map<uint32_t, uint32_t> parent;
    for (uint32_t nodeId : usa1.ids(kNodeClass)) {
        try {
            if (const auto b = usa1.body(*usa1.find(nodeId))) parent[nodeId] = parseNodeBody(*b).parent;
        } catch (const std::exception&) {
        }
    }
    std::unordered_map<uint32_t, bool> under;  // memo: node reaches `root` through its parents
    std::function<bool(uint32_t, int)> reaches = [&](uint32_t n, int depth) -> bool {
        if (n == root) return true;
        if (n == 0 || depth > 64) return false;
        if (auto it = under.find(n); it != under.end()) return it->second;
        const auto p = parent.find(n);
        const bool r = p != parent.end() && reaches(p->second, depth + 1);
        under[n] = r;
        return r;
    };
    for (uint32_t nodeId : usa1.ids(kNodeClass)) {
        if (!reaches(nodeId, 0)) continue;
        try {
            collect(usa1, lookup, nodeId, out);
        } catch (const std::exception&) {
        }
    }
    return out;
}

int lodLevel(const PlacedMesh& p, float d, const LodView& v) {
    const int hi = int(p.levels.size()) - 1;
    if (!p.lod || hi <= 0) return 0;
    const float s = p.lodScale * v.viewScale;
    const float x = d / (v.K * p.baseDistance * s);
    uint32_t bits;
    std::memcpy(&bits, &x, 4);
    const int level = int(bits >> 23) - 0x7e;  // floor(log2 x) + 1 (0x884170)
    return std::clamp(level, 0, hi);
}

float lodFade(float viewDist, float nearFade, float hardCull, float s, float d, const LodView& v) {
    const float K = v.Kfade;
    if (hardCull != 0 && hardCull * K < d) return 0;
    float f = 1;
    const float e = viewDist * K * s;
    if (viewDist != 0 && e < d) {
        const float t = 20.0f - (d - e);  // [0x996154]
        if (t <= 0) return t;
        f = t * 0.05f;  // [0x996158]
    }
    if (nearFade != 0 && s * nearFade * K > d) {
        const float q = d / (s * nearFade * K);
        return std::clamp(q * q * q, 0.0f, 1.0f);
    }
    return f;
}

float placementAlpha(const PlacedMesh& p, int level, float d, const LodView& v) {
    const float s = p.lodScale * v.viewScale;
    if (p.lod && (p.viewDist != 0 || p.hardCull != 0 || p.nearFade != 0)) {
        const float a = lodFade(p.viewDist, p.nearFade, p.hardCull, s, d, v);
        if (!(a >= 0.01f)) return 0;  // [0x99614c]
    }
    // Mesh_Z::Draw 0x80a280: the level mesh's own fade (its +0x11c / +0x124 are taken as 0).
    const float view = level < int(p.levelView.size()) ? p.levelView[size_t(level)] : 0.0f;
    return std::max(0.0f, lodFade(view, 0, 0, s, d, v));
}

std::vector<SubWorldRef> loadSubWorldRefs(const DpcFile& usa1) {
    const uint32_t kWorldRef = asoboCrc("WorldRef_Z");
    std::vector<SubWorldRef> out;
    std::unordered_map<uint32_t, size_t> byId;
    for (uint32_t id : usa1.ids(kWorldRef)) {
        try {
            const DpcFile::Entry& e = *usa1.find(id);
            const std::vector<uint8_t> link = usa1.link(e);
            const auto body = usa1.body(e);
            if (!body || link.size() < 96) continue;
            SubWorldRef r;
            r.id = id;
            std::memcpy(&r.flags, &link[92], 4);  // object flags (P31 out-F: 0x001ffc3c / 0x005ffc3c / 0x00dffc3c)
            ByteReader b(*body);
            b.raw(32);                          // embedded member (8 u32)
            b.raw(size_t(b.count(1u << 20)) * 64);  // OBBs
            for (int k = 0; k < 4; ++k) b.f32();    // xmin, zmin, xmax, zmax
            r.cx = b.f32(), r.cz = b.f32();
            b.raw(size_t(b.count(1u << 20)) * 8);   // outline polygon
            const uint32_t len = b.count(1u << 16);
            const char* c = reinterpret_cast<const char*>(b.raw(len));
            const std::string cmd(c, strnlen(c, len));
            // "OpenBF WORLD\USA1_CR31_343CC3A6;LoadWorldRef DB:>...;CloseBF"
            if (const size_t at = cmd.find("WORLD\\"); at != std::string::npos) {
                const size_t end = cmd.find(';', at);
                r.file = cmd.substr(at + 6, end == std::string::npos ? std::string::npos : end - at - 6) + ".DPC";
            }
            r.kind = r.file.empty() ? SubWorldRef::Resident : (r.flags & 0x800000) ? SubWorldRef::Region : SubWorldRef::Course;
            byId[id] = out.size();
            out.push_back(std::move(r));
        } catch (const std::exception&) {
            // A body that does not parse is not a sub-world the port can stream.
        }
    }
    // The mission that streams a course in: the UserDefine of the USA1 node holding the WorldRef.
    for (uint32_t nodeId : usa1.ids(kNodeClass)) {
        try {
            const auto body = usa1.body(*usa1.find(nodeId));
            if (!body) continue;
            const Node node = parseNodeBody(*body);
            auto it = byId.find(node.object);
            if (it == byId.end() || node.userDefine == 0) continue;
            const DpcFile::Entry* ud = usa1.find(node.userDefine);
            if (ud == nullptr || ud->classCrc != kUserDefineClass) continue;
            const auto udBody = usa1.body(*ud);
            if (!udBody) continue;
            for (const std::string& line : parseUserDefineBody(*udBody))
                if (line.rfind("STREAM_M_TELEPORT ", 0) == 0) out[it->second].mission = line.substr(18);
        } catch (const std::exception&) {
        }
    }
    return out;
}

std::vector<size_t> nearestRegions(const std::vector<SubWorldRef>& refs, double x, double z) {
    std::vector<std::pair<float, size_t>> d;
    for (size_t i = 0; i < refs.size(); ++i)
        if (refs[i].kind == SubWorldRef::Region) d.push_back({float(std::sqrt((refs[i].cx - x) * (refs[i].cx - x) + (refs[i].cz - z) * (refs[i].cz - z))), i});
    std::sort(d.begin(), d.end());  // 0x57d770
    std::vector<size_t> keep;
    for (size_t k = 0; k < std::min<size_t>(2, d.size()); ++k)
        if (d[k].first <= 5000.0f) keep.push_back(d[k].second);  // [0x9dc3f8] = 5000, equality loads
    return keep;
}

CollisionMesh loadCollisionMesh(const std::vector<const DpcFile*>& lookup, uint32_t meshId) {
    CollisionMesh out;
    const Found f = find(lookup, meshId);
    if (f.entry == nullptr || f.entry->classCrc != cls::Mesh) return out;
    const MeshLink link = parseMeshLink(f.file->link(*f.entry));
    const auto body = f.file->body(*f.entry);
    if (!body) return out;
    const Mesh mesh = parseMeshBody(*body, link.flags);
    for (const Batch& b : mesh.batches) {
        if (b.vb >= mesh.vbs.size() || mesh.vbs[b.vb].stride < 28) continue;  // skip morph streams
        const VertexBuffer& vb = mesh.vbs[b.vb];
        const std::vector<uint32_t> tris = batchTriangles(mesh, b);
        // Copy the referenced vertices; positions are float3 at the start of each vertex.
        const uint32_t base = uint32_t(out.positions.size() / 3);
        std::unordered_map<uint32_t, uint32_t> remap;
        for (uint32_t v : tris) {
            auto [it, inserted] = remap.try_emplace(v, base + uint32_t(remap.size()));
            if (inserted) {
                float p[3];
                std::memcpy(p, &vb.data[size_t(v) * vb.stride], 12);
                out.positions.insert(out.positions.end(), p, p + 3);
                out.radius = std::max(out.radius, std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]));
            }
            out.indices.push_back(it->second);
        }
    }
    return out;
}

}  // namespace fuel
