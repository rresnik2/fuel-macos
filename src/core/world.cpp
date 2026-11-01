#include "world.hpp"

#include "assembly.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
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

std::vector<PlacedMesh> collectPlacedMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup) {
    std::vector<PlacedMesh> out;
    for (uint32_t nodeId : dpc.ids(kNodeClass)) {
        try {
            const auto body = dpc.body(*dpc.find(nodeId));
            if (!body) continue;
            const Node node = parseNodeBody(*body);
            Found obj = find(lookup, node.object);
            if (obj.entry != nullptr && obj.entry->classCrc == kLodClass) {
                const auto lodBody = obj.file->body(*obj.entry);
                const Lod lod = lodBody ? parseLodBody(*lodBody) : Lod{};
                obj = lod.objects.empty() ? Found{} : find(lookup, lod.objects.front());
            }
            if (obj.entry == nullptr) continue;
            bool fit = false, normal = false;
            groundFlags(lookup, node.userDefine, fit, normal);
            if (obj.entry->classCrc == cls::Mesh) {
                out.push_back({obj.entry->id, nodeId, node.world, fit, normal});
            } else if (obj.entry->classCrc == kSkinClass) {
                const auto skinBody = obj.file->body(*obj.entry);
                if (!skinBody) continue;
                for (uint32_t mesh : parseSkinBody(*skinBody).meshes) out.push_back({mesh, nodeId, node.world, fit, normal});
            }
        } catch (const std::exception&) {
            // Nodes that do not parse are skipped.
        }
    }
    return out;
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
