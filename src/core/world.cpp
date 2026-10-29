#include "world.hpp"

#include "assembly.hpp"
#include "resources.hpp"

namespace fuel {

namespace {

constexpr uint32_t kNodeClass = 0x85D02528;
constexpr uint32_t kLodClass = 0x73DC6A13;
constexpr uint32_t kSkinClass = 0x53415807;

struct Found {
    const DpcFile* file = nullptr;
    const DpcFile::Entry* entry = nullptr;
};

Found find(const std::vector<const DpcFile*>& lookup, uint32_t id) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) return {f, e};
    return {};
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
            if (obj.entry->classCrc == cls::Mesh) {
                out.push_back({obj.entry->id, nodeId, node.world});
            } else if (obj.entry->classCrc == kSkinClass) {
                const auto skinBody = obj.file->body(*obj.entry);
                if (!skinBody) continue;
                for (uint32_t mesh : parseSkinBody(*skinBody).meshes) out.push_back({mesh, nodeId, node.world});
            }
        } catch (const std::exception&) {
            // Nodes that do not parse are skipped.
        }
    }
    return out;
}

}  // namespace fuel
