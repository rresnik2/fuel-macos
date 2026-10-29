#include "vehicle.hpp"

#include "resources.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

namespace fuel {

namespace {

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("short body");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

uint32_t linkName(const DpcFile& dpc, const DpcFile::Entry& e) {
    const std::vector<uint8_t> link = dpc.link(e);
    return link.size() >= 4 ? rd32(link, 0) : 0;
}

}  // namespace

std::map<std::string, std::vector<Rgb8>> parseVehicleColors(const std::string& tscText) {
    std::map<std::string, std::vector<Rgb8>> out;
    std::istringstream in(tscText);
    std::string line;
    while (std::getline(in, line)) {
        char vehicle[128];
        unsigned r, g, b;
        if (std::sscanf(line.c_str(), " AddVehicleColor MESH_ID_%127s R%uG%uB%u", vehicle, &r, &g, &b) == 4 && r < 256 && g < 256 &&
            b < 256)
            out[vehicle].push_back({uint8_t(r), uint8_t(g), uint8_t(b)});
    }
    return out;
}

uint32_t liveryMaterialName(int lod) { return asoboCrc("LIVERY_XX_LOD" + std::to_string(lod)); }

std::array<uint32_t, 3> findLiveryTextures(const DpcFile& dpc, const std::string& vehicle, int index) {
    std::array<uint32_t, 3> tex{};
    const uint32_t key = asoboCrc(vehicle);
    // MaterialObj_Z body: u32 groupCount; per group {u32 key = CRC(vehicle); u32 n; u32 MaterialAnim_Z[n]}.
    for (uint32_t libId : dpc.ids(cls::MaterialObj)) {
        try {
            const auto body = dpc.body(*dpc.find(libId));
            if (!body) continue;
            size_t o = 4;
            const uint32_t groups = rd32(*body, 0);
            for (uint32_t g = 0; g < groups; ++g) {
                const uint32_t groupKey = rd32(*body, o);
                const uint32_t n = rd32(*body, o + 4);
                o += 8;
                if (groupKey != key) {
                    o += size_t(n) * 4;
                    continue;
                }
                for (int lod = 1; lod <= 3; ++lod) {
                    char name[32];
                    std::snprintf(name, sizeof name, "LIVERY_%02d_LOD%d", index, lod);
                    const uint32_t want = asoboCrc(name);
                    for (uint32_t i = 0; i < n; ++i) {
                        const DpcFile::Entry* anim = dpc.find(rd32(*body, o + i * 4));
                        if (anim == nullptr || linkName(dpc, *anim) != want) continue;
                        // MaterialAnim_Z body +0x3C: the livery's Material_Z; its slot 0 is the livery texture.
                        const auto animBody = dpc.body(*anim);
                        const DpcFile::Entry* mat = animBody ? dpc.find(rd32(*animBody, 0x3C)) : nullptr;
                        const auto matBody = mat ? dpc.body(*mat) : std::nullopt;
                        if (matBody) tex[size_t(lod - 1)] = parseMaterialBody(*matBody).textures[0];
                    }
                }
                return tex;
            }
        } catch (const std::exception&) {
        }
    }
    return tex;
}

}  // namespace fuel
