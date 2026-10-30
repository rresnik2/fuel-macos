#include "vegetation.hpp"
#include "world.hpp"

#include "assembly.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr float kCell = 20.0f;  // one candidate per 20 m cell

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

uint32_t hash(int32_t x, int32_t z, uint32_t salt) {
    uint32_t h = uint32_t(x) * 0x8DA6B343u ^ uint32_t(z) * 0xD8163841u ^ salt * 0xCB1AB31Fu;
    h ^= h >> 13;
    h *= 0x5BD1E995u;
    return h ^ (h >> 15);
}
float unit(uint32_t h) { return float(h & 0xFFFFFF) / float(0x1000000); }

float valueNoise(float x, float z, uint32_t salt) {
    const float fx = std::floor(x), fz = std::floor(z);
    float tx = x - fx, tz = z - fz;
    tx = tx * tx * (3 - 2 * tx);
    tz = tz * tz * (3 - 2 * tz);
    const int32_t ix = int32_t(fx), iz = int32_t(fz);
    const float a = unit(hash(ix, iz, salt)), b = unit(hash(ix + 1, iz, salt));
    const float c = unit(hash(ix, iz + 1, salt)), d = unit(hash(ix + 1, iz + 1, salt));
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * tz;
}

// FUEL's density contrast curve on [-1, 1].
float sharpen(float a) { return a <= 0 ? std::pow(a + 1, 8.0f) - 1 : 1 - std::pow(1 - a, 8.0f); }

float sampleMap(const Raster& r, float x, float z) {
    if (r.raw.empty()) return 0;
    const float fx = std::clamp((x + 70000.0f) / 140000.0f * r.width - 0.5f, 0.0f, float(r.width - 1));
    const float fz = std::clamp((z + 70000.0f) / 140000.0f * r.height - 0.5f, 0.0f, float(r.height - 1));
    const uint32_t x0 = uint32_t(fx), z0 = uint32_t(fz);
    const uint32_t x1 = std::min(x0 + 1, r.width - 1), z1 = std::min(z0 + 1, r.height - 1);
    const float tx = fx - x0, tz = fz - z0;
    const float a = r.at(x0, z0) + (r.at(x1, z0) - r.at(x0, z0)) * tx;
    const float b = r.at(x0, z1) + (r.at(x1, z1) - r.at(x0, z1)) * tx;
    return a + (b - a) * tz;
}

}  // namespace

Vegetation loadVegetation(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto body = world.body(*world.find(gens.front()));
    if (!body) throw std::runtime_error("GenWorld_Z has no body");
    const std::vector<uint8_t>& b = *body;
    // u32, u32, u32 roads; u32 n, maps[n]; u32 n, ground textures[n]; u32 n, materials[n]; u32;
    // u32 groups, each {u32 len, name[len], u32, u32 n, Node_Z[n]}.
    size_t p = 12;
    std::vector<uint32_t> maps;
    const uint32_t nMaps = rd32(b, p);
    for (uint32_t i = 0; i < nMaps; ++i) maps.push_back(rd32(b, p + 4 + size_t(i) * 4));
    p += 4 + size_t(nMaps) * 4;
    p += 4 + size_t(rd32(b, p)) * 4;
    p += 4 + size_t(rd32(b, p)) * 4;
    p += 4;
    Vegetation v;
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        const uint32_t len = rd32(b, p);
        if (p + 4 + len > b.size()) throw std::runtime_error("GenWorld_Z group name overruns");
        const std::string name(reinterpret_cast<const char*>(&b[p + 4]), strnlen(reinterpret_cast<const char*>(&b[p + 4]), len));
        p += 4 + len + 4;
        const uint32_t n = rd32(b, p);
        p += 4;
        std::vector<Species>* list = name == "TREES" ? &v.trees : name == "BUSHES" ? &v.bushes : nullptr;
        for (uint32_t i = 0; i < n; ++i) {
            const uint32_t nodeId = rd32(b, p + size_t(i) * 4);
            if (list == nullptr) continue;
            try {  // Node_Z -> Lod_Z -> first (full-detail) mesh
                const DpcFile::Entry* ne = world.find(nodeId);
                const auto nb = ne ? world.body(*ne) : std::nullopt;
                if (!nb) continue;
                const DpcFile::Entry* le = world.find(parseNodeBody(*nb).object);
                const auto lb = le ? world.body(*le) : std::nullopt;
                if (!lb) continue;
                const Lod lod = parseLodBody(*lb);
                if (lod.objects.empty() || world.find(lod.objects[0]) == nullptr) continue;
                Species sp;
                sp.full = lod.objects[0];
                if (lod.objects.size() > 1 && world.find(lod.objects[1]) != nullptr) sp.impostor = lod.objects[1];
                list->push_back(sp);
            } catch (const std::exception&) {
            }
        }
        p += size_t(n) * 4;
    }
    auto loadMap = [&](size_t slot) {
        Raster r;
        if (slot < maps.size() && maps[slot] != 0)
            if (const DpcFile::Entry* e = world.find(maps[slot]))
                if (const auto mb = world.body(*e)) r = decodeRaster(*mb);
        return r;
    };
    // Clutter: the bushes (shrunk) and the grass clump among the trees, and FUEL's
    // grey-detail rock cluster (it takes the ground colour, so it suits any terrain).
    // Only the genuinely low ones: the BUSHES group also holds saplings.
    auto lowPlant = [&](uint32_t id) {
        try {
            const CollisionMesh cm = loadCollisionMesh({&world}, id);
            float top = 0;
            for (size_t k = 1; k < cm.positions.size(); k += 3) top = std::max(top, cm.positions[k]);
            return !cm.positions.empty() && top < 2.5f;
        } catch (const std::exception&) {
            return false;
        }
    };
    for (const Species& sp : v.bushes)
        if (lowPlant(sp.full)) v.tufts.push_back(sp.full);
    for (uint32_t id : {0x6C1D4E09u})
        if (world.find(id) && lowPlant(id)) v.tufts.push_back(id);
    for (uint32_t id : {0xD98608D9u}) if (world.find(id)) v.stones.push_back(id);
    v.forest = loadMap(7);
    v.humidity = loadMap(2);
    return v;
}

void scatterPlants(const Vegetation& v, const Heightfield& terrain, float x, float z, float radius, std::vector<Plant>& out,
                   bool impostors, float inner) {
    if (v.trees.empty() && v.bushes.empty()) return;
    const int32_t c0x = int32_t(std::floor((x - radius) / kCell)), c1x = int32_t(std::floor((x + radius) / kCell));
    const int32_t c0z = int32_t(std::floor((z - radius) / kCell)), c1z = int32_t(std::floor((z + radius) / kCell));
    for (int32_t cz = c0z; cz <= c1z; ++cz) {
        for (int32_t cx = c0x; cx <= c1x; ++cx) {
            for (int kind = 0; kind < 2; ++kind) {  // a tree candidate, then a bush candidate offset by half a cell
                const std::vector<Species>& species = kind == 0 ? v.trees : v.bushes;
                if (species.empty() || (impostors && kind == 1)) continue;
                const uint32_t h = hash(cx, cz, 17u + uint32_t(kind));
                const float px = cx * kCell + kind * 10.0f + float(h & 15) * 1.2f;
                const float pz = cz * kCell + kind * 10.0f + float((h >> 4) & 15) * 1.2f;
                const float dist = std::hypot(px - x, pz - z);
                if (dist > radius || dist < inner) continue;
                // Density: sharpened forest cover minus clearing noise.
                const float forest = sampleMap(v.forest, px, pz);
                const float noise = std::clamp(0.65f * valueNoise(px / 400, pz / 400, 3) + 0.35f * valueNoise(px / 90, pz / 90, 4), 0.0f, 1.0f);
                float d = std::clamp((sharpen(2 * forest - 1) * 0.5f + 0.5f) - 0.95f * (sharpen(2 * noise * noise - 1) * 0.5f + 0.5f), 0.0f, 1.0f);
                if (kind == 1) d = std::max(d * 0.7f, forest * 0.15f);  // bushes also dot open ground
                if (unit(hash(cx, cz, 91u + uint32_t(kind))) >= d) continue;
                const float hum = sampleMap(v.humidity, px, pz);
                const float y = terrain.smoothHeightAt(px, pz);
                if (y > 3000.0f - 500.0f * hum) continue;  // treeline
                const float dx = (terrain.heightAt(px + 4, pz) - terrain.heightAt(px - 4, pz)) / 8;
                const float dz = (terrain.heightAt(px, pz + 4) - terrain.heightAt(px, pz - 4)) / 8;
                if (1.0f / std::sqrt(1 + dx * dx + dz * dz) < 0.866f) continue;  // steeper than 30 degrees
                const uint32_t pick = hash(cx, cz, 33u + uint32_t(kind));
                const Species* sp = nullptr;
                for (size_t tries = 0; tries < species.size() && sp == nullptr; ++tries) {
                    const Species& cand = species[(pick + tries * 7919u) % species.size()];
                    if (!impostors || cand.impostor != 0) sp = &cand;
                }
                if (sp == nullptr) continue;
                Plant p;
                p.meshId = impostors ? sp->impostor : sp->full;
                p.position = {px, y - 0.1f, pz};
                p.yaw = unit(hash(cx, cz, 57u)) * 6.2831853f;
                // Species meshes are authored small; the world places similar trees at 2-2.7x.
                p.scale = (kind == 0 ? 2.0f : 1.4f) + 0.8f * unit(hash(cx, cz, 71u));
                out.push_back(p);
            }
        }
    }
}

void scatterClutter(const Vegetation& v, const Heightfield& terrain, const SurfaceQuery& surface, float x, float z, float radius,
                    std::vector<Plant>& out) {
    if (!surface || (v.tufts.empty() && v.stones.empty())) return;
    constexpr float kStep = 2.5f;  // one candidate per 2.5 m cell
    const int32_t c0x = int32_t(std::floor((x - radius) / kStep)), c1x = int32_t(std::floor((x + radius) / kStep));
    const int32_t c0z = int32_t(std::floor((z - radius) / kStep)), c1z = int32_t(std::floor((z + radius) / kStep));
    for (int32_t cz = c0z; cz <= c1z; ++cz)
        for (int32_t cx = c0x; cx <= c1x; ++cx) {
            const uint32_t h = hash(cx, cz, 401u);
            const float px = (cx + unit(h)) * kStep, pz = (cz + unit(hash(cx, cz, 403u))) * kStep;
            if (std::hypot(px - x, pz - z) > radius) continue;
            const int type = surface(px, pz);
            if (type == 11 || type == 13) continue;  // roads (tarmac, dirt tracks)
            const GripCategory g = gripCategory(type);
            // Patchy, not uniform: thin out by a noise field.
            const float patch = valueNoise(px / 25, pz / 25, 7);  // 0..1
            const float r = unit(hash(cx, cz, 409u));
            const bool grassy = g == GripCategory::Grass || g == GripCategory::Mud;
            const bool stony = g == GripCategory::Rock || g == GripCategory::Dirt || g == GripCategory::Sand;
            Plant p;
            if (grassy && !v.tufts.empty() && r < 0.25f + 0.45f * patch) {
                p.meshId = v.tufts[hash(cx, cz, 411u) % v.tufts.size()];
                p.scale = 0.45f + 0.5f * unit(hash(cx, cz, 413u));
            } else if ((stony || grassy) && !v.stones.empty() && r > 1.0f - (stony ? 0.18f : 0.03f) * patch) {
                p.meshId = v.stones[hash(cx, cz, 415u) % v.stones.size()];
                p.scale = 0.03f + 0.07f * unit(hash(cx, cz, 417u)) * unit(hash(cx, cz, 419u));
            } else {
                continue;
            }
            p.position = {px, terrain.smoothHeightAt(px, pz) - 0.05f, pz};
            p.yaw = unit(hash(cx, cz, 421u)) * 6.2831853f;
            out.push_back(p);
        }
}

}  // namespace fuel
