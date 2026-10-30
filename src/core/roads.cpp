#include "roads.hpp"

#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr float kRoadUnit = 0.25f;  // GwRoad_Z coordinates are quarter metres

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("road data too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

}  // namespace

RoadNetwork loadRoads(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    // GenWorld_Z: u32, u32 palette, u32 GwRoad_Z; maps; ground textures; then the
    // texture-enum materials (index 0x2a.. Road01-05, 0x34 Dirt01).
    const uint32_t roadId = rd32(*gen, 8);
    size_t p = 12;
    p += 4 + size_t(rd32(*gen, p)) * 4;
    p += 4 + size_t(rd32(*gen, p)) * 4;
    const uint32_t nMaterials = rd32(*gen, p);
    auto textureOf = [&](uint32_t e) -> uint32_t {
        if (e >= nMaterials) return 0;
        const DpcFile::Entry* me = world.find(rd32(*gen, p + 4 + size_t(e) * 4));
        const auto mb = me ? world.body(*me) : std::nullopt;
        try {
            return mb ? parseMaterialBody(*mb).textures[0] : 0;
        } catch (const std::exception&) {
            return 0;
        }
    };

    RoadNetwork net;
    // Road table (from the game's AGW_TerraRoad init): drawn width, texture enum, opacity.
    const uint32_t dirt = textureOf(0x34);
    auto style = [&](int type, float width, uint32_t tex, float opacity) { net.styles[size_t(type)] = {width, tex, opacity}; };
    style(4, 2.5f, dirt, 0.35f);   // SENTIER_PETIT: footpath
    style(5, 4.0f, dirt, 0.45f);   // SENTIER_LARGE: wide trail
    style(6, 5.0f, dirt, 0.7f);    // CHEMIN_DOUBLE: double-rut track
    style(7, 5.0f, dirt, 0.85f);   // FORET_TRES_PETIT
    style(8, 6.0f, dirt, 0.9f);    // FORET_PETIT: forest road
    style(9, 8.0f, dirt, 0.95f);   // FORET_LARGE
    style(10, 6.0f, textureOf(0x2d), 1);   // ROUTE_SIMPLE_NO_LINES
    style(11, 9.0f, textureOf(0x2c), 1);   // ROUTE_SIMPLE: two-lane tarmac
    style(12, 12.0f, textureOf(0x2a), 1);  // ROUTE_DOUBLE
    style(13, 8.0f, dirt, 0.9f);           // ATV_DIRT
    style(15, 12.0f, textureOf(0x2d), 1);  // ATV_TARMAC
    style(16, 8.0f, dirt, 0.9f);           // ATV_FLAT
    style(18, 30.0f, textureOf(0x2b), 1);  // HIGHWAY
    style(19, 30.0f, textureOf(0x2e), 1);  // RUNWAY
    style(20, 8.0f, dirt, 0.9f);           // ATV_BUMP

    const DpcFile::Entry* re = world.find(roadId);
    const auto body = re ? world.body(*re) : std::nullopt;
    if (!body) throw std::runtime_error("GwRoad_Z not found");
    const std::vector<uint8_t>& b = *body;
    // GwRoad_Z: u32 n, f32 bounds[4], then n records {u8 type, u16 count, count x 5-byte
    // big-endian (x: signed 20 bits, z: signed 20 bits) in quarter metres}.
    const uint32_t n = rd32(b, 0);
    p = 20;
    for (uint32_t i = 0; i < n; ++i) {
        if (p + 3 > b.size()) throw std::runtime_error("GwRoad_Z record overruns");
        RoadNetwork::Line line;
        line.type = b[p] & 0x7F;
        const uint16_t count = uint16_t(b[p + 1] | (b[p + 2] << 8));
        p += 3;
        if (p + size_t(count) * 5 > b.size()) throw std::runtime_error("GwRoad_Z points overrun");
        for (uint16_t k = 0; k < count; ++k, p += 5) {
            const uint64_t v = (uint64_t(b[p]) << 32) | (uint64_t(b[p + 1]) << 24) | (uint64_t(b[p + 2]) << 16) | (uint64_t(b[p + 3]) << 8) | b[p + 4];
            int32_t a = int32_t(v >> 20), c = int32_t(v & 0xFFFFF);
            if (a & 0x80000) a -= 1 << 20;
            if (c & 0x80000) c -= 1 << 20;
            line.points.push_back({a * kRoadUnit, c * kRoadUnit});
        }
        if (line.type >= net.styles.size() || net.styles[line.type].width <= 0 || line.points.size() < 2) continue;
        const auto index = uint32_t(net.lines.size());
        std::set<uint64_t> touched;
        for (const auto& pt : line.points)
            touched.insert(RoadNetwork::cellKey(int32_t(std::floor(pt[0] / RoadNetwork::kCell)), int32_t(std::floor(pt[1] / RoadNetwork::kCell))));
        for (uint64_t key : touched) net.cells[key].push_back(index);
        // Segment grid for roadSurfaceAt: each segment is walked every 16 m and every cell
        // within kReach of the walk gets it.
        const auto& pts = line.points;
        for (size_t k = 0; k + 1 < pts.size(); ++k) {
            const float dx = pts[k + 1][0] - pts[k][0], dz = pts[k + 1][1] - pts[k][1];
            const int steps = std::max(1, int(std::ceil(std::hypot(dx, dz) / 16.0f)));
            std::set<uint64_t> cells;
            for (int s = 0; s <= steps; ++s) {
                const float px = pts[k][0] + dx * float(s) / float(steps), pz = pts[k][1] + dz * float(s) / float(steps);
                const int32_t x0 = int32_t(std::floor((px - RoadNetwork::kReach) / RoadNetwork::kSegCell)), x1 = int32_t(std::floor((px + RoadNetwork::kReach) / RoadNetwork::kSegCell));
                const int32_t z0 = int32_t(std::floor((pz - RoadNetwork::kReach) / RoadNetwork::kSegCell)), z1 = int32_t(std::floor((pz + RoadNetwork::kReach) / RoadNetwork::kSegCell));
                for (int32_t cx = x0; cx <= x1; ++cx)
                    for (int32_t cz = z0; cz <= z1; ++cz) cells.insert(RoadNetwork::cellKey(cx, cz));
            }
            for (uint64_t key : cells) net.segCells[key].push_back((uint64_t(index) << 20) | uint64_t(k));
        }
        net.lines.push_back(std::move(line));
    }
    return net;
}

bool isTarmacRoad(int t) { return t == 10 || t == 11 || t == 12 || t == 15 || t == 18 || t == 19; }

int roadSurfaceAt(const RoadNetwork& net, double x, double z) {
    const auto it = net.segCells.find(RoadNetwork::cellKey(int32_t(std::floor(x / RoadNetwork::kSegCell)), int32_t(std::floor(z / RoadNetwork::kSegCell))));
    if (it == net.segCells.end()) return -1;
    int result = -1;
    for (uint64_t ref : it->second) {
        const RoadNetwork::Line& line = net.lines[size_t(ref >> 20)];
        const size_t k = size_t(ref & 0xFFFFF);
        const double ax = line.points[k][0], az = line.points[k][1];
        const double bx = line.points[k + 1][0] - ax, bz = line.points[k + 1][1] - az;
        const double len2 = bx * bx + bz * bz;
        const double t = len2 > 0 ? std::clamp(((x - ax) * bx + (z - az) * bz) / len2, 0.0, 1.0) : 0.0;
        const double d = std::hypot(x - (ax + bx * t), z - (az + bz * t));
        if (d > 0.5 * double(net.styles[line.type].width)) continue;
        if (isTarmacRoad(line.type)) return 11;
        result = 13;
    }
    return result;
}

}  // namespace fuel
