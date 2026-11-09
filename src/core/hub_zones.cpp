#include "hub_zones.hpp"

#include "dpc.hpp"

#include <cmath>
#include <cstring>
#include <deque>

namespace fuel {
namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

struct Reader {
    const std::vector<uint8_t>& b;
    size_t p = 0;
    bool ok = true;
    uint32_t u32() {
        if (p + 4 > b.size()) return ok = false, 0;
        uint32_t v;
        std::memcpy(&v, &b[p], 4);
        p += 4;
        return v;
    }
    float f32() {
        const uint32_t u = u32();
        float v;
        std::memcpy(&v, &u, 4);
        return v;
    }
    void skip(size_t n) {
        if (p + n > b.size()) ok = false, p = b.size();
        else p += n;
    }
};

}  // namespace

HubZones loadHubZones(const DpcFile& world) {
    HubZones out;
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) return out;
    const DpcFile::Entry* e = world.find(gens.front());
    const auto body = e ? world.body(*e) : std::nullopt;
    if (!body) return out;
    // GenWorld_Z body (tools/terrain_extract.py parse_genworld_head): 3 ids, maps, two id lists, u32, groups, 0x83-byte
    // flat surfaces, 0x40-byte objects, 0x30-byte records, then the zone tail (0x77c690):
    //   u32 nP, nP x {f32 x, f32 z}; u32 nE, nE x {u32 a, u32 b}; u32 nZ, nZ x {char name[0x20], u32 n, n x u32 edge}.
    Reader r{*body};
    r.skip(12);
    for (int list = 0; list < 3; ++list) r.skip(size_t(r.u32()) * 4);
    r.skip(4);
    for (uint32_t g = r.u32(); g > 0 && r.ok; --g) {
        r.skip(r.u32());
        r.skip(4);
        r.skip(size_t(r.u32()) * 4);
    }
    r.skip(size_t(r.u32()) * 0x83);
    r.skip(size_t(r.u32()) * 0x40);
    r.skip(size_t(r.u32()) * 0x30);
    const uint32_t nP = r.u32();
    if (!r.ok || nP > 100000) return {};
    for (uint32_t i = 0; i < nP; ++i) {
        const float x = r.f32(), z = r.f32();
        out.points.push_back({x, z});
    }
    const uint32_t nE = r.u32();
    if (!r.ok || nE > 100000) return {};
    std::vector<std::array<int, 2>> edges;
    for (uint32_t i = 0; i < nE; ++i) {
        const int a = int(r.u32()), b = int(r.u32());
        edges.push_back({a, b});
    }
    const uint32_t nZ = r.u32();
    if (!r.ok || nZ > 1000) return {};
    for (uint32_t i = 0; i < nZ && r.ok; ++i) {
        HubZone z;
        if (r.p + 0x20 > body->size()) return {};
        z.name.assign(reinterpret_cast<const char*>(&(*body)[r.p]), strnlen(reinterpret_cast<const char*>(&(*body)[r.p]), 0x20));
        r.skip(0x20);
        for (uint32_t n = r.u32(); n > 0 && r.ok; --n) {
            const uint32_t k = r.u32();
            if (k >= edges.size() || edges[k][0] < 0 || edges[k][1] < 0 || size_t(edges[k][0]) >= nP || size_t(edges[k][1]) >= nP) return {};
            z.edges.push_back(edges[k]);
        }
        out.zones.push_back(std::move(z));
    }
    if (!r.ok) return {};
    // Zone_Normalize 0x6ee8a0: chain the edges into a ring by matching end points (|d| < 1e-6, 0x995fc4).
    auto same = [&](const std::array<float, 2>& a, const std::array<float, 2>& b) {
        return std::fabs(a[0] - b[0]) < 1e-6f && std::fabs(a[1] - b[1]) < 1e-6f;
    };
    for (HubZone& z : out.zones) {
        if (z.edges.empty()) continue;
        std::deque<std::array<float, 2>> ring{out.points[size_t(z.edges[0][0])], out.points[size_t(z.edges[0][1])]};
        std::vector<std::array<int, 2>> pending(z.edges.begin() + 1, z.edges.end());
        for (bool found = true; found;) {
            found = false;
            for (size_t i = 0; i < pending.size(); ++i) {
                const auto& A = out.points[size_t(pending[i][0])];
                const auto& B = out.points[size_t(pending[i][1])];
                if (same(A, ring.front())) ring.push_front(B);
                else if (same(B, ring.front())) ring.push_front(A);
                else if (same(A, ring.back())) ring.push_back(B);
                else if (same(B, ring.back())) ring.push_back(A);
                else continue;
                pending.erase(pending.begin() + long(i));
                found = true;
                break;
            }
        }
        ring.pop_back();  // 0x657c60: the closing duplicate
        z.ring.assign(ring.begin(), ring.end());
        // Zone_Normalize_All 0x6eead0: bounds clamped to +-65536.
        z.minX = z.minZ = 65536, z.maxX = z.maxZ = -65536;
        for (const auto& p : z.ring) {
            z.minX = std::max(-65536.0f, std::min(z.minX, p[0])), z.maxX = std::min(65536.0f, std::max(z.maxX, p[0]));
            z.minZ = std::max(-65536.0f, std::min(z.minZ, p[1])), z.maxZ = std::min(65536.0f, std::max(z.maxZ, p[1]));
        }
    }
    return out;
}

int HubZones::find(float x, float z) const {
    for (size_t i = 0; i < zones.size(); ++i) {
        const HubZone& h = zones[i];
        if (h.ring.size() < 3 || x < h.minX || x > h.maxX || z < h.minZ || z > h.maxZ) continue;
        bool in = false;  // [port] even-odd point in polygon (0x6ee810's test not re-read)
        for (size_t a = 0, b = h.ring.size() - 1; a < h.ring.size(); b = a++) {
            const auto& P = h.ring[a];
            const auto& Q = h.ring[b];
            if ((P[1] > z) != (Q[1] > z) && x < (Q[0] - P[0]) * (z - P[1]) / (Q[1] - P[1]) + P[0]) in = !in;
        }
        if (in) return int(i);
    }
    return -1;
}

// 0x60a7c0: i = 0; j = i + 1, k = j + 1 (wrapping over the remaining vertices); convex when
// cross = (Pk.y - Pj.y)(Pj.x - Pi.x) - (Pk.x - Pj.x)(Pj.y - Pi.y) <= 0, the test inverted for the other orientation
// (shoelace sign, 0x7739a0); an ear (no other remaining vertex inside, 0x7652b0) emits (i, j, k) and drops j; stops
// after n tries without an ear.
std::vector<std::array<int, 3>> earClip(const std::vector<std::array<float, 2>>& ring) {
    std::vector<std::array<int, 3>> tris;
    std::vector<int> idx;
    for (int i = 0; i < int(ring.size()); ++i) idx.push_back(i);
    double area = 0;
    for (size_t a = 0, b = ring.size() - 1; a < ring.size(); b = a++) area += double(ring[b][0]) * ring[a][1] - double(ring[a][0]) * ring[b][1];
    const bool flip = area >= 0;
    auto inside = [&](int p, int a, int b, int c) {
        auto s = [&](int u, int v, int w) {
            return (ring[size_t(v)][0] - ring[size_t(u)][0]) * (ring[size_t(w)][1] - ring[size_t(u)][1]) -
                   (ring[size_t(v)][1] - ring[size_t(u)][1]) * (ring[size_t(w)][0] - ring[size_t(u)][0]);
        };
        const float d1 = s(a, b, p), d2 = s(b, c, p), d3 = s(c, a, p);
        const bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(neg && pos);
    };
    size_t i = 0;
    int fails = 0;
    while (idx.size() >= 3 && fails < int(idx.size())) {
        const size_t j = (i + 1) % idx.size(), k = (j + 1) % idx.size();
        const auto& Pi = ring[size_t(idx[i])];
        const auto& Pj = ring[size_t(idx[j])];
        const auto& Pk = ring[size_t(idx[k])];
        const float cross = (Pk[1] - Pj[1]) * (Pj[0] - Pi[0]) - (Pk[0] - Pj[0]) * (Pj[1] - Pi[1]);
        bool ear = flip ? cross >= 0 : cross <= 0;
        if (ear)
            for (size_t m = 0; m < idx.size() && ear; ++m)
                if (m != i && m != j && m != k && inside(idx[m], idx[i], idx[j], idx[k])) ear = false;
        if (ear) {
            tris.push_back({idx[i], idx[j], idx[k]});
            idx.erase(idx.begin() + long(j));
            if (i >= idx.size()) i = 0;
            fails = 0;
            continue;
        }
        i = (i + 1) % idx.size();
        ++fails;
    }
    return tris;
}

}  // namespace fuel
