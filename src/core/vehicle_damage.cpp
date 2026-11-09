// FUEL's visual vehicle damage (P85): see vehicle_damage.hpp.
#include "vehicle_damage.hpp"

#include "genworld_noise.hpp"
#include "resources.hpp"

#include <cmath>
#include <cstring>

namespace fuel::damage {

Tables loadTables(const DpcFile& anyDpc) { return loadTables(engineExePath(anyDpc)); }

Tables loadTables(const std::string& exe) {
    Tables t;
    const std::vector<float> d = readEngineFloats(exe, 0xa057a8, 30);
    const std::vector<float> r = readEngineFloats(exe, 0xa060e8, 30);
    const std::vector<float> g = readEngineFloats(exe, 0xa06308, 3);
    if (d.size() != 30 || r.size() != 30 || g.size() != 3) return t;
    std::memcpy(t.dirs.data(), d.data(), sizeof t.dirs);
    std::memcpy(t.radius.data(), r.data(), sizeof t.radius);
    std::memcpy(t.norm.data(), g.data(), sizeof t.norm);
    t.ok = true;
    return t;
}

bool slotsChanged(const float prev[10], const float cur[10]) {
    for (int i = 0; i < 10; ++i)
        if (std::fabs(cur[i] - prev[i]) > 0.01f) return true;  // [0x9dbf9c] = 0.01; equal counts as unchanged
    return false;
}

Sphere slotSphere(const Tables& t, int slot, const std::array<float, 3>& mn, const std::array<float, 3>& mx, float v) {
    // asm 0x5c77e7..0x5c7853: centre (min + max) x 0.5 [0x9960c0], half extents (max - min) x 0.5.
    const float h = 0.5f;
    const float c[3] = {(mn[0] + mx[0]) * h, (mn[1] + mx[1]) * h, (mn[2] + mx[2]) * h};
    const float e[3] = {(mx[0] - mn[0]) * h, (mx[1] - mn[1]) * h, (mx[2] - mn[2]) * h};
    const float* D = &t.dirs[size_t(slot) * 3];
    const float* T = &t.radius[size_t(slot) * 3];
    Sphere s;
    for (int k = 0; k < 3; ++k) s.c[size_t(k)] = D[k] * e[k] + c[k];
    // asm 0x5c790f..0x5c7967: (T.z hz + T.y hy + T.x hx) v / (T.y G.y + T.z G.z + T.x G.x), in x87 precision
    const long double num = (static_cast<long double>(T[2]) * e[2] + static_cast<long double>(T[1]) * e[1]) + static_cast<long double>(T[0]) * e[0];
    const long double den = (static_cast<long double>(T[1]) * t.norm[1] + static_cast<long double>(T[2]) * t.norm[2]) +
                            static_cast<long double>(T[0]) * t.norm[0];
    s.r = float(num * v / den);
    return s;
}

namespace {

// 0x7cbda0: squared distance from a point to a node's box (0 inside).
long double boxDist2(const float p[3], const float* node) {
    long double d = 0;
    for (int k = 0; k < 3; ++k) {
        const float lo = node[k], hi = node[4 + k];
        if (p[k] < lo) d += (static_cast<long double>(p[k]) - lo) * (static_cast<long double>(p[k]) - lo);
        else if (hi < p[k]) d += (static_cast<long double>(p[k]) - hi) * (static_cast<long double>(p[k]) - hi);
    }
    return d;
}

}  // namespace

float morphDamage(const Mesh& m, const Sphere& sphere, std::vector<float>& vb) {
    // [0xa67434] (a name CRC the loop compares with target 0, a FUEL bug: it never steps) is never written: 0, no test.
    if (m.morphs.empty()) return -1.0f;  // [0xa05224] = -1
    const size_t nodes = m.morphNodes.size() / 0x20;
    if (nodes == 0) return 0.0f;
    const Mesh::MorphTarget& target = m.morphs.front();
    const float p[3] = {sphere.c[0], sphere.c[1], sphere.c[2]};
    const long double r2 = static_cast<long double>(sphere.r) * sphere.r;  // the node test: x87
    const float r2f = sphere.r * sphere.r;                                  // the point test: stored float (+0x98)
    size_t hits = 0;
    std::vector<uint16_t> stack{1};  // 1-based, the root first
    while (!stack.empty()) {
        const uint16_t idx = stack.back();
        stack.pop_back();
        if (idx == 0 || idx > nodes) continue;
        const uint8_t* rec = &m.morphNodes[size_t(idx - 1) * 0x20];
        float box[8];
        std::memcpy(box, rec, 0x20);
        if (!(boxDist2(p, box) <= r2)) continue;
        uint16_t child[2], range[2];
        std::memcpy(child, rec + 0xc, 4);
        std::memcpy(range, rec + 0x1c, 4);
        if (child[0] != 0 || child[1] != 0) {
            stack.push_back(child[0]);
            stack.push_back(child[1]);
            continue;
        }
        for (uint32_t k = range[0]; k < uint32_t(range[0]) + range[1] && 2 * k + 1 < m.morphPairs.size(); ++k) {
            const uint16_t point = m.morphPairs[2 * k], group = m.morphPairs[2 * k + 1];
            if (size_t(point) * 3 + 2 >= m.collisionVerts.size()) continue;
            const float dx = float(m.collisionVerts[size_t(point) * 3]) * 0.0009765625f - p[0];
            const float dy = float(m.collisionVerts[size_t(point) * 3 + 1]) * 0.0009765625f - p[1];
            const float dz = float(m.collisionVerts[size_t(point) * 3 + 2]) * 0.0009765625f - p[2];
            if (!(dx * dx + dz * dz + dy * dy < r2f)) continue;
            if (vb.empty()) vb.assign(target.vertexDelta.size() * 3, 0.0f);  // the instance's morph VB, zeroed
            if (size_t(group) + 1 >= m.morphGroups.size()) continue;
            const uint16_t delta = m.morphGroups[group], n = m.morphGroups[size_t(group) + 1];
            if (size_t(delta) * 4 + 2 >= target.deltas.size()) continue;
            const float d[3] = {float(target.deltas[size_t(delta) * 4]) * 0.00024414062f, float(target.deltas[size_t(delta) * 4 + 1]) * 0.00024414062f,
                                float(target.deltas[size_t(delta) * 4 + 2]) * 0.00024414062f};
            for (uint16_t j = 0; j < n && size_t(group) + 2 + j < m.morphGroups.size(); ++j) {
                const uint16_t v = m.morphGroups[size_t(group) + 2 + j];
                if (size_t(v) * 3 + 2 < vb.size()) std::memcpy(&vb[size_t(v) * 3], d, sizeof d);
            }
            ++hits;
        }
    }
    return hits != 0 ? 1.0f : 0.0f;
}

}  // namespace fuel::damage
