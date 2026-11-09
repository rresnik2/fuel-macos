// SplineGraph_Z bodies (see spline_graph.hpp).
#include "core/spline_graph.hpp"

#include <cstring>

namespace fuel {

std::shared_ptr<const SplineGraph> parseSplineGraph(const std::vector<uint8_t>& b) {
    size_t o = 0;
    auto u32 = [&](uint32_t& v) {
        if (o + 4 > b.size()) return false;
        std::memcpy(&v, b.data() + o, 4), o += 4;
        return true;
    };
    auto f32s = [&](float* v, size_t n) {
        if (o + 4 * n > b.size()) return false;
        std::memcpy(v, b.data() + o, 4 * n), o += 4 * n;
        return true;
    };
    auto g = std::make_shared<SplineGraph>();
    uint32_t n = 0;
    if (!u32(n) || n > 100000) return nullptr;
    g->points.resize(n);
    for (auto& p : g->points)
        if (!f32s(p.data(), 3)) return nullptr;
    if (!u32(n) || n > 100000) return nullptr;
    g->edges.resize(n);
    for (auto& e : g->edges) {
        if (o + 0xf0 > b.size()) return nullptr;
        uint16_t h[4];
        std::memcpy(h, b.data() + o, 8);
        e.a = h[0], e.b = h[1], e.c = h[2], e.d = h[3];
        std::memcpy(&e.flags, b.data() + o + 8, 4);
        std::memcpy(&e.len, b.data() + o + 12, 4);
        o += 16;
        for (auto& s : e.seg) {
            float v[7];
            f32s(v, 7);
            s.p0 = {v[0], v[1], v[2]}, s.p1 = {v[3], v[4], v[5]}, s.len = v[6];
        }
        if (e.a >= g->points.size() || e.b >= g->points.size()) return nullptr;
    }
    return g;
}

}  // namespace fuel
