// SplineGraph_Z (class 71e0c01c): the authored racing lanes of a world course (P43, notes/p43-water-ai.md).
// Body (USA1.DPC, observed bytes; the edge record is also FUEL's run-time layout, 0xf0 bytes, read by
// 0x625fd0 / 0x63df70 / 0x6256f0 / 0x626a30): u32 point count, points (x, y, z); u32 edge count, edges:
// u16 end points a, b and Bezier controls c, d; u32 flags (+8; the AI's lane tags 8 / 0x10 / 0x20 are set at
// run time); f32 length (+0xc); 8 segments {p0 xyz, p1 xyz, length} (+0x10, 0x1c each; end point = +0xe0).
// A course's graph holds three disjoint loops: the left, middle and right lanes.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace fuel {

struct SplineGraph {
    struct Segment {
        std::array<float, 3> p0{}, p1{};
        float len = 0;
    };
    struct Edge {
        uint16_t a = 0, b = 0, c = 0, d = 0;
        uint32_t flags = 0;
        float len = 0;
        std::array<Segment, 8> seg{};
        const std::array<float, 3>& start() const { return seg[0].p0; }  // +0x10
        const std::array<float, 3>& end() const { return seg[7].p1; }    // +0xe0
    };
    std::vector<std::array<float, 3>> points;
    std::vector<Edge> edges;
};

constexpr uint32_t kSplineGraphClass = 0x71e0c01c;

// Parses a SplineGraph_Z body; nullptr when it does not hold the layout above.
std::shared_ptr<const SplineGraph> parseSplineGraph(const std::vector<uint8_t>& body);

}  // namespace fuel
