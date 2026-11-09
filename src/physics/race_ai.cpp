// FUEL's AI racers: see race_ai.hpp and notes/p29-ai.md. Addresses are FUEL.exe (image base 0x400000).
#include "race_ai.hpp"
#include "core/road_graph.hpp"
#include "core/spline_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>

namespace fuel {

namespace {

constexpr float kPi = 3.14159265f;

float hyp(float x, float z) { return std::sqrt(x * x + z * z); }

// ---- FUEL.exe reading (tables only; no code is run) ----------------------------------------------

struct Exe {
    std::vector<uint8_t> bytes;
    struct Section {
        uint32_t va, size, raw;
    };
    std::vector<Section> sections;
    uint32_t u32(size_t o) const {
        if (o + 4 > bytes.size()) throw std::runtime_error("read past the end of FUEL.exe");
        return uint32_t(bytes[o]) | uint32_t(bytes[o + 1]) << 8 | uint32_t(bytes[o + 2]) << 16 | uint32_t(bytes[o + 3]) << 24;
    }
    size_t offset(uint32_t va) const {
        for (const Section& s : sections)
            if (va >= s.va && va < s.va + s.size) return size_t(va - s.va) + s.raw;
        throw std::runtime_error("address not in FUEL.exe");
    }
    float f32(uint32_t va) const {
        const uint32_t b = u32(offset(va));
        float f;
        std::memcpy(&f, &b, 4);
        return f;
    }
    explicit Exe(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("cannot open " + path);
        bytes.assign(std::istreambuf_iterator<char>(f), {});
        if (bytes.size() < 0x400 || bytes[0] != 'M' || bytes[1] != 'Z') throw std::runtime_error("not a PE file");
        const size_t pe = u32(0x3c);
        const uint32_t base = u32(pe + 24 + 28);
        const unsigned n = unsigned(bytes[pe + 6]) | unsigned(bytes[pe + 7]) << 8;
        const unsigned opt = unsigned(bytes[pe + 20]) | unsigned(bytes[pe + 21]) << 8;
        for (unsigned k = 0; k < n; ++k) {
            const size_t sh = pe + 24 + opt + 40 * k;
            sections.push_back({base + u32(sh + 12), std::max(u32(sh + 8), u32(sh + 16)), u32(sh + 20)});
        }
    }
};

}  // namespace

// IA_RacerInitCornerTable 0x61e540: a run of `fld dword [imm32]` (D9 05) and `fst/fstp dword [eax+disp32]`
// (D9 90 / D9 98) up to `ret`. The stored values are the corner speeds; the code is only parsed.
std::optional<AiTables> loadAiTables(const std::string& fuelExe, std::string* error) {
    static std::mutex m;
    static std::map<std::string, std::pair<std::optional<AiTables>, std::string>> cache;
    std::lock_guard<std::mutex> lock(m);
    auto it = cache.find(fuelExe);
    if (it == cache.end()) {
        std::pair<std::optional<AiTables>, std::string> res;
        try {
            const Exe exe(fuelExe);
            size_t o = exe.offset(0x61e540);
            AiTables t;
            bool have[20] = {};
            float cur = std::numeric_limits<float>::quiet_NaN();
            for (int n = 0; n < 64; ++n) {
                const uint8_t a = exe.bytes.at(o), b = exe.bytes.at(o + 1);
                if (a == 0xc3) break;
                if (a != 0xd9 || (b != 0x05 && b != 0x90 && b != 0x98)) throw std::runtime_error("unexpected instruction at 0x61e540");
                const uint32_t imm = exe.u32(o + 2);
                o += 6;
                if (b == 0x05) {
                    cur = exe.f32(imm);
                    continue;
                }
                if (imm < 0x10b0 || imm > 0x10fc || (imm & 3)) throw std::runtime_error("unexpected store at 0x61e540");
                t.cornerSpeed[(imm - 0x10b0) / 4] = cur;
                have[(imm - 0x10b0) / 4] = true;
            }
            for (int k = 0; k < 20; ++k) {
                const float v = t.cornerSpeed[size_t(k)];
                if (!have[k] || !(v > 0 && v < 150) || (k > 0 && v < t.cornerSpeed[size_t(k - 1)]))
                    throw std::runtime_error("corner speed table at 0x61e540 is not the version the port knows");
            }
            for (int row = 0; row < 6; ++row)
                for (int col = 0; col < 2; ++col) {
                    const uint32_t a = 0xa02c88 + uint32_t(row) * 0x40 + uint32_t(col) * 0x20;
                    for (int k = 0; k < 6; ++k) {
                        const float v = exe.f32(a + uint32_t(k) * 4);
                        if (!(v > 0 && v < 10)) throw std::runtime_error("road cost table at 0xa02c88 is not the version the port knows");
                        t.roadSpeed[size_t(row)][size_t(col)][size_t(k)] = v;
                    }
                    const uint8_t f = exe.bytes.at(exe.offset(a + 0x1c));
                    if (f > 1) throw std::runtime_error("road cost table at 0xa02c88 is not the version the port knows");
                    t.roadForbid3[size_t(row)][size_t(col)] = f != 0;
                }
            res.first = t;
        } catch (const std::exception& e) {
            res.second = std::string("AI tables from FUEL.exe: ") + e.what();
        }
        it = cache.emplace(fuelExe, std::move(res)).first;
    }
    if (error) *error = it->second.second;
    return it->second.first;
}

// ---- the line --------------------------------------------------------------------------------

void RaceLine::build(const std::array<float, 3>& start, const std::vector<std::array<float, 3>>& targets) {
    points.clear();
    along.clear();
    points.push_back(start);
    points.insert(points.end(), targets.begin(), targets.end());
    float total = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0) total += std::hypot(points[i][0] - points[i - 1][0], points[i][2] - points[i - 1][2]);
        along.push_back(total);
    }
    // Plain blocks (IAPathBlock, half-width +0x18 = 30 m) gate to gate.
    path.clear(), pathAlong.clear(), pathHalf.clear(), gateAlong.clear();
    for (size_t i = 0; i < points.size(); ++i) {
        path.push_back({points[i][0], points[i][2]});
        pathAlong.push_back(along[i]);
        pathHalf.push_back(30.0f);
        gateAlong.push_back(along[i]);
    }
}

namespace {
struct RoadSnap {
    bool ok = false;
    size_t line = 0, seg = 0;
    float t = 0, x = 0, z = 0, d = 1e30f;
};
}  // namespace

void RaceLine::routeOnRoads(const RoadNetwork& roads, const std::function<float(int)>& flatHalf) {
    if (points.size() < 2) return;
    // The roads around the course (500 m margin), as a graph of their vertices (1 m keys); a line's
    // end within 6 m of another line's vertex joins it (junctions).
    float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
    for (const auto& p : points) x0 = std::min(x0, p[0]), x1 = std::max(x1, p[0]), z0 = std::min(z0, p[2]), z1 = std::max(z1, p[2]);
    x0 -= 500, z0 -= 500, x1 += 500, z1 += 500;
    std::vector<size_t> lines;
    for (size_t li = 0; li < roads.lines.size(); ++li) {
        const auto& L = roads.lines[li];
        if (L.points.size() < 2 || flatHalf(L.type) < 0) continue;
        bool in = false;
        for (const auto& q : L.points) in = in || (q[0] >= x0 && q[0] <= x1 && q[1] >= z0 && q[1] <= z1);
        if (in) lines.push_back(li);
    }
    if (lines.empty()) return;
    std::map<std::pair<int64_t, int64_t>, int> keyOf;
    std::vector<std::array<float, 2>> node;
    std::vector<std::vector<std::pair<int, float>>> adj;
    auto nodeAt = [&](float x, float z) {
        const auto k = std::make_pair(int64_t(std::llround(x)), int64_t(std::llround(z)));
        auto it = keyOf.find(k);
        if (it != keyOf.end()) return it->second;
        const int id = int(node.size());
        keyOf.emplace(k, id);
        node.push_back({x, z});
        adj.emplace_back();
        return id;
    };
    std::map<size_t, std::vector<int>> vIds;  // line -> node per vertex
    for (size_t li : lines) {
        const auto& L = roads.lines[li];
        auto& ids = vIds[li];
        for (size_t k = 0; k < L.points.size(); ++k) ids.push_back(nodeAt(L.points[k][0], L.points[k][1]));
        for (size_t k = 0; k + 1 < L.points.size(); ++k) {
            const float w = std::hypot(L.points[k + 1][0] - L.points[k][0], L.points[k + 1][1] - L.points[k][1]);
            adj[size_t(ids[k])].push_back({ids[k + 1], w});
            adj[size_t(ids[k + 1])].push_back({ids[k], w});
        }
    }
    {
        // Junctions: line ends near other vertices (a 32 m grid of the vertices).
        std::map<std::pair<int64_t, int64_t>, std::vector<int>> grid;
        for (size_t n = 0; n < node.size(); ++n) grid[{int64_t(std::floor(node[n][0] / 32)), int64_t(std::floor(node[n][1] / 32))}].push_back(int(n));
        for (size_t li : lines) {
            const auto& ids = vIds[li];
            for (int e : {ids.front(), ids.back()}) {
                const auto& p = node[size_t(e)];
                const int64_t cx = int64_t(std::floor(p[0] / 32)), cz = int64_t(std::floor(p[1] / 32));
                for (int64_t dx = -1; dx <= 1; ++dx)
                    for (int64_t dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find({cx + dx, cz + dz});
                        if (it == grid.end()) continue;
                        for (int o : it->second) {
                            if (o == e) continue;
                            const float d = std::hypot(node[size_t(o)][0] - p[0], node[size_t(o)][1] - p[1]);
                            if (d < 6.0f) adj[size_t(e)].push_back({o, d}), adj[size_t(o)].push_back({e, d});
                        }
                    }
            }
        }
    }
    // Snap a point to the nearest routable road (0x627400 / 0x6d19f0): within 100 m, or up to 300 m
    // when the road lies ahead (direction within 45 degrees, the 0.707 test).
    auto snap = [&](float x, float z, float dx, float dz) {
        RoadSnap best;
        for (size_t li : lines) {
            const auto& L = roads.lines[li];
            for (size_t k = 0; k + 1 < L.points.size(); ++k) {
                const float ax = L.points[k][0], az = L.points[k][1];
                const float bx = L.points[k + 1][0] - ax, bz = L.points[k + 1][1] - az;
                const float l2 = bx * bx + bz * bz;
                const float t = l2 > 0 ? std::clamp(((x - ax) * bx + (z - az) * bz) / l2, 0.0f, 1.0f) : 0.0f;
                const float px = ax + bx * t, pz = az + bz * t;
                const float d = std::hypot(x - px, z - pz);
                if (d >= best.d) continue;
                if (d > 100.0f) {
                    if (d > 300.0f || ((px - x) * dx + (pz - z) * dz) / d < 0.707f) continue;
                }
                best = {true, li, k, t, px, pz, d};
            }
        }
        return best;
    };
    auto route = [&](const RoadSnap& a, const RoadSnap& b, std::vector<std::array<float, 2>>& out, std::vector<float>& half) {
        // Dijkstra from the snap of `a` (its segment's two vertices) to the snap of `b`.
        const int nA0 = vIds[a.line][a.seg], nA1 = vIds[a.line][a.seg + 1];
        const int nB0 = vIds[b.line][b.seg], nB1 = vIds[b.line][b.seg + 1];
        std::vector<float> dist(node.size(), 1e30f);
        std::vector<int> prev(node.size(), -1);
        using Q = std::pair<float, int>;
        std::vector<Q> heap;
        auto push = [&](int n, float d, int p) {
            if (d < dist[size_t(n)]) {
                dist[size_t(n)] = d, prev[size_t(n)] = p;
                heap.push_back({d, n});
                std::push_heap(heap.begin(), heap.end(), std::greater<Q>());
            }
        };
        push(nA0, std::hypot(node[size_t(nA0)][0] - a.x, node[size_t(nA0)][1] - a.z), -1);
        push(nA1, std::hypot(node[size_t(nA1)][0] - a.x, node[size_t(nA1)][1] - a.z), -1);
        while (!heap.empty()) {
            std::pop_heap(heap.begin(), heap.end(), std::greater<Q>());
            const Q q = heap.back();
            heap.pop_back();
            if (q.first > dist[size_t(q.second)]) continue;
            for (const auto& [n, w] : adj[size_t(q.second)]) push(n, q.first + w, q.second);
        }
        const float dB0 = dist[size_t(nB0)] + std::hypot(node[size_t(nB0)][0] - b.x, node[size_t(nB0)][1] - b.z);
        const float dB1 = dist[size_t(nB1)] + std::hypot(node[size_t(nB1)][0] - b.x, node[size_t(nB1)][1] - b.z);
        float total = std::min(dB0, dB1);
        std::vector<int> chain;
        if (a.line == b.line && a.seg == b.seg) {
            total = std::hypot(a.x - b.x, a.z - b.z);
        } else {
            if (total >= 1e29f) return -1.0f;
            for (int n = dB0 <= dB1 ? nB0 : nB1; n >= 0; n = prev[size_t(n)]) chain.push_back(n);
            std::reverse(chain.begin(), chain.end());
        }
        out.push_back({a.x, a.z});
        half.push_back(std::max(flatHalf(roads.lines[a.line].type), 2.0f));
        for (int n : chain) {
            out.push_back(node[size_t(n)]);
            half.push_back(half.back());
        }
        out.push_back({b.x, b.z});
        half.push_back(std::max(flatHalf(roads.lines[b.line].type), 2.0f));
        return total;
    };
    // Rebuild the path stretch by stretch.
    std::vector<std::array<float, 2>> P;
    std::vector<float> H, G;
    P.push_back({points[0][0], points[0][2]});
    H.push_back(30.0f);
    G.push_back(0);
    for (size_t k = 0; k + 1 < points.size(); ++k) {
        const float ax = points[k][0], az = points[k][2], bx = points[k + 1][0], bz = points[k + 1][2];
        const float L = std::hypot(bx - ax, bz - az);
        const float dx = L > 0 ? (bx - ax) / L : 0, dz = L > 0 ? (bz - az) / L : 1;
        const RoadSnap sa = snap(ax, az, dx, dz), sb = snap(bx, bz, -dx, -dz);
        std::vector<std::array<float, 2>> R;
        std::vector<float> RH;
        float rl = -1;
        if (sa.ok && sb.ok && L > 20.0f) rl = route(sa, sb, R, RH);
        // A road block that wanders far from the stretch is not taken (sanity bound of the port).
        if (rl > 0 && rl <= 2.5f * L + 200.0f) {
            if (sa.d > 10.0f) H.back() = 30.0f;  // straight block onto the road
            else P.pop_back(), H.pop_back();
            for (size_t i = 0; i < R.size(); ++i) P.push_back(R[i]), H.push_back(RH[i]);
            if (sb.d > 10.0f) H.back() = 30.0f, P.push_back({bx, bz}), H.push_back(30.0f);
            else P.back() = {bx, bz};
        } else {
            H.back() = H.back() > 0 ? 30.0f : 30.0f;
            P.push_back({bx, bz});
            H.push_back(30.0f);
        }
        // Distance along to this target.
        float acc = 0;
        for (size_t i = 1; i < P.size(); ++i) acc += std::hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        G.push_back(acc);
    }
    path = P, pathHalf = H, gateAlong = G;
    pathAlong.assign(path.size(), 0);
    for (size_t i = 1; i < path.size(); ++i) pathAlong[i] = pathAlong[i - 1] + std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
}

void RaceLine::routeOnRoadGraph(const RoadGraph& graph, const std::function<float(int)>& flatHalf, const AiTables* tables,
                            const std::function<float(float, float)>& height) {
    if (points.size() < 2) return;
    // The AI's cost (0x4ef600, row 6 col 0 = the main path, 0x617fc0); the length cost (0x6d7e70) when
    // FUEL.exe's table is not available.
    RoadGraph::CostFn cost;
    if (tables) {
        const auto c = tables->roadSpeed[5][0];
        const bool f3 = tables->roadForbid3[5][0];
        cost = [&graph, c, f3, height](uint32_t r, uint32_t off, uint32_t n, int dir) { return graph.aiCost(r, off, n, dir, c, f3, height); };
    } else {
        cost = [&graph](uint32_t r, uint32_t off, uint32_t n, int) { return graph.lengthCost(r, off, n); };
    }
    std::vector<std::array<float, 2>> P;
    std::vector<float> H, G;
    P.push_back({points[0][0], points[0][2]});
    H.push_back(30.0f);
    G.push_back(0);
    for (size_t k = 0; k + 1 < points.size(); ++k) {
        const float ax = points[k][0], az = points[k][2], bx = points[k + 1][0], bz = points[k + 1][2];
        const float L = std::hypot(bx - ax, bz - az);
        // 0x627400: both ends snap to a road within 10 m (0x6d19f0 returns d^2; d^2 < 100). Its >= 100 branch
        // (0x6d2360) can never run.
        int ra, ka, rb, kb;
        float qax, qaz, qbx, qbz;
        const bool sa = graph.nearestRoad(ax, az, 10.0f, ra, ka, qax, qaz) >= 0;
        const bool sb = graph.nearestRoad(bx, bz, 10.0f, rb, kb, qbx, qbz) >= 0;
        RoadGraph::Route route;
        std::vector<uint32_t> recOf;
        std::vector<std::array<float, 2>> R;
        // IAPathBlockRoad (0x624760): request ends are the gate points, re-snapped within 1000 m.
        if (sa && sb && graph.request(route, ax, az, bx, bz, cost)) R = graph.polyline(route, &recOf);
        if (std::getenv("FUEL_AI_PATHDUMP")) {
            std::printf("  stretch %zu: snap %d %d, %zu points, cost %.0f, types", k, int(sa), int(sb), R.size(), route.cost);
            int last = -1;
            for (size_t i = 0; i < recOf.size(); ++i)
                if (graph.rec(recOf[i]).type != last) last = graph.rec(recOf[i]).type, std::printf(" %d@(%.0f,%.0f)", last, R[i][0], R[i][1]);
            std::printf("\n");
        }
        if (R.size() >= 2) {
            P.pop_back(), H.pop_back();
            for (size_t i = 0; i < R.size(); ++i) {
                const int t = graph.rec(recOf[i]).type;
                P.push_back(R[i]);
                H.push_back(std::max(flatHalf(t), 2.0f));  // lanes = T[+0xcc] (0x624a20)
            }
            P.back() = {bx, bz};
        } else {
            P.push_back({bx, bz});
            H.push_back(30.0f);
        }
        (void)L;
        float acc = 0;
        for (size_t i = 1; i < P.size(); ++i) acc += std::hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        G.push_back(acc);
    }
    path = P, pathHalf = H, gateAlong = G;
    pathAlong.assign(path.size(), 0);
    for (size_t i = 1; i < path.size(); ++i) pathAlong[i] = pathAlong[i - 1] + std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
}


// ---- spline blocks (IAPathBlockSpline, world courses; P43) ------------------------------------

namespace {
// A crossing of a gate line with a lane edge (0x63df70 -> 0x63dda0): edge, segment, point, line parameter,
// and the edge's direction against the gate (0x6253b0: segment direction . gate direction >= 0 = forward).
struct LaneHit {
    int edge = -1, seg = 0;
    float x = 0, z = 0, t = 0;
    bool fwd = true;
};

// 0x63df70: the nearest crossing (smallest t >= 0 along a -> b) with an edge carrying `flag` (0 = any edge).
LaneHit nearestCrossing(const SplineGraph& g, const std::vector<uint32_t>& tags, float ax, float az, float bx, float bz, uint32_t flag) {
    LaneHit best;
    float bestT = 1e20f;
    const float rx = bx - ax, rz = bz - az;
    for (size_t e = 0; e < g.edges.size(); ++e) {
        if (flag != 0 && (tags[e] & flag) == 0) continue;
        for (int k = 0; k < 8; ++k) {
            const auto& sg = g.edges[e].seg[size_t(k)];
            const float sx = sg.p1[0] - sg.p0[0], sz = sg.p1[2] - sg.p0[2];
            const float den = rx * sz - rz * sx;
            if (std::fabs(den) < 1e-9f) continue;
            const float qx = sg.p0[0] - ax, qz = sg.p0[2] - az;
            const float t = (qx * sz - qz * sx) / den, u = (qx * rz - qz * rx) / den;
            if (t < 0 || t > 1 || u < 0 || u > 1) continue;
            if (t <= bestT) bestT = t, best.edge = int(e), best.seg = k, best.t = t, best.x = ax + rx * t, best.z = az + rz * t;
        }
    }
    return best;
}

// 0x625400: the three lane crossings of the gate line c +- n (W + 5), n = (-dz, dx), searched from the +n end:
// the first (lane 0x10), then from 0.3 m past it the next (0x20), then the next (8). mode 0: any edge.
bool gateCrossings(const SplineGraph& g, const std::vector<uint32_t>& tags, float cx, float cz, float dx, float dz, float W, bool mode,
                   LaneHit out[3]) {
    const float nx = -dz, nz = dx;
    float ax = cx + nx * (W + 5), az = cz + nz * (W + 5);
    const float bx = cx - nx * (W + 5), bz = cz - nz * (W + 5);
    const uint32_t flags[3] = {0x10, 0x20, 8};
    for (int i = 0; i < 3; ++i) {
        out[i] = nearestCrossing(g, tags, ax, az, bx, bz, mode ? flags[i] : 0);
        if (out[i].edge < 0) return false;
        const auto& sg = g.edges[size_t(out[i].edge)].seg[size_t(out[i].seg)];
        out[i].fwd = (sg.p1[2] - sg.p0[2]) * dz + dx * (sg.p1[0] - sg.p0[0]) >= 0;
        ax = out[i].x - nx * 0.3f, az = out[i].z - nz * 0.3f;
    }
    return true;
}

bool near01(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    const float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
    return x * x + y * y + z * z < 0.01f;
}

// 0x6256f0: from the start crossing's edge, at its far end the first edge (array order) with the lane flag
// touching it, up to 32 edges or the end crossing's edge. Returns the lane polyline from the start crossing to
// the end crossing, empty when the walk does not reach it (port: FUEL keeps the partial list).
std::vector<std::array<float, 2>> walkLane(const SplineGraph& g, const std::vector<uint32_t>& tags, uint32_t flag, const LaneHit& a, const LaneHit& b,
                                           std::vector<std::pair<int, bool>>* edges = nullptr) {
    std::vector<std::array<float, 2>> P;
    int cur = a.edge;
    bool fwd = a.fwd;
    std::vector<std::pair<int, bool>> list;
    for (int n = 0; n < 32; ++n) {
        list.push_back({cur, fwd});
        if (cur == b.edge) break;
        const std::array<float, 3> far = fwd ? g.edges[size_t(cur)].end() : g.edges[size_t(cur)].start();
        int next = -1;
        for (size_t e = 0; e < g.edges.size() && next < 0; ++e) {
            if ((tags[e] & flag) == 0 || int(e) == cur) continue;
            if (near01(g.edges[e].start(), far)) next = int(e), fwd = true;
            else if (near01(g.edges[e].end(), far)) next = int(e), fwd = false;
        }
        if (next < 0) break;
        cur = next;
    }
    if (list.back().first != b.edge) return P;
    if (edges) *edges = list;  // PK19: the block's edge list (IAPathBlockSpline +0xc0, count +0x3e; 0x6256f0)
    // The polyline: the start crossing, the segment points between, the end crossing.
    P.push_back({a.x, a.z});
    for (size_t i = 0; i < list.size(); ++i) {
        const auto& E = g.edges[size_t(list[i].first)];
        const bool f = list[i].second;
        const bool last = i + 1 == list.size();
        const int s0 = i == 0 ? a.seg : (f ? 0 : 7);
        if (f) {
            const int s1 = last ? b.seg - 1 : 7;
            if (last && i == 0 && b.seg < a.seg) return {};
            for (int k = s0; k <= s1; ++k) P.push_back({E.seg[size_t(k)].p1[0], E.seg[size_t(k)].p1[2]});
        } else {
            const int s1 = last ? b.seg + 1 : 0;
            if (last && i == 0 && b.seg > a.seg) return {};
            for (int k = s0; k >= s1; --k) P.push_back({E.seg[size_t(k)].p0[0], E.seg[size_t(k)].p0[2]});
        }
    }
    P.push_back({b.x, b.z});
    return P;
}

// Nearest crossing (from p along r) of the ray p -> p + r with a polyline; returns the parameter or -1.
float rayPolyline(float px, float pz, float rx, float rz, const std::vector<std::array<float, 2>>& L) {
    float best = -1;
    for (size_t i = 0; i + 1 < L.size(); ++i) {
        const float sx = L[i + 1][0] - L[i][0], sz = L[i + 1][1] - L[i][1];
        const float den = rx * sz - rz * sx;
        if (std::fabs(den) < 1e-9f) continue;
        const float qx = L[i][0] - px, qz = L[i][1] - pz;
        const float t = (qx * sz - qz * sx) / den, u = (qx * rz - qz * rx) / den;
        if (t < 0 || t > 1 || u < 0 || u > 1) continue;
        if (best < 0 || t < best) best = t;
    }
    return best;
}
}  // namespace

int RaceLine::routeOnSplineGraph(const SplineGraph& g, const std::vector<std::array<float, 2>>& gateDir, const std::vector<float>& gateHalf) {
    splineEdges.clear();  // PK19
    if (points.size() < 2 || gateDir.size() != points.size() || gateHalf.size() != points.size() || g.edges.empty()) return 0;
    // Lane tags (0x627590 -> 0x625f80 clears 0x38, 0x625400 mode 0 on the first gate, 0x625fd0 flood fills).
    std::vector<uint32_t> tags(g.edges.size(), 0);
    {
        LaneHit h[3];
        if (!gateCrossings(g, tags, points[0][0], points[0][2], gateDir[0][0], gateDir[0][1], gateHalf[0], false, h)) return 0;
        const uint32_t flags[3] = {0x10, 0x20, 8};
        for (int i = 0; i < 3; ++i) {
            std::vector<int> open{h[i].edge};
            tags[size_t(h[i].edge)] |= flags[i];
            while (!open.empty()) {
                const int e = open.back();
                open.pop_back();
                const auto& E = g.edges[size_t(e)];
                for (size_t f = 0; f < g.edges.size(); ++f) {
                    if (tags[f] & flags[i]) continue;
                    const auto& F = g.edges[f];
                    if (near01(F.start(), E.start()) || near01(F.start(), E.end()) || near01(F.end(), E.start()) || near01(F.end(), E.end()))
                        tags[f] |= flags[i], open.push_back(int(f));
                }
            }
        }
    }
    std::vector<std::array<float, 2>> P;
    std::vector<float> H, G;
    P.push_back({points[0][0], points[0][2]});
    H.push_back(pathHalf.empty() ? 30.0f : pathHalf[0]);
    G.push_back(0);
    int done = 0;
    for (size_t k = 0; k + 1 < points.size(); ++k) {
        // The spline block runs from gate k's exit (centre + dir) to gate k+1's entry (centre - dir, 0x623960(1));
        // its half-width is the larger of the two checkpoint blocks' (0x627590).
        const float W = std::max(gateHalf[k], gateHalf[k + 1]);
        const auto& da = gateDir[k];
        const auto& db = gateDir[k + 1];
        LaneHit ha[3], hb[3];
        std::vector<std::array<float, 2>> M, Lf, Rt;
        std::vector<std::pair<int, bool>> mid;  // PK19: the middle lane's edge list
        bool ok = gateCrossings(g, tags, points[k][0] + da[0], points[k][2] + da[1], da[0], da[1], W, true, ha) &&
                  gateCrossings(g, tags, points[k + 1][0] - db[0], points[k + 1][2] - db[1], db[0], db[1], W, true, hb);
        if (ok) {
            M = walkLane(g, tags, 0x20, ha[1], hb[1], &mid);
            Lf = walkLane(g, tags, 0x10, ha[0], hb[0]);
            Rt = walkLane(g, tags, 8, ha[2], hb[2]);
            ok = M.size() >= 2 && Lf.size() >= 2 && Rt.size() >= 2;
            if (ok) {
                // PK16 0x626400: the middle lane is cut at the sample spacing: the start crossing, then a point every
                // 10 m along the lane polyline (the carry runs across the lane's vertices), none at the end crossing
                // (the block's end gate is added below). (Before PK16 the lane's own vertices were the samples.)
                std::vector<std::array<float, 2>> C{M[0]};
                float want = 10.0f;
                for (size_t i = 0; i + 1 < M.size(); ++i) {
                    const float dx = M[i + 1][0] - M[i][0], dz = M[i + 1][1] - M[i][1], len = std::hypot(dx, dz);
                    if (len < 1e-6f) continue;
                    while (want < len) {
                        C.push_back({M[i][0] + dx * (want / len), M[i][1] + dz * (want / len)});
                        want += 10.0f;
                    }
                    want -= len;
                }
                if (C.size() < 2) C.push_back(M.back());
                M = std::move(C);
            }
        }
        if (std::getenv("FUEL_AI_PATHDUMP"))
            std::printf("  spline stretch %zu: gate (%.1f, %.1f) dir (%.2f, %.2f) half %.1f -> (%.1f, %.1f) dir (%.2f, %.2f) half %.1f: %s\n", k, points[k][0],
                        points[k][2], da[0], da[1], gateHalf[k], points[k + 1][0], points[k + 1][2], db[0], db[1], gateHalf[k + 1], ok ? "lanes" : "kept");
        if (ok) {
            // 0x626a30: per middle-lane point (normal from its neighbours, the last from its predecessor) a 200 m
            // ray to each side; 0x625900 finds the outer lane's nearest crossing unless the ray crosses the previous
            // cross-section (0x75a590: then the previous crossing is kept). The sample is the midpoint of
            // P + n |hl - P| and P - n |hr - P|, half their distance its half-width.
            std::array<float, 2> hl{ha[0].x, ha[0].z}, hr{ha[2].x, ha[2].z};   // the lanes' crossings of the gate line
            std::array<float, 2> el = hl, er = hr;                               // previous cross-section ends
            auto crosses = [](const std::array<float, 2>& a, const std::array<float, 2>& b, const std::array<float, 2>& c, const std::array<float, 2>& d) {
                const float rx = b[0] - a[0], rz = b[1] - a[1], sx = d[0] - c[0], sz = d[1] - c[1];
                const float den = rx * sz - rz * sx;
                if (std::fabs(den) < 1e-9f) return false;
                const float qx = c[0] - a[0], qz = c[1] - a[1];
                const float t = (qx * sz - qz * sx) / den, u = (qx * rz - qz * rx) / den;
                return t >= 0 && t <= 1 && u >= 0 && u <= 1;
            };
            for (size_t i = 0; i < M.size(); ++i) {
                const auto& a = M[i == 0 ? 0 : i - 1];
                const auto& b = M[i + 1 < M.size() ? i + 1 : i];
                float nx = -(b[1] - a[1]), nz = b[0] - a[0];
                const float nl = std::hypot(nx, nz);
                if (nl > 1e-6f) nx /= nl, nz /= nl;
                const std::array<float, 2> p = M[i];
                const std::array<float, 2> qL{p[0] + nx * 200, p[1] + nz * 200}, qR{p[0] - nx * 200, p[1] - nz * 200};
                if (!crosses(p, qL, el, er)) {
                    const float t = rayPolyline(p[0], p[1], nx * 200, nz * 200, Lf);
                    if (t >= 0) hl = {p[0] + nx * 200 * t, p[1] + nz * 200 * t};
                }
                if (!crosses(p, qR, el, er)) {
                    const float t = rayPolyline(p[0], p[1], -nx * 200, -nz * 200, Rt);
                    if (t >= 0) hr = {p[0] - nx * 200 * t, p[1] - nz * 200 * t};
                }
                const float dl = std::hypot(hl[0] - p[0], hl[1] - p[1]), dr = std::hypot(hr[0] - p[0], hr[1] - p[1]);
                el = {p[0] + nx * dl, p[1] + nz * dl}, er = {p[0] - nx * dr, p[1] - nz * dr};
                P.push_back({(el[0] + er[0]) * 0.5f, (el[1] + er[1]) * 0.5f});
                H.push_back(std::max((dl + dr) * 0.5f, 1.0f));
            }
            P.push_back({points[k + 1][0], points[k + 1][2]});
            H.push_back(H.back());
            ++done;
            splineEdges.push_back(std::move(mid));  // PK19: a spline block (kind 5) in path order
        } else {
            // The stretch as already routed (road / plain blocks).
            const float a = k < gateAlong.size() ? gateAlong[k] : 0, b = k + 1 < gateAlong.size() ? gateAlong[k + 1] : a;
            for (size_t i = 0; i < path.size(); ++i)
                if (pathAlong[i] > a + 1e-3f && pathAlong[i] <= b + 1e-3f) P.push_back(path[i]), H.push_back(pathHalf[i]);
            if (P.back()[0] != points[k + 1][0] || P.back()[1] != points[k + 1][2]) P.push_back({points[k + 1][0], points[k + 1][2]}), H.push_back(30.0f);
        }
        float acc = 0;
        for (size_t i = 1; i < P.size(); ++i) acc += std::hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        G.push_back(acc);
    }
    path = P, pathHalf = H, gateAlong = G;
    pathAlong.assign(path.size(), 0);
    for (size_t i = 1; i < path.size(); ++i) pathAlong[i] = pathAlong[i - 1] + std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
    return done;
}

float RaceLine::progress(size_t next, double x, double z) const {
    if (next + 1 >= points.size()) return length();
    const auto& b = points[next + 1];
    const float seg = along[next + 1] - along[next];
    const float d = float(std::hypot(x - b[0], z - b[2]));
    return along[next + 1] - std::min(d, seg + 50.0f);  // a little behind the segment start still counts
}

std::array<float, 3> RaceLine::pointAhead(size_t next, double x, double z, float ahead, float* heading) const {
    if (path.size() < 2) return points.empty() ? std::array<float, 3>{float(x), 0, float(z)} : points[0];
    const size_t i = std::min(next, gateAlong.size() - 2);
    const float s = project(x, z, gateAlong[i], gateAlong[i + 1]) + ahead;
    float dx = 0, dz = 1;
    const auto p = at(s, &dx, &dz);
    if (heading) *heading = std::atan2(dx, dz);
    return p;
}

std::array<float, 3> RaceLine::at(float s, float* dx, float* dz) const {
    if (path.empty()) return {0, 0, 0};
    if (path.size() == 1) return {path[0][0], 0, path[0][1]};
    s = std::clamp(s, 0.0f, pathLength() + finishRun);
    if (s > pathLength()) {  // PK16: the finish block, along its own direction (0x627590: the last gate's dir * 100)
        const auto fd = finishDirection();
        if (dx) *dx = fd[0];
        if (dz) *dz = fd[1];
        const float t = s - pathLength();
        return {path.back()[0] + fd[0] * t, 0.0f, path.back()[1] + fd[1] * t};
    }
    size_t i = size_t(std::upper_bound(pathAlong.begin(), pathAlong.end(), s) - pathAlong.begin());
    i = i == 0 ? 0 : i - 1;
    i = std::min(i, path.size() - 2);
    size_t j = i;
    while (j + 2 < path.size() && pathAlong[j + 1] - pathAlong[j] < 1e-3f) ++j;
    const auto& p = path[j];
    const auto& q = path[j + 1];
    const float len = std::max(pathAlong[j + 1] - pathAlong[j], 1e-3f);
    const float ux = (q[0] - p[0]) / len, uz = (q[1] - p[1]) / len;
    if (dx) *dx = ux;
    if (dz) *dz = uz;
    const float u = (s - pathAlong[j]) / len;  // > 1 past the finish: straight on (the finish block)
    return {p[0] + (q[0] - p[0]) * u, 0.0f, p[1] + (q[1] - p[1]) * u};
}

float RaceLine::project(double x, double z, float lo, float hi) const {
    if (path.size() < 2) return 0;
    lo = std::max(lo, 0.0f);
    hi = std::min(hi, pathLength() + finishRun);
    float best = lo, bestD = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const float a = pathAlong[i];
        const float b = pathAlong[i + 1];
        if (b < lo || a > hi) continue;
        const float len = pathAlong[i + 1] - a;
        if (len < 1e-3f) continue;
        const float ux = (path[i + 1][0] - path[i][0]) / len, uz = (path[i + 1][1] - path[i][1]) / len;
        float t = float((x - path[i][0]) * ux + (z - path[i][1]) * uz);
        t = std::clamp(t, std::max(0.0f, lo - a), std::min(b, hi) - a);
        const float px = path[i][0] + ux * t, pz = path[i][1] + uz * t;
        const float d = float(std::hypot(x - px, z - pz));
        if (d < bestD) bestD = d, best = a + t;
    }
    if (finishRun > 0) {  // PK16: the finish block (the gate's direction)
        const float a = pathLength(), b = a + finishRun;
        if (!(b < lo || a > hi)) {
            const auto fd = finishDirection();
            const auto& e = path.back();
            float t = float((x - e[0]) * fd[0] + (z - e[1]) * fd[1]);
            t = std::clamp(t, std::max(0.0f, lo - a), std::min(b, hi) - a);
            const float px = e[0] + fd[0] * t, pz = e[1] + fd[1] * t;
            const float d = float(std::hypot(x - px, z - pz));
            if (d < bestD) bestD = d, best = a + t;
        }
    }
    return best;
}

void RaceLine::setFinishDir(float dx, float dz) {
    const float l = std::hypot(dx, dz);
    if (l < 1e-3f) return;
    finishDir = {dx / l, dz / l};
    hasFinishDir = true;
}

std::array<float, 2> RaceLine::finishDirection() const {
    if (hasFinishDir) return finishDir;
    for (size_t i = path.size(); i >= 2; --i) {  // the last path segment's direction
        const float dx = path[i - 1][0] - path[i - 2][0], dz = path[i - 1][1] - path[i - 2][1], l = std::hypot(dx, dz);
        if (l > 1e-3f) return {dx / l, dz / l};
    }
    return {0, 1};
}

// 0x624310 / 0x624490: the finish block's parking line. A = the last gate's exit (centre + dir, 0x623960), the
// block's end B = A + dir * 100 (0x627590); the target T = A + dist * dir + 6 m sideways: (-dz, dx) for an even rank,
// (dz, -dx) for an odd one (rank & 0x80000001 == 0); none for rank -1.
RaceLine RaceLine::parkingLine(int rank, float dist) const {
    RaceLine L = *this;
    if (path.size() < 2) return L;
    const auto fd = finishDirection();
    const std::array<float, 2> A{path.back()[0] + fd[0], path.back()[1] + fd[1]};
    float ox = 0, oz = 0;
    if (rank != -1) {
        if ((rank & 1) == 0) ox = -fd[1] * 6.0f, oz = fd[0] * 6.0f;
        else ox = fd[1] * 6.0f, oz = -fd[0] * 6.0f;
    }
    const std::array<float, 2> T{A[0] + fd[0] * dist + ox, A[1] + fd[1] * dist + oz};
    L.path.push_back(A), L.path.push_back(T);
    L.pathHalf.push_back(30.0f), L.pathHalf.push_back(30.0f);  // IAPathBlock default +0x18 = 30
    L.pathAlong.push_back(pathAlong.back() + 1.0f);
    L.pathAlong.push_back(L.pathAlong.back() + std::hypot(T[0] - A[0], T[1] - A[1]));
    L.finishRun = 0;
    return L;
}

float RaceLine::distanceLeft(double x, double z, float s, float straight) const {
    if (path.size() < 2) return 0.0f;
    const float total = pathLength();
    const auto fd = finishDirection();
    const auto foot = at(std::min(s, total));
    const float S1 = std::max(0.0f, total - s) + fd[0] * (foot[0] - float(x)) + fd[1] * (foot[2] - float(z));
    const float bx = path.back()[0] + fd[0] * (1.0f + 100.0f), bz = path.back()[1] + fd[1] * (1.0f + 100.0f);
    const float S2 = float(std::hypot(bx - x, bz - z));
    float v = std::max(S1, S2);
    if (straight >= 0) {
        v = std::max(v, straight);
        if (v > 20.0f * straight) v = 20.0f * straight;
    }
    return v;
}

float RaceLine::halfWidth(float s) const {
    for (size_t k = 1; k < gateAlong.size(); ++k)
        if (std::fabs(s - gateAlong[k]) <= 10.0f) return gateHalfWidth;  // IAPathBlockCheckpoint: the gate's width * 0.5
    if (path.size() < 2) return 30.0f;
    size_t i = size_t(std::upper_bound(pathAlong.begin(), pathAlong.end(), s) - pathAlong.begin());
    i = i == 0 ? 0 : i - 1;
    return pathHalf[std::min(i, pathHalf.size() - 1)];  // the block's: 30 m plain (+0x18), a road's flat half-width
}

// ---- 0x61a9d0 IA_PropagateCorridor -----------------------------------------------------------

void aiPropagateCorridor(std::vector<AiSample>& S, int idx, const std::vector<AiObstacle>& O, float ghostLat) {
    const int n = int(S.size()) - 1;  // racer +0x218
    size_t c = 0;                     // the list cursor (DAT_00a062cc chain)
    // The ghost's own sample.
    if (idx >= 0 && idx <= n) {
        AiSample& a = S[size_t(idx)];
        while (c < O.size()) {
            const AiObstacle& o = O[c];
            if (idx < o.sample) break;
            ++c;
            if (!(idx <= o.sample && o.lo <= a.latMax && a.latMin <= o.hi)) continue;
            if (o.lo <= a.latMin && a.latMax < o.hi) {
                a.latMin = o.lo, a.latMax = o.hi;
                break;
            }
            if ((o.lo + o.hi) * 0.5f <= ghostLat) a.latMin = o.hi;
            else a.latMax = o.lo;
        }
    }
    for (int i = idx + 1; i < n; ++i) {
        if (i > 0) {  // widen by at most 0.1 per metre of the previous segment
            S[size_t(i)].latMin = std::max(S[size_t(i)].latMin, S[size_t(i - 1)].latMin - S[size_t(i - 1)].seg * 0.1f);
            S[size_t(i)].latMax = std::min(S[size_t(i)].latMax, S[size_t(i - 1)].latMax + S[size_t(i - 1)].seg * 0.1f);
        }
        if (c >= O.size()) continue;
        float best = -1e6f, bLo = 0, bHi = 0;
        float hi = S[size_t(i)].latMax, lo = S[size_t(i)].latMin, cur = lo;
        while (c < O.size() && O[c].sample == i) {
            const AiObstacle& o = O[c];
            cur = lo;
            if (lo < o.hi && o.lo < hi) {
                if (o.type == 2) {  // drive into it: clamp and skip the rest of this sample's records
                    if (lo <= o.lo) cur = o.lo;
                    if (o.hi <= hi) hi = o.hi;
                    while (c < O.size() && O[c].sample == i) ++c;
                    lo = cur;
                    break;
                }
                if (o.type == 3 && best < hi - lo) bLo = std::max(o.lo, lo), bHi = std::max(o.hi, hi), best = hi - lo;
                if (lo <= o.lo) {
                    if (o.hi <= hi) {
                        const float left = o.lo - lo;
                        if (left <= hi - o.hi) {
                            if (best < left) bHi = o.lo, cur = o.hi, best = left, bLo = lo;
                        } else {
                            hi = o.lo;
                        }
                    } else {
                        hi = o.lo;
                    }
                } else {
                    cur = o.hi;
                    if (hi < o.hi) hi = o.lo;
                }
            }
            ++c;
            lo = cur;
        }
        if (hi - lo < best) lo = bLo, hi = bHi;
        if (0 < hi - lo) S[size_t(i)].latMin = lo, S[size_t(i)].latMax = hi;
    }
}

void aiBoxObstacles(const std::vector<AiSample>& S, const float p[12], const float half[3], float r, int type, float margin, float halfWidth,
                    std::vector<AiObstacle>& out) {
    // Half-axis vectors (x, z) and the centre.
    const float ax = p[0] * half[0], az = p[2] * half[0], bx = half[1] * p[4], bz = p[6] * half[1], cx = half[2] * p[8], cz = half[2] * p[10];
    const float ox = p[3], oz = p[11];
    const float w = halfWidth + margin;
    float cum = 0, prevSeg = 0;
    const int n = int(S.size()) - 1;
    auto record = [&](int k, float lo, float hi, float s0, float s1) {
        AiObstacle o;
        o.lo = lo, o.hi = hi, o.s0 = s0 + cum, o.s1 = s1 + cum, o.sample = k, o.type = type;
        if ((type == 2 || type == 3) && p[2] * S[size_t(k)].dz + S[size_t(k)].dx * p[0] < 0.7f) o.type = 0;
        out.push_back(o);
    };
    for (int k = 0; k < n; ++k) {
        cum += prevSeg;
        const AiSample& a = S[size_t(k)];
        const float seg = a.seg, dx = a.dx, dz = a.dz;
        prevSeg = seg;
        if (r < 15.0f) {
            const float rx = ox - a.x, rz = oz - a.z;
            const float al = rx * dx + rz * dz;
            const float ext = std::fabs(ax * dx + az * dz) + std::fabs(bx * dx + bz * dz) + std::fabs(cx * dx + cz * dz);
            float s0 = al - ext - 3.0f, s1 = al + ext;
            if (!(s0 <= seg && 0.0f <= s1)) continue;
            s0 = std::max(s0, 0.0f), s1 = std::min(s1, seg);
            const float la = rz * dx - rx * dz;
            const float extL = std::fabs(az * dx - ax * dz) + std::fabs(bz * dx - bx * dz) + std::fabs(cz * dx - cx * dz);
            const float lo = la - extL - w, hi = w + la + extL;
            if (lo <= a.latMax && a.latMin <= hi) record(k, lo, hi, s0, s1);
            continue;
        }
        // Large boxes: the 12 edges (corner pairs of the table at 0xa021f0), clipped to the strip [0, seg].
        float C[8][2];
        for (int i = 0; i < 8; ++i) {
            const float sa = (i & 4) ? -1.0f : 1.0f, sb = (i & 2) ? -1.0f : 1.0f, sc = (i & 1) ? -1.0f : 1.0f;
            C[i][0] = ox - a.x + sa * ax + sb * bx + sc * cx;
            C[i][1] = oz - a.z + sa * az + sb * bz + sc * cz;
        }
        static const int E[12][2] = {{0, 2}, {2, 3}, {3, 1}, {1, 0}, {4, 6}, {6, 7}, {7, 5}, {5, 4}, {0, 4}, {2, 6}, {3, 7}, {1, 5}};  // the cube's edges
        float latLo = 1e6f, latHi = -1e6f, alHi = -1e6f, alLo = 1e6f;
        for (const auto& e : E) {
            float a1 = C[e[0]][1] * dz + C[e[0]][0] * dx, a2 = C[e[1]][1] * dz + C[e[1]][0] * dx;
            if ((a1 > seg && a2 > seg) || (a1 <= 0 && a2 <= 0)) continue;
            float l1 = C[e[0]][0] * -dz + C[e[0]][1] * dx, l2 = -dz * C[e[1]][0] + C[e[1]][1] * dx;
            if ((l1 > a.latMax && l2 > a.latMax) || (l1 <= a.latMin && l2 <= a.latMin)) continue;
            if (seg < a1) {
                if (a1 - a2 < 0.001f) continue;
                l1 = (l1 - l2) * ((seg - a2) / (a1 - a2)) + l2, a1 = seg;
            } else if (a1 < 0) {
                if (a2 - a1 < 0.001f) continue;
                l1 = l2 - (a2 / (a2 - a1)) * (l2 - l1), a1 = 0;
            }
            if (a2 <= seg) {
                if (a2 < 0) {
                    if (a1 - a2 < 0.001f) continue;
                    l2 = l1 - (a1 / (a1 - a2)) * (l1 - l2), a2 = 0;
                }
            } else {
                if (a2 - a1 < 0.001f) continue;
                l2 = (l2 - l1) * ((seg - a1) / (a2 - a1)) + l1, a2 = seg;
            }
            latLo = std::min(latLo, std::min(l1, l2)), latHi = std::max(latHi, std::max(l1, l2));
            alHi = std::max(alHi, std::max(a1, a2)), alLo = std::min(alLo, std::min(a1, a2));
        }
        if (0.0001f <= latHi - latLo && 0.0001f <= alHi - alLo) {
            const float lo = latLo - w, hi = w + latHi;
            if (lo <= a.latMax && a.latMin <= hi) record(k, lo, hi, alLo, alHi);
        }
    }
}

// ---- 0x61acb0 IA_SmoothLineAndAim ------------------------------------------------------------

bool aiSmoothLineAndAim(const std::vector<AiSample>& S, int idx, float s0, const float p0[2], float speedHist, float aim[2],
                        std::vector<AiCorner>& corners, std::vector<std::array<float, 2>>* lineOut) {
    corners.clear();
    const int n = int(S.size()) - 1;  // racer+0x218: index of the last sample
    if (n < 1) return false;
    int i0 = idx + 1;
    if (i0 >= n || idx < 0) return false;
    if (S[size_t(i0)].cum - s0 < 1.0f) i0 = idx + 2;  // the next sample within 1 m: start one further
    if (i0 > n) return false;
    // Cross-sections (10 floats each on the stack: normal, middle, point, lo, hi, t, k).
    struct Rec {
        float nx, nz, mx, mz, px, pz, lo, hi, t, k;
    };
    std::vector<Rec> R;
    R.reserve(size_t(n - i0 + 1));
    float pnx = -S[size_t(i0)].dz, pnz = S[size_t(i0)].dx;  // previous cross-section's normal
    for (int i = i0; i <= n; ++i) {
        const AiSample& a = S[size_t(i - 1)];
        const AiSample& c = S[size_t(i)];
        float dx, dz;
        if (i < n) dx = S[size_t(i + 1)].x - a.x, dz = S[size_t(i + 1)].z - a.z;
        else dx = c.x - a.x, dz = c.z - a.z;
        const float l2 = dx * dx + dz * dz;
        if (l2 > 1e-6f) {
            const float l = std::sqrt(l2);
            dx /= l, dz /= l;
        }
        const float nx = -dz, nz = dx;
        float Ax = c.x + c.latMin * nx, Az = c.z + c.latMin * nz;
        float Bx = c.x + c.latMax * nx, Bz = c.z + c.latMax * nz;
        // Along-coordinates of the ends past the previous sample, on the previous direction (pnz, -pnx):
        // a cross-section reaching behind it is cut there.
        const float fa = (Ax - a.x) * pnz - (Az - a.z) * pnx;
        const float fb = (Bx - a.x) * pnz - (Bz - a.z) * pnx;
        float w;
        if (fa >= 0) {
            if (fb >= 0) {
                w = c.latMax - c.latMin;
            } else {
                const float f = -fb / (-fb + fa);
                Bx = Ax * f + (1 - f) * Bx, Bz = Bz * (1 - f) + Az * f;
                w = hyp(Ax - Bx, Az - Bz);
            }
        } else if (fb <= 0) {
            w = c.latMax - c.latMin;
        } else {
            const float f = -fa / (fb - fa);
            Ax = Bx * f + Ax * (1 - f), Az = Az * (1 - f) + Bz * f;
            w = hyp(Ax - Bx, Az - Bz);
        }
        Rec r;
        r.nx = nx, r.nz = nz;
        r.mx = (Ax + Bx) * 0.5f, r.mz = (Az + Bz) * 0.5f;
        r.px = r.mx, r.pz = r.mz;
        r.lo = -w * 0.5f, r.hi = w * 0.5f;
        r.t = 0, r.k = 0;
        R.push_back(r);
        pnx = nx, pnz = dx;
    }
    const int N = int(R.size());
    // 20 passes: curvature (sine of the turn at each point, the ghost as the point before the first),
    // then each point moves across its section by 2.5 (k_i - (k_i-1 + k_i+1) / 4), within the section.
    for (int pass = 0; pass < 20; ++pass) {
        if (N > 1) {
            float ax = R[0].px - p0[0], az = R[0].pz - p0[1], a2 = ax * ax + az * az;
            for (int j = 0; j + 1 < N; ++j) {
                const float bx = R[size_t(j + 1)].px - R[size_t(j)].px, bz = R[size_t(j + 1)].pz - R[size_t(j)].pz;
                const float b2 = bx * bx + bz * bz;
                if (a2 > 0.1f && b2 > 0.1f) R[size_t(j)].k = (bz * ax - bx * az) / std::sqrt(a2 * b2);
                ax = bx, az = bz, a2 = b2;
            }
            for (int j = 0; j + 1 < N; ++j) {
                Rec& r = R[size_t(j)];
                const float kp = j > 0 ? R[size_t(j - 1)].k : 0.0f, kn = R[size_t(j + 1)].k;
                float t = (r.k - (kn + kp) * 0.25f) * 2.5f + r.t;
                t = std::max(r.lo, std::min(t, r.hi));
                r.t = t;
                r.px = r.mx + r.nx * t, r.pz = r.mz + r.nz * t;
            }
        }
        // The last section (as the code does it: t -= k of the one before).
        Rec& r = R[size_t(N - 1)];
        float t = r.t - (N >= 2 ? R[size_t(N - 2)].k : 0.0f);
        t = std::max(r.lo, std::min(t, r.hi));
        r.t = t;
        r.px = r.mx + r.nx * t, r.pz = r.mz + r.nz * t;
    }
    if (lineOut) {
        lineOut->clear();
        for (const Rec& r : R) lineOut->push_back({r.px, r.pz});
    }
    if (N < 1) return false;
    // The aim: clamp(|v| / 2, 5, 20) metres along the smoothed line from the ghost, then that far
    // from the ghost towards it.
    const float L = std::clamp(speedHist * 0.5f, 5.0f, 20.0f);
    float acc = 0, seg = 0, qx = p0[0], qz = p0[1];
    int j = 0;
    for (; j < N; ++j) {
        const float ax = j == 0 ? p0[0] : R[size_t(j - 1)].px, az = j == 0 ? p0[1] : R[size_t(j - 1)].pz;
        seg = hyp(R[size_t(j)].px - ax, R[size_t(j)].pz - az);
        acc += seg;
        if (acc > L) break;
    }
    if (!(acc > 1)) return false;
    if (j >= N || seg <= 0.01f) {
        const int k = std::min(j, N - 1);
        qx = R[size_t(k)].px, qz = R[size_t(k)].pz;
        if (j < N) qx = j == 0 ? p0[0] : R[size_t(j - 1)].px, qz = j == 0 ? p0[1] : R[size_t(j - 1)].pz;
    } else {
        const float f = (L - (acc - seg)) / seg;
        const float ax = j == 0 ? p0[0] : R[size_t(j - 1)].px, az = j == 0 ? p0[1] : R[size_t(j - 1)].pz;
        qx = R[size_t(j)].px * f + ax * (1 - f), qz = R[size_t(j)].pz * f + az * (1 - f);
    }
    float dx = qx - p0[0], dz = qz - p0[1];
    const float dl = hyp(dx, dz);
    if (dl > 1e-6f) dx *= L / dl, dz *= L / dl;
    aim[0] = p0[0] + dx, aim[1] = p0[1] + dz;
    // Corners in the next 100 m: turns of more than 10 degrees between neighbouring sections, of one
    // sign, up to 30 records (0x61d480).
    float dist = 0;
    for (int k = 0; k + 2 < N && k + 2 < n && dist < 100.0f; ++k) {
        const Rec& a = R[size_t(k)];
        const Rec& b = R[size_t(k + 1)];
        const Rec& c = R[size_t(k + 2)];
        const float s1 = hyp(b.px - a.px, b.pz - a.pz);
        dist += s1;
        const float sn = std::clamp(b.nx * a.nz - b.nz * a.nx, -1.0f, 1.0f);
        const float ang = std::asin(sn) * 57.29578f;
        if (std::fabs(ang) > 10.0f) {
            const float s2 = hyp(c.px - b.px, c.pz - b.pz);
            if (corners.empty()) {
                corners.push_back({b.px, b.pz, ang, s1 + s2});
            } else {
                const float last = corners.back().angle;
                if (!((ang > 0 && last > 0) || (ang < 0 && last < 0))) break;
                corners.push_back({b.px, b.pz, ang, s2});
            }
            if (corners.size() == 30) break;
        } else if (!corners.empty()) {
            break;
        }
    }
    return true;
}

// ---- 0x61dcf0 IA_CornerSpeed -----------------------------------------------------------------

float aiCornerSpeed(const std::vector<AiCorner>& corners, const AiTables& tables, double x, double z) {
    if (corners.empty()) return -1;
    float ang = 0, len = 0;
    for (const AiCorner& c : corners) ang += c.angle, len += c.len;
    float r = len / (ang * 0.017453292f);
    if (r < 0) r = -r;
    // racer + (0x42c - ftol(R * -0.2)) * 4 for R < 100 (ftol truncates), racer+0x10fc otherwise.
    const float v = (r < 100.0f) ? tables.cornerSpeed[size_t(std::min(19, int(-(int)(r * -0.2f))))] : tables.cornerSpeed[19];
    float d, thr;
    if (corners.size() < 2) {
        d = float(std::hypot(x - corners[0].x, z - corners[0].z));
        thr = 35.0f;
    } else {
        d = float(std::hypot(x - corners[1].x, z - corners[1].z));
        thr = corners[1].len + 35.0f;
    }
    return d <= thr ? v : -1.0f;
}

// ---- the racer -------------------------------------------------------------------------------

void RaceDriver::setTarget(float targetTime, int level) {
    target_ = std::max(targetTime, 1.0f);  // K' = max(K, 1) (0x616d70)
    level_ = float(level);
}

size_t aiPassCheckpointBlocks(const RaceLine& line, size_t cursor, float s, AiCheckpointFlag& flag) {
    // IAPath_Progress walks the blocks while vf04 returns 1 (the cut point projects past a block's end); the checkpoint
    // block's vf04 then sets +0x21 / +0x24 when its index +0x2c differs from +0x24 (asm 0x624297..0x6242aa).
    while (cursor < line.gateAlong.size() && s > line.gateAlong[cursor] + 1.0f) flag.passed(int(cursor++));
    return cursor;
}

void RaceDriver::restart() {
    started_ = physical_ = airborne_ = false;
    v_ = {}, f_ = {0, 0, 1}, n_ = {0, 1, 0}, hist_ = {};
    s_ = 0, remaining0_ = -1;
    accel_ = 5, launchT_ = 0;
    stuck_ = -1, reverseT_ = 0, reversing_ = false;
    followT_ = followCap_ = 0;
    corners_.clear();
    samples_.clear();
    cpCursor_ = 0, cpFlag_ = {};  // rain-ai
    resets_ = 0;
    finishState_ = 0, park_.reset();  // PK16
    histPos_ = histVel_ = acc_ = {}, histDt_ = 1000.0f, histInit_ = false, lean_.reset();  // PK31: 0x617aa0 / 0x61c150
    // 0x616d70: S+0x18 by class (row = grid row); S+0x14 = S+0x44 = S+0x18; S+0x3c = 0, S+0x48 = 1, S+0x4c = 0.
    const float K = target_, row = float(row_);
    switch (mode_) {
    case 1: base_ = K + std::min(frac_ * K * 0.12f, 25.0f); break;
    case 2: base_ = K * (row * 0.005f + 1.0f); break;
    case 4: base_ = K * (row * 0.0025f + 1.0f); break;
    default: base_ = K; break;
    }
    budget_ = base_, cap_ = 0, timer_ = 0, gain_ = 1, late_ = 0;
}

float RaceDriver::ghostSpeed() const { return hyp(v_[0], v_[2]); }

// IA speed model 0x617000 (S = racer+0x17c): the ghost's acceleration S+4 = 27.78 / (0-100 time).
// w = the road's off-road weight. Own vehicle unless near the player (within 100 m and leading, or
// within 35 m and behind; never in knock-out mode 4): then the player's vehicle's times scaled by
// k = 1 - clamp((1 - remaining share) S2c, 0, 1) S30 (mode < 3; 0.6 in mode 3).
void RaceDriver::updateAccel(const Sense& s, float dt, float w, float share) {
    const float d = level_;
    const float S2c = mode_ <= 2 ? 1.0f / (0.85f - d * 0.1f) : 1.25f, S30 = d * 0.05f + 0.1f;
    // Own vehicle unless near the player: within 100 m and leading, or within 35 m and behind (never class 4).
    const bool nearPlayer = mode_ != 4 && ((s.playerDist < 100.0f && s.playerAhead > 0) || (s.playerDist < 35.0f && s.playerAhead < 0));
    auto own = [&] { accel_ = 27.777779f / (s.own.off0to100 * w + s.own.road0to100 * (1 - w)); };
    if (!nearPlayer) {
        own();
    } else {
        float k = 1.0f, p3 = 1.0f;
        const float q = 1.0f - share;
        if (late_ < 0 && q > 0.8f) {  // early, in the last 20 %: fade the player's pace out
            if (q > 0.9f) {
                own();
                return;
            }
            p3 = 1.0f - (q - 0.8f) * 10.0f;
        }
        if (mode_ >= 0 && mode_ < 3) k = (1.0f - std::clamp(q * S2c, 0.0f, 1.0f) * S30) * p3;
        else if (mode_ == 3) k = 0.6f;
        const float pv = std::sqrt(s.playerVel[0] * s.playerVel[0] + s.playerVel[1] * s.playerVel[1] + s.playerVel[2] * s.playerVel[2]);
        cap_ = std::max(std::max((s.player.roadKmh * (1 - w) + s.player.offKmh * w) * 0.27777779f * k, pv * k), cap_);
        accel_ = 27.777779f / ((s.player.road0to100 * (1 - w) + s.player.off0to100 * w) * k);
    }
    // Launch (S+0x10 < 5 s): 20 - 0.2 row (S+0xc, 0x616d70) for 2 s, then blended over 3 s.
    if (launchT_ < 5.0f) {
        launchT_ += dt;
        const float t = 5.0f - launchT_;
        const float push = 20.0f - float(row_) * 0.2f;
        accel_ = t <= 3.0f ? t * 0.33333334f * (push - accel_) + accel_ : push;
    }
}

// 0x617610: the predicted finish (race clock + time left m) against the class's target S+0x18. Late:
// S+0x48 = 1.1 and the budget shrinks by 1.25 x the delay over the share left; early: S+0x48 = 1 and
// (when asked) the budget grows by the margin. m = max(S+0x14 r, 1).
void RaceDriver::correct(float r, float& m, bool slower, float clock) {
    float e = clock + m - base_;
    late_ = e;
    if (e > 0) {
        gain_ = 1.1f;
        e *= 1.25f;
    } else {
        gain_ = 1.0f;
        if (!slower) return;
    }
    if (r <= 1e-6f) {
        timer_ = 0;
        return;
    }
    budget_ = std::max(budget_ - e / r, 1.0f);
    m = std::max(budget_ * r, 1.0f);
    timer_ = 0;
}

// 0x617380: the time left m = max(S+0x14 r, 1), corrected every 5 s (S+0x3c). Class 2 (the rubber band):
// ahead of the player by d up to 100 m: m x (1 + 0.1 d / 100); behind and over 35 m away: the
// correction may not slow it, and from rank 3 on m x (1 - 0.15 min(d / 50, 1)).
float RaceDriver::budget(float r, const Sense& s, float dt) {
    const float dist = s.playerDist, pd = s.playerAhead;
    float a = std::fabs(pd);
    if (dist - a < -50.0f) a = dist;
    float m = std::max(budget_ * r, 1.0f);
    auto tick = [&](bool slower) {
        if (timer_ < 5.0f) timer_ += dt;
        else correct(r, m, slower, s.clock);
    };
    if (mode_ == 2) {
        if (pd > 0) {
            m *= std::min(a * 0.01f, 1.0f) * 0.1f + 1.0f;
            gain_ = 1.0f;
        } else if (dist > 35.0f) {
            if (s.rank > 2) {
                tick(false);
                m *= 1.0f - std::clamp(a * 0.02f, 0.0f, 1.0f) * 0.15f;
            } else {
                tick(true);
            }
        }
        return m;
    }
    tick(true);
    return m;
}

RaceDriver::Command RaceDriver::step(const RaceLine& raceLine, const Sense& s, float dt) {
    Command cmd;
    const float total = raceLine.pathLength();
    // PK16 0x61b630: the gate check's result 4 (finished) writes the finish rank and the parking distance into the
    // path requests: the finish block becomes the line to the parking spot T (0x624310 / 0x624490).
    if (s.finishRank >= 0 && finishState_ == 0) {
        finishState_ = 4;
        park_ = raceLine.parkingLine(s.finishRank, float(s.aiCount - s.finishRank + 1) * 6.0f + 5.0f);
    }
    const RaceLine& line = park_ ? *park_ : raceLine;
    if (!started_) {
        started_ = true;
        g_ = gPrev_ = s.pos;
        f_ = {s.fwd[0], 0, s.fwd[2]};
        const float l = hyp(f_[0], f_[2]);
        f_ = l > 1e-4f ? std::array<float, 3>{f_[0] / l, 0, f_[2] / l} : std::array<float, 3>{0, 0, 1};
        s_ = line.project(g_[0], g_[2], 0, 200);
        v_ = {};
    }
    // Physics switch (0x5cb6f0): on with rank < 4 within 150 m, off past rank 3 or 165 m.
    const bool wasPhysical = physical_;
    physical_ = s.forcePhysical || (physical_ ? (s.rank <= 3 && s.switchDist <= 165.0f) : (s.rank < 4 && s.switchDist <= 150.0f));
    // Ghost tracking (0x619330): a simulated vehicle drags the ghost along the ghost's own direction.
    if (physical_) {
        if (!wasPhysical) v_ = s.vel;  // (0x5cb6f0 hands the velocity over; nothing to do the other way)
        const double d = (s.pos[0] - g_[0]) * f_[0] + (s.pos[2] - g_[2]) * f_[2];
        g_[0] += f_[0] * d, g_[2] += f_[2] * d;
        g_[1] = s.pos[1];
    }
    // PK31: 0x619330's acceleration estimate from the vehicle position (the second difference, each step divided by
    // the PREVIOUS frame's dt +0x1100, 1000 at init: the first frames read ~0).
    {
        if (!histInit_) histInit_ = true, histPos_ = {float(s.pos[0]), float(s.pos[1]), float(s.pos[2])};
        const double inv = 1.0 / double(histDt_);
        std::array<float, 3> v;
        for (int k = 0; k < 3; ++k) v[size_t(k)] = float(inv * (double(float(s.pos[size_t(k)])) - histPos_[size_t(k)]));
        for (int k = 0; k < 3; ++k) acc_[size_t(k)] = float(inv * (double(v[size_t(k)]) - histVel_[size_t(k)]));
        histPos_ = {float(s.pos[0]), float(s.pos[1]), float(s.pos[2])};
        histVel_ = v;
        histDt_ = dt;
    }
    const float sLo = s_ - 30.0f, sHi = s_ + 60.0f;
    s_ = line.project(g_[0], g_[2], sLo, sHi);
    // 0x624310 vf04: on the finish block the ghost's nearest point on A -> T reaching T, or within 5 m of it, parks
    // the racer (path +0x38 = 1); 0x61de60 then stops the ghost (state 5).
    bool parked = false;
    if (finishState_ == 4 && park_ && line.path.size() >= 3) {
        const auto& A = line.path[line.path.size() - 2];
        const auto& T = line.path.back();
        if (s_ >= line.pathAlong[line.path.size() - 2] - 1e-3f) {
            const float ex = T[0] - A[0], ez = T[1] - A[1], l2 = ex * ex + ez * ez;
            const float u = l2 > 0 ? float((g_[0] - A[0]) * ex + (g_[2] - A[1]) * ez) / l2 : 1.0f;
            if (u >= 1.0f) {
                parked = true;
            } else {
                const float t = std::max(u, 0.0f);
                parked = std::hypot(T[0] - (A[0] + ex * t), T[1] - (A[1] + ez * t)) < 5.0f;
            }
        }
    }
    if (s.frozen) {
        cmd.physical = true;
        cmd.input = {0, 0, 0, true};
        return cmd;
    }
    // Stuck and reset (0x61c970): the ghost moving under 1 m/s for 2 s backs up at 15 m/s for up to
    // 2 s; still stuck after 5 s, or the vehicle wants a reset: back on the line.
    if (reversing_) {
        reverseT_ += dt;
        if (reverseT_ >= 2.0f) reversing_ = false, reverseT_ = 0;
    }
    bool resetNow = s.overturned;
    const float moved = float(std::hypot(g_[0] - gPrev_[0], g_[2] - gPrev_[2]) + std::fabs(g_[1] - gPrev_[1]));
    if (finishState_ >= 4) {
        // 0x61c970: only a racer still in the race (state < 4) detects being stuck.
        if (!reversing_) stuck_ = -1.0f, reverseT_ = 0;
    } else if (moved <= dt * 1.0f) {
        stuck_ += dt;
        if (stuck_ > 1.0f && !reversing_) reversing_ = true;
        if (stuck_ > 4.0f) resetNow = true;
    } else if (!reversing_) {
        stuck_ = -1.0f, reverseT_ = 0;
    }
    if (resetNow) {
        // Back to the path at the ghost's progress (a free sample: 10 m on when another racer sits there),
        // facing along the line, at rest (0x61c970 / 0x619650 resetting branch).
        float sr = s_;
        for (int tries = 0; tries < 5; ++tries) {
            const auto p = line.at(sr);
            bool taken = false;
            for (const Other& o : s.others) taken = taken || std::hypot(o.x - p[0], o.z - p[2]) < 5.0;
            if (!taken) break;
            sr += 10.0f;
        }
        float dx = 0, dz = 1;
        const auto p = line.at(sr, &dx, &dz);
        float nrm[3] = {0, 1, 0};
        const float h = s.ghostGround ? s.ghostGround(p[0], g_[1], p[2], nrm) : s.ground ? s.ground(p[0], p[2], nrm) : p[1];
        g_ = gPrev_ = {p[0], h, p[2]};
        f_ = {dx, 0, dz};
        v_ = {};
        s_ = sr;
        stuck_ = -1, reversing_ = false, reverseT_ = 0;
        samples_.clear();
        ++resets_;
        cmd.reset = cmd.place = true;
        cmd.physical = physical_;
        cmd.pos = g_;
        const float yaw = std::atan2(dx, dz);
        cmd.rot = {0, std::sin(yaw * 0.5f), 0, std::cos(yaw * 0.5f)};
        if (std::getenv("FUEL_AI_TRACE")) std::printf("ai reset at s=%.0f (%.0f, %.0f)\n", sr, p[0], p[2]);
        return cmd;
    }
    // Path samples (0x6182b0 -> 0x6291f0(200 m, 10 m)): from the ghost's place on the line, re-cut
    // once the ghost is past the second sample; corridor = the block's half-width.
    auto sampleFrom = [&](float sStart) {
        // rain-ai: the cut moves the path cursor (IAPath_Progress 0x628750 at the cut point, 0x6182b0); the checkpoint
        // blocks it runs past raise the request's flag (IAPathBlockCheckpoint::vf04 0x624210).
        cpCursor_ = aiPassCheckpointBlocks(line, cpCursor_, sStart, cpFlag_);
        samples_.clear();
        staticObs_.clear();
        for (int k = 0; k <= 20; ++k) {
            const float sk = sStart + 10.0f * float(k);
            if (sk > line.pathLength() + line.finishRun) break;  // (PK16: the parking line's own length)
            const auto p = line.at(sk);
            AiSample a;
            a.x = p[0], a.z = p[2];
            const float hw = line.halfWidth(sk);
            a.latMin = -hw, a.latMax = hw;
            samples_.push_back(a);
        }
        float cum = 0;
        for (size_t k = 0; k < samples_.size(); ++k) {
            AiSample& a = samples_[k];
            a.cum = cum;
            if (k + 1 < samples_.size()) {
                const float dx = samples_[k + 1].x - a.x, dz = samples_[k + 1].z - a.z, l = std::max(hyp(dx, dz), 1e-4f);
                a.dx = dx / l, a.dz = dz / l, a.seg = l;
                cum += l;
            } else if (k > 0) {
                a.dx = samples_[k - 1].dx, a.dz = samples_[k - 1].dz, a.seg = samples_[k - 1].seg + 1.0f;
            }
        }
        // Static objects on the corridor (when the samples are cut, 0x619c40 -> 0x61a080 param_3):
        // 0x6188e0 bounds the samples +- their widths; each object's box, projected on the cross-section
        // strips it overlaps, blocks its lateral extent widened by the racer's radius (0x6189b0 projects
        // the objects' oriented boxes; the port has their world bounds).
        // Ground too steep to drive up (stand-in for the off-road blocks' ATV path finder, whose step cost
        // 0x61e710 refuses a rise of more than 0.5 per cell): lateral spans of a cross-section where the
        // ground climbs more than 0.5 along the line over 4 m.
        static const bool steepStandIn = !(std::getenv("FUEL_AI_STEEP") && std::atoi(std::getenv("FUEL_AI_STEEP")) == 0);
        if (steepStandIn && s.ground && samples_.size() > 1) {
            float nrm[3];
            for (size_t k = 1; k < samples_.size(); ++k) {
                const AiSample& a = samples_[k];
                float runLo = 0;
                bool inRun = false;
                for (float l = a.latMin; l <= a.latMax + 0.01f; l += 3.0f) {
                    const double px = a.x - a.dz * l, pz = a.z + a.dx * l;
                    const float rise = (s.ground(px + a.dx * 2.0, pz + a.dz * 2.0, nrm) - s.ground(px - a.dx * 2.0, pz - a.dz * 2.0, nrm)) * 0.25f;
                    const bool steep = rise > 0.5f;
                    if (steep && !inRun) runLo = l, inRun = true;
                    if ((!steep || l + 3.0f > a.latMax + 0.01f) && inRun) {
                        staticObs_.push_back({runLo - 1.5f - s.own.radius, (steep ? l : l - 3.0f) + 1.5f + s.own.radius, int(k), 0, a.cum, a.cum + a.seg});
                        inRun = false;
                    }
                }
            }
        }
        if (s.boxes && samples_.size() > 1) {
            // FUEL (P58): the static objects' collision boxes (0x61a080 -> 0x6188e0 bounds, 0x639ba0 gather, 0x619d70
            // per box, 0x6189b0 projection; margin 2.0 [0x99601c]). Boxes flagged 0x4000000 are ramps: type 2 (drive
            // into it) on the AI trails (road types 0-3 under the box, 0x6176f0) or when the world's random byte there
            // is >= 0.5, else 3.
            double x0 = 1e30, z0 = 1e30, x1 = -1e30, z1 = -1e30;
            for (const AiSample& a : samples_) {
                const float r = std::max(-a.latMin, a.latMax);
                x0 = std::min(x0, double(a.x - r)), x1 = std::max(x1, double(a.x + r));
                z0 = std::min(z0, double(a.z - r)), z1 = std::max(z1, double(a.z + r));
            }
            std::vector<DriveWorld::WorldBox> boxes;
            s.boxes(x0, z0, x1, z1, boxes);
            for (const auto& b : boxes) {
                int type = 0;
                if ((b.type & 0x1c000000u) == 0x4000000u) {
                    const float bx = b.frame[3], bz = b.frame[11];
                    const int rt = s.roadType ? s.roadType(bx, bz) : -1;
                    bool into = rt >= 0 && rt <= 3;
                    if (!into) {  // 0x640700: world +0x5bc84 byte table (filled at run time, not found; a fixed hash here)
                        uint32_t h = uint32_t(std::lrint(bz * 10.193578f + bx * 12.317513f) & 0xff) * 2654435761u;
                        h ^= h >> 15, h *= 2246822519u, h ^= h >> 13;
                        into = float(h & 0xff) / 255.0f >= 0.5f;
                    }
                    type = into ? 2 : 3;
                }
                aiBoxObstacles(samples_, b.frame.data(), b.half.data(), b.r, type, 2.0f, s.own.halfWidth, staticObs_);
            }
        } else if (s.statics && samples_.size() > 1) {
            double x0 = 1e30, z0 = 1e30, x1 = -1e30, z1 = -1e30;
            for (const AiSample& a : samples_) {
                const float r = std::max(-a.latMin, a.latMax);
                x0 = std::min(x0, double(a.x - r)), x1 = std::max(x1, double(a.x + r));
                z0 = std::min(z0, double(a.z - r)), z1 = std::max(z1, double(a.z + r));
            }
            std::vector<std::array<double, 6>> boxes;
            s.statics(x0, z0, x1, z1, boxes);
            for (const auto& b : boxes) {
                float nrm[3];
                const double cx = (b[0] + b[3]) * 0.5, cz = (b[2] + b[5]) * 0.5;
                const float h = s.ground ? s.ground(cx, cz, nrm) : float(b[1]);
                if (b[4] < h + 0.5 || b[1] > h + 3.0) continue;  // flat on the ground, or overhead
                const double cxs[4] = {b[0], b[3], b[3], b[0]}, czs[4] = {b[2], b[2], b[5], b[5]};
                for (size_t k = 1; k < samples_.size(); ++k) {
                    const AiSample& a = samples_[k];
                    float amin = 1e30f, amax = -1e30f, lmin = 1e30f, lmax = -1e30f;
                    for (int c = 0; c < 4; ++c) {
                        const float rx = float(cxs[c] - a.x), rz = float(czs[c] - a.z);
                        const float al = rx * a.dx + rz * a.dz, la = -rx * a.dz + rz * a.dx;
                        amin = std::min(amin, al), amax = std::max(amax, al), lmin = std::min(lmin, la), lmax = std::max(lmax, la);
                    }
                    if (amax < -samples_[k - 1].seg * 0.5f || amin > a.seg * 0.5f) continue;
                    if (lmax < a.latMin || lmin > a.latMax) continue;
                    AiObstacle o;
                    o.lo = lmin - s.own.radius, o.hi = lmax + s.own.radius, o.sample = int(k), o.s0 = a.cum;
                    staticObs_.push_back(o);
                }
            }
        }
        // The list is kept sorted by the along start (0x6189b0 inserts in order).
        std::stable_sort(staticObs_.begin(), staticObs_.end(), [](const AiObstacle& a, const AiObstacle& b) { return a.s0 < b.s0; });
    };
    if (samples_.size() < 3) sampleFrom(s_);
    // Where the ghost is on the samples (0x618780 / 0x6178a0).
    auto sampleIndex = [&](float sRel) {
        int i = 0;
        while (i + 1 < int(samples_.size()) && samples_[size_t(i + 1)].cum <= sRel) ++i;
        return i;
    };
    float s0Abs = line.project(samples_[0].x, samples_[0].z, s_ - 220, s_ + 10);
    int idx = sampleIndex(s_ - s0Abs);
    if (idx > 1) {
        sampleFrom(s0Abs + samples_[size_t(idx)].cum);
        s0Abs = line.project(samples_[0].x, samples_[0].z, s_ - 30, s_ + 10);
        idx = sampleIndex(s_ - s0Abs);
    } else {
        // Fresh corridor every frame (0x6182b0 restores the copy at racer+0x230).
        for (size_t k = 0; k < samples_.size(); ++k) {
            const float hw = line.halfWidth(s0Abs + samples_[k].cum);
            samples_[k].latMin = -hw, samples_[k].latMax = hw;
        }
    }
    // rain-ai: 0x6182b0's checkpoint-block check after the cut, every frame (asm 0x618466..0x6184d0).
    cmd.checkpointBlock = cpFlag_.frame();
    const float sRel = s_ - s0Abs;
    // The ghost's lateral offset on the samples.
    const AiSample& si = samples_[size_t(idx)];
    const float ghostLat = float((g_[0] - si.x) * -si.dz + (g_[2] - si.z) * si.dx);
    // Other racers (0x61a080): narrow the corridor on the next two samples, follow one about to be hit.
    std::vector<AiObstacle> obstacles = staticObs_;
    if (followT_ > 0) followT_ -= dt;
    for (const Other& o : s.others) {
        const float rx = float(o.x - g_[0]), rz = float(o.z - g_[2]);
        const float along = rx * f_[0] + rz * f_[2];
        const float rOwn = s.own.radius, rSum = rOwn + o.radius;
        if (!(-(rOwn + 1.0f) <= along && along <= rOwn + 15.0f)) continue;
        const float lat = rz * f_[0] - f_[2] * rx;
        float lo, hi;
        if (lat <= 0) {
            lo = -100.0f, hi = std::min(ghostLat - 0.5f, ghostLat + lat + rSum + 1.0f);
            if (!(samples_[size_t(idx)].latMin < hi)) continue;
        } else {
            lo = std::max(ghostLat + 0.5f, ghostLat + lat - rSum - 1.0f), hi = 100.0f;
            if (!(lo < samples_[size_t(idx)].latMax)) continue;
        }
        for (int k = idx; k < idx + 2 && k < int(samples_.size()); ++k) obstacles.push_back({lo, hi, k, 4, samples_[size_t(k)].cum, samples_[size_t(k)].cum});
        // About to hit it within 2 s (0x619f40, time to contact of the two discs on the current
        // velocities; argument order not confirmed, notes): hold its speed for a moment (+0xec0/+0xec4).
        if (along > 0) {
            const float dx = rx, dz = rz, wx = o.vx - v_[0], wz = o.vz - v_[2];
            const float a = wx * wx + wz * wz, b = dx * wx + dz * wz, c = dx * dx + dz * dz - rSum * rSum;
            float ttc = -1;
            if (c <= 0) ttc = 0;
            else if (a > 1e-8f && b < 0 && b * b - a * c >= 0) ttc = (-b - std::sqrt(b * b - a * c)) / a;
            if (ttc >= 0 && ttc < 2.0f) followT_ = 0.01f, followCap_ = hyp(o.vx, o.vz);
        }
    }
    std::stable_sort(obstacles.begin(), obstacles.end(), [](const AiObstacle& a, const AiObstacle& b) { return a.s0 < b.s0; });
    aiPropagateCorridor(samples_, idx, obstacles, ghostLat);
    // The line and the aim (0x61acb0); none: 20 m straight on.
    float aim[2];
    const float p0[2] = {float(g_[0]), float(g_[2])};
    if (!aiSmoothLineAndAim(samples_, idx, sRel, p0, hyp(hist_[0], hist_[1]), aim, corners_))
        aim[0] = float(g_[0]) + f_[0] * 20.0f, aim[1] = float(g_[2]) + f_[2] * 20.0f;
    // Speed (0x61dae0): no cap of its own before the last 100 m (S+0x1c grows with the distance left);
    // the car ahead, the corners.
    const float w = s.offroad ? std::clamp(s.offroad(g_[0], g_[2]), 0.0f, 1.0f) : 1.0f;
    // 0x616ed0: r = share of the race left (S+0x20 = 1 / the distance left at the first step); the cap
    // S+0x1c = S+0x48 / m x distance left (in the last 100 m never below the current speed).
    const float remaining = std::max(1.0f, total - s_);
    if (remaining0_ <= 0) remaining0_ = remaining;
    const float r = remaining / remaining0_;
    const float m = budget(r, s, dt);
    cap_ = gain_ / m * remaining;
    if (remaining < 100.0f) cap_ = std::max(cap_, hyp(v_[0], v_[2]));
    updateAccel(s, dt, w, r);
    if (nitro_ > 0) nitroApply(cap_, accel_);  // [nitro] physics/ai_nitro.cpp
    float target = cap_;
    if (!reversing_) {
        if (followT_ > 0 && followCap_ < target) target = followCap_;
        if (s.tables) {
            float c = aiCornerSpeed(corners_, *s.tables, g_[0], g_[2]);
            if (c > -1.0f) c *= cornerScale_;  // [upgrades U4] the racer's tyre stage (1 = FUEL)
            if (c > -1.0f && c < target) target = c;
        }
    }
    // 0x61de60: above the cap the speed decays by 0.9985 a millisecond; below it grows at S+4 less 0.1 v.
    float speed = hyp(v_[0], v_[2]);
    if (finishState_ == 4) {
        // Finished (state 4): parked -> stop (state 5); else slow by 0.5 v dt, at least 4 m/s, along the finish line.
        if (parked) {
            finishState_ = 5;
            speed = 0;
        } else {
            speed = std::max(4.0f, speed - 0.5f * speed * dt);
        }
    } else if (target <= speed) {
        for (float t = 0; t < dt; t += 0.001f) speed *= 0.9985f;
        speed = std::max(speed, target);
    } else {
        float a = accel_;
        const float fv = v_[2] * f_[2] + f_[0] * v_[0];
        if (fv > 0) a -= fv * 0.1f;
        speed = std::min(speed + a * dt, target);
    }
    // Velocity (0x61e080): the current direction plus the aim's, at that speed; slopes.
    float adx = aim[0] - float(g_[0]), adz = aim[1] - float(g_[2]);
    {
        const float l = hyp(adx, adz);
        if (l > 1e-3f) adx /= l, adz /= l;
    }
    float vx = v_[0], vz = v_[2];
    {
        const float l = hyp(vx, vz);
        if (l > 1e-3f) vx /= l, vz /= l;
    }
    if (!airborne_) {
        const float smax = std::sin(s.own.maxSlopeDeg * 0.017453292f);
        // 0x61e1ec..0x61e246: a copy of the velocity +0x11b4 normalised (Vec3_Normalize 0x410050, length 1;
        // unchanged under 1e-6): its y is the sine of the climb, compared with sin(P626). (Before P82 the port
        // used the raw vy in m/s: every descent pushed the ghost by 2 |vy| S+4 / sin(P626) a second and
        // kinematic racers ran at up to 700 km/h at the start.)
        float vy = v_[1];
        if (const float l2 = v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2]; l2 > 1e-6f) vy /= std::sqrt(l2);
        if (vy >= 0) {
            if (vy > smax * 0.3f) speed = std::max(speed, vy <= smax * 0.85f ? 10.0f : 20.0f);
        } else {
            speed -= (vy + vy) * (accel_ / smax) * dt;
        }
        vx += adx, vz += adz;
    } else {
        vx = vx * 20.0f + adx, vz = vz * 20.0f + adz;
    }
    float vy = v_[1];
    if (reversing_) vx = -f_[0], vz = -f_[2], speed = 15.0f;
    {
        const float l = hyp(vx, vz);
        if (l > 1e-6f) vx /= l, vz /= l;
    }
    v_ = {vx * speed, vy - dt * 9.81f, vz * speed};
    if (finishState_ == 5) v_ = {0, 0, 0};  // 0x619650 state 5: the ghost's velocity is zero (and it keeps its place)
    // Position (0x61be00): integrate, then the ground: above it by 1 m = airborne; below it = onto it,
    // the velocity along the ground (a bounce of 0.15 of a hard landing, at most 2 m/s, and a loss).
    // 0x619650 runs it for every racer, simulated or not: a simulated racer's ghost starts the frame on the vehicle
    // (0x619330: dragged along its forward axis, y = the vehicle's) and moves and lands like a kinematic one, so its
    // vertical speed follows the ground. (Before P97 the port zeroed the simulated ghost's vy and skipped the ground:
    // a racer switched to kinematic (0x5cb6f0, the ranks reshuffle at the start) left a downhill start with vy = 0
    // and floated 2-11 m over the slope for 1-4 s at the launch speeds: "flying over" the player.)
    gPrev_ = g_;
    hist_ = {(v_[0]), (v_[2])};
    {
        const double yWas = g_[1];
        g_[0] += v_[0] * dt, g_[1] += v_[1] * dt, g_[2] += v_[2] * dt;
        float nrm[3] = {0, 1, 0};
        const bool haveGround = s.ghostGround || s.ground;
        const float h = s.ghostGround ? s.ghostGround(g_[0], yWas, g_[2], nrm) : s.ground ? s.ground(g_[0], g_[2], nrm) : float(g_[1]);
        if (haveGround) n_ = {nrm[0], nrm[1], nrm[2]};
        else if (physical_) n_ = s.up;
        airborne_ = false;
        if (h + 1.0f < g_[1]) {
            airborne_ = true;
        } else if (g_[1] < h) {
            g_[1] = h;
            const std::array<float, 3>& N = n_;
            const float into = -(v_[0] * N[0] + v_[1] * N[1] + v_[2] * N[2]);
            // Along the ground: N x (v x N), normalised.
            const float cx = v_[1] * N[2] - v_[2] * N[1], cy = v_[2] * N[0] - v_[0] * N[2], cz = v_[0] * N[1] - v_[1] * N[0];
            float tx = 0, ty = 0, tz = 0;
            if (cx * cx + cy * cy + cz * cz > 0.01f) {
                tx = N[1] * cz - N[2] * cy, ty = N[2] * cx - N[0] * cz, tz = N[0] * cy - N[1] * cx;
                const float l = std::sqrt(tx * tx + ty * ty + tz * tz);
                tx /= l, ty /= l, tz /= l;
            }
            float tang = v_[1] * ty + v_[2] * tz + v_[0] * tx;
            float up = into;
            if (into > 0) {
                up = 0;
                if (into >= 2.0f) {
                    up = std::min(into * 0.15f, 2.0f);
                    const float rest = into - up;
                    if (rest > 20.0f) tang *= 0.7f;
                    else if (rest > 5.0f) tang *= 1.0f - (rest - 5.0f) * 0.020000001f;
                }
            }
            v_ = {N[0] * up + tx * tang, N[1] * up + ty * tang, tz * tang + N[2] * up};
        }
    }
    // Heading (0x61c4b0): turn the forward axis towards the aim by at most atan(0.2 |v| dt) a frame.
    {
        float dx = aim[0] - float(g_[0]), dz = aim[1] - float(g_[2]);
        const float l = hyp(dx, dz);
        if (l > 1e-6f) dx /= l, dz /= l;
        const float lim = std::atan(hyp(v_[0], v_[2]) * dt * 0.2f);
        const float a0 = std::atan2(f_[0], f_[2]), a1 = std::atan2(dx, dz);
        const float d = std::remainder(a1 - a0, 2.0f * kPi);
        if (hyp(v_[0], v_[2]) > 0.001f && std::fabs(d) > 0.001f && l > 1e-6f) {
            const float a = std::fabs(d) <= lim ? a1 : a0 + (d > 0 ? lim : -lim);
            f_ = {std::sin(a), 0, std::cos(a)};
        }
    }
    // Orientation: yaw from the forward axis, tilted onto the ground normal (0x6443a0 x2).
    std::array<float, 4> rot;
    {
        const float yaw = std::atan2(f_[0], f_[2]);
        const float qy[4] = {0, std::sin(yaw * 0.5f), 0, std::cos(yaw * 0.5f)};
        // Shortest rotation from +y to the normal.
        const float nx = n_[0], ny = n_[1], nz = n_[2];
        float ax = nz, ay = 0, az = -nx;  // (0,1,0) x n
        const float c = std::clamp(ny, -1.0f, 1.0f);
        const float sl = std::sqrt(ax * ax + az * az);
        float qn[4] = {0, 0, 0, 1};
        if (sl > 1e-6f) {
            const float ang = std::acos(c);
            const float sh = std::sin(ang * 0.5f) / sl;
            qn[0] = ax * sh, qn[1] = ay * sh, qn[2] = az * sh, qn[3] = std::cos(ang * 0.5f);
        }
        // rot = qn * qy
        rot = {qn[3] * qy[0] + qn[0] * qy[3] + qn[1] * qy[2] - qn[2] * qy[1], qn[3] * qy[1] - qn[0] * qy[2] + qn[1] * qy[3] + qn[2] * qy[0],
               qn[3] * qy[2] + qn[0] * qy[1] - qn[1] * qy[0] + qn[2] * qy[3], qn[3] * qy[3] - qn[0] * qy[0] - qn[1] * qy[1] - qn[2] * qy[2]};
    }
    // PK31: IA_GhostOrient 0x61c4b0 tail: a simulated racer resets the lean (0x61c150); a kinematic one runs the
    // springs (0x61c1b0) and is placed with Quat_Mul(orient, lean) = lean x orient (0x4f07d0).
    if (physical_) {
        lean_.reset();
    } else {
        lean_.step(dt, acc_, rot, s.own.vehicleClass == 1);
        rot = AiLean::quatMul(rot, lean_.q);
    }
    cmd.physical = physical_;
    cmd.pos = g_;
    cmd.vel = v_;
    cmd.rot = rot;
    if (physical_ && !s.forcePhysical && std::hypot(std::hypot(g_[0] - s.pos[0], g_[2] - s.pos[2]), g_[1] - s.pos[1]) > 20.0) {
        // Pilot_SetTarget 0x5c9c20: an AI vehicle more than 20 m from its ghost is put on it (0x5c9760).
        if (std::getenv("FUEL_AI_TRACE"))
            std::printf("ai 20 m rule: ghost (%.1f, %.1f, %.1f) vehicle (%.1f, %.1f, %.1f) ghost %.1f m/s airborne %d\n", g_[0], g_[1], g_[2], s.pos[0],
                        s.pos[1], s.pos[2], hyp(v_[0], v_[2]), int(airborne_));
        cmd.place = cmd.reset = true;
        ++resets_;
        return cmd;
    }
    if (physical_) {
        // 0x5c97d0 (AI): no steering; throttle or brake towards the ghost's speed along the vehicle's axis.
        const float tgt = v_[0] * s.fwd[0] + v_[1] * s.fwd[1] + v_[2] * s.fwd[2];
        if (std::fabs(tgt) > 1.0f) {
            // 0x5c9844: the reference is the mean wheel rim speed (0x5257b0), not the forward speed.
            const float d = tgt - s.cruiseSpeed;
            if (d > 0) cmd.input.throttle = std::min(d, 1.0f);
            else cmd.input.brake = std::min(-d, 1.0f);
        }
        if (finishState_ == 5) cmd.input.brake = 1.0f;  // parked: held (port: FUEL's pilot holds the vehicle at the ghost)
    } else {
        cmd.place = true;
    }
    return cmd;
}


// ---- PK16: FUEL's global float generator and the AI names (0x620c20) ---------------------------------------------

namespace {
float gFuelRandState = 0.5f;  // 0xa065cc at start-up (.data)
}

// state' = frac mantissa of (state * 8.214545 + 2.0) * 1.4154547: the x87 expression (extended precision) is stored as
// a float, its mantissa bits are put under the exponent of 1.0 and 1.0 is subtracted.
float fuelRandom() {
    const long double prod = ((long double)gFuelRandState * 8.214545249938965L + 2.0L) * 1.4154547452926636L;
    const float f = float(prod);
    uint32_t bits;
    std::memcpy(&bits, &f, 4);
    bits = (bits & 0x7fffff) | 0x3f800000;
    float m;
    std::memcpy(&m, &bits, 4);
    gFuelRandState = m - 1.0f;
    return gFuelRandState;
}

// 0x620c20: ids 1500 + i; then for i = 0..73: u = next random, j = trunc(u (74 - i)), swap slot i with slot i + j.
std::array<int, 74> aiNameShuffle() {
    std::array<int, 74> ids{};
    for (int i = 0; i < 74; ++i) ids[size_t(i)] = i + 0x5dc;
    for (int i = 0; i < 74; ++i) {
        const float u = fuelRandom();
        const int j = int(double(74 - i) * double(u));
        std::swap(ids[size_t(i)], ids[size_t(std::min(i + j, 73))]);
    }
    return ids;
}

}  // namespace fuel
