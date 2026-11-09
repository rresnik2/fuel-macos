#include "road_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("road data too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

// CRT _ftol2 (0x8e01d0): truncation (the StockData grid uses it, not the FPU's round-down mode).
inline int32_t ftol(float v) { return int32_t(v); }

}  // namespace

RoadGraph::RoadGraph(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    const DpcFile::Entry* re = world.find(rd32(*gen, 8));
    const auto body = re ? world.body(*re) : std::nullopt;
    if (!body) throw std::runtime_error("GwRoad_Z not found");
    const std::vector<uint8_t>& b = *body;

    // GwRoad_Z::Load 0x7c41b0 + insert 0x6d0990 (same reading as road_segments): type byte (bit 7 =
    // rigid), points in quarter metres, consecutive duplicates dropped, bbox in 2 m cells.
    const uint32_t n = rd32(b, 0);
    size_t p = 20;
    recs_.reserve(n);
    std::vector<std::array<int32_t, 2>> q;
    for (uint32_t i = 0; i < n; ++i) {
        if (p + 3 > b.size()) throw std::runtime_error("GwRoad_Z record overruns");
        Rec r{};
        r.type = b[p] & 0x7F;
        r.flags = (b[p] & 0x80) ? kRigid : 0;
        const uint16_t count = uint16_t(b[p + 1] | (b[p + 2] << 8));
        p += 3;
        if (p + size_t(count) * 5 > b.size()) throw std::runtime_error("GwRoad_Z points overrun");
        q.clear();
        for (uint16_t k = 0; k < count; ++k, p += 5) {
            const uint64_t v = (uint64_t(b[p]) << 32) | (uint64_t(b[p + 1]) << 24) | (uint64_t(b[p + 2]) << 16) | (uint64_t(b[p + 3]) << 8) | b[p + 4];
            int32_t a = int32_t(v >> 20), c = int32_t(v & 0xFFFFF);
            if (a & 0x80000) a -= 1 << 20;
            if (c & 0x80000) c -= 1 << 20;
            q.push_back({a, c});
        }
        if (q.empty()) q.push_back({0, 0});
        int32_t mnx = q[0][0], mxx = q[0][0], mnz = q[0][1], mxz = q[0][1];
        for (const auto& v : q) {
            mnx = std::min(mnx, v[0]), mxx = std::max(mxx, v[0]);
            mnz = std::min(mnz, v[1]), mxz = std::max(mxz, v[1]);
        }
        r.minX = int16_t(mnx >> 3), r.minZ = int16_t(mnz >> 3), r.maxX = int16_t((mxx + 4) >> 3), r.maxZ = int16_t((mxz + 4) >> 3);
        r.first = uint32_t(pts_.size());
        for (size_t k = 0; k < q.size(); ++k) {
            if (k + 1 < q.size() && q[k] == q[k + 1]) continue;
            pts_.push_back(q[k]);
        }
        r.n = uint16_t(pts_.size() - r.first);
        r.link0 = r.link1 = i;
        r.side0 = 0, r.side1 = 1;
        recs_.push_back(r);
    }

    // Graph links 0x6d0fc0 (as road_segments): records from last to first; each end joins the ring of
    // the first record end seen at exactly that point.
    std::unordered_map<uint64_t, std::pair<uint32_t, uint8_t>> nodes;
    nodes.reserve(size_t(n) * 2);
    for (int64_t i = int64_t(n) - 1; i >= 0; --i) {
        Rec& r = recs_[size_t(i)];
        for (int end = 0; end < 2; ++end) {
            const auto& e = pts_[r.first + (end ? r.n - 1u : 0u)];
            const uint64_t key = (uint64_t(uint32_t(e[0])) << 32) | uint32_t(e[1]);
            auto [it, fresh] = nodes.try_emplace(key, uint32_t(i), uint8_t(end));
            uint32_t& myLink = end ? r.link1 : r.link0;
            uint8_t& mySide = end ? r.side1 : r.side0;
            if (fresh) {
                myLink = uint32_t(i), mySide = uint8_t(end);
                continue;
            }
            Rec& o = recs_[it->second.first];
            uint32_t& oLink = it->second.second ? o.link1 : o.link0;
            uint8_t& oSide = it->second.second ? o.side1 : o.side0;
            myLink = oLink, mySide = oSide;
            oLink = uint32_t(i), oSide = uint8_t(end);
        }
    }

    // 0x6d4130: bit 30 on every type-18 (HIGHWAY) record linked to another record, then flooded over
    // the rings at both ends to a fixed point.
    std::vector<uint32_t> stack;
    for (uint32_t i = 0; i < n; ++i)
        if (recs_[i].type == 18 && (recs_[i].link0 != i || recs_[i].link1 != i)) recs_[i].flags |= kHighway, stack.push_back(i);
    while (!stack.empty()) {
        const uint32_t i = stack.back();
        stack.pop_back();
        for (int end = 0; end < 2; ++end) {
            uint32_t r = end ? recs_[i].link1 : recs_[i].link0;
            int s = end ? recs_[i].side1 : recs_[i].side0;
            for (int guard = 0; guard < 4096 && r != i; ++guard) {
                if (!(recs_[r].flags & kHighway)) recs_[r].flags |= kHighway, stack.push_back(r);
                const Rec& R = recs_[r];
                const uint32_t nr = s ? R.link1 : R.link0;
                s = s ? R.side1 : R.side0;
                r = nr;
            }
        }
    }

    // StockData grid 0x6df8b0 / 0x6dfa10: 128 x 128 cells over [min(min*2) - 1, max(max*2) + 1],
    // items in ascending order inside each cell, cell index by truncation (_ftol2).
    float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
    for (const Rec& r : recs_) {
        x0 = std::min(x0, float(r.minX) * 2), z0 = std::min(z0, float(r.minZ) * 2);
        x1 = std::max(x1, float(r.maxX) * 2), z1 = std::max(z1, float(r.maxZ) * 2);
    }
    x0 -= 1, z0 -= 1, x1 += 1, z1 += 1;
    gx0_ = x0, gz0_ = z0;
    invX_ = 1.0f / ((x1 - x0) / 128.0f), invZ_ = 1.0f / ((z1 - z0) / 128.0f);
    cellCount_.assign(128 * 128, 0);
    cellStart_.assign(128 * 128 + 1, 0);
    auto range = [&](const Rec& r, int& cx0, int& cx1, int& cz0, int& cz1) {
        cx0 = std::max(0, ftol((float(r.minX) * 2 - gx0_) * invX_)), cx1 = std::min(127, ftol((float(r.maxX) * 2 - gx0_) * invX_));
        cz0 = std::max(0, ftol((float(r.minZ) * 2 - gz0_) * invZ_)), cz1 = std::min(127, ftol((float(r.maxZ) * 2 - gz0_) * invZ_));
    };
    for (const Rec& r : recs_) {
        int a, b2, c, d;
        range(r, a, b2, c, d);
        for (int cz = c; cz <= d; ++cz)
            for (int cx = a; cx <= b2; ++cx) ++cellCount_[size_t(cz * 128 + cx)];
    }
    for (size_t i = 0; i < 128 * 128; ++i) cellStart_[i + 1] = cellStart_[i] + cellCount_[i];
    items_.assign(cellStart_.back(), 0);
    std::vector<uint32_t> fill(cellStart_.begin(), cellStart_.end() - 1);
    for (uint32_t i = 0; i < n; ++i) {
        int a, b2, c, d;
        range(recs_[i], a, b2, c, d);
        for (int cz = c; cz <= d; ++cz)
            for (int cx = a; cx <= b2; ++cx) items_[fill[size_t(cz * 128 + cx)]++] = i;
    }
}

float RoadGraph::segDist2(uint32_t ri, float px, float pz, float rad, int& k, float& qx, float& qz) const {
    // 0x6d1520: -1 unless the bbox grown by r contains p; else 0x6d15d0 over the edges (0x4ae1f0).
    const Rec& r = recs_[ri];
    if (px < float(r.minX) * 2 - rad || px > float(r.maxX) * 2 + rad || pz < float(r.minZ) * 2 - rad || pz > float(r.maxZ) * 2 + rad)
        return -1.0f;
    float best = rad * rad;
    for (uint32_t j = 0; j + 1 < r.n; ++j) {
        const auto a = point(ri, j), c = point(ri, j + 1);
        const float bx = c[0] - a[0], bz = c[1] - a[1];
        const float t = (px - a[0]) * bx + (pz - a[1]) * bz;
        float cx, cz;
        if (t <= 0) cx = a[0], cz = a[1];
        else {
            const float l2 = bx * bx + bz * bz;
            if (t >= l2) cx = c[0], cz = c[1];
            else {
                const float f = t / l2;
                cx = a[0] + bx * f, cz = a[1] + bz * f;
            }
        }
        const float d = (px - cx) * (px - cx) + (pz - cz) * (pz - cz);
        if (d < best) best = d, k = int(j), qx = cx, qz = cz;
    }
    const auto l = point(ri, r.n - 1u);
    const float dl = (px - l[0]) * (px - l[0]) + (pz - l[1]) * (pz - l[1]);
    if (dl < best) best = dl, qx = l[0], qz = l[1], k = int(r.n) - 2;
    return best;
}

float RoadGraph::nearestRoad(float x, float z, float r, int& rec, int& k, float& qx, float& qz, const std::vector<int>& types) const {
    // 0x6d19f0: grid cells of [p - r, p + r], z outer, x inner; each record once (bit 22 visited mark).
    float best = r * r;
    rec = -1, k = -1;
    const int cx0 = std::clamp(ftol((x - r - gx0_) * invX_), 0, 127), cx1 = std::clamp(ftol((x + r - gx0_) * invX_), 0, 127);
    const int cz0 = std::clamp(ftol((z - r - gz0_) * invZ_), 0, 127), cz1 = std::clamp(ftol((z + r - gz0_) * invZ_), 0, 127);
    std::vector<uint32_t> seen;
    for (int cz = cz0; cz <= cz1; ++cz)
        for (int cx = cx0; cx <= cx1; ++cx) {
            const size_t c = size_t(cz * 128 + cx);
            for (uint32_t i = cellStart_[c]; i < cellStart_[c + 1]; ++i) {
                const uint32_t ri = items_[i];
                const Rec& R = recs_[ri];
                if ((R.flags & kExcluded) || R.type == 14) continue;
                if (std::find(seen.begin(), seen.end(), ri) != seen.end()) continue;
                seen.push_back(ri);
                if (!types.empty() && std::find(types.begin(), types.end(), int(R.type)) == types.end()) continue;
                int kk = -1;
                float px = 0, pz = 0;
                const float d = segDist2(ri, x, z, r, kk, px, pz);
                if (d < best && d >= 0) best = d, rec = int(ri), k = kk, qx = px, qz = pz;
            }
        }
    return rec < 0 ? -1.0f : best;
}

int RoadGraph::roadAt(float x, float z, const std::function<float(int)>& flatHalf) const {
    // 0x6d1d70: nearest within 30 m, accepted within the type's flat half-width T[+0xcc].
    int rec, k;
    float qx, qz;
    const float d = nearestRoad(x, z, 30.0f, rec, k, qx, qz);
    if (d < 0) return -1;
    const float w = flatHalf(recs_[size_t(rec)].type);
    return d <= w * w ? rec : -1;
}

float RoadGraph::lengthCost(uint32_t r, uint32_t off, uint32_t count) const {
    // 0x4ae340: sum of the point-to-point distances.
    double s = 0;
    for (uint32_t i = off + 1; i < off + count; ++i) {
        const auto a = point(r, i - 1), c = point(r, i);
        s += std::sqrt(double(c[0] - a[0]) * double(c[0] - a[0]) + double(c[1] - a[1]) * double(c[1] - a[1]));
    }
    return float(s);
}

float RoadGraph::aiCost(uint32_t ri, uint32_t off, uint32_t count, int dir, const std::array<float, 6>& c, bool forbid3,
                        const std::function<float(float, float)>& height) const {
    // 0x4ef600: per-metre factor by road type (switch 0x4ef8c8 / 0x4ef8a8), times the slope factor 0x4ef590.
    if (count == 1) return 0;
    const Rec& R = recs_[ri];
    if (R.flags & kBlocked) return 1e6f;
    float f;
    switch (R.type) {
        case 0: case 1: f = 0.2f; break;
        case 2: f = 0.1f; break;
        case 3: if (forbid3) return 1e6f; f = 0.0666667f; break;
        case 6: case 20: f = 1.0f / c[2]; break;
        case 7: case 8: case 9: case 13: case 16: f = 1.0f / c[1]; break;
        case 10: case 11: case 12: case 15: case 17: case 18: case 19: f = 1.0f / c[0]; break;
        case 14: return 1e6f;
        default: f = 1.0f / c[3]; break;  // 4, 5 and > 20
    }
    double cost = 0;
    std::array<float, 2> pa{};
    float ha = 0;
    for (uint32_t s = 0; s < count; ++s) {
        const uint32_t i = dir ? off + s : off + count - 1 - s;
        const auto pb = point(ri, i);
        const float hb = height(pb[0], pb[1]);
        if (s > 0) {
            const float L = std::sqrt((pb[0] - pa[0]) * (pb[0] - pa[0]) + (pb[1] - pa[1]) * (pb[1] - pa[1]));
            if (s == 1 ? L != 0 : L > 1e-6f) {
                const float sl = std::fabs(hb - ha) * (1.0f / L);
                const float cc = hb > ha ? c[4] : c[5];
                const float S = sl < 0.05f ? 1.0f : sl < 0.15f ? (cc + 1.0f) * 0.5f : cc;
                cost += double(S) * L * f;
            }
        }
        pa = pb, ha = hb;
    }
    return float(cost);
}

bool RoadGraph::inEnd1Ring(uint32_t a, uint32_t b) const {
    uint32_t r = recs_[a].link1;
    int e = recs_[a].side1;
    for (int guard = 0; guard < 4096 && r != a; ++guard) {
        if (r == b) return true;
        const Rec& R = recs_[r];
        const uint32_t nr = e ? R.link1 : R.link0;
        e = e ? R.side1 : R.side0;
        r = nr;
    }
    return false;
}

bool RoadGraph::sameRecordDir(uint32_t r, int ks, int ke, float sx, float sz, float ex, float ez) const {
    // 0x6d7820
    if (ks < ke) return true;
    if (ks > ke) return false;
    const auto a = point(r, uint32_t(ks)), c = point(r, uint32_t(ks) + 1);
    return (ex - sx) * (c[0] - a[0]) + (ez - sz) * (c[1] - a[1]) > 0;
}

bool RoadGraph::request(Route& route, float sx, float sz, float ex, float ez, const CostFn& cost) const {
    route = Route{};
    route.valid = true;
    if (nearestRoad(ex, ez, 1000.0f, route.endRec, route.endK, route.epx, route.epz) < 0) return route.valid = false;
    if (nearestRoad(sx, sz, 1000.0f, route.startRec, route.startK, route.spx, route.spz) < 0) return route.valid = false;
    if (route.startRec == route.endRec) {
        route.startDir = sameRecordDir(uint32_t(route.startRec), route.startK, route.endK, route.spx, route.spz, route.epx, route.epz);
        route.endDir = !route.startDir;
        return true;
    }
    if (!search(route, cost)) return route.valid = false;
    return true;
}

bool RoadGraph::search(Route& route, const CostFn& cost) const {
    // 0x6d7f90: two-bucket best-first search with branch and bound (out/p32/roadgraph.md 2.5).
    struct Node {
        int32_t X, Z;
        uint32_t parentRec;
        uint8_t parentDir, state;  // state 0 new, 1 open, 2 closed / goal
        float f, h;
        int next;
    };
    std::vector<Node> nodes;
    nodes.reserve(256);
    std::unordered_map<uint64_t, int> index;
    auto key = [](int32_t X, int32_t Z) { return (uint64_t(uint32_t(X)) << 32) | uint32_t(Z); };
    auto endPt = [&](uint32_t r, int e) { return pts_[recs_[r].first + (e ? recs_[r].n - 1u : 0u)]; };
    // 0x6d7bb0: the node at end e of r (created with parent (r, e) and state 0).
    auto node = [&](uint32_t r, int e) -> int {
        const auto P = endPt(r, e);
        auto [it, fresh] = index.try_emplace(key(P[0], P[1]), int(nodes.size()));
        if (fresh) nodes.push_back({P[0], P[1], r, uint8_t(e), 0, 0, 0, -1});
        return it->second;
    };
    std::unordered_map<uint64_t, float> cache;  // 0x6d7a80 (pure callbacks: caching changes nothing)
    auto C = [&](uint32_t r, int dir) -> float {
        const Rec& R = recs_[r];
        if (dir ? (R.flags & kNoFwd) : (R.flags & kNoRev)) return 1e6f;
        const uint64_t k = (uint64_t(r) << 1) | uint64_t(dir);
        auto it = cache.find(k);
        if (it != cache.end()) return it->second;
        const float c = cost(r, 0, R.n, dir);
        cache.emplace(k, c);
        return c;
    };
    const uint32_t sr = uint32_t(route.startRec), er = uint32_t(route.endRec);
    const int sk = route.startK, ek = route.endK;
    if ((recs_[sr].flags ^ recs_[er].flags) & kHighway) return false;
    // Part costs 0x6d7e90: head = points 0..k, tail = points k..n-1.
    auto part = [&](uint32_t r, int k, bool start, float& A, float& B) {
        const Rec& R = recs_[r];
        const uint32_t kk = uint32_t(std::max(k, 0));
        if (start) {
            A = (R.flags & kNoRev) ? 1e6f : cost(r, 0, kk + 1, 0);
            B = (R.flags & kNoFwd) ? 1e6f : cost(r, kk, R.n - kk, 1);
        } else {
            A = (R.flags & kNoFwd) ? 1e6f : cost(r, 0, kk + 1, 1);
            B = (R.flags & kNoRev) ? 1e6f : cost(r, kk, R.n - kk, 0);
        }
    };
    float As, Bs, Ae, Be;
    part(sr, sk, true, As, Bs);
    part(er, ek, false, Ae, Be);
    float best = 1e30f;
    int bestNode = -1;
    const int S1 = node(sr, 1), S0 = node(sr, 0);
    if (S0 == S1) return false;
    const int E1 = node(er, 1), E0 = node(er, 0);
    if (E0 == E1) return false;
    float v = best;
    if (S1 == E1) v = best = Be + Bs, bestNode = E1;
    else if (S0 == E1) v = best = Be + As, bestNode = E1;
    nodes[size_t(E1)].f = v, nodes[size_t(E1)].h = Be, nodes[size_t(E1)].state = 2;
    if (S1 == E0) v = best = Ae + Bs, bestNode = E0;
    else if (S0 == E0) v = best = Ae + As, bestNode = E0;
    nodes[size_t(E0)].f = v, nodes[size_t(E0)].h = Ae, nodes[size_t(E0)].state = 2;
    route.list.clear();
    if (nodes[size_t(S1)].state == 2 && nodes[size_t(S0)].state == 2) {
        const Node &n0 = nodes[size_t(S0)], &n1 = nodes[size_t(S1)];
        if (n0.f <= n1.f) route.startDir = false, route.endDir = S0 == E1, route.cost = n0.f;
        else route.startDir = true, route.endDir = S1 == E1, route.cost = n1.f;
        return true;
    }
    const float d = std::sqrt((route.spx - route.epx) * (route.spx - route.epx) + (route.spz - route.epz) * (route.spz - route.epz));
    int listA = -1, listB = -1;
    if (nodes[size_t(S0)].state != 2) {
        Node& n = nodes[size_t(S0)];
        n.h = d, n.f = float(double(d) + As), n.next = -1, n.state = 1;
        listA = S0;
    }
    if (nodes[size_t(S1)].state != 2) {
        Node& n = nodes[size_t(S1)];
        n.h = d, n.f = float(double(d) + Bs), n.state = 1;
        n.next = listA;
        listA = S1;
    }
    for (;;) {
        int cur;
        if (listA >= 0) cur = listA, listA = -1;
        else if (listB >= 0) cur = listB, listB = -1;
        else break;
        bool stop = false;
        while (cur >= 0 && !stop) {
            int nextInChain = nodes[size_t(cur)].next;
            nodes[size_t(cur)].state = 2;
            uint32_t r = nodes[size_t(cur)].parentRec;
            int s = nodes[size_t(cur)].parentDir;
            for (int guard = 0; guard < 4096; ++guard) {
                {
                    const Rec& R = recs_[r];
                    const uint32_t nr = s == 0 ? R.link0 : R.link1;
                    s = s == 0 ? R.side0 : R.side1;
                    r = nr;
                }
                if (r == nodes[size_t(cur)].parentRec) break;
                const int dir = s == 0 ? 1 : 0;
                const float c = C(r, dir);
                const double t = (double(nodes[size_t(cur)].f) - nodes[size_t(cur)].h) + c;
                const int N = node(r, dir ? 1 : 0);
                Node& n = nodes[size_t(N)];
                if (n.state != 0) {
                    const double t2 = t + n.h;
                    if (N == E0 || N == E1) {
                        if (!(double(best) > t2)) continue;
                        best = float(t2), n.f = float(t2), n.parentRec = r, n.parentDir = uint8_t(dir), n.state = 2, bestNode = N;
                        continue;
                    }
                    if (t2 < n.f) {
                        if (!(t2 < best)) continue;
                        n.f = float(t2), n.parentRec = r, n.parentDir = uint8_t(dir);
                        if (n.state == 2) n.state = 1, n.next = nextInChain, nextInChain = N;  // re-open into this chain
                        continue;
                    }
                    if (n.state == 2) continue;
                    // N open and not improved: try the reverse edge to improve cur, then restart the ring walk.
                    const double t3 = (double(n.f) - n.h) + C(r, !dir) + nodes[size_t(cur)].h;
                    if (t3 < nodes[size_t(cur)].f) {
                        nodes[size_t(cur)].f = float(t3);
                        nodes[size_t(cur)].parentRec = r, nodes[size_t(cur)].parentDir = uint8_t(s);
                    }
                    continue;
                }
                // a new node
                if (nodes.size() > 9999) {
                    if (bestNode < 0) return false;
                    listA = listB = -1;
                    stop = true;
                    break;
                }
                const float ex = float(n.X) * 0.25f - route.epx, ez = float(n.Z) * 0.25f - route.epz;
                const float d2 = ex * ex + ez * ez;
                const float h = d2 > 1e-5f ? d2 / std::sqrt(d2) : d2;
                n.h = h, n.f = float(t + h), n.parentRec = r, n.parentDir = uint8_t(dir);
                if (!(n.f < best)) {
                    n.state = 2;  // pruned
                    continue;
                }
                n.state = 1;
                if (h <= nodes[size_t(cur)].h) n.next = listA, listA = N;
                else n.next = listB, listB = N;
            }
            cur = nextInChain;
        }
    }
    if (bestNode < 0) return false;
    int n = bestNode;
    for (int guard = 0; guard < 100000 && nodes[size_t(n)].parentRec != sr; ++guard) {
        const uint32_t pr = nodes[size_t(n)].parentRec;
        route.list.push_back(pr);
        n = node(pr, nodes[size_t(n)].parentDir ? 0 : 1);
    }
    std::reverse(route.list.begin(), route.list.end());
    route.cost = best;
    route.startDir = inEnd1Ring(sr, route.list.empty() ? er : route.list.front());
    route.endDir = inEnd1Ring(er, route.list.empty() ? sr : route.list.back());
    return true;
}

std::vector<std::array<float, 2>> RoadGraph::polyline(const Route& rt, std::vector<uint32_t>* recOf) const {
    // The path 0x6da240 walks: start point, the start record towards startDir, each list record (backward
    // when the previous record is in its end-1 ring), the end record from its endDir side, the end point.
    std::vector<std::array<float, 2>> out;
    if (!rt.valid || rt.startRec < 0 || rt.endRec < 0) return out;
    auto add = [&](std::array<float, 2> p, uint32_t r) {
        out.push_back(p);
        if (recOf) recOf->push_back(r);
    };
    const uint32_t sr = uint32_t(rt.startRec), er = uint32_t(rt.endRec);
    add({rt.spx, rt.spz}, sr);
    if (sr == er) {
        if (rt.startK < rt.endK)
            for (int k = rt.startK + 1; k <= rt.endK; ++k) add(point(sr, uint32_t(k)), sr);
        else if (rt.startK > rt.endK)
            for (int k = rt.startK; k > rt.endK; --k) add(point(sr, uint32_t(k)), sr);
        add({rt.epx, rt.epz}, er);
        return out;
    }
    const int ns = recs_[sr].n;
    if (rt.startDir)
        for (int k = rt.startK + 1; k < ns; ++k) add(point(sr, uint32_t(k)), sr);
    else
        for (int k = rt.startK; k >= 0; --k) add(point(sr, uint32_t(k)), sr);
    uint32_t prev = sr;
    for (uint32_t r : rt.list) {
        const int n = recs_[r].n;
        if (inEnd1Ring(r, prev))
            for (int k = n - 2; k >= 0; --k) add(point(r, uint32_t(k)), r);
        else
            for (int k = 1; k < n; ++k) add(point(r, uint32_t(k)), r);
        prev = r;
    }
    const int ne = recs_[er].n;
    if (!rt.endDir)
        for (int k = 1; k <= rt.endK; ++k) add(point(er, uint32_t(k)), er);
    else
        for (int k = ne - 2; k > rt.endK; --k) add(point(er, uint32_t(k)), er);
    add({rt.epx, rt.epz}, er);
    return out;
}


// ---- road loops (free-roam traffic, P58) -----------------------------------------------------------------

std::vector<uint32_t> RoadGraph::recordsInBox(float x0, float z0, float x1, float z1) const {
    // 0x4ae680: metres -> 2 m units, rounded to nearest, clamped to s16.
    auto u2 = [](float v) { return int(std::clamp(std::floor(v * 0.5f + 0.5f), -32768.0f, 32767.0f)); };
    const int bx0 = u2(x0), bz0 = u2(z0), bx1 = u2(x1), bz1 = u2(z1);
    const int cx0 = std::clamp(ftol((x0 - gx0_) * invX_), 0, 127), cx1 = std::clamp(ftol((x1 - gx0_) * invX_), 0, 127);
    const int cz0 = std::clamp(ftol((z0 - gz0_) * invZ_), 0, 127), cz1 = std::clamp(ftol((z1 - gz0_) * invZ_), 0, 127);
    std::vector<uint32_t> out;
    std::vector<uint8_t> seen(recs_.size(), 0);
    for (int cz = cz0; cz <= cz1; ++cz)
        for (int cx = cx0; cx <= cx1; ++cx) {
            const size_t c = size_t(cz * 128 + cx);
            for (uint32_t i = cellStart_[c]; i < cellStart_[c + 1] && out.size() < 20000; ++i) {
                const uint32_t ri = items_[i];
                const Rec& R = recs_[ri];
                if (seen[ri] || (R.flags & kExcluded)) continue;
                seen[ri] = 1;
                if (R.maxX <= bx1 && R.maxZ <= bz1 && R.minX >= bx0 && R.minZ >= bz0) out.push_back(ri);
            }
        }
    return out;
}

bool RoadGraph::loopEligible(uint32_t r) const {
    const Rec& R = recs_[r];
    if ((R.flags & kRigid) || R.type <= 5 || R.type == 13 || R.type == 14 || R.type == 15) return false;
    const auto& a = pts_[R.first];
    const auto& b = pts_[R.first + R.n - 1];
    return !(a[0] == b[0] && a[1] == b[1]);  // Record_IsClosed 0x6d7680 (raw quarter-metre points)
}

uint64_t RoadGraph::endKey(uint32_t r, bool end1) const {
    const auto& p = pts_[recs_[r].first + (end1 ? recs_[r].n - 1u : 0u)];
    return (uint64_t(uint32_t(p[0])) << 32) | uint32_t(p[1]);
}

int RoadGraph::loopDfs(std::unordered_map<uint64_t, LoopNode>& nodes, uint64_t cur, uint64_t prev, uint64_t start, const float box[4], float acc,
                       float& best) const {
    LoopNode& c = nodes[cur];
    c.state = 1;
    const uint32_t parent = c.rec;
    uint32_t r = parent;
    int side = c.dir ? 1 : 0, ret = 0;
    for (int guard = 0; guard < 4096; ++guard) {
        const Rec& R = recs_[r];
        const uint32_t next = side ? R.link1 : R.link0;
        side = side ? R.side1 : R.side0;
        r = next;
        if (r == parent) break;
        // RoadRouter_Node 0x6d7bb0: the node at r's far end (created even for records that are not followed).
        const bool far1 = side == 0;
        const uint64_t key = endKey(r, far1);
        auto it = nodes.find(key);
        if (it == nodes.end()) {
            if (nodes.size() >= 9999) continue;  // the router's node capacity
            it = nodes.emplace(key, LoopNode{r, far1, 0}).first;
        }
        if (!loopEligible(r)) continue;
        const float len = lengthCost(r, 0, recs_[r].n);
        if (it->second.state == 0) {
            const float px = float(int32_t(key >> 32)) * 0.25f, pz = float(int32_t(uint32_t(key))) * 0.25f;
            if (px < box[0] || px > box[2] || pz < box[1] || pz > box[3]) {
                it->second.state = 1;
            } else if (const int k = loopDfs(nodes, key, cur, start, box, acc + len, best); k != 0) {
                ret = k + 1;
                LoopNode& n = nodes[key];
                n.rec = r, n.dir = far1;
            }
        } else if (key != prev && key == start && acc + len > best) {
            best = acc + len;
            LoopNode& s = nodes[start];
            s.rec = r, s.dir = far1;
            ret = 1;
        }
    }
    return ret;
}

bool RoadGraph::findLoop(float x0, float z0, float x1, float z1, Loop& out) const {
    out = Loop{};
    const float box[4] = {x0, z0, x1, z1};
    const std::vector<uint32_t> cand = recordsInBox(x0, z0, x1, z1);
    std::unordered_map<uint64_t, LoopNode> nodes;
    int bestC = 0;
    int bestI = -1;
    auto run = [&](uint32_t i) {
        nodes.clear();
        const uint64_t start = endKey(i, true);
        nodes.emplace(start, LoopNode{i, true, 0});
        float best = 0;
        return loopDfs(nodes, start, ~0ull, start, box, 0, best);
    };
    for (uint32_t i : cand) {
        if (!loopEligible(i)) continue;
        const int c = run(i);
        if (c > bestC) bestC = c, bestI = int(i);
        if (c >= 10) break;
    }
    if (bestC < 3 || bestI < 0) return false;
    run(uint32_t(bestI));
    const uint64_t start = endKey(uint32_t(bestI), true);
    uint64_t node = start;
    do {
        const LoopNode& n = nodes[node];
        out.recs.push_back(n.rec);
        out.length += lengthCost(n.rec, 0, recs_[n.rec].n);
        node = endKey(n.rec, !n.dir);
    } while (node != start && out.recs.size() < 256);
    if (node != start) {
        out = Loop{};
        return false;
    }
    return true;
}

std::array<float, 2> RoadGraph::loopPoint(const Loop& loop, float s, float* dirX, float* dirZ, int* rec) const {
    const size_t n = loop.recs.size();
    if (n == 0) return {0, 0};
    // The record holding s (records summed in list order), walked from its end that does not touch the next one.
    size_t j = 0;
    float start = 0;
    for (; j + 1 < n; ++j) {
        const uint32_t r = loop.recs[j];
        const float len = lengthCost(r, 0, recs_[r].n);
        if (s < start + len) break;
        start += len;
    }
    const uint32_t r = loop.recs[j];
    const uint32_t nb = loop.recs[(j + 1) % n];
    const bool fwd = inEnd1Ring(r, nb);
    const int cnt = int(recs_[r].n);
    int k = fwd ? 0 : cnt - 1;
    const int step = fwd ? 1 : -1;
    float d = s - start, segStart = 0;
    std::array<float, 2> a = point(r, uint32_t(k)), b = a;
    float ux = 0, uz = 0;
    while (true) {
        const int kn = k + step;
        if (kn < 0 || kn >= cnt) break;  // the record's end: extrapolate along the last segment
        b = point(r, uint32_t(kn));
        const float l = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (l > 1e-6f) ux = (b[0] - a[0]) / l, uz = (b[1] - a[1]) / l;
        if (segStart + l >= d) break;
        segStart += l;
        k = kn;
        a = b;
    }
    if (dirX) *dirX = ux;
    if (dirZ) *dirZ = uz;
    if (rec) *rec = int(r);
    return {a[0] + ux * (d - segStart), a[1] + uz * (d - segStart)};
}

}  // namespace fuel
