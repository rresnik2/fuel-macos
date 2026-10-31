#include "road_segments.hpp"

#include "ground.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr float kCell = 64.0f;  // port lookup grid for records

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("road data too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

uint64_t cellKey(int32_t cx, int32_t cz) { return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cz); }

// x87 `fistp` in FUEL's game threads rounds down (p21 H0: _RC_DOWN), after a float store.
inline int32_t rint(double v) { return int32_t(std::floor(double(float(v)))); }
inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
inline double clamp01d(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
// S2(v) = 0.5 + 0.5 v (2 - |v|)
inline double s2(double v) { return 0.5 + 0.5 * v * (2.0 - std::fabs(v)); }

}  // namespace

RoadTypeTable loadRoadTypes(const DpcFile& world) {
    const auto palette = loadGroundPalette(world);
    const std::string exe =
        (std::filesystem::path(world.path()).parent_path().parent_path().parent_path() / "XFinalComponent" / "FUEL.exe").string();
    // The game runs the init more than once before roads are built: with the first run's
    // values (+0xb8 still 0) no road would paint or carve at all (notes p22, Open).
    return buildRoadTypes(
        exe,
        [&](int slot) -> std::array<float, 4> {
            if (slot < 0 || size_t(slot) >= palette.size()) return {0, 0, 0, 0};
            return palette[size_t(slot)];
        },
        true);
}

float RoadSampler::Segment::heightAt(float t, const RoadType& T) const {
    // 0x6f1800: samples every 1/T[1dc] m, linear between them, extrapolated past the ends.
    const float fn = T.at(0x1dc) * len;
    float ext = 0;
    if (t < 0) {
        ext = t * fn;
        t = 0;
    } else if (t > 1) {
        ext = (t - 1) * fn;
        t = 1;
    }
    const float x = t * fn;
    int32_t i = int32_t(std::floor(x));
    const float f = x - float(i);
    i = std::clamp<int32_t>(i, 0, int32_t(h.size()) - 2);
    const float h0 = float(base) + 0.01f * float(h[size_t(i)]), h1 = float(base) + 0.01f * float(h[size_t(i) + 1]);
    return (h1 - h0) * ext + (1 - f) * h0 + f * h1;
}

RoadSampler::RoadSampler(const DpcFile& world, RoadTypeTable types, HeightFn rawHeight)
    : types_(std::move(types)), alt_(std::move(rawHeight)) {
    if (!types_.valid) throw std::runtime_error("road type table not available");
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    const DpcFile::Entry* re = world.find(rd32(*gen, 8));
    const auto body = re ? world.body(*re) : std::nullopt;
    if (!body) throw std::runtime_error("GwRoad_Z not found");
    const std::vector<uint8_t>& b = *body;

    // GwRoad_Z::Load 0x7c41b0 + insert 0x6d0990: type byte (bit 7 = rigid), points in
    // quarter metres; consecutive duplicate points dropped; bbox in 2 m cells.
    const uint32_t n = rd32(b, 0);
    size_t p = 20;
    recs_.reserve(n);
    std::vector<std::array<int32_t, 2>> q;
    std::vector<std::array<int32_t, 2>> ends;  // quarter-metre end points per record (start, end)
    ends.reserve(size_t(n) * 2);
    for (uint32_t i = 0; i < n; ++i) {
        if (p + 3 > b.size()) throw std::runtime_error("GwRoad_Z record overruns");
        Record r{};
        r.type = b[p] & 0x7F;
        r.rigid = (b[p] & 0x80) != 0;
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
            pts_.push_back({float(q[k][0]) * 0.25f, float(q[k][1]) * 0.25f});
        }
        r.count = uint16_t(pts_.size() - r.first);
        r.link0 = r.link1 = i;
        r.side0 = 0, r.side1 = 1;
        ends.push_back(q.front());
        ends.push_back(q.back());
        recs_.push_back(r);
    }

    // Graph links 0x6d0fc0: records from last to first; each end joins the ring of the
    // first record end seen at exactly that point (inserted after it).
    std::unordered_map<uint64_t, std::pair<uint32_t, uint8_t>> nodes;
    nodes.reserve(size_t(n) * 2);
    for (int64_t i = int64_t(n) - 1; i >= 0; --i) {
        Record& r = recs_[size_t(i)];
        for (int end = 0; end < 2; ++end) {
            const auto& e = ends[size_t(i) * 2 + size_t(end)];
            const uint64_t key = (uint64_t(uint32_t(e[0])) << 32) | uint32_t(e[1]);
            auto [it, fresh] = nodes.try_emplace(key, uint32_t(i), uint8_t(end));
            uint32_t& myLink = end ? r.link1 : r.link0;
            uint8_t& mySide = end ? r.side1 : r.side0;
            if (fresh) {
                myLink = uint32_t(i);
                mySide = uint8_t(end);
                continue;
            }
            Record& o = recs_[it->second.first];
            uint32_t& oLink = it->second.second ? o.link1 : o.link0;
            uint8_t& oSide = it->second.second ? o.side1 : o.side0;
            myLink = oLink;
            mySide = oSide;
            oLink = uint32_t(i);
            oSide = uint8_t(end);
        }
    }

    // Port lookup: records by 64 m cells, over their bbox grown by the influence half-width.
    for (uint32_t i = 0; i < n; ++i) {
        const Record& r = recs_[i];
        if (r.type > 20) continue;
        const RoadType& T = types_.types[r.type];
        if (!T.byte(0x1f9) || !T.byte(0x1fb)) continue;  // not built by AGW_TerraRoad (0x6fa8a0)
        const float g = std::max(0.0f, T.at(0x00)) + 4.0f;
        const int32_t x0 = int32_t(std::floor((float(r.minX) * 2 - g) / kCell)), x1 = int32_t(std::floor((float(r.maxX) * 2 + g) / kCell));
        const int32_t z0 = int32_t(std::floor((float(r.minZ) * 2 - g) / kCell)), z1 = int32_t(std::floor((float(r.maxZ) * 2 + g) / kCell));
        for (int32_t cx = x0; cx <= x1; ++cx)
            for (int32_t cz = z0; cz <= z1; ++cz) cells_[cellKey(cx, cz)].push_back(i);
    }
    state_.reset(new std::atomic<uint8_t>[n]);
    for (uint32_t i = 0; i < n; ++i) state_[i].store(0, std::memory_order_relaxed);
    segs_.resize(n);

    const uint32_t seed = genWorldSeed(world);
    const Permutation perm = buildPermutation(world, seed);
    f3f54_ = Fractal(perm, 1, 10.0f, 25.148701f, 0.3f, 0.3f, 5.0f);
    f4f28_ = Fractal(perm, 4, 50.0f, 29.409878f, 0.3f, 0.3f, 5.0f);
    f11cec_ = Fractal(perm, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f);
    f5efc_ = Fractal(perm, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f);
}

RoadSampler::~RoadSampler() = default;

float RoadSampler::edgeJitter(double x, double z) const { return 0.5f * (f11cec_.at(x, z) + f5efc_.at(x, z)); }
float RoadSampler::dirtBumps(double x, double z) const {
    const float v = clamp01((f3f54_.at(x, z) - 1.1f) * 10.0f);
    return (2 - v) * v;
}
float RoadSampler::roughNoise(double x, double z) const { return clamp01(float(s2(2.0 * f4f28_.at(x, z) - 1.0))); }

RoadSampler::P2 RoadSampler::point(uint32_t rec, uint32_t k) const { return pts_[recs_[rec].first + k]; }

float RoadSampler::recordLength(uint32_t rec) const {
    // 0x4ae340: Euclidean length of the packed points.
    const Record& r = recs_[rec];
    double s = 0;
    for (uint32_t k = 1; k < r.count; ++k) {
        const P2 a = point(rec, k - 1), c = point(rec, k);
        s += std::sqrt(double(float((c[0] - a[0]) * 4) * float((c[0] - a[0]) * 4) + float((c[1] - a[1]) * 4) * float((c[1] - a[1]) * 4)));
    }
    return float(s * 0.25);
}

void RoadSampler::ring(uint32_t rec, int end, std::vector<std::pair<uint32_t, int>>& out) const {
    // 0x6d12b0: the records meeting at a node, starting with `rec`, at most 16.
    out.clear();
    uint32_t r = rec;
    int e = end;
    do {
        out.push_back({r, e});
        const Record& R = recs_[r];
        const uint32_t nr = e ? R.link1 : R.link0;
        e = e ? R.side1 : R.side0;
        r = nr;
    } while (r != rec && out.size() < 16);
}

bool RoadSampler::inEnd1Ring(uint32_t a, uint32_t b) const {
    // 0x6d77c0: walks a's end-1 ring; true when b is met before coming back to a.
    uint32_t r = recs_[a].link1;
    int e = recs_[a].side1;
    for (int guard = 0; guard < 4096 && r != a; ++guard) {
        if (r == b) return true;
        const Record& R = recs_[r];
        const uint32_t nr = e ? R.link1 : R.link0;
        e = e ? R.side1 : R.side0;
        r = nr;
    }
    return false;
}

std::pair<int64_t, int64_t> RoadSampler::pairAtNode(uint32_t rec, int end) const {
    // 0x6f6df0: the two records a road continues through at a node: the highest
    // continuation priority T[a4] among chainable, non-rigid records; if more than two
    // share it, the pair whose leaving directions are most opposite.
    std::vector<std::pair<uint32_t, int>> R;
    ring(rec, end, R);
    float pmax = 0;
    int64_t arg = -1;
    for (const auto& [r, e] : R) {
        const Record& rr = recs_[r];
        if (rr.type > 20) continue;
        const RoadType& T = types_.types[rr.type];
        if (!T.byte(0x1fb) || rr.rigid) continue;
        if (T.at(0xa4) > pmax) pmax = T.at(0xa4), arg = r;
    }
    if (arg < 0) return {-1, -1};  // engine reads record -1 here (undefined); no candidates
    const float pa = types_.types[recs_[size_t(arg)].type].at(0xa4);
    std::vector<uint32_t> cand;
    for (const auto& [r, e] : R)
        if (recs_[r].type <= 20 && types_.types[recs_[r].type].at(0xa4) == pa) cand.push_back(r);
    if (cand.size() == 2) return {cand[0], cand[1]};
    if (cand.size() < 2) return {-1, -1};
    std::vector<P2> d(cand.size());
    for (size_t i = 0; i < cand.size(); ++i) {
        const uint32_t c = cand[i];
        const uint32_t n = recs_[c].count;
        const bool atEnd1 = c == rec ? end != 0 : inEnd1Ring(c, rec);
        P2 v = atEnd1 ? P2{point(c, n - 2)[0] - point(c, n - 1)[0], point(c, n - 2)[1] - point(c, n - 1)[1]}
                      : P2{point(c, 1)[0] - point(c, 0)[0], point(c, 1)[1] - point(c, 0)[1]};
        const float l2 = v[0] * v[0] + v[1] * v[1];
        if (l2 > 1e-6f) {
            const float l = std::sqrt(l2);
            v = {v[0] / l, v[1] / l};
        }
        d[i] = v;
    }
    float best = 100000;
    std::pair<int64_t, int64_t> out{-1, -1};
    for (size_t i = 0; i < cand.size(); ++i)
        for (size_t j = i + 1; j < cand.size(); ++j) {
            const float dot = d[i][0] * d[j][0] + d[i][1] * d[j][1];
            if (dot < best) best = dot, out = {cand[i], cand[j]};
        }
    return out;
}

double RoadSampler::altAlong(uint32_t r, uint32_t ref, double x) const {
    // 0x6f64b0: height x metres into record r from the end it shares with `ref`.
    const uint32_t n = recs_[r].count;
    const bool back = r != ref && inEnd1Ring(r, ref);
    auto pt = [&](uint32_t k) { return point(r, back ? n - 1 - k : k); };
    P2 a = pt(0), c = pt(std::min<uint32_t>(1, n - 1));
    for (uint32_t k = 0; k + 1 < n; ++k) {
        a = pt(k), c = pt(k + 1);
        const double e = std::hypot(double(c[0] - a[0]), double(c[1] - a[1]));
        if (x <= e || k + 2 >= n) {
            const double f = e > 0 ? x / e : 0;
            return alt_(a[0] + (c[0] - a[0]) * f, a[1] + (c[1] - a[1]) * f);
        }
        x -= e;
    }
    return alt_(a[0], a[1]);
}

bool RoadSampler::junctionHeight(uint32_t rec, int end, double& h) const {
    // 0x6f65c0: mean height along the two continuing records near the node.
    const auto [a, b] = pairAtNode(rec, end);
    if (a < 0 || b < 0) return false;
    const RoadType& Ta = types_.types[recs_[size_t(a)].type];
    const int32_t n = rint(std::max(Ta.at(0x1f0) * Ta.at(0x1e4), Ta.at(0x1e8)) * Ta.at(0x1dc));
    if (n <= 0) return false;
    double sum = 0, x = 0.5 * Ta.at(0x1e0);
    for (int32_t k = 0; k < n; ++k, x += Ta.at(0x1e0)) sum += altAlong(uint32_t(a), rec, x) + altAlong(uint32_t(b), rec, x);
    h = sum / (2.0 * n);
    return true;
}

void RoadSampler::setEndpoints(Segment& s, P2 a, P2 b, float w) const {
    // 0x6f10c0.
    s.ax = a[0], s.az = a[1], s.bx = b[0], s.bz = b[1];
    const float dx = b[0] - a[0], dz = b[1] - a[1];
    s.len = std::sqrt(dx * dx + dz * dz);
    if (s.len > 0) {
        s.dx = dx / s.len, s.dz = dz / s.len;
        s.tx = s.dx / s.len, s.tz = s.dz / s.len;
    }
    s.c30 = s.dz * a[0] - s.dx * a[1];
    s.c34 = s.tx * a[0] + s.tz * a[1];
    s.w = w;
}

namespace {

// 0x6fa7a0 one end: the end tangent u (unit) against the segment direction d.
int8_t endTangent(float dx, float dz, float ux, float uz, bool start) {
    const float a = dx * ux + dz * uz;
    float c = dz * ux - uz * dx;
    const float m = std::max(std::fabs(c), a);
    if (m > 1e-6f) c *= 0.33333334f / m;
    return int8_t(rint(c * (start ? -3.0f : 3.0f) * 120.0f + 0.5f));
}

}  // namespace

void RoadSampler::build(uint32_t rec, std::vector<Segment>& out) const {
    // 0x6f9680 (roadbuild.md sec. 4-10).
    const Record& main = recs_[rec];
    const RoadType& Tm = types_.types[main.type];
    const float C = Tm.at(0x1f0), rs = Tm.at(0x1dc), step = Tm.at(0x1e0);

    // Chain gather (0x6f66f0).
    Chain ch;
    ch.recs.push_back({rec, true});
    const double marginG = double(Tm.at(0x1e4)) * C + 200.0;
    for (int dir = 0; dir < 2; ++dir) {
        uint32_t cur = rec;
        int end = dir == 0 ? 0 : 1;
        double sum = 0;
        for (int k = 0; k < 20; ++k) {
            const auto [a, b] = pairAtNode(cur, end);
            int64_t nxt = -1;
            if (int64_t(cur) == a) nxt = b;
            else if (int64_t(cur) == b) nxt = a;
            else break;
            if (nxt < 0 || uint32_t(nxt) == rec) break;
            const bool atEnd1 = inEnd1Ring(uint32_t(nxt), cur);
            if (dir == 0) {
                ch.recs.insert(ch.recs.begin(), {uint32_t(nxt), atEnd1});
                ++ch.main;
            } else {
                ch.recs.push_back({uint32_t(nxt), !atEnd1});
            }
            end = atEnd1 ? 0 : 1;
            cur = uint32_t(nxt);
            sum += recordLength(cur);
            if (sum >= marginG) break;
        }
    }

    // Point assembly (0x6f71f0), with its quirk for reversed records.
    std::vector<P2> p;
    size_t s0 = 0;
    for (size_t i = 0; i < ch.recs.size(); ++i) {
        const auto [r, fwd] = ch.recs[i];
        const uint32_t n = recs_[r].count;
        uint32_t lo = 0, hi = n - 1;
        if (i < ch.main) hi = n - 2;
        else if (i > ch.main) lo = 1;
        if (i == ch.main) s0 = p.size();
        if (n < 1 || hi < lo || hi >= n) continue;
        if (fwd)
            for (uint32_t k = lo; k <= hi; ++k) p.push_back(point(r, k));
        else
            for (int64_t k = hi; k >= int64_t(lo); --k) p.push_back(point(r, uint32_t(k)));
    }
    size_t s = s0, e = s0 + main.count - 1;
    if (p.size() < 2 || e >= p.size()) return;

    // Planimetric smoothing (0x6f8f30).
    auto simplify = [&](float tol) {
        const size_t n = p.size();
        if (n <= 2 || e - s <= 2) return;
        std::vector<P2> o;
        P2 A = p[0];
        size_t removed = 0;
        const size_t e0 = e;
        for (size_t i = 0; i + 2 < n; ++i) {
            bool tail = true;
            if (i <= s) {
                o.push_back(p[i]);
                A = p[i];
                if (i < s) tail = true;
            }
            if (i >= s && i + 2 <= e0) {
                const float dx = p[i + 2][0] - A[0], dz = p[i + 2][1] - A[1];
                if (dx * dx + dz * dz < 100.0f) {
                    ++removed;
                } else {
                    const float L = std::sqrt(dx * dx + dz * dz);
                    const float mx = p[i + 1][0] - A[0], mz = p[i + 1][1] - A[1];
                    const float dev = std::fabs((dx * mz - dz * mx) / L);
                    const float thr = std::min(L * tol, 1.0f);
                    if (dev <= thr) ++removed;
                    else o.push_back(p[i + 1]), A = p[i + 1];
                }
            }
            if (tail && i + 2 >= e0) o.push_back(p[i + 2]), A = p[i + 2];
        }
        p.swap(o);
        e -= removed;
    };
    auto densify = [&] {
        std::vector<P2> o;
        size_t ns = s, ne = e;
        for (size_t i = 0; i < p.size(); ++i) {
            o.push_back(p[i]);
            if (i + 1 >= p.size()) break;
            const float dx = p[i + 1][0] - p[i][0], dz = p[i + 1][1] - p[i][1];
            const int32_t k = rint(std::sqrt(dx * dx + dz * dz) * 0.05f);
            for (int32_t j = 1; j <= k; ++j) {
                const float f = float(j) / float(k + 1);
                o.push_back({p[i][0] + dx * f, p[i][1] + dz * f});
            }
            if (k > 0) {
                if (i < s) ns += size_t(k);
                if (i < e) ne += size_t(k);
            }
        }
        p.swap(o);
        s = ns, e = ne;
    };
    auto smooth = [&] {
        if (p.size() < 4) return;
        std::vector<P2> o = p;
        for (size_t i = 1; i + 1 < p.size(); ++i) {
            const float qx = 0.25f * (p[i - 1][0] + 2 * p[i][0] + p[i + 1][0]), qz = 0.25f * (p[i - 1][1] + 2 * p[i][1] + p[i + 1][1]);
            const float dx = qx - p[i][0], dz = qz - p[i][1], d2 = dx * dx + dz * dz;
            if (d2 > 4.0f) {
                const float k = 2.0f / std::sqrt(d2);
                o[i] = {p[i][0] + dx * k, p[i][1] + dz * k};
            } else {
                o[i] = {qx, qz};
            }
        }
        p.swap(o);
    };
    if (!main.rigid) {
        simplify(0.1f);
        densify();
        smooth();
        simplify(0.025f);
    }
    const size_t n = p.size();
    if (n < 2 || e >= n) return;

    // Arc lengths (0x6f7160).
    double Tlen = 0, A0 = 0, A1 = 0;
    for (size_t k = 0; k + 1 < n; ++k) {
        if (k == s) A0 = Tlen;
        Tlen += std::hypot(double(p[k + 1][0] - p[k][0]), double(p[k + 1][1] - p[k][1]));
        if (k + 1 == e) A1 = Tlen;
    }
    if (s == 0) A0 = 0;
    if (Tlen < 0.001) return;
    const double L = A1 - A0;

    // Height profile (0x6f8a00): raw terrain every `step` m along the chain.
    double M = std::max(double(C) * Tm.at(0x1e4), double(Tm.at(0x1e8)));
    M = std::min(M, 0.25 * Tlen);
    const int32_t N = rint(Tlen * rs) + 3 + 2 * rint(M * rs);
    std::vector<double> H(size_t(std::max(N, 0)), 0.0);
    {
        size_t k = 0;
        double edgeStart = 0;
        auto edgeLen = [&](size_t i) { return std::hypot(double(p[i + 1][0] - p[i][0]), double(p[i + 1][1] - p[i][1])); };
        for (int32_t i = 0; i < N; ++i) {
            const double sArc = -M + double(i) * step;
            double local = sArc - edgeStart;
            const double el = edgeLen(k);
            if (local > el && k + 2 < n) {
                local -= el;
                edgeStart += el;
                ++k;
            }
            const double l2 = edgeLen(k);
            double ux = 0, uz = 0;
            if (l2 > 0) ux = (p[k + 1][0] - p[k][0]) / l2, uz = (p[k + 1][1] - p[k][1]) / l2;
            if (sArc >= A0 - M - 10 && sArc <= A1 + M + 10) H[size_t(i)] = alt_(p[k][0] + ux * local, p[k][1] + uz * local);
        }
    }
    // End heights and blend (0x6f81f0).
    {
        const auto& f = ch.recs.front();
        const auto& l = ch.recs.back();
        double h0, h1;
        if (!junctionHeight(f.first, f.second ? 0 : 1, h0)) h0 = alt_(p[0][0], p[0][1]);
        if (!junctionHeight(l.first, l.second ? 1 : 0, h1)) h1 = alt_(p[n - 1][0], p[n - 1][1]);
        auto blendDist = [&](int end) {
            std::vector<std::pair<uint32_t, int>> R;
            ring(rec, end, R);
            float w = 0;
            for (const auto& [r, ee] : R)
                if (recs_[r].type <= 20) w = std::max(w, types_.types[recs_[r].type].at(0x70));
            return double(w);
        };
        const double w0 = blendDist(0), w1 = blendDist(1);
        const double d0 = std::min(w0 + std::min(w0, 1.0) * M, 0.33 * Tlen), d1 = std::min(w1 + std::min(w1, 1.0) * M, 0.33 * Tlen);
        const double E = std::max(std::min(Tlen - (d0 + d1), std::max(M, 50.0)), 0.001);
        const double kLin = Tm.at(0xa0);
        auto W = [](double u) { return s2(2 * clamp01d(u) - 1); };
        for (int32_t i = 0; i < N; ++i) {
            const double sa = -M + double(i) * step;
            const double a0 = W(1 - (sa - d0) / E), a1 = W(1 - (Tlen - sa - d1) / E);
            const double tc = clamp01d((sa - M) / Tlen);
            const double Lh = (1 - tc) * h0 + tc * h1;
            double& h = H[size_t(i)];
            h = h * (1 - kLin) + Lh * kLin;
            h = h * (1 - a0) + a0 * h0;
            h = h * (1 - a1) + a1 * h1;
        }
    }
    // Convolution (0x6f85c0) and bumps (0x6f8900, 0x6f8850).
    const int32_t i0 = rint(rs * A0), cnt = rint(rs * L) + 3, rr = rint(M * rs);
    std::vector<double> wk(size_t(2 * rr + 1));
    double S = 0;
    for (int32_t j = 0; j <= 2 * rr; ++j) {
        const double x = double((rr + 1) - std::abs(rr - j)) / double(rr + 1);
        wk[size_t(j)] = x * (2 - std::fabs(x));
        S += wk[size_t(j)];
    }
    std::vector<double> prof(size_t(std::max(cnt, 0)));
    for (int32_t k = 0; k < cnt; ++k) {
        double a = 0;
        for (int32_t j = 0; j <= 2 * rr; ++j) {
            const int32_t idx = i0 + k + j;
            if (idx >= 0 && idx < N) a += wk[size_t(j)] * H[size_t(idx)];
        }
        double v = a * (1.0 / S);
        const double sa = A0 + double(k) * step;
        if (Tm.at(0x7c) != 0 && Tm.at(0x80) != 0) {
            const double f = sa * 6.28 / Tm.at(0x80);
            v += (std::sin(f) + (std::sin(0.390933 * f) + std::sin(1.839735 * f))) * 0.33 * Tm.at(0x7c) * 0.5;
        }
        if (Tm.at(0x74) != 0) {
            const double f = sa * 6.28 / Tm.at(0x78);
            const double q = (std::sin(0.390933 * f) + 1) * 0.5;
            v += q * q * q * q * Tm.at(0x74);
        }
        prof[size_t(k)] = v;
    }

    // Cuts at T-junctions where the road stops (0x6f6a00 / 0x6f6960).
    double startW = 10, endW = 10;
    int startF = 0, endF = 0;
    if (ch.main == 0 && Tm.at(0x38) <= 0) {
        const auto [a, b] = pairAtNode(rec, 0);
        if (a >= 0 && b >= 0 && a != int64_t(rec) && b != int64_t(rec)) startF = 1, startW = types_.types[recs_[size_t(a)].type].at(0xcc);
    }
    if (ch.main + 1 == ch.recs.size() && Tm.at(0x38) <= 0) {
        const auto [a, b] = pairAtNode(rec, 1);
        if (a >= 0 && b >= 0 && a != int64_t(rec) && b != int64_t(rec)) endF = 1, endW = types_.types[recs_[size_t(a)].type].at(0xcc);
    }
    const double Rj = std::max(5.0, 0.5 * Tm.at(0xcc));
    std::vector<double> cuts;
    if ((Rj + endW) * endF + (Rj + startW) * startF < L) {
        if (startF) cuts.push_back(startW), cuts.push_back(Rj + startW);
        if (endF) {
            const double c = (L - endW) - Rj;
            if ((cuts.empty() ? 0.0 : cuts.back()) < c) cuts.push_back(c), cuts.push_back(L - endW);
        }
    }
    const float Jm = Tm.at(0xcc);
    auto J = [&](double a) {
        const double u0 = clamp01d(std::fabs(a - startW) / Jm), u1 = clamp01d(std::fabs((L - a) - endW) / Jm);
        return clamp01d((1 - u1) * endF + (1 - u0) * startF);
    };
    const float three = 3.0f / C;

    auto emit = [&](P2 S2, P2 B, P2 tS, P2 tE, double Astart) {
        Segment g{};
        setEndpoints(g, S2, B, Tm.at(0x00));
        g.type = main.type;
        g.record = rec;
        g.start = uint16_t(rint((Astart + A0) * 10 + 0.5));
        // Height samples (0x6f16e0).
        const int32_t m = rint(g.len * rs) + 2;
        const int32_t j0 = rint(rs * Astart);
        auto profAt = [&](int32_t k) { return prof.empty() ? 0.0 : prof[size_t(std::clamp<int32_t>(k, 0, int32_t(prof.size()) - 1))]; };
        g.base = int16_t(rint(profAt(j0)));
        g.h.resize(size_t(std::max(m, 2)));
        for (int32_t j = 0; j < int32_t(g.h.size()); ++j) {
            int32_t v = rint((profAt(j0 + j) - g.base) * 100 + 0.5);
            if (v >= 0x7fff) v = 0x7fff;
            else if (v < -0x7fff) v = -0x8000;
            g.h[size_t(j)] = int16_t(v);
        }
        g.pri = Astart >= 75 ? (Astart + g.len <= L - 75 ? 1 : (ch.main != ch.recs.size() - 1)) : (ch.main != 0);
        const float lS = std::hypot(tS[0], tS[1]), lE = std::hypot(tE[0], tE[1]);
        g.tan0 = endTangent(g.dx, g.dz, lS > 0 ? tS[0] / lS : 0, lS > 0 ? tS[1] / lS : 0, true);
        g.tan1 = endTangent(g.dx, g.dz, lE > 0 ? tE[0] / lE : 0, lE > 0 ? tE[1] / lE : 0, false);
        // Curvature (0x6fa660).
        const float l2S = tS[0] * tS[0] + tS[1] * tS[1], l2E = tE[0] * tE[0] + tE[1] * tE[1];
        float qS = l2S > 1e-6f ? (tS[0] * g.dz - tS[1] * g.dx) / l2S : 0, qE = l2E > 1e-6f ? (tE[1] * g.dx - tE[0] * g.dz) / l2E : 0;
        qS = std::clamp(qS, -0.05f, 0.05f), qE = std::clamp(qE, -0.05f, 0.05f);
        g.curv0 = int8_t(rint(qS * 500 + 0.5f)), g.curv1 = int8_t(rint(qE * 500 + 0.5f));
        const int32_t jb0 = rint((J(Astart) * 0.5 + 1) * 50 + 0.5), jb1 = rint((J(Astart + g.len) * 0.5 + 1) * 50 + 0.5);
        g.wid0 = uint8_t(rint((float(jb0 & 0xff) * 0.02f + std::min(25.0f * std::fabs(float(g.curv0) * 0.002f), three)) * 50 + 0.5f));
        g.wid1 = uint8_t(rint((float(jb1 & 0xff) * 0.02f + std::min(25.0f * std::fabs(float(g.curv1) * 0.002f), three)) * 50 + 0.5f));
        out.push_back(std::move(g));
        return double(out.back().len);
    };

    // Emission (0x6f9680): walk the main record's part of the chain.
    P2 Sp = p[s];
    double A = 0;
    P2 tS = s == 0 ? P2{p[1][0] - p[0][0], p[1][1] - p[0][1]} : P2{p[s + 1][0] - p[s - 1][0], p[s + 1][1] - p[s - 1][1]};
    size_t ci = 0;
    for (size_t i = s; i < e; ++i) {
        const P2 P = p[i + 1];
        const P2 tE = (i + 2) < n - 1 ? P2{p[i + 2][0] - p[i][0], p[i + 2][1] - p[i][1]} : P2{p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]};
        while (ci < cuts.size()) {
            const double c = cuts[ci];
            if (!(0 < c && c < L)) {
                ++ci;
                continue;
            }
            const double d = std::hypot(double(P[0] - Sp[0]), double(P[1] - Sp[1]));
            if (A + d < c) break;
            const double f = d > 0 ? (c - A) / d : 0;
            const P2 Cp{float(Sp[0] + (P[0] - Sp[0]) * f), float(Sp[1] + (P[1] - Sp[1]) * f)};
            emit(Sp, Cp, tS, tE, A);
            A += std::hypot(double(Cp[0] - Sp[0]), double(Cp[1] - Sp[1]));
            Sp = Cp;
            ++ci;
        }
        emit(Sp, P, tS, tE, A);
        A += std::hypot(double(P[0] - Sp[0]), double(P[1] - Sp[1]));
        Sp = P;
        tS = tE;
    }
}

void RoadSampler::buildRigid(uint32_t rec, std::vector<Segment>& out) const {
    // 0x6f9150 + 0x6fda10: the record as is, one segment per edge, box-filtered heights.
    const Record& R = recs_[rec];
    const RoadType& T = types_.types[R.type];
    const float C = T.at(0x1f0), rs = T.at(0x1dc), step = T.at(0x1e0);
    const uint32_t n = R.count;
    if (n < 2) return;
    double A = 0;
    auto P = [&](uint32_t k) { return point(rec, k); };
    P2 tS{P(1)[0] - P(0)[0], P(1)[1] - P(0)[1]};
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const P2 a = P(i), b = P(i + 1);
        const P2 tE = (i + 2) < n - 1 ? P2{P(i + 2)[0] - P(i)[0], P(i + 2)[1] - P(i)[1]} : P2{b[0] - a[0], b[1] - a[1]};
        Segment g{};
        setEndpoints(g, a, b, T.at(0x00));
        g.type = R.type;
        g.record = rec;
        g.pri = 0;
        g.start = uint16_t(rint(A * 10 + 0.5));
        const float lS = std::hypot(tS[0], tS[1]), lE = std::hypot(tE[0], tE[1]);
        g.tan0 = endTangent(g.dx, g.dz, lS > 0 ? tS[0] / lS : 0, lS > 0 ? tS[1] / lS : 0, true);
        g.tan1 = endTangent(g.dx, g.dz, lE > 0 ? tE[0] / lE : 0, lE > 0 ? tE[1] / lE : 0, false);
        g.curv0 = g.curv1 = 0;
        g.wid0 = g.wid1 = 50;
        const double m = std::min(std::max(double(C) * T.at(0x1e4), double(T.at(0x1e8))), 0.25 * g.len);
        const int32_t cnt = rint(rs * g.len) + 2, r2 = 2 * rint(rs * m);
        std::vector<double> hr(size_t(std::max(cnt + r2, 1)));
        for (size_t k = 0; k < hr.size(); ++k) {
            const double sa = -m + double(k) * step;
            hr[k] = alt_(a[0] + g.dx * sa, a[1] + g.dz * sa);
        }
        g.base = int16_t(rint(hr[0]));
        g.h.resize(size_t(std::max(cnt, 2)));
        for (int32_t k = 0; k < int32_t(g.h.size()); ++k) {
            double sum = 0;
            int c = 0;
            for (int32_t j = k; j <= k + r2 && j < int32_t(hr.size()); ++j) sum += hr[size_t(j)], ++c;
            int32_t v = rint((sum / std::max(c, 1) - g.base) * 100 + 0.5);
            if (v >= 0x7fff) v = 0x7fff;
            else if (v < -0x7fff) v = -0x8000;
            g.h[size_t(k)] = int16_t(v);
        }
        out.push_back(std::move(g));
        A += out.back().len;
        tS = tE;
    }
}

const std::vector<RoadSampler::Segment>& RoadSampler::segmentsOf(uint32_t rec) const {
    if (state_[rec].load(std::memory_order_acquire) == 2) return segs_[rec];
    std::lock_guard<std::mutex> lock(locks_[rec & 63]);
    if (state_[rec].load(std::memory_order_acquire) != 2) {
        std::vector<Segment> v;
        if (recs_[rec].rigid) buildRigid(rec, v);
        else build(rec, v);
        segs_[rec] = std::move(v);
        state_[rec].store(2, std::memory_order_release);
    }
    return segs_[rec];
}

size_t RoadSampler::builtSegments() const {
    size_t c = 0;
    for (size_t i = 0; i < recs_.size(); ++i)
        if (state_[i].load(std::memory_order_acquire) == 2) c += segs_[i].size();
    return c;
}

void RoadSampler::segmentsNear(double x, double z, std::vector<const Segment*>& out) const {
    out.clear();
    const auto it = cells_.find(cellKey(int32_t(std::floor(x / kCell)), int32_t(std::floor(z / kCell))));
    if (it == cells_.end()) return;
    for (uint32_t r : it->second) {
        const RoadType& T = types_.types[recs_[r].type];
        const double reach = T.at(0x00);
        if (x < recs_[r].minX * 2.0 - reach - 4 || x > recs_[r].maxX * 2.0 + reach + 4 || z < recs_[r].minZ * 2.0 - reach - 4 ||
            z > recs_[r].maxZ * 2.0 + reach + 4)
            continue;
        for (const Segment& s : segmentsOf(r)) {
            const double minx = std::min(s.ax, s.bx) - s.w, maxx = std::max(s.ax, s.bx) + s.w;
            const double minz = std::min(s.az, s.bz) - s.w, maxz = std::max(s.az, s.bz) + s.w;
            if (x >= minx && x <= maxx && z >= minz && z <= maxz) out.push_back(&s);
            if (out.size() >= 128) return;
        }
    }
}

namespace {

// Shared per-segment terms of 0x6fbab0 / 0x6fcaa0 (notes R1).
struct Geo {
    float lateral, t, dist, c, off, sd, e, W, ws, en;
};

Geo geometry(const RoadSampler::Segment& s, double x, double z) {
    Geo g{};
    const float fx = float(x), fz = float(z);
    g.lateral = (s.dz * fx - s.dx * fz) - s.c30;
    g.t = (s.tx * fx + s.tz * fz) - s.c34;
    if (g.t <= 0) g.dist = std::hypot(fx - s.ax, fz - s.az);
    else if (g.t > 1) g.dist = std::hypot(fx - s.bx, fz - s.bz);
    else g.dist = std::fabs(g.lateral);
    g.c = clamp01(g.t);
    const float e64 = float(s.tan0) / 120.0f, e68 = float(s.tan1) / 120.0f;
    g.off = std::clamp((1 - g.c) * g.c * (g.c * e68 + (1 - g.c) * e64) * s.len, -4.0f, 4.0f);
    g.sd = (g.c > 0 && g.c < 1) ? g.lateral : (g.lateral <= 0 ? -g.dist : g.dist);
    g.e = std::fabs(g.off) > 1e-4f ? std::fabs(g.sd + g.off) : g.dist;
    g.W = float(s2(2.0 * g.c - 1.0));
    g.ws = (1 - g.W) * float(s.wid0) * 0.02f + g.W * float(s.wid1) * 0.02f;
    g.en = g.e / g.ws;
    return g;
}

}  // namespace

RoadFields RoadSampler::sample(double x, double z) const {
    RoadFields f;
    std::vector<const Segment*> near;
    segmentsNear(x, z, near);
    if (near.empty()) return f;
    std::stable_partition(near.begin(), near.end(), [](const Segment* s) { return (s->pri & 1) != 0; });

    float N = -1;  // Noise_EdgeJitter, evaluated once on demand
    float bumps = -1;
    float ruts = 1, r64 = 1, r68 = 1, qmin = 1, minEn = 100, minEnAll = 1000, sum = 0, maxPaint = 0;
    float bright[3] = {0, 0, 0};
    const RoadType* widest = nullptr;
    int64_t firstRec = -1, lastRec = -1, lastPriRec = -1;
    float bestPd = 1e7f, bestPriPd = 1e7f, fade = 1;
    bool ownerPri = false, any = false, inRange = false, before = false, after = false;
    float minAlong = 1e9f, maxAlong = -1e9f, dMin[2] = {0, 0}, dMax[2] = {0, 0};

    for (const Segment* sp : near) {
        const Segment& s = *sp;
        const RoadType& T = types_.types[s.type];
        Geo g = geometry(s, x, z);
        if (!(g.dist <= T.at(0xd4))) continue;
        f.inside = std::max(f.inside, T.at(0xbc) - g.dist);
        f.maxT60 = std::max(f.maxT60, T.at(0x60));
        f.minHeight = std::min(f.minHeight, s.heightAt(g.t, T));
        const float pd = g.e - std::fabs(g.lateral) * 0.0001f;
        if (T.at(0x5c) < g.en) {
            if (N < -0.5f) N = edgeJitter(x, z);
            g.en = (g.en - T.at(0x5c)) / ((T.at(0x90) - T.at(0x94)) * N + T.at(0x94)) + T.at(0x5c);
        }
        f.inside2 = std::max(f.inside2, T.at(0xcc) - g.en);
        if (!(g.en < T.at(0xbc))) continue;
        minEnAll = std::min(minEnAll, g.en);
        minEn = std::min(minEn, g.en);
        if (widest == nullptr || widest->at(0xbc) < T.at(0xbc)) widest = &T;
        const float along = float(s.start) * 0.1f + s.len * g.t;
        if (firstRec < 0) firstRec = s.record;
        if (firstRec != int64_t(s.record)) f.junction = 1;
        if (along < minAlong) minAlong = along, dMin[0] = s.dx, dMin[1] = s.dz;
        if (maxAlong < along) maxAlong = along, dMax[0] = s.dx, dMax[1] = s.dz;
        if (N < -0.5f) N = edgeJitter(x, z);
        // Paint: 1 inside the painted width (noise-varied), soft over +-2.75 m (0.18181819 = 1/5.5).
        const float pnt = clamp01((((T.at(0x9c) - T.at(0x98)) * float(s2(2.0 * N - 1.0)) + T.at(0x98)) * T.at(0xc8) - g.en + 2.75f) * 0.18181819f);
        if (pnt > 0.01f) {
            if (bumps < 0) bumps = dirtBumps(x, z);
            f.puddle += bumps * T.at(0x34) * pnt;
        }
        const float k24 = 0.5f / (T.at(0xcc) - 0.5f);
        const float v = (g.sd + g.off) * k24 + 0.5f;
        if (v > -0.4f && v < 1.4f && T.at(0x2c) < ruts) ruts = T.at(0x2c);
        float q = 0;
        if (T.at(0xcc) - 1.5f <= g.en) q = std::min(g.en - (T.at(0xcc) - 1.5f), 1.0f);
        if (T.u32(0xa8) != 0xfff) {
            const bool pri = (s.pri & 1) != 0;
            bool take = false;
            if (lastRec < 0) take = true;
            else if (lastRec == int64_t(s.record)) take = pd < bestPd;
            else if (pri) take = (v > -0.8f && v < 1.8f && !ownerPri) || pd < bestPd;
            else if (!ownerPri) take = pd < bestPd;
            f.tarmac = std::max(f.tarmac, (1 - q) * T.at(0x38));
            if (take) {
                if (v > -0.45f && v < 1.45f) {
                    if (v > 0 && v < 1) {
                        if (g.t < 0) before = true;
                        else if (g.t > 1) after = true;
                        else inRange = true;
                    }
                    f.across = v;
                    any = true;
                    f.along = T.at(0xb0) * k24 * along;
                    ownerPri = pri;
                }
                f.material = uint16_t(T.u32(0xa8));
                if (v <= 0 || v >= 1) {
                    f.decal = 0;
                    if (pri && pd < bestPriPd) {
                        bestPriPd = pd;
                        float xx = -1;
                        if (v < 0.5f || v >= 1.8f) {
                            if (!(v > 0.5f || v <= -0.8f)) xx = v + 0.8f;
                        } else {
                            xx = 1.8f - v;
                        }
                        if (xx >= 0) fade = clamp01(1 - xx * 4);
                    }
                } else if (!pri && lastPriRec != int64_t(s.record)) {
                    f.decal = fade;
                } else {
                    f.decal = 1;
                }
                lastRec = s.record;
                bestPd = pd;
                if (pri) lastPriRec = lastRec;
            }
        }
        qmin = std::min(qmin, q);
        r64 = std::min(r64, T.at(0x64));
        r68 = std::min(r68, T.at(0x68));
        // Road colour (lerp of the two table colours by the edge jitter), weighted by paint.
        for (int c = 0; c < 3; ++c) {
            const float k = (T.at(0x14 + 4 * uint32_t(c)) - T.at(0x04 + 4 * uint32_t(c))) * N + T.at(0x04 + 4 * uint32_t(c));
            f.colour[c] += k * pnt;
            bright[c] += T.at(0x3c + 4 * uint32_t(c)) * pnt;
        }
        sum += pnt;
        maxPaint = std::max(maxPaint, pnt);
    }
    if (!(any && (inRange || before == after))) f.decal = 0;
    if (f.decal < 0.01f) f.material = 0xfff;
    if (widest) {
        const float gg = widest->at(0xc4) - minEn;
        if (gg > 0 && gg < widest->at(0x50)) {
            const float xx = gg / widest->at(0x50);
            const float ff = 2 * (xx <= 0.5f ? 1 - 2 * xx : 2 * xx - 1) - 1;
            const float sv = float(s2(ff));
            f.berm = clamp01(2 * (1 - sv * sv) * widest->at(0x54));
        }
    }
    if (sum > 1e-7f) {
        const float inv = 1 / sum;
        for (int c = 0; c < 4; ++c) f.colour[c] *= inv;
        for (int c = 0; c < 3; ++c) f.colour[c] += (bright[c] * inv - f.colour[c]) * qmin;
        f.colour[3] += (0 - f.colour[3]) * qmin;
        f.puddle *= inv;
        if (f.junction < 0.5f && dMax[0] * dMin[0] + dMax[1] * dMin[1] < 0) f.junction = 1;
        if (ruts > 0.01f) maxPaint *= 1 - ruts * (std::cos(std::clamp(minEnAll * 2.5f, -6.28f, 6.28f)) + 1) * 0.5f;
        f.roughness = 1 - f.tarmac;
        f.roughness = r64 > 0.01f ? (roughNoise(x, z) * (r64 - r68) + r68) * f.roughness : 0;
        f.weight = maxPaint;
    }
    return f;
}

double RoadSampler::carve(double x, double z, double& y, double cover, double detailWeight) const {
    // 0x6fcaa0 (notes R3).
    std::vector<const Segment*> near;
    segmentsNear(x, z, near);
    if (near.empty()) return 0;
    double sumT = 0, sumW = 0, lift = 0, r = 0, blend = 0, minEn = 1000;
    float N = -1, bumps = -1;
    const RoadType* widest = nullptr;
    for (const Segment* sp : near) {
        const Segment& s = *sp;
        const RoadType& T = types_.types[s.type];
        Geo g = geometry(s, x, z);
        if (!(g.dist <= T.at(0x00))) continue;
        if (T.at(0x5c) < g.en) {
            if (N < -0.5f) N = edgeJitter(x, z);
            g.en = (g.en - T.at(0x5c)) / ((T.at(0x90) - T.at(0x94)) * N + T.at(0x94)) + T.at(0x5c);
        }
        const double Rb = double(T.at(0xbc)) / g.ws;
        if (!(Rb - g.en > 0)) continue;
        const double k = clamp01d((Rb - g.en) * T.at(0xc0) * g.ws);
        const double wgt = s2(2 * k - 1) * Rb;
        if (widest == nullptr || widest->at(0xbc) < T.at(0xbc)) widest = &T;
        minEn = std::min(minEn, double(g.en));
        double prof = 0;
        const double xc = T.at(0xcc) - 0.5 - g.e;
        if (xc > 0) {
            const double u = xc / T.at(0xcc);
            prof = (2 - u) * u * T.at(0xcc) * T.at(0x24);
        }
        const double y1 = T.at(0xcc) + 1.0 - g.e;
        if (y1 > 0) prof += std::min(y1 * 0.5, 1.0) * widest->at(0x28);
        lift = std::max(lift, prof);
        double rs = 0;
        if (g.en < T.at(0xc4) - 1.5) rs = 1;
        else if (g.en <= Rb - 0.5 * T.at(0x4c)) rs = clamp01d((Rb - T.at(0x4c) - g.en) / 1.5);
        r = std::max(r, rs);
        if (bumps < 0) bumps = dirtBumps(x, z);
        const double bank = (1 - g.W) * float(s.curv0) * 0.002 + g.W * float(s.curv1) * 0.002;
        const double target = s.heightAt(g.t, T) + ((g.off + g.lateral) * bank * T.at(0x8c) - detailWeight * 1.5) - bumps * T.at(0x34);
        sumT += target * wgt;
        sumW += wgt;
        blend = std::max(blend, s2(2 * clamp01d((Rb - g.en) / T.at(0x4c)) - 1));
    }
    if (!(sumW > 1e-7)) return 0;
    double Hr = sumT / sumW;
    if (widest) {
        const double gg = widest->at(0xc4) - minEn;
        if (gg > 0) {
            double dip = 0;
            if (gg < widest->at(0x50)) {
                const double xx = gg / widest->at(0x50);
                const double ff = 2 * (xx <= 0.5 ? 1 - 2 * xx : 2 * xx - 1) - 1;
                const double sv = s2(ff);
                dip = -(1 - sv * sv) * widest->at(0x54);
            }
            Hr += std::min(clamp01d(Hr - y - 0.5) + dip * cover, 0.0);
        }
        Hr += lift;
    }
    y = blend * Hr + (1 - blend) * y;
    return r;
}

}  // namespace fuel
