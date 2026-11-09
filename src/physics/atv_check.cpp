// --aicheck-vectors part for the off-road path finder (atv_path, P58): step cost 0x61e710 and smoothing 0x61f230
// against FUEL.exe run in Ghidra's emulator on synthetic cells (~/re-work/fuel/notes/p58-work/atv_gen.py; weights
// 1 / 0.35 off road, 1 / 0.9 on roads, box 0..20), and the search's invariants on a synthetic cliff.
#include "physics/atv_path.hpp"
#include "physics/race_ai.hpp"

#include <functional>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fuel {

namespace {
// Samples every 10 m turning by `turnDeg` per sample (as notes/p58-work/obs_gen.py builds them).
std::vector<AiSample> obsSamples(int n, double turnDeg, const std::function<std::pair<float, float>(int)>& lat) {
    std::vector<std::array<double, 2>> pts;
    double x = 0, z = 0, h = 0;
    for (int k = 0; k <= n; ++k) {
        pts.push_back({x, z});
        x += std::sin(h) * 10, z += std::cos(h) * 10, h += turnDeg * M_PI / 180.0;
    }
    std::vector<AiSample> S;
    double cum = 0;
    for (size_t k = 0; k < pts.size(); ++k) {
        double dx, dz, l;
        if (k + 1 < pts.size()) dx = pts[k + 1][0] - pts[k][0], dz = pts[k + 1][1] - pts[k][1], l = std::hypot(dx, dz), dx /= l, dz /= l;
        else dx = S.back().dx, dz = S.back().dz, l = double(S.back().seg) + 1.0;
        const auto [lo, hi] = lat(int(k));
        AiSample a;
        a.x = float(pts[k][0]), a.z = float(pts[k][1]), a.latMin = lo, a.latMax = hi, a.dx = float(dx), a.dz = float(dz), a.seg = float(l), a.cum = float(cum);
        if (k + 1 < pts.size()) cum += l;
        S.push_back(a);
    }
    return S;
}
}  // namespace

void atvCheckVectors(int& ok, int& bad) {
    auto check = [&](const char* id, const char* what, const std::vector<float>& got, const std::vector<float>& want, float tol) {
        bool m = got.size() == want.size();
        for (size_t k = 0; m && k < got.size(); ++k) m = std::fabs(got[k] - want[k]) <= tol * std::max(1.0f, std::fabs(want[k]));
        std::printf("%-4s %-40s %s  got", id, what, m ? "match   " : "MISMATCH");
        for (float g : got) std::printf(" %.7g", g);
        if (!m) {
            std::printf("  want");
            for (float w : want) std::printf(" %.7g", w);
        }
        std::printf("\n");
        (m ? ok : bad)++;
    };
    struct ACell { int x, z; float h, c, slope, forest; bool road; };
    struct ACase { const char* id; ACell a, b; bool diag; float want; };
    const ACase ac[] = {
        {"A1", {5, 7, 100, 1, 1, 0, false}, {5, 8, 100, 1, 1, 0, false}, false, 28.5714293f},
        {"A2", {6, 8, 103, 1, 1.3f, 0.3f, false}, {5, 7, 100, 1, 1.6f, 0.5f, false}, true, 135.311943f},
        {"A3", {5, 7, 97, 1, 1.2f, 0.4f, false}, {5, 8, 100, 1, 1.1f, 0.2f, false}, false, 40.4857157f},
        {"A4", {5, 7, 106, 1, 1, 0, false}, {5, 8, 100, 1, 1, 0, false}, false, -1.0f},
        {"A5", {5, 7, 106, 0.05f, 1, 0, true}, {5, 8, 100, 0.12f, 1, 0, true}, false, 18.4555569f},
        {"A6", {5, 7, 100, -1, 1, 0, false}, {5, 8, 100, 1, 1, 0, false}, false, -1.0f},
        {"A7", {5, 7, 101, 1, 99, 0, false}, {5, 8, 100, 1, 1.5f, 0, false}, false, -1.0f},
        {"A8", {20, 7, 100, 1, 1, 0, false}, {21, 7, 100, 1, 1, 0, false}, false, -1.0f},
        {"A9", {5, 7, 101, 0.3f, 1, 0, true}, {5, 8, 100, 1, 1.4f, 0.6f, false}, false, 30.9457791f},
        {"A10", {6, 8, 92.929f, 1, 1, 0, false}, {5, 7, 100, 1, 1, 0, false}, true, 50.5074213f},
    };
    for (const ACase& c : ac) {
        AtvPathFinder f({});
        f.setWeights(0.35f, 0.9f);
        f.setBoxBounds(0, 0, 20, 20);
        f.prime(c.a.x, c.a.z, c.a.h, {c.a.c, c.a.slope, c.a.forest, c.a.road});
        f.prime(c.b.x, c.b.z, c.b.h, {c.b.c, c.b.slope, c.b.forest, c.b.road});
        const float len = c.diag ? AtvPathFinder::kDiag : 10.0f, inv = c.diag ? AtvPathFinder::kInvDiag : 0.1f;
        check(c.id, "0x61e710 off-road step cost", {f.stepCost(c.a.x, c.a.z, c.b.x, c.b.z, inv, len)}, {c.want}, 2e-6f);
    }
    struct SCase { const char* id; std::vector<std::array<float, 3>> in; std::vector<float> want; };
    const SCase sc[] = {
        {"S1", {{0, 0, 0}, {10, 1, 0}, {10, 2, 10}, {20, 3, 10}, {30, 4, 30}, {40, 5, 30}},
         {0, 0, 0, 7.5f, 1, 2.5f, 12.5f, 2, 7.5f, 20, 3, 15, 30, 4, 25, 40, 5, 30}},
        {"S2", {{0, 0, 0}, {10, 0, 0}, {20, 0, 40}, {30, 0, 0}, {40, 0, 0}}, {0, 0, 0, 10, 0, 10, 20, 0, 25, 30, 0, 10, 40, 0, 0}},
        {"S3", {{0, 0, 0}, {10, 0, 5}, {20, 0, 0}}, {0, 0, 0, 10, 0, 5, 20, 0, 0}},
    };
    for (const SCase& c : sc) {
        auto p = c.in;
        AtvPathFinder::smooth(p);
        std::vector<float> got;
        for (const auto& q : p) got.insert(got.end(), q.begin(), q.end());
        check(c.id, "0x61f230 path smoothing", got, c.want, 1e-6f);
    }
    // Racer obstacles (race_ai, P58): 0x6189b0 box projection and 0x61a9d0 propagation, emulated
    // (notes/p58-work/obs_gen.py; half-width 1.3, margin 2.0). Records: lo hi s0 s1 type sample.
    struct BCase { const char* id; int n; double turn; float lim; double cx, cz, yaw, hx, hy, hz, r; int type; std::vector<float> want; };
    const BCase bc[] = {
        {"B1", 10, 0, 12, -4, 35, 30, 2, 1, 1.5, 3, 0, {-1.78205073f, 9.78205109f, 29.7009621f, 30, 0, 2, -1.78205073f, 9.78205109f, 30, 37.2990379f, 0, 3}},
        {"B2", 10, 0, 12, 1, 52, -80, 1.5, 0.5, 4, 4.5, 2, {-8.49970341f, 6.49970293f, 46.8281937f, 50, 2, 4, -8.49970341f, 6.49970293f, 50, 54.1718063f, 2, 5}},
        {"B3", 10, 0, 12, 0, 52, 90, 1.5, 0.5, 4, 4.5, 3, {-7.30000019f, 7.30000019f, 47.5f, 50, 0, 4, -7.30000019f, 7.30000019f, 50, 53.5f, 0, 5}},
        {"B4", 12, 4, 15, 8, 60, 20, 12, 2, 3, 20, 0, {-12.6413221f, 17.9586792f, 57.4483757f, 60, 0, 5, -12.6641312f, 18.1635189f, 60, 64.0915909f, 0, 6}},
    };
    for (const BCase& c : bc) {
        const float lim = c.lim;
        const auto S = obsSamples(c.n, c.turn, [lim](int) { return std::make_pair(-lim, lim); });
        const double a = c.yaw * M_PI / 180.0, co = std::cos(a), si = std::sin(a);
        const float frame[12] = {float(co), 0, float(-si), float(c.cx), 0, 1, 0, 0, float(si), 0, float(co), float(c.cz)};
        const float half[3] = {float(c.hx), float(c.hy), float(c.hz)};
        std::vector<AiObstacle> out;
        aiBoxObstacles(S, frame, half, float(c.r), c.type, 2.0f, 1.3f, out);
        std::vector<float> got;
        for (const auto& o : out) got.insert(got.end(), {o.lo, o.hi, o.s0, o.s1, float(o.type), float(o.sample)});
        check(c.id, "0x6189b0 box on the corridor", got, c.want, 2e-5f);
    }
    struct PCase { const char* id; int n; double turn; float lim; int idx; float ghost; std::vector<AiObstacle> recs; std::vector<float> want; };
    auto R = [](float lo, float hi, float s0, float s1, int type, int sample) {
        AiObstacle o;
        o.lo = lo, o.hi = hi, o.s0 = s0, o.s1 = s1, o.type = type, o.sample = sample;
        return o;
    };
    const PCase pc[] = {
        {"P1", 10, 0, 10, 1, 0,
         {R(2, 6, 10, 12, 0, 1), R(-2, 1, 30, 32, 0, 3), R(3, 5, 40, 42, 2, 4), R(-4, 4, 50, 52, 3, 5), R(6, 8, 51, 53, 0, 5), R(-12, -5, 60, 62, 0, 6),
          R(5, 15, 61, 63, 0, 6), R(-3, -1, 70, 71, 0, 7), R(0, 2, 71, 72, 0, 7)},
         {-10, 10, -10, 2, -10, 3, -10, -2, -10, -1, -4, 4, -5, 5, -1, 6, -2, 7, -3, 8, -10, 10}},
        {"P2", 8, 3, 6, 2, 1.5f, {R(-8, 8, 20, 22, 0, 2), R(1, 9, 30, 33, 4, 3), R(-9, -4, 40, 41, 0, 4)},
         {-6, 6, -6, 6, -8, 8, -6, 1, -4, 2, -5, 3, -6, 4, -6, 5, -6, 6}},
    };
    for (const PCase& c : pc) {
        const float lim = c.lim;
        auto S = obsSamples(c.n, c.turn, [lim](int) { return std::make_pair(-lim, lim); });
        aiPropagateCorridor(S, c.idx, c.recs, c.ghost);
        std::vector<float> got;
        for (const auto& a : S) got.insert(got.end(), {a.latMin, a.latMax});
        check(c.id, "0x61a9d0 corridor through obstacles", got, c.want, 1e-5f);
    }
    // W1 (port invariants, not emulated): a 30 m high, 20 m thick wall across x = 200..220 m for z < 300 m; the path from (0, 0) to
    // (400, 0) must reach the goal, go round the wall's end, and the field must fall along it.
    AtvWorld w;
    w.height = [](float x, float z) { return x >= 200 && x < 220 && z < 300 ? 30.0f : 0.0f; };
    AtvPathFinder f(w);
    f.setWeights(0.35f, 0.9f);
    f.setStart(0, 0, 0);
    const bool req = f.request(400, 0, 0);
    int frames = 0;
    while (!f.idle() && frames < 1000) atvManagerUpdate({&f}), ++frames;
    const auto path = f.extract(400);
    float maxZ = -1e9f;
    bool mono = true;
    for (size_t k = 1; k + 1 < path.size(); ++k) {
        maxZ = std::max(maxZ, path[k][2]);
        const float here = f.field(AtvPathFinder::toCell(path[k][0]), AtvPathFinder::toCell(path[k][2]));
        if (k > 1 && here > f.field(AtvPathFinder::toCell(path[k - 1][0]), AtvPathFinder::toCell(path[k - 1][2]))) mono = false;
    }
    const bool good = req && f.found() && path.size() > 2 && path.back()[0] == 400 && maxZ >= 300 && mono;
    std::printf("%-4s %-40s %s  %zu points, top z %.0f, %d frames, %d expansions\n", "W1", "off-road search round a wall",
                good ? "match   " : "MISMATCH", path.size(), maxZ, frames, f.expansions());
    (good ? ok : bad)++;
}

}  // namespace fuel
