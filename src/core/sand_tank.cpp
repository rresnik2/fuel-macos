// PK12: STARTFX records and the sand-storm walls (see sand_tank.hpp and notes pk12.md). x87 note: FUEL keeps
// intermediates in 80-bit registers (53-bit precision control) and rounds to float where it stores to memory; the port
// computes in double and rounds to float at the same stores.
#include "sand_tank.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::sand {

namespace {
constexpr float kFar = 10000.0f;     // 0xa2a4e0
constexpr float kExtentScale = 0.8f;  // 0x9dc154: the slot extents -> the mesh scale
constexpr double kPi = 3.1415927410125732;   // 0x9a1550 (float pi)
constexpr double kTwoPi = 6.2831854820251465;  // 0x9dc258 (float 2 pi)
float P(const std::vector<float>& pp, int n) { return n >= 0 && size_t(n) < pp.size() ? pp[size_t(n)] : 0.0f; }
float clamp01(double x) { return x > 0 ? (x >= 1 ? 1.0f : float(x)) : 0.0f; }  // 0 also for NaN (the asm's compares)
int ftol(float v) { return int(v); }  // CRT _ftol: truncation
}  // namespace

Params Params::fromPP(const std::vector<float>& pp) {
    Params p;
    p.rings = P(pp, 5380), p.columns = P(pp, 5381);
    p.bulge = P(pp, 5382), p.bulgeStart = P(pp, 5383);
    p.uvDiffuse[0] = P(pp, 5384), p.uvDiffuse[1] = P(pp, 5385);
    p.scroll0 = P(pp, 5386);
    p.colourBase = P(pp, 5387), p.colourSlope = P(pp, 5388);
    p.height = P(pp, 5389), p.sink = P(pp, 5390);
    p.fogFar = P(pp, 5392), p.fogNear = P(pp, 5393);
    p.uvNormal[0] = P(pp, 5394), p.uvNormal[1] = P(pp, 5395);
    p.scroll1 = P(pp, 5396);
    p.alphaFar = P(pp, 5397), p.alphaNear = P(pp, 5398);
    p.fogMax = P(pp, 5399);
    return p;
}

Obb collisionBoxObb(const std::array<float, 16>& box, const std::array<float, 16>& w) {
    float c[16];  // 0x5857b0: box x node (row vectors)
    for (int r = 0; r < 4; ++r)
        for (int k = 0; k < 4; ++k) {
            double s = 0;
            for (int i = 0; i < 4; ++i) s += double(box[size_t(r * 4 + i)]) * double(w[size_t(i * 4 + k)]);
            c[r * 4 + k] = float(s);
        }
    Obb o{};
    float len[3];
    for (int r = 0; r < 3; ++r) {  // 0x743210 with ESI = C, EDI = the unit OBB (identity, centre 0, extents 0.5)
        len[r] = float(std::sqrt(double(c[r * 4]) * c[r * 4] + double(c[r * 4 + 1]) * c[r * 4 + 1] + double(c[r * 4 + 2]) * c[r * 4 + 2]));
        const double inv = len[r] > 0 ? 1.0 / len[r] : 0.0;
        for (int k = 0; k < 3; ++k) o[size_t(r * 4 + k)] = float(c[r * 4 + k] * inv);
    }
    o[3] = c[12], o[7] = c[13], o[11] = c[14];
    for (int r = 0; r < 3; ++r) o[size_t(12 + r)] = len[r] * 0.5f;
    o[15] = float(std::sqrt(double(o[12]) * o[12] + double(o[13]) * o[13] + double(o[14]) * o[14]));
    return o;
}

bool obbContains(const Obb& o, double x, double y, double z) {
    const double d[3] = {x - o[3], y - o[7], z - o[11]};
    for (int r = 0; r < 3; ++r) {
        const double l = d[0] * o[size_t(r * 4)] + d[1] * o[size_t(r * 4 + 1)] + d[2] * o[size_t(r * 4 + 2)];
        if (std::fabs(l) > o[size_t(12 + r)]) return false;
    }
    return true;
}

// ---- SandMesh --------------------------------------------------------------------------------------------------------

Mesh::Mesh(const Params& p) : p_(p) {
    // ctor 0x5ea170: +0x44 = identity rows 0-2, +0x74..+0x80 = 1 (row 3); vf02 0x5ea2e0: rings / columns by ftol, the
    // points zeroed, the colours left uninitialised (port: zero), CPU + GPU vertices (rings x columns x 0x4c).
    matrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1};
    rings_ = std::max(0, ftol(p.rings));
    cols_ = std::max(0, ftol(p.columns));
    points.assign(size_t(cols_), V3{0, 0, 0});
    colours.assign(size_t(cols_), V4{0, 0, 0, 0});
    cpu_.assign(size_t(rings_) * size_t(cols_), Vertex{});
    drawn_ = cpu_;
}

std::vector<uint16_t> Mesh::indices(int rings, int cols) {
    // 0x5ea2e0 (asm 0x5ea412..0x5ea4e0): for ring r < rings - 1 and column c (c' = (c + 1) % cols), vertex = rings c + r:
    // (c r, c' r+1, c r+1), (c r, c' r, c' r+1).
    std::vector<uint16_t> ix;
    for (int r = 0; r + 1 < rings; ++r)
        for (int c = 0; c < cols; ++c) {
            const int c1 = (c + 1) % cols;
            const uint16_t a = uint16_t(rings * c + r), b = uint16_t(rings * c1 + r + 1), d = uint16_t(rings * c + r + 1),
                           e = uint16_t(rings * c1 + r);
            ix.insert(ix.end(), {a, b, d, a, e, b});
        }
    return ix;
}

void Mesh::fill(int a, int b) {
    const int R = rings_, n = cols_;
    if (R <= 0 || n <= 0) return;
    // Mean base height of the ring points (each y x (1 / n), summed in order).
    double mean = 0;
    {
        const double inv = 1.0 / double(n);
        for (int j = 0; j < n; ++j) mean += double(points[size_t(j)][1]) * inv;
    }
    const float meanF = float(mean);
    const double Sx = matrix[12], Sz = matrix[14];  // +0x74 / +0x7c
    for (int i = 0; i < R; ++i) {
        const float f = float(double(i) / double(R - 1));                                // [0x50]
        const float h0 = float(double(p_.height) * f - double(p_.sink));                // [0x88]
        const float cf = float(double(f) * p_.colourSlope + p_.colourBase);             // [0x34]
        const float top = float(double(p_.height) * f + meanF - double(p_.sink));       // [0x70]
        if (a < n) {
            const float arg = float((double(p_.bulgeStart) + (1.0 - double(p_.bulgeStart)) * f) * kPi);  // the float argument
            const double sb = std::sin(double(arg));
            const float sbF = float(sb);                                                // [0x40]
            const float bulge = float(1.0 + double(p_.bulge) * sb);                     // [0x2c]
            const float sb2 = float(sb * sb);                                           // [0x60]
            for (int j = a; j < n; ++j) {
                const float theta = float(double(j) / double(n - 1) * kTwoPi);
                const float c = float(std::cos(double(theta)));
                const double s = std::sin(double(theta));
                const double dmE = double(c) - s, dpE = double(c) + s;
                const float dm = float(dmE), dp = float(dpE);
                const double L = std::sqrt(double(float(dpE * dpE + dmE * dmE + double(sb2))));
                const double inv = 1.0 / L;
                Vertex& v = cpu_[size_t(j) * size_t(R) + size_t(i)];
                v.normal[0] = float(double(dm) * inv), v.normal[1] = float(double(sbF) * inv), v.normal[2] = float(double(dp) * inv);
                const double A = double(points[size_t(j)][1]) + double(h0);
                const float X = float(double(dm) * Sx * double(bulge));
                const float H = float(A + double(f) * (double(top) - A));
                const float Z = float(double(dp) * Sz * double(bulge));
                if (i == 0) points[size_t(j)][0] = X, points[size_t(j)][2] = Z;
                v.pos[0] = X, v.pos[1] = H, v.pos[2] = Z, v.pos[3] = 1.0f;
                v.u = float(double(j) / double(n - 1));
                v.v = f;
                const V4& col = colours[size_t(j)];
                v.colour[0] = col[0] * cf, v.colour[1] = col[1] * cf, v.colour[2] = col[2] * cf, v.colour[3] = 1.0f;
            }
        }
        // Tangent (v_j - v_j+1) and binormal (t x n) of this ring, every column; the last copies the first.
        for (int j = 0; j < n; ++j) {
            Vertex& v = cpu_[size_t(j) * size_t(R) + size_t(i)];
            if (j == n - 1) {
                const Vertex& v0 = cpu_[size_t(i)];
                std::copy(v0.tangent, v0.tangent + 3, v.tangent);
                std::copy(v0.binormal, v0.binormal + 3, v.binormal);
                continue;
            }
            const Vertex& w = cpu_[size_t((j + 1) % n) * size_t(R) + size_t(i)];
            for (int k = 0; k < 3; ++k) v.tangent[k] = v.pos[k] - w.pos[k];
            const float* t = v.tangent;
            const float* nn = v.normal;
            v.binormal[0] = float(double(nn[2]) * t[1] - double(nn[1]) * t[2]);
            v.binormal[1] = float(double(nn[0]) * t[2] - double(nn[2]) * t[0]);
            v.binormal[2] = float(double(t[0]) * nn[1] - double(nn[0]) * t[1]);
        }
    }
    // The upload (VB vt+8): columns [a, a + b).
    for (int j = std::max(a, 0); j < std::min(a + b, n); ++j)
        for (int i = 0; i < R; ++i) drawn_[size_t(j) * size_t(R) + size_t(i)] = cpu_[size_t(j) * size_t(R) + size_t(i)];
}

void Mesh::update(float dt) {
    scroll1 = float(double(p_.scroll1) * dt + scroll1);  // +0x90 (PP5396)
    scroll0 = float(double(dt) * p_.scroll0 + scroll0);  // +0x8c (PP5386)
    if (!first) {
        fill(0, cols_);
        first = true;
    }
}

// ---- SandTank --------------------------------------------------------------------------------------------------------

int Tank::addSlot() {
    ++count_;
    meshes_.push_back(std::make_unique<Mesh>(p_));  // 0x5ed850 + ctor 0x5ea170, vf02, registered with the FX manager
    states_.push_back(0);                             // 0x5ed650
    matrices_.push_back(Obb{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1});  // 0x5ed740
    return int(meshes_.size()) - 1;
}

void Tank::removeSlot(int slot) {
    --count_;
    if (size_t(slot) < states_.size()) states_[size_t(slot)] = 0;
    if (size_t(slot) < meshes_.size()) meshes_[size_t(slot)].reset();  // 0x5f7d30
    if (count_ == 0) states_.clear(), matrices_.clear(), meshes_.clear();
}

bool Tank::insidePolygon(const V3& cam, const std::vector<V3>& pts, int n, const Obb& m) {
    // 0x5ec9a0: edges k-1 -> k for k = 1 .. n-1 (the closing edge is not tested; ring point n-1 = point 0); outside at
    // the first cross product > 0.
    if (n - 1 < 1) return true;
    for (int k = 1; k <= n - 1; ++k) {
        const V3& p = pts[size_t(k - 1)];
        const V3& q = pts[size_t(k % n)];
        const double cr = (double(p[0]) - q[0]) * (double(cam[2]) - (double(m[11]) + p[2])) +
                          (double(q[2]) - p[2]) * (double(cam[0]) - (double(p[0]) + m[3]));
        if (cr > 0) return false;
    }
    return true;
}

float Tank::segmentDistance(const V3& P, const V3& A, const V3& B) {
    // 0x5eca40 (xz): perpendicular distance when the projection parameter is in (0, 1], else the nearer endpoint.
    const double ax = A[0], az = A[2];
    const double ez = double(B[2]) - az, ex = double(B[0]) - ax;
    const double L2 = ex * ex + ez * ez;
    const double inv = double(1.0L / (long double)L2);  // stored as a double (dVar1)
    const double t = ((double(P[2]) - az) * ez + (double(P[0]) - ax) * ex) * inv;
    if (t > 0 && t <= 1) return float(std::sqrt(L2) * std::fabs(((az - P[2]) * ex - (ax - P[0]) * ez) * inv));
    const double dA = (double(P[2]) - az) * (double(P[2]) - az) + (double(P[0]) - ax) * (double(P[0]) - ax);
    const double dB = (double(P[2]) - B[2]) * (double(P[2]) - B[2]) + (double(P[0]) - B[0]) * (double(P[0]) - B[0]);
    return float(dB <= dA ? std::sqrt(dB) : std::sqrt(dA));
}

void Tank::update(float dt, const V3& cam, const Ground& ground) {
    (void)dt;  // +0x10 / +0x2c (time, displacement by the wind): only the sand particles read them, never drawn (+8 = 0)
    for (auto& m : meshes_)
        if (m) m->instances.clear();  // [port] vf01 0x5eb670 empties the instance list every frame
    if (count_ != 0) {
        double D = kFar;  // local_134
        bool in = false;   // local_161
        for (size_t s = 0; s < matrices_.size(); ++s) {
            if (states_[s] <= 0 || !meshes_[s]) continue;
            Mesh& m = *meshes_[s];
            const Obb M = matrices_[s];
            m.matrix = M;  // 0x5ed5b0 + x 0.8 + 0x743ed0 -> +0x44
            for (int k = 12; k < 15; ++k) m.matrix[size_t(k)] = float(double(M[size_t(k)]) * kExtentScale);
            m.instances.push_back({M[3], 0.0f, M[11]});  // 0x5de5b0 (+ the identity rotation, 0x5de6b0)
            const int n = m.columns();
            const bool world = bool(ground.height);  // the GenWorld handle chain (+0xc -> +0x28) resolves
            if (states_[s] == 1) {
                if (world && n > 0) {
                    for (int j = 0; j < n; ++j) {
                        V3& p = m.points[size_t(j)];
                        const float x = float(double(M[3]) + p[0]), z = float(double(M[11]) + p[2]);
                        p[1] = ground.loaded ? ground.height(x, z) : 0.0f;  // 0x63eb10 + 0x63ea40
                        if (ground.loaded && ground.colour) m.colours[size_t(j)] = ground.colour(x, p[1], z);  // 0x63ecf0
                    }
                }
                // Mean colour: (0, 0, 0, 1) + every column's colour; rgb / n, alpha the plain sum; copied to every column.
                double sum[4] = {0, 0, 0, 1};  // 0xa064c8
                for (int j = 0; j < n; ++j)
                    for (int k = 0; k < 4; ++k) sum[k] += m.colours[size_t(j)][size_t(k)];
                if (n > 0) {
                    const double inv = 1.0 / double(n);
                    const V4 mc{float(sum[0] * inv), float(sum[1] * inv), float(sum[2] * inv), float(sum[3])};
                    for (V4& c : m.colours) c = mc;
                }
                m.fill(0, n);
            } else if (world && n > 0) {  // state 2: one ring point per frame, round robin
                const int j = m.next;
                V3& p = m.points[size_t(j)];
                p[1] = ground.loaded ? ground.height(float(double(M[3]) + p[0]), float(double(M[11]) + p[2])) : 0.0f;  // 0x6407e0
                V4 c{0, 0, 0, 0};  // FUEL: an uninitialised local when the world is not loaded
                if (ground.loaded && ground.colour) c = ground.colour(float(double(p[0]) + M[3]), p[1], float(double(p[2]) + M[11]));
                const double inv = 1.0 / double(n), keep = double(n - 1) / double(n);
                V4& col = m.colours[size_t(j)];
                for (int k = 0; k < 3; ++k) col[size_t(k)] = float(keep * col[size_t(k)] + c[size_t(k)] * inv);
                col[3] = float(double(col[3]) + c[3]);
                m.fill(j, 1);
                m.next = (j + 1) % n;
            }
            // The camera in the wall's ring polygon (any slot so far), else its distance to the nearest edge.
            in = in || insidePolygon(cam, m.points, n, M);
            double d = kFar;
            if (!in) {
                const V3 rel{float(double(cam[0]) - M[3]), cam[1], float(double(cam[2]) - M[11])};
                for (int k = 1; k <= n; ++k) {
                    const double e = segmentDistance(rel, m.points[size_t(k - 1)], m.points[size_t(k % n)]);
                    if (e < d) d = e;
                }
            } else {
                D = 0;
            }
            m.edge = float(d);  // +0x94 (10000 once the camera is inside a wall)
            if (d < D) D = d;
            states_[s] = 2;
        }
        // Wall alpha and the fog floor.
        const float a = clamp01((D - double(p_.alphaNear)) / (double(p_.alphaFar) - double(p_.alphaNear)));
        for (size_t s = 0; s < states_.size(); ++s)
            if (states_[s] != 0 && meshes_[s]) meshes_[s]->alpha = a;
        const float t = clamp01((D - double(p_.fogNear)) / (double(p_.fogFar) - double(p_.fogNear)));
        double f = p_.fogMax;
        if (!in) f = (1.0 - t) * f;
        fog = clamp01(f);
        fogSet = true;  // weather +0x11c
        minEdge = float(D), inside = in;
    }
    for (auto& m : meshes_)  // the FX manager's mesh list (every live slot, state 0 included): vf03 0x5eb590
        if (m) m->update(dt);
}

// ---- STARTFX records -------------------------------------------------------------------------------------------------

void StartFx::fire(const Fire& f) {
    // 0x5bc340: the record of this node, else a new one (0x5bc520 appends 0x70 zero bytes, 0x5bc5a0).
    Record* r = nullptr;
    for (Record& x : records_)
        if (x.node == f.node) {
            r = &x;
            break;
        }
    if (r == nullptr) r = &records_.emplace_back();
    // 0x5c0710 (a record found again is filled again: a SAND one takes a second slot, the first one stays in the tank).
    r->duration = 120.0f;
    r->type = 0;
    r->dead = false;
    r->elapsed = 0;
    r->centre = {0, 0, 0};
    r->node = f.node;
    r->spline = f.spline;
    if (f.hasTime) r->duration = f.time;
    if (f.sand) r->type = 1;
    r->obb = f.obb;  // Node_CollisionBoxObb (G's flags: 0x8000000 cleared, 0x10000 set: not used by the port)
    r->centre = {f.obb[3], f.obb[7], f.obb[11]};
    if (r->type == 1) r->slot = tank_.addSlot();  // 0x5c0850 -> 0x5ec1d0
}

void StartFx::removeAt(size_t i) {
    if (records_[i].type == 1) tank_.removeSlot(records_[i].slot);  // 0x5c0a00 -> 0x5ec290
    records_.erase(records_.begin() + std::ptrdiff_t(i));          // 0x5bc4a0
}

void StartFx::release(uint32_t node) {
    for (size_t i = 0; i < records_.size(); ++i)
        if (records_[i].node == node) {
            removeAt(i);
            break;
        }
}

void StartFx::step(float dt, const std::function<bool(uint32_t)>& hidden) {
    for (size_t i = records_.size(); i-- > 0;)
        if (records_[i].dead) removeAt(i);
    for (Record& r : records_) {  // 0x5c08b0
        r.elapsed = float(double(dt) + r.elapsed);
        if (hidden && hidden(r.node)) {
            r.dead = true;
            continue;
        }
        // r.spline: 0x51b270 evaluates C's spline at clamp01(elapsed / duration) into r.centre (+0x64), which nothing
        // reads; not computed (see notes pk12.md).
        if (r.type == 1 && size_t(r.slot) < tank_.slots()) {
            int& st = tank_.state(r.slot);
            st = (st != 0) + 1;
            tank_.matrix(r.slot) = r.obb;  // 0x743ed0
        }
    }
}

void StartFx::dropHidden(const std::function<bool(uint32_t)>& hidden) {
    for (size_t i = records_.size(); i-- > 0;)
        if (records_[i].dead || (hidden && hidden(records_[i].node))) removeAt(i);
}

}  // namespace fuel::sand
