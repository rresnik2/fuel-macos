// [DEVIATION, the user's request 2026-11-06] Visible vehicle upgrades (U5). See upgrade_visuals.hpp and notes/upg5.md.
#include "upgrade_visuals.hpp"

#include "custom_rider.hpp"
#include "custom_vehicles.hpp"
#include "resources.hpp"
#include "upgrades.hpp"
#include "physics/vehicle_physics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <map>
#include <mutex>
#include <unordered_map>

namespace fuel::upgvis {

namespace {

using V3 = std::array<float, 3>;
V3 add(V3 a, V3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 sub(V3 a, V3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 mul(V3 a, float k) { return {a[0] * k, a[1] * k, a[2] * k}; }
float dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3 cross(V3 a, V3 b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
float len(V3 a) { return std::sqrt(dot(a, a)); }
V3 norm(V3 a) {
    const float l = len(a);
    return l > 1e-9f ? mul(a, 1.0f / l) : V3{0, 1, 0};
}
V3 anyPerp(V3 a) { return norm(cross(a, std::fabs(a[1]) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0})); }

Mat4 identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }
Mat4 matMul(const Mat4& a, const Mat4& b) {  // row vectors: a then b
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            r[size_t(i * 4 + j)] = s;
        }
    return r;
}
Mat4 translation(V3 t) {
    Mat4 m = identity();
    m[12] = t[0], m[13] = t[1], m[14] = t[2];
    return m;
}
Mat4 scaling(float x, float y, float z) {
    Mat4 m = identity();
    m[0] = x, m[5] = y, m[10] = z;
    return m;
}
// A uniform-ish scale about point `c`: k along each axis.
Mat4 scaleAbout(V3 c, float kx, float ky, float kz) { return matMul(matMul(translation(mul(c, -1)), scaling(kx, ky, kz)), translation(c)); }
bool isIdentity(const Mat4& m) {
    const Mat4 i = identity();
    for (size_t k = 0; k < 16; ++k)
        if (std::fabs(m[k] - i[k]) > 1e-7) return false;
    return true;
}

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
bool has(const std::string& s, const char* w) { return s.find(w) != std::string::npos; }

std::atomic<int> gEnabled{-1};

// ---- the registry
std::mutex gMu;
std::unordered_map<uint32_t, std::shared_ptr<const BakedMesh>> gProcedural;
std::unordered_map<uint32_t, Derived> gDerived;

uint32_t keyOf(const std::string& what) { return asoboCrc("DB:>PORT>UPGRADES>" + what); }

// ---- materials (gamma-space colours, as FUEL's textures)
const std::array<CustomMaterial, size_t(Mat::kCount)>& materials() {
    static const std::array<CustomMaterial, size_t(Mat::kCount)> m = [] {
        std::array<CustomMaterial, size_t(Mat::kCount)> a{};
        auto set = [&](Mat k, const char* name, std::array<float, 4> rgba, std::array<float, 4> spec, uint32_t flags,
                       std::array<float, 2> fres = {0.4f, 1.0f}) {
            CustomMaterial& c = a[size_t(k)];
            c.name = name, c.rgba = rgba, c.spec = spec, c.flags = CustomMaterial::Solid | flags, c.fresnel = fres;
        };
        set(Mat::Bottle, "U5_BOTTLE", {0.10f, 0.32f, 0.86f, 1}, {0.9f, 0.9f, 0.9f, 60}, CustomMaterial::Chrome, {0.10f, 0.55f});
        set(Mat::Chrome, "U5_CHROME", {0.72f, 0.72f, 0.74f, 1}, {1, 1, 1, 80}, CustomMaterial::Chrome, {0.55f, 1.0f});
        set(Mat::Dark, "U5_DARK", {0.05f, 0.05f, 0.05f, 1}, {0.05f, 0.05f, 0.05f, 8}, 0);
        set(Mat::Plastic, "U5_PLASTIC", {0.09f, 0.09f, 0.09f, 1}, {0.18f, 0.18f, 0.18f, 14}, 0);
        set(Mat::Spacer, "U5_SPACER", {0.26f, 0.27f, 0.29f, 1}, {0.7f, 0.7f, 0.7f, 40}, CustomMaterial::Chrome, {0.15f, 0.7f});
        set(Mat::SpringS1, "U5_SPRING1", {0.95f, 0.78f, 0.10f, 1}, {0.6f, 0.6f, 0.6f, 40}, 0);
        set(Mat::SpringS2, "U5_SPRING2", {0.98f, 0.45f, 0.06f, 1}, {0.6f, 0.6f, 0.6f, 40}, 0);
        set(Mat::SpringS3, "U5_SPRING3", {0.90f, 0.08f, 0.06f, 1}, {0.6f, 0.6f, 0.6f, 40}, 0);
        set(Mat::Stripe, "U5_STRIPE", {0.88f, 0.88f, 0.86f, 1}, {0.15f, 0.15f, 0.15f, 10}, 0);
        set(Mat::StripeS3, "U5_STRIPE3", {0.98f, 0.80f, 0.08f, 1}, {0.15f, 0.15f, 0.15f, 10}, 0);
        set(Mat::Paint, "U5_PAINT", {1, 1, 1, 0}, {0.6f, 0.6f, 0.6f, 30}, CustomMaterial::Paint);
        set(Mat::Filter, "U5_FILTER", {0.80f, 0.06f, 0.05f, 1}, {0.2f, 0.2f, 0.2f, 10}, 0);
        set(Mat::Carbon, "U5_CARBON", {1, 1, 1, 0}, {0.6f, 0.6f, 0.6f, 30}, CustomMaterial::Paint | CustomMaterial::Carbon);
        return a;
    }();
    return m;
}

// ---- procedural geometry
struct Builder {
    struct Vtx {
        V3 p, n;
        float u = 0, v = 0;
    };
    std::vector<Mat> mats;
    std::vector<uint32_t> fuelMats;  // per slot: a FUEL Material_Z id (0: the procedural material mats[slot])
    std::vector<std::vector<Vtx>> verts;
    std::vector<std::vector<uint32_t>> idx;
    uint32_t useFuel = 0;  // when set, the next primitives take this Material_Z instead of their Mat (the tyre's own)
    int slot(Mat m) {
        for (size_t i = 0; i < mats.size(); ++i)
            if (mats[i] == m && fuelMats[i] == useFuel) return int(i);
        mats.push_back(m), fuelMats.push_back(useFuel), verts.emplace_back(), idx.emplace_back();
        return int(mats.size() - 1);
    }
    uint32_t vtx(int s, V3 p, V3 n, float u = 0, float v = 0) {
        verts[size_t(s)].push_back({p, norm(n), u, v});
        return uint32_t(verts[size_t(s)].size() - 1);
    }
    void tri(int s, uint32_t a, uint32_t b, uint32_t c) { idx[size_t(s)].insert(idx[size_t(s)].end(), {a, b, c}); }
    // A quad a b c d (counter-clockwise seen from the side the normal points to), flat shaded.
    // The same with the face turned so that its normal points along `hint` (right-hand rule normals face out).
    void quadOut(Mat m, V3 a, V3 b, V3 c, V3 d, V3 hint) {
        if (dot(cross(sub(b, a), sub(d, a)), hint) < 0) quad(m, d, c, b, a);
        else quad(m, a, b, c, d);
    }
    void quad(Mat m, V3 a, V3 b, V3 c, V3 d) {
        const int s = slot(m);
        const V3 n = norm(cross(sub(b, a), sub(d, a)));
        const uint32_t i0 = vtx(s, a, n, 0, 0), i1 = vtx(s, b, n, 1, 0), i2 = vtx(s, c, n, 1, 1), i3 = vtx(s, d, n, 0, 1);
        tri(s, i0, i1, i2), tri(s, i0, i2, i3);
    }
    // A tube along a -> b (radius r0 at a, r1 at b), `seg` sides, caps when asked; `inward` flips the normals (a bore).
    void tube(Mat m, V3 a, V3 b, float r0, float r1, int seg, bool capA, bool capB, bool inward = false) {
        const int s = slot(m);
        const V3 ax = norm(sub(b, a)), e1 = anyPerp(ax), e2 = cross(ax, e1);
        const uint32_t base = uint32_t(verts[size_t(s)].size());
        for (int i = 0; i <= seg; ++i) {
            const float t = 2.0f * float(M_PI) * float(i) / float(seg);
            const V3 d = add(mul(e1, std::cos(t)), mul(e2, std::sin(t)));
            const V3 n = inward ? mul(d, -1) : d;
            vtx(s, add(a, mul(d, r0)), n, float(i) / float(seg), 0);
            vtx(s, add(b, mul(d, r1)), n, float(i) / float(seg), 1);
        }
        for (int i = 0; i < seg; ++i) {
            const uint32_t p0 = base + uint32_t(2 * i), p1 = p0 + 1, q0 = p0 + 2, q1 = p0 + 3;
            if (inward) tri(s, p0, p1, q0), tri(s, q0, p1, q1);
            else tri(s, p0, q0, p1), tri(s, q0, q1, p1);
        }
        auto cap = [&](V3 c, float r, V3 n) {
            const uint32_t ci = vtx(s, c, n, 0.5f, 0.5f);
            const uint32_t first = uint32_t(verts[size_t(s)].size());
            for (int i = 0; i <= seg; ++i) {
                const float t = 2.0f * float(M_PI) * float(i) / float(seg);
                vtx(s, add(c, mul(add(mul(e1, std::cos(t)), mul(e2, std::sin(t))), r)), n);
            }
            for (int i = 0; i < seg; ++i) {
                if (dot(n, ax) > 0) tri(s, ci, first + uint32_t(i), first + uint32_t(i) + 1);
                else tri(s, ci, first + uint32_t(i) + 1, first + uint32_t(i));
            }
        };
        if (capA) cap(a, r0, mul(ax, -1));
        if (capB) cap(b, r1, ax);
    }
    // A flat annulus at centre c facing n (radii r0 < r1).
    void annulus(Mat m, V3 c, V3 n, float r0, float r1, int seg) {
        const int s = slot(m);
        const V3 e1 = anyPerp(n), e2 = cross(n, e1);
        const uint32_t base = uint32_t(verts[size_t(s)].size());
        for (int i = 0; i <= seg; ++i) {
            const float t = 2.0f * float(M_PI) * float(i) / float(seg);
            const V3 d = add(mul(e1, std::cos(t)), mul(e2, std::sin(t)));
            vtx(s, add(c, mul(d, r0)), n), vtx(s, add(c, mul(d, r1)), n);
        }
        for (int i = 0; i < seg; ++i) {
            const uint32_t p0 = base + uint32_t(2 * i);
            tri(s, p0, p0 + 1, p0 + 2), tri(s, p0 + 2, p0 + 1, p0 + 3);
        }
    }
    // A hemisphere-ish dome closing a tube end at c along ax (radius r).
    void dome(Mat m, V3 c, V3 ax, float r, int seg) {
        const int s = slot(m);
        const V3 e1 = anyPerp(ax), e2 = cross(ax, e1);
        const int rings = 4;
        const uint32_t base = uint32_t(verts[size_t(s)].size());
        for (int j = 0; j <= rings; ++j) {
            const float ph = 0.5f * float(M_PI) * float(j) / float(rings);
            for (int i = 0; i <= seg; ++i) {
                const float t = 2.0f * float(M_PI) * float(i) / float(seg);
                const V3 d = add(mul(add(mul(e1, std::cos(t)), mul(e2, std::sin(t))), std::cos(ph)), mul(ax, std::sin(ph)));
                vtx(s, add(c, mul(d, r)), d);
            }
        }
        for (int j = 0; j < rings; ++j)
            for (int i = 0; i < seg; ++i) {
                const uint32_t a = base + uint32_t(j * (seg + 1) + i), b = a + 1, c2 = a + uint32_t(seg + 1), d = c2 + 1;
                tri(s, a, b, c2), tri(s, b, d, c2);
            }
    }
    // A tube swept along a polyline (radius r, `sides`): the coil springs.
    void sweep(Mat m, const std::vector<V3>& path, float r, int sides) {
        if (path.size() < 2) return;
        const int s = slot(m);
        V3 t0 = norm(sub(path[1], path[0]));
        V3 nrm = anyPerp(t0);
        const uint32_t base = uint32_t(verts[size_t(s)].size());
        for (size_t k = 0; k < path.size(); ++k) {
            const V3 t = norm(k + 1 < path.size() ? sub(path[k + 1], path[k]) : sub(path[k], path[k - 1]));
            nrm = norm(sub(nrm, mul(t, dot(nrm, t))));  // parallel transport
            const V3 b = cross(t, nrm);
            for (int i = 0; i <= sides; ++i) {
                const float a = 2.0f * float(M_PI) * float(i) / float(sides);
                const V3 d = add(mul(nrm, std::cos(a)), mul(b, std::sin(a)));
                vtx(s, add(path[k], mul(d, r)), d);
            }
        }
        for (size_t k = 0; k + 1 < path.size(); ++k)
            for (int i = 0; i < sides; ++i) {
                const uint32_t a = base + uint32_t(k * size_t(sides + 1) + size_t(i)), b = a + 1, c = a + uint32_t(sides + 1), d = c + 1;
                tri(s, a, b, c), tri(s, b, d, c);
            }
    }
    // A box from its centre, three unit axes and half extents.
    void box(Mat m, V3 c, V3 ax, V3 ay, V3 az, V3 h) {
        auto P = [&](float x, float y, float z) { return add(c, add(add(mul(ax, x * h[0]), mul(ay, y * h[1])), mul(az, z * h[2]))); };
        quadOut(m, P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), az);
        quadOut(m, P(1, -1, -1), P(-1, -1, -1), P(-1, 1, -1), P(1, 1, -1), mul(az, -1));
        quadOut(m, P(1, -1, 1), P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), ax);
        quadOut(m, P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), P(-1, 1, -1), mul(ax, -1));
        quadOut(m, P(-1, 1, 1), P(1, 1, 1), P(1, 1, -1), P(-1, 1, -1), ay);
        quadOut(m, P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), mul(ay, -1));
    }
    uint32_t triangles() const {
        size_t n = 0;
        for (const auto& i : idx) n += i.size() / 3;
        return uint32_t(n);
    }
    bool empty() const { return triangles() == 0; }
    std::shared_ptr<BakedMesh> bake() const {
        auto out = std::make_shared<BakedMesh>();
        auto pack = [](float v) { return uint8_t(std::lround(std::clamp((v * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f))); };
        for (size_t s = 0; s < mats.size(); ++s) {
            if (idx[s].empty()) continue;
            const uint32_t first = uint32_t(out->vertices.size() / BakedMesh::kStride);
            for (const Vtx& v : verts[s]) {
                uint8_t o[BakedMesh::kStride] = {};
                std::memcpy(o, v.p.data(), 12);
                const V3 t = anyPerp(v.n);
                o[12] = pack(t[2]), o[13] = pack(t[1]), o[14] = pack(t[0]), o[15] = 255;
                o[16] = pack(v.n[2]), o[17] = pack(v.n[1]), o[18] = pack(v.n[0]), o[19] = 255;
                std::memcpy(o + 20, &v.u, 4), std::memcpy(o + 24, &v.v, 4);
                out->vertices.insert(out->vertices.end(), o, o + sizeof o);
            }
            BakedBatch b;
            b.material = uint32_t(out->materials.size());
            b.firstIndex = uint32_t(out->indices.size());
            for (uint32_t i : idx[s]) out->indices.push_back(first + i);
            b.indexCount = uint32_t(idx[s].size());
            BakedMaterial bm;
            if (fuelMats[s]) bm.materialId = fuelMats[s];
            else bm.custom = &materials()[size_t(mats[s])];
            out->materials.push_back(bm);
            out->batches.push_back(b);
        }
        return out;
    }
};

// ---- the parts' geometry (vehicle space, rest)
struct Geo {
    bool ok = false;
    std::vector<V3> v;
    struct B {
        uint32_t matName = 0, code = 0, matId = 0;
        std::vector<uint32_t> t;  // triangle corners (indices into v)
        V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
    };
    std::vector<B> batches;
    V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
    V3 centre() const { return mul(add(lo, hi), 0.5f); }
};

V3 xform(const Mat4& m, V3 p) {
    return {float(p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12]), float(p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13]),
            float(p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14])};
}

Geo measure(const DpcFile& dpc, const PlacedPart& part) {
    Geo g;
    const DpcFile::Entry* e = dpc.find(part.meshId);
    if (!e) return g;
    try {
        const MeshLink link = parseMeshLink(dpc.link(*e));
        const auto body = dpc.body(*e);
        if (!body) return g;
        const Mesh m = parseMeshBody(*body, link.flags);
        std::vector<int64_t> vbBase(m.vbs.size(), -1);
        for (const Batch& b : m.batches) {
            if (b.vb >= m.vbs.size() || m.vbs[b.vb].stride < 12) continue;
            const VertexBuffer& vb = m.vbs[b.vb];
            if (vbBase[b.vb] < 0) {
                vbBase[b.vb] = int64_t(g.v.size());
                for (uint32_t i = 0; i < vb.count && size_t(i + 1) * vb.stride <= vb.data.size(); ++i) {
                    float p[3];
                    std::memcpy(p, &vb.data[size_t(i) * vb.stride], 12);
                    g.v.push_back(xform(part.world, {p[0], p[1], p[2]}));
                }
            }
            Geo::B gb;
            const uint32_t mat = b.material < m.materials.size() ? m.materials[b.material] : 0;
            gb.matId = mat;
            if (const DpcFile::Entry* me = mat ? dpc.find(mat) : nullptr) {
                const auto ml = dpc.link(*me);
                if (ml.size() >= 4) std::memcpy(&gb.matName, ml.data(), 4);
                if (auto mb = dpc.body(*me)) gb.code = parseMaterialBody(*mb).flags & 0x1f;
            }
            for (uint32_t i : batchTriangles(m, b)) {
                const uint32_t k = uint32_t(vbBase[b.vb]) + i;
                if (k >= g.v.size()) continue;
                gb.t.push_back(k);
                for (int a = 0; a < 3; ++a) gb.lo[size_t(a)] = std::min(gb.lo[size_t(a)], g.v[k][size_t(a)]), gb.hi[size_t(a)] = std::max(gb.hi[size_t(a)], g.v[k][size_t(a)]);
            }
            gb.t.resize(gb.t.size() / 3 * 3);
            for (int a = 0; a < 3; ++a) g.lo[size_t(a)] = std::min(g.lo[size_t(a)], gb.lo[size_t(a)]), g.hi[size_t(a)] = std::max(g.hi[size_t(a)], gb.hi[size_t(a)]);
            g.batches.push_back(std::move(gb));
        }
        g.ok = !g.v.empty() && g.lo[0] <= g.hi[0];
    } catch (const std::exception&) {
        g.ok = false;
    }
    return g;
}

// The vertices actually used by triangles (skinned / shared VBs can hold more).
std::vector<V3> usedVerts(const Geo& g) {
    std::vector<uint8_t> used(g.v.size(), 0);
    for (const auto& b : g.batches)
        for (uint32_t k : b.t) used[k] = 1;
    std::vector<V3> out;
    for (size_t i = 0; i < g.v.size(); ++i)
        if (used[i]) out.push_back(g.v[i]);
    return out;
}

// Principal axis (power iteration on the covariance) and the two end centres (mean of the outer 10 % along it).
struct Axis {
    V3 c{}, u{0, 1, 0}, e0{}, e1{};
    float length = 0, perp = 0;  // end-to-end length, 80th percentile of the distance from the axis
    bool ok = false;
};
Axis principal(const std::vector<V3>& pts) {
    Axis a;
    if (pts.size() < 4) return a;
    V3 c{};
    for (const V3& p : pts) c = add(c, p);
    c = mul(c, 1.0f / float(pts.size()));
    double C[3][3] = {};
    for (const V3& p : pts) {
        const V3 d = sub(p, c);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) C[i][j] += double(d[size_t(i)]) * d[size_t(j)];
    }
    V3 u{0.3f, 0.9f, 0.3f};
    for (int it = 0; it < 40; ++it) {
        V3 w{};
        for (int i = 0; i < 3; ++i) w[size_t(i)] = float(C[i][0] * u[0] + C[i][1] * u[1] + C[i][2] * u[2]);
        if (len(w) < 1e-12f) break;
        u = norm(w);
    }
    float tmin = 1e9f, tmax = -1e9f;
    for (const V3& p : pts) {
        const float t = dot(sub(p, c), u);
        tmin = std::min(tmin, t), tmax = std::max(tmax, t);
    }
    const float span = tmax - tmin;
    if (!(span > 1e-4f)) return a;
    V3 s0{}, s1{};
    int n0 = 0, n1 = 0;
    std::vector<float> perp;
    for (const V3& p : pts) {
        const float t = dot(sub(p, c), u);
        if (t <= tmin + 0.1f * span) s0 = add(s0, p), ++n0;
        if (t >= tmax - 0.1f * span) s1 = add(s1, p), ++n1;
        perp.push_back(len(sub(sub(p, c), mul(u, t))));
    }
    std::sort(perp.begin(), perp.end());
    a.c = c, a.u = u, a.e0 = mul(s0, 1.0f / float(std::max(n0, 1))), a.e1 = mul(s1, 1.0f / float(std::max(n1, 1)));
    a.length = len(sub(a.e1, a.e0));
    a.perp = perp[std::min(perp.size() - 1, size_t(double(perp.size()) * 0.8))];
    a.ok = a.length > 0.02f;
    return a;
}

// ---- a top-down height map of the body (max y per 4 cm cell), for what sits on top surfaces
struct HeightMap {
    float x0 = 0, z0 = 0, cell = 0.04f;
    int nx = 0, nz = 0;
    std::vector<float> h;
    static constexpr float kNone = -1e9f;
    void init(V3 lo, V3 hi) {
        x0 = lo[0] - cell, z0 = lo[2] - cell;
        nx = std::clamp(int((hi[0] - lo[0]) / cell) + 3, 1, 400);
        nz = std::clamp(int((hi[2] - lo[2]) / cell) + 3, 1, 600);
        h.assign(size_t(nx) * size_t(nz), kNone);
    }
    void raster(V3 a, V3 b, V3 c) {
        const float area = (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
        if (std::fabs(area) < 1e-7f || nx == 0) return;
        const int i0 = std::max(0, int(std::floor((std::min({a[0], b[0], c[0]}) - x0) / cell))),
                  i1 = std::min(nx - 1, int(std::floor((std::max({a[0], b[0], c[0]}) - x0) / cell)));
        const int j0 = std::max(0, int(std::floor((std::min({a[2], b[2], c[2]}) - z0) / cell))),
                  j1 = std::min(nz - 1, int(std::floor((std::max({a[2], b[2], c[2]}) - z0) / cell)));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const float px = x0 + (float(i) + 0.5f) * cell, pz = z0 + (float(j) + 0.5f) * cell;
                const float w0 = ((b[0] - px) * (c[2] - pz) - (c[0] - px) * (b[2] - pz)) / area;
                const float w1 = ((c[0] - px) * (a[2] - pz) - (a[0] - px) * (c[2] - pz)) / area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < -1e-4f || w1 < -1e-4f || w2 < -1e-4f) continue;
                float& o = h[size_t(j) * size_t(nx) + size_t(i)];
                o = std::max(o, w0 * a[1] + w1 * b[1] + w2 * c[1]);
            }
    }
    // Max / min / coverage over a rectangle [xa, xb] x [za, zb].
    struct Stat {
        float max = kNone, min = 1e9f;
        float covered = 0;
    };
    Stat over(float xa, float xb, float za, float zb) const {
        Stat s;
        int n = 0, in = 0;
        for (float z = za; z <= zb + 1e-4f; z += cell)
            for (float x = xa; x <= xb + 1e-4f; x += cell) {
                ++n;
                const int i = int(std::floor((x - x0) / cell)), j = int(std::floor((z - z0) / cell));
                if (i < 0 || j < 0 || i >= nx || j >= nz) continue;
                const float v = h[size_t(j) * size_t(nx) + size_t(i)];
                if (v <= kNone) continue;
                ++in;
                s.max = std::max(s.max, v), s.min = std::min(s.min, v);
            }
        s.covered = n > 0 ? float(in) / float(n) : 0.0f;
        return s;
    }
};

}  // namespace

// ---- public helpers

bool enabled() {
    int e = gEnabled.load();
    if (e < 0) {
        const char* v = std::getenv("FUEL_UPGRADE_VISUALS");
        e = (v && (std::strcmp(v, "0") == 0 || std::strcmp(v, "off") == 0)) ? 0 : 1;
        gEnabled.store(e);
    }
    return e != 0;
}
void setEnabled(bool on) { gEnabled.store(on ? 1 : 0); }

const CustomMaterial* material(Mat m) { return &materials()[size_t(m)]; }

std::array<float, 3> transformPoint(const Mat4& m, const std::array<float, 3>& p) { return xform(m, p); }

Mat4 segmentMap(const std::array<float, 3>& C, const std::array<float, 3>& B, const std::array<float, 3>& B2, float thick) {
    const V3 d0 = sub(B, C), d1 = sub(B2, C);
    const float l0 = len(d0), l1 = len(d1);
    if (!(l0 > 1e-6f) || !(l1 > 1e-6f)) return identity();
    const V3 u = mul(d0, 1.0f / l0), u2 = mul(d1, 1.0f / l1);
    const float k = l1 / l0;
    // S = thick I + (k - thick) u u^T (column vectors), R: u -> u2 (Rodrigues), L = R S, p' = L (p - C) + C.
    float S[3][3], R[3][3], L[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) S[i][j] = (i == j ? thick : 0.0f) + (k - thick) * u[size_t(i)] * u[size_t(j)];
    V3 ax = cross(u, u2);
    const float sn = len(ax), cs = std::clamp(dot(u, u2), -1.0f, 1.0f);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R[i][j] = i == j ? 1.0f : 0.0f;
    if (sn > 1e-7f) {
        ax = mul(ax, 1.0f / sn);
        const float K[3][3] = {{0, -ax[2], ax[1]}, {ax[2], 0, -ax[0]}, {-ax[1], ax[0], 0}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) R[i][j] = (i == j ? cs : 0.0f) + sn * K[i][j] + (1.0f - cs) * ax[size_t(i)] * ax[size_t(j)];
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) L[i][j] = R[i][0] * S[0][j] + R[i][1] * S[1][j] + R[i][2] * S[2][j];
    Mat4 m = identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m[size_t(r * 4 + c)] = L[c][r];  // row-vector form: M = L^T
    for (int c = 0; c < 3; ++c) m[size_t(12 + c)] = C[size_t(c)] - (L[c][0] * C[0] + L[c][1] * C[1] + L[c][2] * C[2]);
    return m;
}

std::shared_ptr<const BakedMesh> proceduralMesh(uint32_t key) {
    std::lock_guard<std::mutex> l(gMu);
    auto it = gProcedural.find(key);
    return it == gProcedural.end() ? nullptr : it->second;
}
const Derived* derivedMesh(uint32_t key) {
    std::lock_guard<std::mutex> l(gMu);
    auto it = gDerived.find(key);
    return it == gDerived.end() ? nullptr : &it->second;  // (entries are never erased: the pointer stays valid)
}
uint32_t baseMesh(uint32_t key) {
    const Derived* d = derivedMesh(key);
    return d ? d->base : key;
}

namespace {

uint32_t registerProcedural(const std::string& what, const Builder& b, const DpcFile* dpc) {
    uint32_t key = keyOf(what);
    while (dpc && dpc->find(key)) key = keyOf(what + "+");  // never a key the archive uses (CRC clash: practically never)
    std::lock_guard<std::mutex> l(gMu);
    if (!gProcedural.count(key)) gProcedural[key] = b.bake();
    return key;
}
uint32_t registerDerived(const Derived& d, const DpcFile* dpc) {
    char b[200];
    std::string what = "DERIVED>" + std::to_string(d.base) + ">C" + std::to_string(d.carbon);
    std::snprintf(b, sizeof b, ">%.3f,%.3f,%.3f", d.carbonParams[0], d.carbonParams[1], d.carbonParams[2]);
    what += b;
    for (uint32_t h : d.hiddenMaterials) what += ">H" + std::to_string(h);
    for (uint32_t c : d.carbonMaterials) what += ">K" + std::to_string(c);
    uint32_t key = keyOf(what);
    while (dpc && dpc->find(key)) key = keyOf(what + "+");
    std::lock_guard<std::mutex> l(gMu);
    gDerived.emplace(key, d);
    return key;
}

}  // namespace

// ---- the plan

Plan makePlan(const AssembledVehicle& av, const VehicleSetup& setup, const Options& o) {
    Plan p;
    p.stages = setup.upgStages;
    p.cls = upg::classOf(setup.name);
    p.opponent = o.opponent;
    p.parts.resize(av.parts.size());
    bool any = false;
    for (int s : p.stages) any = any || s > 0;
    if (!enabled() || !upg::enabled() || !setup.upgraded || !any || p.cls < 0 || av.dpc == nullptr) return p;
    p.active = true;
    p.lift = setup.lift, p.widenFront = setup.widenFront, p.widenRear = setup.widenRear;
    const DpcFile& dpc = *av.dpc;
    auto note = [&](const std::string& s) { p.notes.push_back(s); };
    const bool bike = setup.bike || p.cls == upg::Bike;
    const bool enclosed = p.cls == upg::Muscle || p.cls == upg::Suv || p.cls == upg::Truck || p.cls == upg::Dragster;
    const bool roadName = has(upper(setup.name), "_ROAD") || p.cls == upg::Muscle || p.cls == upg::Dragster;
    const int sS = p.stages[upg::Suspension], sT = p.stages[upg::Tires], sE = p.stages[upg::Engine], sW = p.stages[upg::Weight],
              sNP = p.stages[upg::NitroPower], sNT = p.stages[upg::NitroTank];
    const std::string tag = setup.name + ">L" + std::to_string(av.lod) + (o.opponent ? ">O" : ">P");

    // Stock (pre-U1) hubs: the setup's hubs with the lift and widening taken out.
    struct Hub {
        V3 stock{}, now{};
        float radius = 0.3f;
        int side = 1;  // +1 left (+x), -1 right
        bool front = false;
        float widen = 0;
    };
    std::vector<Hub> hubs;
    float halfTrack = 0;
    for (const auto& w : setup.wheels) {
        Hub h;
        h.front = w.front;
        h.widen = w.front ? setup.widenFront : setup.widenRear;
        h.side = w.hub[0] >= 0 ? 1 : -1;
        h.now = {w.hub[0], w.hub[1], w.hub[2]};
        h.stock = {w.hub[0] - float(h.side) * h.widen, w.hub[1] + setup.lift, w.hub[2]};
        h.radius = w.radius;
        halfTrack = std::max(halfTrack, std::fabs(h.stock[0]));
        hubs.push_back(h);
    }
    if (hubs.empty()) {
        p.active = false;
        return p;
    }
    auto nearestHub = [&](V3 q, bool useNow) {
        int best = 0;
        float bd = 1e9f;
        for (size_t i = 0; i < hubs.size(); ++i) {
            const float d = len(sub(useNow ? hubs[i].now : hubs[i].stock, q));
            if (d < bd) bd = d, best = int(i);
        }
        return best;
    };
    float rearZ = 1e9f, frontZ = -1e9f;
    int rearHub = 0, frontHub = 0;
    for (size_t i = 0; i < hubs.size(); ++i) {
        if (hubs[i].stock[2] < rearZ) rearZ = hubs[i].stock[2], rearHub = int(i);
        if (hubs[i].stock[2] > frontZ) frontZ = hubs[i].stock[2], frontHub = int(i);
    }

    // Measure every drawn part.
    std::vector<Geo> geo(av.parts.size());
    std::vector<std::string> names(av.parts.size());
    for (size_t k = 0; k < av.parts.size(); ++k) {
        const PlacedPart& part = av.parts[k];
        if (!part.drawn()) continue;
        names[k] = av.names ? upper((*av.names)(part.objectName)) : std::string();
        geo[k] = measure(dpc, part);
    }
    auto isWheel = [&](size_t k) { return av.parts[k].placement == Placement::Wheel || av.parts[k].placement == Placement::WheelFlipped; };

    // ---- part roles
    enum Role { None, Wheel, Upright, Axle, Arm, Shock, Fork, Engine, ExhaustPart, Steer, Body };
    std::vector<Role> role(av.parts.size(), None);
    for (size_t k = 0; k < av.parts.size(); ++k) {
        if (!av.parts[k].drawn() || !geo[k].ok) continue;
        const std::string& n = names[k];
        const PlacedPart& part = av.parts[k];
        const V3 T{float(part.world[12]), float(part.world[13]), float(part.world[14])};
        if (isWheel(k)) role[k] = Wheel;
        else if (has(n, "STEER")) role[k] = Steer;
        else if (has(n, "EXHAUST")) role[k] = ExhaustPart;
        else if (has(n, "ENGINE") || has(n, "MOTOR")) role[k] = Engine;
        else if (has(n, "BREAK") || has(n, "BRAKE")) role[k] = Upright;
        else if (has(n, "SUSP") || has(n, "SHOCK") || has(n, "AXLE") || has(n, "SWING") || (bike && has(n, "FORK"))) {
            const Hub& h = hubs[size_t(nearestHub(T, false))];
            const Geo& g = geo[k];
            if (part.placement == Placement::LinkM && len(sub(T, h.stock)) < 0.15f) role[k] = Upright;
            else if (!bike && g.lo[0] < -0.2f && g.hi[0] > 0.2f && std::fabs(g.centre()[1] - h.stock[1]) < 0.4f) role[k] = Axle;
            else role[k] = Arm;  // refined to Shock / Fork below
        } else role[k] = Body;
    }

    // ---- the height map of the body (hull and the other non-moving chassis parts)
    HeightMap hm;
    V3 blo{1e9f, 1e9f, 1e9f}, bhi{-1e9f, -1e9f, -1e9f};
    for (size_t k = 0; k < av.parts.size(); ++k)
        if (role[k] == Body || role[k] == Engine)
            for (int a = 0; a < 3; ++a) blo[size_t(a)] = std::min(blo[size_t(a)], geo[k].lo[size_t(a)]), bhi[size_t(a)] = std::max(bhi[size_t(a)], geo[k].hi[size_t(a)]);
    if (blo[0] < bhi[0]) {
        hm.init(blo, bhi);
        for (size_t k = 0; k < av.parts.size(); ++k)
            if (role[k] == Body || role[k] == Engine)
                for (const auto& b : geo[k].batches)
                    for (size_t t = 0; t + 2 < b.t.size(); t += 3) hm.raster(geo[k].v[b.t[t]], geo[k].v[b.t[t + 1]], geo[k].v[b.t[t + 2]]);
    }

    Builder chassis;  // every chassis extra of the player in one mesh
    std::vector<Builder> wheelX(hubs.size());  // per setup wheel (spacers, stripes, lugs)
    std::vector<Builder> wheelSpacer(hubs.size());  // opponents: spacers only
    const float lift = setup.lift;
    const V3 drop{0, -lift, 0};

    // ---- suspension: re-aim / stretch the parts at the lowered hubs, thicker shocks, coloured coils
    std::vector<bool> shockNear(hubs.size(), false);
    struct LiveBuilder {
        Builder b;
        Live live;
        std::string what;
    };
    std::vector<LiveBuilder> liveBuilders;  // [suspanim]
    const Mat springMat = sS >= 3 ? Mat::SpringS3 : sS == 2 ? Mat::SpringS2 : Mat::SpringS1;
    if (sS > 0) {
        for (size_t k = 0; k < av.parts.size(); ++k) {
            PartEdit& e = p.parts[k];
            if (role[k] == Upright || role[k] == Axle) {
                e.adjust = translation(drop), e.moved = lift > 0, e.why = role[k] == Upright ? "upright" : "axle";
                // [suspanim] follows its hub (an axle: the mean of the pair at that end, rolling with their difference)
                const PlacedPart& part = av.parts[k];
                const V3 T{float(part.world[12]), float(part.world[13]), float(part.world[14])};
                const int w = nearestHub(T, false);
                e.live.kind = Live::Translate, e.live.wheel = w, e.live.D = drop, e.live.pivot = hubs[size_t(w)].stock;
                e.live.steer = role[k] == Upright;
                if (role[k] == Axle)
                    for (size_t j = 0; j < hubs.size(); ++j)
                        if (int(j) != w && hubs[j].front == hubs[size_t(w)].front && hubs[j].side != hubs[size_t(w)].side &&
                            std::fabs(hubs[j].stock[2] - hubs[size_t(w)].stock[2]) < 0.3f)
                            e.live.wheel2 = int(j);
                continue;
            }
            if (role[k] != Arm) continue;
            const Axis ax = principal(usedVerts(geo[k]));
            if (!ax.ok) continue;
            const int hi = nearestHub(ax.c, false);
            const Hub& h = hubs[size_t(hi)];
            const bool slender = ax.perp < 0.35f * ax.length;
            const bool vertical = std::fabs(ax.u[1]) > 0.55f;
            const bool fork = bike && (has(names[k], "FORK") || has(names[k], "SUSPF"));
            const bool shock = !fork && slender && vertical && (av.parts[k].shape == PartShape::SkinBindPose || bike || has(names[k], "2") || has(names[k], "3"));
            V3 C = ax.e0, B = ax.e1;
            if (len(sub(C, h.stock)) < len(sub(B, h.stock))) std::swap(C, B);
            const V3 HC = sub(h.stock, C);
            float sB = std::clamp(dot(sub(B, C), HC) / std::max(dot(HC, HC), 1e-6f), 0.0f, 1.0f);
            if (len(sub(B, h.stock)) < 0.35f * h.radius) sB = 1.0f;
            const V3 B2 = add(B, mul(drop, sB));
            const float thick = (shock || fork) ? 1.0f + 0.1f * float(sS) : 1.0f + 0.04f * float(sS);
            e.adjust = segmentMap(C, B, B2, thick);
            e.moved = !isIdentity(e.adjust);
            e.why = fork ? "fork" : shock ? "shock" : "arm";
            e.live.kind = Live::Segment, e.live.wheel = hi, e.live.C = C, e.live.B = B, e.live.D = drop, e.live.sB = sB, e.live.thick = thick;
            e.live.steer = fork && h.front;  // [suspanim]
            if (shock || fork) {
                role[k] = shock ? Shock : Fork;
                shockNear[size_t(hi)] = true;
            }
            char b[160];
            std::snprintf(b, sizeof b, "%s %s: C (%.2f %.2f %.2f) B (%.2f %.2f %.2f) s %.2f stretch x%.3f thick x%.2f", e.why.c_str(), names[k].c_str(),
                          C[0], C[1], C[2], B[0], B[1], B[2], sB, len(sub(B2, C)) / std::max(len(sub(B, C)), 1e-6f), thick);
            note(b);
            if (!o.opponent && shock) {  // a coloured coil around the stretched shock
                const V3 a = add(C, mul(sub(B2, C), 0.14f)), z = add(C, mul(sub(B2, C), 0.86f));
                const float wire = std::clamp(ax.perp * 0.16f, 0.005f, 0.016f);
                const float R = std::clamp(ax.perp * thick * 0.95f + wire, 0.022f, 0.11f);
                const V3 u = norm(sub(z, a)), e1 = anyPerp(u), e2 = cross(u, e1);
                const float L = len(sub(z, a));
                const int turns = std::clamp(int(L / (R * 1.5f)), 4, 10);
                const int steps = turns * 14;
                std::vector<V3> path;
                for (int i = 0; i <= steps; ++i) {
                    const float t = float(i) / float(steps), ph = 2.0f * float(M_PI) * float(turns) * t;
                    path.push_back(add(add(a, mul(u, L * t)), add(mul(e1, R * std::cos(ph)), mul(e2, R * std::sin(ph)))));
                }
                Builder coil;  // [suspanim] its own mesh: it follows the shock (C fixed, the lower end with the hub)
                coil.sweep(springMat, path, wire, 6);
                Live lv;
                lv.kind = Live::Segment, lv.wheel = hi, lv.C = C, lv.B = B2, lv.sB = sB, lv.thick = 1;
                liveBuilders.push_back({std::move(coil), lv, "coil" + std::to_string(k)});
            }
        }
        // Wheels without a shock part nearby (the custom packs, enclosed bodies whose dampers are not modelled): a
        // procedural coil-over between the arm at the lowered hub and the body above it.
        if (!o.opponent && !bike)
            for (size_t i = 0; i < hubs.size(); ++i) {
                if (shockNear[i]) continue;
                const Hub& h = hubs[i];
                const float s = float(h.side), r = h.radius;
                const V3 lo = {h.stock[0] * 0.80f, h.stock[1] - lift + 0.02f, h.stock[2] + (h.front ? -0.10f : 0.10f) * r};
                const V3 up = {h.stock[0] * 0.55f, h.stock[1] + 0.85f * r, h.stock[2] + (h.front ? -0.12f : 0.12f) * r};
                (void)s;
                const float R = std::clamp(0.13f * r, 0.03f, 0.08f), wire = R * 0.17f;
                Builder co;  // [suspanim] its own mesh: the top fixed on the body, the bottom with the hub
                co.tube(Mat::Dark, lo, up, R * 0.45f, R * 0.45f, 10, true, true);
                co.tube(Mat::Chrome, lo, add(lo, mul(sub(up, lo), 0.55f)), R * 0.32f, R * 0.32f, 10, false, false);
                const V3 a = add(lo, mul(sub(up, lo), 0.12f)), z = add(lo, mul(sub(up, lo), 0.86f));
                const V3 u = norm(sub(z, a)), e1 = anyPerp(u), e2 = cross(u, e1);
                const float L = len(sub(z, a));
                const int turns = std::clamp(int(L / (R * 1.4f)), 4, 9), steps = turns * 14;
                std::vector<V3> path;
                for (int j = 0; j <= steps; ++j) {
                    const float t = float(j) / float(steps), ph = 2.0f * float(M_PI) * float(turns) * t;
                    path.push_back(add(add(a, mul(u, L * t)), add(mul(e1, R * std::cos(ph)), mul(e2, R * std::sin(ph)))));
                }
                co.sweep(springMat, path, wire, 6);
                Live lv;
                lv.kind = Live::Segment, lv.wheel = int(i), lv.C = up, lv.B = lo, lv.sB = 1, lv.thick = 1;
                liveBuilders.push_back({std::move(co), lv, "coilover" + std::to_string(i)});
                note("coil-over (no shock part) at wheel " + std::to_string(i));
            }
    }

    // ---- wheels: wider (and taller, up to the physics radius); spacers where the track grew; S2 / S3 sidewall and tread
    struct WheelGeo {
        float xmin = 0, xmax = 0, r = 0;  // relative to the part's rest hub, vehicle orientation
        int setupWheel = -1;
        bool ok = false;
    };
    std::vector<WheelGeo> wg(hubs.size());
    for (size_t k = 0; k < av.parts.size(); ++k) {
        if (role[k] != Wheel) continue;
        const PlacedPart& part = av.parts[k];
        const V3 T{float(part.world[12]), float(part.world[13]), float(part.world[14])};
        PartEdit& e = p.parts[k];
        e.wheel = nearestHub(T, true);
        e.why = "wheel";
        const std::vector<V3> pts = usedVerts(geo[k]);
        WheelGeo w;
        w.xmin = 1e9f, w.xmax = -1e9f;
        for (const V3& q : pts) {
            const V3 d = sub(q, T);
            w.xmin = std::min(w.xmin, d[0]), w.xmax = std::max(w.xmax, d[0]);
            w.r = std::max(w.r, std::sqrt(d[1] * d[1] + d[2] * d[2]));
        }
        w.ok = w.r > 0.05f && w.xmax > w.xmin;
        w.setupWheel = e.wheel;
        if (!w.ok) continue;
        if (!wg[size_t(e.wheel)].ok || w.r > wg[size_t(e.wheel)].r) wg[size_t(e.wheel)] = w;
        if (sT > 0) {
            e.axle = 1.0f + std::array<float, 3>{0.08f, 0.15f, 0.25f}[size_t(sT - 1)];
            const float want = 1.0f + 0.02f * float(sT), cap = std::max(1.0f, hubs[size_t(e.wheel)].radius / w.r);
            e.radial = std::min(want, cap);
            char b[140];
            std::snprintf(b, sizeof b, "wheel %s -> setup %d: drawn r %.3f physics r %.3f width %.3f: axle x%.2f radial x%.3f", names[k].c_str(), e.wheel, w.r,
                          hubs[size_t(e.wheel)].radius, w.xmax - w.xmin, e.axle, e.radial);
            note(b);
        }
    }
    const float axleK = sT > 0 ? 1.0f + std::array<float, 3>{0.08f, 0.15f, 0.25f}[size_t(sT - 1)] : 1.0f;
    for (size_t i = 0; i < hubs.size(); ++i) {
        const Hub& h = hubs[i];
        const WheelGeo& w = wg[i];
        if (!w.ok) continue;
        const float s = float(h.side);
        float radial = 1;
        for (const PartEdit& e : p.parts)
            if (e.wheel == int(i) && e.radial > radial) radial = e.radial;
        const float xin = (h.side > 0 ? w.xmin : w.xmax) * axleK, xout = (h.side > 0 ? w.xmax : w.xmin) * axleK;
        const float width = (w.xmax - w.xmin) * axleK, rr = w.r * radial;
        if (h.widen >= 0.01f) {  // spacer: from inside the rim to where the stock wheel's inner face was (wheel space)
            const float rs = std::clamp(0.30f * w.r, 0.04f, 0.22f);
            const V3 a{xin + s * 0.35f * width, 0, 0}, b{xin - s * h.widen, 0, 0};
            Builder& B = o.opponent ? wheelSpacer[i] : wheelX[i];
            B.tube(Mat::Spacer, a, b, rs, rs, 16, false, true);
            B.tube(Mat::Spacer, add(b, V3{s * 0.012f, 0, 0}), add(b, V3{-s * 0.004f, 0, 0}), rs * 1.35f, rs * 1.35f, 16, true, true);
            for (int n = 0; n < 5; ++n) {  // studs on the flange
                const float t = 2.0f * float(M_PI) * float(n) / 5.0f;
                const V3 c{b[0] + s * 0.012f, rs * 1.08f * std::cos(t), rs * 1.08f * std::sin(t)};
                B.tube(Mat::Chrome, c, add(c, V3{s * 0.03f, 0, 0}), rs * 0.08f, rs * 0.08f, 6, false, true);
            }
        }
        if (o.opponent || sT < 2) continue;
        // S2 / S3: a sidewall stripe on the outer face (at the sidewall's own bulge for that radius)
        const float r0 = rr * (sT >= 3 ? 0.74f : 0.78f), r1 = rr * (sT >= 3 ? 0.86f : 0.83f);
        float face = 0;
        for (size_t k = 0; k < av.parts.size(); ++k) {
            if (p.parts[k].wheel != int(i) || role[k] != Wheel) continue;
            const PlacedPart& part = av.parts[k];
            const V3 T{float(part.world[12]), float(part.world[13]), float(part.world[14])};
            for (const V3& q : usedVerts(geo[k])) {
                const V3 d = sub(q, T);
                const float rad = std::sqrt(d[1] * d[1] + d[2] * d[2]) * radial;
                if (rad >= r0 - 0.02f && rad <= r1 + 0.02f) face = std::max(face, s * d[0] * axleK);
            }
        }
        if (!(face > 0)) face = s * xout;
        wheelX[i].annulus(sT >= 3 ? Mat::StripeS3 : Mat::Stripe, {s * (face + 0.004f), 0, 0}, {s, 0, 0}, r0, r1, 40);
        if (sT >= 3 && !roadName) {  // S3 off-road: chunky shoulder blocks (mud-terrain), staggered, in the tyre's own material
            uint32_t tyreMat = 0;
            size_t most = 0;
            for (size_t k = 0; k < av.parts.size(); ++k)
                if (p.parts[k].wheel == int(i) && role[k] == Wheel)
                    for (const auto& b : geo[k].batches)
                        if (b.t.size() > most) most = b.t.size(), tyreMat = b.matId;
            wheelX[i].useFuel = tyreMat;
            const float tread = std::clamp(h.radius - rr + 0.004f, 0.006f, 0.014f);
            const int n = std::clamp(int(2.0f * float(M_PI) * rr / 0.11f), 12, 36);
            for (int row = 0; row < 2; ++row)
                for (int j = 0; j < n; ++j) {
                    const float a = 2.0f * float(M_PI) * (float(j) + 0.5f * float(row)) / float(n);
                    const V3 radialDir{0, std::cos(a), std::sin(a)}, tang{0, -std::sin(a), std::cos(a)};
                    const float edge = row == 0 ? std::max(xin, xout) : std::min(xin, xout), inward = row == 0 ? -1.0f : 1.0f;
                    const float xc = edge + inward * 0.09f * width;
                    const V3 c = add(V3{xc, 0, 0}, mul(radialDir, rr - 0.45f * tread));
                    wheelX[i].box(Mat::Plastic, c, {1, 0, 0}, radialDir, tang, {0.10f * width, tread * 0.8f, std::clamp(0.30f * rr / float(n) * 3.2f, 0.018f, 0.06f)});
                }
            wheelX[i].useFuel = 0;
        }
    }

    // ---- fender flares over the widened wheels of enclosed bodies (where a fender sits above the stock wheel)
    if (!o.opponent && sS > 0 && enclosed && hm.nx > 0)
        for (size_t i = 0; i < hubs.size(); ++i) {
            const Hub& h = hubs[i];
            const WheelGeo& w = wg[i];
            if (!w.ok || h.widen < 0.015f) continue;
            const float s = float(h.side);
            const float xa = h.stock[0] - 0.12f, xb = h.stock[0] + 0.12f;
            const HeightMap::Stat st = hm.over(std::min(xa, xb), std::max(xa, xb), h.stock[2] - 0.12f, h.stock[2] + 0.12f);
            const bool fender = st.covered > 0.3f && st.max > h.stock[1] + 0.6f * w.r && st.max < h.stock[1] + w.r + 0.55f;
            if (!fender) {
                note("no fender above wheel " + std::to_string(i) + ": spacer only");
                continue;
            }
            float radial = 1;
            for (const PartEdit& e : p.parts)
                if (e.wheel == int(i) && e.radial > radial) radial = e.radial;
            const float stockOut = h.stock[0] + (h.side > 0 ? w.xmax : w.xmin);
            const float newOut = h.now[0] + (h.side > 0 ? w.xmax : w.xmin) * axleK;
            if (s * (newOut - stockOut) < 0.01f) continue;
            const float x0 = stockOut - s * 0.04f, x1 = newOut + s * 0.015f;
            const float rin = w.r * radial + 0.03f + 0.25f * lift, rout = rin + 0.04f;
            const int seg = 26;
            const float a0 = -4.0f * float(M_PI) / 180.0f, a1 = 184.0f * float(M_PI) / 180.0f;
            auto P = [&](float x, float r, float a) { return V3{x, h.stock[1] + r * std::sin(a), h.stock[2] + r * std::cos(a)}; };
            for (int j = 0; j < seg; ++j) {
                const float a = a0 + (a1 - a0) * float(j) / float(seg), b = a0 + (a1 - a0) * float(j + 1) / float(seg);
                const float m = 0.5f * (a + b);
                const V3 radial{0, std::sin(m), std::cos(m)};
                chassis.quadOut(Mat::Plastic, P(x0, rout, b), P(x1, rout, b), P(x1, rout, a), P(x0, rout, a), radial);           // outer skin
                chassis.quadOut(Mat::Plastic, P(x0, rin, a), P(x1, rin, a), P(x1, rin, b), P(x0, rin, b), mul(radial, -1));      // underside
                chassis.quadOut(Mat::Plastic, P(x1, rin, b), P(x1, rin, a), P(x1, rout, a), P(x1, rout, b), V3{s, 0, 0});        // the lip
            }
            note("flare over wheel " + std::to_string(i));
        }

    // ---- engine: tips at the outlets, ENGINE / EXHAUST parts grown, a hood scoop or intake pods
    {
        // Outlets: the SFX_EXHAUST slots, else EXHAUST parts, else EXHAUST batches of the hull, else the rear (the nitro rule).
        bool fallbackOutlets = false;
        if (!av.exhausts.empty()) {
            for (const auto& x : av.exhausts) p.outlets.push_back({x.pos, x.dir, 0.04f});
            p.outletsFromSfx = true;
        }
        if (p.outlets.empty())
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (role[k] != ExhaustPart) continue;
                const Axis ax = principal(usedVerts(geo[k]));
                if (!ax.ok) continue;
                const V3 mid{0, (blo[1] + bhi[1]) * 0.5f, (blo[2] + bhi[2]) * 0.5f};
                auto away = [&](V3 q) { return std::hypot(q[0] - mid[0], q[2] - mid[2]); };
                V3 out = ax.e1, in = ax.e0;
                if (away(in) > away(out)) std::swap(in, out);
                p.outlets.push_back({out, norm(sub(out, in)), std::clamp(ax.perp, 0.02f, 0.07f)});
                if (sE > 0) {  // the pipe grows from its inner end
                    PartEdit& e = p.parts[k];
                    const float g = 1.0f + 0.1f * float(sE);
                    e.adjust = scaleAbout(in, g, g, g), e.moved = true, e.why = "exhaust";
                }
            }
        if (p.outlets.empty())
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (role[k] != Body) continue;
                for (const auto& b : geo[k].batches) {
                    if (b.matName != asoboCrc("EXHAUST") && b.matName != asoboCrc("EXHAUST_LOD1")) continue;
                    for (int side : {-1, 1}) {
                        std::vector<V3> pts;
                        for (uint32_t t : b.t)
                            if (float(side) * geo[k].v[t][0] > -0.02f) pts.push_back(geo[k].v[t]);
                        const Axis ax = principal(pts);
                        if (!ax.ok) continue;
                        V3 out = ax.e0[2] < ax.e1[2] ? ax.e0 : ax.e1, in = ax.e0[2] < ax.e1[2] ? ax.e1 : ax.e0;
                        if (std::fabs(ax.u[0]) > 0.8f) out = float(side) * ax.e0[0] > float(side) * ax.e1[0] ? ax.e0 : ax.e1, in = out == ax.e0 ? ax.e1 : ax.e0;
                        bool dup = false;
                        for (const Outlet& q : p.outlets) dup = dup || len(sub(q.pos, out)) < 0.12f;
                        // only an end in the rear quarter of the wheelbase (a header's bend mid-vehicle is not an outlet)
                        const float zMid = 0.5f * (rearZ + frontZ), wb = frontZ - rearZ;
                        if (!dup && out[2] < zMid - 0.25f * wb) p.outlets.push_back({out, norm(sub(out, in)), std::clamp(ax.perp, 0.02f, 0.06f)});
                    }
                }
            }
        if (!p.outletsFromSfx)  // outlets from the model's pipes: aimed back (a muffler's axis can lie across the body)
            for (Outlet& q : p.outlets)
                if (q.dir[2] > -0.6f) q.dir = norm(add(q.dir, V3{0, 0, -1.4f}));
        if (p.outlets.empty()) {  // drive.mm's nitro outlet rule
            float back = 1e9f, yBot = 1e9f, yTop = -1e9f;
            for (const auto& b : setup.chassis)
                back = std::min(back, b.centre[2] - b.halfExtents[2]), yBot = std::min(yBot, b.centre[1] - b.halfExtents[1]),
                yTop = std::max(yTop, b.centre[1] + b.halfExtents[1]);
            const Hub& rh = hubs[size_t(rearHub)];
            const bool small = bike || halfTrack < 0.6f;
            const float z = small ? rh.stock[2] - 0.5f * rh.radius - 0.05f : std::min(rh.stock[2] - 0.9f * rh.radius, back) - 0.05f;
            const float y = small && yTop > yBot ? yBot + (bike ? 0.5f : 0.6f) * (yTop - yBot) : rh.stock[1] + 0.15f * rh.radius;
            const std::vector<float> xs = bike ? std::vector<float>{-0.12f} : small ? std::vector<float>{-0.3f, 0.3f} : std::vector<float>{-0.35f * halfTrack, 0.35f * halfTrack};
            for (float x : xs) p.outlets.push_back({{x, y, z}, norm(V3{0, 0.15f, -1}), 0.035f});
            fallbackOutlets = true;
            note("outlets: the rear rule (no exhaust in the model)");
        }
        // Exhaust stacks modelled as ADD parts (tall, up to the roof: TRUCK_ALL_4's Add1 / Add2): grown from their base, the
        // outlets on them with them.
        if (sE > 0 && !o.opponent)
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (role[k] != Body || !has(names[k], "ADD")) continue;
                const Geo& g = geo[k];
                const float tall = g.hi[1] - g.lo[1], wide = std::max(g.hi[0] - g.lo[0], g.hi[2] - g.lo[2]);
                if (!(tall > 2.5f * wide) || g.hi[1] < bhi[1] - 0.6f) continue;
                const float gk = 1.0f + 0.1f * float(sE);
                PartEdit& e = p.parts[k];
                e.adjust = scaleAbout({g.centre()[0], g.lo[1], g.centre()[2]}, gk, gk, gk), e.moved = true, e.why = "stack";
                for (Outlet& q : p.outlets)
                    if (q.pos[0] > g.lo[0] - 0.1f && q.pos[0] < g.hi[0] + 0.1f && q.pos[2] > g.lo[2] - 0.1f && q.pos[2] < g.hi[2] + 0.1f && q.pos[1] > g.lo[1])
                        q.pos = xform(e.adjust, q.pos);
                note("stack " + names[k] + " grown x" + std::to_string(gk).substr(0, 4));
            }
        const bool pack = custom::find(setup.name) != nullptr;
        for (const Outlet& q : p.outlets) {
            char b[120];
            std::snprintf(b, sizeof b, "outlet (%.2f %.2f %.2f) dir (%.2f %.2f %.2f) r %.3f%s", q.pos[0], q.pos[1], q.pos[2], q.dir[0], q.dir[1], q.dir[2],
                          q.radius, p.outletsFromSfx ? " (SFX_EXHAUST)" : fallbackOutlets ? " (rear rule)" : "");
            note(b);
        }
        if (sE > 0 && !o.opponent) {
            const float k = std::array<float, 3>{1.2f, 1.45f, 1.7f}[size_t(sE - 1)];
            for (Outlet& q : p.outlets) {
                const bool small = bike || halfTrack < 0.6f;
                const float rt = std::clamp(q.radius * k, small ? 0.018f : 0.025f, small ? 0.032f + 0.004f * float(sE) : 0.08f), L = rt * 3.6f;
                const V3 a = sub(q.pos, mul(q.dir, 0.35f * L)), b = add(q.pos, mul(q.dir, 0.65f * L));
                chassis.tube(Mat::Chrome, a, b, rt * 0.92f, rt, 16, true, false);
                if (fallbackOutlets || (pack && std::fabs(q.dir[2]) > 0.7f)) {  // no pipe in the model (or a pack's tailpipe under the body): one runs forward to the tip
                    const V3 fwd = norm(V3{-q.dir[0], 0, -q.dir[2]});
                    chassis.tube(Mat::Dark, add(a, mul(fwd, 0.45f + 0.4f * halfTrack)), add(a, mul(fwd, 0.01f)), rt * 0.62f, rt * 0.62f, 10, true, false);
                }
                chassis.tube(Mat::Dark, b, sub(b, mul(q.dir, 0.5f * L)), rt * 0.80f, rt * 0.80f, 16, false, true, true);
                chassis.annulus(Mat::Chrome, b, q.dir, rt * 0.80f, rt, 16);
                q.pos = add(b, mul(q.dir, 0.01f));  // the flame leaves the tip
            }
            note("tips x" + std::to_string(k).substr(0, 4) + " at " + std::to_string(p.outlets.size()) + " outlets");
        }
        // ENGINE parts: grown about their base
        bool enginePart = false;
        V3 engTop{}, engLo{}, engHi{};
        for (size_t k = 0; k < av.parts.size(); ++k) {
            if (role[k] != Engine) continue;
            enginePart = true;
            const Geo& g = geo[k];
            if (sE > 0 && !o.opponent) {
                PartEdit& e = p.parts[k];
                const float gk = 1.0f + 0.05f * float(sE);
                e.adjust = scaleAbout({g.centre()[0], g.lo[1], g.centre()[2]}, gk, gk, gk), e.moved = true, e.why = "engine";
            }
            if (g.hi[1] > engTop[1] || engTop == V3{}) engTop = {g.centre()[0], g.hi[1], g.centre()[2]}, engLo = g.lo, engHi = g.hi;
        }
        if (sE >= 2 && !o.opponent) {
            bool scoop = false;
            if (enclosed && hm.nx > 0) {
                const float w = sE >= 3 ? 0.46f : 0.36f, d = sE >= 3 ? 0.70f : 0.50f, hgt = sE >= 3 ? 0.10f : 0.065f;
                const Hub& fh = hubs[size_t(frontHub)];
                for (float dz : {0.25f, 0.45f, 0.05f, 0.65f, -0.15f}) {
                    const float zc = fh.stock[2] + dz;
                    const HeightMap::Stat st = hm.over(-w * 0.5f, w * 0.5f, zc - d * 0.5f, zc + d * 0.5f);
                    if (st.covered < 0.95f || st.max - st.min > 0.11f || st.max > bhi[1] - 0.22f || st.max < fh.stock[1] + 0.15f) continue;
                    // A wedge: open (dark) front, top falling to the rear, body colour.
                    const float yb = st.min - 0.01f, yf = st.max + hgt, yr = st.max + 0.015f;
                    const float zf = zc + d * 0.5f, zr = zc - d * 0.5f, xw = w * 0.5f, xt = w * 0.40f;
                    const V3 fL{xw, yb, zf}, fR{-xw, yb, zf}, fLt{xt, yf, zf}, fRt{-xt, yf, zf};
                    const V3 rL{xw, yb, zr}, rR{-xw, yb, zr}, rLt{xt, yr, zr}, rRt{-xt, yr, zr};
                    const Mat skinM = sW > 0 ? Mat::Carbon : Mat::Paint;  // body colour, carbon with a weight stage
                    chassis.quadOut(skinM, fLt, rLt, rRt, fRt, V3{0, 1, 0});   // top
                    chassis.quadOut(skinM, fL, rL, rLt, fLt, V3{1, 0, 0});     // left side
                    chassis.quadOut(skinM, rR, fR, fRt, rRt, V3{-1, 0, 0});    // right side
                    chassis.quadOut(skinM, rL, rR, rRt, rLt, V3{0, 0, -1});    // back
                    chassis.quadOut(Mat::Dark, fR, fL, fLt, fRt, V3{0, 0, 1});      // the opening
                    {  // a chrome frame round the opening
                        const float t = 0.012f;
                        const V3 f{0, 0, 0.003f};
                        chassis.quadOut(Mat::Chrome, add(fR, f), add(fL, f), add(add(fL, f), V3{0, t, 0}), add(add(fR, f), V3{0, t, 0}), V3{0, 0, 1});
                        chassis.quadOut(Mat::Chrome, add(add(fRt, f), V3{0, -t, 0}), add(add(fLt, f), V3{0, -t, 0}), add(fLt, f), add(fRt, f), V3{0, 0, 1});
                    }
                    scoop = true;
                    char b[120];
                    std::snprintf(b, sizeof b, "hood scoop %.2f x %.2f x %.2f at z %.2f on y %.2f..%.2f", w, d, hgt, zc, st.min, st.max);
                    note(b);
                    break;
                }
            }
            if (!scoop && enginePart) {  // intake pods on top of the engine
                const int n = sE >= 3 ? 4 : 2;
                const float sx = engHi[0] - engLo[0], sz = engHi[2] - engLo[2];
                const bool alongZ = sz >= sx;
                const float span = std::min(alongZ ? sz : sx, 0.6f), r = std::clamp(span / float(n) * 0.32f, 0.022f, 0.05f);
                const float gk = 1.0f + 0.05f * float(sE);
                const V3 top{engTop[0], engLo[1] + (engTop[1] - engLo[1]) * gk, engTop[2]};
                for (int j = 0; j < n; ++j) {
                    const float off = (float(j) - 0.5f * float(n - 1)) * r * 2.6f;
                    const V3 c = add(top, alongZ ? V3{0, 0, off} : V3{off, 0, 0});
                    chassis.tube(Mat::Chrome, add(c, V3{0, -0.02f, 0}), add(c, V3{0, r * 1.2f, 0}), r * 0.75f, r * 0.85f, 14, true, false);
                    chassis.tube(Mat::Filter, add(c, V3{0, r * 1.2f, 0}), add(c, V3{0, r * 2.4f, 0}), r, r * 0.9f, 14, false, true);
                }
                note("intake pods x" + std::to_string(n) + " on the engine");
            }
        }
        if (sE > 0 && !o.opponent) {
            p.engineGainDb = 1.5f * float(sE);
            p.engineShelfDb = 2.0f * float(sE);
        }
    }

    // ---- weight: carbon paint (derived meshes), optional bodywork hidden
    std::vector<std::vector<uint32_t>> hiddenBatches(av.parts.size());
    if (sW > 0) {
        p.carbon = sW;
        if (sW >= 2 && !o.opponent)
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (!av.parts[k].drawn() || role[k] != Body) continue;
                const std::string& n = names[k];
                bool hide = has(n, "BUMPER") || has(n, "SPOILER") || has(n, "RETRO") || has(n, "MIRROR") || has(n, "WHEELADD") || has(n, "FENDER");
                if (!hide && has(n, "ADD") && av.parts[k].dyn >= 0 && size_t(av.parts[k].dyn) < av.dyn.size()) {
                    const vlinks::DynRecord& d = av.dyn[size_t(av.parts[k].dyn)];
                    const Geo& g = geo[k];
                    const float tall = g.hi[1] - g.lo[1], wide = std::max(g.hi[0] - g.lo[0], g.hi[2] - g.lo[2]);
                    hide = !d.points.empty() && d.rpm == 0 && d.tpm == 0 && tall < 1.5f * wide;  // loose panels, flaps, spare wheels
                }
                if (hide) {
                    p.parts[k].hide = true, p.parts[k].why = "hidden: " + n;
                    note("hidden part " + n);
                }
            }
        if (sW >= 3 && !o.opponent)
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (role[k] != Body || p.parts[k].hide) continue;
                for (const auto& b : geo[k].batches) {
                    static const uint32_t bumper = asoboCrc("BUMPER"), net = asoboCrc("NET"), wheel = asoboCrc("WHEEL"), rim = asoboCrc("RIM"),
                                          rims = asoboCrc("RIMS"), tires = asoboCrc("TIRES"), tire = asoboCrc("TIRE"), spare = asoboCrc("SPARE");
                    const uint32_t m = b.matName;
                    bool hide = m == bumper || m == net;
                    if (m == wheel || m == rim || m == rims || m == tires || m == tire || m == spare) {  // a spare wheel: away from every hub
                        const V3 c = mul(add(b.lo, b.hi), 0.5f);
                        hide = true;
                        for (const Hub& h : hubs) hide = hide && std::hypot(c[0] - h.stock[0], c[2] - h.stock[2]) > 0.6f * h.radius;
                    }
                    if (hide && std::find(hiddenBatches[k].begin(), hiddenBatches[k].end(), m) == hiddenBatches[k].end()) {
                        hiddenBatches[k].push_back(m);
                        char s[96];
                        std::snprintf(s, sizeof s, "hidden batch %08x of %s", m, names[k].c_str());
                        note(s);
                    }
                }
            }
    }
    // Derived meshes: carbon on the paint (codes 17 / 19) and the hidden batches.
    {
        float pzLo = 1e9f, pzHi = -1e9f;
        for (size_t k = 0; k < av.parts.size(); ++k)
            if (role[k] == Body || role[k] == Engine)
                for (const auto& b : geo[k].batches)
                    if (b.code == 17 || b.code == 19) pzLo = std::min(pzLo, b.lo[2]), pzHi = std::max(pzHi, b.hi[2]);
        // The body skin: FUEL paints many bodies with a texture (code 18) and keeps the palette paint for a few panels or
        // decals, so the carbon also tints the largest opaque code 0 / 18 material of the biggest body part (not glass,
        // lamps, chrome, engine, wheels, exhaust).
        std::vector<uint32_t> skin;
        if (p.carbon > 0) {
            static const std::vector<uint32_t> no = [] {
                std::vector<uint32_t> v;
                for (const char* n : {"GLASS", "LIGHT", "LIGHT_HEAD", "LIGHT_BACK", "LIGHT_STOP", "LIGHT_POSITION", "CHROME", "ENGINE", "ENGINE1",
                                      "MAT_ENGINE", "TIRE", "TIRES", "WHEEL", "RIM", "RIMS", "EXHAUST", "EXHAUST_LOD1", "INTERIOR", "SEAT", "NET",
                                      "BUMPER", "SUSP", "STEER", "WINDOW", "VITRE"})
                    v.push_back(asoboCrc(n));
                return v;
            }();
            size_t big = SIZE_MAX, bigTris = 0;
            for (size_t k = 0; k < av.parts.size(); ++k) {
                if (role[k] != Body || p.parts[k].hide) continue;
                size_t t = 0;
                for (const auto& b : geo[k].batches) t += b.t.size();
                if (t > bigTris) bigTris = t, big = k;
            }
            if (big != SIZE_MAX) {
                std::map<uint32_t, size_t> tris;
                for (const auto& b : geo[big].batches)
                    if ((b.code == 0 || b.code == 18) && std::find(no.begin(), no.end(), b.matName) == no.end()) tris[b.matName] += b.t.size();
                uint32_t best = 0;
                size_t bt = 0;
                for (const auto& [n, t] : tris)
                    if (t > bt) bt = t, best = n;
                if (best != 0 && bt * 4 >= bigTris) {
                    skin.push_back(best);
                    char b[96];
                    std::snprintf(b, sizeof b, "carbon body skin %08x (%zu of %zu triangles)", best, bt / 3, bigTris / 3);
                    note(b);
                }
            }
        }
        const float zFront = pzLo < pzHi ? pzLo + 0.58f * (pzHi - pzLo) : 0.0f;
        for (size_t k = 0; k < av.parts.size(); ++k) {
            if (!av.parts[k].drawn() || p.parts[k].hide || role[k] == Wheel) continue;
            bool paint = false;
            for (const auto& b : geo[k].batches)
                paint = paint || b.code == 17 || b.code == 19 || (role[k] == Body && std::find(skin.begin(), skin.end(), b.matName) != skin.end());
            const bool carbon = p.carbon > 0 && paint && (role[k] == Body || role[k] == Engine || role[k] == ExhaustPart);
            if (!carbon && hiddenBatches[k].empty()) continue;
            Derived d;
            d.base = av.parts[k].meshId;
            d.hiddenMaterials = hiddenBatches[k];
            if (carbon) {
                d.carbon = p.carbon;
                const float tz = float(av.parts[k].world[14]);  // (vehicle parts carry no rotation but the wheels')
                static const bool debug = std::getenv("FUEL_U5_CARBON_DEBUG") != nullptr;  // the carbon drawn red (checks)
                d.carbonParams = {debug ? 9.0f : float(p.carbon), zFront - tz, 0.012f, 0};
                if (role[k] == Body) d.carbonMaterials = skin;
            }
            p.parts[k].mesh = registerDerived(d, &dpc);
        }
    }

    // ---- nitrous: flame size (power), bottles (tank)
    if (sNP > 0) {
        p.flameLength = std::array<float, 3>{1.25f, 1.5f, 1.8f}[size_t(sNP - 1)];
        p.flameSize = std::array<float, 3>{1.1f, 1.2f, 1.3f}[size_t(sNP - 1)];
    }
    if (sNT > 0 && !o.opponent) {
        const int n = sNT;
        const float grow = 1.0f + 0.06f * float(n - 1);
        auto bottle = [&](V3 a, V3 b, float r) {  // a = base, b = the domed end with the valve
            const V3 u = norm(sub(b, a));
            const V3 bd = sub(b, mul(u, r));
            chassis.tube(Mat::Bottle, a, bd, r, r, 18, true, false);
            chassis.dome(Mat::Bottle, bd, u, r, 18);
            chassis.tube(Mat::Chrome, b, add(b, mul(u, r * 0.55f)), r * 0.22f, r * 0.22f, 10, false, true);
            chassis.tube(Mat::Chrome, add(b, mul(u, r * 0.35f)), add(add(b, mul(u, r * 0.35f)), mul(anyPerp(u), r * 0.5f)), r * 0.1f, r * 0.1f, 6, false, true);
            for (float t : {0.22f, 0.62f}) {  // straps
                const V3 c = add(a, mul(sub(bd, a), t));
                chassis.tube(Mat::Dark, sub(c, mul(u, r * 0.12f)), add(c, mul(u, r * 0.12f)), r * 1.04f, r * 1.04f, 18, false, false);
            }
        };
        if (bike) {
            const Hub& rh = hubs[size_t(rearHub)];
            const float r = 0.042f * grow, L = 0.30f;
            const float y = rh.stock[1] + 0.36f, z = rh.stock[2] + 0.30f;
            float hw = 0.08f;  // the tail's half width there
            for (size_t k = 0; k < av.parts.size(); ++k)
                if (role[k] == Body)
                    for (const V3& q : usedVerts(geo[k]))
                        if (std::fabs(q[2] - z) < L * 0.6f && std::fabs(q[1] - y) < r * 1.5f) hw = std::max(hw, std::fabs(q[0]));
            hw = std::min(hw, 0.24f);
            const std::vector<float> xs = n == 1 ? std::vector<float>{1} : std::vector<float>{1, -1};
            for (float sx : xs) bottle({sx * (hw + r + 0.01f), y, z - L * 0.5f}, {sx * (hw + r + 0.01f), y, z + L * 0.5f}, r);
            if (n >= 3 && hm.nx > 0) {
                const HeightMap::Stat st = hm.over(-0.05f, 0.05f, rh.stock[2] - 0.05f, rh.stock[2] + 0.25f);
                const float yt = (st.max > HeightMap::kNone ? st.max : y + 0.2f) + r + 0.005f;
                bottle({0, yt, rh.stock[2] - 0.05f}, {0, yt, rh.stock[2] + 0.30f}, r * 0.9f);
            }
            note("bottles x" + std::to_string(n) + " by the tail");
        } else if (hm.nx > 0) {
            const float r = std::clamp(0.055f * halfTrack / 0.75f, 0.045f, 0.10f) * grow;
            const float L = std::clamp(halfTrack * 0.8f, 0.36f, 0.95f);
            const float pitch = 2.35f * r;
            const Hub& rh = hubs[size_t(rearHub)];
            float bestTop = 1e9f, bestZ = 0;
            for (float dz : {0.0f, 0.25f, -0.25f, 0.5f, -0.5f, 0.75f}) {
                const float zc = rh.stock[2] + dz;
                const HeightMap::Stat st = hm.over(-L * 0.5f, L * 0.5f, zc - pitch * 0.5f * float(n), zc + pitch * 0.5f * float(n));
                if (st.covered < 0.9f || st.max < rh.stock[1] + 0.4f * rh.radius) continue;
                if (st.max < bestTop - 0.05f) bestTop = st.max, bestZ = zc;
            }
            if (bestTop < 1e8f) {
                for (int j = 0; j < n; ++j) {
                    const float z = bestZ + (float(j) - 0.5f * float(n - 1)) * pitch;
                    const float yb = hm.over(-L * 0.5f, L * 0.5f, z - r, z + r).max + r + 0.004f;
                    const float side = j % 2 == 0 ? 1.0f : -1.0f;  // valves alternate sides
                    bottle({-side * L * 0.5f, yb, z}, {side * L * 0.5f, yb, z}, r);
                }
                char b[120];
                std::snprintf(b, sizeof b, "bottles x%d r %.3f L %.2f at z %.2f on y %.2f", n, r, L, bestZ, bestTop);
                note(b);
            } else {
                note("bottles: no place found");
            }
        }
    }

    // ---- register the extras
    auto finish = [&](Builder& b, int wheel, const std::string& what) {
        if (b.empty()) return;
        Extra x;
        x.wheel = wheel, x.what = what, x.triangles = b.triangles();
        x.key = registerProcedural(tag + ">" + what + (wheel >= 0 ? ">W" + std::to_string(wheel) : std::string()) + ">" + upg::formatStages([&] {
                                       upg::Stages s;
                                       for (int c = 0; c < upg::kCategories; ++c) s[c] = p.stages[size_t(c)];
                                       return s;
                                   }()) + ">" + std::to_string(p.lift) + "," + std::to_string(p.widenFront) + "," + std::to_string(p.widenRear),
                                   b, &dpc);
        p.extraTriangles += x.triangles;
        p.extras.push_back(x);
    };
    finish(chassis, -1, "chassis");
    for (LiveBuilder& lb : liveBuilders) {  // [suspanim]
        const size_t before = p.extras.size();
        finish(lb.b, -1, lb.what);
        if (p.extras.size() == before) continue;
        Extra x = p.extras.back();
        p.extras.pop_back();
        x.what = lb.what.rfind("coilover", 0) == 0 ? "coilover" : "coil";
        x.live = lb.live;
        p.liveExtras.push_back(x);
    }
    for (size_t i = 0; i < hubs.size(); ++i) {
        finish(wheelX[i], int(i), "wheel");
        finish(wheelSpacer[i], int(i), "spacer");
    }
    return p;
}

std::string describe(const Plan& p) {
    if (!p.active) return "no visible upgrades";
    int moved = 0, hidden = 0, derived = 0, wheels = 0;
    for (const PartEdit& e : p.parts) {
        moved += e.moved, hidden += e.hide, derived += e.mesh != 0;
        wheels += e.wheel >= 0 && (e.axle != 1 || e.radial != 1);
    }
    char b[300];
    std::snprintf(b, sizeof b, "%s%s: %d parts moved, %d hidden, %d derived meshes (carbon %d), %d wheels scaled, %zu extras (%u triangles), %zu outlets, "
                  "flame x%.2f / x%.2f, engine +%.1f dB (shelf +%.1f)",
                  upg::className(p.cls), p.opponent ? " (opponent)" : "", moved, hidden, derived, p.carbon, wheels, p.extras.size(), p.extraTriangles,
                  p.outlets.size(), p.flameLength, p.flameSize, p.engineGainDb, p.engineShelfDb);
    return b;
}

std::vector<ScenePart> restScene(const AssembledVehicle& av, const VehicleSetup& setup, const Plan& p) {
    std::vector<ScenePart> out;
    const Mat4 up = translation({0, p.active ? p.lift : 0.0f, 0});
    for (size_t k = 0; k < av.parts.size(); ++k) {
        const PlacedPart& part = av.parts[k];
        if (!part.drawn()) continue;
        const PartEdit* e = p.active && k < p.parts.size() ? &p.parts[k] : nullptr;
        if (e && e->hide) continue;
        ScenePart s;
        s.part = int(k);
        s.mesh = e && e->mesh ? e->mesh : part.meshId;
        const bool wheel = part.placement == Placement::Wheel || part.placement == Placement::WheelFlipped;
        if (!p.active) s.world = part.world;
        else if (wheel) {
            const int w = e->wheel;
            float dx = 0;
            if (w >= 0 && size_t(w) < setup.wheels.size()) {
                const float wide = setup.wheels[size_t(w)].front ? setup.widenFront : setup.widenRear;
                dx = setup.wheels[size_t(w)].hub[0] >= 0 ? wide : -wide;
            }
            s.world = matMul(matMul(scaling(e->axle, e->radial, e->radial), part.world), translation({dx, 0, 0}));
        } else {
            s.world = matMul(matMul(part.world, e->adjust), up);
        }
        out.push_back(s);
    }
    if (!p.active) return out;
    std::vector<Extra> all = p.extras;
    all.insert(all.end(), p.liveExtras.begin(), p.liveExtras.end());  // [suspanim] coils / coil-overs at the static stance
    for (const Extra& x : all) {
        ScenePart s;
        s.mesh = x.key;
        if (x.wheel < 0) s.world = up;
        else if (size_t(x.wheel) < setup.wheels.size()) {
            const auto& w = setup.wheels[size_t(x.wheel)];
            s.world = translation({w.hub[0], w.hub[1] + p.lift, w.hub[2]});  // the physics hub at rest, the body lifted
        }
        out.push_back(s);
    }
    return out;
}

}  // namespace fuel::upgvis
