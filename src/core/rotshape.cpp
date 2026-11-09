// PK28: RotShape_Z (see rotshape.hpp and notes pk28.md section 4).
#include "core/rotshape.hpp"

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/world.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace fuel::rotshape {

namespace {
constexpr uint32_t kNodeClass = 0x85D02528, kWorldClass = 0x39B67ECB;

struct Cursor {
    const std::vector<uint8_t>& b;
    size_t o = 0;
    uint32_t u32() {
        if (o + 4 > b.size()) throw std::runtime_error("RotShape_Z body too short");
        uint32_t v;
        std::memcpy(&v, b.data() + o, 4);
        o += 4;
        return v;
    }
    uint16_t u16() {
        if (o + 2 > b.size()) throw std::runtime_error("RotShape_Z body too short");
        uint16_t v;
        std::memcpy(&v, b.data() + o, 2);
        o += 2;
        return v;
    }
    const uint8_t* take(size_t n) {
        if (o + n > b.size()) throw std::runtime_error("RotShape_Z body too short");
        const uint8_t* p = b.data() + o;
        o += n;
        return p;
    }
};

std::array<float, 3> f3(const uint8_t* p) {
    std::array<float, 3> v;
    std::memcpy(v.data(), p, 12);
    return v;
}
std::array<float, 3> cross(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
std::array<float, 3> norm(const std::array<float, 3>& a) {
    const float l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    return l > 0 ? std::array<float, 3>{a[0] / l, a[1] / l, a[2] / l} : a;
}
}  // namespace

Shape parse(uint32_t id, const std::vector<uint8_t>& link, const std::vector<uint8_t>& body) {
    if (link.size() < 96) throw std::runtime_error("RotShape_Z link too short");
    Shape s;
    s.id = id;
    std::memcpy(&s.flags, link.data() + 92, 4);
    std::memcpy(&s.viewDist, link.data() + 88, 4);
    std::memcpy(s.sphere.data(), link.data() + 8, 16);
    Cursor c{body};
    const uint32_t nPos = c.u32();  // 0x777d50 -> +0x6c
    std::vector<std::array<float, 3>> pos(nPos);
    for (auto& p : pos) p = f3(c.take(12));
    const uint32_t n74 = c.u32();  // 0x813cc0 -> +0x74 (16-byte records)
    c.take(size_t(n74) * 16);
    s.unused74 = n74;
    const uint32_t nMat = c.u32();  // 0x6dd8b0 -> +0x7c
    std::vector<uint32_t> mat(nMat);
    for (auto& m : mat) m = c.u32();
    if (s.flags & 2)  // the per-entry words read again into +0x80 (vf03 0x87c8e4)
        for (auto& m : mat) m = c.u32();
    const uint32_t nAxes = c.u32();  // 0x777d50 -> +0x84 (two per point, 0x18 stride in the draw)
    std::vector<std::array<float, 3>> axes(nAxes);
    for (auto& a : axes) a = f3(c.take(12));
    const uint32_t n8c = c.u32();  // 0x740b80 -> +0x8c (8-byte records)
    c.take(size_t(n8c) * 8);
    s.unused8c = n8c;
    const uint32_t nHandles = c.u32();  // -> +0x98
    s.materials.resize(nHandles);
    for (auto& h : s.materials) h = c.u32();
    const uint32_t nf = c.u32();  // +0x9c (float)
    std::memcpy(&s.nearFade, &nf, 4);
    s.front = c.u16() != 0;  // +0xa0
    if (c.o != body.size()) throw std::runtime_error("RotShape_Z body has trailing bytes");
    if (mat.size() < nPos || axes.size() < size_t(nPos) * 2) throw std::runtime_error("RotShape_Z arrays disagree");
    s.points.resize(nPos);
    for (uint32_t i = 0; i < nPos; ++i) {
        Point& p = s.points[i];
        p.pos = pos[i];
        p.upLeft = axes[size_t(i) * 2];
        p.downRight = axes[size_t(i) * 2 + 1];
        p.material = mat[i];
    }
    return s;
}

float fade(const Shape& s, float d, float nodeScale, float viewScale, float Kfade, float nodeAlpha) {
    if (s.nearFade == 0 && s.viewDist == 0 && nodeAlpha >= 1) return 1.0f;  // 0x8834f0: view +0x160 left as it is
    if (nodeAlpha == 0) return 0.0f;
    LodView v;
    v.Kfade = Kfade;
    return lodFade(s.viewDist, s.nearFade, 0.0f, nodeScale * viewScale, d, v) * nodeAlpha;
}

const std::array<QuadVertex, 4>& quad() {
    // 0x859bc0 (asm 0x859f1e..0x859f9e): strip order, stride 0x28, uv at +0x20.
    static const std::array<QuadVertex, 4> q = {{
        {{1, 1, 1}, {1, 0}},
        {{0, 1, 1}, {0, 0}},
        {{1, 0, 0}, {1, 1}},
        {{0, 0, 0}, {0, 1}},
    }};
    return q;
}

std::array<float, 3> corner(const Shape& s, const Point& p, int k, const std::array<float, 3>& pv, const std::array<float, 3>& up, float scale,
                            float alpha) {
    if (!(alpha > 1.0f / 256.0f)) return pv;  // GOURAUD_DFLT.w <= C_INV256: collapsed to the pivot
    const auto& P = quad()[size_t(k)].P;
    std::array<float, 3> pos;
    for (int a = 0; a < 3; ++a) pos[size_t(a)] = (1 - P[size_t(a)]) * p.downRight[size_t(a)] + P[size_t(a)] * p.upLeft[size_t(a)];
    std::array<float, 3> X, Y, Z;
    if (s.front) {  // vh_front
        Z = norm(pv);
        X = cross(norm(up), Z);
        Y = cross(Z, X);
        for (int a = 0; a < 3; ++a) X[size_t(a)] *= scale, Y[size_t(a)] *= scale, Z[size_t(a)] *= -scale;
    } else {  // vh_normal
        Y = norm(up);
        X = norm(cross(Y, pv));
        Z = cross(X, Y);
        for (int a = 0; a < 3; ++a) X[size_t(a)] *= scale, Y[size_t(a)] *= scale, Z[size_t(a)] *= scale;
    }
    std::array<float, 3> v;
    for (int a = 0; a < 3; ++a) v[size_t(a)] = X[size_t(a)] * pos[0] + Y[size_t(a)] * pos[1] + Z[size_t(a)] * pos[2] + pv[size_t(a)];
    return v;
}

std::vector<Placement> collect(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, bool worldRootOnly, const std::string& sub) {
    std::vector<Placement> out;
    std::unordered_map<uint32_t, uint32_t> parent;
    std::vector<std::pair<uint32_t, Node>> nodes;
    for (uint32_t id : dpc.ids(kNodeClass)) {
        try {
            if (const auto b = dpc.body(*dpc.find(id))) {
                Node n = parseNodeBody(*b);
                parent[id] = n.parent;
                nodes.emplace_back(id, n);
            }
        } catch (const std::exception&) {
        }
    }
    uint32_t root = 0;
    if (worldRootOnly)
        for (uint32_t id : dpc.ids(kWorldClass))
            if (const auto b = dpc.body(*dpc.find(id)); b && b->size() >= 4) std::memcpy(&root, b->data(), 4);
    auto reaches = [&](uint32_t n) {
        for (int depth = 0; depth < 64 && n != 0; ++depth) {
            if (n == root) return true;
            const auto it = parent.find(n);
            if (it == parent.end()) return false;
            n = it->second;
        }
        return false;
    };
    std::unordered_map<uint32_t, std::shared_ptr<const Shape>> cache;
    for (const auto& [id, n] : nodes) {
        if (n.object == 0) continue;
        if (std::fabs(n.world[12]) < 50 && std::fabs(n.world[14]) < 50) continue;  // library copies at the origin
        if (worldRootOnly && !reaches(id)) continue;
        std::shared_ptr<const Shape> sh;
        if (auto it = cache.find(n.object); it != cache.end()) {
            sh = it->second;
        } else {
            for (const DpcFile* f : lookup) {
                const DpcFile::Entry* e = f ? f->find(n.object) : nullptr;
                if (!e) continue;
                if (e->classCrc == kClass) try {
                        if (const auto b = f->body(*e)) sh = std::make_shared<const Shape>(parse(n.object, f->link(*e), *b));
                    } catch (const std::exception&) {
                    }
                break;
            }
            cache[n.object] = sh;
        }
        if (!sh || sh->points.empty()) continue;
        Placement p;
        p.nodeId = id, p.world = n.world, p.lodScale = n.lodScale, p.shape = sh, p.sub = sub;
        out.push_back(std::move(p));
    }
    return out;
}

}  // namespace fuel::rotshape
