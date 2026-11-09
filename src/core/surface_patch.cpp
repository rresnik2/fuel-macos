#include "surface_patch.hpp"

#include "material_anim.hpp"
#include "reader.hpp"
#include "resources.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

struct Found {
    const DpcFile* file = nullptr;
    const DpcFile::Entry* entry = nullptr;
};
Found find(const std::vector<const DpcFile*>& lookup, uint32_t id) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) return {f, e};
    return {};
}

template <size_t N>
std::vector<std::array<float, N>> readFloats(ByteReader& r) {
    std::vector<std::array<float, N>> out(r.count(1'000'000));
    for (auto& v : out)
        for (float& f : v) f = r.f32();
    return out;
}

// The x87 runs at 53 bits (the game's control word); intermediates are doubles, stored state is float.
using F = float;
using D = double;

}  // namespace

SurfaceBody parseSurfaceBody(const std::vector<uint8_t>& body) {  // Surface_Z::vf03 0x80f220
    ByteReader r(body);
    SurfaceBody s;
    s.points = readFloats<3>(r);    // 0x777d50 -> +0x6c
    s.corner16 = readFloats<4>(r);  // 0x813cc0 -> +0x74
    r.skipArray(0x20);              // +0x7c (0x7bb7c0); empty in the shipped data
    r.skipArray(0x20);              // +0x84
    const uint32_t np = r.count(100'000);
    s.patches.resize(np);           // +0x8c (0x7bb830 defaults; the file record replaces them)
    for (SurfacePatch& p : s.patches) {
        const uint8_t* q = r.raw(0xb0);
        std::memcpy(&p.flags, q, 2);
        std::memcpy(p.edges.data(), q + 4, 8);
        std::memcpy(p.sphere.data(), q + 0x10, 16);
        std::memcpy(p.frame.data(), q + 0x40, 48);
        std::memcpy(p.half.data(), q + 0x70, 16);
        std::memcpy(p.corner16.data(), q + 0x80, 8);
        std::memcpy(p.colour.data(), q + 0x88, 8);
        std::memcpy(p.normal.data(), q + 0x90, 8);
        std::memcpy(p.uv0.data(), q + 0x98, 8);
        std::memcpy(p.uv1.data(), q + 0xa0, 8);
        std::memcpy(&p.materialAnim, q + 0xac, 4);
    }
    s.edges.resize(r.count(1'000'000));  // 0x80f770 -> +0x94
    for (auto& e : s.edges)
        for (uint16_t& v : e) v = r.u16();
    s.normals = readFloats<3>(r);  // +0x9c
    s.colours = readFloats<3>(r);  // +0xa4
    s.uvs = readFloats<2>(r);      // +0xac
    s.list16.resize(r.count(1'000'000));  // +0xb4
    for (uint16_t& v : s.list16) v = r.u16();
    r.skipArray(0x20);  // +0xbc (0x7bbab0)
    s.hasGrid = r.get<uint8_t>() != 0;  // then the grid (0x818350 / 0x81a1e0), not needed to draw or collide
    return s;
}

std::array<float, 64> patchControlNet(const SurfaceBody& s, const SurfacePatch& p) {  // 0x80ce20
    std::array<float, 64> o{};
    auto edge = [&](int k) -> const std::array<uint16_t, 4>& {
        if (p.edges[size_t(k)] >= s.edges.size()) throw std::runtime_error("patch edge out of range");
        return s.edges[p.edges[size_t(k)]];
    };
    auto put = [&](int slot, uint16_t point) {
        if (point >= s.points.size()) throw std::runtime_error("patch point out of range");
        const auto& v = s.points[point];
        o[size_t(slot * 4)] = v[0], o[size_t(slot * 4 + 1)] = v[1], o[size_t(slot * 4 + 2)] = v[2], o[size_t(slot * 4 + 3)] = 1.0f;
    };
    // Edge k from its start corner: corner [f], handle [2 + f], handle [3 - f], with f = flags bit k + 1.
    const int slots[4][3] = {{0, 1, 2}, {3, 7, 11}, {15, 14, 13}, {12, 8, 4}};
    for (int k = 0; k < 4; ++k) {
        const auto& e = edge(k);
        const int f = (p.flags >> (k + 1)) & 1;
        put(slots[k][0], e[size_t(f)]);
        put(slots[k][1], e[size_t(2 + f)]);
        put(slots[k][2], e[size_t(3 - f)]);
    }
    auto inner = [&](int dst, int a, int b, int corner) {
        for (int c = 0; c < 4; ++c) o[size_t(dst * 4 + c)] = F(D(o[size_t(a * 4 + c)]) + D(o[size_t(b * 4 + c)]) - D(o[size_t(corner * 4 + c)]));
    };
    inner(5, 1, 4, 0);    // in_EAX[0x14] = [4] + [0x10] - [0]
    inner(6, 7, 2, 3);    // [0x18] = [0x1c] + [8] - [0xc]
    inner(10, 11, 14, 15);  // [0x28] = [0x2c] + [0x38] - [0x3c]
    inner(9, 13, 8, 12);  // [0x24] = [0x34] + [0x20] - [0x30]
    return o;
}

namespace {

// Forward-difference state of one cubic (p0..p3) at step h: value, first, second and third differences.
struct Cubic {
    F v, d1, d2, d3;
};

}  // namespace

std::vector<SurfaceVertex> tessellatePatch(const SurfaceBody& s, const SurfacePatch& p, int n) {  // 0x846f80
    if (n < 1) return {};
    const std::array<float, 64> net = patchControlNet(s, p);
    auto at = [&](int r, int c, int k) { return D(net[size_t((r * 4 + c) * 4 + k)]); };
    const D h = 1.0 / D(n);
    const D h2 = h * h, h3 = h2 * h;  // kept on the x87 stack (asm 0x846fa1)
    const F s6 = F(h3 * 6.0);
    // Columns c = 0..3 along v (rows of the net), xyz.
    Cubic col[4][3];
    for (int c = 0; c < 4; ++c)
        for (int k = 0; k < 3; ++k) {
            const D p0 = at(0, c, k), p1 = at(1, c, k), p2 = at(2, c, k), p3 = at(3, c, k);
            const F A = F(p3 + p2 * -3.0 + p0 * -1.0 + p1 * 3.0);  // [0xa05224] = -1
            const F B = F(p2 * 3.0 + p1 * -6.0 + p0 * 3.0);
            const F C = F(p0 * -3.0 + p1 * 3.0);
            Cubic& q = col[c][k];
            q.v = F(p0);
            q.d1 = F(D(A) * h3 + D(B) * h2 + D(C) * h);
            q.d3 = F(D(s6) * A);
            q.d2 = F(D(B) * 2.0 * h2 + q.d3);
        }
    const int m = n + 1;
    std::vector<SurfaceVertex> out(size_t(m) * size_t(m));
    for (int r = 0; r < m; ++r) {
        for (int k = 0; k < 3; ++k) {
            const D q0 = col[0][k].v, q1 = col[1][k].v, q2 = col[2][k].v, q3 = col[3][k].v;
            const F A = F(q2 * -3.0 + q0 * -1.0 + q1 * 3.0 + q3);
            const F B = F(q2 * 3.0 + q1 * -6.0 + q0 * 3.0);
            const F C = F(q0 * -3.0 + q1 * 3.0);
            F v = F(q0), d1 = F(D(C) * h + D(A) * h3 + D(B) * h2), d2 = F(D(B) * (D(h2) + h2));
            for (int j = 0; j < m; ++j) {
                d2 = F(D(d2) + D(s6) * A);
                out[size_t(r * m + j)].pos[size_t(k)] = v;
                v = F(D(v) + d1);
                d1 = F(D(d2) + d1);
            }
        }
        for (auto& c4 : col)
            for (Cubic& q : c4) {
                q.v = F(D(q.v) + q.d1);
                q.d1 = F(D(q.d2) + q.d1);
                q.d2 = F(D(q.d3) + q.d2);
            }
    }
    // Corner attributes, bilinear by differences: start a0, step along u (a1 - a0) h, rows add (a3 - a0) h to the
    // start and ((a0 + a2) - (a1 + a3)) h^2 to the step.
    auto bilinear = [&](const auto& table, const std::array<uint16_t, 4>& idx, int comps, auto store) {
        for (uint16_t i : idx)
            if (i >= table.size()) throw std::runtime_error("patch attribute out of range");
        for (int k = 0; k < comps; ++k) {
            const D a0 = table[idx[0]][size_t(k)], a1 = table[idx[1]][size_t(k)], a2 = table[idx[2]][size_t(k)], a3 = table[idx[3]][size_t(k)];
            F start = F(a0), du = F((a1 - a0) * h), dv = F((a3 - a0) * h);
            const F cross = F(((a0 + a2) - (a1 + a3)) * h2);
            for (int r = 0; r < m; ++r) {
                F v = start;
                for (int j = 0; j < m; ++j) {
                    store(out[size_t(r * m + j)], k, v);
                    v = F(D(v) + du);
                }
                start = F(D(start) + dv);
                du = F(D(du) + cross);
            }
        }
    };
    bilinear(s.uvs, p.uv0, 2, [](SurfaceVertex& o, int k, F v) { o.uv0[size_t(k)] = v; });
    bilinear(s.uvs, p.uv1, 2, [](SurfaceVertex& o, int k, F v) { o.uv1[size_t(k)] = v; });
    bilinear(s.colours, p.colour, 3, [](SurfaceVertex& o, int k, F v) { o.colour[size_t(k)] = v; });
    // Normal bytes: fistp of n 127 + 128 under the game's round-down control word.
    bilinear(s.normals, p.normal, 3, [](SurfaceVertex& o, int k, F v) {
        o.normal[size_t(2 - k)] = uint8_t(int(std::floor(D(v) * 127.0 + 128.0)) & 0xff);
    });
    return out;
}

std::vector<std::array<float, 3>> patchCollisionGrid(const SurfaceBody& s, const SurfacePatch& p, int nu, int nv) {  // 0x7cf0a0
    if (nu < 1 || nv < 1) return {};
    const std::array<float, 64> net = patchControlNet(s, p);
    auto at = [&](int r, int c, int k) { return D(net[size_t((r * 4 + c) * 4 + k)]); };
    const F hu = F(1.0 / D(nu)), hu2 = F(D(hu) * hu), hu3 = F(D(hu2) * hu);
    const F hv = F(1.0 / D(nv)), hv2 = F(D(hv) * hv), hv3 = F(D(hv2) * hv);
    Cubic col[4][3];
    for (int c = 0; c < 4; ++c)
        for (int k = 0; k < 3; ++k) {
            const D p0 = at(0, c, k), p1 = at(1, c, k), p2 = at(2, c, k), p3 = at(3, c, k);
            const D A = p3 + ((p1 * 3.0 + -p0) - p2 * 3.0);
            const D B = (p0 * 3.0 - p1 * 6.0) + p2 * 3.0;
            const D C = p1 * 3.0 + -p0 * 3.0;
            Cubic& q = col[c][k];
            q.v = F(p0);
            q.d1 = F(B * hv2 + A * hv3 + C * hv);
            q.d2 = F(A * 6.0 * hv3 + B * 2.0 * hv2);
            q.d3 = F(A * 6.0 * hv3);
        }
    std::vector<std::array<float, 3>> out(size_t(nu + 1) * size_t(nv + 1));
    for (int r = 0; r <= nv; ++r) {
        for (int k = 0; k < 3; ++k) {
            const D q0 = col[0][k].v, q1 = col[1][k].v, q2 = col[2][k].v, q3 = col[3][k].v;
            const D A = q3 + ((-q0 + q1 * 3.0) - q2 * 3.0);
            const D B = q2 * 3.0 + (q0 * 3.0 - q1 * 6.0);
            const F a6 = F(A * 6.0);
            F v = F(q0), d1 = F(B * hu2 + A * hu3 + (q1 * 3.0 + -q0 * 3.0) * hu), d2 = F(D(a6) * hu3 + B * 2.0 * hu2);
            for (int j = 0; j <= nu; ++j) {
                out[size_t(r * (nu + 1) + j)][size_t(k)] = v;
                v = F(D(v) + d1);
                d1 = F(D(d2) + d1);
                d2 = F(D(a6) * hu3 + d2);
            }
        }
        for (auto& c4 : col)
            for (Cubic& q : c4) {
                q.v = F(D(q.v) + q.d1);
                q.d1 = F(D(q.d1) + q.d2);
                q.d2 = F(D(q.d3) + q.d2);
            }
    }
    return out;
}

std::vector<uint32_t> patchGridTriangles(int nu, int nv) {  // 0x811bc0
    std::vector<uint32_t> t;
    const uint32_t w = uint32_t(nu + 1);
    for (uint32_t r = 0; r < uint32_t(nv); ++r)
        for (uint32_t j = 0; j < uint32_t(nu); ++j) {
            const uint32_t a = (r + 1) * w + j, b = a + 1, c = r * w + j, d = c + 1;
            t.insert(t.end(), {a, b, c, b, d, c});
        }
    return t;
}

uint32_t patchMaterial(const std::vector<const DpcFile*>& lookup, const SurfacePatch& p) {  // vf05 0x80f6d0
    const Found f = find(lookup, p.materialAnim);
    if (f.entry == nullptr || f.entry->classCrc != cls::MaterialAnimZ) return 0;
    const auto body = f.file->body(*f.entry);
    if (!body) return 0;
    try {
        return parseMaterialAnimBody(*body).material;
    } catch (const std::exception&) {
        return 0;
    }
}

bool bakeSurface(const std::vector<const DpcFile*>& lookup, uint32_t surfaceId, BakedMesh& out, std::string* error) {
    auto fail = [&](const std::string& e) {
        if (error) *error = e;
        return false;
    };
    const Found f = find(lookup, surfaceId);
    if (f.entry == nullptr || f.entry->classCrc != cls::Surface) return fail("no such Surface_Z");
    const auto body = f.file->body(*f.entry);
    if (!body) return fail("Surface_Z has no body");
    out = {};
    try {
        const SurfaceBody s = parseSurfaceBody(*body);
        for (const SurfacePatch& p : s.patches) {
            const uint32_t mat = patchMaterial(lookup, p);
            uint32_t drawFlags = 0;
            if (const Found mf = find(lookup, mat); mf.entry != nullptr && mf.entry->classCrc == cls::Material)
                if (const auto mb = mf.file->body(*mf.entry)) drawFlags = parseMaterialBody(*mb).flags;
            if (mat == 0 || (drawFlags & 0x80)) continue;  // no material / not drawn (0x872df0: flags 0x80)
            const int n = patchDrawLevel(drawFlags);
            const std::vector<SurfaceVertex> v = tessellatePatch(s, p, n);
            const uint32_t base = out.vertexCount();
            for (const SurfaceVertex& sv : v) {
                uint8_t o[BakedMesh::kStride] = {};
                std::memcpy(o, sv.pos.data(), 12);
                o[12] = o[13] = o[14] = 128, o[15] = 255;  // no tangent in FUEL's patch vertex (0x846f80 leaves +0xc)
                o[16] = sv.normal[0], o[17] = sv.normal[1], o[18] = sv.normal[2], o[19] = 255;
                std::memcpy(o + 20, sv.uv0.data(), 8);
                std::memcpy(o + 28, sv.uv1.data(), 8);
                out.vertices.insert(out.vertices.end(), o, o + sizeof o);
            }
            uint32_t mi = 0;
            while (mi < out.materials.size() && out.materials[mi].materialId != mat) ++mi;
            if (mi == out.materials.size()) out.materials.push_back(BakedMaterial{mat, 0, 0});
            BakedBatch b;
            b.material = mi;
            b.firstIndex = uint32_t(out.indices.size());
            for (uint32_t i : patchGridTriangles(n, n)) out.indices.push_back(base + i);
            b.indexCount = uint32_t(out.indices.size()) - b.firstIndex;
            out.batches.push_back(b);
        }
    } catch (const std::exception& e) {
        return fail(e.what());
    }
    if (out.indices.empty()) return fail("Surface_Z draws nothing");
    return true;
}

}  // namespace fuel
