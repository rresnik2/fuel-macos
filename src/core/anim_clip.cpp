// FUEL's keyframe animation; see anim_clip.hpp.
#include "anim_clip.hpp"

#include "reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel::anim {

const Mat4 kIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

Quat slerp(Quat a, Quat b, double f) {
    if (f == 0) return a;
    if (f == 1) return b;
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (d < 0) {
        for (double& c : b) c = -c;
        d = -d;
    }
    Quat o;
    if (d >= 1.0) {  // 0x41d4b0: lerp, then normalize
        double n = 0;
        for (size_t i = 0; i < 4; ++i) o[i] = a[i] + (b[i] - a[i]) * f, n += o[i] * o[i];
        n = std::sqrt(n);
        if (n > 0)
            for (double& c : o) c /= n;
        return o;
    }
    const double th = std::acos(d), s = std::sin(th);
    const double wa = std::sin((1 - f) * th) / s, wb = std::sin(f * th) / s;
    for (size_t i = 0; i < 4; ++i) o[i] = wa * a[i] + wb * b[i];
    return o;
}

Quat qmul(const Quat& a, const Quat& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

Quat eulerZYX(const Vec3& e) {
    const Quat qx = {std::sin(e[0] / 2), 0, 0, std::cos(e[0] / 2)};
    const Quat qy = {0, std::sin(e[1] / 2), 0, std::cos(e[1] / 2)};
    const Quat qz = {0, 0, std::sin(e[2] / 2), std::cos(e[2] / 2)};
    return qmul(qmul(qz, qy), qx);
}

double hermite1(double p0, double m0, double p1, double m1, double f) {
    return (((2 * p0 - 2 * p1 + m0 + m1) * f + (3 * p1 - 3 * p0 - 2 * m0 - m1)) * f + m0) * f + p0;
}

Vec3 hermite(const Vec3& p0, const Vec3& m0, const Vec3& p1, const Vec3& m1, double f) {
    return {hermite1(p0[0], m0[0], p1[0], m1[0], f), hermite1(p0[1], m0[1], p1[1], m1[1], f), hermite1(p0[2], m0[2], p1[2], m1[2], f)};
}

Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            r[size_t(i * 4 + j)] = s;
        }
    return r;
}

Mat4 localMatrix(const Vec3& scale, const Quat& qin, const Vec3& t) {
    Quat q = qin;
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0)
        for (double& c : q) c /= n;
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    Mat4 m = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
              2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
              2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
              t[0], t[1], t[2], 1};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m[size_t(r * 4 + c)] *= scale[size_t(r)];
    return m;
}

Mat4 invertAffine(const Mat4& m) {
    double a[3][3], inv[3][3];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) a[r][c] = m[size_t(r * 4 + c)];
    const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                       a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (std::fabs(det) < 1e-12) return kIdentity;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            const int r1 = (c + 1) % 3, r2 = (c + 2) % 3, c1 = (r + 1) % 3, c2 = (r + 2) % 3;
            inv[r][c] = (a[r1][c1] * a[r2][c2] - a[r1][c2] * a[r2][c1]) / det;
        }
    Mat4 o = kIdentity;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) o[size_t(r * 4 + c)] = inv[r][c];
    for (int c = 0; c < 3; ++c) o[size_t(12 + c)] = -(m[12] * inv[0][c] + m[13] * inv[1][c] + m[14] * inv[2][c]);
    return o;
}

bool Clip::parse(const std::vector<uint8_t>& body) {
    try {
        ByteReader r(body);
        duration = r.f32();
        r.u32();
        flags = r.u16();
        r.u16();
        auto copy = [&](auto& v, size_t elem) {
            const uint32_t n = r.count();
            v.resize(n);
            if (n) std::memcpy(v.data(), r.raw(size_t(n) * elem), size_t(n) * elem);
        };
        static_assert(sizeof(QKey) == 12 && sizeof(EKey) == 40 && sizeof(VKey) == 24, "key layout");
        copy(a_, 12);
        copy(b_, 40);
        auto copyV = [&](std::vector<VKey>& v, uint16_t& fl) {  // 24-byte keys
            fl = r.u16();
            copy(v, 24);
        };
        copyV(c_, cFlags_);
        copyV(d_, dFlags_);
        e_.clear();
        msgs_.clear();
        for (uint32_t i = 0, n = r.count(); i < n; ++i) {  // E: events per key time
            EvKey k;
            k.t = r.f32();
            const uint32_t m = r.count();
            const uint8_t* mp = r.raw(size_t(m) * 20);
            std::vector<Msg> ms(m);
            for (uint32_t j = 0; j < m; ++j) {
                std::memcpy(&ms[j], mp + size_t(j) * 20, 20);  // PK29: {kind, a, b, c, d}
                k.kinds.push_back(ms[j].kind);
            }
            e_.push_back(std::move(k));
            msgs_.push_back(std::move(ms));
        }
        // PK29: the key arrays of the material / visibility tracks (Load 0x7c4980 file order: +0x78, +0x84, +0x9c,
        // +0x90, +0xa8, +0x6c, then +0xb4 = the morph keys, unused: no shipped Animation_Z has a morph track +0x30).
        static_assert(sizeof(SKey) == 12 && sizeof(Msg) == 20, "key layout");
        auto copyS = [&](SArr& s) {
            s.flags = r.u16();
            copy(s.keys, 12);
        };
        copyS(fKeys_);
        copyS(gKeys_);
        copyV(h_, hFlags_);
        copyV(i_, iFlags_);
        copyS(jKeys_);
        copyS(kKeys_);
        r.u16();
        r.skipArray(12);
        const uint32_t nt = r.count();
        const uint8_t* tp = r.raw(size_t(nt) * 0x1c);
        tracks.clear();
        for (uint32_t i = 0; i < nt; ++i) {
            const uint8_t* p = tp + size_t(i) * 0x1c;
            Track t;
            uint16_t u[10];
            std::memcpy(&t.bone, p, 4);
            std::memcpy(u, p + 8, 20);
            t.d0 = u[0], t.dn = u[1], t.a0 = u[2], t.an = u[3], t.b0 = u[4], t.bn = u[5], t.c0 = u[6], t.cn = u[7];
            t.e0 = u[8], t.en = u[9];
            if (size_t(t.e0) + t.en > e_.size()) t.en = 0;
            if (size_t(t.d0) + t.dn > d_.size() || size_t(t.a0) + t.an > a_.size() || size_t(t.b0) + t.bn > b_.size() ||
                size_t(t.c0) + t.cn > c_.size())
                return false;
            tracks.push_back(t);
        }
        matTracks.clear();  // PK29: N, the material tracks (anim +0x20 / +0x24)
        for (uint32_t i = 0, n = r.count(); i < n; ++i) {
            const uint8_t* p = r.raw(0x1c);
            MatTrack m;
            uint16_t u[10];
            std::memcpy(&m.name, p, 4);
            std::memcpy(u, p + 8, 20);
            m.u0 = u[0], m.un = u[1], m.e0 = u[2], m.en = u[3], m.d0 = u[4], m.dn = u[5], m.r0 = u[6], m.rn = u[7], m.v0 = u[8], m.vn = u[9];
            if (size_t(m.u0) + m.un > fKeys_.keys.size()) m.un = 0;
            if (size_t(m.v0) + m.vn > gKeys_.keys.size()) m.vn = 0;
            if (size_t(m.d0) + m.dn > h_.size()) m.dn = 0;
            if (size_t(m.e0) + m.en > i_.size()) m.en = 0;
            if (size_t(m.r0) + m.rn > jKeys_.keys.size()) m.rn = 0;
            matTracks.push_back(m);
        }
        visTracks.clear();  // PK29: O, the visibility tracks (anim +0x28 / +0x2c)
        for (uint32_t i = 0, n = r.count(); i < n; ++i) {
            const uint8_t* p = r.raw(12);
            VisTrack v;
            std::memcpy(&v.name, p, 4);
            std::memcpy(&v.k0, p + 8, 2);
            std::memcpy(&v.kn, p + 10, 2);
            if (size_t(v.k0) + v.kn > kKeys_.keys.size()) v.kn = 0;
            visTracks.push_back(v);
        }
        r.skipArray(12);  // P: morph tracks (none shipped)
        r.expectEnd();
        return duration > 0;
    } catch (const std::exception&) {
        return false;
    }
}

void Clip::events(int track, double from, double to, bool closed, std::vector<std::pair<float, uint32_t>>& out) const {
    if (track < 0 || size_t(track) >= tracks.size()) return;
    const Track& tr = tracks[size_t(track)];
    for (size_t i = tr.e0; i < size_t(tr.e0) + tr.en; ++i) {
        const double k = e_[i].t;
        if (k < from || (closed ? k > to : k >= to)) continue;
        for (uint32_t kind : e_[i].kinds) out.push_back({e_[i].t, kind});
    }
}

int Clip::track(uint32_t boneCrc) const {
    for (size_t i = 0; i < tracks.size(); ++i)
        if (tracks[i].bone == boneCrc) return int(i);
    return -1;
}

namespace {

Vec3 vkey(const int16_t* v) { return {v[0] / 4096.0, v[1] / 4096.0, v[2] / 4096.0}; }

// 0x7d4ed0: s16/4096 vector keys, Hermite (step with header flag 8).
template <class K>
Vec3 sampleVec(const std::vector<K>& keys, size_t first, size_t n, uint16_t hdrFlags, double t) {
    auto times = [&](size_t i) { return double(keys[first + i].t); };
    const int i = segment(times, n, t);
    if (i <= 0) return vkey(keys[first + (i == 0 ? 0 : n - 1)].v);
    const K& k0 = keys[first + size_t(i) - 1];
    const K& k1 = keys[first + size_t(i)];
    if (hdrFlags & 8) return vkey(k0.v);
    const double f = (t - k0.t) / (double(k1.t) - k0.t);
    return hermite(vkey(k0.v), vkey(k0.out), vkey(k1.v), vkey(k1.in), f);
}

}  // namespace

void Clip::sample(int ti, double t, Pose& p) const {
    if (ti < 0 || size_t(ti) >= tracks.size()) return;
    const Track& tr = tracks[size_t(ti)];
    if (tr.dn >= 1) p.t = sampleVec(d_, tr.d0, tr.dn, dFlags_, t);
    if (tr.cn >= 1) p.s = sampleVec(c_, tr.c0, tr.cn, cFlags_, t);
    if (tr.an >= 1) {  // 0x7d62d0
        auto q = [&](size_t i) {
            const QKey& k = a_[tr.a0 + i];
            return Quat{k.v[0] * 0.0005, k.v[1] * 0.0005, k.v[2] * 0.0005, k.v[3] * 0.0005};
        };
        auto times = [&](size_t i) { return double(a_[tr.a0 + i].t); };
        const int i = segment(times, tr.an, t);
        if (i <= 0) p.q = q(i == 0 ? 0 : tr.an - 1u);
        else p.q = slerp(q(size_t(i) - 1), q(size_t(i)), (t - times(size_t(i) - 1)) / (times(size_t(i)) - times(size_t(i) - 1)));
    } else if (tr.bn >= 1) {  // 0x7d6660
        auto times = [&](size_t i) { return double(b_[tr.b0 + i].t); };
        const int i = segment(times, tr.bn, t);
        auto e = [&](const float* v) { return Vec3{v[0], v[1], v[2]}; };
        if (i <= 0) {
            p.q = eulerZYX(e(b_[tr.b0 + (i == 0 ? 0 : tr.bn - 1u)].e));
        } else {
            const EKey& k0 = b_[tr.b0 + size_t(i) - 1];
            const EKey& k1 = b_[tr.b0 + size_t(i)];
            const double f = (t - k0.t) / (double(k1.t) - k0.t);
            p.q = eulerZYX(hermite(e(k0.e), e(k0.out), e(k1.e), e(k1.in), f));
        }
    }
}


// ---- PK29: event windows, visibility and material tracks (PlayAnim_Z 0x7bc5c0) ------------------------------------

void Clip::eventWindow(int track, double from, double time, double end, std::vector<std::pair<float, Msg>>& out) const {
    if (track < 0 || size_t(track) >= tracks.size()) return;
    const Track& tr = tracks[size_t(track)];
    if (time < from) {  // 0x73b590: the wrap, [from, end] first, then from 0
        eventWindow(track, from, end, end, out);
        from = 0;
    }
    const bool closed = !(time < end);  // time >= end: up to the last key
    for (size_t i = tr.e0; i < size_t(tr.e0) + tr.en; ++i) {
        const double k = e_[i].t;
        if (k < from || (!closed && !(k < time))) continue;
        for (const Msg& m : msgs_[i]) out.push_back({e_[i].t, m});
    }
}

double Clip::sampleScalar(const SArr& a, size_t first, size_t n, double t) {  // 0x72f620
    if (n == 0) return 0;
    auto key = [&](size_t i) -> const SKey& { return a.keys[first + i]; };
    size_t c = 1;  // the cursor: segment [k(c-1), k(c)); it only moves forward, past the end -> the last value
    while (c < n && (t < key(c - 1).t || !(t < key(c).t))) ++c;
    if (c >= n) return key(n - 1).v / 4096.0;
    const SKey &k0 = key(c - 1), &k1 = key(c);
    if (a.flags & 8) return k0.v / 4096.0;  // hold
    const double f = (t - k0.t) / (double(k1.t) - k0.t);
    return hermite1(k0.v / 4096.0, k0.out / 4096.0, k1.v / 4096.0, k1.in / 4096.0, f);
}

double Clip::visValue(size_t i, double t) const {
    if (i >= visTracks.size()) return 1;
    return sampleScalar(kKeys_, visTracks[i].k0, visTracks[i].kn, t);
}

double Clip::visLastValue(size_t i) const {
    if (i >= visTracks.size() || visTracks[i].kn == 0) return -1;
    return kKeys_.keys[size_t(visTracks[i].k0) + visTracks[i].kn - 1].v / 4096.0;
}

Clip::MatSample Clip::sampleMaterial(size_t i, double t) const {
    MatSample s;
    if (i >= matTracks.size()) return s;
    const MatTrack& m = matTracks[i];
    if (m.un) s.hasU = true, s.u = sampleScalar(fKeys_, m.u0, m.un, t);
    if (m.vn) s.hasV = true, s.v = sampleScalar(gKeys_, m.v0, m.vn, t);
    if (m.dn) s.hasDiffuse = true, s.diffuse = sampleVec(h_, m.d0, m.dn, hFlags_, t);   // 0x7d4ed0
    if (m.en) s.hasEmissive = true, s.emissive = sampleVec(i_, m.e0, m.en, iFlags_, t);
    if (m.rn) s.hasAngle = true, s.angle = sampleScalar(jKeys_, m.r0, m.rn, t);
    return s;
}

}  // namespace fuel::anim
