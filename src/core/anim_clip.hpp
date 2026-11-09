// FUEL's keyframe animation (P47, notes/p47-input-anim.md §2; helper decodes notes/p47-work/anim-objects.md §3-4
// and rtc.md §2.3): the Animation_Z body (Load 0x7c4980, p9 S7, bone-track record corrected: +8 D translation,
// +0xc A quaternion, +0x10 B Euler, +0x14 C scale, +0x18 E events) and the samplers shared with the RTC player:
//  - quaternion keys (s16 x 0.0005) with slerp 0x431010 (q1 negated when dot < 0; dot >= 1 -> lerp + normalize);
//  - Euler keys (0x7d6660): Hermite per angle, then q = qz * qy * qx;
//  - s16/4096 vector keys (0x7d4ed0) and the RTC's float vector / s16 scalar keys (0x7d5640, 0x72f750): cubic Hermite
//    on f = (t - t0) / (t1 - t0) with m0 = key0.outTan, m1 = key1.inTan, tangents NOT scaled by the interval;
//    a track flag 8 (or a VTRACK key flag byte < 0) holds key0.
// Interval search (all samplers): keys k-1, k with t0 <= t < t1; at or past the last key -> the last value.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace fuel::anim {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // x y z w
using Mat4 = std::array<double, 16>; // row-vector (D3DX) convention, translation in 12..14

Quat slerp(Quat a, Quat b, double f);             // 0x431010
Quat eulerZYX(const Vec3& e);                     // 0x645730 products: qz * qy * qx
Quat qmul(const Quat& a, const Quat& b);          // Hamilton product a * b
Vec3 hermite(const Vec3& p0, const Vec3& m0, const Vec3& p1, const Vec3& m1, double f);
double hermite1(double p0, double m0, double p1, double m1, double f);
Mat4 mul(const Mat4& a, const Mat4& b);           // applies a first, then b
Mat4 localMatrix(const Vec3& scale, const Quat& q, const Vec3& t);  // S * R(q) * T (p9 S9)
Mat4 invertAffine(const Mat4& m);
extern const Mat4 kIdentity;

// Locates the key pair for time t in times[0..n): returns i (1..n-1) with times[i-1] <= t < times[i], or -1 when t is
// at/after the last key (use the last), or 0 when t is before the first (use the first; FUEL's forward cursor
// never meets this case: clips start at 0).
template <class Times>
int segment(const Times& times, size_t n, double t) {
    if (n == 0) return -1;
    if (t < times(0)) return 0;
    for (size_t i = 1; i < n; ++i)
        if (t < times(i)) return int(i);
    return -1;
}

struct Pose {  // one bone or node: rotation, scale, translation (local)
    Quat q{0, 0, 0, 1};
    Vec3 s{1, 1, 1}, t{0, 0, 0};
};

// An Animation_Z resource.
class Clip {
public:
    bool parse(const std::vector<uint8_t>& body);  // false on malformed data
    float duration = 0;
    uint16_t flags = 0;  // +0x14: bit 0 loop, bit 1 no blend-in
    bool loops() const { return (flags & 1) != 0; }
    struct Track {
        uint32_t bone = 0;  // bone name CRC
        uint16_t d0 = 0, dn = 0, a0 = 0, an = 0, b0 = 0, bn = 0, c0 = 0, cn = 0;
        uint16_t e0 = 0, en = 0;  // +0x18 / +0x1a: the track's event keys (E)
    };
    std::vector<Track> tracks;
    int track(uint32_t boneCrc) const;  // -1 when the clip has no track for the bone
    // Overwrites the channels the track animates (count >= 1; count 1 = that key) in `p` at time t.
    void sample(int track, double t, Pose& p) const;
    // P76: the track's event messages (20 B, u32 +0 = kind) with key time in [from, to), or [from, to] when `closed`
    // (0x7a7bc0 window as the RTC's 0x73b2b0), in key order: appends (time, kind).
    void events(int track, double from, double to, bool closed, std::vector<std::pair<float, uint32_t>>& out) const;

    // PK29: the whole 20-byte event message {kind, a, b, c, d} (0x7bc5c0 reads a..d: 0x24 sound a / flags b / volume c,
    // 0x20 FX data d, 0xd loop count a).
    struct Msg {
        uint32_t kind = 0, a = 0, b = 0, c = 0, d = 0;
    };
    // The 0x73b590 window of 0x7bc5c0 / 0x74a0b0 over the track's events, `from` .. `time` with the clip end `end`:
    // time < from wraps ([from, end] then [0, time)); else [from, time), or [from, end] once time >= end. In key order.
    void eventWindow(int track, double from, double time, double end, std::vector<std::pair<float, Msg>>& out) const;

    // PK29: PlayAnim_Z's visibility tracks (Animation_Z +0x28 / +0x2c, 0xc B {u32 name, s16 id, s16, s16 first key,
    // s16 count} into the scalar keys +0x6c) and material tracks (+0x20 / +0x24, 0x1c B {u32 name, s16 id, s16, then
    // (first, count) pairs: +8 -> keys +0x78, +0xc -> +0x90, +0x10 -> +0x9c, +0x14 -> +0xa8, +0x18 -> +0x84}).
    struct VisTrack {
        uint32_t name = 0;  // a Skin_Z mesh (group) name CRC, matched by 0x66ce90
        uint16_t k0 = 0, kn = 0;
    };
    std::vector<VisTrack> visTracks;
    // 0x72f620 on the track's keys: (value at t) / 4096; 0x7a6ce0 hides the group when it is 0.
    double visValue(size_t i, double t) const;
    // 0x7a6de0 (play-start catch-up 0x64f3f0): the value of the track's last key; -1 when it has none.
    double visLastValue(size_t i) const;
    struct MatTrack {
        uint32_t name = 0;  // a Material_Z name CRC (the instance's material list, cloned per instance by 0x66c0e0)
        uint16_t u0 = 0, un = 0, v0 = 0, vn = 0;  // +8 -> obj +0x7c (uvOffset.x), +0x18 -> +0x80 (uvOffset.y)
        uint16_t d0 = 0, dn = 0, e0 = 0, en = 0;  // +0x10 -> obj +0xc (diffuse), +0xc -> +0x1c (emissive)
        uint16_t r0 = 0, rn = 0;                  // +0x14 -> obj +0x58 (uvAngle)
    };
    std::vector<MatTrack> matTracks;
    struct MatSample {
        bool hasU = false, hasV = false, hasDiffuse = false, hasEmissive = false, hasAngle = false;
        double u = 0, v = 0, angle = 0;
        Vec3 diffuse{}, emissive{};
    };
    // 0x813e50: the sub-tracks present, sampled at t (then vt+0x44 = Material_Z::UpdateUVMatrix 0x695ee0).
    MatSample sampleMaterial(size_t i, double t) const;

private:
    struct SKey { float t; int16_t v, in, out; };  // 0x72f620's 12-byte scalar keys (m0 = k0.out, m1 = k1.in)
    struct SArr {
        uint16_t flags = 0;
        std::vector<SKey> keys;
    };
    static double sampleScalar(const SArr& a, size_t first, size_t n, double t);
    SArr fKeys_, gKeys_, jKeys_, kKeys_;  // anim +0x78, +0x84, +0xa8, +0x6c
    std::vector<std::vector<Msg>> msgs_;  // per E key, parallel to e_
    uint16_t hFlags_ = 0, iFlags_ = 0;
    struct QKey { float t; int16_t v[4]; };
    struct EKey { float t; float e[3], in[3], out[3]; };
    struct VKey { float t; int16_t v[3], in[3], out[3]; uint16_t pad; };
    std::vector<QKey> a_;
    std::vector<EKey> b_;
    std::vector<VKey> c_, d_;
    std::vector<VKey> h_, i_;  // PK29: anim +0x9c (diffuse), +0x90 (emissive) material keys
    struct EvKey {
        float t;
        std::vector<uint32_t> kinds;  // u32 +0 of each 20-byte message
    };
    std::vector<EvKey> e_;
    uint16_t cFlags_ = 0, dFlags_ = 0;
    std::vector<int> byBone_;
};

}  // namespace fuel::anim
