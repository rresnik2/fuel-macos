// FUEL's real-time cut-scenes; see rtc.hpp.
#include "rtc.hpp"

#include "reader.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel::rtc {
namespace {

template <class T>
void readKeys(ByteReader& r, std::vector<T>& v) {
    const uint32_t n = r.count();
    v.resize(n);
    if (n) std::memcpy(v.data(), r.raw(size_t(n) * sizeof(T)), size_t(n) * sizeof(T));
}

std::vector<EventKey> readEvents(ByteReader& r) {
    std::vector<EventKey> out(r.count());
    for (EventKey& k : out) {
        k.time = r.f32();
        k.msgs.resize(r.count());
        for (Msg& m : k.msgs) m.type = r.u32(), m.a = r.u32(), m.b = r.u32(), m.c = r.u32(), m.d = r.u32();
    }
    return out;
}

// The pair of keys around t (anim::segment), for keys with a time member.
template <class K, class TimeOf>
int seg(const std::vector<K>& k, TimeOf tof, double t) {
    return anim::segment([&](size_t i) { return tof(k[i]); }, k.size(), t);
}

}  // namespace

bool parse(const std::vector<uint8_t>& body, Rtc& o, std::string* error) {
    static_assert(sizeof(VKey) == 40 && sizeof(SKey) == 12 && sizeof(FKey) == 16 && sizeof(RKey) == 12, "key layout");
    try {
        ByteReader r(body);
        o = Rtc{};
        o.duration = r.f32();
        o.nodes.resize(r.count());
        for (Node& n : o.nodes) {
            n.id = r.u32();
            n.e = r.u16();
            readKeys(r, n.rot);
            n.scaleFlags = r.u16();
            readKeys(r, n.scale);
            n.transFlags = r.u16();
            readKeys(r, n.trans);
            n.events = readEvents(r);
        }
        o.cameras.resize(r.count());
        for (Camera& c : o.cameras) {
            c.id = r.u32();
            c.e = r.u16();
            c.rollFlags = r.u16();
            readKeys(r, c.roll);
            c.fovFlags = r.u16();
            readKeys(r, c.fov);
            c.farFlags = r.u16();
            readKeys(r, c.farT);
            c.nearFlags = r.u16();
            readKeys(r, c.nearT);
        }
        o.targets.resize(r.count());
        for (uint32_t& t : o.targets) t = r.u32();
        o.lights = r.count();
        for (size_t i = 0; i < o.lights; ++i) {  // 0x7d3c90 (none in the shipped files)
            r.u32();
            r.u16();
            r.u16();
            r.skipArray(24);
            r.u16();
            r.skipArray(12);
            r.u16();
            r.skipArray(12);
        }
        o.binds.resize(r.count());
        for (Bind& b : o.binds) {
            b.parentId = r.u32(), b.nodeId = r.u32(), b.unk = r.u32(), b.parentE = r.i32();
            b.flag = *r.raw(1);
            b.nodeRes = r.u32(), b.res2 = r.u32();
        }
        o.objects.resize(r.count());
        for (Object& f : o.objects) f.e = r.u32(), f.nodeId = r.u32(), f.unk = r.u32(), f.res = r.u32(), f.anim = r.u32(), f.userDefine = r.u32();
        r.skipArray(4);  // G
        r.skipArray(4);  // H
        o.events = readEvents(r);
        r.expectEnd();
        for (const Node& n : o.nodes)
            if (n.e >= o.binds.size()) throw std::runtime_error("node bind index out of range");
        for (const Bind& b : o.binds)
            if (b.parentE >= int32_t(o.binds.size())) throw std::runtime_error("bind parent out of range");
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

anim::Vec3 sampleV(const std::vector<VKey>& k, uint16_t, double t) {
    if (k.empty()) return {0, 0, 0};
    auto v3 = [](const float* f) { return anim::Vec3{f[0], f[1], f[2]}; };
    const int i = seg(k, [](const VKey& x) { return x.frame / 60.0; }, t);
    if (i <= 0) return v3(k[i == 0 ? 0 : k.size() - 1].v);
    const VKey& k0 = k[size_t(i) - 1];
    const VKey& k1 = k[size_t(i)];
    if (int8_t(k0.flags & 0xff) < 0) return v3(k0.v);  // step key
    const double t0 = k0.frame / 60.0, t1 = k1.frame / 60.0;
    return anim::hermite(v3(k0.v), v3(k0.out), v3(k1.v), v3(k1.in), (t - t0) / (t1 - t0));
}

double sampleS(const std::vector<SKey>& k, uint16_t fl, double t) {
    if (k.empty()) return 0;
    const int i = seg(k, [](const SKey& x) { return double(x.t); }, t);
    if (i <= 0) return k[i == 0 ? 0 : k.size() - 1].v / 4096.0;
    const SKey& k0 = k[size_t(i) - 1];
    const SKey& k1 = k[size_t(i)];
    if (fl & 8) return k0.v / 4096.0;
    return anim::hermite1(k0.v / 4096.0, k0.out / 4096.0, k1.v / 4096.0, k1.in / 4096.0, (t - k0.t) / (double(k1.t) - k0.t));
}

double sampleF(const std::vector<FKey>& k, uint16_t fl, double t) {
    if (k.empty()) return 0;
    const int i = seg(k, [](const FKey& x) { return double(x.t); }, t);
    if (i <= 0) return k[i == 0 ? 0 : k.size() - 1].v;
    const FKey& k0 = k[size_t(i) - 1];
    const FKey& k1 = k[size_t(i)];
    if (fl & 8) return k0.v;
    return anim::hermite1(k0.v, k0.out, k1.v, k1.in, (t - k0.t) / (double(k1.t) - k0.t));
}

anim::Quat sampleR(const std::vector<RKey>& k, double t) {
    if (k.empty()) return {0, 0, 0, 1};
    auto q = [](const RKey& x) { return anim::Quat{x.q[0] * 0.0005, x.q[1] * 0.0005, x.q[2] * 0.0005, x.q[3] * 0.0005}; };
    const int i = seg(k, [](const RKey& x) { return double(x.t); }, t);
    if (i <= 0) return q(k[i == 0 ? 0 : k.size() - 1]);
    const RKey& k0 = k[size_t(i) - 1];
    const RKey& k1 = k[size_t(i)];
    return anim::slerp(q(k0), q(k1), (t - k0.t) / (double(k1.t) - k0.t));
}

void Player::start() {
    time_ = loopStart_ = 0;
    flags_ = count_ = 0;
    finished_ = false;
}

// 0x73b2b0: keys with from <= k < to, or k <= to when the window closes at the end.
void Player::collect(double from, double to, bool closed, std::vector<Fired>* fired, bool) {
    if (fired == nullptr) return;
    auto in = [&](double k) { return k >= from && (closed ? k <= to : k < to); };
    for (const EventKey& k : rtc_.events)
        if (in(k.time))
            for (const Msg& m : k.msgs) fired->push_back({k.time, -1, m});
    for (size_t n = 0; n < rtc_.nodes.size(); ++n)
        for (const EventKey& k : rtc_.nodes[n].events)
            if (in(k.time))
                for (const Msg& m : k.msgs) fired->push_back({k.time, int(n), m});
}

bool Player::nextLoopEnd(float& at) const {
    for (const EventKey& k : rtc_.events) {  // keys are in time order (0x73b190 takes the first k >= t)
        if (!(k.time >= time_)) continue;
        for (const Msg& m : k.msgs)
            if (m.type == 0xd && m.a == 0) {
                at = k.time;
                return true;
            }
    }
    return false;
}

bool PauseHold::update(float dt, Player& p, std::vector<Fired>* fired, bool streamingBusy) {
    if (!holding) return false;
    timer += dt;
    if (streamingBusy) timer = 0;
    if (!(timer > 0.5f)) return false;
    holding = false;
    float at = 0;
    if (p.nextLoopEnd(at)) p.seek(at + 0.01f, fired);
    return true;
}

void Player::seek(float t, std::vector<Fired>* fired) {
    if (time_ < t && fired != nullptr) {
        const size_t first = fired->size();
        collect(time_, t, false, fired, false);
        for (size_t i = first; i < fired->size(); ++i) (*fired)[i].skipping = true;
    }
    flags_ &= ~(2 | 8);
    time_ = t;
}

void Player::advance(float dt, std::vector<Fired>& fired) {
    if (finished_ || dt <= 0) return;
    double prev = time_, t = time_ + dt;
    const double dur = rtc_.duration;
    // Loop markers (unless flag 8): 0xc = loop start, 0xd = jump back (count (s16)a: n repeats, 0 = forever).
    for (int guard = 0; guard < 64 && !(flags_ & 8); ++guard) {
        bool jumped = false;
        const double end = std::min(t, dur);
        const bool closed = t >= dur;  // 0x73b2b0: the window closes at the end
        for (const EventKey& k : rtc_.events) {
            if (k.time < prev || (closed ? k.time > end : k.time >= end)) continue;
            for (const Msg& m : k.msgs) {
                if (m.type == 0xc && !(flags_ & 2)) loopStart_ = k.time, flags_ |= 2;
                if (m.type != 0xd) continue;
                bool jump = true;
                if (!(flags_ & 4)) {
                    count_ = int16_t(m.a), flags_ |= 4;
                } else {
                    const int c = count_;
                    count_ = int16_t(count_ - 1);
                    if (c != 0 && count_ == 0) flags_ &= ~6, jump = false;
                }
                if (jump) {
                    collect(prev, k.time, false, &fired, false);  // Evaluate at the marker
                    const double over = t - k.time;
                    prev = loopStart_;
                    t = loopStart_ + over;
                    jumped = true;
                    break;
                }
            }
            if (jumped) break;
        }
        if (!jumped) break;
    }
    collect(prev, std::min(t, dur), t >= dur, &fired, false);
    if (t >= dur) t = dur, finished_ = true;  // FUEL: clamp, Evaluate, agent end handler (vt+0xa4)
    time_ = t;
}

void Player::evaluate(const Mat4& father, std::vector<Mat4>& world) const {
    const size_t n = rtc_.binds.size();
    std::vector<int> nodeOf(n, -1);
    for (size_t i = 0; i < rtc_.nodes.size(); ++i) nodeOf[rtc_.nodes[i].e] = int(i);
    world.assign(n, anim::kIdentity);
    std::vector<char> done(n, 0);
    const double t = time_;
    auto local = [&](size_t e) {
        if (nodeOf[e] < 0) return anim::kIdentity;
        const Node& nd = rtc_.nodes[size_t(nodeOf[e])];
        const anim::Quat q = sampleR(nd.rot, t);
        const double s = nd.scale.empty() ? 1.0 : sampleV(nd.scale, nd.scaleFlags, t)[0];  // only .x (uniform)
        const anim::Vec3 tr = sampleV(nd.trans, nd.transFlags, t);
        return anim::localMatrix({s, s, s}, q, tr);
    };
    // Parents first (binds are stored parent before child in the shipped files; resolve iteratively anyway).
    for (int pass = 0; pass < 64; ++pass) {
        bool all = true;
        for (size_t e = 0; e < n; ++e) {
            if (done[e]) continue;
            const int p = rtc_.binds[e].parentE;
            if (p >= 0 && !done[size_t(p)]) {
                all = false;
                continue;
            }
            world[e] = anim::mul(local(e), p >= 0 ? world[size_t(p)] : father);
            done[e] = 1;
        }
        if (all) break;
    }
}

int Player::cameraById(uint32_t id) const {
    for (size_t i = 0; i < rtc_.cameras.size(); ++i)
        if (rtc_.cameras[i].id == id) return int(i);
    return -1;
}

Player::CamPose Player::camera(size_t ci, const std::vector<Mat4>& world) const {
    CamPose p;
    const Camera& c = rtc_.cameras[ci];
    const Mat4& m = world[c.e];
    p.eye = {m[12], m[13], m[14]};
    p.up = {m[4], m[5], m[6]};
    p.target = {m[12] + m[8], m[13] + m[9], m[14] + m[10]};  // no look-at node: along the node's +z
    if (ci < rtc_.targets.size())
        for (size_t e = 0; e < rtc_.binds.size(); ++e)
            if (rtc_.binds[e].nodeId == rtc_.targets[ci]) {
                p.target = {world[e][12], world[e][13], world[e][14]};  // Camera_Z +0x844
                break;
            }
    p.fov = sampleS(c.fov, c.fovFlags, time_);
    p.roll = sampleS(c.roll, c.rollFlags, time_);
    p.nearZ = 1.0;  // scripted RTCs force near 1, far 50000 every frame (0x53c150)
    return p;
}

// ---- P96: global bone ids, 0x103 'AMEO' ----------------------------------------------------------------

BoneRegistry::BoneRegistry() {
    static const char* const kFixed[] = {"TETE",   "POIDRO", "POIGAU",   "COUDRO",  "COUGAU",   "EPADRO",    "EPAGAU",   "HANDRO", "HANGAU",
                                         "GENDRO", "GENGAU", "PIEDRO",   "PIEGAU",  "BASDOS",   "SELLE",     "SOL",      "TREE",   "LINK",
                                         "MOVE",   "CAMERA", "COU",      "MILDOS",  "HAUDOS",   "DDPGAU",    "DDPDRO",   "HIPS",   "BACK",
                                         "LEFTHAND", "LEFTLEG", "LEFTFOOT", "RIGHTHAND", "RIGHTLEG", "RIGHTFOOT", "TRONCHE1"};
    for (const char* n : kFixed) id(asoboCrc(n));
    for (int i = 1; i <= 16; ++i) id(asoboCrc("DYN" + std::to_string(i)));  // sprintf "DYN%d" 0x747d6b
    for (const char* n : {"SKIN", "SHOES", "CLOTHE", "HAIRS", "MOUTH", "FACE"}) id(asoboCrc(n));
}

int BoneRegistry::find(uint32_t c) const {
    const auto it = std::find(names_.begin(), names_.end(), c);
    return it == names_.end() ? -1 : int(it - names_.begin());
}

int BoneRegistry::id(uint32_t c) {
    const int f = find(c);
    if (f >= 0) return f;
    names_.push_back(c);
    return int(names_.size()) - 1;
}

// P100 [DEVIATION from FUEL, user decision 2026-11-05; notes/p100-limiter-cables.md]: AMEO target a pairs with the bone
// named LINKa (case-insensitive: LINK1, Link2, Link3, Link4), i.e. four straight cables. FUEL's own mapping (P96: the
// instance's registration-order ids, a = 1..4 -> LINK1, Link3, Link2, Link4) hangs the two middle targets from each
// other's hooks, so the two middle cables cross under the belly; the port does not show that. The registry / palettes
// arguments stay for the callers (and `ameoJoint(..., fuelOrder = true)` keeps the decoded FUEL pairing).
int ameoJoint(BoneRegistry reg, const std::vector<std::vector<uint32_t>>& palettes, const std::vector<uint32_t>& bones, int a, bool fuelOrder) {
    if (!fuelOrder) {
        if (a < 1 || a > 9) return -1;
        const uint32_t c = asoboCrc("LINK" + std::to_string(a));
        for (size_t b = 0; b < bones.size(); ++b)
            if (bones[b] == c) return int(b);
        return -1;
    }
    for (const auto& l : palettes)  // Skel_Z::Load
        for (uint32_t c : l) reg.id(c);
    std::vector<int> run;  // bones the instance registers (0x66c0e0; pre-order 0x675e60 = the stored bone order), by id
    for (size_t b = 0; b < bones.size(); ++b)
        if (reg.find(bones[b]) < 0) reg.id(bones[b]), run.push_back(int(b));
    const int k = a + 0x49 - kAmeoBase;
    return k >= 0 && size_t(k) < run.size() ? run[size_t(k)] : -1;
}

// ---- PK13 ---------------------------------------------------------------------------------------------------------

void AttachSlot::step(float dt, const anim::Vec3& nodeP, const anim::Quat& nodeQ, const anim::Vec3& curP, const anim::Quat& curQ, anim::Vec3& outP,
                      anim::Quat& outQ) {
    done = false;
    if (duration > elapsed) {  // fcomp [+0x20] vs [+0x24]: C0 = C3 = 0
        if (elapsed == 0.0f) start = curP, startQ = curQ;  // latched from the attached node (+0x48) world
        const float f = 1.0f - elapsed / duration;
        for (size_t i = 0; i < 3; ++i) outP[i] = double(float(nodeP[i]) + (float(start[i]) - float(nodeP[i])) * f);
        outQ = anim::slerp(nodeQ, startQ, f);
    } else {
        outP = nodeP, outQ = nodeQ;
        done = true;
    }
    elapsed += dt;
}

anim::Quat quatOfRows(const Mat4& w) {
    double m[3][3];
    for (int r = 0; r < 3; ++r) {
        const double n = std::sqrt(w[size_t(r * 4)] * w[size_t(r * 4)] + w[size_t(r * 4 + 1)] * w[size_t(r * 4 + 1)] + w[size_t(r * 4 + 2)] * w[size_t(r * 4 + 2)]);
        for (int c = 0; c < 3; ++c) m[r][c] = n > 0 ? w[size_t(r * 4 + c)] / n : 0;
    }
    // localMatrix's layout (row-vector): m[0][1] = 2(xy + zw), m[1][0] = 2(xy - zw), ...
    anim::Quat q;
    const double tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2;
        q = {(m[1][2] - m[2][1]) / s, (m[2][0] - m[0][2]) / s, (m[0][1] - m[1][0]) / s, 0.25 * s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q = {0.25 * s, (m[0][1] + m[1][0]) / s, (m[2][0] + m[0][2]) / s, (m[1][2] - m[2][1]) / s};
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q = {(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[2][0] - m[0][2]) / s};
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q = {(m[2][0] + m[0][2]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[0][1] - m[1][0]) / s};
    }
    return q;
}

Mat4 overlayFather(const anim::Vec3& eye, const anim::Vec3& r, const anim::Vec3& u, const anim::Vec3& f, double fovH) {
    const double k = fovH > 0 ? std::tan(fovH / 2) / std::tan(kOverlayFov / 2) : 1.0;
    Mat4 m{};
    for (size_t i = 0; i < 3; ++i) {
        m[i] = r[i] * k, m[4 + i] = u[i] * k, m[8 + i] = f[i];
        m[12 + i] = eye[i] + f[i] * kOverlayDepth;
    }
    m[15] = 1;
    return m;
}

}  // namespace fuel::rtc
