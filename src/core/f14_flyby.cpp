// PK27: the F14 fly-by (f14_flyby.hpp). The arithmetic follows FUEL's operation order (x87, 53-bit precision): the
// sums are done in double, the values FUEL stores to memory are rounded to float where it stores them.
#include "f14_flyby.hpp"

#include "race_helicopter.hpp"

#include <cmath>

namespace fuel {

F14Flyby::Q4 F14Flyby::fromArc(const V3& from, const V3& to) { return RaceHelicopter::fromArc(from, to); }

void F14Flyby::init(float pp80v, float pp81v, float pp82v) {  // F14Move_G vf00 0x51a000
    pp80 = pp80v, pp81 = pp81v, pp82 = pp82v;
    timer = pp80 - kFirstPassEarly;
    shown = false, soundOn = false, passes = 0;
}

// 0x51a380. [esp+0x1c..0x24] holds the forward, then the centre (floats); t is stored as a float before fsin / fcos.
bool F14Flyby::spawn(bool havePlayer, const V3& playerPos, const V3& playerFwd) {
    if (!havePlayer) return false;
    player = playerPos;  // 0x420330 -> +0x48
    // 0x421ab0, 0x40fc80: flattened and normalised when x^2 + z^2 > 1e-6 [0x995fc4] (else left as it is)
    V3 f = playerFwd;
    const double l2 = double(f[0]) * f[0] + double(f[2]) * f[2];
    if (1e-6 < l2) {
        const double inv = 1.0 / std::sqrt(double(float(l2)));
        f[0] = float(f[0] * inv), f[1] = 0.0f, f[2] = float(f[2] * inv);
    }
    const V3 c{float(double(f[0]) * kAhead + player[0]), float(double(f[1]) * kAhead + player[1]), float(double(f[2]) * kAhead + player[2])};
    const float r = rand01 ? rand01() : 0.0f;                       // 0x6460f0
    const float t = float(double(r) * double(6.2831854820251465));  // x [0x9dc258], fst dword
    const float a = float(std::sin(double(t)));                     // 0x40f3d0 fsin, stored
    const double b = std::cos(double(t));                           // 0x40f3f0 fcos, kept on the x87 stack
    const double s = pp81;                                          // [pp + 0x144]
    vel = {float(s * a), float(s * 0.0), float(s * b)};
    const double ak = double(a) * kHalfPass, bk = b * kHalfPass;
    pos = {float(double(c[0]) - ak), c[1], float(double(c[2]) - bk)};
    end = {float(double(c[0]) + ak), c[1], float(double(c[2]) + bk)};
    pos[1] = float(double(ground ? ground(pos[0], pos[1], pos[2]) : 0.0f) + pp82);  // 0x445500 + [pp + 0x148]
    quat = fromArc({0, 0, 1}, {a, 0.0f, float(b)});                                  // 0x6443a0, EBX = 0xa06350
    nodePos = pos, nodeQuat = quat;                                                  // 0x65feb0, 0x65f820
    // 0x4ea4a0 / 0x4ea930 / 0x4e9da0: the emitter starts (relPrev = the descriptor's zero vector [0xa66208]).
    soundOn = true;
    relPrev = {0, 0, 0};
    pitch = 1.0f;
    ++passes;
    return true;
}

void F14Flyby::step(float dt, bool worldReady, bool havePlayer, const V3& playerPos, const V3& playerFwd) {  // 0x51a050
    if (!worldReady) return;  // local_34 (the world object) null
    if (!(timer < 0.0f)) {    // fcomp 0 vs +0x20, test ah 0x41 / jp: >= 0 (or NaN) counts
        timer = float(double(dt) + timer);
        if (timer < pp80) return;
        if (!spawn(havePlayer, playerPos, playerFwd)) return;
        shown = true;  // 0x65ede0(node, 0)
        timer = kFlying;
    }
    // Flying. Node +0x2c &= ~0x8000000 (no collision). The node's world position (+0xd4 +0x30) before the move.
    const V3 old = nodePos;
    pos = {float(double(vel[0]) * dt + pos[0]), float(double(vel[1]) * dt + pos[1]), float(double(vel[2]) * dt + pos[2])};
    const V3 ahead{float(double(vel[0]) * kLookAhead + pos[0]), float(double(vel[1]) * kLookAhead + pos[1]),
                   float(double(vel[2]) * kLookAhead + pos[2])};
    const float target = float(double(ground ? ground(ahead[0], ahead[1], ahead[2]) : 0.0f) + pp82);  // [pp + 0x148] + 0x445500
    // Math_ExpSmooth 0x4100b0 (ECX = &target, ESI = EDI = &pos.y): |d| < 1e-6 snaps, else y + d (1 - 1 / 1.05^(30.30303 dt))
    {
        const double dx = double(target) - pos[1];
        const float d = float(dx);  // fst [esp]: the product below reads the stored float
        if (std::fabs(dx) < 1e-6) pos[1] = target;
        else pos[1] = float((1.0 - 1.0 / std::pow(double(kClimbRate), double(float(double(dt) * double(30.303030014038086f))))) * d + pos[1]);
    }
    const double side = (double(end[2]) - pos[2]) * vel[2] + (double(end[1]) - pos[1]) * vel[1] + (double(end[0]) - pos[0]) * vel[0];
    if (!(side < 0.0)) {  // fldz / fcom: 0 < side or 0 == side (or NaN) keeps flying
        V3 dir{float(double(pos[0]) - old[0]), float(double(pos[1]) - old[1]), float(double(pos[2]) - old[2])};
        const double l2 = double(dir[0]) * dir[0] + double(dir[1]) * dir[1] + double(dir[2]) * dir[2];
        if (1e-6 < float(l2)) {  // 0x410050 (1.0): normalised only when |dir|^2 > 1e-6
            const double inv = 1.0 / std::sqrt(double(float(l2)));
            dir = {float(dir[0] * inv), float(dir[1] * inv), float(dir[2] * inv)};
        }
        quat = fromArc({0, 0, 1}, dir);
        nodePos = pos, nodeQuat = quat;  // 0x65feb0, 0x65f820
        return;
    }
    timer = 0.0f;  // the next pass PP80 from now
    shown = false;  // node +0x2c |= 0x40000, 0x65eed0
    soundOn = false;  // 0x4ead30 -> 0x401a30 / 0x4e9e80
}

float F14Flyby::dopplerPitch(double distNow, double distPrev, float dt) {  // 0x4ea560
    const double x = (distNow - distPrev) / dt * double(-0.009999999776482582f);
    double v = -0.10000000149011612;              // [0x9dca30]
    if (-0.10000000149011612 < x) {               // fcom: C0 = -0.1 < x
        v = x;
        if (!(0.10000000149011612 > x)) v = 0.10000000149011612;  // [0x99648c]: 0.1 <= x
    }
    return float(v + 1.0);  // + [0x9963b8], pushed as a float to 0x732e30
}

float F14Flyby::doppler(float dt, const V3& cam) {
    if (!soundOn || !(dt > 0)) return pitch;  // [port] a stopped world (dt 0) never reaches 0x4eabf0
    const V3 rel{float(double(cam[0]) - nodePos[0]), float(double(cam[1]) - nodePos[1]), float(double(cam[2]) - nodePos[2])};  // 0x4ea8c0
    // Both squared lengths are pushed as floats to 0x40ed20 (fsqrt); |new| is stored as a float ([esp+0x14]), |old| stays
    // on the x87 stack. Sums: (y^2 + x^2) + z^2 for the new vector, (x^2 + y^2) + z^2 for the old one.
    const float now = float(std::sqrt(double(float((double(rel[1]) * rel[1] + double(rel[0]) * rel[0]) + double(rel[2]) * rel[2]))));
    const double prev = std::sqrt(double(float((double(relPrev[0]) * relPrev[0] + double(relPrev[1]) * relPrev[1]) + double(relPrev[2]) * relPrev[2])));
    pitch = dopplerPitch(now, prev, dt);
    relPrev = rel;
    return pitch;
}

}  // namespace fuel
