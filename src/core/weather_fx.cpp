// FUEL's rain splash and thunder tanks: see weather_fx.hpp and notes p48-effects2.md.
#include "weather_fx.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace fuel::wfx {

namespace {
float P(const std::vector<float>& pp, int n) { return n >= 0 && size_t(n) < pp.size() ? pp[size_t(n)] : 0.0f; }
float lerpf(float a, float b, float t) { return (b - a) * t + a; }
}  // namespace

SplashParams SplashParams::fromPP(const std::vector<float>& pp) {
    SplashParams p;
    p.radius = P(pp, 5037);
    p.life = P(pp, 5039);
    p.colour = {P(pp, 5030), P(pp, 5031), P(pp, 5032), P(pp, 5033)};
    p.scale = {P(pp, 5034), P(pp, 5035), P(pp, 5036)};
    return p;
}

void SplashTank::start(int n, const RndInt& well) {  // 0x5e0bc0
    splashes_.assign(size_t(std::max(n, 0)), Splash{});
    ring_.assign(size_t(3 * std::max(n, 0)), V3{0, 0, 0});  // 0x5e0e20
    ringIdx_ = 0;
    drawn_ = n;
    // populate 0x5e0e90: phase = age = (1 - i / N) x life, lift 0.2, a random sign for the u flip.
    for (int i = 0; i < n; ++i) {
        Splash& s = splashes_[size_t(i)];
        s.phase = s.age = (1.0f - float(i) / float(n)) * p_.life;
        s.flip = well() < 1 ? -0.0625f : 0.0625f;
    }
    cursor_ = 0;
}

void SplashTank::setAmount(float amount) {  // 0x5e0990
    drawn_ = int(float(splashes_.size()) * amount);  // ftol
}

void SplashTank::update(float dt, const V3& tankPos, const Rnd& game, const Ground& ground) {  // 0x5e10e0
    if (drawn_ <= 0 || splashes_.empty()) return;
    const int drawn = std::min(drawn_, int(splashes_.size()));
    for (int i = 0; i < drawn; ++i) splashes_[size_t(i)].age += dt;
    const float r2 = p_.radius * p_.radius;
    if (cursor_ >= drawn) cursor_ = 0;
    int k = cursor_;
    for (int moved = 0; splashes_[size_t(k)].age >= p_.life; ++moved) {
        if (moved > 0x1f) return;
        V3& rp = ring_[size_t(ringIdx_)];
        const float dx = tankPos[0] - rp[0], dy = tankPos[1] - rp[1], dz = tankPos[2] - rp[2];
        if (r2 < dx * dx + dy * dy + dz * dz) {  // out of range: a new random point (three 0x6460f0 draws, z first)
            const float a = game(), b = game(), c = game();
            rp = {(c + c - 1) * p_.radius + tankPos[0], (b + b - 1) * p_.radius + tankPos[1], tankPos[2] + p_.radius * (a + a - 1)};
            rp[1] = (ground ? ground(rp[0], rp[2]) : 0.0f) + p_.lift;  // 0x63ea40 when the world is there
        }
        Splash& s = splashes_[size_t(k)];
        s.flip = -s.flip;  // the u of the quad's corners swaps with the sign
        s.pos = rp;
        s.age = std::fmod(s.age, p_.life);  // 0x8e05fa
        if (++ringIdx_ >= int(ring_.size())) ringIdx_ = 0;
        if (++cursor_ >= drawn) cursor_ = 0;
        if (++k >= drawn) k = 0;
    }
}

ThunderParams ThunderParams::fromPP(const std::vector<float>& pp) {
    ThunderParams p;
    p.intervalMin = P(pp, 5230), p.intervalMax = P(pp, 5231);
    p.driftMin = P(pp, 5232), p.driftMax = P(pp, 5233);
    p.distMin = P(pp, 5234), p.distMax = P(pp, 5235);
    p.rangeBolt = P(pp, 5238), p.rangeCloud = P(pp, 5239);
    p.intensity = P(pp, 5240);
    p.lifeMin = P(pp, 5241), p.lifeMax = P(pp, 5242);
    p.widthRatio = P(pp, 5243);
    p.above = P(pp, 5245);
    p.second = P(pp, 5246);
    return p;
}

Bolt& ThunderTank::spawn(const V3& tankPos, float layer, const RndInt& well, const Rnd& game, const Ground& ground, const V3* at) {
    // 0x5e2d40. WELL512 integers (0x5fd710) and its uniform ((bits & 0x7fffff | 1.0f) - 1) as the game uses them.
    auto wellU = [&]() {
        uint32_t b = (uint32_t(well()) & 0x7fffffu) | 0x3f800000u;
        float f;
        std::memcpy(&f, &b, 4);
        return f - 1.0f;
    };
    bolts_.push_back(Bolt{});
    Bolt& b = bolts_.back();
    const int32_t t = well();
    b.type = int((uint32_t(t < 0 ? -int64_t(t) : t)) & 3u);
    const float a = lerpf(p_.driftMin, p_.driftMax, wellU());
    const float g0 = game(), g1 = game(), g2 = game();
    b.vel = {g2 * (a + a) - a, 0.0f, g0 * (a + a) - a};  // +0x38 = 0 (the y draw g1 is overwritten)
    (void)g1;
    b.u0 = float(well() % 3) / 4.0f;  // C remainder: negative for negative draws (the sampler wraps)
    const float rU = wellU(), thU = wellU();
    const float th = thU * 6.2831855f;
    const float r = lerpf(p_.distMin, p_.distMax, rU);
    const float c = std::cos(th), s = std::sin(th);  // 0x40f3f0 / 0x40f3d0
    float x = (c - s) * r + tankPos[0], z = (s + c) * r + tankPos[2];
    float y = ground ? ground(x, z) : tankPos[1];  // 0x445500 when the world is there
    if (at) x = (*at)[0], y = (*at)[1], z = (*at)[2];
    b.ground = {x, y, z};
    b.top = {x, tankPos[1] + p_.above + layer, z};
    b.height = b.top[1] - b.ground[1];
    b.width = b.height * p_.widthRatio;
    b.life = lerpf(p_.lifeMin, p_.lifeMax, game());
    b.fadeTime = 0.3f;
    b.range = b.type == 2 ? p_.rangeCloud : p_.rangeBolt;
    return b;
}

void ThunderTank::update(float dt, const V3& tankPos, const V3& forward, float layer, const RndInt& well, const Rnd& game,
                         const Ground& ground, const ThunderHook& hook) {  // 0x5e32a0
    if (amount_ < 0.95f) {
        if (stormOn_ && hook) hook(ThunderEvent{ThunderEvent::StormStop, 0x67});
        stormOn_ = false;
        drawn_ = 0;
        return;
    }
    if (!stormOn_ && hook) hook(ThunderEvent{ThunderEvent::StormStart, 0x67});
    stormOn_ = true;
    if (external) {
        for (const V3& at : pending_) spawn(tankPos, layer, well, game, ground, &at);
        if (!pending_.empty()) strikeEvent(tankPos, forward, game, hook);
        pending_.clear();
    } else if (0.0f < timer_) {
        timer_ -= dt;
    } else {
        timer_ = game() * (p_.intervalMax - p_.intervalMin) + p_.intervalMin;
        spawn(tankPos, layer, well, game, ground);
        uint32_t bits = (uint32_t(well()) & 0x7fffffu) | 0x3f800000u;
        float u;
        std::memcpy(&u, &bits, 4);
        if (u - 1.0f < p_.second) spawn(tankPos, layer, well, game, ground);
        strikeEvent(tankPos, forward, game, hook);
    }
    drawn_ = 0;
    float nearest = 99999.0f;
    for (size_t i = 0; i < bolts_.size();) {
        Bolt& b = bolts_[i];
        b.life -= dt;
        if (0.0f <= b.life) {
            b.frac = std::max(std::min(b.life, b.fadeTime), 0.0f) / b.fadeTime;
            if (b.type != 2) {
                ++drawn_;
                const float dx = b.ground[0] - tankPos[0], dy = b.ground[1] - tankPos[1], dz = b.ground[2] - tankPos[2];
                nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
                nearest_ = nearest;
            } else {
                for (int k = 0; k < 3; ++k) b.top[size_t(k)] += b.vel[size_t(k)] * dt, b.ground[size_t(k)] += b.vel[size_t(k)] * dt;
            }
            ++i;
        } else {
            bolts_.erase(bolts_.begin() + long(i));  // 0x5e3ca0
        }
    }
}

void ThunderTank::strikeEvent(const V3& tankPos, const V3& forward, const Rnd& game, const ThunderHook& hook) {
    {
        const Bolt& last = bolts_.back();
        ThunderEvent ev;
        ev.kind = ThunderEvent::Strike;
        ev.type = last.type;
        ev.pos = last.top;
        const float dx = last.ground[0] - tankPos[0], dz = last.ground[2] - tankPos[2];
        ev.distance = std::sqrt(dx * dx + dz * dz);
        if (last.type != 2 && 0.5f < forward[0] * dx + forward[2] * dz)  // in front (asm 0x5e35a7: unnormalised)
            ev.flash = 0.45f * (1.0f - ev.distance / p_.distMax);       // [0x9dd418] 0.45, white, 1 s (HUD +0x4830)
        float g = game() * 3.9999f;  // [0x9dd2c4]
        ev.sound = 0x63 + int(g);
        lastFlash = ev.flash;
        if (hook) hook(ev);
    }
}

std::array<ThunderTank::Flash, 4> ThunderTank::flashes() const {
    std::array<Flash, 4> f{};  // unused slots: position 0, strength 0 (range 2000)
    if (amount_ < 0.95f) return f;
    for (size_t i = 0; i < bolts_.size() && i < 4; ++i) f[i] = {bolts_[i].top, bolts_[i].frac * p_.intensity};
    return f;
}

float ThunderTank::stormVolume() const { return std::clamp((amount_ - 0.95f) * 2.0f, 0.0f, 1.0f); }

std::vector<BoltQuad> boltQuads(const ThunderTank& t) {  // 0x5e3110
    std::vector<BoltQuad> out;
    if (!t.storm()) return out;
    for (const Bolt& b : t.bolts()) {
        if (b.type == 2) continue;
        BoltQuad q;
        q.centre = {b.ground[0], b.height * 0.5f + b.ground[1], b.ground[2]};
        q.halfW = b.width * 0.5f, q.halfH = b.height * 0.5f;
        q.u0 = b.u0, q.u1 = b.u0 + 0.25f;
        q.rgb = t.params().intensity;
        q.alpha = b.frac;
        out.push_back(q);
    }
    return out;
}

// ---- PK24: the screen projections (lens water / mud / dirt) --------------------------------------------------------
// Decode: notes/pk24.md (decomp 0x5fd390, 0x5fd270, 0x5fff30, 0x600a40, 0x600d10, 0x600f70, 0x600160, 0x5ff8b0, 0x5ffa70).
namespace {
float urand(const RndInt& well) {  // (bits & 0x7fffff | 0x3f800000) - 1: [0, 1)
    const uint32_t b = (uint32_t(well()) & 0x7fffffu) | 0x3f800000u;
    float f;
    std::memcpy(&f, &b, 4);
    return f - 1.0f;
}
int absMod(const RndInt& well, int m) {  // (abs(r)) % m as 0x5fff30 / 0x600d10 / 0x600a40 compute it
    const int32_t r = well();
    const uint32_t a = r < 0 ? uint32_t(0) - uint32_t(r) : uint32_t(r);
    return int(a % uint32_t(m));
}
constexpr float kPi = 3.14159265358979f;
}  // namespace

LensParams LensParams::fromPP(const std::vector<float>& pp) {
    LensParams p;
    p.dropMin = P(pp, 5510), p.dropMax = P(pp, 5511), p.dropRain = P(pp, 5512);
    p.splashMin = P(pp, 5513), p.splashMax = P(pp, 5514), p.splashSpeed = P(pp, 5515);
    p.dropSizeMin = P(pp, 5516), p.dropSizeMax = P(pp, 5517);
    p.splashSizeMin = P(pp, 5518), p.splashSizeMax = P(pp, 5519);
    p.mudMin = P(pp, 5520), p.mudMax = P(pp, 5521);
    p.mudSizeMin = P(pp, 5522), p.mudSizeMax = P(pp, 5523);
    return p;
}

bool ScreenFx::anyWater() const {
    for (const LensEntry& e : entries_)
        if (e.kind == LensEntry::Water) return true;
    return false;
}

LensEntry& ScreenFx::spawnBase(LensEntry::Kind k) {  // 0x600680: colour (0.5, 0.5, 0.5, 1), unit size, corners (0,0) (1,0) (1,1) (0,1)
    LensEntry e;
    e.kind = k;
    e.uv = {0, 0, 1, 0, 1, 1, 0, 1};
    e.spawn = clock_;
    e.life = 1;
    e.atlas = 30;
    entries_.push_back(e);
    return entries_.back();
}

void ScreenFx::spawnMud(const RndInt& well) {  // 0x5fff30
    LensEntry& e = spawnBase(LensEntry::Mud);
    e.life = 4;
    const float s = lerpf(p_.mudSizeMin, p_.mudSizeMax, urand(well));
    const float x = urand(well) * 2 - 1;
    e.size = {s, s};
    e.pos = {x, (std::fabs(x) * 0.7f + 0.3f) * urand(well) - 1.0f};
    e.vel = {0, 0};  // x 0.0: the splats are static
    e.angle = urand(well) * 2 * kPi - kPi;
    e.atlas = 0x20 + absMod(well, 3);
}

void ScreenFx::spawnDrop(const RndInt& well, const AtlasRect& rect) {  // 0x600a40
    LensEntry& e = spawnBase(LensEntry::Water);
    e.life = 4;
    const float s = lerpf(p_.dropSizeMin, p_.dropSizeMax, urand(well));
    e.size = {s, s};
    float x = std::sqrt(urand(well)), y = std::sqrt(urand(well));  // 0x40f970 (u, 0.5)
    if (well() < 1) x = -x;
    if (well() < 1) y = -y;
    e.pos = {x, y};
    e.angle = urand(well) * 2 * kPi - kPi;
    e.atlas = 0x26 + absMod(well, 4);
    const std::array<float, 4> r = rect(e.atlas);  // u0, v0, du, dv
    e.uv = {r[0], r[1] + r[3], r[0] + r[2], r[1] + r[3], r[0] + r[2], r[1], r[0], r[1]};
}

void ScreenFx::spawnSplash(bool a, bool b, const RndInt& well) {  // 0x600d10
    LensEntry& e = spawnBase(LensEntry::Water);
    e.life = 4;
    const float s = lerpf(p_.splashSizeMin, p_.splashSizeMax, urand(well));
    e.size = {s, s};
    float x = urand(well) * 2 - 1;
    const float ax = std::fabs(x);
    const float yv = urand(well);
    if (!a && x < 0) x = -x;        // no wheels of the first pair in water: x >= 0 (as coded)
    if (!b) x = -std::fabs(x);      // none of the second pair: x <= 0
    e.pos = {x, (ax * 0.7f + 0.3f) * yv - 1.0f};
    e.spawn -= e.life * 0.00390625f;  // back-dated by life / 256
    e.atlas = 0x23 + absMod(well, 3);
}

void ScreenFx::spawnDirt() {  // 0x5ffa70: the top strip, the bottom strip, the left and right edges; atlas 30, life 100 s
    struct S {
        float size[2], pos[2], uv[8];
    };
    static const S strips[4] = {
        {{16.0f / 9.0f, 0.05f}, {0, 0.95f}, {0, 0.05f, 1, 0.05f, 1, 0, 0, 0}},
        {{16.0f / 9.0f, 0.25f}, {0, -0.75f}, {0, 1, 1, 1, 1, 0.75f, 0, 0.75f}},
        {{0.17578125f, 0.7f}, {-0.9f, 0.2f}, {0, 0.75f, 0.1f, 0.75f, 0.1f, 0.05f, 0, 0.05f}},
        {{0.17578125f, 0.7f}, {0.9f, 0.2f}, {0.9f, 0.75f, 1, 0.75f, 1, 0.05f, 0.9f, 0.05f}},
    };
    for (const S& s : strips) {
        LensEntry& e = spawnBase(LensEntry::Dirt);
        e.life = 100;
        e.size = {s.size[0], s.size[1]};
        e.pos = {s.pos[0], s.pos[1]};
        for (int i = 0; i < 8; ++i) e.uv[size_t(i)] = s.uv[i];
        e.atlas = 30;
    }
}

void ScreenFx::update(float dt, float rain, const LensVehicle& veh, const RndInt& well, const AtlasRect& atlasRect) {
    clock_ += dt;  // manager vf17: +0xd8
    const float speed = veh.present ? std::sqrt(veh.velocity[0] * veh.velocity[0] + veh.velocity[1] * veh.velocity[1] + veh.velocity[2] * veh.velocity[2]) : 0.0f;
    // ---- 0x5fd390: water on the lens
    if (veh.present) vel_ = veh.velocity;
    if (rain > 0.05f) {
        dropTimer_ -= dt;
        if (dropTimer_ <= 0) {
            const float k = (1.0f - rain) * p_.dropRain + 1.0f;
            dropTimer_ = k * p_.dropMin + urand(well) * (k * p_.dropMax - k * p_.dropMin);
            spawnDrop(well, atlasRect);
        }
    }
    splashTimer_ -= dt;
    if (splashTimer_ <= 0 && veh.present && p_.splashSpeed <= speed) {
        bool a = false, b = false;
        for (int w = 0; w < veh.wheels && w < 16; ++w)
            if (veh.waterDepth[size_t(w)] > 0.01f) ((w & 2) == 0 ? a : b) = true;
        if (a || b) {
            splashTimer_ = lerpf(p_.splashMin, p_.splashMax, urand(well));
            spawnSplash(a, b, well);
        }
    }
    // ---- 0x5fd270: mud on the lens
    mudTimer_ -= dt;
    if (mudTimer_ <= 0 && veh.present && 3.0f <= speed && veh.dust) {
        mudTimer_ = lerpf(p_.mudMin, p_.mudMax, urand(well));
        spawnMud(well);
    }
    // ---- the projections' vf02, in list order (mud, water, dirt): the base cull 0x6005c0, then each update
    entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                  [&](const LensEntry& e) { return e.kind != LensEntry::Dirt && e.spawn + e.life < clock_; }),
                   entries_.end());
    const float k = std::clamp(0.5f * std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1] + vel_[2] * vel_[2]), 0.0f, 1.0f);
    const float sk = std::clamp(0.05f * k, 0.0f, 0.2f);
    for (LensEntry& e : entries_) {
        if (e.kind == LensEntry::Mud) {  // 0x600160 (the splats' velocity is 0)
            e.rgba[3] = (1.0f - (clock_ - e.spawn) / e.life) * e.startAlpha;
            e.pos[0] += e.vel[0] * dt, e.pos[1] += e.vel[1] * dt;
        } else if (e.kind == LensEntry::Water) {  // 0x600f70: slides down when slow, streams outward when fast
            e.rgba[3] = (1.0f - (clock_ - e.spawn) / e.life) * e.startAlpha;
            float vx = e.pos[0], vy = e.pos[1];
            const float l2 = vx * vx + vy * vy;
            if (l2 > 1e-6f) {
                const float l = std::sqrt(l2);
                vx = e.pos[0] * (sk / l);
                vy = (sk / l) * e.pos[1];
            }
            vy = (vy + 0.05f) * k - 0.05f;
            vx *= k;
            e.pos[0] += vx * dt, e.pos[1] += vy * dt;
        }
    }
    // ---- 0x5ff8b0: DirtScreen vf02
    const bool haveStrips = std::any_of(entries_.begin(), entries_.end(), [](const LensEntry& e) { return e.kind == LensEntry::Dirt; });
    float a = 0;
    if (veh.present && veh.dirtValid) {
        const float d = veh.dirt;
        if (!haveStrips) {
            if (d > 0.1f) spawnDirt();
        } else if (d <= 0.1f) {
            entries_.erase(std::remove_if(entries_.begin(), entries_.end(), [](const LensEntry& e) { return e.kind == LensEntry::Dirt; }), entries_.end());
        }
        a = d > 0.5f ? (d - 0.5f) + (d - 0.5f) + 1.0f : d + d;
    }
    const bool valid = veh.present && veh.dirtValid;
    for (LensEntry& e : entries_) {
        if (e.kind != LensEntry::Dirt) continue;
        e.life += dt;
        e.pos[0] += e.vel[0] * dt, e.pos[1] += e.vel[1] * dt;
        e.rgba = {1.5f, 1.5f, 1.5f, valid ? a : 1.0f};  // colour (0.5, 0.5, 0.5, 1) x 3; alpha: the dirt parameter
    }
}
// ---- PK24 end

}  // namespace fuel::wfx
