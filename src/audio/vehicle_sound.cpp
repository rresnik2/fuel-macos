#include "vehicle_sound.hpp"

#include "core/genworld_noise.hpp"
#include "core/vehicle_params.hpp"
#include "core/vibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fuel::snd {

namespace {
constexpr uint32_t kSurfaceTable = 0xa06160;  // 18 x {i32 slow, i32 fast, i32 skid, f32 gravel, i32 cap PP}
constexpr float kTwoPi = 6.2831855f;

float clamp01(float v) { return v > 0 ? (v < 1 ? v : 1.0f) : 0.0f; }  // 0x410330(v, 0, 1) / 0x5ddf90
float len3(Vec3f v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

// Linear move toward a target (0x4e9850): |t - c| < 1e-6 -> t.
float approach(float cur, float tgt, float rate, float dt) {
    if (std::fabs(tgt - cur) < 1e-6f) return tgt;
    return tgt < cur ? std::max(cur - rate * dt, tgt) : std::min(cur + rate * dt, tgt);
}

// Vector into the chassis frame (inverse rotation by the body quaternion; x side, y up, z forward).
Vec3f toBody(Vec3f v, const std::array<float, 4>& q) {
    const float x = -q[0], y = -q[1], z = -q[2], w = q[3];
    const float tx = 2 * (y * v.z - z * v.y), ty = 2 * (z * v.x - x * v.z), tz = 2 * (x * v.y - y * v.x);
    return {v.x + w * tx + (y * tz - z * ty), v.y + w * ty + (z * tx - x * tz), v.z + w * tz + (x * ty - y * tx)};
}

// Index of the dominant ground class: the largest weight > 0 (first wins ties), -1 if none.
int dominantClass(const std::array<float, 18>& w) {
    int best = -1;
    float m = 0;
    for (int k = 0; k < 18; ++k)
        if (w[size_t(k)] > 0 && w[size_t(k)] > m) best = k, m = w[size_t(k)];
    return best;
}
}  // namespace

// ---- tuning ---------------------------------------------------------------------------------------

Tuning Tuning::load(const std::string& exe, const std::string& pp001) {
    Tuning t;
    t.pp.assign(6000, 0.0f);
    std::ifstream f(pp001, std::ios::binary);
    if (!f) {
        t.error = "cannot read " + pp001;
        return t;
    }
    std::string line;
    int rows = 0;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tag, platform;
        int n = 0;
        float v = 0;
        if (!(ls >> tag >> n >> v) || tag != "PP" || n < 0 || n >= 6000) continue;
        if (ls >> platform && platform != "PC") continue;  // the PC build reads its own rows
        t.pp[size_t(n)] = v, ++rows;
    }
    if (rows < 1000 || t.P(2009) <= 0 || t.P(2020) <= 0) {
        t.error = "pp001.tsc has no sound rows";
        return t;
    }
    const std::vector<float> raw = readEngineFloats(exe, kSurfaceTable, 18 * 5);
    if (raw.size() != 18 * 5) {
        t.error = "cannot read the surface sound table from " + exe;
        return t;
    }
    uint64_t h = 1469598103934665603ull;
    const auto* bytes = reinterpret_cast<const uint8_t*>(raw.data());
    for (size_t i = 0; i < raw.size() * 4; ++i) h = (h ^ bytes[i]) * 1099511628211ull;
    t.surfaceHash = h;
    for (size_t k = 0; k < 18; ++k) {
        int32_t v[5];
        std::memcpy(v, &raw[k * 5], sizeof v);
        SurfaceRow& r = t.surface[k];
        r.slow = v[0], r.fast = v[1], r.skid = v[2];
        std::memcpy(&r.gravelRate, &v[3], 4);
        r.pitchCapPP = v[4];
        if (r.slow > 0xff || r.fast > 0xff || r.skid > 0xff || r.pitchCapPP < 0 || r.pitchCapPP >= 6000) {
            t.error = "FUEL.exe surface sound table looks wrong (not the version the port knows)";
            return t;
        }
    }
    t.ok = true;
    return t;
}

// ---- Game_SoundMgr mix ----------------------------------------------------------------------------

void GameMix::Fader::update(float dt) {  // 0x4eaf00
    if (tgt > cur) cur = up > 0 ? approach(cur, tgt, up, dt) : tgt;
    else if (tgt < cur) cur = down > 0 ? approach(cur, tgt, down, dt) : tgt;
    if (ovr > 0) tgt = ovr;
}

GameMix::GameMix(const Tuning& t) : t_(t) {
    // 0x4eaf90: every fader at 1; rates (up, down): 0 (1, -1), 1..3 (0.25, 0.5), 4 (1, -1, override 1), 5 (0.25, 0.5).
    for (int i = 1; i <= 5; ++i)
        if (i != 4) rec[size_t(i)].up = 0.25f, rec[size_t(i)].down = 0.5f;
    rec[4].ovr = 1;
    loadCats(cat, 2020);
    loadCats(catTarget, 2020);
}

void GameMix::loadCats(std::array<float, kCats>& out, int base) const {  // 0x4eb0a0 (+0x7c.. <- PP base..)
    static constexpr Cat order[15] = {Global, Motor, Bang, Rolling, Skid, Impact, Limiter, Absorber, Amb0, Amb1, Amb2, Amb3, Amb4, Amb5, Gravel};
    for (int i = 0; i < 15; ++i) out[size_t(order[i])] = t_.P(base + i);
}

void GameMix::setView(int v) {
    view = v;
    loadCats(catTarget, ppBaseForView(v));
}

void GameMix::update(float dt) {
    for (size_t c = 0; c < kCats; ++c) cat[c] = approach(cat[c], catTarget[c], 2.0f, dt);  // 0x4eb2a0
    for (Fader& f : rec) f.update(dt);                                                      // 0x4eb270
}

float GameMix::gain(Cat c) const { return master * cat[Global] * rec[4].cur * cat[c]; }

float GameMix::motorGain(bool player) const {
    float g = gain(Motor);
    if (player) g *= std::min({rec[0].cur, rec[1].cur, rec[2].cur, rec[3].cur});
    return g;
}

// ---- bank -----------------------------------------------------------------------------------------

VehicleBank loadVehicleBank(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const Tuning& t) {
    VehicleBank b;
    b.variant = variant;
    b.engine = EngineModel::fromParams(params, t.P(2009));
    b.wheels = std::clamp(int(params.size() > 45 ? params[45] + 0.5f : 4.0f), 1, 6);
    const SoundBank bank = readSoundBank(dpc, variant);
    for (const auto& [symbol, id] : bank) {
        if (symbol >= b.bySymbol.size()) continue;
        try {
            b.bySymbol[symbol] = Pcm::from(decodeSound(dpc, id));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: sound %02x: %s\n", symbol, e.what());
        }
    }
    if (!b.bySymbol[2]) throw std::runtime_error(variant + ": no engine idle sample in its sound bank");
    return b;
}

// ---- inputs ---------------------------------------------------------------------------------------

VehicleInput VehicleInput::from(const VehicleState& st, float dt) {
    VehicleInput in;
    in.dt = dt;
    in.soundRpm = st.soundRpm;
    in.engineThrottle = st.engineThrottle;
    in.revLimiter = st.revLimiter;
    in.velocity = {st.velocity[0], st.velocity[1], st.velocity[2]};
    in.position = {float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2])};
    in.rotation = st.chassis.rotation;
    in.impact = {st.soundImpact[0], st.soundImpact[1], st.soundImpact[2]};
    in.impactMeter = st.impactMeter;
    in.slipPedal = st.slipPedal;
    for (const VehicleState::WheelFx& fx : st.wheelFx) {
        Wheel w;
        w.load = fx.load;
        w.depth = fx.depth;
        w.slip = fx.slip;
        // 0x5c8730: |hub point velocity + R(q) k x veh+0x15b8| = the contact patch's sliding speed
        // (the rotated vector is Inferred to be the rolling direction x radius; veh+0x15b8 = -rim / r).
        const float rim = fx.spin * fx.radius;
        const float sx = fx.velocity[0] - fx.forward[0] * rim, sy = fx.velocity[1] - fx.forward[1] * rim, sz = fx.velocity[2] - fx.forward[2] * rim;
        w.slipSpeed = std::sqrt(sx * sx + sy * sy + sz * sz);
        w.classes = fx.classes;
        in.wheels.push_back(w);
    }
    return in;
}

// ---- vehicle sound --------------------------------------------------------------------------------

VehicleSound::VehicleSound(std::shared_ptr<const VehicleBank> bank, const Tuning& t, bool player, uint32_t seed)
    : bank_(std::move(bank)), t_(t), player_(player), rng_(seed ? seed : 1) {
    // 0x5d8a40: symbols and fade rates per slot.
    auto set = [&](int i, int symbol, float in, float out) { slots_[size_t(i)].symbol = symbol, slots_[size_t(i)].fadeIn = in, slots_[size_t(i)].fadeOut = out; };
    for (int i = 0; i < 12; ++i) set(i, i + 2 == 8 ? 0 : i + 2, 0, 0);                         // engine
    for (int i = 0; i < 17; ++i) set(12 + i, 0x0e + i, 1, 1);                                   // skids
    for (int i = 0; i < 34; ++i) set(29 + i, 0x20 + i, 1, 2);                                   // rolling, wind
    set(29 + 0x21, 0x41, 1, 0.3f);                                                               // whine
    for (int i = 0; i < 10; ++i) set(63 + i, 0x42 + i, 10, 0x42 + i == 0x47 ? 2.5f : 3.0f);    // impacts, scrape
    for (int i = 0; i < 6; ++i) set(73 + i, 0x54 + i, 0, 0);  // absorbers (fade-in from 0x5c7f70: Unknown, port 0)
    set(79, 0x5a, 100, 0.5f);                                                                    // big thump
    for (int i = 0; i < 4; ++i) set(80 + i, 0x5e + i, 0, 0);                                    // backfire
    for (int i = 0; i < 3; ++i) set(84 + i, 0x5b + i, 0, 0);
    set(87, 0x62, 0, 0);
    for (int i = 0; i < 8; ++i) set(88 + i, 0x4c + i, 0, 0);                                    // gravel
    impactTimer_ = 1000;                    // +0x2ec8
    gravelInterval_ = t_.P(2152);           // +0x2eec
}

float VehicleSound::rand01() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return float(rng_ >> 8) * (1.0f / 16777216.0f);
}

int VehicleSound::slotOf(int symbol) const {
    for (int i = 0; i < kSlots; ++i)
        if (slots_[size_t(i)].symbol == symbol && symbol != 0) return i;
    return -1;
}

VehicleSound::Slot* VehicleSound::bySymbol(int symbol) {
    const int i = slotOf(symbol);
    return i < 0 ? nullptr : &slots_[size_t(i)];
}

void VehicleSound::setTarget(int symbol, float v) {
    if (Slot* s = bySymbol(symbol)) s->target = clamp01(v);
}

// 0x5d98c0: the engine layers.
void VehicleSound::motor(const VehicleInput& in, float gain) {
    const EngineModel& e = bank_->engine;
    // Load +0x2ec4 -> 1.1 t / (t + 0.1) at PP 2008 per second (t = veh+0x1088, the engine throttle).
    const float thr = in.engineThrottle;
    load_ = approach(load_, (thr * 1.1f) / (thr + 0.1f), t_.P(2008), in.dt);
    phase_ += in.dt;
    if (phase_ > kTwoPi) phase_ -= kTwoPi;
    const float wobble = std::cos(phase_ * 6.0f) * std::sin(phase_ * 21.0f) * std::cos(phase_) * 0.66f + 30.0f;
    x_ = std::clamp(e.normalised(in.soundRpm) * wobble * 0.032258064f, 0.001f, 0.99f);
    const float xc = e.curve(x_);
    // The speed duck: V = gain x max(0.75, 1 - min(|v| / 35, 1)).
    const float f = 1.0f - std::min(len3(in.velocity) * 0.028571429f, 1.0f);
    const float V = (f > 0.75f ? std::min(f, 1.0f) : 0.75f) * gain;
    const EngineModel::Mix m = e.mixAt(xc);
    if (Slot* s = bySymbol(2)) s->target = std::min(std::max(m.weight[0] * V, 0.0f), 1.0f), s->pitch = m.pitch[0];
    for (int L = 1; L < EngineModel::kLayers; ++L) {
        if (Slot* s = bySymbol(L + 2)) s->target = clamp01((1.0f - load_) * m.weight[size_t(L)] * V), s->pitch = m.pitch[size_t(L)];
        if (Slot* s = bySymbol(L + 8)) s->target = clamp01(m.weight[size_t(L)] * load_ * V), s->pitch = m.pitch[size_t(L)];
    }
}

// 0x5d9d60: backfire. A pop when the engine throttle (veh+0x1088, shift cut included) is at most P653,
// the normalised sound rpm at least P654, the sound rpm falling faster than P646 rpm/s (frame to frame,
// not smoothed) and the pop timer has run out. The timer restarts at P645 x (1 + 0.5 rand); the volume
// is clamp((rate - P646) / P646, 0, 1) x the bang gain; one of the four pops at random, only if its
// slot is free (eight tries). +0x2ebc (an exhaust flash level, decays 2/s) is set to 1.
void VehicleSound::bang(const VehicleInput& in, float gain) {
    const EngineModel& e = bank_->engine;
    if (flash_ > 0) flash_ = std::max(0.0f, flash_ - 2.0f * in.dt);
    popTimer_ -= in.dt;
    rpmRate_ = (in.soundRpm - prevRpm_) / in.dt;
    prevRpm_ = in.soundRpm;
    if (e.popThrottleMax < in.engineThrottle || e.normalised(in.soundRpm) < e.popRpmMin || e.popRateThreshold <= rpmRate_ || popTimer_ >= 0) return;
    popTimer_ = (rand01() * 0.5f + 1.0f) * e.popInterval;
    const float vol = clamp01(-((rpmRate_ - e.popRateThreshold) / -e.popRateThreshold));
    for (int tries = 0; tries < 8; ++tries) {
        const int k = std::min(int(rand01() * 3.9999f), 3);
        Slot& s = slots_[size_t(80 + k)];
        if (s.voice >= 0) continue;  // busy
        flash_ = 1;
        s.target = clamp01(vol * gain);
        ++pops_;
        lastPopVol_ = s.target;
        rumble("CLUTCH", 1, 1.0f);  // P89: every pop (DAT_00a66760)
        return;
    }
}

// 0x5dad60: wind, transmission whine, rolling per wheel and ground class.
void VehicleSound::rolling(const VehicleInput& in, float gain) {
    std::array<float, 34> vol{}, pitch{};
    pitch.fill(1.0f);
    const float v = len3(in.velocity);
    vol[0x20] = clamp01(v * 0.028571429f);               // 0x40 wind
    vol[0x21] = bank_->engine.whineGain * x_ * gain;     // 0x41 whine (P650)
    pitch[0x21] = x_ * 0.5f + 1.0f;
    const int n = bank_->wheels;
    for (int w = 0; w < n && w < int(in.wheels.size()); ++w) {
        const VehicleInput::Wheel& wh = in.wheels[size_t(w)];
        if (!(wh.load > 0.01f)) continue;
        if (wh.depth <= 0) {
            const int c = dominantClass(wh.classes);
            if (c < 0) continue;
            const Tuning::SurfaceRow& r = t_.surface[size_t(c)];
            float slow, fast = 0;
            if (v > 4.0f) {
                if (v > 40.0f) slow = 0, fast = 1;
                else slow = 1.0f - (v - 4.0f) * 0.027777778f, fast = 1.0f - slow;
            } else {
                slow = v * 0.25f;
            }
            if (r.fast < 0 && fast > 0) slow = 1, fast = 0;
            if (r.slow >= 0 && unsigned(r.slow - 0x20) < 0x22) vol[size_t(r.slow - 0x20)] += slow / float(n);
            if (r.fast >= 0 && unsigned(r.fast - 0x20) < 0x22) vol[size_t(r.fast - 0x20)] += fast / float(n);
        } else {  // in water: 0x24 / 0x25 over 2..20 m/s, a quarter per wheel
            float a, b = 0;
            if (v > 2.0f) {
                if (v > 20.0f) a = 0, b = 1;
                else a = 1.0f - (v - 2.0f) * 0.055555556f, b = 1.0f - a;
            } else {
                a = 0.5f * v;
            }
            vol[4] += a * 0.25f;
            vol[5] += b * 0.25f;
        }
    }
    for (int i = 0; i < 34; ++i) {
        Slot& s = slots_[size_t(29 + i)];
        s.target = clamp01(clamp01(vol[size_t(i)]) * gain);
        s.pitch = pitch[size_t(i)];
    }
}

// 0x5da7c0: skids per wheel and ground class; feeds the gravel accumulator and ducks the engine
// (fader 1 target = 1 - (1 - PP 2221) x min(total skid, 1)).
float soundSlip(const VehicleInput::Wheel& wh, float speed, float pedal, float k) {
    if (!(wh.load > 0.01f)) return 0.0f;
    const float s = (speed < 2.0f ? 0.0f : pedal) * k * 2.0f + wh.slip;
    return s > k ? s - k : 0.0f;
}

void VehicleSound::skid(const VehicleInput& in, float gain, GameMix& mix) {
    std::array<float, 17> vol{}, pitchAdd{};
    const int n = bank_->wheels;
    const float speed = len3(in.velocity), k = bank_->engine.skidThreshold;
    for (int w = 0; w < n && w < int(in.wheels.size()); ++w) {
        VehicleInput::Wheel wh = in.wheels[size_t(w)];
        // P103: 0x5da710 (asm 0x5da710..0x5da7b7): s = slip ratio (0x523c50) + 2 x pedal (0x5cd8f0, 0 below 2 m/s) x P634,
        // and the wheel slides only by what s exceeds P634 (s - P634, else 0). The port used the bare ratio, so every
        // loaded wheel "skidded" at its friction use (the player's skid loop at full volume through a whole race).
        wh.slip = soundSlip(wh, speed, in.slipPedal, k);
        if (wh.depth > 0 || !(wh.slip > 0)) continue;
        const int c = dominantClass(wh.classes);
        if (c < 0) continue;
        const Tuning::SurfaceRow& r = t_.surface[size_t(c)];
        if (r.skid >= 0 && unsigned(r.skid - 0x0e) < 0x11) {
            const size_t k = size_t(r.skid - 0x0e);
            vol[k] += wh.slip / float(n);
            const float over = std::max(wh.slipSpeed - bank_->engine.skidPitchRef, 0.0f);
            pitchAdd[k] = std::min(over / (float(n) * 5.0f) + pitchAdd[k], t_.P(r.pitchCapPP) - 1.0f);
        }
        gravelAccum_ += wh.slip * r.gravelRate;
    }
    float total = 0;
    for (int i = 0; i < 17; ++i) {
        const float v = clamp01(vol[size_t(i)]);
        Slot& s = slots_[size_t(12 + i)];
        s.target = clamp01(v * gain);
        total += v;
        s.pitch = std::clamp(pitchAdd[size_t(i)] + 1.0f, 1.0f, 1.3f);
    }
    mix.rec[1].tgt = total > 0 ? 1.0f - (1.0f - t_.P(2221)) * std::min(total, 1.0f) : 1.0f;
}

// 0x5dac50: gravel ticks: the accumulated skid x gravel rate (mean over wheels) runs a timer; past the
// interval a random one of eight one-shots fires if free, at that level; the next interval is
// PP 2152 x (1 - PP 2153 rand).
void VehicleSound::gravel(const VehicleInput& in, float gain) {
    if (gravelAccum_ > 0) {
        gravelAccum_ /= float(bank_->wheels);
        gravelTimer_ += gravelAccum_ * in.dt;
        if (gravelInterval_ < gravelTimer_) {
            Slot& s = slots_[size_t(88 + std::min(int(rand01() * 8.0f), 7))];
            if (s.voice < 0) {
                s.target = clamp01(clamp01(gravelAccum_) * gain);
                gravelTimer_ = 0;
                gravelInterval_ = (1.0f - rand01() * t_.P(2153)) * t_.P(2152);
            }
        }
    }
    gravelAccum_ = 0;
}

// 0x5d9f70: impacts, scrape, landings and the water splash, from the frame's impact vector c
// (phys+0xf2c). An impact sounds when |c| falls after a frame above PP 2001.
void VehicleSound::impacts(const VehicleInput& in, float gain, GameMix& mix) {
    const float len = len3(in.impact);
    impactTimer_ += in.dt;
    // Scrape (0x47): |c| > PP 2003 and the body velocity within 45 deg of c's line: clamp(|v x c^| PP 2004).
    float scrape = 0;
    if (len > t_.P(2003)) {
        const float vl = len3(in.velocity);
        if (vl > 1e-6f) {
            const Vec3f cn{in.impact.x / len, in.impact.y / len, in.impact.z / len};
            const Vec3f vn{in.velocity.x / vl, in.velocity.y / vl, in.velocity.z / vl};
            const float d = std::fabs(vn.x * cn.x + vn.y * cn.y + vn.z * cn.z);
            if (d > 0.707f) {
                const Vec3f& v = in.velocity;
                const Vec3f cr{v.y * cn.z - v.z * cn.y, v.z * cn.x - v.x * cn.z, v.x * cn.y - v.y * cn.x};
                scrape = clamp01(len3(cr) * t_.P(2004)) * gain;
                if (in.impactMeter > 0) rumble("IMPACT_SMALL", 1, 1.0f);  // P89: while scraping with a hit this frame
            }
        }
    }
    setTarget(0x47, scrape);
    if (len < prevImpactLen_ && t_.P(2001) < prevImpactLen_) {
        const Vec3f b = toBody(prevImpact_, in.rotation);
        const float horiz = std::sqrt(b.z * b.z + b.x * b.x), up = std::fabs(b.y);
        if (horiz <= up) {  // landing: 0x48 from below (y <= 0) or 0x4a, clamp(|y| PP 2002)
            mix.rec[0].tgt = t_.P(2220);
            setTarget(b.y <= 0 ? 0x48 : 0x4a, clamp01(up * t_.P(2002)) * gain);
        } else if (impactTimer_ > 0.25f) {  // hits: front (|z|) and side (|x|) by tier PP 2150 / 2151
            impactTimer_ = 0;
            const float fz = std::fabs(b.z) * gain, fx = std::fabs(b.x) * gain;
            const float hit = prevImpactLen_;
            const int tier = hit >= t_.P(2150) ? (hit >= t_.P(2151) ? 2 : 1) : 0;
            static constexpr int front[3] = {0x42, 0x43, 0x44}, side[3] = {0x45, 0x45, 0x46};
            setTarget(front[tier], fz);
            setTarget(side[tier], fx);
            mix.rec[0].tgt = t_.P(2220);
            static constexpr const char* pad[3] = {"IMPACT_SMALL", "IMPACT_MEDIUM", "IMPACT_HIGH"};
            if (in.impactMeter > 0) rumble(pad[tier], tier + 1, 1.0f);  // P89: hits (not landings) with a crash meter
        }
    }
    // Water splash 0x4b: falling faster than PP 2154, faster than PP 2155 and a wheel deeper than
    // PP 2156; latched until every wheel is out (depth <= 0.01).
    const int n = std::min(bank_->wheels, int(in.wheels.size()));
    if (splashLatch_) {
        bool wet = false;
        for (int w = 0; w < n; ++w) wet |= in.wheels[size_t(w)].depth > 0.01f;
        splashLatch_ = wet;
    }
    if (!splashLatch_) {
        const Vec3f& v = in.velocity;
        if (v.y < t_.P(2154) && t_.P(2155) * t_.P(2155) < v.x * v.x + v.y * v.y + v.z * v.z) {
            for (int w = 0; w < n; ++w)
                if (t_.P(2156) < in.wheels[size_t(w)].depth) {
                    setTarget(0x4b, clamp01(gain));  // level: Inferred (the x87 operands are not resolved)
                    splashLatch_ = true;
                }
        }
    }
    prevImpactLen_ = len;
    prevImpact_ = in.impact;
}

// 0x5d9800: the rev limiter hit (phys+0x142a) restarts 0x5d at the motor gain.
void VehicleSound::limiter(const VehicleInput& in, float gain, Mixer& out) {
    if (!in.revLimiter) return;
    Slot* s = bySymbol(0x5d);
    if (!s) return;
    if (s->voice >= 0) out.stop(s->voice);  // 0x5de1c0
    s->voice = -1;
    s->current = 0;
    s->target = clamp01(gain);
}

// 0x5db350: suspension thumps. Per wheel the compression rate (veh+0x15e8 change / dt); when a wheel
// that was compressing slows its compression and its timer passed 1 s: volume (rate - P651) PP 2011 P652,
// one-shot at 4/n x min(volume, 1) x gain. 0x5a (big thump) at the gain when the thumps sum past 0.25.
void VehicleSound::absorbers(const VehicleInput& in, float gain) {
    const int n = bank_->wheels;
    bool any = false;
    for (int w = 0; w < n && w < 6; ++w) {
        absorberTimer_[size_t(w)] += in.dt;
        const float load = w < int(in.wheels.size()) ? in.wheels[size_t(w)].load : 0.0f;
        const float rate = (load - prevLoad_[size_t(w)]) * (1.0f / in.dt);
        prevLoad_[size_t(w)] = load;
        const float prev = absorberRate_[size_t(w)];
        if (bank_->engine.absorberRumbleThreshold < std::fabs(rate)) {  // P89: ABSORBER_<w+1> on a hard compression change
            const float v = (std::fabs(rate) - bank_->engine.absorberRumbleThreshold) * bank_->engine.absorberRumbleScale;
            static constexpr const char* name[6] = {"ABSORBER_1", "ABSORBER_2", "ABSORBER_3", "ABSORBER_4", "ABSORBER_5", "ABSORBER_6"};
            rumble(name[w], 1, float(std::clamp(v, 0.0f, 1.0f) * double(1.0f / float(n))));
        }
        if (prev > 0 && std::fabs(rate) < std::fabs(prev) && absorberTimer_[size_t(w)] > 1.0f) {
            const float v = (std::fabs(prev) - bank_->engine.absorberThreshold) * t_.P(2011) * bank_->engine.absorberScale;
            if (v > 0) {
                slots_[size_t(73 + w)].target = clamp01((1.0f / float(n)) * 4.0f * std::min(v, 1.0f) * gain);
                any = true;
                absorberTimer_[size_t(w)] = 0;
            }
        }
        absorberRate_[size_t(w)] = rate;
    }
    Slot& big = slots_[79];
    big.target = 0;
    if (any) {
        float sum = 0;
        for (int w = 0; w < n && w < 6; ++w) sum += slots_[size_t(73 + w)].target;
        if (sum > 0.25f) big.target = clamp01(gain);
    }
}

// P89: Game 0x557b40 -> VibrationManager_C 0x4e4530 (the player's pad; the manager applies the option and gates).
void VehicleSound::rumble(const char* name, int priority, float scale) {
    if (vib_ && player_) vib_->add(name, priority, scale);
}

// 0x5ddfc0: fade toward the target, start / stop the voice, set its pitch and volume.
void VehicleSound::process(Slot& s, float dt, const VehicleInput& in, Mixer& out) {
    if (s.symbol <= 0 || s.symbol >= int(bank_->bySymbol.size())) return;
    const std::shared_ptr<const Pcm>& pcm = bank_->bySymbol[size_t(s.symbol)];
    if (!pcm) return;
    const float before = s.current;
    if (s.current < s.target) s.current = s.fadeIn > 0 ? approach(s.current, s.target, s.fadeIn, dt) : s.target;
    else if (s.target < s.current) s.current = s.fadeOut > 0 ? approach(s.current, s.target, s.fadeOut, dt) : s.target;
    const bool alive = s.voice >= 0 && out.alive(s.voice);
    if (!alive) s.voice = -1;
    if (s.current <= 0) {
        if (alive) {
            out.stop(s.voice);
            s.voice = -1;
            s.current = s.target = 0;
        }
        return;
    }
    if (before == 0 || pcm->loops) {
        if (!alive) {
            Mixer::PlayParams p;
            p.minDist = 50, p.maxDist = 350;  // the vehicle owner's +0x88 / +0x8c (0x5d8a40)
            p.pos = in.position;
            if (&s - slots_.data() < 12) p.toneDb = toneDb_, p.shelfDb = shelfDb_;  // [upgrades U5] the engine layers
            s.voice = out.play(pcm, p);
        }
    } else if (!alive) {  // a one-shot that has played
        s.current = s.target = 0;
        return;
    }
    if (s.voice < 0) return;
    out.setPosition(s.voice, in.position);
    out.setPitch(s.voice, s.pitch);
    out.setVolume(s.voice, s.current);
}

void VehicleSound::update(const VehicleInput& in, GameMix& mix, Mixer& out) {
    if (!started_) {  // 0x5d8a40 runs on the first frame
        prevRpm_ = in.soundRpm;
        for (size_t w = 0; w < prevLoad_.size() && w < in.wheels.size(); ++w) prevLoad_[w] = in.wheels[w].load;
        started_ = true;
    }
    const float dt = std::max(in.dt, 1e-4f);
    VehicleInput step = in;
    step.dt = dt;
    // Engine: every vehicle; the others at 0.8 (0x5d8ed0).
    motor(step, mix.motorGain(player_) * (player_ ? 1.0f : 0.8f));
    for (int i = 0; i < 12; ++i) process(slots_[size_t(i)], dt, step, out);
    if (!player_) return;
    // The player's fader targets (0x5d8ed0): fader 0 back to 1; faders 2 and 5 to PP 2223 / 2224 while
    // no wheel touches the ground (0x52e620); fader 3 to PP 2225 when 0x4e6f80 holds (Unknown; port: never).
    bool ground = false;
    for (const VehicleInput::Wheel& w : step.wheels) ground |= w.load > 1e-6f;
    mix.rec[0].tgt = 1;
    mix.rec[2].tgt = ground ? 1.0f : t_.P(2223);
    mix.rec[5].tgt = ground ? 1.0f : t_.P(2224);
    mix.rec[3].tgt = 1;
    bang(step, mix.gain(GameMix::Bang));
    for (int i = 80; i < 84; ++i) process(slots_[size_t(i)], dt, step, out);
    rolling(step, mix.gain(GameMix::Rolling));
    for (int i = 29; i < 63; ++i) process(slots_[size_t(i)], dt, step, out);
    skid(step, mix.gain(GameMix::Skid), mix);
    for (int i = 12; i < 29; ++i) process(slots_[size_t(i)], dt, step, out);
    gravel(step, mix.gain(GameMix::Gravel));
    for (int i = 88; i < 96; ++i) process(slots_[size_t(i)], dt, step, out);
    impacts(step, mix.gain(GameMix::Impact), mix);
    for (int i = 63; i < 73; ++i) process(slots_[size_t(i)], dt, step, out);
    limiter(step, mix.gain(GameMix::Motor), out);
    for (int i = 84; i < 87; ++i) process(slots_[size_t(i)], dt, step, out);
    absorbers(step, mix.gain(GameMix::Absorber));
    for (int i = 73; i < 80; ++i) process(slots_[size_t(i)], dt, step, out);
}

void VehicleSound::stop(Mixer& out) {
    for (Slot& s : slots_) {
        if (s.voice >= 0) out.stop(s.voice);
        s.voice = -1;
        s.current = s.target = 0;
    }
}

}  // namespace fuel::snd
