// FUEL's world ambience (Game_SoundMgr emitters, 0x4e81e0). See ambience.hpp and notes/p45-audio-ambience.md.
#include "ambience.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::snd {

namespace {
float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
}  // namespace

float dayFactor(double t) {  // 0x5b98d0
    if (t > 19800.0) {
        if (t < 23400.0) return float((t - 19800.0) * 0.00027777778);
        if (t <= 63000.0) return 1.0f;
        if (t <= 66600.0) return float(1.0 - (t - 63000.0) * 0.00027777778);
    }
    return 0.0f;
}

float approach(float cur, float tgt, float rate, float dt) {  // 0x4e9850
    if (std::fabs(tgt - cur) < 1e-6f) return tgt;
    if (tgt < cur) {
        const float v = cur - rate * dt;
        return tgt <= v ? v : tgt;
    }
    const float v = rate * dt + cur;
    return v <= tgt ? v : tgt;
}

Ambience::Ambience(std::array<std::shared_ptr<const Pcm>, 0x100> bank, const Tuning& t, std::function<float()> rand01)
    : bank_(std::move(bank)), t_(t), rnd_(std::move(rand01)) {
    // 0x4e80e0: symbols and rates.
    for (int i = 0; i < 4; ++i) wind[size_t(i)].symbol = 0x6a + i, wind[size_t(i)].up = wind[size_t(i)].down = 1;
    day.symbol = 0x73, night.symbol = 0x74;
    day.up = day.down = night.up = night.down = 1;
    for (int i = 0; i < 9; ++i) {
        animals[size_t(i)].frog = i >= 5;
        animals[size_t(i)].symbol = 0x75 + i;
    }
}

bool Ambience::playing(int slot, int serial, const Mixer& out) const {  // 0x4ead30 + 0x4eac60
    if (slot < 0 || serial == -1 || size_t(slot) >= emitters.size()) return false;
    const Emitter& e = emitters[size_t(slot)];
    return e.serial == serial && e.voice >= 0 && out.alive(e.voice);
}

bool Ambience::start(int symbol, const Mixer::PlayParams& p, float volume, Mixer& out, int& slot, int& serial) {  // 0x4e9da0
    if (symbol < 0 || symbol >= 0x100 || !bank_[size_t(symbol)]) return false;
    for (size_t i = 0; i < emitters.size(); ++i) {
        Emitter& e = emitters[i];
        if (e.voice >= 0 && out.alive(e.voice)) continue;  // voice handle valid: emitter busy
        e.voice = out.play(bank_[size_t(symbol)], p);
        if (e.voice < 0) return false;
        out.setVolume(e.voice, volume);
        e.volume = volume;
        e.serial = nextSerial_++;
        slot = int(i), serial = e.serial;
        return true;
    }
    return false;
}

void Ambience::stop(int& slot, int& serial, Mixer& out) {  // 0x4e9e80
    if (playing(slot, serial, out)) {
        Emitter& e = emitters[size_t(slot)];
        out.stop(e.voice);
        e.voice = -1, e.serial = -1;
    }
    slot = serial = -1;
}

void Ambience::stopAll(Mixer& out) {
    for (Emitter& e : emitters) {
        if (e.voice >= 0) out.stop(e.voice);
        e = Emitter{};
    }
    for (Bed* b : {&wind[0], &wind[1], &wind[2], &wind[3], &day, &night}) b->cur = b->tgt = 0, b->slot = b->serial = -1;
    for (Animal& a : animals) a.active = false, a.timer = -1, a.slot = a.serial = -1;
    for (int k = 0; k < 2; ++k) rainSlots[k] = rainSerial[k] = -1;
    thunderSlot = thunderSerial = -1;
    for (auto& [k, d] : decorLive_)
        if (d.voice >= 0) out.stop(d.voice);
    decorLive_.clear();
}

float Ambience::gain(const GameMix& mix, int cat, int pp, float v, bool player) const {  // 0x4e7820 / 0x4e78c0
    const float base = mix.master * mix.cat[GameMix::Amb0];
    float duck = 1;
    if (player && v > 1.0f) {
        const float P = t_.P(pp);
        duck = v <= 30.0f ? 1 - (1 - P) * (1 - (30.0f - v) * 0.03448276f) : P;
    }
    return duck * mix.cat[size_t(cat)] * base;
}

void Ambience::updateBed(Bed& b, float dt, Mixer& out) {  // 0x4e79d0
    if (b.tgt > b.cur) b.cur = b.up > 0 ? approach(b.cur, b.tgt, b.up, dt) : b.tgt;
    else if (b.tgt < b.cur) b.cur = b.down > 0 ? approach(b.cur, b.tgt, b.down, dt) : b.tgt;
    if (b.cur <= 0) {
        if (playing(b.slot, b.serial, out)) {
            b.cur = b.tgt = 0;
            stop(b.slot, b.serial, out);
        }
        return;
    }
    if (!playing(b.slot, b.serial, out)) {
        Mixer::PlayParams p;
        p.positional = false, p.attenuate = false;  // flags 0x40
        start(b.symbol, p, b.cur, out, b.slot, b.serial);
    }
    if (playing(b.slot, b.serial, out)) {
        Emitter& e = emitters[size_t(b.slot)];
        e.volume = b.cur;
        out.setVolume(e.voice, b.cur);
    }
}

void Ambience::updateAnimal(Animal& a, float dt, float density, float camY, const AmbientInput& in, float w, Mixer& out) {
    if (!a.frog) {  // BirdAnimal vt[0] 0x4e7c80
        if (!(camY <= 2000.0f) || !(density > 0.1f) || !(dayFactor(in.timeOfDay) >= 0.1f)) return;
        if (a.timer == -1.0f) {
            if (in.cloud < 0.99f && w <= 0.6f && !in.mission) a.timer = (in.rain * 10.0f + 5.0f) * rnd_() * density + 1.0f;
            else a.timer = 10000.0f;  // [0xa2a4e0]
            return;
        }
    } else {  // FrogAnimal vt[0] 0x4e7f00
        if (!(density > 0.1f) || !(camY <= 1500.0f) || in.rain == 0.0f) return;
        if (a.timer == -1.0f) {
            a.timer = 1.0f + ((1.0f - in.rain) * 10.0f + 5.0f) * rnd_() * density;
            return;
        }
    }
    a.timer -= dt;
    if (a.timer >= 0) return;
    const Vec3f base = in.player ? in.playerPos : Vec3f{};
    a.pos = {(rnd_() - 0.5f) * 50.0f + base.x, base.y, 0};
    a.pos.z = (rnd_() - 0.5f) * 50.0f + base.z;
    a.playFor = 5.0f;
    Mixer::PlayParams p;
    p.pos = a.pos, p.minDist = 0, p.maxDist = a.frog ? 100.0f : 50.0f;
    start(a.symbol, p, gainB, out, a.slot, a.serial);
    a.active = true;
}

void Ambience::update(float dt, const AmbientInput& in, const GameMix& mix, Mixer& out) {
    if (in.paused) return;
    gainA = gain(mix, GameMix::Amb3, 2201, in.playerSpeed, in.player);
    gainB = gain(mix, GameMix::Amb5, 2203, in.playerSpeed, in.player);
    const float y = in.camera.y;
    float alt = 0;
    if (2500.0f < y) alt = std::min((y - 2500.0f) * 0.0005f, 1.0f);
    for (Bed& b : wind) b.tgt = 0;
    const float wl = std::sqrt(in.wind[0] * in.wind[0] + in.wind[1] * in.wind[1] + in.wind[2] * in.wind[2]);
    const float pp5460 = t_.P(5460);
    float w = pp5460 != 0 ? wl / pp5460 : 0.0f;
    w = w > 0.1f ? std::min(w, 1.0f) : 0.1f;
    if (w < 0.5f) {
        wind[0].tgt = clamp01((1 - 2 * w) * gainA);
        wind[1].tgt = clamp01(2 * w * gainA);
    } else {
        const float k = (w - 0.5f) * 2;
        wind[1].tgt = clamp01((1 - k) * gainA);
        wind[2].tgt = clamp01(k * gainA);
    }
    wind[3].tgt = clamp01(gainA * alt);
    for (Bed& b : wind) updateBed(b, dt, out);
    for (Animal& a : animals) {
        if (!a.active) {
            updateAnimal(a, dt, in.forest, y, in, w, out);
        } else {
            a.playFor -= dt;
            if (a.playFor < 0) {
                stop(a.slot, a.serial, out);
                a.timer = -1;
                a.active = false;
            }
        }
    }
    const float d = dayFactor(in.timeOfDay);
    const float wet = 1 - in.rain * 10.0f;
    float c4 = clamp01((1 - in.forest) * wet * d * in.patch);
    float c8 = clamp01(in.patch * (1 - d) * (1 - in.forest) * wet);
    if (alt > 0 || in.snow > 0) c4 = c8 = 0;
    day.tgt = clamp01(gainB * c4);
    updateBed(day, dt, out);
    night.tgt = clamp01(gainB * c8);
    updateBed(night, dt, out);
    updateRain(in, mix, out);          // 0x5e10e0
    updateThunder(dt, in, mix, out);   // 0x5e32a0
    updateDecor(in, out);
}

void Ambience::updateRain(const AmbientInput& in, const GameMix& mix, Mixer& out) {  // SpecialEffectsSplashTank_G 0x5e10e0
    const int drawn = int(std::floor(in.rain * 1024.0f));  // the tank's amount: floor(amount x 1024) (0x5de850)
    if (drawn == 0) {
        for (int k = 0; k < 2; ++k) stop(rainSlots[k], rainSerial[k], out);
        return;
    }
    const float G = gain(mix, GameMix::Amb2, 2200, in.playerSpeed, in.player);  // 0x5e09d0
    const float f = float(drawn) / 1024.0f;
    const float vol[2] = {(1 - f) * G * 0.5f, G * f * 0.5f};
    for (int k = 0; k < 2; ++k) {
        if (!playing(rainSlots[k], rainSerial[k], out)) {
            Mixer::PlayParams p;
            p.positional = false, p.attenuate = false;  // flags 0x40
            start(0x68 + k, p, vol[k], out, rainSlots[k], rainSerial[k]);
        }
    }
    if (playing(rainSlots[0], rainSerial[0], out) && playing(rainSlots[1], rainSerial[1], out))
        for (int k = 0; k < 2; ++k) {
            Emitter& e = emitters[size_t(rainSlots[k])];
            e.volume = vol[k];
            out.setVolume(e.voice, vol[k]);
        }
}

void Ambience::updateThunder(float dt, const AmbientInput& in, const GameMix& mix, Mixer& out) {  // SpecialEffectsThunder_G 0x5e32a0
    if (!(0.95f <= in.cloud)) {
        stop(thunderSlot, thunderSerial, out);
        return;
    }
    const float G = gain(mix, GameMix::Amb1, 2202, in.playerSpeed, in.player);  // 0x5e2b90
    const float amount = std::clamp((in.cloud - 0.95f) * 2.0f, 0.0f, 1.0f);
    if (!playing(thunderSlot, thunderSerial, out)) {
        Mixer::PlayParams p;
        p.positional = false, p.attenuate = false;  // flags 0x40
        start(0x67, p, G * amount, out, thunderSlot, thunderSerial);
    }
    if (playing(thunderSlot, thunderSerial, out)) {
        Emitter& e = emitters[size_t(thunderSlot)];
        e.volume = G * amount;
        out.setVolume(e.voice, e.volume);
    }
    if (0.0f < thunderTimer) {
        thunderTimer -= dt;
        return;
    }
    const float lo = t_.P(5230), hi = t_.P(5231);
    thunderTimer = rnd_() * (hi - lo) + lo;
    // Bolt 0x5e2d40: a point on a ring around the tank centre (the camera), rotated 45 degrees.
    auto bolt = [&] {
        const float a = rnd_() * 6.2831855f;
        const float r = t_.P(5234) + rnd_() * (t_.P(5235) - t_.P(5234));
        Vec3f p{(std::cos(a) - std::sin(a)) * r + in.camera.x, in.camera.y, (std::sin(a) + std::cos(a)) * r + in.camera.z};
        if (ground_) p.y = ground_(p.x, p.z);
        lastStrike = p;
        ++strikes;
        if (onLightning) onLightning(p);
    };
    bolt();
    if (rnd_() < t_.P(5246)) bolt();  // WELL512 U(0, 1) < PP5246
    const int symbol = 0x63 + int(std::floor(rnd_() * 3.9999f));
    lastThunderSymbol = symbol;
    Mixer::PlayParams p;
    p.pos = lastStrike, p.minDist = 0, p.maxDist = 30000.0f;  // descriptor: position, 0 / 30000 m, flags 0x4f
    int slot = -1, serial = -1;
    start(symbol, p, G, out, slot, serial);
}

void Ambience::updateDecor(const AmbientInput& in, Mixer& out) {
    ++frame_;
    for (auto& [k, d] : decorLive_) d.seen = false;
    for (const AmbientInput::Decor& s : in.decor) {
        const auto bank = decorBank_.find(s.kind);
        if (bank == decorBank_.end()) continue;
        const uint64_t key = (uint64_t(uint32_t(std::lround(s.pos.x * 4.0f))) * 0x9e3779b1u) ^ (uint64_t(uint32_t(std::lround(s.pos.z * 4.0f))) << 32) ^ s.kind;
        DecorVoice& d = decorLive_[key];
        d.seen = true, d.kind = s.kind, d.pos = s.pos;
        if (((key ^ (key >> 29)) & 3) != (frame_ & 3)) continue;  // a quarter of the list per frame (0x4e9b70)
        const float dx = in.camera.x - s.pos.x, dy = in.camera.y - s.pos.y, dz = in.camera.z - s.pos.z;
        const bool near = dx * dx + dy * dy + dz * dz <= 50.0f * 50.0f;  // node +0x8c = 50 m
        const bool live = d.voice >= 0 && out.alive(d.voice);
        if (!near) {
            if (live) out.stop(d.voice);
            d.voice = -1;
        } else if (!live) {
            Mixer::PlayParams p;  // flags 0xf: positional, distance gain; min 5 / max 50 (0x516cb0)
            p.pos = s.pos, p.minDist = 5.0f, p.maxDist = 50.0f;
            d.voice = out.play(bank->second, p);
        }
    }
    for (auto it = decorLive_.begin(); it != decorLive_.end();) {  // the object is gone (0x516db0 -> 0x4e9f10)
        if (!it->second.seen) {
            if (it->second.voice >= 0) out.stop(it->second.voice);
            it = decorLive_.erase(it);
        } else {
            ++it;
        }
    }
}

void Ambience::teleportQueued(Mixer& out) {  // 0x57e650 (in game): symbol 0xfa, not positional
    Mixer::PlayParams p;
    p.positional = false, p.attenuate = false;
    int slot = -1, serial = -1;
    start(0xfa, p, 1.0f, out, slot, serial);
}

void Ambience::countdown(int n, Mixer& out) {  // MissionHUD 0x4b3da0
    if (n < 0) {
        lastCount_ = -1;
        return;
    }
    if (n == lastCount_) return;
    lastCount_ = n;
    Mixer::PlayParams p;
    p.positional = false, p.attenuate = false;  // descriptor flags 0x40
    int slot = -1, serial = -1;
    start(n > 0 ? 0xf7 : 0xf8, p, 1.0f, out, slot, serial);
}

}  // namespace fuel::snd
