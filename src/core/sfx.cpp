// FUEL's game special effects; see sfx.hpp and notes p30-effects.md for the decode.
#include "sfx.hpp"

#include "genworld_noise.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace fuel::sfx {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kK = -2.0f;  // .data 0xa2a56c (-2.0): the deceleration factor of the wheel particles (read below)

float len3(const V3& v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
V3 sub3(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 add3(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 mul3(const V3& a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
float lerpf(float a, float b, float t) { return (b - a) * t + a; }
// The wheel code's clamp: below lo -> lo, at or above hi -> hi.
float clampW(float x, float lo, float hi) {
    float r = lo < x ? x : lo;
    if (hi <= x) r = hi;
    return x < lo ? lo : r;
}

// ---- the FX atlas: replay of the static initialiser 0x8e6ae0 (straight-line x87 stores) ----
constexpr uint32_t kAtlasInit = 0x8e6ae0, kAtlasInitEnd = 0x8e7740;  // ret at 0x8e771b
constexpr uint32_t kAtlasTable = 0xa66c24, kAtlasStride = 0x1c;
constexpr int kAtlasCount = 51;

std::vector<uint8_t> exeBytes(const std::string& exe, uint32_t va, size_t n) {
    const std::vector<float> f = readEngineFloats(exe, va, (n + 3) / 4);
    std::vector<uint8_t> b(f.size() * 4);
    if (!f.empty()) std::memcpy(b.data(), f.data(), b.size());
    return b;
}

}  // namespace

std::vector<AtlasEntry> replayAtlas(const std::string& exe, std::string* error) {
    std::vector<AtlasEntry> out;
    const std::vector<uint8_t> code = exeBytes(exe, kAtlasInit, kAtlasInitEnd - kAtlasInit);
    if (code.size() < kAtlasInitEnd - kAtlasInit) {
        if (error) *error = "cannot read the FX atlas initialiser from " + exe;
        return out;
    }
    auto u32 = [&](size_t o) { uint32_t v; std::memcpy(&v, &code[o], 4); return v; };
    std::vector<double> st;  // x87 stack, st[back] = st(0)
    struct Val { bool isFloat = false; double f = 0; uint32_t u = 0; std::string name; };
    std::vector<std::pair<uint32_t, Val>> mem;
    auto store = [&](uint32_t a, const Val& v) {
        for (auto& m : mem)
            if (m.first == a) { m.second = v; return; }
        mem.emplace_back(a, v);
    };
    Val eax, esi;
    uint32_t edx = 0;
    size_t ip = 0;
    try {
        for (int steps = 0; steps < 4000; ++steps) {
            if (ip + 12 > code.size()) throw std::runtime_error("ran off the initialiser");
            const uint8_t b0 = code[ip], b1 = code[ip + 1];
            if (b0 == 0x56 || b0 == 0x5e) { ++ip; continue; }  // push / pop esi
            if (b0 == 0xc3) break;                             // ret
            if (b0 == 0x33 && b1 == 0xc0) { eax = Val{}; ip += 2; continue; }
            if (b0 == 0x33 && b1 == 0xf6) { esi = Val{}; ip += 2; continue; }
            if (b0 == 0xba) { edx = u32(ip + 1); ip += 5; continue; }  // mov edx, string
            if (b0 == 0xe8) {                                          // call 0x669160 (name CRC): keep the name
                const uint32_t target = uint32_t(kAtlasInit + ip + 5 + u32(ip + 1));
                if (target != 0x669160) throw std::runtime_error("unexpected call");
                const std::vector<uint8_t> s = exeBytes(exe, edx, 64);
                eax = Val{};
                for (uint8_t c : s) {
                    if (c == 0) break;
                    eax.name.push_back(char(c));
                }
                ip += 5;
                continue;
            }
            if (b0 == 0xa3) { store(u32(ip + 1), eax); ip += 5; continue; }
            if (b0 == 0x89 && b1 == 0x35) { store(u32(ip + 2), esi); ip += 6; continue; }
            if (b0 == 0xc7 && b1 == 0x05) { Val v; v.u = u32(ip + 6); store(u32(ip + 2), v); ip += 10; continue; }
            if (b0 == 0xd9) {
                if (b1 == 0xee) { st.push_back(0.0); ip += 2; continue; }
                if (b1 == 0xe8) { st.push_back(1.0); ip += 2; continue; }
                if (b1 >= 0xc8 && b1 <= 0xcf) {
                    const size_t i = b1 - 0xc8;
                    if (i >= st.size()) throw std::runtime_error("fxch underflow");
                    std::swap(st.back(), st[st.size() - 1 - i]);
                    ip += 2;
                    continue;
                }
                const uint32_t a = u32(ip + 2);
                if (b1 == 0x05) {
                    const std::vector<float> f = readEngineFloats(exe, a, 1);
                    if (f.empty()) throw std::runtime_error("bad fld");
                    st.push_back(f[0]);
                    ip += 6;
                    continue;
                }
                if (b1 == 0x15 || b1 == 0x1d) {
                    if (st.empty()) throw std::runtime_error("fst underflow");
                    Val v;
                    v.isFloat = true, v.f = st.back();
                    store(a, v);
                    if (b1 == 0x1d) st.pop_back();
                    ip += 6;
                    continue;
                }
            }
            char m[96];
            std::snprintf(m, sizeof m, "unknown instruction %02x %02x at 0x%x", b0, b1, unsigned(kAtlasInit + ip));
            throw std::runtime_error(m);
        }
        auto get = [&](uint32_t a) -> const Val* {
            for (auto& m : mem)
                if (m.first == a) return &m.second;
            return nullptr;
        };
        for (int i = 0; i < kAtlasCount; ++i) {
            const uint32_t b = kAtlasTable + kAtlasStride * uint32_t(i);
            AtlasEntry e;
            const Val* n = get(b);
            const Val *u0 = get(b + 8), *v0 = get(b + 12), *du = get(b + 16), *dv = get(b + 20);
            if (!n || n->name.empty() || !u0 || !v0 || !du || !dv) throw std::runtime_error("incomplete atlas entry");
            e.material = n->name;
            e.u0 = float(u0->f), e.v0 = float(v0->f), e.du = float(du->f), e.dv = float(dv->f);
            out.push_back(e);
        }
    } catch (const std::exception& ex) {
        out.clear();
        if (error) *error = std::string("FX atlas from FUEL.exe: ") + ex.what();
    }
    return out;
}

Tables loadTables(const std::string& exe, const std::string& pp001) {
    Tables t;
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
        std::string tag;
        int n = 0;
        float v = 0;
        if (ls >> tag >> n >> v && tag == "PP" && n >= 0 && n < 6000) t.pp[size_t(n)] = v, ++rows;
    }
    if (t.P(5070) <= 0 || t.P(5071) <= 0 || rows < 1000) {
        t.error = "pp001.tsc has no wheel effect rows";
        return t;
    }
    std::string err;
    t.atlas = replayAtlas(exe, &err);
    if (t.atlas.size() != size_t(kAtlasCount)) {
        t.error = err.empty() ? "FX atlas incomplete" : err;
        return t;
    }
    const std::vector<float> seed = readEngineFloats(exe, 0xa02260, 16);
    const std::vector<float> k = readEngineFloats(exe, 0xa2a56c, 1);
    if (seed.size() != 16 || k.empty() || k[0] != kK) {
        t.error = "FUEL.exe is not the version the effects know (seed / 0xa2a56c)";
        return t;
    }
    std::memcpy(t.wellSeed.data(), seed.data(), 64);
    t.ok = true;
    return t;
}

// ---- WELL512 (0x5fd710) ----
uint32_t Well512::next() {
    const uint32_t a = s_[i_], c = s_[(i_ - 3) & 15], d7 = s_[(i_ - 7) & 15];
    const uint32_t b = ((a * 2u ^ c) << 15) ^ c ^ a;
    const uint32_t e = d7 ^ (d7 >> 11);
    const uint32_t n = b ^ e;
    s_[i_] = n;
    i_ = (i_ - 1) & 15;
    const uint32_t z = s_[i_];
    s_[i_] = ((((e << 10) ^ b) << 16 ^ z) * 4u) ^ ((n & 0xfed22169u) << 5) ^ n ^ b ^ z;
    return s_[i_];
}

float Well512::uniform() {
    const uint32_t bits = (next() & 0x7fffffu) | 0x3f800000u;
    float f;
    std::memcpy(&f, &bits, 4);
    return f - 1.0f;
}

// ---- fades ----
uint16_t fadeCode(Fade f) {
    switch (f) {
    case Fade::One: return 0x000;
    case Fade::OneMinusT: return 0x101;
    case Fade::OneMinusTSq: return 0x301;
    case Fade::SqrtT: return 0x401;
    case Fade::Quarter: return 0x003;
    case Fade::SqrtQuarter: return 0x403;
    case Fade::LogPuff: return 0x005;
    }
    return 0;
}

float fadeValue(Fade f, float age, float life) {
    const bool valid = life > 1e-6f;
    const float t = valid ? age / life : 1.0f;
    switch (f) {
    case Fade::One: return 1.0f;
    case Fade::OneMinusT: return 1.0f - t;
    case Fade::OneMinusTSq: return (1.0f - t) * (1.0f - t);
    case Fade::SqrtT: return std::sqrt(std::max(t, 0.0f));
    case Fade::Quarter: return t * 0.75f + 0.25f;
    case Fade::SqrtQuarter: return std::sqrt(t * 0.75f + 0.25f);
    case Fade::LogPuff: {
        if (!valid) return 1.0f;
        const float l = std::log(t * 2.35f + 0.38f);
        return 1.0f - l * l;
    }
    }
    return 1.0f;
}

std::array<float, 4> codeParams(uint16_t code) {
    float c1 = 0, t = 0, sn = 0, q = 0, qs = 0, inv = 0;
    switch (code & 0xff) {
    case 0: c1 = 1; break;
    case 1: t = 1; break;
    case 2: sn = 1; break;
    case 3: q = 1; break;
    case 4: qs = 1; break;
    default: break;
    }
    if (code & 0x100) inv = 1;
    float pw = 1.0f;
    if (code & 0x200) pw = (code & 0x800) ? 4.0f : 2.0f;
    else if (code & 0x400) pw = (code & 0x800) ? 0.25f : 0.5f;
    return {c1 * 100.0f + t * 0.01f, sn * 100.0f + q * 0.01f, inv * 0.01f + qs * 100.0f, 0.0f + pw * 100.0f};
}

float parameterValue(const std::array<float, 4>& P, float life) {
    float lo[4], hi[4];
    for (int i = 0; i < 4; ++i) {
        const float fr = P[size_t(i)] - std::floor(P[size_t(i)]);
        hi[i] = (P[size_t(i)] - fr) / 100.0f;
        lo[i] = fr * 100.0f;
    }
    const float sl = std::sin(life * kPi);  // C_SINCOS1.x = pi (I: sin(pi t), p26 2.5)
    float v = hi[0] + lo[0] * life + hi[1] * sl + lo[1] * (0.25f + 0.75f * life) + hi[2] * (0.25f + 0.75f * sl);
    v = v + lo[2] * ((1.0f - v) - v);
    return std::pow(v, hi[3]);
}

std::array<float, 4> rgbToHsv(const std::array<float, 4>& c) {  // 0x5f4be0
    const float mx = std::max(std::max(c[0], c[1]), c[2]);
    const float mn = std::min(std::min(c[0], c[1]), c[2]);
    std::array<float, 4> o{0, 0, mx, c[3]};
    o[1] = mx > 0 ? (mx - mn) / mx : 0.0f;
    if (o[1] == 0) return o;
    const float d = mx - mn;
    if (mx == c[0]) o[0] = (c[1] - c[2]) / d;
    else if (mx == c[1]) o[0] = (c[2] - c[0]) / d + 2.0f;
    else o[0] = (c[0] - c[1]) / d + 4.0f;
    if (o[0] < 0) o[0] += 6.0f;
    return o;
}

std::array<float, 4> hsvToRgb(const std::array<float, 4>& h) {  // 0x5f4a90
    if (h[1] == 0) return {h[2], h[2], h[2], h[3]};
    float hh = h[0] == 6.0f ? 0.0f : h[0];
    const int i = int(std::floor(hh));  // ftol under _RC_DOWN
    const float f = hh - float(i);
    const float p = (1 - h[1]) * h[2], q = (1 - h[1] * f) * h[2], t = (1 - (1 - f) * h[1]) * h[2];
    switch (i) {
    case 0: return {h[2], t, p, h[3]};
    case 1: return {q, h[2], p, h[3]};
    case 2: return {p, h[2], t, h[3]};
    case 3: return {p, q, h[2], h[3]};
    case 4: return {t, p, h[2], h[3]};
    case 5: return {h[2], p, q, h[3]};
    default: return {0, 0, 0, h[3]};  // the engine leaves the output untouched
    }
}

// ---- emitter setters (0x4c4b50 & co.: set the range, redraw the current value) ----
namespace {
void setRange(Well512& r, Emitter::R& x, float lo, float hi) {
    x.lo = lo, x.hi = hi;
    x.cur = r.uniform() * (hi - lo) + lo;
}
void setRange3One(Well512& r, Emitter::R3& x, const V3& lo, const V3& hi) {  // 0x4c4b10 + 0x4c4cf0: one random
    x.lo = lo, x.hi = hi;
    const float u = r.uniform();
    for (int k = 0; k < 3; ++k) x.cur[size_t(k)] = x.lo[size_t(k)] + (x.hi[size_t(k)] - x.lo[size_t(k)]) * u;
}
void redraw3(Well512& r, Emitter::R3& x) {  // per axis (0x4c4da0 / 0x4c5eb0 / 0x4c5f80)
    for (int k = 0; k < 3; ++k) x.cur[size_t(k)] = x.lo[size_t(k)] + r.uniform() * (x.hi[size_t(k)] - x.lo[size_t(k)]);
}
void setColours(Well512& r, Emitter& e, const C4& a, const C4& b) {  // 0x4c4bf0 + 0x4c4f70
    e.colourA = a, e.colourB = b;
    const float u = r.uniform();
    for (int k = 0; k < 4; ++k) e.colour[size_t(k)] = (b[size_t(k)] - a[size_t(k)]) * u + a[size_t(k)];
}
Emitter makeEmitter(Well512& r) {  // 0x4c4730
    Emitter e;
    setRange(r, e.rotSpeed, 0, 0);
    setRange(r, e.angle, 0, 0);
    setRange(r, e.spacing, 1, 1);
    e.interval.lo = e.interval.hi = 1;
    e.interval.cur = r.range(1, 1);
    setRange(r, e.size, 0, 0);
    setRange(r, e.life, 1, 1);
    r.uniform();  // +0x15c [0.02, 0.035]
    redraw3(r, e.vel);
    redraw3(r, e.accel);
    redraw3(r, e.offset);
    return e;
}
}  // namespace

// ---- System ----
System::System(const Tables& t) : t_(t), rng_(t.wellSeed) {}

void System::initParticle(Emitter& e, Particle& p) {  // 0x4c5a90
    p.spawnTime = clock_;
    p.alphaFn = e.alphaFn, p.sizeFn = e.sizeFn;
    p.killFlags = e.killFlags, p.motion = e.motion;
    p.pos = add3(e.pos, e.offset.cur);
    p.start = (e.motion == 3 || e.motion == 4) ? p.pos : e.offset.cur;
    p.angle = e.angle.cur, p.rotSpeed = e.rotSpeed.cur;
    p.accel = e.accel.cur, p.vel = e.vel.cur;
    p.life = e.life.cur, p.size = e.size.cur;
    p.age = 0;
    p.colour = e.colour;
    p.gpu = e.gpuStatic;
    const AtlasEntry& a = t_.atlas[size_t(std::clamp(e.atlas, 0, int(t_.atlas.size()) - 1))];
    float uA = a.u0, uB = a.du + a.u0;  // particle +0x5c = 1, +0x60 = 0 (not set by the init: I)
    if ((e.uvFlags & 0x10) && int32_t(rng_.next()) < 0) std::swap(uA, uB);
    p.uv[0] = uA, p.uv[1] = a.v0, p.uv[2] = uB, p.uv[3] = a.v0 + a.dv;
    p.drawAlpha = 0, p.drawSize = 0;  // the alpha byte is first written by the particle step
}

int System::emit(Emitter& e) {  // 0x4c56e0
    int index = 0;
    for (int n = 0; n < e.perEmission; ++n) {
        Particle p;
        initParticle(e, p);
        if (e.gpuStatic) {
            index = int(e.live.size());  // allocated then freed at once (0x4c6160 / 0x4c60d0)
            gpu_.push_back(p);
        } else {
            e.live.push_back(p);
            index = int(e.live.size()) - 1;
        }
        e.lastEmit = e.pos;
        e.burstDone = true;
        e.interval.cur = rng_.range(e.interval.lo, e.interval.hi);
        redraw3(rng_, e.accel);
        e.size.cur = rng_.range(e.size.lo, e.size.hi);
        e.angle.cur = rng_.range(e.angle.lo, e.angle.hi);
        e.rotSpeed.cur = rng_.range(e.rotSpeed.lo, e.rotSpeed.hi);
        const float u = rng_.uniform();
        for (int k = 0; k < 4; ++k) e.colour[size_t(k)] = (e.colourB[size_t(k)] - e.colourA[size_t(k)]) * u + e.colourA[size_t(k)];
        redraw3(rng_, e.vel);
        redraw3(rng_, e.offset);
        rng_.uniform();  // +0x15c
        ++e.emissions;
        e.life.cur = rng_.range(e.life.lo, e.life.hi);
    }
    return index;
}

void System::stepParticle(Emitter& e, Particle& p, float dt) {  // 0x4c5ce0, then 0x5deca0
    const float a = p.age, a2 = a * a * 0.5f;
    switch (p.motion) {
    case 0: p.pos = add3(p.pos, mul3(p.vel, dt)); break;
    case 1: p.pos = add3(p.start, e.pos); break;
    case 3: p.pos = add3(add3(p.start, mul3(p.vel, a)), mul3(p.accel, a2)); break;
    case 4:
        p.start = add3(p.start, e.delta);
        p.pos = add3(add3(p.start, mul3(p.vel, a)), mul3(p.accel, a2));
        break;
    default: break;
    }
    p.angle += p.rotSpeed * dt;
    p.age += dt;
    p.drawAlpha = fadeValue(p.alphaFn, p.age, p.life) * p.colour[3];
    p.drawSize = fadeValue(p.sizeFn, p.age, p.life) * p.size;
}

void System::runEmitter(Emitter& e, float dt, const V3& eye) {  // 0x4c53b0
    e.delta = sub3(e.pos, e.prevPos);
    if (e.delta[0] * e.delta[0] + e.delta[1] * e.delta[1] + e.delta[2] * e.delta[2] > 400.0f) e.delta = {0, 0, 0};
    // Particle loop (0x4c5960).
    for (size_t i = 0; i < e.live.size();) {
        Particle& p = e.live[i];
        stepParticle(e, p, dt);
        bool dead = (p.killFlags & 1) && p.life < p.age;
        if (p.killFlags & 2) {
            const V3 d = sub3(p.pos, eye);
            dead = dead || d[0] * d[0] + d[1] * d[1] + d[2] * d[2] > 40000.0f;
        }
        if (dead) e.live.erase(e.live.begin() + long(i));
        else ++i;
    }
    if (e.stopped) return;
    const int mode = e.spawnMode & 0xf;
    if (mode == 1) {
        e.timer += dt;
        if (e.timer > e.interval.cur) {
            e.timer = 0;
            emit(e);
        }
    } else if (mode == 2) {
        const V3 dv = sub3(e.lastEmit, e.pos);
        float d = len3(dv);
        if (d <= e.spacing.hi * 10.0f) {
            const V3 keep = e.pos;
            V3 walk = e.pos;
            int last = 0;
            if (e.spacing.cur <= d) {
                do {
                    e.pos = walk;
                    last = emit(e);
                    e.lastEmit = e.pos;
                    if (1e-6f < d) walk = add3(walk, mul3(dv, e.spacing.cur / d));
                    d -= e.spacing.cur;
                    e.spacing.cur = rng_.range(e.spacing.lo, e.spacing.hi);
                } while (e.spacing.cur <= d);
            }
            e.pos = keep;
            if (last != 0) e.lastEmit = keep;
        } else {
            e.lastEmit = e.pos;
        }
    } else if (mode == 3 && !e.burstDone) {
        emit(e);
    }
    e.age += dt;
    if (((e.dieMode & 1) && e.duration < e.age) || ((e.dieMode & 2) && e.maxEmissions < e.emissions)) e.stop();
}

void System::update(float dt, const std::vector<WheelFeed>& feeds, const V3& eye) {
    clock_ += dt;
    for (Wheel& w : wheels_) ++w.idle;
    for (const WheelFeed& f : feeds) {
        Wheel* w = nullptr;
        for (Wheel& c : wheels_)
            if (c.key == f.key) w = &c;
        if (!w) {
            if (!f.active) continue;
            wheels_.push_back({f.key, std::make_unique<WheelEmitter>(*this, f.player), 0});
            w = &wheels_.back();
        }
        w->idle = 0;
        if (f.active) w->w->update(dt, f.in);
        else w->w->stopAll();
    }
    for (Wheel& w : wheels_) {
        if (w.idle > 0) w.w->stopAll();
        for (Emitter* e : {&w.w->dust, &w.w->stones, &w.w->projection, &w.w->chunks}) runEmitter(*e, dt, eye);
    }
    // A wheel that is gone for good and has nothing left to draw is dropped.
    wheels_.erase(std::remove_if(wheels_.begin(), wheels_.end(),
                                 [](const Wheel& w) {
                                     return w.idle > 600 && w.w->dust.live.empty() && w.w->stones.live.empty() &&
                                            w.w->projection.live.empty() && w.w->chunks.live.empty();
                                 }),
                  wheels_.end());
    // GPU particles die at spawn + life (0x5ff610 retires ring slots by their death time).
    gpu_.erase(std::remove_if(gpu_.begin(), gpu_.end(), [&](const Particle& p) { return clock_ > p.spawnTime + p.life; }), gpu_.end());
    if (gpu_.size() > 32 * 256) gpu_.erase(gpu_.begin(), gpu_.begin() + long(gpu_.size() - 32 * 256));  // 256 buckets of 32
}

void System::collect(std::vector<const Particle*>& out) const {
    out.clear();
    for (const Particle& p : gpu_) out.push_back(&p);
    for (const Wheel& w : wheels_)
        for (const Emitter* e : {&w.w->dust, &w.w->stones, &w.w->projection, &w.w->chunks})
            for (const Particle& p : e->live)
                if (p.age > 0) out.push_back(&p);
}

// ---- Wheel emitter ----
WheelEmitter::WheelEmitter(System& sys, bool player) : sys_(sys), player_(player) { setup(); }

void WheelEmitter::setup() {  // 0x5efc00 (the dry-land form, +0x3c == 0)
    Well512& r = sys_.rng();
    auto P = [&](int n) { return sys_.P(n); };
    const V3 vlo{-0.9f, 0.3f, -0.9f}, vhi{0.9f, 1.5f, 0.9f};
    const C4 white{1, 1, 1, 1};

    dust = makeEmitter(r);  // +0x44
    setRange(r, dust.size, 1.6f, 2.5f);
    setRange(r, dust.life, P(5070), P(5071));
    setRange(r, dust.spacing, P(5072), P(5073));
    setRange3One(r, dust.vel, vlo, vhi);
    setColours(r, dust, white, white);  // U: the setup's colour arguments; replaced every frame
    dust.spawnMode = 2, dust.uvFlags = 0x18, dust.motion = 3, dust.killFlags = 1;
    dust.alphaFn = Fade::OneMinusTSq, dust.sizeFn = Fade::SqrtQuarter;
    redraw3(r, dust.offset);
    dust.atlas = 47, dust.kind = 0xe;
    setRange(r, dust.angle, -0.3f, 0.3f);
    dust.gpuStatic = true;

    stones = makeEmitter(r);  // +0x58
    setRange(r, stones.size, 1.6f, 2.5f);
    setRange(r, stones.life, P(5090), P(5091));
    setRange3One(r, stones.vel, vlo, vhi);
    setColours(r, stones, white, white);
    stones.spawnMode = 2;
    setRange(r, stones.spacing, P(5092), P(5093));
    stones.uvFlags = 0x18, stones.motion = 3, stones.killFlags = 1;
    stones.alphaFn = Fade::One, stones.sizeFn = Fade::One;
    setRange(r, stones.angle, -kPi, kPi);
    redraw3(r, stones.offset);
    stones.atlas = 42, stones.kind = 0xe, stones.gpuStatic = true;

    projection = makeEmitter(r);  // +0x4c
    setRange(r, projection.size, 0, 0);
    setRange(r, projection.life, 0, 0);
    redraw3(r, projection.offset);
    setRange3One(r, projection.vel, {0, 0, 0}, {0, 0, 0});
    setRange(r, projection.spacing, 0, 0);
    setColours(r, projection, white, white);
    projection.spawnMode = 2, projection.motion = 3, projection.killFlags = 1, projection.uvFlags = 0x10;
    projection.alphaFn = Fade::OneMinusTSq, projection.sizeFn = Fade::Quarter;
    projection.atlas = 43;
    redraw3(r, projection.accel);
    projection.kind = 0xe, projection.gpuStatic = true;

    chunks = makeEmitter(r);  // +0x5c
    setRange(r, chunks.size, 1.6f, 2.5f);
    setRange(r, chunks.life, P(5070), P(5071));
    setRange(r, chunks.spacing, P(5072), P(5073));
    setRange3One(r, chunks.vel, vlo, vhi);
    setColours(r, chunks, white, white);
    chunks.spawnMode = 2, chunks.uvFlags = 0x18, chunks.motion = 3, chunks.killFlags = 1;
    chunks.alphaFn = Fade::OneMinusT, chunks.sizeFn = Fade::SqrtQuarter;
    redraw3(r, chunks.offset);
    chunks.atlas = 44;
    setRange(r, chunks.angle, -kPi, kPi);
    chunks.kind = 0xe, chunks.gpuStatic = true;
}

void WheelEmitter::stopAll() {  // 0x5f0af0
    stopped_ = true;
    dust.stop(), stones.stop(), projection.stop(), chunks.stop();
}

void WheelEmitter::surfaceRules(const WheelInput& in, bool rainFlag) {  // 0x5f29d0
    const auto& c = in.classes;
    const bool tarmac = c[0] > 0.5f;
    const bool rough = !tarmac && c[2] + c[5] + c[6] + c[7] > 0.5f;           // classes 2, 5, 6, 7
    const bool soft = !tarmac && !rough && c[8] + c[9] + c[10] > 0.5f;          // 8, 9, 10
    const bool snowy = !tarmac && !rough && !soft && c[15] + c[16] + c[17] > 0.5f;  // 15, 16, 17
    const float wetSum = c[11] + c[12];                                          // 11, 12
    const bool other = !tarmac && !rough && !soft && !snowy && !(wetSum > 0.5f);
    const bool moving = mode != 3, slipMode = mode == 1 || mode == 2;
    if (in.depth <= 0) {
        if (in.rain <= 0.1f) {
            if (wetSum > 0.5f) {  // 0x5f2bc0
                dust.stop(), projection.stop(), stones.stop(), chunks.start();
            } else if (tarmac) {  // 0x5f2d30: smoke while sliding
                projection.stop(), stones.stop(), chunks.stop();
                if (mode == 0 || mode == 3) dust.stop();
                else dust.start();
            } else if (rough) {  // 0x5f2e30
                dust.stop(), projection.stop(), chunks.stop(), stones.start();
            } else if (other || soft || snowy) {  // 0x5f2e80 / 0x5f2c10 / 0x5f2ca0
                projection.stop(), stones.stop(), chunks.stop();
                if (moving) dust.start();
                else dust.stop();
            }
        } else {  // 0x5f2f10: wet weather
            dust.stop(), chunks.stop();
            if (tarmac) projection.start(), stones.stop();
            else if (!(0.5f <= wetSum)) projection.start();
            if (rainFlag) dust.stop(), stones.stop(), chunks.stop(), projection.start();  // 0x5f2de0
        }
    } else {  // 0x5f2de0: in water / soft ground
        dust.stop(), stones.stop(), chunks.stop(), projection.start();
    }
    (void)slipMode;
}

void WheelEmitter::baseUpdate(float dt, const WheelInput& in) {  // 0x5f0ef0
    Well512& r = sys_.rng();
    auto P = [&](int n) { return sys_.P(n); };
    surfaceRules(in, in.rain > 0.5f && in.depth <= 0);
    const float moved = len3(sub3(pos_, prev_));
    const float vlen = len3(in.velocity);
    const bool slip = !(in.slip <= P(5132)) || (vlen < 0.1f && std::fabs(in.spin) > 5.0f);
    const bool air = P(5131) < moved;  // moved this frame (I: "air" in p26; it is the emitter's travel)
    mode = air ? (slip ? 1 : 0) : (slip ? 2 : 3);
    // Rolling direction, spawn offset (half the +0x70 = 2 radius) and the backward throw direction.
    float fl = len3(in.forward);
    const V3 dir = fl > 0 ? mul3(in.forward, 1.0f / fl) : V3{0, 0, 1};
    const V3 offset = mul3(dir, 0.5f * (2.0f * in.radius));
    const float sg = in.spin == 0 ? 0.0f : (in.spin >= 0 ? 1.0f : -1.0f);
    const V3 back = mul3(dir, -sg);
    const float s = std::min(std::fabs(in.spin) * 0.028571429f, 1.0f);
    const C4 hsv = rgbToHsv(in.groundColour);
    const bool onTarmac = in.classes[0] > 0.5f;
    C4 dustB{1, 1, 1, 1};

    if (!dust.stopped) {
        C4 A, B;
        if (onTarmac && mode == 2) {
            A = B = {1, 1, 1, 0.3f};  // tyre smoke
        } else {
            A = hsvToRgb({hsv[0], hsv[1], std::max(P(5075), 0.0f), hsv[3]});
            A[3] = P(5078);
            B = hsvToRgb({hsv[0], hsv[1], P(5075) + P(5076), hsv[3]});
            B[3] = P(5079);
        }
        dustB = B;
        setColours(r, dust, A, B);
        dust.size.lo = clampW((P(5081) - P(5080)) * s, P(5083), P(5082)) * in.intensity[0];
        dust.size.hi = clampW((P(5081) + P(5080)) * s, P(5083), P(5082)) * in.intensity[0];
        V3 v0 = mul3(in.velocity, P(5077));
        if (mode == 2) {
            dust.spawnMode = 1;
            setRange(r, dust.interval, 0.03f, 0.08f);
            v0 = mul3(back, 1.5f * 2.0f);
        } else {
            dust.spawnMode = 2;
        }
        setRange3One(r, dust.vel, v0, v0);
        dust.perEmission = 1;
        dust.spacing.lo = dust.spacing.hi = lerpf(P(5072), P(5073), s);
        dust.life.lo = dust.life.hi = P(5071);
        dust.rotSpeed.lo = 0.1f, dust.rotSpeed.hi = P(5088);
        const float k = (1.0f - P(5089)) / P(5070) * kK;
        dust.accel.lo = dust.accel.hi = {v0[0] * k, lerpf(P(5086), P(5087), s) + v0[1] * k, v0[2] * k};
        dust.offset.lo = dust.offset.hi = offset;
    }
    if (!stones.stopped) {
        const C4 A = hsvToRgb({hsv[0], hsv[1] * 0.7f, hsv[2] * P(5090) - P(5091), 1.0f});
        const C4 B = hsvToRgb({hsv[0], hsv[1] * 0.7f, hsv[2] * P(5090) + P(5091), 1.0f});
        setColours(r, stones, A, B);
        setRange(r, stones.size, clampW(s * P(5098) - P(5097), P(5099), P(5100)) * in.intensity[2],
                 clampW(s * P(5098) + P(5097), P(5099), P(5100)) * in.intensity[2]);
        stones.angle.lo = -P(5103), stones.angle.hi = P(5103);
        stones.perEmission = 1;
        stones.life.lo = P(5093), stones.life.hi = P(5094);
        stones.spacing.lo = P(5095), stones.spacing.hi = P(5096);
        stones.rotSpeed.lo = 0.1f, stones.rotSpeed.hi = P(5104);
        const V3 v0 = mul3(in.velocity, P(5105));
        setRange3One(r, stones.vel, v0, v0);
        const float k = kK * (1.0f - P(5106)) / P(5093);
        stones.accel.lo = stones.accel.hi = {v0[0] * k, v0[1] * k + lerpf(P(5107), P(5108), s), v0[2] * k};
        if (mode == 2) {
            stones.spawnMode = 1;
            setRange(r, stones.interval, 0.03f, 0.08f);
        } else {
            stones.spawnMode = 2;
        }
    }
    if (!projection.stopped) {
        const bool notTarmacShallow = !(onTarmac || 0.1f < in.depth);
        const V3 v0 = mul3(in.velocity, P(5123));
        projection.vel.lo = projection.vel.hi = v0;
        const float w = std::sin(phase_) * 0.2f + 1.0f;
        float lo = std::max(P(5111) * s, P(5111)), hi = std::max(P(5112), P(5112) * s);
        if (onTarmac) lo *= 0.75f, hi *= 0.75f;
        if (in.depth <= 0) {
            const float m = (in.rain + 1.0f) * 0.5f;
            lo *= m, hi *= m;
        }
        lo = std::max(lo * w, 0.1f), hi = std::max(hi * w, 0.1f);
        if (notTarmacShallow) lo *= P(5129), hi *= P(5129);
        projection.life.lo = projection.life.hi = lerpf(P(5115), P(5116), s);
        projection.offset.lo = projection.offset.hi = offset;
        projection.angle.lo = -P(5125), projection.angle.hi = P(5125);
        projection.rotSpeed.lo = P(5126) * 0.5f, projection.rotSpeed.hi = P(5126);
        projection.size.lo = lo * in.intensity[3], projection.size.hi = hi * in.intensity[3];
        projection.spacing.lo = projection.spacing.hi = lerpf(P(5113), P(5114), s);
        const float k = kK * (1.0f - P(5124)) / P(5115);
        projection.accel.lo = projection.accel.hi = {v0[0] * k, v0[1] * k + lerpf(P(5120), P(5121), s), v0[2] * k};
        projection.colourA = projection.colourB = dustB;  // I: the stack slot the engine passes (0x5f25c8) holds the dust's B colour
        if (mode == 2) {
            projection.spawnMode = 1;
            projection.interval.lo = 0.03f, projection.interval.hi = 0.08f;
        } else {
            projection.spawnMode = 2;
        }
    }
    if (!chunks.stopped) {
        if (s < 0.15f) chunks.stop();
        const C4 A = hsvToRgb({hsv[0], hsv[1] * 0.7f, P(5280) - P(5281), 1.0f});
        const C4 B = hsvToRgb({hsv[0], hsv[1] * 0.7f, P(5281) + P(5280), 1.0f});
        setColours(r, chunks, A, B);
        setRange(r, chunks.size, clampW(P(5288) * s - P(5287), P(5289), P(5290)) * in.intensity[1],
                 clampW(P(5288) * s + P(5287), P(5289), P(5290)) * in.intensity[1]);
        chunks.perEmission = 1;
        chunks.life.lo = P(5283), chunks.life.hi = P(5284);
        chunks.spacing.lo = chunks.spacing.hi = lerpf(P(5285), P(5286), s);
        setRange3One(r, chunks.vel, {0, P(5293), 0}, {0, P(5293), 0});
        chunks.rotSpeed.lo = 0.1f, chunks.rotSpeed.hi = P(5294);
        if (mode == 2) {
            chunks.spawnMode = 1;
            chunks.interval.lo = 0.03f, chunks.interval.hi = 0.08f;
        } else {
            chunks.spawnMode = 2;
        }
    }
}

void WheelEmitter::playerUpdate(const WheelInput& in) {  // 0x5eefc0 (after the base update with +0x20 = 1)
    Well512& r = sys_.rng();
    auto P = [&](int n) { return sys_.P(n); };
    const float s = std::min(std::fabs(in.spin) * 0.028571429f, 1.0f);
    const float speed = std::min(len3(in.velocity) * 0.028571429f, 1.0f);  // +0x7c (0x5f0e60)
    const float x = (mode == 2 || mode == 1) ? std::max(speed, s) : speed;
    auto relVel = [&](float k) { return mul3(in.velocity, -k); };
    // Each player emitter: CPU particles (kind 0xb, billboard_soft), motion 4 (they ride with the wheel).
    auto common = [&](Emitter& e, int base, Fade sizeFn) {
        e.gpuStatic = false, e.kind = 0xb, e.motion = 4;
        setRange(r, e.angle, P(base + 12), P(base + 13));
        setRange(r, e.rotSpeed, P(base + 14), P(base + 15));
        e.alphaFn = Fade::LogPuff, e.sizeFn = sizeFn;
    };
    if (!dust.stopped) {  // rows 5550-5565, by x
        C4 c = dust.colour;
        c[3] = lerpf(P(5560), P(5561), x);
        setColours(r, dust, c, c);
        common(dust, 5550, Fade::One);
        setRange3One(r, dust.offset, {0, 0, 0}, {0, 0, 0});
        const float sz = lerpf(P(5558), P(5559), x) * in.intensity[0];
        setRange(r, dust.size, sz, sz);
        const float sp = lerpf(P(5550), P(5551), x);
        setRange(r, dust.spacing, sp, sp);
        if (mode != 2) {
            const V3 v = relVel(lerpf(P(5554), P(5555), x));
            setRange3One(r, dust.vel, v, v);
        }
        const V3 a{0, lerpf(P(5556), P(5557), x), 0};
        dust.accel.lo = dust.accel.hi = a, dust.accel.cur = a;
    }
    if (!projection.stopped) {  // rows 5580-5595, by s
        common(projection, 5580, Fade::Quarter);
        // 5580.. angle/rot rows are 5592-5595 (base + 12 .. 15 of 5580)
        if (mode != 2) {
            const V3 v = relVel(lerpf(P(5584), P(5585), s));
            setRange3One(r, projection.vel, v, v);
        }
        const V3 a{0, lerpf(P(5586), P(5587), s), 0};
        projection.accel.lo = projection.accel.hi = a, projection.accel.cur = a;
        const float sp = lerpf(P(5580), P(5581), s);
        setRange(r, projection.spacing, sp, sp);
        const float lf = lerpf(P(5582), P(5583), s);
        setRange(r, projection.life, lf, lf);
        float sz = lerpf(P(5588), P(5589), s) * in.intensity[3];
        if (in.classes[0] > 0.5f) sz *= 0.75f;
        if (in.depth <= 0) sz = (in.rain + 1.0f) * 0.5f * sz * 0.5f;
        setRange(r, projection.size, sz, sz);
    }
    auto bits = [&](Emitter& e, int base, float intensity) {  // +0x58 (5610-5625) and +0x5c (5640-5655)
        common(e, base, Fade::One);
        C4 c = e.colour;
        c[3] = lerpf(P(base + 10), P(base + 11), x);
        setColours(r, e, c, c);
        if (mode != 2) {
            const V3 v = relVel(lerpf(P(base + 4), P(base + 5), s));
            setRange3One(r, e.vel, v, v);
        }
        const V3 a{0, lerpf(P(base + 6), P(base + 7), s), 0};
        e.accel.lo = e.accel.hi = a, e.accel.cur = a;
        const float sp = lerpf(P(base), P(base + 1), s);
        setRange(r, e.spacing, sp, sp);
        const float lf = lerpf(P(base + 2), P(base + 3), s);
        setRange(r, e.life, lf, lf);
        const float sz = lerpf(P(base + 8), P(base + 9), s) * intensity;
        setRange(r, e.size, sz, sz);
    };
    if (!stones.stopped) bits(stones, 5610, in.intensity[2]);
    if (!chunks.stopped) bits(chunks, 5640, in.intensity[1]);
}

void WheelEmitter::update(float dt, const WheelInput& in) {
    // 0x5ceb60: (re)start a stopped wheel emitter, then place it (0x5f0c20).
    if (stopped_) {  // 0x5f0a90
        stopped_ = false;
        dust.start(), stones.start(), projection.start(), chunks.start();
    }
    prev_ = havePos_ ? pos_ : in.contact;
    pos_ = in.contact;
    havePos_ = true;
    phase_ += 4.0f * dt;  // 0x5f0b50
    auto place = [&](Emitter& e, float k) { e.setPosition({pos_[0], pos_[1] + e.size.cur * k, pos_[2]}); };
    place(dust, 0.5f), place(projection, 0.4f), place(stones, 0.4f), place(chunks, 0.4f);
    baseUpdate(dt, in);
    if (player_) playerUpdate(in);
}

}  // namespace fuel::sfx
