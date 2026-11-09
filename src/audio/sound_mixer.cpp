#include "sound_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstdio>
#include <cstdlib>

namespace fuel::snd {

std::shared_ptr<const Pcm> Pcm::from(const Sound& s) {
    auto p = std::make_shared<Pcm>();
    p->rate = int(s.rate);
    p->channels = s.channels;
    p->loops = s.loops;
    p->data.resize(s.samples.size());
    for (size_t i = 0; i < s.samples.size(); ++i) p->data[i] = float(s.samples[i]) / 32768.0f;
    return p;
}

int volumeMb(float g) {  // 0x8346c0: g < 1e-6 -> -10000; else trunc(868 ln g) clamped to [-10000, 0]
    if (!(g >= 1e-6f)) return -10000;
    const int mb = int(std::trunc(std::log(double(g)) * 868.0));
    return std::clamp(mb, -10000, 0);
}

int panMb(float p) {  // 0x834630
    const double v = p >= 0 ? std::log(1.0 - double(p)) * -868.0 : std::log(double(p) + 1.0) * 868.0;
    if (!std::isfinite(v)) return p >= 0 ? 10000 : -10000;
    return std::clamp(int(std::trunc(v)), -10000, 10000);
}

float mbToAmplitude(int mb) { return float(std::pow(10.0, double(mb) / 2000.0)); }

float distanceGain(float d, float mn, float mx) {  // 0x733150 (attenuating voices)
    const float over = d - mn;
    if (!(over > 0)) return 1.0f;
    const float span = mx - mn;
    if (!(over < span)) return 0.0f;
    return std::min(1.0f - over / span, std::max(mx * 0.05f, mn) / d);
}

namespace {
float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3f sub(Vec3f a, Vec3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
}  // namespace

Mixer::Voice* Mixer::find(int h) {
    if (h < 0) return nullptr;
    Voice& v = v_[size_t(h & 0xff) % kVoices];
    return v.used && v.serial == uint32_t(h) >> 8 ? &v : nullptr;
}
const Mixer::Voice* Mixer::find(int h) const { return const_cast<Mixer*>(this)->find(h); }

bool Mixer::audible(const PlayParams& p) const {  // 0x732960: within 1.5 x max distance of any camera
    if (!p.positional) return true;
    const float r = 1.5f * p.maxDist;
    for (const Listener& l : listeners_) {
        const Vec3f d = sub(p.pos, l.pos);
        if (dot(d, d) < r * r) return true;
    }
    return listeners_.empty();
}

int Mixer::play(std::shared_ptr<const Pcm> pcm, const PlayParams& p) {
    if (!pcm || pcm->frames() == 0) return -1;
    std::lock_guard<std::mutex> lock(m_);
    if (!audible(p)) return -1;
    for (size_t i = 0; i < v_.size(); ++i) {
        Voice& v = v_[i];
        if (v.used) continue;
        v = Voice{};
        v.used = true;
        v.serial = serial_++ & 0xffffff;
        if (v.serial == 0) v.serial = serial_++ & 0xffffff;
        v.pcm = std::move(pcm);
        v.p = p;
        if (v.p.minDist == 0 && v.p.maxDist == 0) v.p.minDist = 6, v.p.maxDist = 140;  // 0x733080
        if (p.toneDb != 0 || p.shelfDb != 0) v.tone = true, v.tc = toneCoefficients(p.toneDb, p.shelfDb);  // [upgrades U5]
        return int(i) | int(v.serial << 8);
    }
    return -1;  // "Not enough free track"
}

// [upgrades U5] RBJ cookbook low shelf (f0 200 Hz, S = 1) with the overall gain folded into the numerator.
std::array<float, 5> Mixer::toneCoefficients(float toneDb, float shelfDb, float rate) {
    const double A = std::pow(10.0, double(shelfDb) / 40.0), w0 = 2.0 * M_PI * 200.0 / double(rate);
    const double cw = std::cos(w0), alpha = std::sin(w0) / 2.0 * std::sqrt(2.0), sa = 2.0 * std::sqrt(A) * alpha;
    const double b0 = A * ((A + 1) - (A - 1) * cw + sa), b1 = 2 * A * ((A - 1) - (A + 1) * cw), b2 = A * ((A + 1) - (A - 1) * cw - sa);
    const double a0 = (A + 1) + (A - 1) * cw + sa, a1 = -2 * ((A - 1) + (A + 1) * cw), a2 = (A + 1) + (A - 1) * cw - sa;
    const double g = std::pow(10.0, double(toneDb) / 20.0);
    return {float(g * b0 / a0), float(g * b1 / a0), float(g * b2 / a0), float(a1 / a0), float(a2 / a0)};
}

bool Mixer::alive(int h) const {
    std::lock_guard<std::mutex> lock(m_);
    const Voice* v = find(h);
    return v && !v->finished;
}

void Mixer::stop(int h) {
    std::lock_guard<std::mutex> lock(m_);
    if (Voice* v = find(h)) *v = Voice{};
}

void Mixer::setVolume(int h, float x) {
    std::lock_guard<std::mutex> lock(m_);
    if (Voice* v = find(h)) v->volume = x;
}

void Mixer::setPitch(int h, float x) {
    std::lock_guard<std::mutex> lock(m_);
    if (Voice* v = find(h)) v->pitch = x;
}

void Mixer::setPosition(int h, Vec3f pos) {
    std::lock_guard<std::mutex> lock(m_);
    if (Voice* v = find(h)) v->p.pos = pos;
}

void Mixer::setDistances(int h, float mn, float mx) {
    std::lock_guard<std::mutex> lock(m_);
    if (Voice* v = find(h)) v->p.minDist = mn, v->p.maxDist = mx;
}

void Mixer::setListeners(const std::vector<Listener>& l) {
    std::lock_guard<std::mutex> lock(m_);
    listeners_ = l;
}

void Mixer::setBusVolume(float x) {
    std::lock_guard<std::mutex> lock(m_);
    bus_ = std::clamp(x, 0.0f, 1.0f);  // 0x7346c0 clamps to [0, 1]
}

// ---- PK27: SoundManager vt+0x48 0x734710: the voice bus +0x20f4 = clamp(v, 0, 1) (set by 0x554130 from the saved voice
// level +0x25c, n / 16).
void Mixer::setVoiceBus(float x) {
    std::lock_guard<std::mutex> lock(m_);
    voiceBus_ = std::clamp(x, 0.0f, 1.0f);
}
// ---- PK27 end

void Mixer::stopAll() {
    std::lock_guard<std::mutex> lock(m_);
    for (Voice& v : v_) v = Voice{};
}

int Mixer::activeVoices() const {
    std::lock_guard<std::mutex> lock(m_);
    int n = 0;
    for (const Voice& v : v_) n += v.used && !v.finished;
    return n;
}

namespace {
std::atomic<int> gLimiterDefault{1};
bool limiterOn() {
    static const char* e = std::getenv("FUEL_AUDIO_LIMITER");
    if (e && *e) return std::string(e) != "off" && std::string(e) != "0";
    return gLimiterDefault.load(std::memory_order_relaxed) != 0;
}
constexpr float kCeiling = 0.96605f;      // -0.3 dBFS
constexpr float kReleaseTau = 0.15f;      // s (one-pole)
}  // namespace

void Mixer::setLimiterDefault(bool on) { gLimiterDefault.store(on ? 1 : 0, std::memory_order_relaxed); }

// P100: stereo-linked lookahead peak limiter. Output y[n] = x[n - D] * g[n], D = kLook - 1 (latency 2.5 ms). The requested gain
// r[i] = min(1, ceil / max(|L|, |R|)) of every incoming sample; m[j] = min of r over the last kLook samples; g = the
// kLook-sample boxcar mean of m (so the gain has fallen to r[k] by the time sample k leaves the delay: a smooth
// attack of 2.5 ms that never overshoots), then a one-pole release (150 ms) that only ever moves toward g from below.
// While r == 1 for a whole window the gain is exactly 1 (state reset, no float drift): the mix passes bit-identical.
void Mixer::limit(float* L, float* R, int frames) {
    Limiter& s = lim_;
    const float rel = 1.0f - std::exp(-1.0f / (kReleaseTau * float(kRate)));
    for (int i = 0; i < frames; ++i) {
        const int slot = int(s.n % kLook);
        const float a = std::max(std::fabs(L[i]), std::fabs(R[i]));
        const float r = a > kCeiling ? kCeiling / a : 1.0f;
        // sliding minimum over the last kLook requests (monotonic deque of absolute indices)
        while (s.dqCount > 0 && s.n - s.dqIdx[size_t(s.dqHead)] >= kLook) s.dqHead = (s.dqHead + 1) % (kLook + 1), --s.dqCount;
        s.req[size_t(slot)] = r;
        while (s.dqCount > 0) {
            const long long idx = s.dqIdx[size_t((s.dqHead + s.dqCount - 1) % (kLook + 1))];
            if (s.req[size_t(idx % kLook)] >= r) --s.dqCount; else break;
        }
        s.dqIdx[size_t((s.dqHead + s.dqCount) % (kLook + 1))] = s.n;
        ++s.dqCount;
        const float mnew = s.req[size_t(s.dqIdx[size_t(s.dqHead)] % kLook)];
        const float mold = s.m[size_t(slot)];
        s.m[size_t(slot)] = mnew;
        float g;
        if (mnew == 1.0f) ++s.run1; else s.run1 = 0;
        if (s.run1 >= kLook) {
            s.sum = kLook, g = 1.0f;
        } else {
            s.sum += double(mnew) - double(mold);
            g = float(s.sum / kLook);
        }
        if (g < s.gs) s.gs = g; else s.gs += (g - s.gs) * rel;
        if (g == 1.0f && s.gs > 0.99999f) s.gs = 1.0f;
        s.dl[size_t(slot)] = L[i], s.dr[size_t(slot)] = R[i];
        const size_t old = size_t((slot + 1) % kLook);  // written kLook - 1 samples ago
        const float dl = s.dl[old], dr = s.dr[old];
        L[i] = std::clamp(dl * s.gs, -1.0f, 1.0f);
        R[i] = std::clamp(dr * s.gs, -1.0f, 1.0f);
        if (s.gs < 1.0f) stats_.limited += 2, stats_.minGain = std::min(stats_.minGain, s.gs);
        ++s.n;
    }
}

void Mixer::update() {
    std::lock_guard<std::mutex> lock(m_);
    for (Voice& v : v_) {
        if (!v.used) continue;
        if (v.finished) {  // a one-shot that played out frees its voice
            v = Voice{};
            continue;
        }
        float g3 = 1, pan = 0;
        if (v.p.positional && !listeners_.empty()) {
            if (!audible(v.p)) {  // 0x732f40 stops a voice beyond 1.5 x max distance
                v = Voice{};
                continue;
            }
            g3 = 0;
            float sumX = 0;
            for (const Listener& l : listeners_) {
                const Vec3f d = sub(v.p.pos, l.pos);
                const Vec3f c{dot(d, l.right), dot(d, l.up), dot(d, l.forward)};
                const float dist = std::sqrt(dot(c, c));
                g3 = std::max(g3, v.p.attenuate ? distanceGain(dist, v.p.minDist, v.p.maxDist) : 1.0f);
                if (dist > 1e-6f) sumX += c.x * (0.9f / dist);  // 0x530ba0(0.9 / len)
            }
            if (v.pcm->channels == 1) pan = sumX / float(listeners_.size());
        }
        const int mb = volumeMb((v.p.voiceBus ? voiceBus_ : bus_) * v.volume * g3);  // PK27: 0x732f40 picks the voice bus +0x20f4
        const int pmb = panMb(pan);
        const float amp = mbToAmplitude(mb);
        v.ampL = amp * (pmb > 0 ? mbToAmplitude(-pmb) : 1.0f);
        v.ampR = amp * (pmb < 0 ? mbToAmplitude(pmb) : 1.0f);
        const long freq = std::clamp<long>(std::lround(std::floor(double(v.pcm->rate) * double(v.pitch))), 100, 100000);
        v.step = float(double(freq) / double(kRate));
    }
}

void Mixer::setAux(std::function<void(float*, float*, int)> f) {
    std::lock_guard<std::mutex> lock(m_);
    aux_ = std::move(f);
}

void Mixer::setPauseMask(uint32_t mask) {  // PK17: WinSound_Z vf15 0x833cc0
    std::lock_guard<std::mutex> lock(m_);
    pauseMask_ = mask;
}

void Mixer::render(float* L, float* R, int frames) {
    std::fill(L, L + frames, 0.0f);
    std::fill(R, R + frames, 0.0f);
    std::lock_guard<std::mutex> lock(m_);
    for (Voice& v : v_) {
        if (!v.used || v.finished || (v.ampL <= 0 && v.ampR <= 0 && v.step <= 0)) continue;
        if (pauseMask_ & 1) continue;  // PK17: paused (flag 2): no output, the cursor stays
        const Pcm& p = *v.pcm;
        const size_t n = p.frames();
        const int ch = p.channels;
        double c = v.cursor;
        for (int i = 0; i < frames; ++i) {
            if (c >= double(n)) {
                if (!p.loops) {
                    v.finished = true;
                    break;
                }
                c = std::fmod(c, double(n));
            }
            const size_t i0 = size_t(c);
            const size_t i1 = i0 + 1 < n ? i0 + 1 : (p.loops ? 0 : i0);
            const float f = float(c - double(i0));
            auto tone = [&v](float x, int k) {  // [upgrades U5] the engine stage's shelf (direct form I)
                const float y = v.tc[0] * x + v.tc[1] * v.tx1[k] + v.tc[2] * v.tx2[k] - v.tc[3] * v.ty1[k] - v.tc[4] * v.ty2[k];
                v.tx2[k] = v.tx1[k], v.tx1[k] = x, v.ty2[k] = v.ty1[k], v.ty1[k] = y;
                return y;
            };
            if (ch == 1) {
                float s = p.data[i0] + (p.data[i1] - p.data[i0]) * f;
                if (v.tone) s = tone(s, 0);
                L[i] += s * v.ampL;
                R[i] += s * v.ampR;
            } else {
                float l = p.data[i0 * 2] + (p.data[i1 * 2] - p.data[i0 * 2]) * f;
                float r = p.data[i0 * 2 + 1] + (p.data[i1 * 2 + 1] - p.data[i0 * 2 + 1]) * f;
                if (v.tone) l = tone(l, 0), r = tone(r, 1);
                L[i] += l * v.ampL;
                R[i] += r * v.ampR;
            }
            c += v.step;
        }
        v.cursor = c;
    }
    if (aux_) aux_(L, R, frames);
    for (int i = 0; i < frames; ++i) {
        const float a = std::max(std::fabs(L[i]), std::fabs(R[i]));
        stats_.peak = std::max(stats_.peak, a);
        stats_.clipped += uint64_t(std::fabs(L[i]) > 1.0f) + uint64_t(std::fabs(R[i]) > 1.0f);
        stats_.sumSq += double(L[i]) * L[i] + double(R[i]) * R[i];
    }
    if (limiterOn()) {
        limit(L, R, frames);
    } else {  // the 16-bit primary buffer clips
        for (int i = 0; i < frames; ++i) {
            L[i] = std::clamp(L[i], -1.0f, 1.0f);
            R[i] = std::clamp(R[i], -1.0f, 1.0f);
        }
    }
    for (int i = 0; i < frames; ++i) {
        stats_.peakOut = std::max(stats_.peakOut, std::max(std::fabs(L[i]), std::fabs(R[i])));
        stats_.clippedOut += uint64_t(std::fabs(L[i]) >= 1.0f) + uint64_t(std::fabs(R[i]) >= 1.0f);
        stats_.sumSqOut += double(L[i]) * L[i] + double(R[i]) * R[i];
    }
    stats_.samples += uint64_t(frames) * 2;
}

std::string Mixer::describe() const {
    std::lock_guard<std::mutex> lock(m_);
    std::string out;
    char b[256];
    for (size_t i = 0; i < v_.size(); ++i) {
        const Voice& v = v_[i];
        if (!v.used || v.finished) continue;
        std::snprintf(b, sizeof b, "    voice %2zu: %zu fr x%d %s %s vol %.3f pitch %.3f amp %.3f/%.3f at (%.1f, %.1f, %.1f) %g..%g m\n", i, v.pcm->frames(),
                      v.pcm->channels, v.pcm->loops ? "loop" : "once", v.p.positional ? "3D" : "2D", v.volume, v.pitch, v.ampL, v.ampR, v.p.pos.x,
                      v.p.pos.y, v.p.pos.z, v.p.minDist, v.p.maxDist);
        out += b;
    }
    return out;
}

Mixer::Stats Mixer::takeStats() {
    std::lock_guard<std::mutex> lock(m_);
    const Stats s = stats_;
    stats_ = {};
    return s;
}

}  // namespace fuel::snd
