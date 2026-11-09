// [DEVIATION, the user's request 2026-11-06] The nitrous sound, synthesized (nitro_sound.hpp).
#include "audio/nitro_sound.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace fuel::snd {
namespace {

constexpr float kRate = 44100.0f;
constexpr float kTwoPi = 6.2831853f;

struct Noise {  // xorshift32: the same bytes every run
    uint32_t s = 0x6d2b79f5u;
    float next() {
        s ^= s << 13, s ^= s >> 17, s ^= s << 5;
        return float(s) / 2147483648.0f - 1.0f;
    }
};

// Chamberlin state-variable band-pass at fc, damping q (1/Q).
struct BandPass {
    float low = 0, band = 0, f = 0, q = 0;
    BandPass(float fc, float damp) : f(2.0f * std::sin(3.1415927f * fc / kRate)), q(damp) {}
    float step(float x) {
        low += f * band;
        const float high = x - low - q * band;
        band += f * high;
        return band;
    }
};

float onePole(float fc) { return 1.0f - std::exp(-kTwoPi * fc / kRate); }

void normalise(std::vector<float>& d, float peak) {
    float m = 0;
    for (float x : d) m = std::max(m, std::fabs(x));
    if (m > 0)
        for (float& x : d) x *= peak / m;
}

std::shared_ptr<const Pcm> makeLoop() {
    const int n = int(1.6f * kRate), fade = int(0.2f * kRate);
    std::vector<float> raw(size_t(n + fade));
    Noise nz;
    BandPass hiss(2200.0f, 0.7f), air(5200.0f, 1.2f);
    float brown = 0, burn = 0;
    const float aBurn = onePole(130.0f);
    double ph = 0;
    for (size_t i = 0; i < raw.size(); ++i) {
        const float t = float(i) / kRate;
        const float w = nz.next();
        brown = brown * 0.995f + w * 0.02f;
        burn += aBurn * (brown - burn);
        const float f = 620.0f * (1.0f + 0.01f * std::sin(kTwoPi * 5.5f * t));
        ph += double(kTwoPi * f / kRate);
        const float flutter = 1.0f + 0.12f * std::sin(kTwoPi * 11.0f * t) * std::sin(kTwoPi * 0.7f * t);
        raw[i] = (0.55f * hiss.step(w) + 0.18f * air.step(w)) * flutter + 2.2f * burn + 0.05f * float(std::sin(ph));
    }
    auto pcm = std::make_shared<Pcm>();
    pcm->rate = int(kRate), pcm->channels = 1, pcm->loops = true;
    pcm->data.assign(raw.begin(), raw.begin() + n);
    for (int i = 0; i < fade; ++i) {  // the tail crossfaded into the head: a seamless loop
        const float k = float(i) / float(fade);
        pcm->data[size_t(i)] = raw[size_t(n + i)] * (1.0f - k) + raw[size_t(i)] * k;
    }
    normalise(pcm->data, 0.6f);
    return pcm;
}

std::shared_ptr<const Pcm> makeIgnition() {
    const int n = int(0.45f * kRate);
    auto pcm = std::make_shared<Pcm>();
    pcm->rate = int(kRate), pcm->channels = 1;
    pcm->data.resize(size_t(n));
    Noise nz;
    nz.s = 0x1b873593u;
    float lp = 0, lp2 = 0;
    double ph = 0;
    for (int i = 0; i < n; ++i) {
        const float t = float(i) / kRate;
        const float env = (1.0f - std::exp(-t / 0.005f)) * std::exp(-t / 0.12f);
        const float fc = 300.0f + 3700.0f * std::exp(-t / 0.08f);
        const float a = onePole(fc);
        lp += a * (nz.next() - lp);
        lp2 += a * (lp - lp2);
        const float f = 45.0f + 25.0f * std::exp(-t / 0.1f);
        ph += double(kTwoPi * f / kRate);
        const float thump = float(std::sin(ph)) * std::exp(-t / 0.15f) * (1.0f - std::exp(-t / 0.004f));
        pcm->data[size_t(i)] = 1.4f * lp2 * env + 0.6f * thump;
    }
    normalise(pcm->data, 0.7f);
    return pcm;
}

std::shared_ptr<const Pcm> makeRelease() {
    const int n = int(0.35f * kRate);
    auto pcm = std::make_shared<Pcm>();
    pcm->rate = int(kRate), pcm->channels = 1;
    pcm->data.resize(size_t(n));
    Noise nz;
    nz.s = 0x85ebca6bu;
    float lp = 0;
    const float a = onePole(1500.0f);
    for (int i = 0; i < n; ++i) {
        const float t = float(i) / kRate;
        const float w = nz.next();
        lp += a * (w - lp);
        const float env = (1.0f - std::exp(-t / 0.003f)) * std::exp(-t / 0.07f);
        pcm->data[size_t(i)] = (w - lp) * env;
    }
    normalise(pcm->data, 0.55f);
    return pcm;
}

}  // namespace

const NitroBank& NitroBank::get() {
    static const NitroBank bank = [] {
        NitroBank b;
        b.loop = makeLoop(), b.ignition = makeIgnition(), b.release = makeRelease();
        return b;
    }();
    return bank;
}

void NitroVoice::update(float dt, float amount, const std::array<double, 3>& pos, float speed, float gain, Mixer& mix) {
    (void)dt;
    const NitroBank& bank = NitroBank::get();
    Mixer::PlayParams pp;
    pp.pos = {float(pos[0]), float(pos[1]), float(pos[2])};
    if (amount > 0.02f && was_ <= 0.02f) {  // a boost kicks in: the ignition, then the hiss
        ++starts_;
        const int h = mix.play(bank.ignition, pp);
        if (h >= 0) mix.setVolume(h, std::min(1.0f, 0.85f * gain));
        if (loop_ < 0) loop_ = mix.play(bank.loop, pp);
    }
    if (loop_ >= 0) {
        if (!mix.alive(loop_)) {
            loop_ = -1;
        } else {
            mix.setPosition(loop_, pp.pos);
            mix.setVolume(loop_, std::min(1.0f, 0.6f * gain * amount));
            mix.setPitch(loop_, 0.9f + 0.15f * amount + 0.002f * std::min(std::fabs(speed), 60.0f));
        }
    }
    if (amount <= 0.001f && was_ > 0.001f) {  // over: the hiss stops, the blow-off
        if (loop_ >= 0) mix.stop(loop_), loop_ = -1;
        const int h = mix.play(bank.release, pp);
        if (h >= 0) mix.setVolume(h, std::min(1.0f, 0.5f * gain));
    }
    was_ = amount;
}

int nitroSoundCheck() {
    int bad = 0;
    const NitroBank& b = NitroBank::get();
    auto stats = [](const Pcm& p, float& peak, bool& finite) {
        peak = 0, finite = true;
        for (float x : p.data) finite = finite && std::isfinite(x), peak = std::max(peak, std::fabs(x));
    };
    struct Row {
        const char* name;
        const Pcm* pcm;
        float seconds, peak;
        bool loops;
    };
    const Row rows[] = {{"loop", b.loop.get(), 1.6f, 0.6f, true}, {"ignition", b.ignition.get(), 0.45f, 0.7f, false}, {"release", b.release.get(), 0.35f, 0.55f, false}};
    for (const Row& r : rows) {
        float peak = 0;
        bool finite = true;
        if (r.pcm) stats(*r.pcm, peak, finite);
        const float secs = r.pcm ? float(r.pcm->frames()) / float(r.pcm->rate) : 0;
        // the loop's seam: the last sample against the first, against the loop's own largest step between neighbours
        float seam = 0, step = 0;
        if (r.pcm && r.loops && r.pcm->data.size() > 2) {
            seam = std::fabs(r.pcm->data.back() - r.pcm->data.front());
            for (size_t i = 1; i < r.pcm->data.size(); ++i) step = std::max(step, std::fabs(r.pcm->data[i] - r.pcm->data[i - 1]));
        }
        const bool ok = r.pcm && finite && std::fabs(secs - r.seconds) < 0.01f && std::fabs(peak - r.peak) < 1e-3f && r.pcm->loops == r.loops &&
                        (!r.loops || seam <= step);
        bad += !ok;
        std::printf("  [nitro] sound %-8s %.2f s, peak %.3f%s%s: %s\n", r.name, secs, peak, r.loops ? ", loops, seam " : "",
                    r.loops ? (std::to_string(seam) + " (largest step " + std::to_string(step) + ")").c_str() : "", ok ? "ok" : "MISMATCH");
    }
    // A voice over a boost: 0.5 s idle, 0.12 s ramp up, 2 s held, 0.25 s down, 0.5 s after.
    Mixer mix;
    NitroVoice v;
    const float dt = 1.0f / 60;
    int voicesIdle = 0, voicesHeld = 0, voicesAfter = 0, playingHeld = 0, startsAt = -1, stopsAt = -1;
    for (int k = 0; k < 200; ++k) {
        const float t = float(k) * dt;
        const float a = t < 0.5f ? 0.0f : t < 2.62f ? std::min(1.0f, (t - 0.5f) / 0.12f) : std::max(0.0f, 1.0f - (t - 2.62f) / 0.25f);
        const bool was = v.playing();
        v.update(dt, a, {0, 0, 0}, 20, 1.0f, mix);
        mix.update();
        if (!was && v.playing() && startsAt < 0) startsAt = k;
        if (was && !v.playing() && stopsAt < 0) stopsAt = k;
        if (t < 0.5f) voicesIdle = std::max(voicesIdle, mix.activeVoices());
        if (t > 1.0f && t < 2.5f) voicesHeld = std::max(voicesHeld, mix.activeVoices()), playingHeld += v.playing();
        if (t > 3.2f) voicesAfter = std::max(voicesAfter, mix.activeVoices() - 0);
    }
    const bool ok = voicesIdle == 0 && startsAt == 31 && playingHeld == 89 && voicesHeld >= 1 && stopsAt > 157 && stopsAt < 175 && v.starts() == 1;
    bad += !ok;
    std::printf("  [nitro] voice over a 2 s boost: idle voices %d, loop from frame %d to %d, held %d of 89 frames, voices while held %d, after %d, "
                "starts %d: %s\n",
                voicesIdle, startsAt, stopsAt, playingHeld, voicesHeld, voicesAfter, v.starts(), ok ? "ok" : "MISMATCH");
    v.stop(mix);
    return bad;
}

void NitroVoice::stop(Mixer& mix) {
    if (loop_ >= 0) mix.stop(loop_);
    loop_ = -1;
    was_ = 0;
}

}  // namespace fuel::snd
