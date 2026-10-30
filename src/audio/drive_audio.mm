#include "drive_audio.h"

#import <AVFoundation/AVFoundation.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>

namespace fuel {

namespace {

// Rolling/skid class: the game picks one of 18 ground classes per wheel. The
// port does not know the terrain's class yet, so it uses the first of these
// (a gravel/dirt-like set by the table's gravel rate) that the bank covers.
struct SurfaceClass {
    uint32_t rollSlow, rollFast, skid;
};
constexpr SurfaceClass kSurfaces[] = {
    {0x2e, 0x2f, 0x11},  // class 2, gravel rate 0.3
    {0x30, 0x31, 0x12},  // class 4, 0.5
    {0x20, 0x21, 0x0e},  // class 0
    {0x2c, 0x2d, 0x10},  // class 1
    {0x22, 0x23, 0x0f},  // class 3
};
constexpr int kSurfaceClassNumber[] = {2, 4, 0, 1, 3};

constexpr float kLoadTime = 0.15f;     // seconds, throttle -> engine load smoothing
constexpr float kImpactDecel = 25.0f;  // m/s^2
constexpr float kAirTime = 0.4f;       // s in the air before a landing counts
constexpr float kMaxPeakQuiet = 200;   // samples quieter than this are placeholders

int16_t peakOf(const Sound& s) {
    int peak = 0;
    for (int16_t v : s.samples) peak = std::max(peak, std::abs(int(v)));
    return int16_t(std::min(peak, 32767));
}

// Decodes a bank symbol; an absent symbol or one that fails to decode gives an empty sound.
Sound tryDecode(const DpcFile& dpc, const SoundBank& bank, uint32_t symbol) {
    const auto it = bank.find(symbol);
    if (it == bank.end()) return {};
    try {
        return decodeSound(dpc, it->second);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: sound %02x: %s\n", symbol, e.what());
        return {};
    }
}

}  // namespace

VehicleSounds loadVehicleSounds(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params) {
    VehicleSounds v;
    v.variant = variant;
    v.engine = EngineModel::fromParams(params);
    const SoundBank bank = readSoundBank(dpc, variant);
    v.idleOrOff[0] = tryDecode(dpc, bank, snd::EngineIdle);
    for (uint32_t L = 1; L < EngineModel::kLayers; ++L) {
        v.idleOrOff[L] = tryDecode(dpc, bank, snd::EngineOff + L);
        v.on[L] = tryDecode(dpc, bank, snd::EngineOn + L);
    }
    if (v.idleOrOff[0].samples.empty()) throw std::runtime_error(variant + ": no engine idle sample in its sound bank");
    for (size_t i = 0; i < std::size(kSurfaces); ++i) {
        v.rollSlow = tryDecode(dpc, bank, kSurfaces[i].rollSlow);
        if (v.rollSlow.samples.empty()) continue;
        v.rollFast = tryDecode(dpc, bank, kSurfaces[i].rollFast);
        v.skid = tryDecode(dpc, bank, kSurfaces[i].skid);
        v.surfaceClass = kSurfaceClassNumber[i];
        break;
    }
    v.wind = tryDecode(dpc, bank, snd::Wind);
    for (uint32_t s = snd::ImpactFirst; s <= snd::ImpactLast; ++s) {
        Sound sound = tryDecode(dpc, bank, s);
        if (!sound.samples.empty() && !sound.loops && peakOf(sound) >= kMaxPeakQuiet) v.impacts.push_back(std::move(sound));
    }
    return v;
}

struct DriveAudio::Impl {
    struct Voice {
        AVAudioPlayerNode* player = nil;
        AVAudioUnitVarispeed* speed = nil;
        bool valid() const { return player != nil; }
        void set(float volume, float rate) const {
            if (!valid()) return;
            player.volume = std::clamp(volume, 0.0f, 1.0f);
            speed.rate = std::clamp(rate, 0.25f, 4.0f);
        }
    };

    AVAudioEngine* engine = nil;
    bool started = false, muted = false;
    float master = 0.8f;
    std::unique_ptr<VehicleSounds> sounds;
    std::vector<AVAudioNode*> nodes;  // everything attached for the current vehicle
    std::array<Voice, EngineModel::kLayers> idleOrOff, on;
    Voice rollSlow, rollFast, skid, wind;
    std::vector<AVAudioPCMBuffer*> impactBuffers;
    std::vector<AVAudioPlayerNode*> impactPlayers;
    size_t nextImpactPlayer = 0;

    float load = 0, prevSpeed = 0, airTime = 0, sinceImpact = 1;
    int prevGround = 4;

    static AVAudioPCMBuffer* makeBuffer(const Sound& s, bool loop) {
        AVAudioFormat* fmt = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:double(s.rate) channels:AVAudioChannelCount(s.channels)];
        const size_t ch = size_t(s.channels);
        // A loop is cross-faded over its seam (the game's loops do not always meet cleanly):
        // the buffer starts `fade` frames in and its last `fade` frames blend into the skipped start.
        size_t frames = s.frames(), skip = 0, fade = 0;
        if (loop) {
            fade = std::min<size_t>(size_t(s.rate) * 7 / 1000, frames / 8);
            skip = fade;
        }
        const size_t outFrames = frames - skip;
        AVAudioPCMBuffer* buf = [[AVAudioPCMBuffer alloc] initWithPCMFormat:fmt frameCapacity:AVAudioFrameCount(outFrames)];
        buf.frameLength = AVAudioFrameCount(outFrames);
        for (size_t c = 0; c < ch; ++c) {
            float* dst = buf.floatChannelData[c];
            auto at = [&](size_t frame) { return float(s.samples[frame * ch + c]) / 32768.0f; };
            for (size_t i = 0; i < outFrames; ++i) dst[i] = at(i + skip);
            for (size_t i = 0; i < fade; ++i) {
                const float w = float(i + 1) / float(fade + 1);
                dst[outFrames - fade + i] = at(frames - fade + i) * (1 - w) + at(i) * w;
            }
        }
        return buf;
    }

    Voice makeVoice(const Sound& s, bool loop = true) {
        Voice v;
        if (s.samples.empty()) return v;
        AVAudioPCMBuffer* buf = makeBuffer(s, loop);
        v.player = [AVAudioPlayerNode new];
        v.speed = [AVAudioUnitVarispeed new];
        [engine attachNode:v.player];
        [engine attachNode:v.speed];
        [engine connect:v.player to:v.speed format:buf.format];
        [engine connect:v.speed to:engine.mainMixerNode format:buf.format];
        nodes.push_back(v.player);
        nodes.push_back(v.speed);
        v.player.volume = 0;
        [v.player scheduleBuffer:buf atTime:nil options:loop ? AVAudioPlayerNodeBufferLoops : 0 completionHandler:nil];
        return v;
    }

    void teardown() {
        if (engine == nil) return;
        [engine stop];
        for (AVAudioNode* n : nodes) [engine detachNode:n];
        nodes.clear();
        idleOrOff = {};
        on = {};
        rollSlow = rollFast = skid = wind = {};
        impactBuffers.clear();
        impactPlayers.clear();
        started = false;
    }

    void build() {
        const VehicleSounds& s = *sounds;
        for (size_t L = 0; L < EngineModel::kLayers; ++L) {
            idleOrOff[L] = makeVoice(s.idleOrOff[L]);
            if (L > 0) on[L] = makeVoice(s.on[L]);
        }
        rollSlow = makeVoice(s.rollSlow);
        rollFast = makeVoice(s.rollFast);
        skid = makeVoice(s.skid);
        wind = makeVoice(s.wind);
        if (!s.impacts.empty()) {
            for (const Sound& i : s.impacts) impactBuffers.push_back(makeBuffer(i, false));
            for (int k = 0; k < 3; ++k) {
                AVAudioPlayerNode* p = [AVAudioPlayerNode new];
                [engine attachNode:p];
                [engine connect:p to:engine.mainMixerNode format:impactBuffers[0].format];
                nodes.push_back(p);
                impactPlayers.push_back(p);
            }
        }
        engine.mainMixerNode.outputVolume = muted ? 0.0f : master;
        NSError* err = nil;
        [engine prepare];
        if (![engine startAndReturnError:&err]) {
            std::fprintf(stderr, "fuelview: audio: cannot start the engine: %s\n", err.localizedDescription.UTF8String);
            return;
        }
        started = true;
        for (const Voice* v : {&rollSlow, &rollFast, &skid, &wind})
            if (v->valid()) [v->player play];
        for (size_t L = 0; L < EngineModel::kLayers; ++L) {
            if (idleOrOff[L].valid()) [idleOrOff[L].player play];
            if (on[L].valid()) [on[L].player play];
        }
        for (AVAudioPlayerNode* p : impactPlayers) [p play];
    }

    void impact(float intensity) {
        if (impactBuffers.empty() || !started) return;
        intensity = std::clamp(intensity, 0.0f, 1.0f);
        // Harder hits take later samples of the bank's impact set.
        const size_t pick = std::min(impactBuffers.size() - 1, size_t(intensity * float(impactBuffers.size())));
        AVAudioPlayerNode* p = impactPlayers[nextImpactPlayer++ % impactPlayers.size()];
        p.volume = 0.35f + 0.65f * intensity;
        [p scheduleBuffer:impactBuffers[pick] atTime:nil options:AVAudioPlayerNodeBufferInterrupts completionHandler:nil];
        if (!p.playing) [p play];
    }
};

DriveAudio::DriveAudio() : impl_(std::make_unique<Impl>()) {
    impl_->engine = [AVAudioEngine new];
}

DriveAudio::~DriveAudio() {
    if (impl_->engine != nil) [impl_->engine stop];
}

void DriveAudio::setVehicle(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params) {
    Impl& m = *impl_;
    m.teardown();
    m.sounds.reset();
    try {
        m.sounds = std::make_unique<VehicleSounds>(loadVehicleSounds(dpc, variant, params));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: audio: %s: %s\n", variant.c_str(), e.what());
        return;
    }
    m.build();
    m.load = 0;
    m.airTime = 0;
    m.prevGround = 4;
}

void DriveAudio::update(float dt, const VehicleState& st, float throttle) {
    Impl& m = *impl_;
    if (!m.started || !m.sounds || dt < 1e-4f) return;
    const VehicleSounds& s = *m.sounds;
    const float speed = std::fabs(st.speedKmh) / 3.6f;
    const float groundFrac = st.wheels.empty() ? 1.0f : float(st.wheelsOnGround) / float(st.wheels.size());

    // Engine: crossfaded layers, on-load/off-load split by the smoothed throttle.
    m.load += (std::clamp(throttle, 0.0f, 1.0f) - m.load) * (1.0f - std::exp(-dt / kLoadTime));
    const EngineModel::Mix mix = s.engine.mix(st.rpm);
    const float V = std::clamp(1.0f - speed / 35.0f, 0.75f, 1.0f);
    m.idleOrOff[0].set(mix.weight[0] * V, mix.pitch[0]);
    for (size_t L = 1; L < EngineModel::kLayers; ++L) {
        m.idleOrOff[L].set((1 - m.load) * mix.weight[L] * V, mix.pitch[L]);
        m.on[L].set(m.load * mix.weight[L] * V, mix.pitch[L]);
    }

    // Tyres: rolling fades from the slow to the fast loop over 4..40 m/s; skid follows the slip proxy.
    const float slow = speed < 4.0f ? speed / 4.0f : std::max(0.0f, 1.0f - (speed - 4.0f) / 36.0f);
    const float fast = speed < 4.0f ? 0.0f : std::min(1.0f, (speed - 4.0f) / 36.0f);
    const bool hasFast = m.rollFast.valid();
    m.rollSlow.set(0.8f * groundFrac * (hasFast ? slow : std::min(1.0f, speed / 4.0f)), 1.0f);
    m.rollFast.set(0.8f * groundFrac * fast, 1.0f);
    m.skid.set(std::min(1.0f, st.skid * 1.2f) * (speed > 1.5f ? 1.0f : speed / 1.5f), 1.0f + 0.3f * st.skid);
    m.wind.set(0.7f * std::min(1.0f, speed / 35.0f), 1.0f);

    // Impacts: a sharp loss of speed, or touching down after a jump.
    m.sinceImpact += dt;
    const float decel = (m.prevSpeed - speed) / dt;
    if (st.wheelsOnGround == 0) m.airTime += dt;
    if (m.sinceImpact > 0.3f) {
        if (m.prevGround == 0 && st.wheelsOnGround > 0 && m.airTime > kAirTime) {
            m.impact(0.3f + std::min(m.airTime, 2.0f) * 0.35f);
            m.sinceImpact = 0;
        } else if (decel > kImpactDecel && m.prevSpeed > 3.0f) {
            m.impact(decel / 120.0f + 0.2f);
            m.sinceImpact = 0;
        }
    }
    if (st.wheelsOnGround > 0) m.airTime = 0;
    m.prevGround = st.wheelsOnGround;
    m.prevSpeed = speed;
}

void DriveAudio::toggleMute() {
    impl_->muted = !impl_->muted;
    impl_->engine.mainMixerNode.outputVolume = impl_->muted ? 0.0f : impl_->master;
}

bool DriveAudio::muted() const { return impl_->muted; }

void DriveAudio::setMasterVolume(float v) {
    impl_->master = std::clamp(v, 0.0f, 1.0f);
    impl_->engine.mainMixerNode.outputVolume = impl_->muted ? 0.0f : impl_->master;
}

int audioCheck(const std::vector<std::pair<const DpcFile*, std::string>>& variants, const std::map<std::string, std::vector<float>>& params) {
    int failures = 0;
    for (const auto& [dpc, name] : variants) {
        std::printf("%s (%s)\n", name.c_str(), dpc->path().c_str());
        try {
            const auto it = params.find(name);
            if (it == params.end()) throw std::runtime_error("no driving parameters");
            const VehicleSounds v = loadVehicleSounds(*dpc, name, it->second);
            const EngineModel& e = v.engine;
            std::printf("  engine: idle %.0f rpm, max %.0f rpm, gain P649 %.2f; surface class %d; %zu impact samples\n", e.idleRpm, e.maxRpm, e.gain,
                        v.surfaceClass, v.impacts.size());
            auto line = [](const char* what, const Sound& s) {
                if (s.samples.empty()) std::printf("  %-10s (missing)\n", what);
                else std::printf("  %-10s %7zu frames  %5u Hz  %s%s\n", what, s.frames(), s.rate, s.channels == 2 ? "stereo " : "mono ", s.loops ? "loop" : "one-shot");
            };
            line("idle", v.idleOrOff[0]);
            for (int L = 1; L < EngineModel::kLayers; ++L) {
                char label[16];
                std::snprintf(label, sizeof label, "off L%d", L);
                line(label, v.idleOrOff[size_t(L)]);
                std::snprintf(label, sizeof label, "on  L%d", L);
                line(label, v.on[size_t(L)]);
            }
            line("roll slow", v.rollSlow);
            line("roll fast", v.rollFast);
            line("skid", v.skid);
            line("wind", v.wind);
            for (size_t i = 0; i < v.impacts.size(); ++i) {
                char label[16];
                std::snprintf(label, sizeof label, "impact %zu", i);
                line(label, v.impacts[i]);
            }
            std::printf("  layer weights / pitches (idle, mid, redline rpm):\n");
            const float rpms[3] = {e.idleRpm, 0.5f * (e.idleRpm + e.maxRpm), e.maxRpm};
            for (float rpm : rpms) {
                const EngineModel::Mix m = e.mix(rpm);
                float sum = 0;
                std::printf("    %6.0f rpm  w:", rpm);
                for (int L = 0; L < EngineModel::kLayers; ++L) {
                    std::printf(" %.2f", m.weight[size_t(L)]);
                    sum += m.weight[size_t(L)];
                }
                std::printf("  sum %.3f (P649 %.2f)  pitch:", sum, e.gain);
                for (int L = 0; L < EngineModel::kLayers; ++L) std::printf(" %.2f", m.pitch[size_t(L)]);
                std::printf("\n");
                if (std::fabs(sum - e.gain) > 0.02f * e.gain) {
                    std::printf("    ** weights do not sum to P649\n");
                    ++failures;
                }
            }
            // Each layer should play at pitch ~1 when the engine runs at its recorded rpm.
            std::printf("  pitch at each layer's native rpm:");
            for (int L = 0; L < EngineModel::kLayers; ++L) {
                const float rpm = e.nativeRpm[size_t(L)];
                const EngineModel::Mix m = e.mix(rpm);
                const bool inRange = rpm >= e.idleRpm && rpm <= e.maxRpm;
                std::printf(" L%d@%.0f=%.2f%s", L, rpm, m.pitch[size_t(L)], inRange ? "" : "(outside range)");
            }
            std::printf("\n");
        } catch (const std::exception& ex) {
            std::printf("  FAILED: %s\n", ex.what());
            ++failures;
        }
    }
    return failures;
}

}  // namespace fuel
