#include "drive_audio.h"

#import <AVFoundation/AVFoundation.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <stdexcept>

namespace fuel {

namespace {

// Tyre sound classes: the game picks one of 18 ground classes per wheel, each with a
// rolling slow/fast loop and a skid loop (agent Q's table at 0xa06160; 0 = no sample).
// The port maps its surface types to the classes in core/surface.cpp. Classes 10 and
// 12 repeat 9 and 11; 13..17 are the shared 0x26/0x18 set, which the port does not use.
struct TyreIds {
    uint32_t rollSlow, rollFast, skid;
};
constexpr TyreIds kTyreIds[kSoundClasses] = {
    {0x20, 0x21, 0x0e}, {0x2c, 0x2d, 0x10}, {0x2e, 0x2f, 0x11}, {0x22, 0x23, 0x0f}, {0x30, 0x31, 0x12}, {0x32, 0x33, 0x13},
    {0x34, 0x35, 0x14}, {0x36, 0x37, 0x15}, {0x38, 0x39, 0x16}, {0x29, 0, 0x1a},    {0x29, 0, 0x1a},    {0x27, 0x28, 0x19},
    {0x27, 0x28, 0x19}, {0x26, 0, 0x18},    {0x26, 0, 0x18},    {0x26, 0, 0x18},    {0x26, 0, 0x18},    {0x26, 0, 0x18}};
constexpr int kUsedClasses[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11};  // those surfaceSoundClass() returns
constexpr int kFallbackClass = 2;                                    // dirt: for a class the bank lacks

constexpr float kClassFade = 8.0f;     // 1/s, how fast tyre classes crossfade
constexpr float kAbsorberGain = 0.05f; // volume per m/s above the threshold, before P652 (the game's global is unknown)
constexpr float kScrapeMin = 1.0f;     // m/s of sliding before the scrape starts

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
    for (int c : kUsedClasses) {
        VehicleSounds::Tyre& t = v.tyres[size_t(c)];
        t.rollSlow = tryDecode(dpc, bank, kTyreIds[c].rollSlow);
        if (kTyreIds[c].rollFast) t.rollFast = tryDecode(dpc, bank, kTyreIds[c].rollFast);
        t.skid = tryDecode(dpc, bank, kTyreIds[c].skid);
    }
    // A class the bank lacks sounds like dirt.
    for (int c : kUsedClasses)
        if (v.tyres[size_t(c)].rollSlow.samples.empty()) v.tyres[size_t(c)] = v.tyres[kFallbackClass];
    v.wind = tryDecode(dpc, bank, snd::Wind);
    for (uint32_t s = snd::ImpactFirst; s <= snd::ImpactLast; ++s) {
        Sound sound = tryDecode(dpc, bank, s);
        if (!sound.samples.empty() && !sound.loops && peakOf(sound) >= kMaxPeakQuiet) v.impacts.push_back(std::move(sound));
    }
    v.scrape = tryDecode(dpc, bank, snd::Scrape);
    for (uint32_t id = snd::BackfireFirst; id <= snd::BackfireLast; ++id) {
        Sound sound = tryDecode(dpc, bank, id);
        if (!sound.samples.empty() && peakOf(sound) >= kMaxPeakQuiet) v.backfires.push_back(std::move(sound));
    }
    for (uint32_t id = snd::AbsorberFirst; id <= snd::AbsorberBig; ++id) {
        Sound sound = tryDecode(dpc, bank, id);
        if (sound.samples.empty() || peakOf(sound) < kMaxPeakQuiet) continue;
        if (id == snd::AbsorberBig) v.bigAbsorber = true;
        v.absorbers.push_back(std::move(sound));
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
    struct TyreVoices {
        Voice rollSlow, rollFast, skid;
        float weight = 0, skidLevel = 0;  // smoothed share of wheels on this class, and its skid
    };
    std::array<TyreVoices, kSoundClasses> tyres;
    Voice wind, scrape;
    float scrapeLevel = 0;
    std::vector<AVAudioPCMBuffer*> impactBuffers;
    std::vector<AVAudioPlayerNode*> impactPlayers;
    size_t nextImpactPlayer = 0;

    // One-shot sets (backfire, suspension thumps): buffers in bank order and a few players.
    struct OneShots {
        std::vector<AVAudioPCMBuffer*> buffers;
        std::vector<AVAudioPlayerNode*> players;
        size_t next = 0;
    };
    OneShots backfire, absorber;

    float load = 0, prevSpeed = 0, airTime = 0, sinceImpact = 1;
    int prevGround = 4;
    float prevRpm = 0, rpmRate = 0, sincePop = 1, popGap = 0.15f;
    std::vector<float> compressionPeak, sinceThump;

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
        tyres = {};
        wind = scrape = {};
        impactBuffers.clear();
        impactPlayers.clear();
        backfire = {};
        absorber = {};
        started = false;
    }

    void build() {
        const VehicleSounds& s = *sounds;
        for (size_t L = 0; L < EngineModel::kLayers; ++L) {
            idleOrOff[L] = makeVoice(s.idleOrOff[L]);
            if (L > 0) on[L] = makeVoice(s.on[L]);
        }
        for (int c : kUsedClasses) {
            const VehicleSounds::Tyre& t = s.tyres[size_t(c)];
            tyres[size_t(c)].rollSlow = makeVoice(t.rollSlow);
            tyres[size_t(c)].rollFast = makeVoice(t.rollFast);
            tyres[size_t(c)].skid = makeVoice(t.skid);
        }
        wind = makeVoice(s.wind);
        scrape = makeVoice(s.scrape);
        auto makeOneShots = [&](OneShots& set, const std::vector<Sound>& sounds, int players) {
            if (sounds.empty()) return;
            for (const Sound& snd : sounds) set.buffers.push_back(makeBuffer(snd, false));
            for (int k = 0; k < players; ++k) {
                AVAudioPlayerNode* p = [AVAudioPlayerNode new];
                [engine attachNode:p];
                [engine connect:p to:engine.mainMixerNode format:set.buffers[0].format];
                nodes.push_back(p);
                set.players.push_back(p);
            }
        };
        makeOneShots(backfire, s.backfires, 2);
        makeOneShots(absorber, s.absorbers, 4);
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
        // AVAudioPlayerNode -play throws ("player did not see an IO cycle") when the engine has no
        // working output (device busy, e.g. another copy of the game): play silently instead of crashing.
        @try {
        for (int c : kUsedClasses)
            for (const Voice* v : {&tyres[size_t(c)].rollSlow, &tyres[size_t(c)].rollFast, &tyres[size_t(c)].skid})
                if (v->valid()) [v->player play];
        for (const Voice* v : {&wind, &scrape})
            if (v->valid()) [v->player play];
        for (size_t L = 0; L < EngineModel::kLayers; ++L) {
            if (idleOrOff[L].valid()) [idleOrOff[L].player play];
            if (on[L].valid()) [on[L].player play];
        }
        for (AVAudioPlayerNode* p : impactPlayers) [p play];
        for (OneShots* set : {&backfire, &absorber})
            for (AVAudioPlayerNode* p : set->players) [p play];
        } @catch (NSException* e) {
            std::fprintf(stderr, "fuelview: audio: %s; continuing without sound\n", e.reason.UTF8String);
            started = false;
            [engine stop];
        }
    }

    static void fire(OneShots& set, size_t index, float volume) {
        if (set.buffers.empty() || set.players.empty()) return;
        AVAudioPlayerNode* p = set.players[set.next++ % set.players.size()];
        p.volume = std::clamp(volume, 0.0f, 1.0f);
        [p scheduleBuffer:set.buffers[std::min(index, set.buffers.size() - 1)] atTime:nil options:AVAudioPlayerNodeBufferInterrupts completionHandler:nil];
        if (!p.playing) [p play];
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
    m.prevRpm = 0;
    m.rpmRate = 0;
    m.sincePop = 1;
    m.scrapeLevel = 0;
    m.compressionPeak.clear();
    m.sinceThump.clear();
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

    // Tyres: each wheel on the ground adds 1/n to the class of its surface; every class
    // crossfades in and out. Rolling fades from the slow to the fast loop over 4..40 m/s;
    // the skid loops follow each wheel's own slip.
    const size_t nWheels = std::max<size_t>(1, st.wheels.size());
    std::array<float, kSoundClasses> target{}, skidTarget{};
    bool anySurface = false;
    for (size_t i = 0; i < st.wheelSurface.size(); ++i) {
        if (st.wheelSurface[i] < 0) continue;
        anySurface = true;
        const size_t c = size_t(surfaceSoundClass(st.wheelSurface[i]));
        target[c] += 1.0f / float(nWheels);
        if (i < st.wheelSkid.size()) skidTarget[c] += st.wheelSkid[i] / float(nWheels);
    }
    if (!anySurface) {  // no surface information: dirt under every wheel on the ground
        target[kFallbackClass] = groundFrac;
        skidTarget[kFallbackClass] = st.skid * groundFrac;
    }
    const float blend = 1.0f - std::exp(-dt * kClassFade);
    const float slow = speed < 4.0f ? speed / 4.0f : std::max(0.0f, 1.0f - (speed - 4.0f) / 36.0f);
    const float fast = speed < 4.0f ? 0.0f : std::min(1.0f, (speed - 4.0f) / 36.0f);
    for (int c : kUsedClasses) {
        Impl::TyreVoices& t = m.tyres[size_t(c)];
        t.weight += (target[size_t(c)] - t.weight) * blend;
        t.skidLevel += (skidTarget[size_t(c)] - t.skidLevel) * blend;
        const bool hasFast = t.rollFast.valid();
        t.rollSlow.set(0.8f * t.weight * (hasFast ? slow : std::min(1.0f, speed / 4.0f)), 1.0f);
        t.rollFast.set(0.8f * t.weight * fast, 1.0f);
        t.skid.set(std::min(1.0f, t.skidLevel * 1.2f) * (speed > 1.5f ? 1.0f : speed / 1.5f), 1.0f + 0.3f * st.skid);
    }
    m.wind.set(0.7f * std::min(1.0f, speed / 35.0f), 1.0f);

    // Scrape: the loop while the body slides against the ground or an object.
    const float scrapeTarget = std::clamp((st.scrapeSpeed - kScrapeMin) * 0.15f, 0.0f, 1.0f);
    m.scrapeLevel += (scrapeTarget - m.scrapeLevel) * (1.0f - std::exp(-dt * (scrapeTarget > m.scrapeLevel ? 30.0f : 10.0f)));
    m.scrape.set(0.8f * m.scrapeLevel, 0.9f + 0.3f * std::min(1.0f, st.scrapeSpeed / 20.0f));

    // Backfire: a pop when the revs fall faster than P646 while the engine is high and off the throttle.
    const float rate = m.prevRpm > 0 ? (st.rpm - m.prevRpm) / dt : 0.0f;
    m.rpmRate += (rate - m.rpmRate) * (1.0f - std::exp(-dt / 0.05f));
    m.prevRpm = st.rpm;
    m.sincePop += dt;
    const float rpmNorm = (st.rpm - s.engine.idleRpm) / (s.engine.maxRpm - s.engine.idleRpm);
    if (!m.backfire.buffers.empty() && m.rpmRate < s.engine.popRateThreshold && rpmNorm > s.engine.popRpmMin &&
        throttle < s.engine.popThrottleMax && m.sincePop > m.popGap) {
        Impl::fire(m.backfire, size_t(std::rand()) % m.backfire.buffers.size(), 0.55f + 0.45f * float(std::rand() % 100) / 100.0f);
        m.sincePop = 0;
        m.popGap = s.engine.popInterval * (1.0f + 0.5f * float(std::rand() % 100) / 100.0f);
    }

    // Suspension thumps: a wheel that was compressing faster than P651 starts to rebound.
    if (m.compressionPeak.size() != st.wheelCompression.size()) {
        m.compressionPeak.assign(st.wheelCompression.size(), 0.0f);
        m.sinceThump.assign(st.wheelCompression.size(), 1.0f);
    }
    float thump = 0;
    for (size_t i = 0; i < st.wheelCompression.size(); ++i) {
        m.sinceThump[i] += dt;
        const float v = st.wheelCompression[i];
        if (v > 0) {
            m.compressionPeak[i] = std::max(m.compressionPeak[i], v);
        } else if (m.compressionPeak[i] > 0) {
            const float over = m.compressionPeak[i] - s.engine.absorberThreshold;
            if (over > 0 && m.sinceThump[i] > 0.3f) {
                const float vol = std::min(1.0f, over * kAbsorberGain * s.engine.absorberScale * 4.0f / float(nWheels));
                thump += vol;
                if (vol > 0.03f && !m.absorber.buffers.empty()) {
                    const size_t small = s.bigAbsorber ? m.absorber.buffers.size() - 1 : m.absorber.buffers.size();
                    Impl::fire(m.absorber, small ? std::min(small - 1, size_t(vol * float(small))) : 0, vol);
                    m.sinceThump[i] = 0;
                }
            }
            m.compressionPeak[i] = 0;
        }
    }
    if (thump > 0.25f && s.bigAbsorber) Impl::fire(m.absorber, m.absorber.buffers.size() - 1, std::min(1.0f, thump));

    // Impacts: a sharp loss of speed, or touching down after a jump.
    m.sinceImpact += dt;
    const float decel = (m.prevSpeed - speed) / dt;
    if (st.wheelsOnGround == 0) m.airTime += dt;
    if (m.sinceImpact > 0.3f) {
        if (m.prevGround == 0 && st.wheelsOnGround > 0 && m.airTime > kAirTime) {
            m.impact(0.3f + std::min(m.airTime, 2.0f) * 0.35f);
            m.sinceImpact = 0;
        } else if (st.impactSpeed > 2.5f) {  // the body hit something
            m.impact(0.2f + st.impactSpeed / 14.0f);
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
            std::printf("  engine: idle %.0f rpm, max %.0f rpm, gain P649 %.2f; %zu impact, %zu backfire, %zu absorber samples\n", e.idleRpm, e.maxRpm, e.gain,
                        v.impacts.size(), v.backfires.size(), v.absorbers.size());
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
            for (int c : kUsedClasses) {
                char label[24];
                std::snprintf(label, sizeof label, "c%d slow", c);
                line(label, v.tyres[size_t(c)].rollSlow);
                std::snprintf(label, sizeof label, "c%d fast", c);
                line(label, v.tyres[size_t(c)].rollFast);
                std::snprintf(label, sizeof label, "c%d skid", c);
                line(label, v.tyres[size_t(c)].skid);
            }
            line("wind", v.wind);
            line("scrape", v.scrape);
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
