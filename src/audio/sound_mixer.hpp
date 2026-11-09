// FUEL's sound output, ported from SoundManager_Z / WinSound_Z (DirectSound) as a software mixer.
//
// Decoded (notes/p41-audio.md):
//  - voice start 0x8340d0 -> 0x732ca0: refused when the source is farther than 1.5 x its max distance
//    from every camera (0x732960, distances from the owner object +0x88/+0x8c, default 6 / 140 m);
//  - per frame 0x8342a0 -> 0x732f40: gain = bus volume (SoundManager +0x20ec) x voice volume (+0x6c)
//    x 3D gain (0x733150); a positional voice beyond 1.5 x max distance is stopped;
//  - 3D gain 0x733150, per camera: d = distance in camera space; d <= min -> 1; min < d < max ->
//    min(1 - (d - min)/(max - min), max(0.05 max, min)/d); d >= max -> 0; the largest over cameras.
//    Pan = 0.9 x (camera-space x)/d averaged over the cameras (not for stereo samples);
//  - DirectSound: volume = trunc(868 ln g) mB clamped to [-10000, 0] (0x8346c0), pan = -868 ln(1 - p)
//    (p >= 0) or 868 ln(1 + p) mB clamped to +-10000 (0x834630), frequency = round(rate x pitch) Hz
//    clamped to 100..100000 (0x732e30, 0x834730); no doppler, no DirectSound 3D (positional maths is
//    the engine's own). Output 44.1 kHz stereo (0x833820 primary buffer).
// The resampler (linear) and the mixing precision are the port's (DirectSound's are not in FUEL.exe).
#pragma once

#include "core/sound.hpp"

#include <array>
#include <functional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fuel::snd {

// A decoded sample as float frames (mono or interleaved stereo).
struct Pcm {
    int rate = 44100, channels = 1;
    bool loops = false;
    std::vector<float> data;
    size_t frames() const { return data.size() / size_t(channels); }
    static std::shared_ptr<const Pcm> from(const Sound& s);
};

struct Vec3f {
    float x = 0, y = 0, z = 0;
};

// One camera: position and its right / up / forward axes (world).
struct Listener {
    Vec3f pos, right{1, 0, 0}, up{0, 1, 0}, forward{0, 0, 1};
};

// DirectSound's conversions (WinSound_Z), exposed for the checks.
int volumeMb(float gain);          // 0x8346c0
int panMb(float pan);              // 0x834630
float mbToAmplitude(int mb);       // 10^(mB / 2000)
// 3D gain for one camera at distance `d` (0x733150).
float distanceGain(float d, float minDist, float maxDist);

class Mixer {
public:
    static constexpr int kVoices = 80;  // WinSound_Z voice records (0x50)
    static constexpr int kRate = 44100;

    struct PlayParams {
        bool positional = true;  // flag 1 (else voice flag 0x40: no 3D)
        bool attenuate = true;   // flag 2 (else 0x80: no distance gain)
        float minDist = 6, maxDist = 140;  // owner +0x88 / +0x8c (0 / 0 -> 6 / 140)
        Vec3f pos;
        // PK27: voice flags 0x10000 with 0x400 (the dialog streams, 0x8329c0) play on the voice bus +0x20f4 instead of the
        // SFX bus (0x732f40). No port sound sets it: the only live dialog would be the player voice line 0x41ea80, whose
        // queue 0x41ebb0 has no live caller in retail.
        bool voiceBus = false;
        // [upgrades U5, DEVIATION] the engine stage's note (core/upgrade_visuals Plan::engineGainDb / engineShelfDb): a gain
        // and an RBJ low shelf at 200 Hz on this voice's resampled output (0 / 0 = FUEL's voice, untouched).
        float toneDb = 0, shelfDb = 0;
    };
    // [upgrades U5] the low shelf + gain of a voice (exposed for the checks): biquad coefficients b0 b1 b2 a1 a2 (a0 = 1) x gain.
    static std::array<float, 5> toneCoefficients(float toneDb, float shelfDb, float rate = float(kRate));

    // Starts `pcm`; returns a handle (>= 0) or -1 when no voice is free or the source is too far.
    int play(std::shared_ptr<const Pcm> pcm, const PlayParams& p);
    bool alive(int handle) const;
    void stop(int handle);
    void setVolume(int handle, float v);   // voice volume (0..1)
    void setPitch(int handle, float p);    // frequency = round(rate x p)
    void setPosition(int handle, Vec3f pos);
    // P96: the owner's distances follow its node scale (0x733080: +0x88 / +0x8c, 0 / 0 -> 6 / 140, x the length of the
    // world matrix's first column), re-read every frame by the 3D gain (0x733150).
    void setDistances(int handle, float minDist, float maxDist);
    void setListeners(const std::vector<Listener>& l);
    void setBusVolume(float v);            // SoundManager +0x20ec (the SFX volume)
    void setVoiceBus(float v);             // PK27: SoundManager +0x20f4 (the voice volume, vt+0x48)
    // PK17: WinSound_Z vf15 0x833cc0 (pause mask +0x20e0): bit 0 pauses every SFX voice (voice flag 2: the buffer
    // stops where it is and resumes from there), bit 1 the voice-bus (0x10) voices, bit 2 the dialog streams, bit 3 the
    // music voice. The port's mixer voices are all plain SFX (no dialogs; the music is the aux stream, never paused by
    // the menus' masks 3 / 7 / 0xb), so bit 0 pauses them all.
    void setPauseMask(uint32_t mask);
    uint32_t pauseMask() const { return pauseMask_; }
    void stopAll();
    int activeVoices() const;

    // Game thread, once per frame: 3D gain, pan, culling -> DirectSound volume / pan / frequency.
    void update();
    // Another source mixed in before the 16-bit clip (the music voice), called on the audio thread.
    void setAux(std::function<void(float*, float*, int)> f);
    // Audio thread: mixes `frames` stereo frames at 44.1 kHz (adds nothing to a voice that is not set up).
    void render(float* left, float* right, int frames);
    // P96: what render() produced (for the headless dump): the largest |sample| before the 16-bit clip and the
    // number of clipped samples (both channels), and the sample count.
    // P100: peak / clipped = the mix BEFORE the output stage (what FUEL's 16-bit buffer would clip); peakOut / clippedOut
    // = after the limiter; minGain = the lowest limiter gain, limited = samples with gain < 1 (both channels).
    struct Stats {
        float peak = 0, peakOut = 0, minGain = 1;
        uint64_t clipped = 0, clippedOut = 0, limited = 0, samples = 0;
        double sumSq = 0, sumSqOut = 0;  // for RMS before / after
    };
    // P100 [DEVIATION from FUEL, user decision 2026-11-05; notes/p100-limiter-cables.md]: settings.txt `audio_limiter =
    // on|off` (default on; FUEL_AUDIO_LIMITER overrides) replaces the hard clip of the final mix with a smooth
    // stereo-linked lookahead peak limiter (2.5 ms lookahead, release ~150 ms, ceiling -0.3 dBFS). Below the ceiling
    // the gain is exactly 1: the mix is only delayed by kLook samples.
    static constexpr int kLook = 110;
    static void setLimiterDefault(bool on);
    Stats takeStats();
    // P96: one line per live voice (frames, channels, loop, gains, position) for the headless dump's trace.
    std::string describe() const;

private:
    struct Voice {
        uint32_t serial = 0;
        bool used = false;
        std::shared_ptr<const Pcm> pcm;
        PlayParams p;
        float volume = 1, pitch = 1;
        // what the "DirectSound buffer" plays with (written by update, read by render)
        float ampL = 0, ampR = 0, step = 1;
        double cursor = 0;
        bool finished = false;
        // [upgrades U5] PlayParams::toneDb / shelfDb: the biquad (gain folded into b) and its state per channel
        bool tone = false;
        std::array<float, 5> tc{};
        float tx1[2] = {0, 0}, tx2[2] = {0, 0}, ty1[2] = {0, 0}, ty2[2] = {0, 0};
    };
    mutable std::mutex m_;
    std::array<Voice, kVoices> v_;
    uint32_t serial_ = 1;
    std::vector<Listener> listeners_;
    float bus_ = 1;
    float voiceBus_ = 1;  // PK27
    uint32_t pauseMask_ = 0;  // PK17
    std::function<void(float*, float*, int)> aux_;
    Stats stats_;
    // limiter state (audio thread, under m_)
    struct Limiter {
        std::array<float, kLook> dl{}, dr{};    // delay line (kLook - 1 samples)
        std::array<float, kLook> m{};           // windowed minimum of the requested gain, last kLook values
        std::array<float, kLook> req{};         // requested gain per sample
        std::array<long long, kLook + 1> dqIdx{};     // monotonic deque for the sliding minimum (indices, absolute)
        int dqHead = 0, dqCount = 0;
        long long n = 0;
        double sum = kLook;                     // boxcar sum of m
        int run1 = kLook;                       // consecutive m == 1
        float gs = 1;                           // released gain
        Limiter() { m.fill(1.0f), req.fill(1.0f); }
    } lim_;
    void limit(float* L, float* R, int frames);
    Voice* find(int handle);
    const Voice* find(int handle) const;
    bool audible(const PlayParams& p) const;  // 0x732960
};

}  // namespace fuel::snd
