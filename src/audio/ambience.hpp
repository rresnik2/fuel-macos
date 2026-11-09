// FUEL's world ambience, ported from Game_SoundMgr (vtable 0x9ae734): its pool of 32 sound emitters and
// the procedural ambience it drives every frame.
//
// Decoded (notes/p45-audio-ambience.md sections 2-4):
//  - 0x4e9980: 32 emitters, each an "IT_SOUND" object (template hash 0x669160("IT_SOUND") = the USA1 Lod_Z
//    73c7706b whose sound bank holds symbols 0x63..0x7d, 0xf7..0xfb), owner distances 0 / 191 m.
//    0x4e9da0 starts a sound on the first emitter whose voice is free and hands back {slot, serial};
//    0x4ead30 = that emitter still plays it; 0x4e9e80 stops it. The descriptor (0x4ea4a0) defaults to
//    volume 1, pitch 1, 0 / 192 m, flags 0x4f; 0x40 = not positional.
//  - 0x4e9b70 per frame (not paused): 0x4e81e0, the category faders, the emitters (0x4eabf0).
//  - 0x4e81e0 ambience, around camera 0:
//     altitude a = clamp((y - 2500) x 0.0005) above 2500 m;
//     wind w = clamp(|wind| / PP5460, 0.1, 1); beds 0x6a / 0x6b / 0x6c crossfade over w (two at a time:
//       w < 0.5: (1 - 2w, 2w, 0), else (0, 1 - 2(w - 0.5), 2(w - 0.5))) x gain A; bed 0x6d = a x A;
//     5 BirdAnimal (0x4e7c80, symbols 0x75..0x79) and 4 FrogAnimal (0x4e7f00, 0x7a..0x7d) one-shots;
//     day bed 0x73 = (1 - forest)(1 - 10 rain) day x patch, night bed 0x74 = patch (1 - day)(1 - forest)
//       (1 - 10 rain), both 0 above 2500 m or with snow, x gain B.
//    forest = GenWorld_ForestDensity 0x6bd200 at the camera, patch = 0x4e8a10 (|F_fd8 + F_1fac| / 2),
//    day = 0x5b98d0 (0 before 05:30, ramps to 1 at 06:30, 1 to 17:30, 0 at 18:30).
//    Beds (0x4e79d0) move to their target at 1/s (0x4e9850), start a non-positional loop on an emitter
//    when above 0, set its volume, stop at 0.
//  - Gains: A = 0x4e7820 = master x Amb0 (M+0xa0) x Amb3 (M+0xac) x duck(PP2201); B = 0x4e78c0 = master x
//    Amb0 x Amb5 (M+0xb4) x duck(PP2203); duck(v) = 1 below 1 m/s of player speed, falling linearly to the PP
//    value at 30 m/s (factor 1 - (1 - PP)(1 - (30 - v)/29)).
//  - Birds: below 2000 m, forest > 0.1, day >= 0.1. Armed with timer = 1 + (10 rain + 5) rand forest when the
//    sky is not overcast (cloud < 0.99), the wind is light (w <= 0.6) and no mission runs; otherwise 10000 s.
//    Then a 5 s one-shot at the player +- 25 m (x, z), max 50 m, volume B.
//    Frogs: below 1500 m, forest > 0.1, rain != 0; timer = 1 + ((1 - rain) 10 + 5) rand forest; max 100 m.
//  - MissionHUD 0x4b3da0: start countdown beeps, 0xf7 at each new number, 0xf8 at "GO" (not positional).
//  P49 (notes/p49-audio3.md):
//  - Rain 0x5e10e0 (SpecialEffectsSplashTank_G update): while the rain amount drawn = floor(rain x 1024) is not 0,
//    two non-positional loops 0x68 (light) and 0x69 (heavy) run with volumes (1 - f) G / 2 and f G / 2, f = drawn / 1024,
//    G = 0x5e09d0 = master x Amb0 x Amb2 x duck(PP2200); both stop when it is 0.
//  - Thunder 0x5e32a0 (SpecialEffectsThunder_G update, amount = the cloud cover): below 0.95 nothing; from 0.95 a
//    non-positional loop 0x67 at G' clamp(2 (cloud - 0.95)), G' = 0x5e2b90 = master x Amb0 x Amb1 x duck(PP2202);
//    a timer U(PP5230, PP5231) s: on expiry a lightning bolt (0x5e2d40) and, with probability PP5246, a second one;
//    the bolt lies at (cos a - sin a, sin a + cos a) r from the tank centre (a uniform, r = U(PP5234, PP5235)),
//    and one of 0x63..0x66 (trunc(rand x 3.9999)) plays there, volume G', 0 / 30000 m, positional. The bolt
//    itself (flash, mesh) belongs to the effects port: `onLightning` is called for each bolt.
//  - Decor fires (IT_DECORMESH_G activation 0x516cb0 registers a node whose class bank has symbol 0, 5 / 50 m;
//    0x4e9b70 visits a quarter of the list per frame: the camera beyond the node's 50 m stops the voice, else
//    the bank symbol 0 loop starts with flags 0xf). Port: `AmbientInput::decor` = the AutoMesh spawns whose
//    templates are IT_DECORMESH_G (clone.tsc) with a bank symbol 0 (USA1: DebrisFeu_* fire crackle loop).
//  - Event one-shots: 0xfa (Teleport_Queue 0x57e650, when in game) = teleportQueued(). 0xf9 (MissionHUD 0x4b36f0,
//    called per HUD frame by 0x4b34c0 while mission +0x198 is 2 or 3) and 0xfb (no start site in FUEL.exe) not ported.
// Not ported: beds 0x6e..0x72 (no writer of their targets in FUEL.exe).
#pragma once

#include "audio/sound_mixer.hpp"
#include "audio/vehicle_sound.hpp"

#include <array>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace fuel::snd {

struct AmbientInput {
    Vec3f camera;              // camera 0 (DAT_00a7c084 +0x24 -> +0xd4 +0x30)
    bool player = false;       // 0x421f60
    Vec3f playerPos;
    float playerSpeed = 0;     // 0x422290: |vehicle +0x820|
    float cloud = 0, rain = 0, snow = 0;  // weather +0xb0, +0xb8, +0xbc
    std::array<float, 3> wind{};          // weather +0xc4..+0xcc
    double timeOfDay = 8 * 3600;          // seconds
    float forest = 0;          // 0x6bd200 at the camera
    float patch = 0;           // 0x4e8a10 at the camera
    bool mission = false;      // the mission state is not 0 / 1 (0x62abd0 +0x48)
    bool paused = false;       // DAT_00a7c094 +4
    // Decor objects with a sound of their own (IT_DECORMESH_G, bank symbol 0): position and the template's
    // name CRC (the key of Ambience::setDecorBank).
    struct Decor {
        Vec3f pos;
        uint32_t kind = 0;
    };
    std::vector<Decor> decor;
};

float dayFactor(double timeOfDay);  // 0x5b98d0
// 0x4e9850: `cur` towards `tgt` at `rate` per second.
float approach(float cur, float tgt, float rate, float dt);

class Ambience {
public:
    // `bank` = the IT_SOUND samples by symbol; `rand01` = the game's uniform [0, 1).
    Ambience(std::array<std::shared_ptr<const Pcm>, 0x100> bank, const Tuning& t, std::function<float()> rand01);
    void update(float dt, const AmbientInput& in, const GameMix& mix, Mixer& out);  // 0x4e9b70 -> 0x4e81e0
    void countdown(int number, Mixer& out);  // MissionHUD 0x4b3da0: hud::countdown(c) each frame (-1 = none)
    void stopAll(Mixer& out);
    // P49: the symbol-0 sound of each decor template (name CRC -> sound).
    void setDecorBank(std::unordered_map<uint32_t, std::shared_ptr<const Pcm>> b) { decorBank_ = std::move(b); }
    // Ground height at (x, z) for the thunder bolts (0x445500); without it the camera height is used.
    void setGround(std::function<float(float, float)> g) { ground_ = std::move(g); }
    // Called for every lightning bolt the thunder timer creates (0x5e2d40): the hook for the effects port's flash.
    std::function<void(const Vec3f&)> onLightning;
    // Teleport_Queue 0x57e650: event symbol 0xfa (non-positional one-shot).
    void teleportQueued(Mixer& out);
    // Rain / thunder state for the checks.
    int rainSlots[2] = {-1, -1}, rainSerial[2] = {-1, -1};  // 0x68 / 0x69
    int thunderSlot = -1, thunderSerial = -1;              // 0x67
    float thunderTimer = 0;                                // +0x78
    int strikes = 0, lastThunderSymbol = -1;
    Vec3f lastStrike;
    size_t decorVoices() const { return decorLive_.size(); }  // decor sources in range of the list
    size_t decorPlaying(const Mixer& out) const {            // of them, with a live voice
        size_t n = 0;
        for (const auto& [k, d] : decorLive_) n += d.voice >= 0 && out.alive(d.voice);
        return n;
    }

    struct Bed {
        float cur = 0, tgt = 0, up = -1, down = -1;  // +0, +4, +8, +0xc (rates <= 0: jump)
        int slot = -1, serial = -1, symbol = -1;     // +0x10, +0x14, +0x18
    };
    struct Animal {
        bool frog = false;
        bool active = false;     // +0xc
        Vec3f pos;               // +0x10
        float playFor = 0;       // +0x1c
        float timer = -1;        // +0x20
        int slot = -1, serial = -1, symbol = -1;  // +0x4 handle, +0x24
    };
    std::array<Bed, 4> wind;      // +0x2c, +0x48, +0x64, +0x80 (0x6a..0x6d)
    Bed day, night;               // +0x290 (0x73), +0x2ac (0x74)
    std::array<Animal, 9> animals;  // 5 birds (+0x128, stride 0x28), 4 frogs (+0x1f0)
    float gainA = 0, gainB = 0;   // the last 0x4e7820 / 0x4e78c0

    struct Emitter {
        int voice = -1;   // mixer handle (+0x68)
        int serial = -1;  // +0x64
        float volume = 1; // +0xa0
    };
    std::array<Emitter, 32> emitters;  // +0x2d0
    bool playing(int slot, int serial, const Mixer& out) const;  // 0x4ead30

private:
    std::array<std::shared_ptr<const Pcm>, 0x100> bank_;
    const Tuning& t_;
    std::function<float()> rnd_;
    std::unordered_map<uint32_t, std::shared_ptr<const Pcm>> decorBank_;
    std::function<float(float, float)> ground_;
    struct DecorVoice {
        int voice = -1;
        bool seen = false;
        uint32_t kind = 0;
        Vec3f pos;
    };
    std::unordered_map<uint64_t, DecorVoice> decorLive_;
    uint32_t frame_ = 0;
    void updateRain(const AmbientInput& in, const GameMix& mix, Mixer& out);                    // 0x5e10e0
    void updateThunder(float dt, const AmbientInput& in, const GameMix& mix, Mixer& out);       // 0x5e32a0
    void updateDecor(const AmbientInput& in, Mixer& out);                                       // 0x4e9b70 decor list
    int nextSerial_ = 0;  // +0x2c8
    int lastCount_ = -1;  // MissionHUD +0xdac
    float gain(const GameMix& mix, int cat, int pp, float speed, bool player) const;  // 0x4e7820 / 0x4e78c0
    bool start(int symbol, const Mixer::PlayParams& p, float volume, Mixer& out, int& slot, int& serial);  // 0x4e9da0
    void stop(int& slot, int& serial, Mixer& out);  // 0x4e9e80
    void updateBed(Bed& b, float dt, Mixer& out);   // 0x4e79d0
    void updateAnimal(Animal& a, float dt, float density, float camY, const AmbientInput& in, float windClamped, Mixer& out);
};

}  // namespace fuel::snd
