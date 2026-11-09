// [DEVIATION, the user's request 2026-11-06] The nitrous sound (notes nitro.md). FUEL has no boost sound (its BOOST was
// dead data), and its banks have nothing fitting, so the port synthesizes it in code at start-up (no audio files):
//  - a seamless 1.6 s loop: band-passed noise (the gas hiss, ~2.2 kHz), a low burn (brown noise under ~130 Hz) and a
//    faint injector whine (620 Hz with a slow vibrato);
//  - the ignition, a 0.45 s one-shot as a boost kicks in: a noise burst whose low-pass sweeps 4 kHz -> 300 Hz and a
//    falling 70 -> 45 Hz thump;
//  - the blow-off, a 0.35 s high-passed "pssh" as it ends.
// Played through FUEL's mixer (sound_mixer.*) as positional voices at the vehicle (FUEL's default 6 / 140 m distances),
// one set per vehicle: the player's and the opponents' (quieter).
#pragma once

#include "audio/sound_mixer.hpp"

#include <array>
#include <memory>

namespace fuel::snd {

struct NitroBank {
    std::shared_ptr<const Pcm> loop, ignition, release;
    static const NitroBank& get();  // built once (deterministic noise: the same bytes every run)
};

// --audio-check: the bank (lengths, peaks, a seamless loop, nothing invalid) and a voice through a mixer over a boost
// (ignition + loop on, loop volume follows the amount, release + loop off; nothing for amount 0). Returns failures.
int nitroSoundCheck();

class NitroVoice {
public:
    // One frame: `amount` = the boost applied 0..1 (VehicleState::nitro.amount), `pos` = the vehicle (world), `speed`
    // m/s, `gain` = 1 for the player, less for opponents. Starts, steers and stops the voices on `mix`.
    void update(float dt, float amount, const std::array<double, 3>& pos, float speed, float gain, Mixer& mix);
    void stop(Mixer& mix);
    bool playing() const { return loop_ >= 0; }
    int starts() const { return starts_; }

private:
    int loop_ = -1;
    float was_ = 0;
    int starts_ = 0;
};

}  // namespace fuel::snd
