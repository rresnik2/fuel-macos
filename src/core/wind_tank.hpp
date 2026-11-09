// P65: FUEL's wind tanks and the race weather that starts them, decoded from FUEL.exe
// (notes ~/re-work/fuel/notes/p65-storm-visuals.md).
//
// The WeatherManager (init 0x5bb680) owns, besides rain / snow / clouds / sand, two GPU particle tanks drawn around the
// camera with wind.vhl, both started at load (vf+0xc) and shown by the amount the weather apply 0x5bba30 gives them
// from w = clamp01(|weather wind +0xc4| / PP5460):
//   - SpecialEffectsWindTank_G (+0x148, ctor 0x5e4560, update vf05 0x5e47e0, constants vf10 0x5e4650; SubCode 0x19
//     vh_wind / ph_wind, FX atlas entry 8): amount a = w / 0.3 below 0.3, 1 - (w - 0.3) / 0.7 up to 0.6, then 0;
//   - SpecialEffectsTornadoDebris_G (+0x154, ctor 0x5e71a0, update vf05 0x5e7400, constants vf10 0x5e7280, vertices
//     vf07 0x5e7590, 512 particles; SubCode 0x1a vh_debris / ph_sfxScattering, FX atlas entry 0x1c): amount
//     b = (w - 0.3) / 0.7 from 0.3 to 0.6, then (w - 0.6) x 2.5.
// The particles sit in a unit cube that wraps (MapTankCoordinate) round a box of half size TankScale centred TankScale
// ahead of the camera, and drift with the wind (displacement += normalise(wind) x clamp(|wind|, lo, hi) x 0.3 dt).
// The SuperCell tank (0x5e3e80 ctor, vf04 0x5e3f70) has no caller in the shipped game.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace fuel::storm {

struct WindTankLook {
    int count = 0;           // tank +4 (vf+0xc): 512 debris; the wind tank's (0x5bb680 passes none: [I] 512)
    int atlas = 0;           // tank +0x4c: 8 wind, 0x1c debris
    float scale = 70;        // +0x38 TankScale: PP5273 / PP5503
    float particle = 0.3f;   // C_USER2.x ParticleScale: PP5272 / PP5502
    float frequency = 3;     // C_USER2.y WaveFrequency: PP5271 / PP5501
    float amplitude = 5;     // C_USER2.z WaveAmplitude: PP5270 / PP5500
    float distRatio = 0.6f;  // C_USER2.w DistanceScaleRatio: PP5276 / PP5506 (vh_wind uses 0.6)
    float speedMin = 0.3f, speedMax = 2;  // vf05: PP5274 / 5275, PP5504 / 5505
    static WindTankLook load(const std::function<float(int)>& P, bool debris);
};

// P90 [DEVIATION from FUEL, user decision 2026-11-05; notes ~/re-work/fuel/notes/p90-debris-smooth.md]: settings.txt
// `wind_debris = smooth | fuel` (default smooth; FUEL_WIND_DEBRIS=smooth|fuel overrides). FUEL's debris tank drifts
// 0.3 clamp(|wind|, 10, 20) TANK UNITS per second (227 m/s at TankScale 70) and its 40 m wave turns at ~32 rad/s, so
// each leaf jumps ~40 m every frame and cannot be followed (P88: that is FUEL's own behaviour). `smooth` keeps FUEL's
// tank (count, sizes, sheet, the box round the camera, the wind direction, the spread of the 40 m wave) but
//   - drifts in metres: 0.3 clamp(|wind|, lo, hi) m/s (3-6 m/s), i.e. FUEL's drift / TankScale;
//   - freezes the 40 m wave at FUEL's starting phase (10 |q| + 10 w: the same spread round the box) and adds a small
//     swirl (swirlRadius on the wave's own curve, turning at swirlRate x 0.5..1.5 per particle, +-) that also spins
//     the sprite;
//   - fades each particle by its distance to the box faces (fadeWidth) so the wrap never pops in view;
//   - keeps FUEL's exact motion near a tornado (weight 1 within fuelNear of a funnel base, 0 beyond fuelFar).
// `fuel` is FUEL's motion exactly (place()). The wind tank (vh_wind) is FUEL's in both modes (already smooth).
enum class DebrisMotion { Smooth, Fuel };
DebrisMotion debrisMotionFrom(const std::string& value, DebrisMotion fallback = DebrisMotion::Smooth);
const char* debrisMotionName(DebrisMotion m);
struct SmoothDebris {
    float swirlRadius = 1.5f;  // m, on the wave curve (c - s, s - c, s + c), |d/dangle| 1..2.24
    float swirlRate = 1.2f;    // rad/s (x 0.5..1.5 by |w|, the sign of w): the swirl and the sprite spin
    float fadeWidth = 10.0f;   // m: alpha = saturate(distance to the nearest box face / fadeWidth)
    float fuelNear = 100.0f, fuelFar = 200.0f;  // m (x z) from the nearest tornado base (funnel base radius 60, vortex 50)
};
// The weight of FUEL's motion for a camera `dist` metres (x z) from the nearest tornado base (< 0: none): 1 - the
// smoothstep from fuelNear to fuelFar; always 1 in DebrisMotion::Fuel.
float debrisFuelWeight(DebrisMotion m, float dist, const SmoothDebris& s = {});

// The two amounts 0x5bba30 gives the wind (a) and tornado debris (b) tanks for a weather wind speed.
std::array<float, 2> windTankAmounts(float windSpeed, float pp5460);

class WindTank {
public:
    WindTank(const WindTankLook& look, uint32_t seed);
    // vf05: `wind` = the tank's wind vector (+0x20, the weather's +0xc4 after the gusts, 0x5bba30).
    void update(float dt, const std::array<float, 3>& wind);
    const WindTankLook& look() const { return look_; }
    const std::array<float, 3>& displacement() const { return disp_; }  // +0x6c / +0x74 (C_USER1.xyz)
    // P90 (deviation, see DebrisMotion): the smooth mode's drift (tank units, = metres / TankScale) and its clock (s).
    const std::array<float, 3>& smoothDisplacement() const { return smoothDisp_; }
    float smoothClock() const { return clock_; }
    // vf07 (0x5e7590) vertex data per particle: unit position (x, y, z) and w in [-1, 1), and its atlas column 0..3
    // (u = column / 4 .. + 1 / 4 of the tank's atlas rect, the debris sheet's four pieces).
    struct Particle {
        float x, y, z, w;
        float column;
    };
    const std::vector<Particle>& particles() const { return parts_; }
    int drawn(float amount) const;  // 0x5de850: floor(amount x count)
    // P88: vh_wind / vh_debris (wind.vhl) for one particle on the CPU: the wrapped world centre (MapTankCoordinate +
    // the sin displacement) and the sprite angle, for `adj` = C_USER0 (eye + camera rotation x (0, 0, TankScale)).
    struct Placed {
        std::array<float, 3> pos;
        float angle;
        std::array<int, 3> cell;  // the wrap cell of MapTankCoordinate (changes when the particle wraps)
        float alpha = 1;          // P90: the smooth mode's face fade (FUEL: 1, ph_sfxScattering reads no vertex alpha)
    };
    Placed place(const Particle& q, const std::array<float, 3>& adj) const;
    // P90 (deviation): the smooth mode's placement (the GPU's vsWindTank does the same).
    Placed placeSmooth(const Particle& q, const std::array<float, 3>& adj, const SmoothDebris& s = {}) const;

private:
    WindTankLook look_;
    std::array<float, 3> disp_{};
    std::array<float, 3> smoothDisp_{};  // P90
    float clock_ = 0;                    // P90
    std::vector<Particle> parts_;
};

// AddMissionParam's weather strings (0x5759c0, upper-cased): SUN zeroes cloud / rain / snow / wind; TOR the tornado;
// C# cloud, R# rain, S# snow, W# wind, G# fog (percent x 0.01).
struct WeatherToken {
    bool tornado = false, sun = false;
    std::optional<float> cloud, rain, snow, wind, fog;
};
WeatherToken parseWeatherToken(const std::string& s);
// 0x632250: the race weather runs from the start value to the end value by t = clamp(|p - start|^2 / |finish - start|^2,
// 0, 1) in x z (race +0x250 start, +0x25c finish; p the player); the wind (race +0x2f8 / +0x2fc) overrides the weather
// wind (+0x118, +0x100 = w; 0x5b89e0 speed = PP5460 w).
float raceWeatherProgress(const std::array<float, 3>& start, const std::array<float, 3>& finish, const std::array<double, 3>& p);
// The race's wind override: nullopt unless both strings give a wind (W# or SUN): AddMissionParam 0x575b00 sets the
// mission flag +0x1b0 only when the start and the end wind are both >= 0 (P83, asm 0x575f70).
std::optional<float> raceWind(const std::string& start, const std::string& end, float t);

}  // namespace fuel::storm
