// CUSTOM VEHICLES (port deviation): see ev_whine.hpp.
#include "audio/ev_whine.hpp"

#include <cmath>
#include <memory>

namespace fuel::snd {

namespace {

constexpr int kRate = 44100;

// One second of the motor at `rpm`: orders 2 / 4 / 11 of the shaft frequency with the given weights, plus a little
// deterministic noise (the inverter's hiss) so the tone is not a pure sine. Whole-hertz frequencies loop seamlessly.
std::shared_ptr<const Pcm> tone(float rpm, float w2, float w4, float w11, float noise, float level) {
    auto p = std::make_shared<Pcm>();
    p->rate = kRate;
    p->channels = 1;
    p->loops = true;
    p->data.resize(kRate);
    const double shaft = std::max(1.0, double(rpm) / 60.0);
    const double f2 = std::round(2 * shaft), f4 = std::round(4 * shaft), f11 = std::round(11 * shaft);
    const double fv = 3.0;  // a slow beat between the two inverter sidebands
    uint32_t seed = 0x9e3779b9u ^ uint32_t(rpm);
    double lp = 0;
    for (int i = 0; i < kRate; ++i) {
        const double t = double(i) / kRate;
        seed = seed * 1664525u + 1013904223u;
        const double n = (double(seed >> 8) / double(1u << 24)) * 2 - 1;
        lp += 0.08 * (n - lp);  // soft hiss
        const double s = w2 * std::sin(2 * M_PI * f2 * t) +
                         w4 * (0.7 * std::sin(2 * M_PI * f4 * t) + 0.3 * std::sin(2 * M_PI * (f4 + fv) * t)) +
                         w11 * std::sin(2 * M_PI * f11 * t) + noise * lp;
        p->data[size_t(i)] = float(level * s);
    }
    return p;
}

}  // namespace

void synthesizeEvBank(VehicleBank& b) {
    const EngineModel& e = b.engine;
    // idle: a faint hum of the cooling pump / inverter at the idle rpm
    b.bySymbol[2] = tone(std::max(e.idleRpm, 300.0f), 0.5f, 0.15f, 0.0f, 0.2f, 0.08f);
    for (int L = 1; L < EngineModel::kLayers; ++L) {
        const float rpm = e.nativeRpm[size_t(L)];
        b.bySymbol[size_t(L + 2)] = tone(rpm, 0.35f, 0.25f, 0.10f, 0.25f, 0.30f);  // off load (coasting / regen)
        b.bySymbol[size_t(L + 8)] = tone(rpm, 0.30f, 0.80f, 0.25f, 0.20f, 0.45f);  // on load
    }
    for (uint32_t s = snd::BackfireFirst; s <= snd::BackfireLast; ++s) b.bySymbol[s].reset();
}

}  // namespace fuel::snd
