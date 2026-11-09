// [DEVIATION, the user's request 2026-11-06] The AI opponents' nitrous (ai_nitro.hpp).
#include "physics/ai_nitro.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace fuel {

// The ghost's boost (race_ai.cpp calls it in the speed model 0x61dae0 after the cap and the acceleration are set, while
// the racer's tank pushes): the cap x (1 + aiSpeedGain x power x amount), the acceleration + thrust x power x amount.
// The corner and follow caps still apply after it.
void RaceDriver::nitroApply(float& cap, float& accel) const {
    const nitro::Params& p = nitro::params();  // [upgrades U1] nitroPower_ = the racer's power (its nitrous stage)
    cap *= 1.0f + p.aiSpeedGain * nitroPower_ * nitro_;
    accel += p.thrust * nitroPower_ * nitro_;
}

namespace nitro {

namespace {
float headingOf(const RaceLine& line, float s) {
    float dx = 0, dz = 1;
    line.at(s, &dx, &dz);
    return std::atan2(dx, dz);
}
float angleDeg(float a, float b) { return std::fabs(std::remainder(a - b, 6.2831853f)) * 57.29578f; }
}  // namespace

float pathTurn(const RaceLine& line, float s, float from, float to, float step) {
    if (line.path.size() < 2) return 0;
    const float h0 = headingOf(line, s + from);
    float worst = 0;
    for (float d = from + step; d <= to + 1e-3f; d += step) worst = std::max(worst, angleDeg(headingOf(line, s + d), h0));
    return worst;
}

bool AiNitro::decide(float dt, const AiView& v, const Params& p) {
    cool_ = std::max(0.0f, cool_ - dt);
    // 0 = leading the player by 150 m or more, 0.5 = level with the player, 1 = trailing by 150 m or more.
    const float a = std::clamp(0.5f + v.trail / 300.0f, 0.0f, 1.0f);
    const float jitter = float((v.seed * 7) % 5) - 2.0f;  // -2..2 degrees per racer
    const float straight = 8.0f + 10.0f * a + jitter;      // degrees allowed over the look-ahead
    const float tight = 28.0f;                             // a bend in the near stretch: never
    const float reserve = std::max(p.fraction(p.minStart), 0.55f * (1.0f - a));
    const float burstMax = 0.8f + 2.4f * a;                // s per burst
    bool want = false;
    if (!v.frozen && !v.finished && !v.wrecked && !v.airborne && v.speed > 6.0f && v.turnNear < tight) {
        if (want_)
            want = v.level > 0 && burst_ < burstMax && v.turnAhead < straight + 6.0f;
        else
            want = cool_ <= 0 && v.level >= reserve && (v.turnAhead < straight || (v.turnBehind > 25.0f && v.turnAhead < straight + 8.0f));
    }
    if (want) {
        if (!want_) ++bursts_, burst_ = 0;
        burst_ += dt, time_ += dt;
    } else if (want_) {
        cool_ = 1.0f + 1.5f * (1.0f - a);
    }
    want_ = want;
    return want;
}

bool autopilotNitro() {
    static const bool on = [] {
        const char* e = std::getenv("FUEL_NITRO_AUTOPILOT");
        return e && std::atoi(e) != 0;
    }();
    return on;
}

bool aiNitroStep(AiNitro& n, RaceDriver& ai, const RaceLine& line, const RaceDriver::Sense& s, const VehicleState& st, float dt, int seed, bool use,
                 const Params& p) {
    if (!use || !enabled() || !st.nitro.on) {
        ai.setNitro(0);
        return false;
    }
    AiView v;
    v.level = st.nitro.level;
    v.frozen = s.frozen;
    v.finished = s.finishRank >= 0 || ai.finishState() != 0;
    v.wrecked = st.wrecked;
    v.airborne = st.wheelsOnGround == 0;
    v.speed = std::hypot(s.vel[0], s.vel[2]);
    const float at = ai.progress();
    v.turnNear = pathTurn(line, at, 0, v.speed * 1.2f + 15.0f);
    v.turnAhead = pathTurn(line, at, 0, std::clamp(v.speed * 2.5f + 30.0f, 40.0f, 150.0f));
    v.turnBehind = pathTurn(line, at, -40.0f, 0.0f);
    v.trail = -s.playerAhead;  // playerAhead > 0: the racer leads
    v.seed = seed;
    const bool want = n.decide(dt, v, p);  // [upgrades U4] the racer's own Params (tank seconds, minimum start, recharge)
    ai.setNitro(st.nitro.amount, p.power);  // the ghost follows the tank (the last step's amount: the same rules as the vehicle), its power
    return want;
}

}  // namespace nitro
}  // namespace fuel
