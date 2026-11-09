// [DEVIATION, the user's request 2026-11-06] The AI opponents' nitrous (notes ~/re-work/fuel/notes/nitro.md). FUEL's AI
// has no boost; this file is the port's. It is kept apart from race_ai.* (FUEL's decoded ghost racer): race_ai only
// gets two hooks, RaceDriver::setNitro (the racer's tank amount this frame) and one line in its speed model that calls
// RaceDriver::nitroApply (defined here): while boosting, the ghost's speed cap and acceleration are raised.
//
// The racers use the same tanks as the player (DriveWorld, physics/nitro.hpp: 3 s, idle recharge, bonus fills). When
// to press the button (AiNitro::decide), from what the racer already knows:
//  - the path ahead (RaceLine, the AI's racing line): the largest heading change over a look-ahead that grows with the
//    speed; boost on straights and out of corners (a bend just behind, a straight ahead), never with a tight bend in the
//    near stretch (a burst under way stops when one comes up), never in the air, the countdown, wrecked or finished;
//  - the race against the player: trailing racers boost sooner, on gentler bends, and burn the whole tank; leading
//    racers keep half a tank in reserve and take short bursts on clear straights.
#pragma once

#include "physics/nitro.hpp"
#include "physics/race_ai.hpp"

namespace fuel::nitro {

// The largest heading change (degrees) of the path between `from` and `to` metres past s, against its heading at
// s + from, sampled every `step` metres (RaceLine::at directions).
float pathTurn(const RaceLine& line, float s, float from, float to, float step = 10.0f);

struct AiView {
    float level = 0;          // the tank, 0..1
    bool frozen = false, finished = false, wrecked = false, airborne = false;
    float speed = 0;          // m/s
    float turnNear = 0;       // degrees over the near stretch (speed x 1.2 + 15 m)
    float turnAhead = 0;      // degrees over the look-ahead (speed x 2.5 + 30 m, 40..150)
    float turnBehind = 0;     // degrees over the last 40 m (a corner just left)
    float trail = 0;          // metres the player leads this racer by (> 0: the racer trails the player)
    int seed = 0;             // per racer: a little variety in the straight limit
};

class AiNitro {
public:
    // This frame's button.
    bool decide(float dt, const AiView& v, const Params& p);
    bool want() const { return want_; }
    int bursts() const { return bursts_; }
    float time() const { return time_; }
    void reset() { *this = AiNitro{}; }

private:
    bool want_ = false;
    float burst_ = 0, cool_ = 0;
    int bursts_ = 0;
    float time_ = 0;
};

// The hook of RaceSession::drive (main.mm) for one racer: reads its tank (st.nitro) and the path, decides, sets the
// ghost's boost (RaceDriver::setNitro); returns the Nitro button of its DriveInput. `use` false (nitro off, or the
// player's autopilot without FUEL_NITRO_AUTOPILOT=1): no boost. `p` = the racer's own nitro::Params
// (DriveWorld::nitroParams: its nitrous stages' tank, recharge and power; the process's for a stock vehicle) [upgrades U4].
bool aiNitroStep(AiNitro& n, RaceDriver& ai, const RaceLine& line, const RaceDriver::Sense& s, const VehicleState& st, float dt, int seed,
                 bool use, const Params& p);
// FUEL_NITRO_AUTOPILOT=1: the player's autopilot (headless runs, films) boosts like an opponent.
bool autopilotNitro();

}  // namespace fuel::nitro
