// The rules of one event, independent of rendering and physics: gates per racer, laps,
// timers and checkpoint bonuses, knock-outs, the helicopter of chopper races and the
// hunted racers of "catch AI" challenges, and the result. The caller moves the racers
// (player = racer 0) and hands their positions to update().
//
// Decoded (notes/p17-events.md): the game modes (0x4fe2d0), timers (EMD_End default
// 1200 s, AddMissionTime 4th value), checkpoint bonuses, AI path points (flag 4).
// Approximated (marked "port" below): gate radius, knock-out and catch distances,
// helicopter speed and the "keep close" distance, stars by position.
#pragma once

#include "races.hpp"

#include <array>
#include <string>
#include <vector>

namespace fuel {

struct RaceRules {
    using Vec3 = std::array<float, 3>;
    enum State { Countdown, Running, Finished };
    enum Outcome { Pending, Won, Lost };

    static constexpr float kGateRadius = 25.0f;      // port: the scripts give no gate width
    static constexpr float kCatchDistance = 6.0f;    // port: "touch" = centres within 6 m
    static constexpr float kHeliKeepClose = 500.0f;  // port: farther than this for kHeliGrace seconds loses
    static constexpr float kHeliGrace = 10.0f;

    struct Racer {
        size_t next = 0;        // gate it heads for (route.size() once finished)
        double finished = -1;   // clock at the finish
        bool out = false;       // knocked out (knock-out races) or caught (hunts)
        double outAt = -1;
    };

    Race race;
    int mode = Race::GmInvalid;          // Race::gameMode()
    std::vector<Vec3> route;             // the player's gates in order (per lap), then the finish
    std::vector<Vec3> aiPath;            // helicopter / hunted racers: start, path points, finish
    std::vector<Vec3> aiTargets;         // aiPath without its start (what hunted racers head for)
    std::vector<Racer> racers;           // [0] = player
    State state = Countdown;
    Outcome outcome = Pending;
    std::string reason;                  // why it ended
    double clock = -3.0;                 // seconds since the start (negative in the countdown)
    float timeLeft = 0;                  // timed modes: seconds left (0 = no limit)
    bool timed = false;
    // Helicopter chase (mode 2): the helicopter flies aiPath at a constant speed.
    bool helicopter = false;
    Vec3 heli{};
    float heliSpeed = 0, heliTravelled = 0, heliFar = 0;
    bool heliLanded = false;
    std::vector<size_t> knockedAt;       // knock-out: gate index of each elimination so far

    // `racerCount` includes the player. `targetTime` (AddMissionTime 1st value) sets the helicopter's flight time.
    void begin(const Race& r, size_t racerCount, float targetTime);
    // Advances by dt with every racer's position (x, y, z); racers beyond the vector stay put.
    void update(double dt, const std::vector<Vec3>& positions);

    bool usesRoute(size_t i) const;      // racer i races the gates (false: helicopter, hunted racers)
    bool soloTimed() const { return timed && racers.size() == 1; }
    double progress(size_t i, const Vec3& at) const;  // metres along the course (for positions)
    std::vector<int> positions(const std::vector<Vec3>& at) const;  // 1-based, racer order
    int playerPosition(const std::vector<Vec3>& at) const { return positions(at)[0]; }
    int stars(const std::vector<Vec3>& at) const;  // port: 3/2/1 for 1st/2nd/3rd when won or placed
    std::string result(const std::vector<Vec3>& at) const;  // one line: outcome, time, position, reason
    const std::vector<Vec3>& lineFor(size_t i) const { return usesRoute(i) ? route : aiTargets; }
};

}  // namespace fuel
