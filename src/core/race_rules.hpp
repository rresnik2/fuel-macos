// The rules of one event, independent of rendering and physics: gates per racer, laps,
// timers and checkpoint bonuses, knock-outs, the helicopter of chopper races and the
// hunted racers of "catch AI" challenges, and the result. The caller moves the racers
// (player = racer 0) and hands their positions to update().
//
// Decoded (notes/p17-events.md, notes/p38-scripting.md): the game modes (0x4fe2d0), timers
// (EMD_End default 1200 s, AddMissionTime 4th value), checkpoint bonuses; p38: every racer
// goes through every checkpoint (0x4f6600 copies all of them, AI path points included) and
// the finish line closes each lap (0x5c1ca0 / 0x5c1d40); a gate is passed when the racer's
// move (previous position -> current + 10%) crosses the gate line (0x5c1ad0 / 0x75a440)
// within R of its centre, gate width 0x4fdea0 from the road under it, R = the widest
// half-width (0x5c1360), or when the record's byte +0 is set (rain-ai: an AI racer whose path
// cursor ran past the gate's checkpoint block, 0x6182b0; aiCheckpointBlock); knock-out
// eliminations 0x6217b0.
// PK07: the chopper races' helicopter flies FUEL's own model (race_helicopter.hpp: speed from the distance and time
// left, 2 x when the player is close, terrain-following cruise height, wobble, attitude); it lands (the player
// loses) when its race record passes the finish line.
// Approximated (marked "port" below): catch distance (FUEL: a vehicle contact, 0x51f6c0 ->
// 0x62fbc0) and the helicopter's path length (a polyline through the points, FUEL: the road route). Stars follow FUEL's career rule
// (0x630dd0 / 0x578b50, P39: rank 0/1/2 = 3/2/1 stars; medals: rank = the race's level, won = 1st); career progress lives in career.hpp.
#pragma once

#include "race_helicopter.hpp"
#include "races.hpp"

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace fuel {

struct RaceRules {
    using Vec3 = std::array<float, 3>;
    enum State { Countdown, Running, Finished };
    enum Outcome { Pending, Won, Lost };

    // 0x62c540: the gate radius before 0x5c1360 replaces it: 25 m, 15 m in baja scenarios (7, 8).
    // Used (with half-width = radius) when no road width is available.
    static constexpr float kGateRadius = 25.0f, kGateRadiusBaja = 15.0f;
    static constexpr float kCatchDistance = 6.0f;    // port: "touch" = centres within 6 m

    struct Racer {
        size_t next = 0;        // gate it heads for (route.size() once finished)
        double finished = -1;   // clock at the finish
        bool out = false;       // knocked out (knock-out races) or caught (hunts)
        double outAt = -1;
        Vec3 prev{};            // position at the previous update (0x5c1d40 keeps it at +0xe4)
        bool havePrev = false;
        // rain-ai: the race record's byte +0 (0x5c1d40: `RaceRec_TestGate() || byte +0` passes the current gate and
        // clears it). AI racers get it from 0x6182b0 (aiCheckpointBlock); the helicopter's is RaceHelicopter::forcePass.
        bool forcePass = false;
        bool forcePass1 = false;  // byte +1, set with it; cleared only by the record inits (no reader found, heli-fix)
    };
    // One gate of `route` (0x5c1360): centre and the two ends of its line (x, z).
    struct Gate {
        Vec3 centre{};
        float ax = 0, az = 0, bx = 0, bz = 0;
        float half = 0;
    };
    std::vector<Gate> gates;             // parallel to `route`
    float gateRadius = 0;                // R: a racer must be within R (2-D) of the gate centre

    Race race;
    int mode = Race::GmInvalid;          // Race::gameMode()
    std::vector<Vec3> route;             // the player's gates in order (per lap), then the finish
    std::vector<Vec3> aiPath;            // helicopter / hunted racers: start, path points, finish
    std::vector<Vec3> aiTargets;         // aiPath without its start (what hunted racers head for)
    std::vector<Racer> racers;           // [0] = player
    std::vector<size_t> finishOrder;     // PK16: racer indices in the order they crossed the finish (race +0x3d8, 0x5c27d0)
    int finishRank(size_t i) const {     // 0x5c27d0: the index in that list, -1 when not (yet) finished
        for (size_t k = 0; k < finishOrder.size(); ++k)
            if (finishOrder[k] == i) return int(k);
        return -1;
    }
    State state = Countdown;
    Outcome outcome = Pending;
    std::string reason;                  // why it ended
    double clock = -3.0;                 // seconds since the start (negative in the countdown)
    float timeLeft = 0;                  // timed modes: seconds left (0 = no limit)
    bool timed = false;
    // Helicopter chase (mode 2): PK07, the helicopter entry flown by RaceHelicopter (0x6202c0) towards the gates
    // (`heliNext` = its race record's passed gates); `heli` is its display position (flight position + wobble).
    bool helicopter = false;
    Vec3 heli{};
    RaceHelicopter hel;
    size_t heliNext = 0;
    Vec3 heliPrev{};                     // the record's previous position (0x5c1d40)
    bool heliHavePrev = false;
    bool heliLanded = false;             // its record passed the finish
    std::vector<size_t> knockedAt;       // knock-out: gate index of each elimination so far

    // Ground height at (x, z), set before begin(): points whose script height is more than
    // kSnap metres off the ground are put on it (FUEL snaps race points to the ground, 0x460220,
    // radius parameter 30.0 at 0x9dbf90; Inferred), and gates at y = 0 get a height.
    std::function<float(float x, float z)> ground;
    static constexpr float kSnap = 30.0f;
    // Full gate width at (x, z) (0x4fdea0; missionGateWidth below), set before begin(); unset:
    // every gate is 2 x kGateRadius wide.
    std::function<float(float x, float z)> gateWidth;

    // `racerCount` includes the player. `targetTime` (AddMissionTime 1st value) sets the helicopter's flight time.
    void begin(const Race& r, size_t racerCount, float targetTime);
    // Advances by dt with every racer's position (x, y, z); racers beyond the vector stay put.
    void update(double dt, const std::vector<Vec3>& positions);

    bool usesRoute(size_t i) const;      // racer i races the gates (false: helicopter, hunted racers)
    // PK07: one frame of 0x6202c0 (the mission is running): flight, wobble, gates of the helicopter's record, attitude.
    void heliStep(float dt, const Vec3& player);
    // 0x620460 (in heliStep): the 40 m waypoint step, the record's gate pass (0x5c1d40: line test or the record's
    // pass byte) and the next target. heli-fix vectors HP (EmuRun of 0x620460 with 0x5c1d40 / 0x5c1ad0 / 0x75a440).
    void heliRecord();
    bool soloTimed() const { return timed && racers.size() == 1; }
    double progress(size_t i, const Vec3& at) const;  // metres along the course (for positions)
    std::vector<int> positions(const std::vector<Vec3>& at) const;  // 1-based, racer order
    int playerPosition(const std::vector<Vec3>& at) const { return positions(at)[0]; }
    // medals: the race's level (race +0x3e8: 0 gold, 1 silver, 2 bronze; -1 unknown), set by the session (result line only).
    int level = -1;
    int stars(const std::vector<Vec3>& at) const;  // a won race: 3/2/1 for level 0/1/2 (0x630dd0, 0x578b50); challenge-only modes: 0
    std::string result(const std::vector<Vec3>& at) const;  // one line: outcome, time, position, reason
    const std::vector<Vec3>& lineFor(size_t i) const { return usesRoute(i) ? route : aiTargets; }
    // 0x5c1ad0 type 1: racer i, moving from `from` to `to`, passes gate g.
    bool passes(const Gate& g, const Vec3& from, const Vec3& to) const;
    // rain-ai, 0x6182b0 (asm 0x618497..0x6184d0): AI racer i's path cursor passed the checkpoint block of the line point at
    // `centre` five frames ago (RaceDriver::Command::checkpointBlock). When that point is the race record's target (x, z
    // within 5 m: squared distance < [0x996184] = 25) the record's bytes +0 / +1 are set (racer +0x11cc / +0x11cd), so the
    // next update passes the gate even if the racer's move missed the gate segment. Not for the player (FUEL's player has
    // no AI path); the caller only hands on opponents' blocks.
    void aiCheckpointBlock(size_t i, const Vec3& centre);

    // PK01: a position and a quaternion (x, y, z, w), as the race object keeps its start (+0x70 / +0x7c).
    struct Pose {
        Vec3 pos{};
        std::array<float, 4> quat{0, 0, 0, 1};
    };
    // 0x62d480 (offline): the race's start pose, the mission start with y on the ground (`ground`, 0x6407e0) and
    // the quaternion of EMD_SetStartRot (0x620b50 -> 0x644980, the whole rotation vector).
    Pose startPose() const;
    // The heading FUEL's vehicle code gets from a quaternion: (0, 0, 1) [0xa06350] turned by it (0x410510),
    // flattened (y = 0) and Quat_FromArc'd from (0, 0, 1) (0x529480 -> 0x6443a0): atan2(x, z), 0 when flat.
    static float headingOf(const std::array<float, 4>& q);
    // 0x62ce40, the transform of the "checkpoint respawn" request 0x5279c0 (the player's vehicle only, every
    // physics step): false while the race state (+0x48) is 0, 1, 5, 6 or 7 (no race, running, ended); in states
    // 2, 3 and 4 (before the start) true with the start pose when the elapsed clock (+0x244) is above 0, else
    // the attach slot of a running cut-scene that holds the player (0x539b30, `rtcSlot`; TELEPORT_HELICO), else
    // the start pose. The race state machine (0x633190) writes -15 to +0x244 every frame in states 2-4 and
    // Race_Start (0x62e9c1) -10, so before the start it is the cut-scene's slot or the start pose. Online races
    // (+0x3b8) ask 0x62d250 instead (`online`: its answer; the port has no online races). The boost-gauge reset
    // 0x4232d0 (player mover +0x2f5c / +0x2f62, read only by the debug car HUD) has no port counterpart.
    // (The name is FUEL's; no checkpoint is involved: CheckpointVolAgent_G 0x56fb00 writes Player_G +0x124 /
    // +0x130, which nothing live reads, and MissionManager_G has no listeners: notes/pk01.md.)
    static bool checkpointRespawnPose(int raceState, float elapsed, const Pose& start, const Pose* rtcSlot, Pose& out,
                                      const bool* online = nullptr);
};

class RoadGraph;
struct RoadTypeParams;
// 0x4fdea0: a mission gate's full width at (x, z) from the road there: the nearest road record
// (0x6d19f0 within 1000 m, then within 20 m and no farther than 2 m: squared distance <= 4) gives
// w = road type +0xc8 for types 0-5, 7, 8, 10, 11, 13, 14, 16 m for type 18, +0xc4 for the rest;
// width = 2 max(8, w + 2). Not on a road: 20 m.
float missionGateWidth(const RoadGraph& graph, const RoadTypeParams* types, size_t typeCount, float x, float z);
// 0x75a440: segments a-b and c-d intersect (x, z), with FUEL's handling of collinear segments.
bool segmentsCross(float ax, float az, float bx, float bz, float cx, float cz, float dx, float dz);

}  // namespace fuel
