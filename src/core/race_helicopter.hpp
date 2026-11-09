// PK07: the race helicopter of the chopper races (scenario 11, "helico"; 19 career events): FUEL's IAManipulator
// helicopter entry (0x1f8 bytes at IAManipulator +0x26d2c) flown frame by frame while the mission is running.
// Decoded in notes/p98/b006.md section 7, b051 (attitude), b068 (wobble), pk07.md (the corrections), emulated
// on FUEL.exe (fuelphysvec HC vectors):
//   0x620080 init, 0x6202c0 per frame, 0x620530 cruise height, 0x620370 wobble, 0x620460 waypoints,
//   0x620670 steering / speed, 0x620980 attitude.
// The gate bookkeeping of 0x620460 (the helicopter's race record passing gates) is done by RaceRules; the
// pieces here are the flight itself. heli-fix: the 40 m step's record byte +0 (H +0x78) is the record's "pass the
// current gate" flag, read and cleared by port_RaceRules_update 0x5c1d40 (`TestGate() || rec[0]`): the helicopter
// passes each checkpoint within 40 m (xz) of it, only the finish needs the line crossing.
#pragma once

#include <array>
#include <functional>

namespace fuel {

struct RaceHelicopter {
    using V3 = std::array<float, 3>;
    using Q4 = std::array<float, 4>;  // x y z w

    // Constants (FUEL.exe .rdata).
    static constexpr float kStartAhead = 100.0f;  // 0x620080: the start is 100 m along the mission rotation
    static constexpr float kStartUp = 3.0f;       // and 3 m above the ground
    static constexpr float kCruiseUp = 28.0f;     // 0x9dd3a0
    static constexpr float kWaypoint = 40.0f;     // 0x620460: within 40 m (xz) of the target
    static constexpr float kFar = 500.0f;         // 0x9dbfc4: the steering regime / climb limit boundary
    static constexpr float kClose = 150.0f;       // 0x9dca04: the player is this close (or ahead): 2 x speed
    static constexpr float kSmoothRate = 1.01f;   // 0x9dcef0 (Cam_SmoothV3)

    // Entry fields (offsets in the 0x1f8-byte entry).
    V3 target{};                // +0x00 (y = the cruise height once flying)
    V3 dir{};                   // +0x0c last steering direction (target - display position, normalised)
    float requiredSpeed = 0;    // +0x18 path length / time A
    float prevSpeed = 0;        // +0x1c
    float speed = 0;            // +0x20
    float timeLeft = 0;         // +0x24
    V3 vel{};                   // +0x28
    float cruise = 0;           // +0x34
    int waypoint = 0;           // +0x38
    float pitchRaw = 0;         // +0x3c
    float rollRaw = 0;          // +0x40 (stored by 0x620980, read by nothing in the entry's update)
    float phase = 0;            // +0x44 wobble phase
    V3 pos{};                   // +0x48 flight position
    V3 disp{};                  // +0x5c display position (position + wobble)
    Q4 quat{0, 0, 0, 1};        // +0x68 attitude
    // Record bytes +0x78 / +0x79 = record +0 / +1 (0x620460 sets both with the 40 m step when the record's target is
    // within 5 m of +0x00; Race_Start 0x62e650 does the same, 0x5c1c70 is another setter). +0 makes 0x5c1d40 pass
    // the record's current gate and is cleared there (RaceRules::heliRecord); +1 is only cleared by the record inits.
    bool forcePass = false;     // record +0 (H +0x78)
    bool forcePass1 = false;    // record +1 (H +0x79)
    V3 missionStart{}, missionEnd{};  // mission +0x0c / +0x18

    std::function<float(float x, float z)> ground;  // 0x6407e0

    // 0x620080. `start` = display start x/z (+ rotation applied by the caller), `startQuat` the mission rotation.
    void init(const V3& start, const V3& end, const Q4& startQuat, float pathLength, float timeA, const V3& missionStartPos);

    // The steps of 0x6202c0, in FUEL's order (timeLeft -= dt first).
    void cruiseHeight();                                    // 0x620530
    void wobble(float dt);                                  // 0x620370
    void waypointStep(int gateCount, const V3& recTarget);  // 0x620460's 40 m part (index + record flags)
    void velocity(float dt, const V3& player);              // 0x620670
    void integrate(float dt);                               // position += velocity dt
    void attitude(float dt);                                // 0x620980

    // Shared helpers (also used by the vectors).
    static Q4 fromArc(const V3& from, const V3& to);  // 0x6443a0: the rotation taking EBX (from) to EAX (to)
};

}  // namespace fuel
