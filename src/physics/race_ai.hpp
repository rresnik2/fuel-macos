// FUEL's AI racers (P29, notes p29-ai.md), decoded from FUEL.exe.
//
// FUEL drives an opponent as a kinematic "ghost" (the IA racer record, IAManipulator_G 0x621930 ->
// IA_RacerUpdate 0x619650) that runs along the racing line at a speed of its own; the vehicle follows:
//  - near the player (rank < 4 and within 150 m, Pilot_PhysicsSwitch 0x5cb6f0) the vehicle is simulated
//    and pulled to the ghost by guided target forces (0x522830 AI branch: lateral position pull,
//    velocity match, heading torque), with throttle and brake from the ghost's speed (0x5c97d0) and no
//    steering input;
//  - otherwise the vehicle is placed on the ghost every frame.
// The line: the checkpoint path cut into blocks (IAPath_BuildBlocks 0x627590), sampled every 10 m for
// 200 m ahead (0x6182b0 -> 0x6291f0) with a lateral corridor per sample; other racers narrow the
// corridor (0x61a080, 0x61a9d0); a min-curvature relaxation through the corridor gives the line and
// the aim point 5..20 m ahead (0x61acb0), and the corners in the next 100 m (> 10 deg per sample)
// give a speed cap from a radius table (0x61dcf0, table set at 0x61e540).
// Speed (0x61dae0 -> 0x616ed0 / 0x617380 / 0x617610 / 0x617000, 0x61de60): every racer races against
// the event's target time (AddMissionTime -> mission +0x80..+0x88, one per level), its class adding a
// margin; the cap is the distance left over the time it has left, the time budget corrected every
// 5 s from the race clock (late: 10 % faster and a shorter budget); class 2 adjusts to the player
// (the rubber band); acceleration 27.78 / (0-100 time) less 0.1 v, near the player the player's
// vehicle's; above the cap the speed decays by 0.9985 per ms. Ghost motion 0x61e080 / 0x61be00, heading 0x61c4b0,
// stuck/reset 0x61c970.
#pragma once

#include "physics/vehicle_physics.hpp"
#include "core/roads.hpp"
#include "core/rider_anim.hpp"  // PK31: AiLean

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace fuel {

// Tables of FUEL.exe the AI uses, read from the player's FUEL.exe at run time (never stored in the port).
struct AiTables {
    // Corner speed (m/s) by turn radius in 5 m bins (0..95 m; the last bin also for >= 100 m): the
    // constants IA_RacerInitCornerTable 0x61e540 stores at racer+0x10b0..+0x10fc.
    std::array<float, 20> cornerSpeed{};
    // Road-route cost contexts of the path manager (0x4efad0 -> 0x4ef600; .data 0xa02c88 + (row-1)*0x40 +
    // col*0x20): speeds c0..c5 (tarmac, forest/ATV, double/bump, trails 4/5, uphill, downhill) and +0x1c
    // "type 3 forbidden". The AI racers use row 6 (0x617fc0): col 0 main path, col 1 variant.
    std::array<std::array<std::array<float, 6>, 2>, 6> roadSpeed{};
    std::array<std::array<bool, 2>, 6> roadForbid3{};
};
std::optional<AiTables> loadAiTables(const std::string& fuelExe, std::string* error = nullptr);

// The racing line as the AI sees it: FUEL's path blocks between the checkpoints (IAPath_BuildBlocks
// 0x627590) flattened to a polyline with a corridor half-width per stretch: plain blocks (straight, 30 m),
// road blocks where both ends of a stretch snap to the road network (the road, its flat half-width,
// route 0x6d95e0 / lanes 0x624a20), checkpoint blocks (the gate) and the finish block (100 m on).
// Also the plain start -> targets polyline the race rules measure progress on.
struct RaceLine {
    std::vector<std::array<float, 3>> points;  // start, then each target
    std::vector<float> along;                  // path length from the start to each point
    float finishRun = 100;                     // IAPathBlockFinish: the line goes on 100 m past the finish
    // PK16: the finish block runs along the last checkpoint's direction (IAPath_BuildBlocks 0x627590: from the gate's
    // exit, + dir * 100); unset: the last path segment's direction.
    std::array<float, 2> finishDir{0, 0};
    bool hasFinishDir = false;
    void setFinishDir(float dx, float dz);
    std::array<float, 2> finishDirection() const;  // unit (x, z) of the finish block
    // PK16 0x624310 / 0x624490 (IAPathBlockFinish vf04 / vf05): a copy whose finish block is the parking line from
    // the gate's exit A (centre + dir) to T = A + dist * dir + 6 m sideways (side by the parity of `rank`).
    // `dist` = 6 (N - rank + 1) + 5 (0x61b630); rank < 0: no sideways offset (+0x30 == -1).
    RaceLine parkingLine(int rank, float dist) const;
    // PK16 0x628c40 + 0x5c21a0 (racer record +0x170, the HUD's distance left and the race order's key): from the
    // path position `s` of the racer at (x, z): max(S1, S2) where S1 = the blocks' lengths left (the finish block 0)
    // + finish dir . (the foot point - (x, z)) and S2 = the straight distance to the finish block's end (100 m past
    // the gate's exit); then max with the straight length through the remaining gates (0x5c2070, `straight`) and
    // at most 20 x that. `straight` < 0: only the path part (no clamp).
    float distanceLeft(double x, double z, float s, float straight) const;
    float gateHalfWidth = 12.5f;               // IAPathBlockCheckpoint half-width near a gate (approximation, notes)
    // The AI's path (blocks flattened): points, distance along, half-width of the stretch from each point.
    std::vector<std::array<float, 2>> path;
    std::vector<float> pathAlong, pathHalf;
    std::vector<float> gateAlong;              // distance along the path of each target
    // PK19: per spline block (kind 5, path order) its middle-lane edge list (IAPathBlockSpline +0xc0 u16, bit 15 = walked
    // backwards; count +0x3e; 0x6255d0 -> 0x6256f0 flag 0x20): (edge index, forward). Filled by routeOnSplineGraph.
    std::vector<std::vector<std::pair<int, bool>>> splineEdges;
    float length() const { return along.empty() ? 0.0f : along.back(); }
    float pathLength() const { return pathAlong.empty() ? 0.0f : pathAlong.back(); }
    // Plain blocks between the targets.
    void build(const std::array<float, 3>& start, const std::vector<std::array<float, 3>>& targets);
    // Road blocks where both ends of a stretch lie near the road network: `flatHalf(type)` is the road
    // type's flat half-width (AGW_TerraRoad +0xcc), < 0 for types the AI does not route on.
    void routeOnRoads(const struct RoadNetwork& roads, const std::function<float(int type)>& flatHalf);
    // The same with FUEL's road graph (road_graph.hpp, p32): gate ends snapped within 10 m (0x627400 ->
    // 0x6d19f0), routed by 0x6d8e20 / 0x6d7f90 with the AI's cost (row 6 col 0; `height` = terrain height for
    // its slope term). The default since P46 (FUEL_AI_ROADGRAPH=0 turns it off): FUEL's cost prefers the invisible
    // AI trails (types 0-3, 0.2 / 0.1 / 0.067 per metre), whose jump structures need FUEL's static contacts.
    void routeOnRoadGraph(const class RoadGraph& graph, const std::function<float(int type)>& flatHalf,
                      const AiTables* tables, const std::function<float(float x, float z)>& height);
    // Spline blocks (IAPathBlockSpline 0x6255d0 / 0x626a30, P43) for world courses with racing lanes
    // (SplineGraph_Z): the lanes are tagged by their crossings of the first gate line (0x625400 mode 0 +
    // flood fill 0x625fd0); each stretch then walks the middle lane from its crossing of one gate line to the
    // next (0x6256f0) and its samples are the midpoints between the outer lanes, half their distance as the
    // half-width. A stretch whose gates miss a lane keeps the path already built (FUEL falls back to its road
    // / off-road blocks). gateDir / gateHalf: per point (start, then each target), the gate's forward
    // direction (x, z) and half-width (the checkpoint block's +0x18).
    int routeOnSplineGraph(const struct SplineGraph& graph, const std::vector<std::array<float, 2>>& gateDir,
                           const std::vector<float>& gateHalf);
    // Distance along the line of the point (x, z) heading for target `next` (0-based target index).
    float progress(size_t next, double x, double z) const;
    // A point on segment `next` (towards target `next`): the foot of (x, z), moved `ahead` metres
    // further along the path. Writes the path's heading there.
    std::array<float, 3> pointAhead(size_t next, double x, double z, float ahead, float* heading = nullptr) const;

    // The path point `s` metres along (s may run past the end, by finishRun) and the unit direction there.
    std::array<float, 3> at(float s, float* dx = nullptr, float* dz = nullptr) const;
    // Distance along the path of the foot of (x, z), searched in [lo, hi].
    float project(double x, double z, float lo, float hi) const;
    // Half-width of the corridor at `s`: the block's (30 m plain, the road's flat half-width on a road
    // block), the gate's within 10 m of a target.
    float halfWidth(float s) const;
};

// Path samples (racer+0x870, 0x20 bytes each): position, lateral bounds, direction, segment length, distance along.
struct AiSample {
    float x = 0, z = 0, latMin = 0, latMax = 0, dx = 0, dz = 1, seg = 0, cum = 0;
};
// A corner record of 0x61acb0 (0x10 bytes): start point, turn (degrees), length.
struct AiCorner {
    float x = 0, z = 0, angle = 0, len = 0;
};
// An obstacle on the corridor (0x61a080 / 0x619210 records, 0x1c bytes): lateral interval at a sample.
// Types (record +0x10): 0 static object, 2 drive into it (a ramp on a trail), 3 a ramp elsewhere, 4 another racer.
struct AiObstacle {
    float lo = 0, hi = 0;
    int sample = 0;
    int type = 0;
    float s0 = 0, s1 = 0;  // along the samples (record +8 / +0xc; the list is kept sorted by s0)
};
// 0x6189b0 (P58): an oriented box (frame 3 x 4 rows: axis i in [4i..4i+2], centre in column 3; half extents;
// bounding radius r) against samples 0..n-1: per sample whose strip it overlaps, a record with its lateral extent
// widened by halfWidth + margin (racer +0x220 + the caller's 2.0) and its along extent (from 3 m before). Boxes
// with r < 15 project their three half-axes; larger ones clip their 12 edges to the strip. Types 2/3 turn to 0
// when the box's x axis is more than ~45 deg off the sample direction (dot < 0.7).
void aiBoxObstacles(const std::vector<AiSample>& samples, const float frame[12], const float half[3], float r, int type, float margin,
                    float halfWidth, std::vector<AiObstacle>& out);
// IA_SmoothLineAndAim 0x61acb0: line through the corridor from the ghost at p0 (sample index `idx`,
// distance along `s0`), aim point clamp(speedHist/2, 5, 20) metres out, corner records.
// Returns false when there is nothing ahead. Exposed for the test vectors.
bool aiSmoothLineAndAim(const std::vector<AiSample>& samples, int idx, float s0, const float p0[2], float speedHist,
                        float aim[2], std::vector<AiCorner>& corners, std::vector<std::array<float, 2>>* line = nullptr);
// 0x61dcf0: corner speed cap for the ghost at (x, z), or -1 when no corner is close.
float aiCornerSpeed(const std::vector<AiCorner>& corners, const AiTables& tables, double x, double z);
// 0x61a9d0: the obstacles on the ghost's sample push its bounds away from the ghost (lateral ghostLat) or, when
// one covers the whole corridor, become it; from sample idx+1 on: widening limit (0.1 per metre), then each
// obstacle in list order carves the corridor (type 2: clamp into it and stop; 3: remember the corridor as a
// gap; otherwise keep the side with room, remembering the better left gap). `obstacles` sorted by s0.
void aiPropagateCorridor(std::vector<AiSample>& samples, int idx, const std::vector<AiObstacle>& obstacles, float ghostLat = 0);

// The vehicle data the AI reads (VehiclesParam numbers, Pn = VehiclePP n).
struct AiVehicle {
    float roadKmh = 150, offKmh = 100;      // P619 / P620 (0x421480(0x26a/0x26b) reads P + 4*0x26a = P619)
    float road0to100 = 3, off0to100 = 3.2f;  // P623 / P624 (0x26e/0x26f): seconds 0-100 km/h
    float maxSlopeDeg = 90;                  // pm+0xf98 (taken as P626, see notes)
    float radius = 1.5f;                     // pm+0x2e94 (taken as half the wheelbase diagonal)
    float halfWidth = 1.3f;                  // racer +0x220 = pm+0x2e98 / 2 (0x61c850; taken as half the track + tyre)
    int vehicleClass = 0;                    // PK31: VehicleInfo+0x38 (BIKE 1: the lean springs' bike limits, 0x61c1b0)
};

// rain-ai: an AI racer's checkpoint-block flag: its path request's +0x21 / +0x24 (racer +0x35 / +0x38; 0 / -1 at 0x6239e0)
// and the frame count racer +0x1140 (0 at 0x617aa0). IAPathBlockCheckpoint::vf04 0x624210 raises it when the path cursor
// (IAPath_Progress 0x628750) runs past a checkpoint block whose index (block +0x2c = the line point) differs from the
// last one. 0x6182b0 (asm 0x618466..0x6184d0), every frame the racer drives: with the flag up and an index >= 0 it counts
// the frame; on the fifth it clears the flag and compares the block with the race record's target (RaceRules::
// aiCheckpointBlock). FUEL's AI racers pass their gates this way when their line misses the gate segment.
struct AiCheckpointFlag {
    bool up = false;   // +0x21
    int index = -1;    // +0x24
    int frames = 0;    // racer +0x1140
    void passed(int k) {
        if (k != index) up = true, index = k;
    }
    // One frame of 0x6182b0's check: the block index handed on this frame, else -1.
    int frame() {
        if (!up || index < 0 || ++frames <= 4) return -1;
        frames = 0, up = false;
        return index;
    }
};
// rain-ai: the path cursor moved to `s` metres along `line` (IAPath_Progress 0x628750 at 0x6182b0's cut point): every
// checkpoint block from line point `cursor` on that it runs past raises `flag` with the point's index. FUEL's block runs
// from the gate's entry to its exit (centre -/+ the gate direction, 0x623960(1.0)) and is passed when the cut point
// projects beyond the exit, however far aside (IAPathBlock::vf04 0x623f30); port: the point's distance along the path
// + 1 m. Returns the new cursor.
size_t aiPassCheckpointBlocks(const RaceLine& line, size_t cursor, float s, AiCheckpointFlag& flag);

class RaceDriver {
public:
    // mode / frac: the AI class of IA_RaceSetup 0x620e30 (0, 1, 2 by random counts per race; 3 catch-AI,
    // 4 knock-out); row: grid row (launch push 20 - 0.2 row m/s^2 for 2 s, 0x616d70 / 0x617000).
    explicit RaceDriver(int mode = 1, float frac = 0, int row = 0) : mode_(mode), frac_(frac), row_(row) {}
    // The race's target time for the level (AddMissionTime value `level`, race manager +0x3e8) and the
    // level: the speed model's init 0x616d70 (S+0x18 by class, S+0x2c, S+0x30).
    void setTarget(float targetTime, int level);

    struct Other {  // another racer (0x61a080)
        double x = 0, z = 0;
        float vx = 0, vz = 0, radius = 1.5f;
    };
    struct Sense {
        std::array<double, 3> pos{};        // vehicle model origin (ground contact)
        std::array<float, 3> vel{}, fwd{0, 0, 1}, up{0, 1, 0};
        float fwdSpeed = 0;                 // m/s along the vehicle's forward axis
        bool overturned = false;            // the vehicle's own wreck / overturn logic wants a reset (0x1429)
        bool frozen = false;                // the countdown: hold still (IA_RacerUpdate frozen = race state <= 4)
        bool forcePhysical = false;         // the player's autopilot: always simulated
        int rank = 0;                       // pm+0x2e8c: nearness rank among the AI (0 = closest to the player)
        float playerDist = 0;               // metres to the player (x, z; 0x61dae0 for the speed model)
        float switchDist = 0;               // pm+0x2e88 (0x5d3300): 3D distance to the player, x 5 out of the camera's view
        float playerAhead = 0;              // player's remaining distance minus ours (> 0: we lead)
        // PK16: the finish order (race +0x3d8, 0x5c27d0) index of this racer once it has crossed the finish, else -1;
        // aiCount = the race's NbIA (race manager 0x62c760 +0x50).
        int finishRank = -1;
        int aiCount = 0;
        // PK16 0x5257b0: the mean over all wheels of the rim speed with the dead zone (0x5c97d0's throttle reference
        // for an AI vehicle, instead of the forward speed).
        float cruiseSpeed = 0;
        float clock = 0;                    // race clock (race manager +0x244)
        std::array<float, 3> playerVel{};
        AiVehicle own, player;
        std::vector<Other> others;
        // World: ground height and normal at (x, z); off-road weight (road type +0xb4, 1 off any road).
        std::function<float(double x, double z, float n[3])> ground;
        std::function<float(double x, double z)> offroad;
        // The ghost's ground (0x61b810): a ray from 2 m above y, 100 m down, against the static objects (mask
        // 0x202/0x10fc1) wins when above the ground height (+ the water level for vehicles with P631 > 0.5).
        std::function<float(double x, double y, double z, float n[3])> ghostGround;
        // Static objects' world bounds (min x y z, max x y z) over an x/z rectangle (0x6188e0 -> 0x6438f0).
        std::function<void(double, double, double, double, std::vector<std::array<double, 6>>&)> statics;
        // P58: the static objects' AI boxes (0x639ba0 gather -> 0x619d70) over an x/z rectangle; when set it replaces
        // `statics`. roadType(x, z): the type of the road record under the point (0x6d1d70), -1 off roads (0x6176f0).
        std::function<void(double, double, double, double, std::vector<DriveWorld::WorldBox>&)> boxes;
        std::function<int(double x, double z)> roadType;
        const AiTables* tables = nullptr;
    };
    struct Command {
        bool physical = false;              // simulate the vehicle with `input` and the guide
        DriveInput input;
        // Guide targets (Pilot_SetTarget 0x5c9c20, SetTargetVelocity 0x5c96e0): world ghost point, velocity, heading.
        std::array<double, 3> pos{};
        std::array<float, 3> vel{};
        std::array<float, 4> rot{0, 0, 0, 1};
        bool place = false;                 // put the vehicle on pos/rot/vel (kinematic, or a reset)
        bool reset = false;                 // a reset (counted)
        // rain-ai, 0x6182b0 (asm 0x618466..0x6184d0): the line point (start = 0, then the targets) whose checkpoint block
        // the path cursor passed, handed on the fifth frame after; -1 = none. The caller compares it with the racer's
        // race record target (RaceRules::aiCheckpointBlock).
        int checkpointBlock = -1;
    };
    Command step(const RaceLine& line, const Sense& s, float dt);
    void restart();
    int resets() const { return resets_; }
    int mode() const { return mode_; }
    float frac() const { return frac_; }
    bool physical() const { return physical_; }
    // PK16: racer +0x12a8 after the finish: 0 racing, 4 running out to the parking spot, 5 parked.
    int finishState() const { return finishState_; }
    float ghostSpeed() const;
    float progress() const { return s_; }
    // The speed model for traces and --ai-check's gap report: S+0x1c cap, S+4 accel, S+0x14 budget, S+0x18 target.
    float speedCap() const { return cap_; }
    float accel() const { return accel_; }
    float budgetTime() const { return budget_; }
    float targetTime() const { return base_; }
    const std::array<double, 3>& ghost() const { return g_; }
    // PK31: the lean (0x61c1b0) and the acceleration estimate +0x111c it reads (0x619330).
    const AiLean& lean() const { return lean_; }
    const std::array<float, 3>& acceleration() const { return acc_; }

private:
    void updateAccel(const Sense& s, float dt, float w, float remainingShare);
    float budget(float r, const Sense& s, float dt);   // 0x617380 (+ 0x617610): time left, s
    void correct(float r, float& m, bool slower, float clock);  // 0x617610
    int mode_;
    float frac_;
    int row_;
    bool started_ = false, physical_ = false, airborne_ = false;
    std::array<double, 3> g_{}, gPrev_{};     // ghost position +0x1174, previous +0x1180
    std::array<float, 3> v_{}, f_{0, 0, 1};    // ghost velocity +0x11b4, forward +0x11c0
    std::array<float, 3> n_{0, 1, 0};         // ground normal under the ghost +0x1148
    std::array<float, 2> hist_{};              // +0x1110 velocity estimate (x, z)
    float s_ = 0, remaining0_ = -1;            // distance along; initial remaining (S+0x20 = 1 / it)
    float accel_ = 5, launchT_ = 0;            // S+4, S+0x10
    float target_ = 1, level_ = 1;             // the target time (K' >= 1) and the level (race manager +0x3e8)
    float base_ = 1, budget_ = 1, cap_ = 0;    // S+0x18, S+0x14, S+0x1c
    float timer_ = 0, gain_ = 1, late_ = 0;    // S+0x3c, S+0x48, S+0x4c
    float stuck_ = -1, reverseT_ = 0;          // +0x1fc, +0x204
    bool reversing_ = false;                   // +0x202
    float followT_ = 0, followCap_ = 0;        // +0xec8, +0xec4 (flag +0xec0 = followT_ > 0)
    std::vector<AiCorner> corners_;            // +0xecc.. (count +0x10ac)
    std::vector<AiSample> samples_;
    std::vector<AiObstacle> staticObs_;        // static objects on the samples (rebuilt with them, 0x61a080 param_3)
    // rain-ai: the path cursor's checkpoint blocks (IAPathBlockCheckpoint, one per line point, from the gate's entry to its
    // exit 1 m each side of the centre, 0x623ac0 / 0x627590): the next point whose block the cursor has not passed, and
    // the flag the passes raise.
    size_t cpCursor_ = 0;
    AiCheckpointFlag cpFlag_;
    int resets_ = 0;
    // PK16: finish state (racer +0x12a8 = 4 / 5) and the parking line (IAPathBlockFinish with path +0x30 / +0x34).
    int finishState_ = 0;
    std::optional<RaceLine> park_;
    // PK31: 0x619330's history of the vehicle position: +0x1104 position, +0x1110 velocity, +0x111c acceleration,
    // +0x1100 the previous frame's dt (1000 at init 0x617aa0), and the lean +0x1128.. / +0x119c.
    std::array<float, 3> histPos_{}, histVel_{}, acc_{};
    float histDt_ = 1000.0f;
    bool histInit_ = false;
    AiLean lean_;
    // ---- [nitro] (physics/ai_nitro.*, the port's deviation): the racer's tank amount this frame; while > 0 the speed
    // model's cap and acceleration are raised (nitroApply, defined in ai_nitro.cpp).
public:
    void setNitro(float amount, float power = 1) { nitro_ = amount, nitroPower_ = power; }  // [upgrades U1] power: the racer's
private:
    void nitroApply(float& cap, float& accel) const;
    float nitro_ = 0, nitroPower_ = 1;
    // ---- [upgrades U4] (core/upgrades aiFactors, the port's deviation): the racer's corner caps (0x61dcf0) x its tyre
    // stage's sqrt(1 + lateral grip gain). 1 = FUEL.
public:
    void setCornerScale(float k) { cornerScale_ = k; }
    float cornerScale() const { return cornerScale_; }
private:
    float cornerScale_ = 1;
};

// PK16 0x620c20: the AI racer names, the text ids 1500..1573 shuffled with FUEL's global float generator (state
// 0xa065cc, offset 0xa065c8; the shuffle is a swap of slot i with i + trunc(u (74 - i)). Racer k (0-based among the
// opponents) is named TT result[k].
std::array<int, 74> aiNameShuffle();
// The generator step the shuffle uses (0x620c20 / 0x4f07d0 ...): u = frac-mantissa trick of (state * 8.214545 + 2) * 1.4154547.
float fuelRandom();

}  // namespace fuel
