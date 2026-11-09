// P61: FUEL's tornadoes (TornadoAgent_G family), decoded from FUEL.exe (notes ~/re-work/fuel/notes/p61-storms.md).
//
// Agents (class registrations 0x5427e0 / 0x543600: TornadoAgent_G <- BaseAgent_G; MotherTornadoAgent_G and
// SmallTornadoAgent_G <- TornadoAgent_G; "Tornade" <- MotherTornadoAgent_G). Every one registers in the tornado list
// 0xa66c08 (vt[24] 0x5bc800); the list holds the agents whose node is shown (node +0x2c & 0x40000 clear, 0x5bc860).
//   - Mother tornadoes: the USA1 nodes named "Tornade" (two, CR21 and CR36 valleys), radius vt[105] = 50 (0x4d5670),
//     manipulator TornadoManipulator_G vf18 0x5bea80: spin about y by the millisecond clock (one turn in 250 s,
//     0x5be880), travel at 25 m/s towards a target when one is set, ping-pong within 10 m; y = the ground (0x5be320).
//     A race whose start weather holds "TOR" (AddMissionParam, parser 0x5759c0 -> mission +0x18c) sets the route at
//     the race start (0x631bd0 -> 0x6313b0): A = start + 300 f + 100 p, B = finish + 300 d + 100 p (d = the start->finish
//     direction, p = y x d flipped to the side the start heading points to, f = the start heading).
//   - The small tornado: one SmallTornadoAgent_G (USA1 library node, radius 20, 0x4d54d0), hidden until an
//     IT_TornadoPath launches it (IT_TornadoPathMove vt[17] 0x518dc0: the camera within 2 x PLAYERDIST of the path
//     node and some tornado within 5000 m of it -> path = the path node's children (0x518c30: position, SPEED n or
//     -1), start at the first point at the path's SPEED). SmallTornadoManipulator_G vf18 0x5bf5f0 walks the points
//     (next within 10 m, horizontal), hides when the path ends or no other tornado is within 5000 m (0x5bcc20 /
//     0x5bf560).
//   - Near the camera (< 2000 m, horizontal) each frame: a force volume 0x6810a0 (strength 30000 mother / 100000
//     small, radius = the agent's radius, flags 0x209) and, for the small one, the IT_ANIMATED proximity start
//     0x5181f0 (every IT_ANIMATED within 50 m of the tornado plays its event slot 334).
// Queries: 0x5bc860 (list, max 32), 0x5bc950 (nearest horizontal distance and its radius), 0x5bcaa0 (nearest position),
// 0x5bcc20 (nearest other: squared distance above 1). Force volumes 0x681540 (the vortex branch, flag 0x200).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fuel::storm {

using V3 = std::array<double, 3>;

struct PathPoint {
    V3 p{};
    float speed = -1;  // child "SPEED n", else DAT_00a05224 = -1 (the path's SPEED)
};

struct TornadoPath {
    uint32_t node = 0, parent = 0;  // the IT_TornadoPath node, its "Tornade" parent
    V3 pos{};
    float playerDist = 200, speed = 10;  // vt[24] 0x518760: PLAYERDIST / SPEED (defaults 200 / 10)
    std::string resetMission;            // "RESET_M_START <m>": the mission whose start rewinds it (0x513d70)
    std::vector<PathPoint> points;       // 0x518c30: children (+0xa4, then +0xac) world positions
};

// PK06: a TornadoEventVolumeAgent_G (vf24 0x57ab40: the node's CollisionVol_Z box 0 placed by the node matrix, hidden,
// registered in the 64 x 64 id grid 0xa66870). The debris picker 0x5bd430 asks 0x57acf0: does a sphere of 20 m at the
// tornado touch one of them?
struct EventVolume {
    uint32_t node = 0;
    V3 centre{};
    std::array<V3, 3> axis{};  // unit axes (world)
    V3 half{};                 // extents along them
    // Sphere (p, r) against the box: the closest point of the box to p is within r.
    bool touches(const V3& p, double r) const;
};

struct StormData {
    std::vector<EventVolume> eventVolumes;  // PK06
    std::vector<uint32_t> mothers;  // "Tornade" nodes
    std::vector<V3> motherPos;      // their world positions (the node matrix)
    std::vector<TornadoPath> paths;
    bool hasSmall = false;          // a SmallTornadoAgent_G Lod exists (the agent 0x518dc0 finds by name)
};
StormData loadStormData(const DpcFile& usa1, const std::vector<const DpcFile*>& lookup);

// One force volume (0x6810a0, record 0x3c bytes in world +0xbc).
struct ForceVolume {
    V3 centre{};
    float strength = 0;  // +0x1c
    float radius = 0;    // +0x20 (+0x24 = 1 / radius)
    uint32_t flags = 0x209;
    std::array<float, 3> dir{0, 1, 0};  // +0x10 (0x6810a0: up, 0xa06320; flag 1 = its length > 0.1)
};
// 0x681540 for one vortex volume (flag 0x200): the force on a body at `p` (bodyRadius = body +0x408); false when the
// body is outside (horizontal distance >= radius + bodyRadius). F = S (-0.25 n.z + k' n.x, 0.15, 0.25 n.x + k' n.z),
// n = the horizontal unit vector from the centre, k' = 1 - 1.2 k, k = clamp(1 - |0.25 dy| / radius, 0, 1).
bool vortexForce(const ForceVolume& v, const V3& p, float bodyRadius, std::array<float, 3>& force);
// P65: 0x681540's other branch (flag 0x200 clear), the radial / directional "wind" and "magnet" volume: inside when the
// 3D distance d < radius + bodyRadius; strength >= 0: k = 1 - (x / r)^2, x = max(d - bodyRadius - 1, 0.001 if < 0);
// strength < 0 (magnet): k = 2 sqrt(q (1 - q)), q = d / r (0 from q >= 1), plus 30 x (v^ . n_xz) when the body moves
// faster than 0.01 m/s towards outside (the 0x530b40 body test taken as true [I]); F = max(k, 0.01) x strength (x
// `scale` with flag 4); the direction n = (body - centre) / d (flag 0x40: (d.x, 2 d.y + 1, d.z) normalised; flag 4:
// no y), or `dir` with flag 1. Returns |F| in `magnitude`. No volume made by the shipped game takes this branch
// (0x6810a0, the only maker, always sets 0x200: the tornado vortices).
bool radialForce(const ForceVolume& v, const V3& p, float bodyRadius, const std::array<float, 3>& bodyVel, float scale,
                 std::array<float, 3>& force, float& magnitude);

class Storms {
public:
    struct World {
        std::function<float(double x, double z)> ground;  // 0x5be320 (ray down from +100 m; the port: the terrain)
        // P65, for the debris spawner 0x5bd590 (item pick 0x5bd430): the forest density at the tornado (0x45dd50 ->
        // 0x45dd90 = GenWorld_ForestDensity 0x6bd200) and whether a TornadoEventVolumeAgent_G holds it (0x57acf0, 20 m).
        std::function<float(double x, double z)> forest;
        std::function<bool(const V3& p)> inEventVolume;
    };
    // P65: one throw of the tornado debris spawner 0x5bd590: the IT_Debris item (clone.tsc), where it starts, its
    // velocity (0x533d70 -> agent +0x820.. at the launch) and the path kind (0x5bd430's return).
    struct DebrisThrow {
        const char* item = "";
        V3 pos{};
        std::array<float, 3> vel{};
        int mode = 0;      // 0 lobbed up beside the tornado, 1 shot at the player, 2 dropped ahead of the player, 3 from the side
        bool near = false; // camera within 250 m (agent +0xea9 / +0xdf8 / +0xeb2 cleared)
    };
    Storms(StormData data, World world);

    // The race start (0x631bd0): with the "TOR" weather token, the mother tornado nearest the start gets the route.
    void startRace(bool tornado, const std::array<float, 3>& start, const std::array<float, 3>& startRot,
                   const std::array<float, 3>& finish);
    void endRace();
    // ---- PK14: KillMission's weather cleanup 0x5b96a0 (only caller 0x62baa0; ECX = WeatherManager g_Game+0x3330): the
    // weather's tornado handle list +0x168 is emptied (0x5b9c40) and every agent of the tornado registry 0xa66c08 (walked
    // backwards) goes back to the object factory (0x4f1b60: node 0x40000 at once, no 0x5be290 fade-out; class bit 0 ->
    // vt+0xd4; 0x730ee0 removes the agent). [port] the port's agents are permanent: the small tornado is hidden at once
    // and the mothers go back to rest at their group node (as a fresh spawn: no route, no fade, timers 0).
    void killMission();
    // ---- end PK14
    // RESET_M_START <mission> (0x513d70 on the IT_TornadoPath nodes): their movers forget the small tornado.
    void missionStart(const std::string& mission);
    // One frame. camera = 0x421980's point (the reference the game measures storms from); clockMs = the game's
    // millisecond clock (0x6dded0).
    void update(float dt, const V3& camera, uint64_t clockMs);
    // P65: the player vehicle for the debris paths (0x421e70 position, 0x422630 velocity, 0x421ab0 forward); call
    // before update.
    void setPlayer(const V3& pos, const std::array<float, 3>& vel, const std::array<float, 3>& forward) {
        player_ = pos, playerVel_ = vel, playerFwd_ = forward;
    }
    // PK06: 0x57acf0 over the loaded TornadoEventVolumeAgent_G boxes (sphere of `r` m, 20 in 0x5bd430).
    bool inEventVolume(const V3& p, double r = 20.0) const;
    size_t eventVolumeCount() const { return data_.eventVolumes.size(); }
    const EventVolume& eventVolume(size_t i) const { return data_.eventVolumes[i]; }
    // P65: this frame's debris throws (0x5bd590 from the mothers' and the small tornado's update).
    const std::vector<DebrisThrow>& throws() const { return throws_; }
    World& world() { return world_; }

    // 0x5bc860: positions and radii of the listed (shown) agents, at most 32.
    int list(V3* pos, float* radius, int max = 32) const;
    // 0x5bc950: horizontal distance to the nearest listed agent (1e15 squared -> 3.16e7 when none) and its radius.
    float nearestDistance(double x, double z, float* radius) const;
    // 0x5bcaa0: the nearest listed agent's position; false when none.
    bool nearestPosition(double x, double z, V3& out) const;
    // 0x5bcc20: horizontal distance to the nearest agent farther than 1 m.
    float nearestOther(double x, double z) const;

    // This frame's force volumes (0x6810a0 calls) and PhysObj_ForceVolumes (0x710e30 -> 0x681540: the strongest
    // volume's force; the caller caps it at 150 x mass).
    const std::vector<ForceVolume>& volumes() const { return volumes_; }
    // 0x681540's selection: a vortex volume writes its force; a radial one only when stronger than the last kept.
    std::array<float, 3> forceAt(const V3& p, float bodyRadius, const std::array<float, 3>& bodyVel = {0, 0, 0},
                                 float scale = 1.0f) const;
    // This frame's 0x5181f0 points (IT_ANIMATED within 50 m start).
    const std::vector<V3>& animTriggers() const { return animTriggers_; }

    // For drawing (SpecialEffectsTornado_G at the agent position; height / radius PP5210 / 5211 mother, 5212 / 5213
    // small, 0x5e6860).
    struct Visible {
        V3 pos{};
        float spin = 0;  // radians (0x5be880's clock angle)
        bool small = false;
        float age = 0;   // seconds since shown
        int id = -1;     // the agent (registration order), stable while it lives
        bool fading = false;  // PK06: 0x5be290 set the look's state 1 (alpha towards 0 at 1 / PP5217 per second)
        bool frozen = false;  // PK06: 0x5bf5f0 returned after the fade call (no other tornado within 5 km): no look update
    };
    std::vector<Visible> visibles() const;
    // PK06: the alpha (+8) of agent `id`'s SpecialEffectsTornado_G, which 0x5be290 reads to know when the fade-out is over
    // (the viewer owns the looks); without a probe a fade ends at once.
    void setLookAlpha(std::function<float(int id)> f) { lookAlpha_ = std::move(f); }
    // PK06: the weather's generator tornado is gone (0x5b89e0 -> 0x5be290): the race's mother tornado fades out, then
    // leaves (node hidden, loop sounds stopped). Called when the race ends [I].
    void fadeOutRace();
    // PK06: this frame's 0x516570 requests (the manipulators' vf18 when the camera is within 200 m): the sphere
    // (position, the agent's radius) inside which every intact IT_DECORDYNMESH_G breaks.
    struct BreakSphere {
        V3 pos{};
        float radius = 0;
    };
    const std::vector<BreakSphere>& breakSpheres() const { return breaks_; }
    bool anyActive() const;
    size_t pathCount() const { return paths_.size(); }

private:
    struct Agent {
        bool small = false;
        bool shown = false;  // node 0x40000 clear (in the list)
        V3 pos{};
        float radius = 50;
        float spin = 0;
        float age = 0;
        // TornadoManipulator_G route (+0x40c / +0x418, +0x424 moving, +0x425 ping-pong, +0x426 which target)
        bool moving = false, pingPong = false, toFirst = false;
        V3 targetA{}, targetB{};
        // SmallTornadoManipulator_G path (+0x400 / +0x404, index +0x3fc, speed +0x408)
        std::vector<PathPoint> path;
        int index = 0;
        float pathSpeed = 10;
        float debrisTimer = 0;  // P65: +0x388 (mother) / +0x38c (small), 0 at vf00
        bool fading = false;    // PK06: 0x5be290 ran: the look is in state 1
        bool frozen = false;    // PK06: this frame's update stopped at the 5 km test (0x5bf5f0)
    };
    struct PathState {
        bool linked = false;  // +0x380 handle to the small agent
    };
    void wander(Agent& a, float dt, uint64_t clockMs);  // 0x5be880
    void smallStep(Agent& a, float dt);                 // 0x5bf5f0 path part
    void hide(Agent& a);                                // 0x5bf560 (with the fade-out 0x5be290)
    bool fadeOut(Agent& a);                             // 0x5be290: state 1; true when the look is gone
    std::function<float(int)> lookAlpha_;
    std::vector<BreakSphere> breaks_;

    StormData data_;
    World world_;
    std::vector<Agent> agents_;  // registration order: mothers, then the small one
    int small_ = -1;
    std::vector<TornadoPath> paths_;
    std::vector<PathState> pathState_;
    std::vector<ForceVolume> volumes_;
    std::vector<V3> animTriggers_;
    // P65: the debris spawner.
    void debris(Agent& a, float dt, double camDist2d, const V3& camera);  // the < 1000 m timer, then 0x5bd590
    float rand01();  // [port] Rand01_ParkMillerShuffle / 0x6460f0 stand-in
    uint32_t rng_ = 0x2545f491u;
    V3 player_{};
    std::array<float, 3> playerVel_{}, playerFwd_{0, 0, 1};
    std::vector<DebrisThrow> throws_;
};

// AddMissionParam's weather string (0x5759c0, upper-cased): "TOR" anywhere sets the mission's tornado byte +0x18c.
bool weatherHasTornado(const std::string& weather);

}  // namespace fuel::storm
