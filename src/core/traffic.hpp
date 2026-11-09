// FUEL's free-roam road traffic (TrafficManipulator_G, vf17 0x5c3010), decoded and ported by P58
// (notes/p58-traffic.md). Pure logic: which loop vehicles exist, where they are and how they are posed; the
// viewer (traffic_view) gives them vehicles.
//
// Around the camera, a 4 x 4 window of 2000 m cells (0x5c36b0), each with a road loop in its 4 km box
// (RoadGraph::findLoop, 0x6daff0). A loop of length L carries n = clamp(trunc(L / 17 x 0.02833), 1, 16) virtual
// vehicles, evenly spaced, at 17 m/s; their places are a function of a millisecond clock (0x6de150), so they
// keep moving while nobody sees them. The 5 nearest virtual vehicles within 450 m of the camera and 350 m of
// the point 225 m ahead of it get a real vehicle (slots, 0x5c33f0 / 0x5c3d40 / 0x5c3d70): a fleet jeep or big
// truck (MESH_ID_FLOTTE_JEEP_1 / _BIGTRUCK_1) placed on the right-hand lane every frame (0x5c4980). A vehicle
// pushed more than 5 m off its place freezes its loop's clock (state 1), which catches up later (0x5c3530).
#pragma once

#include "road_graph.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel {

class Traffic {
public:
    struct World {
        // Lane: the road type's flat half-width (AGW_TerraRoad +0xcc); the lane offset is max(0.5 x it, 1.5).
        std::function<float(int type)> flatHalf;
        // Ground height and normal under (x, z) (0x6407e0 / 0x640690).
        std::function<float(float x, float z, float n[3])> ground;
    };
    Traffic(const RoadGraph& graph, World world) : graph_(graph), world_(std::move(world)) {}

    // A real traffic vehicle (one of the 5 slots, this+0x2c + i x 0x44).
    struct Vehicle {
        bool active = false;
        bool spawned = false;              // this frame (0x5c4640)
        int model = 0;                     // 0 MESH_ID_FLOTTE_JEEP_1, 1 MESH_ID_FLOTTE_BIGTRUCK_1, 2 special (TRAFFIC_TM)
        int special = -1;                  // P61: index into the specials (setSpecials), -1 for fleet traffic
        int colour = 0;                    // vehicle +0x514 (0..9)
        std::array<double, 3> pos{};       // target on the ground (slot +0x1c)
        std::array<float, 4> rot{0, 0, 0, 1};
        std::array<float, 3> vel{};        // 17 m/s along its heading
    };
    static constexpr int kSlots = 5;
    // One frame: the camera (x, z) and its forward direction, `where(i)` = where slot i's vehicle really is
    // (x, z; for the freeze check). TrafficOFF / a clock going back reset everything (0x5c3210).
    void update(float dt, float camX, float camZ, float fwdX, float fwdZ, const std::function<std::array<double, 2>(int slot)>& where);
    const std::array<Vehicle, kSlots>& vehicles() const { return out_; }
    void reset();          // 0x5c3210
    // P61: the special catchable vehicles (career AddHubTrafficVehicle entries, 0x5c5290 registers the uncaught ones):
    // one vehicle per entry on the road loop found in its box (0x5c4370: RoadGraph::findLoop, 17 m/s, period L / 17,
    // its own clock state as a cell's), a candidate for the 5 slots within 1000 m of the camera (0x5c4070). The
    // entries are considered when their box is within kSpecialRange of the camera [port: FUEL's grid 0x5c5f70].
    struct SpecialDef {
        int hub = -1, index = -1;
        float x0 = 0, x1 = 0, z0 = 0, z1 = 0;
        float frac = 0;  // entry +0x30 (0x5c5290: 0x640700, a random [I]): the place on the loop as a fraction of P
    };
    void setSpecials(std::vector<SpecialDef> defs);
    void removeSpecial(int i);  // caught: the entry leaves the list
    const std::vector<SpecialDef>& specials() const { return specialDefs_; }
    static constexpr float kSpecialRange = 4000.0f;
    bool enabled = true;   // DAT_009f874f (console TrafficON / TrafficOFF; on by default)
    int loopsFound() const;

private:
    struct Cell {
        uint32_t key = 0x7fff7fff;
        int x = 0, z = 0;             // cell coordinates (2000 m)
        RoadGraph::Loop loop;
        float P = 0, speed = 17.0f;   // +0x40c period L / 17, +0x414
        int state = 0;                // +0x418
        float f = 1;                  // +0x41c
        uint64_t tref = 0;            // +0x420
        int n = 0;                    // +0x428
        int8_t slot[16];              // +0x42c
        Cell() { std::fill(std::begin(slot), std::end(slot), int8_t(-1)); }
    };
    struct Candidate { float d2 = 1e24f; int cell = -1, k = 0; float s = 0; };
    struct Slot {
        bool free = true, placed = false;
        int cell = -1, k = 0;
        float s = 0;
        int model = 0, colour = 0;
        std::array<float, 3> target{};
        std::array<float, 2> fwd{0, 1};
        std::array<float, 4> q{0, 0, 0, 1};
    };
    void freeVehicle(int cell, int k);      // 0x5c4890
    int offer(float d2, int cell, int k, float s);   // 0x5c33f0
    void stateMachine(Cell& c, uint64_t& tc, float dt);  // 0x5c3530
    void drive(int slot, float dt, const std::function<std::array<double, 2>(int)>& where);  // 0x5c4980

    Cell& cellAt(int ci) { return ci < 16 ? cells_[ci] : specials_[size_t(ci - 16)]; }
    void updateSpecials(float dt, float camX, float camZ, uint64_t G);  // 0x5c4070 / 0x5c4370

    const RoadGraph& graph_;
    World world_;
    Cell cells_[16];
    std::vector<SpecialDef> specialDefs_;
    std::vector<Cell> specials_;       // per entry: its loop (+0xc), clock state; slot[0] = its vehicle slot
    std::vector<char> specialSearched_;
    Candidate cand_[5];
    Slot slots_[kSlots];
    std::array<Vehicle, kSlots> out_{};
    uint64_t gameMs_ = 0, last_ = 0;        // the clock G, the last traffic time this+0x8620
};

}  // namespace fuel
