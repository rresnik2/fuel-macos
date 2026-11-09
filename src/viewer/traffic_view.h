// P58: FUEL's free-roam road traffic (core/traffic.hpp) on the viewer's drive session. The 5 traffic slots get
// vehicles from a pool added to the session (jeeps and big trucks, MESH_ID_FLOTTE_*), placed every frame on their
// loop (FUEL places traffic kinematically away from the player; the port always does, notes); idle pool
// vehicles are parked out of the way and not drawn.
#pragma once

#include "core/traffic.hpp"
#include "drive.h"

#include <functional>
#include <memory>
#include <vector>

namespace fuel { class MeshRenderer; }

class TrafficView {
public:
    // pool[m] = drive-session indices of the vehicles of model m (0 jeep, 1 big truck, 2 special TRAFFIC_TM; P61);
    // renderer vehicle = index - 1.
    TrafficView(const fuel::RoadGraph& graph, fuel::Traffic::World world, std::array<std::vector<int>, 3> pool);
    // P61: the catch (0x5c53b0, from the contact resolver 0x51f6c0 for bodies with flag 2): the special vehicle whose
    // position (+0x4a8) is nearest the player within sqrt(10) m (squared horizontal distance < 10) is caught once;
    // onCatch(hub, index) gives the reward (FreeRoam::catchTraffic). [port: tested every frame, not on contact.]
    std::function<bool(int hub, int index)> onCatch;
    size_t caughtCount() const { return caught_.size(); }
    // Before DriveSession::update: the camera (eye, forward) of the last frame.
    void step(float dt, DriveSession& d, simd_float3 eye, simd_float3 fwd);
    // After the opponents' parts were moved: hides the idle pool vehicles.
    void hideIdle(const DriveSession& d, fuel::MeshRenderer& renderer) const;
    int active() const;
    fuel::Traffic& traffic() { return traffic_; }

private:
    void park(DriveSession& d, int index, simd_float3 eye) const;
    fuel::Traffic traffic_;
    std::array<std::vector<int>, 3> pool_;
    std::vector<int> caught_;  // special indices caught this session (flags 0x102 cleared)
    std::array<int, fuel::Traffic::kSlots> slotVehicle_;  // slot -> drive index (-1)
    std::vector<int> idle_;                               // drive indices not in use this frame
};
