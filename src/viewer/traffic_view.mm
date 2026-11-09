// P58: free-roam traffic on the drive session (see traffic_view.h).
#include "traffic_view.h"

#include "renderer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

TrafficView::TrafficView(const fuel::RoadGraph& graph, fuel::Traffic::World world, std::array<std::vector<int>, 3> pool)
    : traffic_(graph, std::move(world)), pool_(std::move(pool)) {
    slotVehicle_.fill(-1);
}

void TrafficView::park(DriveSession& d, int index, simd_float3 eye) const {
    // Out of the way: 5 km above the camera, still (the pose is set again every frame).
    d.physics().holdPose(index, {eye.x + 50.0 * index, eye.y + 5000.0, eye.z}, {0, 0, 0, 1}, {0, 0, 0});
}

void TrafficView::step(float dt, DriveSession& d, simd_float3 eye, simd_float3 fwd) {
    float fx = fwd.x, fz = fwd.z;
    const float fl = std::hypot(fx, fz);
    if (fl > 1e-6f) fx /= fl, fz /= fl;
    traffic_.update(dt, eye.x, eye.z, fx, fz, [&](int slot) -> std::array<double, 2> {
        const int i = slotVehicle_[size_t(slot)];
        if (i < 0) return {1e9, 1e9};
        const fuel::VehicleState st = d.state(i);
        return {st.chassis.position[0], st.chassis.position[2]};
    });
    const auto& veh = traffic_.vehicles();
    std::array<std::vector<bool>, 3> used{std::vector<bool>(pool_[0].size(), false), std::vector<bool>(pool_[1].size(), false),
                                          std::vector<bool>(pool_[2].size(), false)};
    // Keep each live slot on its vehicle; a slot that (re)spawned or changed model takes a free one of its model.
    for (int s = 0; s < fuel::Traffic::kSlots; ++s) {
        const auto& v = veh[size_t(s)];
        int& i = slotVehicle_[size_t(s)];
        if (!v.active) {
            i = -1;
            continue;
        }
        const auto& P = pool_[size_t(v.model)];
        auto it = std::find(P.begin(), P.end(), i);
        if (v.spawned || it == P.end()) i = -1;
        else used[size_t(v.model)][size_t(it - P.begin())] = true;
    }
    for (int s = 0; s < fuel::Traffic::kSlots; ++s) {
        const auto& v = veh[size_t(s)];
        int& i = slotVehicle_[size_t(s)];
        if (!v.active || i >= 0) continue;
        const auto& P = pool_[size_t(v.model)];
        for (size_t k = 0; k < P.size(); ++k)
            if (!used[size_t(v.model)][k]) {
                used[size_t(v.model)][k] = true, i = P[k];
                break;
            }
    }
    idle_.clear();
    for (int m = 0; m < 3; ++m)
        for (size_t k = 0; k < pool_[size_t(m)].size(); ++k)
            if (!used[size_t(m)][k]) idle_.push_back(pool_[size_t(m)][k]);
    for (int s = 0; s < fuel::Traffic::kSlots; ++s) {
        const int i = slotVehicle_[size_t(s)];
        if (i < 0) continue;
        const auto& v = veh[size_t(s)];
        d.physics().setGuide(i, false);
        d.physics().holdPose(i, v.pos, v.rot, v.vel);  // Pilot_SetTarget 0x5c9c20 / SetTargetVelocity 0x5c96e0 (kinematic)
        d.setInput(i, {0, 0, 0, false});
    }
    for (int i : idle_) park(d, i, eye);
    // P61: the catch 0x5c53b0 (nearest special vehicle to the player, squared horizontal distance < 10).
    if (onCatch && d.vehicleCount() > 0) {
        const fuel::VehicleState me = d.state(0);
        int best = -1;
        double bd = 1e20;
        for (int s = 0; s < fuel::Traffic::kSlots; ++s) {
            const auto& v = veh[size_t(s)];
            const int i = slotVehicle_[size_t(s)];
            if (!v.active || v.special < 0 || i < 0) continue;
            if (std::find(caught_.begin(), caught_.end(), v.special) != caught_.end()) continue;
            const fuel::VehicleState st = d.state(i);
            const double dx = st.chassis.position[0] - me.chassis.position[0], dz = st.chassis.position[2] - me.chassis.position[2];
            if (dx * dx + dz * dz < bd) bd = dx * dx + dz * dz, best = v.special;
        }
        if (best >= 0 && bd >= 10.0 && std::getenv("FUEL_TRAFFIC_CATCHTEST")) {  // check: put the player beside it
            for (int s = 0; s < fuel::Traffic::kSlots; ++s)
                if (veh[size_t(s)].special == best && slotVehicle_[size_t(s)] >= 0) {
                    const fuel::VehicleState st = d.state(slotVehicle_[size_t(s)]);
                    d.teleport(st.chassis.position[0] + 2.5, st.chassis.position[2], 0);
                    std::printf("traffic: catch test, player moved beside special vehicle %d\n", best);
                }
        }
        if (best >= 0 && bd < 10.0) {
            const auto& e = traffic_.specials()[size_t(best)];
            caught_.push_back(best);
            if (onCatch(e.hub, e.index)) std::printf("traffic: caught special vehicle %d of hub %d\n", e.index, e.hub);
        }
    }
}

void TrafficView::hideIdle(const DriveSession& d, fuel::MeshRenderer& renderer) const {
    // A speck a thousand kilometres under the world.
    simd_float4x4 m = matrix_identity_float4x4;
    m.columns[0].x = m.columns[1].y = m.columns[2].z = 1e-6f;
    m.columns[3] = simd_make_float4(0, -1e6f, 0, 1);
    for (int i : idle_) renderer.setVehicleWorlds(size_t(i - 1), std::vector<simd_float4x4>(d.parts(i).size(), m));
}

int TrafficView::active() const {
    return int(std::count_if(slotVehicle_.begin(), slotVehicle_.end(), [](int i) { return i >= 0; }));
}
