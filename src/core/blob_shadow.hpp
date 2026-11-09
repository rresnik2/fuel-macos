// FUEL's blob shadows (FakeShadows_G, notes p77-shadows-lights.md 1): a soft dark quad under every wheel and the
// body of each vehicle that is not the player's (the player's vehicle has the car occlusion omni instead).
// Producers 0x5d86b0 (wheel) / 0x5d8420 (body), called from the vehicle update 0x5cb8c0; record 0x509c00 / 0x509cb0;
// drawn by FakeShadows_G::vf19 0x509630.
#pragma once

#include <array>
#include <vector>

namespace fuel::blob {

using V3 = std::array<float, 3>;

// One FakeShadows record (0x30 bytes): a (+0x00, forward half axis), b (+0x0c, side half axis), centre (+0x18),
// side / fwd sizes (+0x24 / +0x28), alpha (+0x2c). The light / weather factor is applied by the renderer.
struct Record {
    V3 a{}, b{}, centre{};
    float side = 0, fwd = 0, alpha = 0;
};

// 0x509c00 + 0x509cb0: p, normal n, direction f (flattened to xz unless vertical), sizes and alpha.
Record makeRecord(const V3& p, const V3& n, const V3& f, float side, float fwd, float alpha);

// 0x5cb8c0: clamp(1 - DLIGHT_INVINTENSITY, 0.3, 1) x clamp(1 - fog, 0.2, 1).
float alphaFactor(float invIntensity, float fog);

// 0x5095b0: a corner P (view space) moved towards the camera E: d = normalize(E - P), P + d ((1 - |d.y|)^2 0.15 + 0.05),
// d.y the view-space up component (`up` = the view's up axis in the frame P and E are given in).
V3 nudge(const V3& P, const V3& E, const V3& up);

// 0x5ca7a0: the body blob's side (+0x2ea0) and forward (+0x2ea4) sizes from the merged half extents of the vehicle
// Lod_Z's boxes (hx, hz) and the wheels' local positions in the engine's order (0 / 2 front, 1 / 3 rear).
std::array<float, 2> bodySizes(float hx, float hz, const std::vector<V3>& wheelLocal);

// Everything 0x5cb8c0 / 0x5d86b0 / 0x5d8420 read from one vehicle in a frame.
struct VehicleInput {
    bool player = false;          // +0x1a48
    float cameraDistance = 0;     // +0x10a4 (x 3 behind the camera)
    std::array<float, 4> quat{0, 0, 0, 1};  // body rotation veh+0x4c0 (x, y, z, w)
    std::vector<V3> contacts;     // +0x13d8[i], engine wheel order (0 / 2 front, 1 / 3 rear)
    // q x +0x12b8[i] (0x4db890): the wheel's heading, (0, 0, 1) for rear wheels and (+0x1a44 steer, 0, 1) for wheels
    // 0 / 2 in body space; the port passes the drawn wheel's heading (axle x up). Only its xz direction is used.
    std::vector<V3> headings;
    std::vector<float> loads;     // +0x15e8[i]
    std::array<float, 2> bodySize{};  // +0x2ea0 / +0x2ea4 (bodySizes)
    std::array<float, 4> pp{};    // VehiclePP 49, 50, 51, 52 (wheel radius / width by i & 1)
};

// The per-vehicle fades (+0x1094 body, +0x19c0[i] wheels) and the records of a frame.
class Producer {
public:
    // `raceState` = g_Game+0x334c +0x48 (2..4 skip the distance / load gates). Records are appended to `out`.
    void update(int vehicle, float dt, int raceState, const VehicleInput& in, std::vector<Record>& out);
    void clear() { fades_.clear(); }

private:
    struct Fades {
        float body = 0;
        std::array<float, 16> wheel{};
    };
    std::vector<Fades> fades_;
};

}  // namespace fuel::blob
