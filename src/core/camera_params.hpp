// FUEL's per-vehicle camera parameters (Player_GetCamParam 0x428620, vslot 99 of the player vehicle
// object; notes p25-camera-rider.md A1/A2). The values are VehiclePP 407..571 of the player's own
// VehiclesParam.tsc, 33 ids (406..438) per view 1..5: id k of view v is VehiclePP k + 1 + 33 (min(v, 5) - 1).
// Nothing from the game is compiled in; `P` is the parsed parameter block (index = parameter number).
#pragma once

#include <vector>

namespace fuel {

struct CameraParams {
    // k[n] = id 406 + n (A2): k0 distance base / FP eye forward, k1 look-target forward offset, k2 FP eye side,
    // k3 target lift / FP eye height, k4 lean shift, k5 extra pitch (deg), k6/k7/k8 FOV rest/speed/boost (deg),
    // k9 near cap, k10 ground term / FP head spring gain, k11/k12 follow stiffness (grip/sliding),
    // k13..k30 sway rates and gains (ids 419..436), k31/k32 no reader (U-C5).
    float k[33] = {};
    float operator[](int n) const { return k[n]; }
};

// 0x428620: v = min(view, 5); value of id = P[id + 1 + 33 (v - 1)]. View 0 is never used by the camera.
float cameraParam(const std::vector<float>& P, int id, int view);
CameraParams cameraParams(const std::vector<float>& P, int view);

// Other VehiclePP the cameras and the rider read (A2, A9, R7, R8).
namespace campp {
inline constexpr int kLandAmp = 404, kLandRate = 405, kLandHeight = 406;  // veh+0xc20/+0xc24/+0xc28 (landing dip)
inline constexpr int kBlurLo = 572, kBlurHi = 573, kBlurMax = 574;        // speed blur
inline constexpr int kTopSpeed = 619;                                    // km/h (SpeedFrac 0x423380)
inline constexpr int kRiderSway = 632;                                   // rider sway scale (0x5d7e00)
inline constexpr int kEject = 655;                                       // rider ejected on a wreck when > 0.5
}  // namespace campp

}  // namespace fuel
