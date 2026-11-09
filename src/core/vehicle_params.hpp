// Vehicle tuning data: the numbered parameters of Param/VehiclesParam.tsc
// and the physics proxy shapes (the "<variant>_DYN" Lod_Z in the vehicle DPC).
#pragma once

#include "dpc.hpp"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace fuel {

// `VehiclePP 0 "MESH_ID_<V>"`, then `VehiclePP <i> <value>` for i = 1..678,
// then `EndOfVehiclePP`. Values are indexed by parameter number (index 0 unused).
std::map<std::string, std::vector<float>> parseVehiclesParam(const std::string& tscText);

// Parameter numbers whose meaning is known (see docs/formats.md).
namespace vp {
inline constexpr int Mass = 2;
inline constexpr int FrontDriven = 46, RearDriven = 47;
inline constexpr int WheelRadiusFront = 49, WheelRadiusRear = 50;
inline constexpr int MaxRpm = 324, IdleRpm = 325, TorqueScale = 327;
inline constexpr int TorqueCurve = 338;  // 11 samples at rpm/max = 0, 0.1 .. 1.0
inline constexpr int GearCount = 349, FinalDrive = 350, ReverseGear = 351, FirstGear = 352;
inline constexpr int TopSpeedKmh = 617;
}  // namespace vp

struct ProxySphere {
    std::array<float, 3> centre{};
    float radius = 0;
    uint32_t name = 0;  // CRC; front-left d8b7a397, front-right a828d8ed, rear-left fc33776c, rear-right 8cac0c16
};
struct ProxyBox {
    std::array<float, 3> centre{}, halfExtents{};
    float radius = 0;  // the record's bounding radius (+0xa0 of the engine's box)
};
struct VehicleProxy {
    std::vector<ProxySphere> spheres;
    std::vector<ProxyBox> boxes;
};

// The physics proxy of a variant: spheres (wheels) and boxes (chassis) in
// vehicle space, y up, ground at y = 0. Throws if the DPC has none.
VehicleProxy findVehicleProxy(const DpcFile& dpc, const std::string& variant);

}  // namespace fuel
