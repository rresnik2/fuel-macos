// Vehicle paint and liveries.
//
// Each vehicle has a 10-colour palette (VehiclesDef.tsc, `AddVehicleColor`)
// and 9 liveries stored in a per-vehicle MaterialObj_Z. The bodywork
// materials named LIVERY_XX_LOD<k> get the selected livery's texture in
// slot 0; its alpha masks where the paint colour shows (alpha 0 = paint).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t MaterialObj = 0x32A7DC67;
inline constexpr uint32_t MaterialAnim = 0xE48C96A6;
}  // namespace cls

using Rgb8 = std::array<uint8_t, 3>;

// Palettes by vehicle name (the part after MESH_ID_), in file order.
std::map<std::string, std::vector<Rgb8>> parseVehicleColors(const std::string& tscText);

// Livery texture (Bitmap_Z ID) for LOD 1..3 of one vehicle's livery
// `index` (1..9); 0 where the DPC has none.
std::array<uint32_t, 3> findLiveryTextures(const DpcFile& dpc, const std::string& vehicle, int index);

// Name CRC of the bodywork materials that take livery LOD `lod` (1..3).
uint32_t liveryMaterialName(int lod);

}  // namespace fuel
