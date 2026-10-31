// The rider's (archetype, action) -> animation slot table and the action properties, as FUEL.exe builds
// them at vehicle load: Rider_BuildSlotTable 0x5d6950 (with 0x5d7390 QUAD/QUAD_TRAIL, 0x5d7700 SUV,
// 0x5d7850 TRUCK) and Rider_InitActionProps 0x5d3660 (notes p25-camera-rider.md R2/R3).
//
// Both are immediate stores in code, not data in the game files. The port keeps no copy: it reads the
// functions' machine code from the player's own FUEL.exe and replays the stores with a tiny x86 interpreter
// (mov/lea/xor/sub/cmp, the jump table and the jumps), as road_types.cpp does for the road table.
#pragma once

#include <array>
#include <optional>
#include <string>

namespace fuel {

inline constexpr int kRiderActions = 47;  // 0..46; 47 = "none"

// Archetype enum of VehicleInfo+0x40 (parser 0x56aa90; p9 S11).
int riderArchetypeIndex(const std::string& name);  // -1 when unknown

struct RiderActionProps {
    int group = 0, priority = 0, mode = 0;  // rec veh+0x270 + 12 action
};

struct RiderTables {
    std::array<std::array<int, kRiderActions>, 11> slots{};  // [archetype][action] -> animation slot
    std::array<RiderActionProps, kRiderActions> props{};
};

// Replays 0x5d6950 for the 11 archetypes and 0x5d3660 from `exePath`; nothing (with `error`) when the file
// cannot be read or its code is not the build the port knows. Cached per path.
std::optional<RiderTables> loadRiderTables(const std::string& exePath, std::string* error = nullptr);

}  // namespace fuel
