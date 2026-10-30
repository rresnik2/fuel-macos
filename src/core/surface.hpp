// Surfaces under the wheels: the type at a world point (ground map overridden by
// roads), its grip, rolling resistance and the tyre sound class it maps to.
#pragma once

#include "ground.hpp"
#include "roads.hpp"

#include <functional>

namespace fuel {

// World (x, z) -> surface type 0..39 (see surfaceName()), or -1 when unknown.
using SurfaceQuery = std::function<int(double x, double z)>;

// Ground-map type, replaced by ASPHALT (11) within half a tarmac road's width and
// DIRT_FLAT (13) within half a dirt road's width. Either pointer may be null.
struct SurfaceMap {
    const GroundMaps* ground = nullptr;
    const RoadNetwork* roads = nullptr;
    int at(double x, double z) const;
};

enum class GripCategory { Tarmac, Dirt, Grass, Sand, Snow, Mud, Rock, Count };
GripCategory gripCategory(int surfaceType);  // Dirt for unknown types
const char* gripCategoryName(GripCategory c);
float gripMultiplier(GripCategory c);        // scales the tyre friction
float rollingResistance(GripCategory c);     // Crr: extra drag as a fraction of weight
inline float surfaceGrip(int type) { return gripMultiplier(gripCategory(type)); }

// The game's tyre sound classes (agent Q: 18 classes, each a rolling slow/fast loop and a
// skid loop in the vehicle's sound bank). Their names are not known; the port maps
// surface types to them by sound character (see docs/formats.md).
inline constexpr int kSoundClasses = 18;
int surfaceSoundClass(int surfaceType);  // 0..17; 2 (dirt) for unknown types

}  // namespace fuel
