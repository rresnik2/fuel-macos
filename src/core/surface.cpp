#include "surface.hpp"

namespace fuel {

int SurfaceMap::at(double x, double z) const {
    if (roads != nullptr)
        if (const int r = roadSurfaceAt(*roads, x, z); r >= 0) return r;
    return ground != nullptr ? ground->surfaceAt(x, z) : -1;
}

namespace {

using G = GripCategory;
// By surface type index (the 40 names of ground.cpp).
constexpr G kCategory[kSurfaceTypes] = {
    G::Dirt, G::Dirt, G::Dirt,    // EARTH, EARTH2, DIRT_FAIRLY_ROUGH
    G::Rock, G::Rock, G::Dirt,    // STONE, STONE2, PEBBLE
    G::Sand, G::Sand,             // SAND, SAND2
    G::Snow, G::Snow,             // SNOW, SNOW2
    G::Mud,                       // PLOUGH
    G::Tarmac, G::Tarmac,         // ASPHALT, ASPHALT2
    G::Dirt, G::Dirt,             // DIRT_FLAT, DIRT_ROUGH
    G::Rock, G::Rock,             // ROCK, ROCK2
    G::Grass, G::Sand, G::Grass,  // STEPPENL, DESERTNL, HILLSNL
    G::Rock, G::Grass,            // ROCKNL, PLAINNL
    G::Grass, G::Grass,           // GRASS, GRASS2
    G::Dirt, G::Dirt,             // CRACKED, DIRT_FAIRLY_FLAT
    G::Rock, G::Rock,             // BIG_STONE, BIG_STONE2
    G::Dirt, G::Rock,             // STEEP_EARTH, STEEP_STONE
    G::Mud, G::Mud,               // HUMIDEARTH, HUMIDEARTH2
    G::Dirt, G::Dirt,             // ROUGHEARTH, ROUGHEARTH2
    G::Rock, G::Rock,             // ROUGHROCK, ROUGHROCK2
    G::Mud, G::Mud,               // WETSAND, WETSAND2
    G::Sand, G::Sand};            // ROUGHSAND, ROUGHSAND2

constexpr float kGrip[int(G::Count)] = {1.15f, 1.0f, 0.9f, 0.75f, 0.6f, 0.65f, 0.95f};
constexpr float kCrr[int(G::Count)] = {0.0f, 0.01f, 0.03f, 0.08f, 0.05f, 0.07f, 0.01f};
constexpr const char* kNames[int(G::Count)] = {"tarmac", "dirt", "grass", "sand", "snow", "mud", "rock"};

// Tyre sound class by surface type: chosen by the sound table's gravel rate (0 = hard and
// smooth ... 0.8 = very loose) and the loops' character (see docs/formats.md).
constexpr int8_t kSoundClass[kSurfaceTypes] = {
    2, 2, 4,      // EARTH, EARTH2, DIRT_FAIRLY_ROUGH
    1, 1, 5,      // STONE, STONE2, PEBBLE
    8, 8,         // SAND, SAND2
    9, 9,         // SNOW, SNOW2
    4,            // PLOUGH
    0, 0,         // ASPHALT, ASPHALT2
    3, 4,         // DIRT_FLAT, DIRT_ROUGH
    1, 1,         // ROCK, ROCK2
    7, 8, 7,      // STEPPENL, DESERTNL, HILLSNL
    1, 7,         // ROCKNL, PLAINNL
    7, 7,         // GRASS, GRASS2
    3, 3,         // CRACKED, DIRT_FAIRLY_FLAT
    6, 6,         // BIG_STONE, BIG_STONE2
    4, 6,         // STEEP_EARTH, STEEP_STONE
    11, 11,       // HUMIDEARTH, HUMIDEARTH2
    4, 4,         // ROUGHEARTH, ROUGHEARTH2
    6, 6,         // ROUGHROCK, ROUGHROCK2
    11, 11,       // WETSAND, WETSAND2
    8, 8};        // ROUGHSAND, ROUGHSAND2

}  // namespace

GripCategory gripCategory(int t) { return t >= 0 && t < kSurfaceTypes ? kCategory[t] : G::Dirt; }
const char* gripCategoryName(GripCategory c) { return kNames[int(c)]; }
float gripMultiplier(GripCategory c) { return kGrip[int(c)]; }
float rollingResistance(GripCategory c) { return kCrr[int(c)]; }
int surfaceSoundClass(int t) { return t >= 0 && t < kSurfaceTypes ? kSoundClass[t] : 2; }

}  // namespace fuel
