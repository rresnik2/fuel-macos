// FUEL's procedural randomness, shared by the vegetation scatter and the flat
// surfaces. Everything derives from one 256-entry permutation built from the
// seed stored in GenWorld_Z, so the world comes out the same every run.
// Format notes: docs/formats.md ("GenWorld_Z procedural layers").
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

using Permutation = std::array<uint8_t, 256>;

// The permutation for a GenWorld seed (USA1: 41). Needs the engine's base shuffle,
// read once from FUEL.exe in the game data folder that holds `world`.
Permutation buildPermutation(const DpcFile& world, uint32_t seed);

// `count` little-endian floats at virtual address `va` of the player's FUEL.exe
// (next to the game data that holds `world`); empty when it cannot be read.
// Engine constant tables are read this way instead of being copied into the port.
std::vector<float> readEngineFloats(const DpcFile& world, uint32_t va, size_t count);

// Path of the player's FUEL.exe next to the game data that holds `world`.
std::string engineExePath(const DpcFile& world);

// The seed stored in the world's GenWorld_Z (the u32 after the materials list).
// Throws if the world has no GenWorld_Z.
uint32_t genWorldSeed(const DpcFile& world);

// FUEL's position hash: the permutation entry at
// floor(x * 12.317513 + z * 10.193578) & 255 (fistp in the game's round-down FPU mode).
uint8_t positionHash(const Permutation& p, double x, double z);

// Midpoint-displacement fractal (value noise with `levels` subdivisions of a
// `size` metre top cell, bilinear inside the finest cell). The constants are
// the engine's; outputs are roughly 0..1 but not clamped.
class Fractal {
public:
    Fractal() = default;
    Fractal(const Permutation& p, int levels, float size, float seed, float lowAmp, float highAmp, float gain);
    float at(double x, double z) const;
    bool valid() const { return levels_ > 0; }

private:
    struct Level {
        float edgeScale = 0, edgeBias = 0, centreScale = 0, centreBias = 0;
    };
    Permutation table_{};
    int levels_ = 0;
    float finest_ = 1;
    int32_t seedOffset_ = 0;
    uint32_t mulX_ = 0, mulZ_ = 0;
    std::vector<Level> level_;
    float rnd(int32_t x, int32_t z) const;
};

}  // namespace fuel
