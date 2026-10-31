// FUEL's per-road-type terrain parameters (the AGW_TerraRoad table at 0xa677e0, one
// 0x200-byte record per GwRoad type), as the engine computes them at load.
//
// The table is not data in the game files: AGW_TerraRoad::Init (FUEL.exe 0x6f24a0) fills
// it with constants in code. The port keeps no copy of those values. It reads the
// function's machine code from the player's own FUEL.exe and replays it with a small
// x86/x87 interpreter (straight-line stores, the FPU arithmetic between them, the
// derived-field helper 0x6f1240 and the record copy 0x6f5690), twice, as the engine's
// two GenWorld loads do (notes p21 H6: the second run's values are the ones queries see).
#pragma once

#include "dpc.hpp"

#include <array>
#include <optional>
#include <string>

namespace fuel {

// The fields the ground height uses (record offsets in the comments).
struct RoadTypeParams {
    float reach = 0;     // +0x00 T0: segments further than this do nothing
    float crown = 0;     // +0x24 T24: crown/ditch bump amplitude
    float verge = 0;     // +0x28 T28: verge bump
    float noise = 0;     // +0x34 T34: weight of the height noise (bank fractal 0x3f54)
    float fade = 0;      // +0x4c T4c: blend width
    float ditch = 0;     // +0x50 T50: ditch width
    float ditchDepth = 0;  // +0x54 T54
    float stretch = 0;   // +0x5c T5c: lateral distance beyond which the noise stretch applies
    float bumpAmp = 0;   // +0x74 T74: ATV bumps
    float bumpLen = 0;   // +0x78 T78
    float waveAmp = 0;   // +0x7c T7c: profile undulation
    float waveLen = 0;   // +0x80 T80
    float band = 0;      // +0xbc Tbc = T0 - T60
    float core = 0;      // +0xc4 Tc4 = Tbc - T4c
    float flat = 0;      // +0xcc Tcc
    float smooth = 0;    // +0x1f0 * +0x1e4: profile smoothing half-window base (M)
    float smoothMin = 0; // +0x1e8: lower bound of M
};

// The 21 records (index = GwRoad type), or nothing with `error` set when FUEL.exe cannot be
// read or its code is not the version the port knows (unknown instructions, values out of
// range). Cached per process.
std::optional<std::array<RoadTypeParams, 21>> loadRoadTypes(const DpcFile& world, std::string* error = nullptr);

}  // namespace fuel
