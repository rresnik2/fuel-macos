// FUEL's road-type table (g_RoadTypes: 21 blocks of 0x200 bytes at 0xa677e0 in FUEL.exe).
//
// The values are not stored anywhere in the exe: the engine fills the table at start-up
// with straight-line code (FUN_006f24a0). Instead of copying numbers into the port, this
// module reads the player's own FUEL.exe and runs that code in a tiny x86/x87 emulator
// (only the ~45 instruction forms the routine uses; anything else fails cleanly).
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

namespace fuel {

struct RoadType {                 // one 0x200-byte block of g_RoadTypes (FUEL.exe 0xa677e0 + type*0x200)
    std::array<float, 128> f{};   // the block as floats: f[off/4] (offsets 0x00..0x1fc)
    std::array<uint8_t, 0x200> raw{};  // the same block as bytes (for the u8 fields +0x1d8..+0x1fc and the u32 +0xa8/+0xac)
    float at(uint32_t off) const { return f[off / 4]; }
    uint8_t byte(uint32_t off) const { return raw[off]; }
    uint32_t u32(uint32_t off) const;  // little endian from raw
    std::string name;             // +0xd8
};

struct RoadTypeTable {
    std::array<RoadType, 21> types;
    bool valid = false;
};

// Rebuilds the table from the code in `exePath` (the player's FUEL.exe, read only).
// Colours come from the GenWorld palette: the code copies GW+0x47c64 / +0x47cc4 / +0x47d04
// (float4 each, the GenWorld Colors palette slots 15, 21 and 25: slot k = GW+0x47b74+16k);
// `palette(slot)` must return that float4.
// `laterInit` true = the values of a second run of the init (the engine reads +0xb8 before
// it writes it, so a repeat run sees the previous run's value).
// valid is false when the exe is missing, is another build (the code bytes are checked
// against a hash) or the emulation meets something it does not know.
RoadTypeTable buildRoadTypes(const std::string& exePath,
                             const std::function<std::array<float, 4>(int slot)>& palette,
                             bool laterInit);

}  // namespace fuel
