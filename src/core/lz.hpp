// Asobo LZ, the compression used for DPC object bodies.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fuel {

// Decompresses one compressed body. `src` starts with the 8-byte prefix
// (u32 decSize, u32 compSize) that precedes the bit stream. Throws
// std::runtime_error on malformed input.
std::vector<uint8_t> lzDecompress(const uint8_t* src, size_t srcSize, size_t outSize);

}  // namespace fuel
