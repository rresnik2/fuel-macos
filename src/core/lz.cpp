#include "lz.hpp"

#include <stdexcept>

namespace fuel {

// Format: a big-endian u32 of flags, whose low 2 bits set the length/distance
// split, then 30 items (flag bits 31..2, MSB first). A set bit is a
// big-endian u16 back reference, a clear bit one literal byte.
std::vector<uint8_t> lzDecompress(const uint8_t* src, size_t srcSize, size_t outSize) {
    std::vector<uint8_t> out;
    out.reserve(outSize + 64);
    size_t i = 8;
    for (;;) {
        if (i + 4 > srcSize) throw std::runtime_error("LZ input overrun (flags)");
        uint32_t flags = (uint32_t(src[i]) << 24) | (uint32_t(src[i + 1]) << 16) |
                         (uint32_t(src[i + 2]) << 8) | uint32_t(src[i + 3]);
        i += 4;
        const unsigned shift = flags & 3;
        const unsigned lenShift = 14 - shift;
        const unsigned distMask = 0x3FFFu >> shift;
        for (int k = 0; k < 30; ++k) {
            if (flags & 0x80000000u) {
                if (i + 2 > srcSize) throw std::runtime_error("LZ input overrun (match)");
                const unsigned v = (unsigned(src[i]) << 8) | src[i + 1];
                i += 2;
                const size_t len = (v >> lenShift) + 3;
                const size_t dist = (v & distMask) + 1;
                if (dist > out.size()) throw std::runtime_error("LZ back-reference before start");
                const size_t start = out.size() - dist;
                for (size_t j = 0; j < len; ++j) {  // byte by byte: runs may overlap
                    const uint8_t b = out[start + j];
                    out.push_back(b);
                }
            } else {
                if (i >= srcSize) throw std::runtime_error("LZ input overrun (literal)");
                out.push_back(src[i++]);
            }
            if (out.size() >= outSize) {
                out.resize(outSize);
                return out;
            }
            if (i >= srcSize) throw std::runtime_error("LZ input overrun");
            flags <<= 1;
        }
    }
}

}  // namespace fuel
