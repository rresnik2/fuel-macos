// fuelterrain: decodes the world heightfield and prints its layout.
//
//   fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...]
//
// --pgm compares every sample with a 16-bit PGM export (raw * 65535 / 4095).

#include "core/dpc.hpp"
#include "core/terrain.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...]\n");
        return 2;
    }
    try {
        const auto t0 = std::chrono::steady_clock::now();
        fuel::DpcFile dpc(argv[1]);
        const fuel::Heightfield hf = fuel::loadWorldHeightfield(dpc);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        uint16_t lo = 0xFFFF, hi = 0;
        for (uint16_t v : hf.raw) lo = std::min(lo, v), hi = std::max(hi, v);
        std::printf("%u x %u samples, %.3f m spacing, origin (%.1f, %.1f), %.4f m per unit, heights %.1f..%.1f m (%.2f s)\n", hf.width,
                    hf.height, hf.spacing, hf.originX, hf.originZ, hf.metresPerUnit, lo * hf.metresPerUnit, hi * hf.metresPerUnit, secs);
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            if (a == "--at" && i + 2 < argc) {
                const float x = std::strtof(argv[i + 1], nullptr), z = std::strtof(argv[i + 2], nullptr);
                std::printf("height at (%.1f, %.1f) = %.2f m\n", x, z, hf.heightAt(x, z));
                i += 2;
            } else if (a == "--pgm" && i + 1 < argc) {
                std::ifstream f(argv[++i], std::ios::binary);
                std::string magic;
                unsigned w = 0, h = 0, maxv = 0;
                f >> magic >> w >> h >> maxv;
                f.get();
                if (magic != "P5" || w != hf.width || h != hf.height || maxv != 65535) {
                    std::printf("PGM header does not match\n");
                    return 1;
                }
                size_t mismatches = 0;
                for (size_t k = 0; k < hf.raw.size(); ++k) {
                    unsigned char px[2];
                    f.read(reinterpret_cast<char*>(px), 2);
                    const unsigned ref = (unsigned(px[0]) << 8) | px[1];
                    const unsigned mine = hf.raw[k] <= 4095 ? unsigned(hf.raw[k] * 65535.0 / 4095.0 + 0.5) : 65535u;
                    mismatches += ref != mine;
                }
                std::printf("PGM comparison: %zu of %zu samples differ\n", mismatches, hf.raw.size());
                if (mismatches) return 1;
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelterrain: %s\n", e.what());
        return 1;
    }
    return 0;
}
