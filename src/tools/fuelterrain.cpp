// fuelterrain: decodes the world heightfield and prints its layout.
//
//   fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...] [--seat]
//
// --pgm compares every sample with a 16-bit PGM export (raw * 65535 / 4095).
// --seat measures how many objects the sub-worlds next to USA1.DPC place on the
// ground, with the raw and the levelled (flat surfaces) heightfield.

#include "core/dpc.hpp"
#include "core/ground.hpp"
#include "core/terrain.hpp"
#include "core/world.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...] [--ground OUT.ppm SIZE]\n");
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
            if (a == "--levelled") {  // apply the flat surfaces to later --at queries
                fuel::Heightfield& h = const_cast<fuel::Heightfield&>(hf);
                std::printf("flat surfaces applied: %zu\n", fuel::applyFlatSurfaces(h, dpc));
            } else if (a == "--seat") {
                std::vector<std::unique_ptr<fuel::DpcFile>> subs;
                for (const auto& e : std::filesystem::directory_iterator(std::filesystem::path(argv[1]).parent_path())) {
                    if (e.path().extension() != ".DPC" || e.path().filename() == "USA1.DPC") continue;
                    try {
                        auto d = std::make_unique<fuel::DpcFile>(e.path().string());
                        if (d->isCurrentVersion()) subs.push_back(std::move(d));
                    } catch (const std::exception&) {
                    }
                }
                fuel::Heightfield levelled = hf;
                const size_t used = fuel::applyFlatSurfaces(levelled, dpc);
                size_t total = 0, fit = 0, onRaw = 0, onLevelled = 0, nearRaw = 0, nearLevelled = 0;
                for (const auto& sub : subs)
                    for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, {sub.get(), &dpc})) {
                        const float x = pm.world[12], y = pm.world[13], z = pm.world[14];
                        if (std::fabs(x) < 50 && std::fabs(z) < 50) continue;
                        if (pm.fitToGround) {
                            ++fit;
                            continue;
                        }
                        ++total;
                        const float r = std::fabs(y - hf.smoothHeightAt(x, z)), l = std::fabs(y - levelled.smoothHeightAt(x, z));
                        onRaw += r < 0.5f, onLevelled += l < 0.5f, nearRaw += r < 2.0f, nearLevelled += l < 2.0f;
                    }
                std::printf("flat surfaces used: %zu; placements: %zu fixed + %zu FITTOGROUND\n", used, total, fit);
                std::printf("within 0.5 m: raw %zu, levelled %zu; within 2 m: raw %zu, levelled %zu\n", onRaw, onLevelled, nearRaw, nearLevelled);
            } else if (a == "--at" && i + 2 < argc) {
                const float x = std::strtof(argv[i + 1], nullptr), z = std::strtof(argv[i + 2], nullptr);
                std::printf("height at (%.1f, %.1f) = %.2f m\n", x, z, hf.heightAt(x, z));
                i += 2;
            } else if (a == "--ground" && i + 2 < argc) {  // --ground FILE.ppm SIZE: the colour map
                const auto g0 = std::chrono::steady_clock::now();
                const fuel::GroundMaps g = fuel::buildGroundMaps(dpc, hf, uint32_t(std::atoi(argv[i + 2])));
                std::printf("ground maps %u x %u in %.2f s\n", g.size, g.size,
                            std::chrono::duration<double>(std::chrono::steady_clock::now() - g0).count());
                std::ofstream out(argv[i + 1], std::ios::binary);
                out << "P6\n" << g.size << " " << g.size << "\n255\n";
                for (size_t k = 0; k < size_t(g.size) * g.size; ++k) out.write(reinterpret_cast<const char*>(&g.colour[k * 4]), 3);
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
