// fuelterrain: decodes the world heightfield and prints its layout.
//
//   fuelterrain USA1.DPC [--pgm REFERENCE.pgm] [--at X Z ...] [--seat]
//
// --pgm compares every sample with a 16-bit PGM export (raw * 65535 / 4095).
// --seat measures how many objects the sub-worlds next to USA1.DPC place on the
// ground, with the raw and the levelled (flat surfaces) heightfield.

#include "core/dpc.hpp"
#include "core/ground.hpp"
#include "core/ground_point.hpp"
#include "core/roads.hpp"
#include "core/terrain.hpp"
#include "core/vegetation.hpp"
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
            } else if (a == "--grass" && i + 3 < argc) {
                // --grass X Z R: the decoded ground generator (grass, rock, fields) around (X, Z),
                // with timing and statistics; also a map of the area as grass.ppm (R/G/B = grass/rock/field).
                const float x = std::strtof(argv[i + 1], nullptr), z = std::strtof(argv[i + 2], nullptr);
                const int r = std::atoi(argv[i + 3]);
                i += 3;
                fuel::Heightfield levelled = hf;
                fuel::applyFlatSurfaces(levelled, dpc);
                std::unique_ptr<fuel::RoadNetwork> roads;
                try {
                    roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(dpc));
                } catch (const std::exception&) {
                }
                const fuel::GroundGenerator gen(dpc, levelled, roads.get());
                if (std::getenv("FUEL_FIELDSCAN")) {  // the whole world, one point per 100 m
                    long counts[3] = {}, popOk = 0, flatLow = 0;
                    for (double pz = -69950; pz < 70000; pz += 100)
                        for (double px = -69950; px < 70000; px += 100) {
                            const fuel::GroundPoint p = gen.at(px, pz);
                            counts[std::clamp(p.field, 0, 2)]++;
                            flatLow += p.normal[1] > 0.8707f && p.height < 1500;
                        }
                    std::printf("fieldscan: none %ld ploughed %ld crops %ld; flat and low %ld\n", counts[0], counts[1], counts[2], flatLow);
                    (void)popOk;
                }
                if (std::getenv("FUEL_GPOINT")) std::printf("mean T0 over 1 km: %.3f\n", gen.meanT0(x, z, 1000, 40));
                if (std::getenv("FUEL_GPOINT")) {  // a line of points eastwards, one per 5 m
                    for (int k = 0; k < r; k += 5) {
                        const fuel::GroundPoint p = gen.at(x + k, z);
                        std::printf("  %+5d: y %7.1f n.y %.3f hum %.2f/%.2f road %.2f cover %.2f high %.2f rock %.2f field %d grass %.2f/%.2f size %.2f\n", k,
                                    p.height, p.normal[1], p.humidity, p.surfaceHumidity, p.road, p.cover, p.highGround, p.rock, p.field, p.grass,
                                    p.groundGrass, p.grassSize);
                        const fuel::GroundColours c = gen.colours(x + k, z, p);
                        std::printf("         colour %.3f %.3f %.3f far %.3f %.3f %.3f road %.3f %.3f %.3f occ %.2f forest %.2f rockSlope %.2f tarmac %.0f\n",
                                    c.base[0], c.base[1], c.base[2], c.far[0], c.far[1], c.far[2], c.road[0], c.road[1], c.road[2],
                                    c.occlusion, c.forest, p.rockSlope, p.tarmac);
                    }
                }
                const auto g0 = std::chrono::steady_clock::now();
                std::vector<fuel::GrassTuft> tufts;
                gen.grass(int32_t(x) - r, int32_t(z) - r, int32_t(x) + r, int32_t(z) + r, tufts);
                const double gs = std::chrono::duration<double>(std::chrono::steady_clock::now() - g0).count();
                std::printf("grass: %zu tufts over %d x %d m in %.1f ms (%.2f us per point)\n", tufts.size(), 2 * r, 2 * r, gs * 1e3,
                            gs * 1e6 / (4.0 * r * r));
                double sumSize = 0;
                int tiles[16] = {};
                for (const auto& t : tufts) sumSize += t.size, tiles[t.tile & 15]++;
                if (!tufts.empty()) std::printf("  mean size %.2f m; tiles", sumSize / double(tufts.size()));
                for (int k = 0; k < 9; ++k) std::printf(" %d", tiles[k]);
                std::printf("\n");
                // Coarse map: one point per 4 m.
                const int n = r / 2;
                std::ofstream out("grass.ppm", std::ios::binary);
                out << "P6\n" << n << " " << n << "\n255\n";
                double sum[6] = {};
                int fields[3] = {}, excluded = 0;
                for (int j = 0; j < n; ++j)
                    for (int k = 0; k < n; ++k) {
                        const double px = x - r + k * 4.0, pz = z - r + j * 4.0;
                        const fuel::GroundPoint p = gen.at(px, pz);
                        sum[0] += p.groundGrass, sum[1] += p.rock, sum[2] += p.road, sum[3] += p.surfaceHumidity, sum[4] += p.grassSize,
                            sum[5] += p.cover;
                        fields[std::clamp(p.field, 0, 2)]++;
                        excluded += gen.excludesPlants(px, pz);
                        const unsigned char px3[3] = {(unsigned char)(p.groundGrass * 255), (unsigned char)(p.rock * 255),
                                                      (unsigned char)(p.field * 120)};
                        out.write(reinterpret_cast<const char*>(px3), 3);
                    }
                const double cnt = double(n) * n;
                std::printf("  means: groundGrass %.3f rock %.3f road %.3f surfHumidity %.3f size %.3f cover %.3f\n", sum[0] / cnt,
                            sum[1] / cnt, sum[2] / cnt, sum[3] / cnt, sum[4] / cnt, sum[5] / cnt);
                std::printf("  fields: none %d ploughed %d crops %d; in no-plant flat surfaces: %d (of %d)\n", fields[0], fields[1], fields[2],
                            excluded, n * n);
                try {  // trees and bushes with and without the ground tests
                    const fuel::Vegetation veg = fuel::loadVegetation(dpc);
                    std::vector<fuel::Plant> before, after;
                    fuel::scatterPlants(veg, levelled, roads.get(), nullptr, x, z, float(r), before);
                    fuel::scatterPlants(veg, levelled, roads.get(), nullptr, x, z, float(r), after, false, 0, &gen);
                    std::printf("  plants: %zu without the ground tests, %zu with\n", before.size(), after.size());
                } catch (const std::exception& e) {
                    std::printf("  plants: %s\n", e.what());
                }
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
