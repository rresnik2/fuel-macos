// Bakes the rider for a vehicle and prints what it chose. With a reference
// OBJ (vertices in the same batch order) it reports the largest position
// difference.
//   fuelrider DATA_DIR VARIANT [REFERENCE.obj]
#include "core/rider.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: fuelrider DATA_DIR VARIANT [REFERENCE.obj]\n");
        return 2;
    }
    try {
        const std::string data = argv[1], variant = argv[2];
        std::ifstream f(data + "/LocalizedComponent/GameTsc/Story/VehiclesDef.tsc");
        std::stringstream tsc;
        tsc << f.rdbuf();
        const std::string arch = fuel::vehicleArchetype(tsc.str(), variant);
        fuel::DpcFile pilot(data + "/CommonComponent/DATAS/PILOT.DPC");
        fuel::Rider rider(pilot);
        const int slot = rider.ridingSlot(arch);
        const fuel::BakedMesh m = rider.bake(slot);
        std::printf("%s: archetype %s, slot %d, %u vertices, %zu triangles, %zu batches, %zu materials\n", variant.c_str(), arch.c_str(),
                    slot, m.vertexCount(), m.indices.size() / 3, m.batches.size(), m.materials.size());
        if (argc > 3) {
            std::ifstream obj(argv[3]);
            std::string line;
            uint32_t i = 0;
            double worst = 0, sum = 0;
            while (std::getline(obj, line))
                if (line.rfind("v ", 0) == 0 && i < m.vertexCount()) {
                    double x, y, z;
                    std::sscanf(line.c_str() + 2, "%lf %lf %lf", &x, &y, &z);
                    const auto p = m.position(i++);
                    const double d = std::max({std::fabs(p[0] - x), std::fabs(p[1] - y), std::fabs(p[2] - z)});
                    worst = std::max(worst, d);
                    sum += d;
                }
            std::printf("compared %u vertices: max abs difference %.2e m, mean %.2e m%s\n", i, worst, i ? sum / i : 0.0,
                        i == m.vertexCount() ? "" : "  (COUNT MISMATCH)");
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelrider: %s\n", e.what());
        return 1;
    }
    return 0;
}
