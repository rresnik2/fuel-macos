// fueldrive: drives a vehicle on the world terrain without a window and prints
// what happens (a check for the physics and its tuning).
//
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT [X Z]

#include "core/dpc.hpp"
#include "core/terrain.hpp"
#include "core/vehicle_params.hpp"
#include "physics/vehicle_physics.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: fueldrive DATA_DIR VEHICLE_DPC VARIANT [X Z]\n");
        return 2;
    }
    try {
        const std::string data = argv[1];
        fuel::DpcFile world(data + "/CommonComponent/WORLD/USA1.DPC");
        const fuel::Heightfield terrain = fuel::loadWorldHeightfield(world);
        fuel::DpcFile vehicleDpc(data + "/" + argv[2]);
        std::ifstream f(data + "/CommonComponent/Param/VehiclesParam.tsc");
        std::stringstream text;
        text << f.rdbuf();
        const auto params = fuel::parseVehiclesParam(text.str());
        const auto it = params.find(argv[3]);
        if (it == params.end()) throw std::runtime_error(std::string("no parameters for ") + argv[3]);
        const fuel::VehicleSetup setup = fuel::makeVehicleSetup(vehicleDpc, argv[3], it->second);
        std::printf("%s: %.0f kg, %zu wheels r=%.2f, %s, %.0f-%.0f rpm, %.0f Nm, %zu gears, final %.2f, nominal top %.0f km/h\n",
                    setup.name.c_str(), setup.mass, setup.wheels.size(), setup.wheels[0].radius,
                    setup.frontDriven && setup.rearDriven ? "4WD" : setup.frontDriven ? "FWD" : "RWD", setup.idleRpm, setup.maxRpm,
                    setup.peakTorque, setup.gears.size(), setup.finalDrive, setup.topSpeedKmh);

        const double x = argc > 5 ? std::atof(argv[4]) : -1000, z = argc > 5 ? std::atof(argv[5]) : -30000;
        fuel::DriveWorld sim(terrain);
        sim.spawn(setup, x, z, 0.0f);
        auto report = [&](const char* phase, double t) {
            const fuel::VehicleState s = sim.state();
            const double ground = terrain.smoothHeightAt(s.chassis.position[0], s.chassis.position[2]);
            const auto& q = s.chassis.rotation;
            const double fx = 2 * (q[0] * q[2] + q[3] * q[1]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
            std::printf("  heading %6.1f deg ", std::atan2(fx, fz) * 180 / M_PI);
            std::printf("  %-9s t=%5.1fs  %6.1f km/h  gear %d  %5.0f rpm  wheels on ground %d  height above terrain %.2f m  up.y %.2f\n", phase,
                        t, s.speedKmh, s.gear, s.rpm, s.wheelsOnGround, s.chassis.position[1] - ground,
                        1 - 2 * (s.chassis.rotation[0] * s.chassis.rotation[0] + s.chassis.rotation[2] * s.chassis.rotation[2]));
        };
        double t = 0;
        auto run = [&](const char* phase, double seconds, fuel::DriveInput in) {
            for (int i = 0; i < int(seconds * 60); ++i) {
                sim.update(1.0f / 60, in);
                t += 1.0 / 60;
                if (i % 60 == 59 || (std::getenv("FUEL_TRACE") && t < 1.0 && i % 5 == 0)) report(phase, t);
            }
        };
        run("settle", 2, {});
        run("throttle", 12, {1, 0, 0, false});
        run("brake", 4, {0, 1, 0, false});
        run("turn", 6, {0.6f, 0, 1, false});
        const fuel::VehicleState end = sim.state();
        std::printf("travelled to (%.1f, %.1f, %.1f)\n", end.chassis.position[0], end.chassis.position[1], end.chassis.position[2]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fueldrive: %s\n", e.what());
        return 1;
    }
    return 0;
}
