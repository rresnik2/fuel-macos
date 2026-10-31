// fueldrive: drives a vehicle on the world terrain without a window and prints
// what happens (a check for the physics and its tuning).
//
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT [X Z]
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --surfaces
//     the same throttle-then-brake test on tarmac, grass and sand: top speed,
//     0-80 km/h time, braking distance from 80 km/h and the surfaces under the wheels
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --behaviour [SURFACE_TYPE]
//     on FUEL's terrain, on a flat stretch, with one surface type under the wheels (default 11 =
//     asphalt): 0-100 km/h time, top speed, braking distance from 100 (or the top speed), turning
//     circle at 30 km/h, and a jump (2 m drop at 60 km/h): landing peak, bounces, settle time

#include "core/dpc.hpp"
#include "core/ground.hpp"
#include "core/roads.hpp"
#include "core/surface.hpp"
#include "core/terrain.hpp"
#include "core/vehicle_params.hpp"
#include "physics/vehicle_physics.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <memory>
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
        const bool behaviour = argc > 4 && std::string(argv[4]) == "--behaviour";
        const fuel::Heightfield terrain = fuel::loadLevelledHeightfield(world);
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

        if (behaviour) {
            const int type = argc > 5 ? std::atoi(argv[5]) : 11;
            // The straightest, flattest 800 m of tarmac road within 6 km of the Yellowstone hub (roads are carved
            // smooth into the ground; off-road FUEL's detail fractals make the ride bumpy).
            const fuel::RoadNetwork roads = fuel::loadRoads(world);
            double bx = -51358, bz = -30043, best = 1e9;
            float byaw = 0;
            for (const auto& line : roads.lines) {
                if (!fuel::isTarmacRoad(line.type)) continue;
                for (size_t k = 0; k + 1 < line.points.size(); ++k) {
                    const auto& p = line.points[k];
                    if (std::hypot(p[0] + 51358, p[1] + 30043) > 6000) continue;
                    const float yaw = std::atan2(line.points[k + 1][0] - p[0], line.points[k + 1][1] - p[1]);
                    // Follow the polyline 800 m: total heading change and height range.
                    double along = 0, turn = 0, lo = 1e9, hi = -1e9;
                    float prevYaw = yaw;
                    for (size_t j = k; j + 1 < line.points.size() && along < 800; ++j) {
                        const auto &a0 = line.points[j], &a1 = line.points[j + 1];
                        const float y = std::atan2(a1[0] - a0[0], a1[1] - a0[1]);
                        float dy = y - prevYaw;
                        while (dy > M_PI) dy -= 2 * M_PI;
                        while (dy < -M_PI) dy += 2 * M_PI;
                        turn += std::fabs(dy);
                        prevYaw = y;
                        along += std::hypot(a1[0] - a0[0], a1[1] - a0[1]);
                        const double h = terrain.smoothHeightAt(a1[0], a1[1]);
                        lo = std::min(lo, h), hi = std::max(hi, h);
                    }
                    if (along < 800) continue;
                    const double sc = turn * 20 + (hi - lo);
                    if (sc < best) best = sc, bx = p[0], bz = p[1], byaw = yaw;
                }
            }
            std::printf("test road at (%.0f, %.0f) heading %.0f deg, score (20 x turn rad + height range m) %.2f, surface %s\n", bx, bz, byaw * 180 / M_PI, best,
                        fuel::surfaceName(type));
            auto make = [&] {
                auto w = std::make_unique<fuel::DriveWorld>(terrain);
                w->setSurfaceQuery([type](double, double) { return type; });
                w->spawn(setup, bx, bz, byaw);
                for (int i = 0; i < 90; ++i) w->update(1.0f / 60, {});
                return w;
            };
            constexpr float dt = 1.0f / 60;
            // 0-100 and top speed.
            auto sim = make();
            double t = 0, t100 = -1, t60 = -1;
            float top = 0;
            for (int i = 0; i < 45 * 60; ++i) {
                sim->update(dt, {1, 0, 0, false});
                t += dt;
                const float v = sim->state().speedKmh;
                top = std::max(top, v);
                if (t60 < 0 && v >= 60) t60 = t;
                if (t100 < 0 && v >= 100) t100 = t;
            }
            // Braking from 100 (or 90 % of the top speed).
            sim = make();
            const float from = std::min(100.0f, top * 0.9f);
            for (int i = 0; i < 40 * 60 && sim->state().speedKmh < from; ++i) sim->update(dt, {1, 0, 0, false});
            const auto p0 = sim->state().chassis.position;
            double brake = -1, tb = 0;
            for (int i = 0; i < 20 * 60; ++i) {
                sim->update(dt, {0, 1, 0, false});
                tb += dt;
                const auto s = sim->state();
                if (s.speedKmh < 1.0f) {
                    brake = std::hypot(s.chassis.position[0] - p0[0], s.chassis.position[2] - p0[2]);
                    break;
                }
            }
            // Turning circle at 30 km/h: full right lock, throttle holding 30 km/h; radius from the path's
            // heading change over distance in the last 4 s.
            sim = make();
            auto heading = [](const fuel::VehicleState& s) {
                const auto& q = s.chassis.rotation;
                return std::atan2(2 * (q[0] * q[2] + q[3] * q[1]), 1 - 2 * (q[0] * q[0] + q[1] * q[1]));
            };
            for (int i = 0; i < 12 * 60 && sim->state().speedKmh < 30; ++i) sim->update(dt, {1, 0, 0, false});
            double dist = 0, turned = 0, prevH = heading(sim->state());
            auto prevP = sim->state().chassis.position;
            float vsum = 0;
            int n = 0;
            for (int i = 0; i < 8 * 60; ++i) {
                const float v = sim->state().speedKmh;
                sim->update(dt, {std::clamp((30 - v) * 0.15f + 0.3f, 0.0f, 1.0f), v > 33 ? 0.3f : 0.0f, 1, false});
                const auto s = sim->state();
                if (i >= 4 * 60) {
                    double dh = heading(s) - prevH;
                    while (dh > M_PI) dh -= 2 * M_PI;
                    while (dh < -M_PI) dh += 2 * M_PI;
                    turned += dh;
                    dist += std::hypot(s.chassis.position[0] - prevP[0], s.chassis.position[2] - prevP[2]);
                    vsum += s.speedKmh, ++n;
                }
                prevH = heading(s);
                prevP = s.chassis.position;
            }
            const double radius = std::fabs(turned) > 1e-3 ? dist / std::fabs(turned) : -1;
            // Jump: 2 m drop at 60 km/h.
            sim = make();
            for (int i = 0; i < 20 * 60 && sim->state().speedKmh < 60; ++i) sim->update(dt, {1, 0, 0, false});
            {
                const auto q = sim->state().chassis.rotation;
                const float fx = 2 * (q[0] * q[2] + q[3] * q[1]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
                const float v = sim->state().speedKmh / 3.6f;
                const float l = std::hypot(fx, fz);
                sim->nudge(0, 2.0, {fx / l * v, 0, fz / l * v});
            }
            double air = 0, settle = -1, maxTilt = 0;
            int bounces = 0, prevOn = 0;
            bool landed = false;
            for (int i = 0; i < 6 * 60; ++i) {
                sim->update(dt, {0.5f, 0, 0, false});
                const auto s = sim->state();
                const auto& q = s.chassis.rotation;
                const double upY = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
                if (!landed) {
                    air += dt;
                    if (s.wheelsOnGround > 0) landed = true;
                } else {
                    maxTilt = std::max(maxTilt, std::acos(std::clamp(upY, -1.0, 1.0)) * 180 / M_PI);
                    if (prevOn > 0 && s.wheelsOnGround == 0) ++bounces;
                    if (settle < 0 && i > 30 && s.wheelsOnGround == int(s.wheels.size())) settle = (i + 1) * dt - air;
                }
                prevOn = s.wheelsOnGround;
            }
            std::printf("%-14s 0-60 %5.2f s  0-100 %5.2f s  top %6.1f km/h  brake %3.0f-0 %5.1f m (%.2f s)  turn circle @%.0f km/h %5.1f m  "
                        "jump: air %.2f s, max tilt %.0f deg, %d bounce(s), all wheels down after %.2f s\n",
                        setup.name.c_str(), t60, t100, top, from, brake, tb, n ? vsum / n : 0.0f, radius > 0 ? 2 * radius : -1, air, maxTilt, bounces,
                        settle);
            return 0;
        }
        if (argc > 4 && std::string(argv[4]) == "--surfaces") {
            const fuel::GroundMaps ground = fuel::buildGroundMaps(world, terrain, 2334);
            const fuel::RoadNetwork roads = fuel::loadRoads(world);
            const fuel::SurfaceMap map{&ground, &roads};
            struct Spot {
                const char* label;
                double x, z;
                fuel::GripCategory want;
            };
            const Spot spots[] = {{"tarmac road", -51865, -30203, fuel::GripCategory::Tarmac},
                                  {"grass (Yellowstone hub)", -51358, -30043, fuel::GripCategory::Grass},
                                  {"dune sand", -1000, -40000, fuel::GripCategory::Sand}};
            // A heading from (x, z) along which the ground stays in the wanted grip category and
            // flat for 400 m: score = metres in the category minus a slope penalty.
            auto score = [&](double x, double z, float yaw, fuel::GripCategory want) {
                const double h0 = terrain.smoothHeightAt(x, z);
                double metres = 0;
                for (int d = 0; d <= 400; d += 10) {
                    const double px = x + std::sin(yaw) * d, pz = z + std::cos(yaw) * d;
                    if (fuel::gripCategory(map.at(px, pz)) != want) break;
                    metres = d;
                }
                return metres - 3 * std::fabs(terrain.smoothHeightAt(x + std::sin(yaw) * 400, z + std::cos(yaw) * 400) - h0);
            };
            std::printf("%-24s %-22s %9s %9s %10s  %s\n", "spot", "ground type", "top km/h", "0-80 s", "brake m", "wheel surfaces at 80 km/h");
            auto runTest = [&](const char* label, const Spot& sp, float yaw, const fuel::SurfaceQuery& query) {
            fuel::DriveWorld sim(terrain);
            sim.setSurfaceQuery(query);
            const int under = query(sp.x, sp.z);
            auto surfaces = [&](const fuel::VehicleState& s) {
                std::string out;
                for (int t : s.wheelSurface) out += (out.empty() ? "" : ",") + std::string(t < 0 ? "air" : fuel::surfaceName(t));
                return out;
            };
            // Run 1: full throttle for 15 s -> top speed. Run 2: to 80 km/h, then brake to a stop.
            sim.spawn(setup, sp.x, sp.z, yaw);
            for (int i = 0; i < 120; ++i) sim.update(1.0f / 60, {});
            float top = 0;
            for (int i = 0; i < 15 * 60; ++i) {
                sim.update(1.0f / 60, {1, 0, 0, false});
                top = std::max(top, sim.state().speedKmh);
            }
            sim.spawn(setup, sp.x, sp.z, yaw);
            for (int i = 0; i < 120; ++i) sim.update(1.0f / 60, {});
            double t = 0;
            float s80 = -1;
            std::string wheels;
            while (t < 30) {
                sim.update(1.0f / 60, {1, 0, 0, false});
                t += 1.0 / 60;
                const fuel::VehicleState s = sim.state();
                if (s.speedKmh >= 80) {
                    s80 = float(t);
                    wheels = surfaces(s);
                    break;
                }
            }
            double brake = -1;
            if (s80 > 0) {
                const auto p0 = sim.state().chassis.position;
                for (int i = 0; i < 20 * 60; ++i) {
                    sim.update(1.0f / 60, {0, 1, 0, false});
                    const fuel::VehicleState s = sim.state();
                    if (s.speedKmh < 1.0f) {
                        brake = std::hypot(s.chassis.position[0] - p0[0], s.chassis.position[2] - p0[2]);
                        break;
                    }
                }
            }
            std::printf("%-24s %-22s %9.1f %9.2f %10.1f  %s\n", label, fuel::surfaceName(under), top, s80, brake, wheels.c_str());
            };
            for (const Spot& spot : spots) {
                // The test needs a straight, flat run of the wanted surface: take the best start
                // and heading within 4 km (score = metres of surface ahead, up to 400, minus a
                // slope penalty and 1 per 50 m of distance from the given point).
                Spot sp = spot;
                float yaw = 0;
                double best = -1e9;
                auto consider = [&](double cx, double cz, float heading) {
                    const double sc = score(cx, cz, heading, spot.want) - std::hypot(cx - spot.x, cz - spot.z) / 50;
                    if (sc > best) best = sc, yaw = heading, sp.x = cx, sp.z = cz;
                };
                if (spot.want == fuel::GripCategory::Tarmac) {
                    for (const auto& line : roads.lines) {
                        if (!fuel::isTarmacRoad(line.type)) continue;
                        for (size_t k = 0; k + 1 < line.points.size(); ++k) {
                            const auto& p = line.points[k];
                            if (std::hypot(p[0] - spot.x, p[1] - spot.z) > 4000) continue;
                            const float heading = std::atan2(line.points[k + 1][0] - p[0], line.points[k + 1][1] - p[1]);
                            consider(p[0], p[1], heading);
                            consider(p[0], p[1], heading + float(M_PI));
                        }
                    }
                } else {
                    for (int dx = -16; dx <= 16; ++dx)
                        for (int dz = -16; dz <= 16; ++dz) {
                            const double cx = spot.x + dx * 250.0, cz = spot.z + dz * 250.0;
                            if (fuel::gripCategory(map.at(cx, cz)) != spot.want) continue;
                            for (int h = 0; h < 16; ++h) consider(cx, cz, h * float(M_PI) / 8);
                        }
                }
                if (std::hypot(sp.x - spot.x, sp.z - spot.z) > 30)
                    std::printf("  (%s: moved to (%.0f, %.0f) heading %.0f deg for a straight flat run)\n", spot.label, sp.x, sp.z, yaw * 180 / M_PI);
                runTest(spot.label, sp, yaw, [&](double px, double pz) { return map.at(px, pz); });
            }
            // The same test at the grass hub with one surface type forced under all wheels,
            // so only the grip differs (no slope or road-edge effects).
            std::printf("\nforced surface at the grass hub (same start and heading):\n");
            const Spot hub = {"", -51358, -30043, fuel::GripCategory::Grass};
            float hubYaw = 0;
            double hubBest = -1e9;
            for (int h = 0; h < 16; ++h) {
                const double sc = score(hub.x, hub.z, h * float(M_PI) / 8, hub.want);
                if (sc > hubBest) hubBest = sc, hubYaw = h * float(M_PI) / 8;
            }
            for (int type : {11, 13, 22, 6, 8, 30, 15}) {
                char label[64];
                std::snprintf(label, sizeof label, "%s grip %.2f", fuel::gripCategoryName(fuel::gripCategory(type)), fuel::surfaceGrip(type));
                runTest(label, hub, hubYaw, [type](double, double) { return type; });
            }
            return 0;
        }
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
            if (const fuel::fv::FuelVehicle* m = sim.model(0); m && std::getenv("FUEL_TRACE")) {
                std::printf("      loads");
                for (int w = 0; w < m->wheelCount(); ++w) std::printf(" %.3f", m->wheelLoad(w));
                std::printf("  slip");
                for (int w = 0; w < m->wheelCount(); ++w) std::printf(" %.2f", m->wheelUtil(w) > 1e4f ? -1.0f : m->wheelUtil(w));
                std::printf("  rims");
                for (int w = 0; w < m->wheelCount(); ++w) std::printf(" %.1f", m->body().S.auxVel[1 + w]);
                std::printf("  steer accel %.3f  airtime %.2f\n", m->steer().accel, m->airtime());
            }
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
