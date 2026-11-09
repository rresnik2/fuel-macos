// fueldrive: drives a vehicle on the world terrain without a window and prints
// what happens (a check for the physics and its tuning).
//
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT [X Z]
//     (CUSTOM VEHICLES: VEHICLE_DPC "-" = the archive of the custom vehicle VARIANT; an absolute path is used as is)
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --surfaces
//     the same throttle-then-brake test on tarmac, grass and sand: top speed,
//     0-80 km/h time, braking distance from 80 km/h and the surfaces under the wheels
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --behaviour [SURFACE_TYPE]
//     on FUEL's terrain, on a flat stretch, with one surface type under the wheels (default 11 =
//     asphalt): 0-100 km/h time, top speed, braking distance from 100 (or the top speed), turning
//     circle at 30 km/h, and a jump (2 m drop at 60 km/h): landing peak, bounces, settle time
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --start-hold [RACE]
//     PK01: the pre-start hold (0x5279c0 -> 0x62ce40) at a career race's start (default CR2-Yell-CP): pushed and
//     throttled in race state 4 the vehicle stays on FUEL's start pose (heading from EMD_SetStartRot); released
//     in state 5 it drives off; overturned it respawns at a breadcrumb (not the start); Player_G vf137 0x42bd30
//     puts it 0.3 m over a node. Exit status 1 on a failure.
//   fueldrive DATA_DIR VEHICLE_DPC VARIANT --flat [--powers A,B,..] [--pp N=V] [--ppmul N=F] [--surface T]
//                                                 [--secs S] [--trace SECONDS] [--nitro-hold T0-T1,..]
//     [port bench] full throttle from rest on a synthetic flat plane (no world, no terrain detail, one surface
//     type, default 11 = asphalt), once per engine_power value (FuelVehicle::powerScale, default 1): 0-60 / 0-100 /
//     0-130 km/h, top speed in S seconds (default 40), the gear changes, the time the driven wheels slide before
//     100 km/h. --pp / --ppmul set / multiply a VehiclePP value (repeatable). --trace prints every frame up to that
//     time: speed, gear, rpm, throttle, clutch, pitch, and per wheel load, slip ratio 0x523c50, rim speed, dOmega.
//     --nitro-hold T0-T1[,..]: the Nitro button held over those seconds (--nitro-ungoverned: the boost's torque outside
//     the traction governor, as before the merge); --gov R,DOWN,UP, --no-gov-clutch, --no-governor, --no-hold-release:
//     the governor's bench knobs (notes truck-power.md). --straight: steer for the start line 30 m ahead (bikes and
//     SUV_ALL_2 drift off an unsteered straight; the U1 stage sweep uses it). The FLAT line ends with the speed at the end.
//     --torque S (per-vehicle torqueScale), --lift M, --widen F,R (physical suspension lift / track widening, metres);
//     --handling: ride height, 2 m / 5 m landings, step steer at 60/80/100, full-lock circle and the steepest side
//     slope it stays upright on, instead of the launch (upgrade plan 2.2.1).
//     --upgrades e,s,t,w,np,nt (or FUEL_UPGRADES; each 0..3): the vehicle's upgrade stages (core/upgrades.hpp), applied
//     per engine_power (the 2.6 torque cap reads it) after --pp / --ppmul and before --torque / --lift / --widen, which
//     then override the stages' values when given. The FLAT / HANDLING line gets an UPG line before it.

#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "core/upgrades.hpp"  // [upgrades U1]
#include "core/dpc.hpp"
#include "core/race_rules.hpp"
#include "core/races.hpp"
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
#include <vector>

namespace {

// CUSTOM VEHICLES: VEHICLE_DPC relative to DATA_DIR, absolute, or "-" for a custom vehicle's pack archive.
std::string dpcPath(const std::string& data, const std::string& arg, const char* variant = nullptr) {
    fuel::custom::setDataDir(data);
    if (arg == "-" && variant)
        if (const fuel::custom::Vehicle* v = fuel::custom::find(variant)) return v->dpc;
    return !arg.empty() && arg[0] == '/' ? arg : data + "/" + arg;
}

double gTraceUntil = 0, gTraceT = 0;
void substepTrace(const fuel::fv::FuelVehicle& m, int phase) {
    if (gTraceT > gTraceUntil) return;
    std::printf("    ss%d rpm %6.0f a0 %8.1f |", phase, m.rpm(), m.auxAcc(0));
    for (int k = 0; k < m.wheelCount(); ++k) {
        const auto& r = m.wheel(k);
        const float fl = -(r.fric.x * r.dirW.x + r.fric.y * r.dirW.y + r.fric.z * r.dirW.z);
        const auto pv = m.pointVelocity(r.hubW);
        const float vg = pv.x * r.dirW.x + pv.y * r.dirW.y + pv.z * r.dirW.z;
        std::printf(" w%d %s r %5.2f rim %6.2f vg %6.2f F %+6.2f acc %+8.1f dO %6.1f", k, r.slidingDriven ? "S" : "g", std::min(m.wheelSlipRatio(k), 99.99f),
                    m.rimSpeed(k), vg, fl, m.auxAcc(1 + k), m.driveDOmega(k));
    }
    std::printf("\n");
}

// --flat --handling: ride height, landings, step steer, circle and side-slope rollover (upgrade plan section 2.2.1).
int handlingBench(const fuel::VehicleSetup& setup, const fuel::Heightfield& plane, int type, float power) {
    constexpr float dt = 1.0f / 60;
    fuel::fv::FuelVehicle::powerScale = power;
    auto upY = [](const fuel::VehicleState& s) {
        const auto& q = s.chassis.rotation;
        return 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
    };
    auto pitchDeg = [](const fuel::VehicleState& s) {
        const auto& q = s.chassis.rotation;
        return std::asin(std::clamp(2 * (q[1] * q[2] - q[3] * q[0]), -1.0f, 1.0f)) * 180 / float(M_PI);
    };
    auto rollDeg = [](const fuel::VehicleState& s) {  // body x axis . world up
        const auto& q = s.chassis.rotation;
        return std::asin(std::clamp(2 * (q[0] * q[1] + q[3] * q[2]), -1.0f, 1.0f)) * 180 / float(M_PI);
    };
    auto make = [&](const fuel::Heightfield& hf, double x, double z) {
        auto w = std::make_unique<fuel::DriveWorld>(hf);
        w->setSurfaceQuery([type](double, double) { return type; });
        w->spawn(setup, x, z, 0);
        for (int i = 0; i < 90; ++i) w->update(dt, {});
        return w;
    };
    auto speedUp = [&](fuel::DriveWorld& w, float kmh) {
        for (int i = 0; i < 25 * 60 && w.state().speedKmh < kmh; ++i) w.update(dt, {1, 0, 0, false});
        return w.state().speedKmh >= kmh;
    };
    // Ride: model origin above the plane (100 m) after 1.5 s at rest, mean wheel load.
    auto w = make(plane, 0, 0);
    const fuel::fv::FuelVehicle* m = w->model(0);
    float load = 0;
    for (int k = 0; k < m->wheelCount(); ++k) load += m->wheelLoad(k) / float(m->wheelCount());
    const float ride = float(w->state().chassis.position[1] - 100.0);
    const float comY = float(w->state().chassis.position[1] - 100.0) + m->comLocal().y;
    std::printf("HANDLING %s power %.2f torque %.2f lift %.3f widen %.3f,%.3f | ride %.3f com %.3f half-track %.3f,%.3f load %.3f", setup.name.c_str(),
                power, setup.torqueScale, setup.lift, setup.widenFront, setup.widenRear, ride, comY, std::fabs(m->hubModel(0).x),
                std::fabs(m->hubModel(1 % m->wheelCount()).x), load);
    // Landings: 2 m at 60 km/h and 5 m at 80 km/h, half throttle; bounces, settle, chassis hits, pitch swings, wreck.
    for (const auto [drop, kmh] : {std::pair{2.0, 60.0f}, std::pair{5.0, 80.0f}}) {
        auto d = make(plane, 0, 0);
        speedUp(*d, kmh);
        {
            const auto q = d->state().chassis.rotation;
            const float fx = 2 * (q[0] * q[2] + q[3] * q[1]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
            const float v = d->state().speedKmh / 3.6f, l = std::hypot(fx, fz);
            d->nudge(0, drop, {fx / l * v, 0, fz / l * v});
        }
        double air = 0, settle = -1, peakLoad = 0;
        int bounces = 0, prevOn = 0, hits = 0, swings = 0, frames = 0, bottoms = 0;
        float maxTilt = 0, prevRate = 0, prevPitch = 0;
        bool landed = false, wrecked = false;
        const int r0 = d->respawns(0);
        for (int i = 0; i < 6 * 60; ++i) {
            d->update(dt, {0.5f, 0, 0, false});
            const auto s = d->state();
            const fuel::fv::FuelVehicle* dm = d->model(0);
            wrecked = wrecked || s.wrecked || d->respawns(0) != r0;
            if (!landed) {
                air += dt;
                if (s.wheelsOnGround > 0) landed = true;
                prevPitch = pitchDeg(s);
                continue;
            }
            ++frames;
            for (int k = 0; k < dm->wheelCount(); ++k) peakLoad = std::max(peakLoad, double(dm->wheelLoad(k)));
            maxTilt = std::max(maxTilt, std::acos(std::clamp(upY(s), -1.0f, 1.0f)) * 180 / float(M_PI));
            if (prevOn > 0 && s.wheelsOnGround == 0) ++bounces;
            if (s.scrapeSpeed > 0) ++hits;  // the chassis (not a wheel) touching the ground
            for (int k = 0; k < dm->wheelCount(); ++k)
                if (dm->wheelLoad(k) >= 0.99f * std::min(dm->param(dm->front(k) ? 71 : 72), dm->wheel(k).radius - 0.2f)) {
                    ++bottoms;
                    break;
                }
            const float p = pitchDeg(s), rate = (p - prevPitch) / dt;
            if (frames > 1 && rate * prevRate < 0 && std::fabs(rate) > 10) ++swings;  // pitch-rate reversals faster than 10 deg/s
            prevRate = rate, prevPitch = p;
            if (settle < 0 && i > 30 && s.wheelsOnGround == int(s.wheels.size())) settle = air + frames * dt - air;
            prevOn = s.wheelsOnGround;
        }
        std::printf(" | drop%.0f air %.2f bnc %d settle %.2f chassis %d bottom %d swings %d tilt %.0f peak %.3f%s", drop, air, bounces, settle, hits, bottoms,
                    swings, maxTilt, peakLoad, wrecked ? " WRECK" : "");
    }
    // Step steer: full right lock at speed, throttle held; the largest roll, the fewest wheels down, rolled over.
    for (const float kmh : {60.0f, 80.0f, 100.0f}) {
        auto d = make(plane, 0, 0);
        if (!speedUp(*d, kmh)) {
            std::printf(" | step%.0f -", kmh);
            continue;
        }
        float maxRoll = 0;
        int minOn = 99;
        bool rolled = false;
        const int r0 = d->respawns(0);
        for (int i = 0; i < 4 * 60; ++i) {
            d->update(dt, {1, 0, 1, false});
            const auto s = d->state();
            maxRoll = std::max(maxRoll, std::fabs(rollDeg(s)));
            minOn = std::min(minOn, s.wheelsOnGround);
            rolled = rolled || upY(s) < 0.3f || d->respawns(0) != r0 || s.wrecked;
        }
        std::printf(" | step%.0f roll %.1f min %d%s", kmh, maxRoll, minOn, rolled ? " ROLLED" : "");
    }
    // Circle: full lock, full throttle 15 s from 20 km/h: the largest roll, speed of the first wheel lift, rolled.
    {
        auto d = make(plane, 0, 0);
        speedUp(*d, 20);
        float maxRoll = 0, liftKmh = -1, vmax = 0;
        bool rolled = false;
        const int r0 = d->respawns(0);
        for (int i = 0; i < 15 * 60; ++i) {
            d->update(dt, {1, 0, 1, false});
            const auto s = d->state();
            maxRoll = std::max(maxRoll, std::fabs(rollDeg(s)));
            vmax = std::max(vmax, s.speedKmh);
            if (liftKmh < 0 && s.wheelsOnGround < int(s.wheels.size()) - 1) liftKmh = s.speedKmh;
            rolled = rolled || upY(s) < 0.3f || d->respawns(0) != r0 || s.wrecked;
        }
        std::printf(" | circle roll %.1f lift2@%.0f vmax %.0f%s", maxRoll, liftKmh, vmax, rolled ? " ROLLED" : "");
    }
    // Side slope: planes tilted about the vehicle's heading (rising to +x), 20 km/h straight along it for 4 s:
    // the steepest slope it stays upright on (2.5 deg steps).
    float maxSlope = 0;
    for (float deg = 10; deg <= 75; deg += 2.5f) {
        fuel::Heightfield hf;
        hf.width = hf.height = 17;
        hf.spacing = 500;
        hf.originX = hf.originZ = -4000;
        hf.metresPerUnit = 0.25f;
        hf.raw.resize(size_t(hf.width) * hf.height);
        for (uint32_t r = 0; r < hf.height; ++r)
            for (uint32_t c = 0; c < hf.width; ++c)
                hf.raw[size_t(r) * hf.width + c] = uint16_t((4100.0 + (hf.originX + c * hf.spacing) * std::tan(deg * M_PI / 180)) / hf.metresPerUnit);
        auto d = make(hf, 0, 0);
        bool rolled = false;
        const int r0 = d->respawns(0);
        for (int i = 0; i < 4 * 60; ++i) {
            const float v = d->state().speedKmh;
            d->update(dt, {std::clamp((20 - v) * 0.1f, 0.0f, 1.0f), 0, 0, false});
            const auto s = d->state();
            rolled = rolled || upY(s) < 0.3f || d->respawns(0) != r0 || s.wrecked;
        }
        if (rolled) break;
        maxSlope = deg;
    }
    std::printf(" | slope %.1f\n", maxSlope);
    return 0;
}

// --flat --rough: [upgrades U1, CHANGED: user, suspension = travel] a synthetic off-road course on a 1 m heightfield,
// driven at full throttle steering for the centre line: 100 m run-up, WHOOPS (0.35 m sine humps every 10 m, 200 m),
// ROCKS (0.25 m random bumps on a 4 m grid, 200 m), RUTS (two 0.3 m deep grooves with 0.3 m side-to-side waves, 200 m),
// a JUMP (2.5 m ramp over 12 m, then a 3 m drop to the floor), a 300 m run-out, then a 20 deg SIDE SLOPE (300 m,
// rising to +x). Per section: time, mean speed, airtime, bottoming frames (a wheel at its max depth), chassis-contact
// frames, wheel contact share, the largest tilt; resets (wreck / overturn) for the whole run.
int roughBench(const fuel::VehicleSetup& setup, int type, float power) {
    constexpr float dt = 1.0f / 60;
    fuel::fv::FuelVehicle::powerScale = power;
    fuel::Heightfield hf;
    hf.width = 161, hf.height = 1701;
    hf.spacing = 1;
    hf.originX = -80, hf.originZ = -100;
    hf.metresPerUnit = 0.005f;
    hf.raw.resize(size_t(hf.width) * hf.height);
    auto rnd = [](int a, int b) {
        uint32_t h = uint32_t(a) * 73856093u ^ uint32_t(b) * 19349663u;
        h ^= h >> 13, h *= 0x5bd1e995u, h ^= h >> 15;
        return float(h & 0xffff) / 65535.0f;
    };
    // sections along z (metres): run-up 0..100, whoops 100..300, rocks 300..500, ruts 500..700, jump 700..712 + drop,
    // run-out 712..1000, side slope 1000..1300 (then flat)
    auto height = [&](float x, float z) -> float {
        float h = 0;
        if (z >= 100 && z < 300) h = 0.35f * std::pow(std::sin(float(M_PI) * (z - 100) / 10.0f), 2.0f);
        else if (z >= 300 && z < 500) {
            const int gx = int(std::floor(x / 4)), gz = int(std::floor(z / 4));
            const float fx = x / 4 - float(gx), fz = z / 4 - float(gz);
            auto r = [&](int a, int b) { return rnd(a, b) * 0.25f; };
            const float a = r(gx, gz) * (1 - fx) + r(gx + 1, gz) * fx, b = r(gx, gz + 1) * (1 - fx) + r(gx + 1, gz + 1) * fx;
            h = a * (1 - fz) + b * fz;
        } else if (z >= 500 && z < 700) {
            const float c = 0.3f * std::sin((z - 500) / 15.0f);  // the ruts wander side to side
            for (float g : {-0.9f, 0.9f}) h -= 0.3f * std::exp(-std::pow((x - c - g) / 0.35f, 2.0f));
            h += 0.3f;
        } else if (z >= 700 && z < 712) h = 2.5f * (z - 700) / 12.0f + 0.3f - 0.3f * 0;  // the ramp
        if (z >= 712 && z < 715) h = -0.5f;  // the drop's floor (lower than the ramp's 2.5 m lip)
        if (z >= 1000 && z < 1300) h += std::tan(20.0f * float(M_PI) / 180) * x;
        return h;
    };
    for (uint32_t r = 0; r < hf.height; ++r)
        for (uint32_t c = 0; c < hf.width; ++c) {
            const float x = hf.originX + float(c), z = hf.originZ + float(r);
            hf.raw[size_t(r) * hf.width + c] = uint16_t(std::lround((60.0f + height(x, z)) / hf.metresPerUnit));
        }
    fuel::DriveWorld w(hf);
    w.setSurfaceQuery([type](double, double) { return type; });
    w.spawn(setup, 0, 0, 0);
    for (int i = 0; i < 90; ++i) w.update(dt, {});
    const fuel::fv::FuelVehicle* m = w.model(0);
    struct Sec {
        const char* name;
        float z0, z1;
        double t = 0, air = 0, contact = 0, frames = 0;
        int bottoms = 0, chassis = 0;
        float tilt = 0;
    };
    Sec secs[] = {{"whoops", 100, 300}, {"rocks", 300, 500}, {"ruts", 500, 700}, {"jump", 700, 800}, {"slope", 1000, 1300}};
    const int r0 = w.respawns(0);
    bool wrecked = false;
    double t = 0;
    for (int i = 0; i < 120 * 60; ++i) {
        const fuel::VehicleState s = w.state();
        const double z = s.chassis.position[2];
        if (z > 1300) break;
        fuel::DriveInput in{1, 0, 0, false};
        const auto& q = s.chassis.rotation;
        const float fx = 2 * (q[0] * q[2] + q[3] * q[1]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
        const float err = std::remainder(std::atan2(float(-s.chassis.position[0]), 25.0f) - std::atan2(fx, fz), 2 * float(M_PI));
        in.steer = std::clamp(-2.0f * err, -1.0f, 1.0f);
        w.update(dt, in);
        t += dt;
        const fuel::VehicleState n = w.state();
        wrecked = wrecked || n.wrecked;
        const double nz = n.chassis.position[2];
        for (Sec& sc : secs) {
            if (nz < sc.z0 || nz >= sc.z1) continue;
            sc.t += dt, sc.frames += 1;
            if (n.wheelsOnGround == 0) sc.air += dt;
            sc.contact += double(n.wheelsOnGround) / double(n.wheels.size());
            if (n.scrapeSpeed > 0) ++sc.chassis;
            for (int k = 0; k < m->wheelCount(); ++k)
                if (m->wheelLoad(k) >= 0.99f * m->param(m->front(k) ? 71 : 72)) {
                    ++sc.bottoms;
                    break;
                }
            const auto& qq = n.chassis.rotation;
            sc.tilt = std::max(sc.tilt, std::acos(std::clamp(1 - 2 * (qq[0] * qq[0] + qq[2] * qq[2]), -1.0f, 1.0f)) * 180 / float(M_PI));
        }
    }
    std::printf("ROUGH %s power %.2f", setup.name.c_str(), power);
    for (const Sec& sc : secs)
        std::printf(" | %s t %.2f v %.1f air %.2f bottom %d chassis %d contact %.2f tilt %.0f", sc.name, sc.t, sc.t > 0 ? (sc.z1 - sc.z0) / sc.t * 3.6 : 0.0,
                    sc.air, sc.bottoms, sc.chassis, sc.frames > 0 ? sc.contact / sc.frames : 0.0, sc.tilt);
    std::printf(" | total %.2f resets %d%s\n", t, w.respawns(0) - r0, wrecked ? " WRECK" : "");
    return 0;
}

// --flat: see the header. Returns the exit status.
int flatBench(const std::string& data, int argc, char** argv) {
    std::vector<float> powers{1.0f};
    std::vector<std::pair<int, float>> set, mul;
    int type = 11;
    float secs = 40, trace = 0, lift = 0, widenF = 0, widenR = 0, torque = 1;
    bool handling = false;
    bool rough = false;  // --rough: the off-road course (roughBench)
    bool straight = false;  // --straight: steer for the start line 30 m ahead (as fuelview's --ai-check test player)
    std::string nitroHold;  // --nitro-hold T0-T1[,..]: the Nitro button held over those seconds of the run (nitro::scriptedHold)
    for (int a = 5; a < argc; ++a) {
        const std::string o = argv[a];
        const char* v = a + 1 < argc ? argv[a + 1] : "";
        if (o == "--powers") {
            powers.clear();
            std::stringstream ss(v);
            for (std::string x; std::getline(ss, x, ',');) powers.push_back(std::stof(x));
            ++a;
        } else if (o == "--pp" || o == "--ppmul") {
            const std::string s = v;
            const size_t eq = s.find('=');
            if (eq == std::string::npos) throw std::runtime_error(o + " wants N=V");
            (o == "--pp" ? set : mul).push_back({std::stoi(s.substr(0, eq)), std::stof(s.substr(eq + 1))});
            ++a;
        } else if (o == "--surface") {
            type = std::atoi(v), ++a;
        } else if (o == "--secs") {
            secs = std::stof(v), ++a;
        } else if (o == "--trace") {
            trace = std::stof(v), ++a;
        } else if (o == "--gov") {
            std::sscanf(v, "%f,%f,%f", &fuel::fv::FuelVehicle::kGovRatio, &fuel::fv::FuelVehicle::kGovDown, &fuel::fv::FuelVehicle::kGovUp), ++a;
        } else if (o == "--lift") {
            lift = std::stof(v), ++a;
        } else if (o == "--widen") {
            std::sscanf(v, "%f,%f", &widenF, &widenR), ++a;
        } else if (o == "--torque") {
            torque = std::stof(v), ++a;
        } else if (o == "--upgrades") {
            fuel::upg::Stages st;
            std::string err;
            if (!fuel::upg::parseStages(v, st, &err)) throw std::runtime_error("bad --upgrades: " + err);
            fuel::upg::setPlayerStages(st), ++a;
        } else if (o == "--nitro-ungoverned") {
            fuel::fv::FuelVehicle::nitroGoverned = false;
        } else if (o == "--nitro-hold") {
            nitroHold = v, ++a;
        } else if (o == "--hold") {  // [upgrades U1] FuelVehicle::kHoldFloor, kHoldRatio
            std::sscanf(v, "%f,%f", &fuel::fv::FuelVehicle::kHoldFloor, &fuel::fv::FuelVehicle::kHoldRatio), ++a;
        } else if (o == "--straight") {
            straight = true;
        } else if (o == "--rough") {
            rough = true;
        } else if (o == "--handling") {
            handling = true;
        } else if (o == "--no-hold-release") {
            fuel::fv::FuelVehicle::holdRelease = false;
        } else if (o == "--no-gov-clutch") {
            fuel::fv::FuelVehicle::govClutch = false;
        } else if (o == "--no-governor") {
            fuel::fv::FuelVehicle::governorOn = false;
        } else if (o == "--substeps") {
            gTraceUntil = std::stof(v), ++a;
            fuel::fv::FuelVehicle::substepHook = substepTrace;
        } else {
            throw std::runtime_error("unknown option " + o);
        }
    }
    // A flat plane 160 km across at 100 m: the B-spline of a constant map (no detail fractals, no roads).
    fuel::Heightfield plane;
    plane.width = plane.height = 17;
    plane.spacing = 10000;
    plane.originX = plane.originZ = -80000;
    plane.metresPerUnit = 1;
    plane.raw.assign(size_t(plane.width) * plane.height, 100);
    fuel::DpcFile vehicleDpc(dpcPath(data, argv[2], argv[3]));  // CUSTOM VEHICLES: "-" = the pack of VARIANT, or an absolute path
    std::istringstream f(fuel::custom::readScript(data + "/CommonComponent/Param/VehiclesParam.tsc"));
    std::stringstream text;
    text << f.rdbuf();
    const auto params = fuel::parseVehiclesParam(text.str());
    const auto it = params.find(argv[3]);
    if (it == params.end()) throw std::runtime_error(std::string("no parameters for ") + argv[3]);
    std::vector<float> P = it->second;
    for (const auto& [n, x] : set) {
        if (size_t(n) >= P.size()) P.resize(size_t(n) + 1, 0.0f);
        P[size_t(n)] = x;
    }
    for (const auto& [n, x] : mul)
        if (size_t(n) < P.size()) P[size_t(n)] *= x;
    const fuel::VehicleSetup base = fuel::makeVehicleSetup(vehicleDpc, argv[3], P);
    // [upgrades U1] the stages per power (the torque cap), then the explicit knobs.
    auto staged = [&](float power) {
        fuel::VehicleSetup s = base;
        const fuel::upg::Result r = fuel::upg::apply(s, fuel::upg::playerStages(), power);
        if (!fuel::upg::playerStages().stock())
            std::printf("UPG %s power %.3f stages %s %s\n", s.name.c_str(), power, fuel::upg::formatStages(r.stages).c_str(), fuel::upg::describe(r).c_str());
        if (torque != 1) s.torqueScale = torque;
        if (lift != 0) s.lift = lift;
        if (widenF != 0 || widenR != 0) s.widenFront = widenF, s.widenRear = widenR;
        return s;
    };
    constexpr float dt = 1.0f / 60;
    if (handling) return handlingBench(staged(powers.front()), plane, type, powers.front());
    if (rough) {
        for (const float p : powers) roughBench(staged(p), type, p);
        return 0;
    }
    for (const float power : powers) {
        const fuel::VehicleSetup setup = staged(power);
        fuel::fv::FuelVehicle::powerScale = power;
        fuel::DriveWorld w(plane);
        w.setSurfaceQuery([type](double, double) { return type; });
        w.spawn(setup, 0, 0, 0);
        for (int i = 0; i < 90; ++i) w.update(dt, {});
        const fuel::fv::FuelVehicle* m = w.model(0);
        double t = 0, t60 = -1, t100 = -1, t130 = -1, tTop = 0, slide100 = 0, dist = 0, d5 = -1, d10 = -1;
        float top = 0;
        int gear = w.state().gear, maxGear = gear;
        std::string shifts;
        const auto p0 = w.state().chassis.position;
        for (int i = 0; i < int(secs * 60); ++i) {
            gTraceT = t + dt;
            fuel::DriveInput in{1, 0, 0, false};
            in.nitro = !nitroHold.empty() && fuel::nitro::scriptedHold(nitroHold.c_str(), t);
            if (straight) {  // aim at the start line (x = x0) 30 m ahead along +z: steer +1 = right
                const fuel::VehicleState s0 = w.state();
                const auto& q = s0.chassis.rotation;
                const float fx = 2 * (q[0] * q[2] + q[3] * q[1]), fz = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
                const float err = std::remainder(std::atan2(float(p0[0] - s0.chassis.position[0]), 30.0f) - std::atan2(fx, fz), 2 * float(M_PI));
                in.steer = std::clamp(-2.0f * err, -1.0f, 1.0f);
            }
            w.update(dt, in);
            t += dt;
            const fuel::VehicleState s = w.state();
            const float v = s.speedKmh;
            if (v > top + 0.05f) top = v, tTop = t;
            if (t60 < 0 && v >= 60) t60 = t;
            if (t100 < 0 && v >= 100) t100 = t;
            if (t130 < 0 && v >= 130) t130 = t;
            bool sliding = false;
            for (int k = 0; k < m->wheelCount(); ++k) {
                const bool drv = m->front(k) ? m->param(46) > 0.5f : m->param(47) > 0.5f;
                if (drv && m->wheelLoad(k) > 1e-6f && m->wheelSlipRatio(k) > 1) sliding = true;
            }
            if (t100 < 0 && sliding) slide100 += dt;
            if (s.gear != gear) {
                char b[32];
                std::snprintf(b, sizeof b, "%s%d@%.2f", shifts.empty() ? "" : ",", s.gear, t);
                shifts += b;
                gear = s.gear;
                maxGear = std::max(maxGear, gear);
            }
            if (t <= trace + 1e-6) {
                const auto& q = s.chassis.rotation;
                const double fy = 2 * (q[1] * q[2] - q[3] * q[0]);  // forward (+z) . world up = pitch (nose up +)
                std::printf("  t %5.2f v %6.2f g %d>%d sh %.2f cl %.2f rpm %5.0f thr %.2f gov %.2f n %.2f pitch %+5.2f |", t, v, s.gear, m->gearTarget(), m->shiftTimer(),
                            m->clutchNow(), m->rpm(), m->engineThrottle(), m->tractionGovernor(), m->nitro(), std::asin(std::clamp(fy, -1.0, 1.0)) * 180 / M_PI);
                for (int k = 0; k < m->wheelCount(); ++k) {
                    // Longitudinal friction (the wheel record's accumulated friction along the wheel's direction; + = drive)
                    // and the ground speed of the hub along it minus the rim speed (+ = the rim is slower: braking slip).
                    const auto& r = m->wheel(k);
                    const float fl = -(r.fric.x * r.dirW.x + r.fric.y * r.dirW.y + r.fric.z * r.dirW.z);
                    const auto pv = m->pointVelocity(r.hubW);
                    const float vg = pv.x * r.dirW.x + pv.y * r.dirW.y + pv.z * r.dirW.z;
                    std::printf(" w%d L %.3f r %5.2f rim %6.2f sl %+5.2f F %+6.2f dO %6.1f%s", k, m->wheelLoad(k), std::min(m->wheelSlipRatio(k), 99.99f),
                                m->rimSpeed(k), vg - m->rimSpeed(k), fl, m->driveDOmega(k), m->wheelSlipRatio(k) > 1 && m->wheelLoad(k) > 1e-6f ? "*" : " ");
                }
                std::printf("\n");
            }
            dist = std::hypot(s.chassis.position[0] - p0[0], s.chassis.position[2] - p0[2]);
            if (d5 < 0 && t >= 5 - 1e-6) d5 = dist;
            if (d10 < 0 && t >= 10 - 1e-6) d10 = dist;
        }
        std::printf("FLAT %s power %.3f 0-60 %.2f 0-100 %.2f 0-130 %.2f top %.1f at %.1f s  slide<100 %.2f s  dist %.0f m  gear %d  shifts %s  end %.1f d5 %.1f d10 %.1f\n",
                    setup.name.c_str(), power, t60, t100, t130, top, tTop, slide100, dist, maxGear, shifts.c_str(), w.state().speedKmh, d5, d10);
        std::fflush(stdout);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: fueldrive DATA_DIR VEHICLE_DPC VARIANT [X Z]\n");
        return 2;
    }
    try {
        const std::string data = argv[1];
        if (argc > 4 && std::string(argv[4]) == "--flat") return flatBench(data, argc, argv);
        fuel::DpcFile world(data + "/CommonComponent/WORLD/USA1.DPC");
        const bool nitroTest = argc > 4 && std::string(argv[4]) == "--nitro";  // [nitro]
        const bool behaviour = argc > 4 && std::string(argv[4]) == "--behaviour";
        const fuel::Heightfield terrain = fuel::loadLevelledHeightfield(world);
        fuel::DpcFile vehicleDpc(dpcPath(data, argv[2], argv[3]));  // CUSTOM VEHICLES: "-" = the pack of VARIANT, or an absolute path
        std::istringstream f(fuel::custom::readScript(data + "/CommonComponent/Param/VehiclesParam.tsc"));
        std::stringstream text;
        text << f.rdbuf();
        const auto params = fuel::parseVehiclesParam(text.str());
        const auto it = params.find(argv[3]);
        if (it == params.end()) throw std::runtime_error(std::string("no parameters for ") + argv[3]);
        fuel::VehicleSetup setup = fuel::makeVehicleSetup(vehicleDpc, argv[3], it->second);
        if (const char* ep = std::getenv("FUEL_ENGINE_POWER")) fuel::fv::FuelVehicle::powerScale = std::stof(ep);
        if (!fuel::upg::playerStages().stock())  // [upgrades U1] FUEL_UPGRADES
            std::printf("UPG %s %s\n", setup.name.c_str(), fuel::upg::describe(fuel::upg::apply(setup, fuel::upg::playerStages(), fuel::fv::FuelVehicle::powerScale)).c_str());
        std::printf("%s: %.0f kg, %zu wheels r=%.2f, %s, %.0f-%.0f rpm, %.0f Nm, %zu gears, final %.2f, nominal top %.0f km/h\n",
                    setup.name.c_str(), setup.mass, setup.wheels.size(), setup.wheels[0].radius,
                    setup.frontDriven && setup.rearDriven ? "4WD" : setup.frontDriven ? "FWD" : "RWD", setup.idleRpm, setup.maxRpm,
                    setup.peakTorque, setup.gears.size(), setup.finalDrive, setup.topSpeedKmh);

        if (std::getenv("FUEL_PRINT_SETUP")) {  // the wheels and the chassis boxes, model space (the nitro outlets' placement)
            for (const auto& w : setup.wheels) std::printf("  wheel hub %.2f %.2f %.2f r %.2f w %.2f\n", w.hub[0], w.hub[1], w.hub[2], w.radius, w.width);
            for (const auto& b : setup.chassis)
                std::printf("  box centre %.2f %.2f %.2f half %.2f %.2f %.2f\n", b.centre[0], b.centre[1], b.centre[2], b.halfExtents[0], b.halfExtents[1],
                            b.halfExtents[2]);
        }
        if (nitroTest) {  // [nitro] on a flat plane (100 m up, 64 km square, the surface type given, default asphalt): no
            // road runs long enough for the top speeds.
            const int type = argc > 5 ? std::atoi(argv[5]) : 11;
            // FUEL_ENGINE_POWER: settings.txt's engine_power for this bench (FuelVehicle::powerScale; default 1 = FUEL),
            // to check the boost against the traction governor on the extra torque (notes truck-power.md, merge).
            if (const char* ep = std::getenv("FUEL_ENGINE_POWER")) fuel::fv::FuelVehicle::powerScale = std::stof(ep);
            fuel::Heightfield flat;
            flat.width = flat.height = 64, flat.spacing = 1000, flat.originX = flat.originZ = -32000, flat.metresPerUnit = 1;
            flat.raw.assign(size_t(64 * 64), 100);
            constexpr float dt = 1.0f / 60;
            auto make = [&] {
                auto w = std::make_unique<fuel::DriveWorld>(flat);
                w->setSurfaceQuery([type](double, double) { return type; });
                w->spawn(setup, 0, -25000, 0);
                for (int i = 0; i < 90; ++i) w->update(1.0f / 60, {});
                return w;
            };
            // The boost on this vehicle: top speed with the button held (an endless tank), 0-100
            // with a 3 s boost from the start, the speed a 3 s boost adds at 80 km/h (or 60 % of the top) and near the top.
            fuel::nitro::Params& np = fuel::nitro::params();
            const float tank0 = np.tankSeconds;
            auto drive = [&](bool nitro, float until, float maxT, double* t100) {
                auto w = make();
                double t = 0;
                float top = 0;
                for (int i = 0; i < int(maxT * 60); ++i) {
                    fuel::DriveInput in{1, 0, 0, false};
                    in.nitro = nitro;
                    w->update(dt, in);
                    t += dt;
                    const float v = w->state().speedKmh;
                    top = std::max(top, v);
                    if (t100 && *t100 < 0 && v >= 100) *t100 = t;
                    if (until > 0 && v >= until) break;
                }
                return std::make_pair(std::move(w), top);
            };
            const float top = drive(false, 0, 45, nullptr).second;
            np.tankSeconds = 1e6f;
            const float topN = drive(true, 0, 45, nullptr).second;
            np.tankSeconds = tank0;
            double t100 = -1, t100n = -1;
            drive(false, 0, 25, &t100);
            drive(true, 0, 25, &t100n);
            auto gain = [&](float from, bool nitro) {  // km/h gained over 3 s from `from` (the tank full)
                auto [w, unused] = drive(false, from, 40, nullptr);
                (void)unused;
                w->resetNitro();
                const float v0 = w->state().speedKmh;
                static const bool trace = std::getenv("FUEL_NITRO_BENCH_TRACE") != nullptr;
                for (int i = 0; i < 180; ++i) {
                    fuel::DriveInput in{1, 0, 0, false};
                    in.nitro = nitro;
                    w->update(dt, in);
                    if (trace && i % 15 == 14) {
                        const fuel::VehicleState st = w->state();
                        std::printf("  trace %s from %.0f t %.2f: %6.1f km/h rpm %5.0f gear %d wheels %d skid %.2f nitro %.2f\n", nitro ? "nitro" : "none ", from,
                                    (i + 1) * dt, st.speedKmh, st.rpm, st.gear, st.wheelsOnGround, st.skid, st.nitro.amount);
                    }
                }
                return w->state().speedKmh - v0;
            };
            const float from = std::min(80.0f, top * 0.6f), near = top * 0.95f;
            const float g0 = gain(from, false), g1 = gain(from, true), h0 = gain(near, false), h1 = gain(near, true);
            std::printf("%-14s top %6.1f km/h, nitro held %6.1f (+%4.1f %%)  0-100 %5.2f s, with a 3 s boost %5.2f s  "
                        "3 s at %3.0f km/h: +%5.1f km/h (none +%4.1f)  at %3.0f: +%5.1f (none +%4.1f)\n",
                        setup.name.c_str(), top, topN, top > 0 ? (topN / top - 1) * 100 : 0.0f, t100, t100n, from, g1, g0, near, h1, h0);
            // Control: a 3 s boost from 20 km/h and from `from` straight ahead (the nose's largest pitch up, the least
            // wheels down, upright at the end), and in a held turn (steer 0.35 at `from`: heading turned, the largest
            // slip angle, upright at the end), against the same without the boost.
            struct Ctl {
                float pitch = 0, slip = 0, turned = 0, up = 1;
                int wheels = 99;
            };
            auto axes = [](const std::array<float, 4>& q, float& fx, float& fy, float& fz, float& uy) {
                const float x = q[0], y = q[1], z = q[2], w = q[3];
                fx = 2 * (x * z + w * y), fy = 2 * (y * z - w * x), fz = 1 - 2 * (x * x + y * y);  // q * (0, 0, 1)
                uy = 1 - 2 * (x * x + z * z);                                                     // (q * (0, 1, 0)).y
            };
            auto control = [&](float start, float steer, bool nitro) {
                auto [w, unused] = drive(false, start, 40, nullptr);
                (void)unused;
                w->resetNitro();
                Ctl c;
                float fx, fy, fz, uy;
                axes(w->state().chassis.rotation, fx, fy, fz, uy);
                const float h0 = std::atan2(fx, fz);
                float hPrev = h0, turned = 0;
                for (int i = 0; i < 180; ++i) {
                    fuel::DriveInput in{1, 0, steer, false};
                    in.nitro = nitro;
                    w->update(dt, in);
                    const fuel::VehicleState st = w->state();
                    axes(st.chassis.rotation, fx, fy, fz, uy);
                    c.pitch = std::max(c.pitch, std::asin(std::clamp(fy, -1.0f, 1.0f)) * 57.29578f);
                    c.wheels = std::min(c.wheels, st.wheelsOnGround);
                    const float h = std::atan2(fx, fz);
                    turned += std::remainder(h - hPrev, 6.2831853f), hPrev = h;
                    const float vx = st.velocity[0], vz = st.velocity[2], sp = std::hypot(vx, vz);
                    if (sp > 3) c.slip = std::max(c.slip, std::fabs(std::remainder(std::atan2(vx, vz) - h, 6.2831853f)) * 57.29578f);
                    c.up = uy;
                }
                c.turned = std::fabs(turned) * 57.29578f;
                return c;
            };
            const Ctl l0 = control(20, 0, false), l1 = control(20, 0, true), s0 = control(from, 0, false), s1 = control(from, 0, true);
            const Ctl t0 = control(from, 0.35f, false), t1 = control(from, 0.35f, true);
            std::printf("%-14s control (none / nitro): from 20 km/h pitch %4.1f / %4.1f deg, wheels down >= %d / %d; at %3.0f pitch %4.1f / %4.1f, "
                        ">= %d / %d wheels; turn 0.35: %5.1f / %5.1f deg, slip %4.1f / %4.1f deg, upright %.2f / %.2f\n",
                        setup.name.c_str(), l0.pitch, l1.pitch, l0.wheels, l1.wheels, from, s0.pitch, s1.pitch, s0.wheels, s1.wheels, t0.turned, t1.turned,
                        t0.slip, t1.slip, t0.up, t1.up);
            return 0;
        }
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
        if (argc > 4 && std::string(argv[4]) == "--start-hold") {  // PK01
            const std::string want = argc > 5 ? argv[5] : "CR2-Yell-CP";
            std::ifstream mf(data + "/LocalizedComponent/GameTsc/Story/miss_official.tsc");
            std::stringstream mt;
            mt << mf.rdbuf();
            fuel::Race race;
            bool found = false;
            for (const fuel::Race& r : fuel::parseMissions(mt.str()))
                if (r.name == want) race = r, found = true;
            if (!found) throw std::runtime_error("no race " + want);
            fuel::RaceRules rules;
            rules.ground = [&terrain](float x, float z) { return float(terrain.smoothHeightAt(x, z)); };
            rules.begin(race, 1, 0);
            const fuel::RaceRules::Pose sp = rules.startPose();
            const float heading = fuel::RaceRules::headingOf(sp.quat);
            int bad = 0;
            auto expect = [&bad](bool ok, const char* what, double got, double lim) {
                std::printf("  %-58s %10.3f (limit %g) %s\n", what, got, lim, ok ? "ok" : "FAILED");
                bad += !ok;
            };
            auto yawOf = [](const std::array<float, 4>& q) {  // heading of the model's +z
                const double x = q[0], y = q[1], z = q[2], w = q[3];
                return std::atan2(2 * (x * z + w * y), 1 - 2 * (x * x + y * y));
            };
            auto angle = [](double a, double b) { return std::fabs(std::remainder(a - b, 2 * M_PI)); };
            const auto& first = race.checkpoints.empty() ? race.finish : race.checkpoints[0];
            std::printf("%s: start (%.1f, %.1f, %.1f), FUEL heading %.1f deg (first gate %.1f deg)\n", race.name.c_str(), sp.pos[0], sp.pos[1], sp.pos[2],
                        heading * 180 / M_PI, std::atan2(first[0] - sp.pos[0], first[2] - sp.pos[2]) * 180 / M_PI);
            fuel::DriveWorld w(terrain);
            w.spawn(setup, sp.pos[0], sp.pos[2], heading);
            w.setStartHold([&](double& x, double& z, float& h) {
                x = sp.pos[0], z = sp.pos[2], h = heading;
                return true;
            });
            constexpr float dt = 1.0f / 60;
            w.setRaceState(4);  // PREPARING (the countdown)
            for (int i = 0; i < 30; ++i) w.update(dt, {});
            const fuel::VehicleState s0 = w.state();
            w.nudge(0, 1.5, {6, 2, -4});  // a shove
            for (int i = 0; i < 120; ++i) w.update(dt, {1, 0, 0.5f, false});  // full throttle, steering, for 2 s
            const fuel::VehicleState s1 = w.state();
            const double moved = std::hypot(s1.chassis.position[0] - s0.chassis.position[0], s1.chassis.position[2] - s0.chassis.position[2]);
            expect(moved < 0.05, "state 4: shoved and throttled 2 s, metres off the start", moved, 0.05);
            expect(std::fabs(s1.chassis.position[1] - s0.chassis.position[1]) < 0.05, "state 4: height change (m)", s1.chassis.position[1] - s0.chassis.position[1], 0.05);
            expect(s1.speedKmh < 1.0f, "state 4: speed (km/h)", s1.speedKmh, 1.0);
            expect(angle(yawOf(s1.chassis.rotation), heading) < 0.01, "state 4: heading off FUEL's start heading (rad)", angle(yawOf(s1.chassis.rotation), heading), 0.01);
            const double ds = std::hypot(s1.chassis.position[0] - sp.pos[0], s1.chassis.position[2] - sp.pos[2]);
            expect(ds < 1.5, "state 4: horizontal distance model origin - start (m)", ds, 1.5);
            w.setRaceState(5);  // RUNNING: 0x62ce40 answers false
            for (int i = 0; i < 180; ++i) w.update(dt, {1, 0, 0, false});
            const fuel::VehicleState s2 = w.state();
            const double off = std::hypot(s2.chassis.position[0] - sp.pos[0], s2.chassis.position[2] - sp.pos[2]);
            expect(off > 10, "state 5: metres driven in 3 s (the hold let go)", off, 10);
            // Overturned in the race: the vehicle's own respawn (overturn > 1 s and slow, 0x529ce0 -> 0x5288d0 10 m
            // + 0x5273b0): a breadcrumb, never the start (nor a checkpoint volume: 0x56fb00's pose has no reader).
            const int r0 = w.respawns(0);
            const auto p2 = s2.chassis.position;
            const std::array<float, 4> roof{0, 0, 1, 0};  // upside down
            w.holdPose(0, {p2[0], p2[1] + 1.0, p2[2]}, roof, {0, 0, 0});
            for (int i = 0; i < 240 && w.respawns(0) == r0; ++i) w.update(dt, {});
            const fuel::VehicleState s3 = w.state();
            expect(w.respawns(0) == r0 + 1, "overturned: respawns", w.respawns(0) - r0, 1);
            const double toStart = std::hypot(s3.chassis.position[0] - sp.pos[0], s3.chassis.position[2] - sp.pos[2]);
            const double toWreck = std::hypot(s3.chassis.position[0] - p2[0], s3.chassis.position[2] - p2[2]);
            std::printf("  respawned %.1f m from the overturn, %.1f m from the start\n", toWreck, toStart);
            expect(toStart > 5, "overturned: respawn not on the start (m from it)", toStart, 5);
            // Player_G vf137 0x42bd30: node 40 m ahead, 0.1 m over the ground (LevelManipulator 0x57e0f0 state 3).
            const double nx = sp.pos[0] + 40 * std::sin(heading), nz = sp.pos[2] + 40 * std::cos(heading);
            const double ny = terrain.smoothHeightAt(nx, nz) + 0.1;
            const std::array<float, 4> q{0, float(std::sin(heading * 0.5)), 0, float(std::cos(heading * 0.5))};
            w.placeOnNode(0, {nx, ny, nz}, q);
            const fuel::VehicleState s4 = w.state();
            expect(std::fabs(s4.chassis.position[1] - (ny + 0.3)) < 1e-3, "0x42bd30: model origin - node y (want 0.3 m)", s4.chassis.position[1] - ny, 0.3);
            expect(angle(yawOf(s4.chassis.rotation), heading) < 1e-3, "0x42bd30: heading = the node's (rad)", angle(yawOf(s4.chassis.rotation), heading), 1e-3);
            std::printf("start hold: %s\n", bad ? "FAILED" : "ok");
            return bad ? 1 : 0;
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
