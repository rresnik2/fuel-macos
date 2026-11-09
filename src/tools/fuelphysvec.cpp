// fuelphysvec: runs the test vectors of FUEL's vehicle physics decode (notes p23-vehicle-physics.md,
// "Test vectors"; values from Ghidra EmulatorHelper runs of FUEL.exe on synthetic inputs, no game data)
// through the port's functions and prints match / MISMATCH per vector. Exit status 1 on any mismatch.
//
//   fuelphysvec            all vectors
#include "core/genworld_noise.hpp"
#include "core/race_helicopter.hpp"
#include "core/f14_flyby.hpp"  // PK27
#include "core/race_rules.hpp"
#include "core/road_segments.hpp"
#include "core/resources.hpp"
#include "core/vehicle_damage.hpp"
#include "core/vehicle_links.hpp"  // PK22
#include "physics/breadcrumbs.hpp"
#include "physics/fuel_collide.hpp"
#include "physics/fuel_contacts.hpp"
#include "physics/fuel_vehicle.hpp"
#include "physics/race_ai.hpp"  // rain-ai

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

#include "physics/ai_nitro.hpp"  // [nitro]
#include "physics/nitro.hpp"
#include "core/upgrades.hpp"  // [upgrades U1]
#include "core/custom_vehicles.hpp"
#include "core/dpc.hpp"
#include "core/vehicle_params.hpp"
#include "physics/vehicle_physics.hpp"
#include <filesystem>
#include <map>
#include <fstream>
#include <sstream>
#include "core/susp_anim.hpp"        // [suspanim]
#include "core/upgrade_visuals.hpp"  // [suspanim]

namespace fuel::fv {
// Access to the vehicle's private state for single-function vectors.
struct FuelVehicleTest {
    static void set(FuelVehicle& v, float speed, const Inputs& in, const Vec3& w) {
        v.speed_ = speed;
        v.in_ = in;
        v.body_.S.w = w;
    }
    static void soft(FuelVehicle& v, float f, const float frac[kGroundClasses]) {
        for (int w = 0; w < v.nWheels_; ++w) {
            v.C_[size_t(w)].soft = f;
            for (int k = 0; k < kGroundClasses; ++k) v.frac_[size_t(w)][size_t(k)] = frac[k];
        }
    }
    static void wheels(FuelVehicle& v, std::initializer_list<int> sliding) {
        for (int w = 0; w < v.nWheels_; ++w) v.setWheelSliding(w, false);
        for (int w : sliding) v.setWheelSliding(w, true);
    }
    static void notAllLoaded(FuelVehicle& v, float t) { v.notAllLoaded_ = t; }
    static void rpmState(FuelVehicle& v, float f, float speed, int gear, const float* load, const float* fn, const float* den, const float* T,
                         float t) {
        v.body_.S.auxVel[0] = f;
        v.speed_ = speed;
        v.gear_ = gear;
        for (int w = 0; w < v.nWheels_; ++w) {
            v.wheels_[size_t(w)].load = load[w];
            v.wheels_[size_t(w)].sumFn = fn[w];
            v.wheels_[size_t(w)].sumDen = den[w];
        }
        for (int i = 0; i < 3; ++i) v.shiftTimes_[i] = T[i];
        v.shiftT_ = t;
    }
    // The state of notes/p24-work/drive_gen.py (identity orientation, CoM at the model origin).
    struct DriveState {
        Vec3 v{0.5f, 0, 12}, w{0, 0.3f, 0};
        float rpm = 4000, rims[4] = {13.0f, 12.0f, 12.5f, 11.8f}, loads[4] = {0.12f, 0.15f, 0.0f, 0.15f};
        float ratio[4][2] = {{2.0f, 1.0f}, {0.5f, 1.0f}, {0.5f, 1.0f}, {0.5f, 1.0f}};
        Vec3 fric0{0.1f, 0, -3.0f};
        Vec3 dirs[4] = {{-0.0998f, 0, 0.995f}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
        float rolling = 0.2f, soft = 0.5f;
        int gear = 1;
        float thr = 0.8f, accel = 0.8f, brake = 0;
        bool hb = false, ai = true;
    };
    static void apply(FuelVehicle& v, const DriveState& d) {
        BodyState& S = v.body_.S;
        S.q = {};
        S.x = {};
        S.v = d.v, S.w = d.w;
        S.auxVel[0] = d.rpm;
        for (int k = 0; k < 4; ++k) {
            S.auxVel[1 + k] = d.rims[k];
            WheelRec& r = v.wheels_[size_t(k)];
            r.load = d.loads[k];
            r.sumFn = d.ratio[k][0], r.sumDen = d.ratio[k][1];
            r.fric = k == 0 ? d.fric0 : Vec3{};
            r.hubW = S.x + r.hubL;
            v.wheelDirPost_[k] = d.dirs[k];
            v.C_[size_t(k)].rolling = d.rolling, v.C_[size_t(k)].soft = d.soft, v.C_[size_t(k)].hard = 1 - d.soft;
        }
        v.st_.dir0 = d.dirs[0], v.st_.dir2 = d.dirs[2];
        v.gear_ = d.gear;
        v.engThrottle_ = d.thr;
        v.boostMul_ = 1;
        v.player_ = !d.ai;
        v.in_ = Inputs{};
        v.in_.accel = d.accel, v.in_.brake = d.brake, v.in_.handbrake = d.hb;
        v.shiftT_ = 0;
        v.shiftTimes_[0] = v.shiftTimes_[1] = v.shiftTimes_[2] = 0.05f;
        v.autoBrake_ = 0;
    }
    static void driveline(FuelVehicle& v) { v.driveline(); }
    // notes/p24-work/body_gen.py: orientation yaw about y, velocity, spin, airtime, steer channel B, slide accel, gear.
    static void bodyForces(FuelVehicle& v, float yaw, Vec3 vel, Vec3 w, float airtime, float chB, float acc, int gear, const float loads[4],
                           const FuelVehicle::Guide& guide = {}) {
        v.guide_ = guide;
        BodyState& S = v.body_.S;
        S.q = axisAngle({0, 1, 0}, yaw);
        S.x = {};
        S.v = vel, S.w = w;
        for (int k = 0; k < 4; ++k) {
            WheelRec& r = v.wheels_[size_t(k)];
            r.load = loads[k];
            r.hubW = rotate(S.q, r.hubL);
        }
        v.speed_ = len(vel);
        v.airtime_ = airtime;
        v.st_.chB = chB, v.st_.accel = acc;
        v.refUp_ = {0, 1, 0};
        v.gear_ = gear;
        v.wrecked_ = 0, v.airborneLong_ = false;
        v.body_.boundR = 1.4f;
        v.body_.F = v.body_.T = {};
        v.bodyForces();
    }
    static const float* dOmega(const FuelVehicle& v) { return v.dOmega_; }
    // [nitro] the push on a vehicle with every wheel loaded, gear `g`, forward speed `vf` (body frame = world here).
    static Vec3 nitroPush(FuelVehicle& v, float amount, float top, float vf, int g, int loaded) {
        for (int w = 0; w < v.nWheels_; ++w) v.wheels_[size_t(w)].load = w < loaded ? 0.01f : 0.0f;
        v.vfwd_ = vf, v.gear_ = g;
        v.setNitro(amount, top);
        v.body_.F = v.body_.T = {};
        v.nitroPush();
        return v.body_.F;
    }
    static float mass(const FuelVehicle& v) { return v.body_.mass; }
    // [nitro] +0x13c0 after `steps` preStep slices of dt with the driven wheels sliding or gripping (all loaded).
    static float nitroTorque(FuelVehicle& v, float amount, bool sliding, int steps, float dt) {
        v.setNitro(amount, 40);
        for (int w = 0; w < v.nWheels_; ++w) v.setWheelSliding(w, sliding, 0.01f);
        for (int i = 0; i < steps; ++i) v.boostMul_ = 1, v.nitroTorque(dt);  // preStep resets +0x13c0 first
        return v.boostMul_;
    }
};
}  // namespace fuel::fv

using namespace fuel::fv;

namespace {

int failures = 0, passes = 0;

bool close(float a, float b, float rel = 2e-5f, float abs = 2e-6f) { return std::fabs(a - b) <= std::max(abs, rel * std::fabs(b)); }

void check(const char* id, const char* what, std::initializer_list<float> got, std::initializer_list<float> want, float rel = 2e-5f) {
    bool ok = got.size() == want.size();
    auto g = got.begin();
    for (auto w = want.begin(); ok && w != want.end(); ++w, ++g) ok = close(*g, *w, rel);
    std::printf("%-6s %-34s %s  got", id, what, ok ? "match   " : "MISMATCH");
    for (float x : got) std::printf(" %.8g", x);
    if (!ok) {
        std::printf("  want");
        for (float x : want) std::printf(" %.8g", x);
    }
    std::printf("\n");
    (ok ? passes : failures)++;
}

// T1-T4: penalty contact 0x721d50, generic friction path.
void penaltyVectors() {
    struct Case {
        const char* id;
        float depth;
        Vec3 n, point, v, w;
        float set1[8];
        Vec3 force, torque;
        float friction;
    };
    const float S1[8] = {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f};
    const Case cases[] = {
        {"T1", 0.05f, {0, 1, 0}, {0, 0, 0}, {2, -0.5f, 0}, {0, 0, 0}, {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f}, {-3263.4404f, 1317.4999f, 0},
         {0, 0, -979.03217f}, -10.878135f},
        {"T2", 0.1f, {0, 1, 0}, {0, 0, 0}, {1, 0.4f, 0}, {0, 0, 0}, {0.9f, 0.5f, 0.6f, 3, 2, 0.3f, 0, 0.2f}, {-1478.1107f, 588.0f, 0},
         {0, 0, -443.43323f}, -4.927036f},
        {"T3", 0.02f, {0.5f, 0.8660254f, 0}, {0, 0, 0}, {0.1f, -0.2f, 0}, {0, 0, 0}, {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f},
         {-835.53076f, 777.0743f, 0}, {0, 0, -250.65924f}, -2.7851026f},
        {"T4", 0.4f, {0, 1, 0}, {0.5f, 0, 0}, {0, -1, 0}, {0, 0, 1}, {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f}, {-4108.6167f, 2850.0f, 0},
         {0, 0, 192.41495f}, -13.695389f},
    };
    for (const Case& cs : cases) {
        RigidBody A;
        A.mass = 300;
        A.invMass = 1.0f / 300;
        A.S.x = {0, 0.3f, 0};
        A.S.v = cs.v;
        A.S.w = cs.w;
        Contact c;
        c.p = cs.point;
        c.n = cs.n;
        c.depth = cs.depth;
        c.a = &A;
        ContactMat& m = c.m;
        m.c = 0.05f, m.bias = 0.5f, m.maxDepth = 0.32f, m.preload = 0.1f, m.dampC = 0.3f, m.dampR = 0.2f, m.quadC = 0.4f, m.quadR = 0.4f;
        m.maxN = 20, m.loadFloor = 0, m.loadBlend = 1, m.extraDepth = 0, m.loadExp = 1;
        for (int i = 0; i < 8; ++i) m.set1[i] = cs.set1[i], m.set2[i] = S1[i];
        penaltyForce(c);
        check(cs.id, "0x721d50 force", {A.F.x, A.F.y, A.F.z}, {cs.force.x, cs.force.y, cs.force.z});
        check(cs.id, "0x721d50 torque", {A.T.x, A.T.y, A.T.z}, {cs.torque.x, cs.torque.y, cs.torque.z});
        // friction per kg = (F.x)/m (the friction is along x in all four)
        check(cs.id, "0x721d50 friction per kg", {A.F.x / 300}, {cs.friction});
    }
}

// TY1-TY3: tyre solver 0x720cc0.
void tyreVectors() {
    struct Case {
        const char* id;
        Vec3 u;
        float Nf;
        float s1[8], s2[8];
        bool slide;
        Vec3 rim50;
        float ratio;
        Vec3 fr;
        float r198, r19c, r1a0;
    };
    const Case cases[] = {
        {"TY1", {0.5f, 0, 3}, 15, {1.7f, 0.68f, 0.9f, 2, 0, 0, 0, 0.02f}, {1.5f, 0.6f, 0.7f, 2, 0, 0, 0, 0.01f}, false, {0, 0, -0.2f},
         0.031746031f, {-1, 0, 0.4f}, 0.4347826f, 0.06666667f, 2.1f},
        {"TY2", {0.3f, 0, 4}, 1, {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f}, {0.8f, 0.3f, 0.5f, 2, 0, 0, 0, 0.05f}, true, {0, 0, 0},
         166.666656f, {-0.6f, 0, -5.1609416f}, 0.4347826f, 200, 1.2f},
        {"TY3", {2, 0, 0}, 10, {1.0f, 0.2f, 0.7f, 50, 0, 0, 0, 0.1f}, {1.0f, 0.5f, 0.6f, 8, 2, 0.3f, 0.05f, 0.2f}, false, {0, 0, 0},
         1.08209348f, {-18.928427f, 0, 0}, 0.4347826f, 1.8828427f, 1.74f},
    };
    for (const Case& cs : cases) {
        WheelRec rec;
        rec.mat.maxDepth = 0.23f;
        rec.slidingDriven = cs.slide;
        rec.rimAccW = cs.rim50;
        rec.dirW = {0, 0, 1};
        Vec3 fr;
        const float ratio = tyreFriction({0, 0, 1}, cs.Nf, cs.s1, cs.s2, false, &rec, 0.1f, cs.u, fr);
        check(cs.id, "0x720cc0 ratio", {ratio}, {cs.ratio});
        check(cs.id, "0x720cc0 friction", {fr.x, fr.y, fr.z}, {cs.fr.x, cs.fr.y, cs.fr.z});
        check(cs.id, "0x720cc0 rec 198/19c/1a0", {rec.compRatio, rec.sumFn, rec.sumDen}, {cs.r198, cs.r19c, cs.r1a0});
    }
}

void integratorVectors() {
    {
        BodyState S;
        S.v = {1, -2, 0.5f}, S.w = {0.1f, 0.2f, 0.3f}, S.a = {3, -17, 4}, S.alpha = {1, 2, -3};
        integrateVelocity(S, 0.01f);
        check("INT1", "0x7150a0 v", {S.v.x, S.v.y, S.v.z}, {1.03f, -2.17f, 0.54f});
        check("INT1", "0x7150a0 w", {S.w.x, S.w.y, S.w.z}, {0.11f, 0.22f, 0.27f});
    }
    {
        BodyState S;
        S.q = {0, std::sin(0.25f), 0, std::cos(0.25f)};
        S.x = {1, 2, 3};
        S.v = {2, 0, 1};
        S.w = {1, 0, 0};
        integratePosition(S, 0.1f, 0.0025f, 0.0025f);
        check("INT2", "0x714d20 q", {S.q.x, S.q.y, S.q.z, S.q.w}, {0.048425436f, 0.24709478f, 0.012365044f, 0.96770155f});
        check("INT2", "0x714d20 x", {S.x.x, S.x.y, S.x.z}, {1.2f, 2, 3.1f});
        check("INT2", "0x714d20 Kahan residual", {S.xComp.x, S.xComp.y, S.xComp.z}, {-4.47e-8f, 0, 9.69e-8f}, 0.01f);
    }
    {
        BodyState S;
        S.x = {1, 2, 3};
        S.v = {0.04f, 0, 0};
        integratePosition(S, 0.1f, 0.0025f, 0.0025f);
        check("INT3", "0x714d20 deadband", {S.x.x, S.x.y, S.x.z}, {1, 2, 3});
    }
}

// A vehicle with synthetic parameters (no game data) and four wheel spheres.
FuelVehicle synthetic(std::vector<std::pair<int, float>> params, int wheels = 4) {
    std::vector<float> P(680, 0.0f);
    P[2] = 300;
    P[3] = 300;
    P[45] = 4;
    for (int i : {4, 5, 6, 7, 8, 9}) P[size_t(i)] = 1;
    P[49] = P[50] = 0.3f;
    P[53] = P[54] = 3;
    P[71] = P[72] = 0.3f;
    P[75] = P[76] = 0.05f;
    P[324] = 8000, P[325] = 2000, P[323] = 8, P[349] = 4, P[350] = 4;
    for (auto [i, v] : params) P[size_t(i)] = v;
    ProxyShapes proxy;
    const uint32_t names[4] = {0xD8B7A397, 0xFC33776C, 0xA828D8ED, 0x8CAC0C16};
    const Vec3 pos[4] = {{0.4f, 0.3f, 0.8f}, {0.4f, 0.3f, -0.8f}, {-0.4f, 0.3f, 0.8f}, {-0.4f, 0.3f, -0.8f}};
    for (int w = 0; w < wheels; ++w) proxy.spheres.push_back({pos[w], 0.3f, names[w]});
    proxy.boxes.push_back({{0, 0.6f, 0}, {0.5f, 0.3f, 1.0f}});
    return FuelVehicle(P, proxy, nullptr, true);
}

void torqueVectors() {
    std::vector<std::pair<int, float>> p = {{324, 8000}, {327, 1000}};
    for (int i = 0; i <= 10; ++i) p.push_back({338 + i, 0.1f * float(i)});
    FuelVehicle v = synthetic(p);
    check("TQ", "0x524c70 torque at 4400 / 4440", {v.engineTorque(4400), v.engineTorque(4440)}, {550.000042f, 555.000059f});
}

void steerAccelVectors() {  // 0x521f80 (steer_ref.py)
    std::vector<std::pair<int, float>> p;
    for (int j = 0; j < 8; ++j) p.push_back({204 + j, float(j + 1)});
    p.push_back({212, 9});
    for (int j = 0; j < 8; ++j) p.push_back({213 + j, float(11 + j)});
    p.insert(p.end(), {{221, 19}, {222, 0.1f}, {223, 0.05f}, {224, 20}, {225, 0.2f}, {226, 0.04f}, {227, 10}});
    for (int j = 0; j < 8; ++j) p.push_back({228 + j, float(21 + j)});
    p.push_back({236, 29});
    for (int j = 0; j < 8; ++j) p.push_back({237 + j, float(31 + j)});
    p.push_back({245, 39});
    for (int k = 0; k < 18; ++k) p.push_back({246 + k, k == 0 ? 1.25f : 1.0f + 0.01f * float(k)});
    p.insert(p.end(), {{264, 0}, {265, 1.5f}, {270, 3}, {271, 1.2f}, {272, 1.5f}, {273, 1.3f}, {274, 2.0f}, {312, 1}, {313, 0.5f},
                       {322, 0}, {397, 1}});
    struct Case {
        const char* id;
        float v, steer, wy, drift, f, thr, brk;
        bool hb;
        std::initializer_list<int> sliding;
        float F0, fb, chA, chC;
        float surf[3];
        float drift1, F1, out;
    };
    const Case cases[] = {
        {"V1", 10, 0.5f, 0.2f, 0, 0.25f, 0.8f, 0, false, {0}, 0, 0, 0.3f, 0.2f, {1, 0, 0}, 0.18517375f, 3.704951f, 1.4819804f},
        {"V2", 25, 0.5f, -0.3f, 0.4f, 0.6f, 0, 0.6f, true, {0, 1}, 5, -0.5f, -0.7f, -0.1f, {0.5f, 0, 0.5f}, 0.28185427f, 8.495129f,
         -8.919885f},
        {"V3", 3, 0.5f, 0.2f, 0, 0.25f, 0.8f, 0, false, {}, 2, 0, 0.3f, 0.2f, {1, 0, 0}, 0.18517375f, 1.4698858f, 0.58795434f},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        Inputs in;
        in.steer = cs.steer, in.accel = cs.thr, in.brake = cs.brk, in.handbrake = cs.hb;
        FuelVehicleTest::set(v, cs.v, in, {0, cs.wy, 0});
        float frac[kGroundClasses] = {cs.surf[0], cs.surf[1], cs.surf[2]};
        FuelVehicleTest::soft(v, cs.f, frac);
        FuelVehicleTest::wheels(v, cs.sliding);
        v.setGear(1);
        v.setAutoBrake(0);
        v.setSlopeGrip(1);
        auto& st = v.steer();
        st.drift = cs.drift, st.accelMag = cs.F0, st.feedback = cs.fb, st.chA = cs.chA, st.chC = cs.chC;
        v.steerAccel(1.0f / 60);
        check(cs.id, "0x521f80 drift/0x13c4/0x13c8", {st.drift, st.accelMag, st.accel}, {cs.drift1, cs.F1, cs.out}, 5e-5f);
    }
}

void steerAngleVectors() {  // 0x524710 (steerangle_ref.py)
    const std::vector<std::pair<int, float>> p = {{295, 0.5f}, {296, 2.0f}, {297, 40}, {298, 10}, {299, 12}, {300, 14}, {301, 16},
                                                  {302, 0.6f}, {303, 0.7f}, {304, 0.8f}, {305, 0.9f}, {314, 5}, {397, 1}};
    struct Case {
        const char* id;
        float v, air, thr, brk;
        bool hb;
        float chB, chD, fb, base0;
        std::initializer_list<int> sat;
        float base, tanF;
        Vec3 dirB, dirF;
        bool checkDirs;
    };
    const Case cases[] = {
        {"A1", 15, 0, 0.5f, 0, false, 0.4f, 0.3f, 0.1f, 0.05f, {}, 0.057357695f, 0.05449874f, {-0.051741257f, 0, 0.9986605f},
         {-0.054417986f, 0, 0.9985182f}, true},
        {"A2", 8, 0, 0.5f, 0, false, -0.6f, 0.3f, -0.6f, 0.05f, {0, 2}, 0.08088157f, -0.023013609f, {}, {0.023007516f, 0, 0.9997353f},
         false},
        {"A3", 15, 1, 0.5f, 0.7f, true, 0.4f, 0.3f, 0.1f, 0.05f, {}, 0.08054758f, 0.04027379f, {}, {}, false},
        {"A4", 30, 0, 0, 0.3f, false, 0.4f, -0.5f, 0.1f, 0.05f, {}, 0.050681897f, 0.010126432f, {}, {}, false},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        Inputs in;
        in.accel = cs.thr, in.brake = cs.brk, in.handbrake = cs.hb;
        FuelVehicleTest::set(v, cs.v, in, {});
        FuelVehicleTest::wheels(v, cs.sat);
        v.setAirtime(cs.air);
        v.setGear(1);
        auto& st = v.steer();
        st.chB = cs.chB, st.chD = cs.chD, st.feedback = cs.fb, st.base = cs.base0;
        v.steerAngle(1.0f / 60);
        check(cs.id, "0x524710 0x135c/0x1354", {st.base, st.tanF}, {cs.base, cs.tanF}, 5e-5f);
        if (cs.checkDirs) {
            check(cs.id, "0x524710 dirB", {st.dirB.x, st.dirB.y, st.dirB.z}, {cs.dirB.x, cs.dirB.y, cs.dirB.z}, 5e-5f);
            check(cs.id, "0x524710 dirF", {st.dirF.x, st.dirF.y, st.dirF.z}, {cs.dirF.x, cs.dirF.y, cs.dirF.z}, 5e-5f);
        } else if (cs.dirF.z != 0) {
            check(cs.id, "0x524710 dirF", {st.dirF.x, st.dirF.y, st.dirF.z}, {cs.dirF.x, cs.dirF.y, cs.dirF.z}, 5e-5f);
        }
    }
}

void channelVectors() {  // 0x52e830 (input_ref.py)
    const std::vector<std::pair<int, float>> p = {{306, 6}, {307, 7}, {308, 8}, {309, 9}, {310, 10}, {311, 11}, {315, 1.5f}, {316, 1.6f},
                                                  {317, 1.7f}, {318, 1.8f}, {319, 1.9f}, {320, 2.0f}, {321, 4}, {105, 0.5f}, {106, 0.3f}};
    struct Case {
        const char* id;
        float s, A, B, C, D, wy, t;
        float oA, oB, oC, oD, fb, ot;
    };
    const Case cases[] = {
        {"I1", 0.6f, 0.1f, 0.5f, -0.2f, 0.36f, 0.8f, 1, 0.18346104f, 0.43508586f, -0.18145311f, 0.36f, 0, 0},
        {"I2", 0.02f, -0.3f, 0, 0.9f, -0.0004f, 0.8f, 1, -0.19799209f, 2.2256274e-4f, 0.8833078f, 4.5125485e-5f, 0.24f, 1.0166667f},
        {"I3", -1, 0.2f, -0.5f, -1, -2, -2, 0, 0.09799208f, -0.5556407f, -1.0f, -1.9842352f, 0, 0},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        Inputs in;
        in.steer = cs.s;
        FuelVehicleTest::set(v, 0, in, {0, cs.wy, 0});
        v.setRefUp({0, 1, 0});
        auto& st = v.steer();
        st.chA = cs.A, st.chB = cs.B, st.chC = cs.C, st.chD = cs.D, st.sinceSteer = cs.t;
        v.steerChannels(1.0f / 60);
        check(cs.id, "0x52e830 channels A-D", {st.chA, st.chB, st.chC, st.chD}, {cs.oA, cs.oB, cs.oC, cs.oD}, 5e-5f);
        check(cs.id, "0x52e830 feedback/0x13e0", {st.feedback, st.sinceSteer}, {cs.fb, cs.ot}, 5e-5f);
    }
}

void airVector() {  // 0x52bb60 AIR1
    FuelVehicle v = synthetic({{32, 180}, {33, 0.1f}, {36, 6}, {41, 0.2f}, {42, 0.5f}, {43, 0.6f}, {44, 3}});
    Inputs in;
    in.steer = 0.5f;
    in.stickZ = -0.4f;
    FuelVehicleTest::set(v, 0, in, {0.1f, 0.2f, 0.3f});
    for (int w = 0; w < 4; ++w) v.setWheelSliding(w, false, 0);
    v.setRefUp({0.6f, 0.8f, 0});
    v.setAirTimer(0.5f);
    v.assists(1.0f / 60);
    const Vec3 w = v.body().S.w;
    check("AIR1", "0x52bb60 w / 0x1350", {w.x, w.y, w.z, v.airTimer()}, {0.054f, 0.1425f, 0.231f, 0.51666665f}, 5e-5f);
}

// DL1-DL3: driveline 0x5211e0 (engine 0x520560, clutch 0x520b00, brakes 0x520fd0, tyre reaction / rolling
// 0x520d40), emulated in P24 (notes/p24-work/drive_gen.py, dl.cmd).
void drivelineVectors() {
    std::vector<std::pair<int, float>> p = {{46, 1}, {47, 1}, {48, 0}, {49, 0.3f}, {50, 0.3f}, {53, 3}, {54, 4}, {68, 16}, {91, 0.01f},
                                            {92, 0}, {129, 0.5f}, {153, 20}, {323, 8}, {324, 8000}, {325, 2000}, {327, 75000}, {328, 0.8f},
                                            {329, 0.9f}, {350, 4.05f}, {351, -7}, {352, 4.845f}, {359, 1}, {360, 1}, {377, 0}, {378, 1},
                                            {379, 70000}, {380, 385}, {381, 2000}, {395, 200}, {396, 400}, {397, 1500}, {398, 0}};
    const float curve[11] = {0, 0.8f, 0.85f, 0.9f, 0.95f, 1, 1, 1, 1, 0.95f, 0};
    for (int i = 0; i < 11; ++i) p.push_back({338 + i, curve[i]});
    struct Case {
        const char* id;
        FuelVehicleTest::DriveState st;
        float acc[5], vel[5], dw[4];
    };
    FuelVehicleTest::DriveState s1, s2, s3;
    s2.brake = 0.7f, s2.hb = true, s2.ai = false, s2.accel = 0, s2.thr = 0;
    s3.gear = 0;
    for (auto& r : s3.ratio) r[0] = 0.5f, r[1] = 1;
    const Case cases[] = {
        {"DL1", s1, {22101.643f, -664.9882f, 0.0f, -721.2483f, 0.0f}, {4000, 13.0f, 11.88f, 12.5f, 12.12f},
         {-808.74725f, -522.607f, -721.2483f, -504.4018f}},
        {"DL2", s2, {-3600.0f, 11.248755f, 0.0f, -46.666664f, 0.0f}, {4000, 13.0f, 11.88f, 12.5f, 12.12f},
         {-75.00667f, -471.04f, -46.666664f, -470.5824f}},
        {"DL3", s3, {8655.0f, 0.0f, 0.0f, -66.666664f, 0.0f}, {4000, 11.746748f, 11.88f, 12.5f, 12.12f},
         {-95.00667f, -126.04f, -66.666664f, -125.582405f}},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        FuelVehicleTest::apply(v, cs.st);
        FuelVehicleTest::driveline(v);
        const BodyState& S = v.body().S;
        const float* dw = FuelVehicleTest::dOmega(v);
        check(cs.id, "0x5211e0 aux accelerations", {S.auxAcc[0], S.auxAcc[1], S.auxAcc[2], S.auxAcc[3], S.auxAcc[4]},
              {cs.acc[0], cs.acc[1], cs.acc[2], cs.acc[3], cs.acc[4]}, 1e-4f);
        check(cs.id, "0x5211e0 aux velocities", {S.auxVel[0], S.auxVel[1], S.auxVel[2], S.auxVel[3], S.auxVel[4]},
              {cs.vel[0], cs.vel[1], cs.vel[2], cs.vel[3], cs.vel[4]}, 1e-4f);
        check(cs.id, "0x5211e0 d-omega (+0x12a4)", {dw[0], dw[1], dw[2], dw[3]}, {cs.dw[0], cs.dw[1], cs.dw[2], cs.dw[3]}, 1e-4f);
    }
}

// BF1-BF3: body forces 0x522830 (gravity 0x530cb0, aero drag 0x530d20, downforce, anti-roll 0x52c900, slide-steer
// couple), emulated in P24 (notes/p24-work/body_gen.py, bf.cmd). No water manager (water block skipped).
void bodyForceVectors() {
    const std::vector<std::pair<int, float>> p = {{87, 50}, {88, 40}, {93, 0.0001f}, {94, 0.002f}, {95, 1.0f}, {96, 1.2f}, {97, 1.0f},
                                                  {98, 0.75f}, {99, 20}, {100, 0.05f}, {101, 2.0f}, {102, 1.5f}, {103, 2.5f}, {104, 2.0f},
                                                  {266, 1.0f}, {267, 1.5f}, {268, 0.8f}, {269, 1.2f}, {287, 0.01f}, {288, -17}, {289, -15},
                                                  {291, 0.2f}, {292, 0.25f}, {577, 2}, {579, 1.5f}, {580, 0.8f}, {581, 1.6f}, {46, 1}, {47, 1}};
    struct Case {
        const char* id;
        float yaw;
        Vec3 v, w;
        float air, chB, acc;
        int gear;
        float loads[4];
        Vec3 F, T;
    };
    const Case cases[] = {
        {"BF1", 0.0f, {0.5f, 0.2f, 12}, {0.1f, 0.3f, -0.05f}, 0, 0.4f, 1.2f, 1, {0.12f, 0.15f, 0, 0.15f}, {262.53415f, -5642.2603f, -113.016525f},
         {-166.58199f, -1133.3313f, 83.29099f}},
        {"BF2", 0.5f, {3, 0.2f, 11}, {0.1f, -0.4f, 0.2f}, 0, -0.7f, -0.8f, -1, {0.12f, 0.15f, 0.1f, 0.15f}, {155.62605f, -5735.6084f, -90.66908f},
         {-50.06648f, -562.5192f, -47.478012f}},
        {"BF3", 0.3f, {2, -3, 20}, {0.5f, 0.2f, 0.1f}, 0.5f, 0, 0, 2, {0, 0, 0, 0}, {-32.017937f, -5919.7007f, -320.17932f},
         {-312.5263f, -93.75789f, -62.505253f}},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        FuelVehicleTest::bodyForces(v, cs.yaw, cs.v, cs.w, cs.air, cs.chB, cs.acc, cs.gear, cs.loads);
        const RigidBody& b = v.body();
        check(cs.id, "0x522830 force", {b.F.x, b.F.y, b.F.z}, {cs.F.x, cs.F.y, cs.F.z}, 2e-4f);
        check(cs.id, "0x522830 torque", {b.T.x, b.T.y, b.T.z}, {cs.T.x, cs.T.y, cs.T.z}, 2e-4f);
    }
}

// GF1-GF3: the guided target forces of the AI (0x522830 tail, P29 decode of U5), the body forces of the BF
// setup plus the AI branch (+0x1421, owner +0x1a49, targets +0xefc / +0xeec / +0xf0c), emulated in P29
// (notes/p29-work/gf_gen.py).
void guidedForceVectors() {
    const std::vector<std::pair<int, float>> p = {{87, 50}, {88, 40}, {93, 0.0001f}, {94, 0.002f}, {95, 1.0f}, {96, 1.2f}, {97, 1.0f},
                                                  {98, 0.75f}, {99, 20}, {100, 0.05f}, {101, 2.0f}, {102, 1.5f}, {103, 2.5f}, {104, 2.0f},
                                                  {266, 1.0f}, {267, 1.5f}, {268, 0.8f}, {269, 1.2f}, {287, 0.01f}, {288, -17}, {289, -15},
                                                  {291, 0.2f}, {292, 0.25f}, {577, 2}, {579, 1.5f}, {580, 0.8f}, {581, 1.6f}, {46, 1}, {47, 1}};
    struct Case {
        const char* id;
        float yaw;
        Vec3 v, w;
        float loads[4];
        Vec3 tp, tv;
        float tyaw;
        Vec3 F, T;
    };
    const Case cases[] = {
        {"GF1", 0.0f, {0.5f, 0.2f, 12}, {0.1f, 0.3f, -0.05f}, {0.12f, 0.15f, 0.1f, 0.15f}, {3, 0.5f, 2}, {1, 0, 15}, 0.6f,
         {9908.033f, -5790.113f, 10599.861f}, {-42.53254f, 2212.5828f, 21.26627f}},
        {"GF2", 0.5f, {3, 0.2f, 11}, {0.1f, -0.4f, 0.2f}, {0.12f, 0.15f, 0.1f, 0.15f}, {-4, 0, -1}, {2, 0, 8}, -0.4f,
         {-12989.29f, -5735.6084f, -7376.905f}, {-41.658546f, -2576.1238f, -30.662144f}},
        {"GF3", 0.0f, {0, 0, 20}, {0, 0, 0}, {0.12f, 0.15f, 0.1f, 0.15f}, {0.3f, 0, 0}, {0, 0, 20}, 0.1f,
         {0.0f, -6270.0f, -15.68f}, {-23.999996f, 0.0f, 11.999998f}},
    };
    for (const Case& cs : cases) {
        FuelVehicle v = synthetic(p);
        FuelVehicle::Guide g;
        g.pos = g.rot = true;
        g.p = cs.tp, g.v = cs.tv;
        g.q = axisAngle({0, 1, 0}, cs.tyaw);
        FuelVehicleTest::bodyForces(v, cs.yaw, cs.v, cs.w, 0, 0, 0, 1, cs.loads, g);
        const RigidBody& b = v.body();
        check(cs.id, "0x522830 AI guide force", {b.F.x, b.F.y, b.F.z}, {cs.F.x, cs.F.y, cs.F.z}, 2e-4f);
        check(cs.id, "0x522830 AI guide torque", {b.T.x, b.T.y, b.T.z}, {cs.T.x, cs.T.y, cs.T.z}, 2e-4f);
    }
}

void derivedVectors() {
    // Gravity (0x530cb0): F = (0, m Pg, 0). Anti-roll example (QUAD): d = 0.02 -> F = m 0.1 min(50 0.02, 9.81) = 0.1 m.
    FuelVehicle v = synthetic({{287, 0.01f}, {288, -17}, {289, -15}, {87, 50}, {88, 50}});
    check("GRAV", "gravity force / mass, P288", {-17.0f}, {-17.0f});
    const float lift = 300 * 0.1f * std::min(50 * 0.02f, 9.81f);
    check("AROLL", "anti-roll lift (QUAD d=0.02)", {lift / 300}, {0.1f});
    (void)v;
}

// PK01: the pre-start hold. CP: EmuRun of 0x62ce40 (ESI = race object: state +0x48, elapsed +0x244, online +0x3b8,
// start +0x70 / +0x7c; 0x4232d0 / 0x559ec0 stubbed, 0x539b30 stubbed with "slot found" and its slot record, 0x62d250
// with 0) against fuel::RaceRules::checkpointRespawnPose: AL, then the out position and quaternion (zero when false).
// RO: EmuRun of 0x529480 (the world flag chain game +0x31fc -> +0x908 -> +0x5c and the vehicle's world handles set
// up in emulator memory, 0x40fe70 stubbed with the terrain height and 0x640690 with the normal; 0x410510, 0x6443a0
// and 0x410260 run for real) against fv::respawnOrient (command files: scratchpad cp/gen.py, cp/genro.py).
void startHoldVectors() {
    using Pose = fuel::RaceRules::Pose;
    const Pose start{{10.5f, 20.25f, -30.75f}, {0.1f, 0.2f, 0.3f, 0.927362f}};
    const Pose slot{{-5.5f, 7.25f, 9.0f}, {0.5f, -0.5f, 0.5f, 0.5f}};
    struct C {
        const char* id;
        int state;
        float elapsed;
        int online, found;  // online: the 0x62d250 stub answers 0
        int want;           // AL
        const Pose* pose;   // the out pose FUEL wrote (nullptr: untouched zeros)
    };
    const C cs[] = {
        {"CP1", 0, -15, 0, 0, 0, nullptr},     {"CP2", 1, -15, 0, 0, 0, nullptr},     {"CP3", 2, -15, 0, 0, 1, &start},
        {"CP4", 3, -15, 0, 0, 1, &start},      {"CP5", 4, -15, 0, 0, 1, &start},      {"CP6", 5, -15, 0, 0, 0, nullptr},
        {"CP7", 6, -15, 0, 0, 0, nullptr},     {"CP8", 7, -15, 0, 0, 0, nullptr},     {"CP9", 4, 5, 0, 1, 1, &start},
        {"CP10", 3, -15, 0, 1, 1, &slot},      {"CP11", 4, 0, 0, 1, 1, &slot},        {"CP12", 2, -10, 0, 1, 1, &slot},
        {"CP13", 4, -15, 1, 0, 0, nullptr},    {"CP14", 4, -15, 0, 0, 1, &start},     {"CP15", 6, 5, 0, 1, 0, nullptr},
    };
    for (const C& c : cs) {
        Pose out{{0, 0, 0}, {0, 0, 0, 0}};
        const bool no = false;
        const bool got = fuel::RaceRules::checkpointRespawnPose(c.state, c.elapsed, start, c.found ? &slot : nullptr, out, c.online ? &no : nullptr);
        const Pose want = c.pose ? *c.pose : Pose{{0, 0, 0}, {0, 0, 0, 0}};
        check(c.id, "0x62ce40 hold pose", {got ? 1.0f : 0.0f, out.pos[0], out.pos[1], out.pos[2], out.quat[0], out.quat[1], out.quat[2], out.quat[3]},
              {float(c.want), want.pos[0], want.pos[1], want.pos[2], want.quat[0], want.quat[1], want.quat[2], want.quat[3]});
    }
    struct R {
        const char* id;
        float rv[3], n[3], y, terrain;
        bool flag;
        float want[4];
    };
    const R rs[] = {
        {"RO1", {0.150381f, 0.828158f, -0.046308f}, {0.1f, 0.98f, -0.05f}, 863.7f, 863.7f, false, {-0.0027272536f, 0.40307483f, -0.056700904f, 0.91340476f}},
        {"RO2", {0, -2.5f, 0}, {0, 1, 0}, 100, 99.5f, false, {0, -0.9489846f, 0, 0.31532234f}},
        {"RO3", {0.3f, 3.0f, 0.2f}, {-0.2f, 0.95f, 0.15f}, 50, 49.2f, false, {-0.09729077f, 0.9895326f, 0.083649494f, 0.06604853f}},
        {"RO4", {0, 1, 0}, {0.3f, 0.9f, 0.1f}, 10, 9, false, {0, 0.47942555f, 0, 0.87758255f}},
        {"RO5", {0, 1, 0}, {0.3f, 0.9f, 0.1f}, 10, 11.5f, false, {0, 0.47942555f, 0, 0.87758255f}},
        {"RO6", {0, 1, 0}, {0.3f, 0.9f, 0.1f}, 10, 10, true, {0, 0.47942555f, 0, 0.87758255f}},
        {"RO7", {1.4f, 0.2f, 0}, {0.05f, 0.99f, 0}, 5, 5.3f, false, {0.009010903f, 0.35705912f, -0.023564303f, 0.93374103f}},
    };
    for (const R& c : rs) {
        fuel::RaceRules rr;
        rr.race.startRot = {c.rv[0], c.rv[1], c.rv[2]};
        float q[4];
        const auto sq = rr.startPose().quat;  // 0x644980, as the vectors' generator
        for (int k = 0; k < 4; ++k) q[k] = sq[size_t(k)];
        const float l = std::sqrt(c.n[0] * c.n[0] + c.n[1] * c.n[1] + c.n[2] * c.n[2]);
        const float n[3] = {c.n[0] / l, c.n[1] / l, c.n[2] / l};
        respawnOrient(q, c.y, c.terrain, n, c.flag);
        check(c.id, "0x529480 respawn orientation", {q[0], q[1], q[2], q[3]}, {c.want[0], c.want[1], c.want[2], c.want[3]}, 1e-5f);
    }
}

// PK07: the race helicopter (core/race_helicopter): EmuRun of 0x620670 (steering / speed; 0x421f60 stubbed with the
// player's position), 0x620530 (cruise height; the handle chain set up, 0x6407e0 stubbed with two heights), 0x620370
// (wobble) and 0x620980 (attitude) on a synthetic 0x1f8-byte entry (fuelphysvec_heli.inc).
// heli-fix: the helicopter's race record (RaceRules::heliRecord = 0x620460 with port_RaceRules_update 0x5c1d40: the 40 m
// waypoint step sets the record's pass byte +0, which passes the current gate) and FUEL's wobble feedback after time A
// (0x620370 / 0x620670 frames: the raw steering vector grows the display offset), emulated on FUEL.exe
// (fuelphysvec_helirec.inc); HD*c checks the port's safety clamp in RaceRules::heliStep (FUEL has none).
#include "fuelphysvec_helirec.inc"
void heliRecordVectors() {
    using fuel::RaceHelicopter;
    using Vec3 = std::array<float, 3>;
    auto V3 = [](const float (&a)[3]) { return RaceHelicopter::V3{a[0], a[1], a[2]}; };
    // The course of hp.py: three checkpoints and the finish (HP3: the first gate line 200 m long).
    static const HeliGate kBase[] = {{{100, 50, 0}, 100, -9, 100, 9}, {{200, 60, 50}, 200, 41, 200, 59}, {{300, 70, 0}, 300, -9, 300, 9}, {{400, 80, 0}, 400, -10, 400, 10}};
    auto course = [&](fuel::RaceRules& rr, bool wide) {
        rr.gates.clear();
        for (const HeliGate& g : kBase) {
            fuel::RaceRules::Gate o;
            o.centre = {g.centre[0], g.centre[1], g.centre[2]};
            o.ax = g.ax, o.az = g.az, o.bx = g.bx, o.bz = g.bz;
            rr.gates.push_back(o);
        }
        if (wide) rr.gates[0].az = -100, rr.gates[0].bz = 100;
    };
    for (const HeliRec& c : kHeliRec) {
        fuel::RaceRules rr;
        course(rr, c.wide);
        rr.heliNext = size_t(c.passed);
        rr.heliPrev = {c.prev[0], c.prev[1], c.prev[2]};
        rr.heliHavePrev = true;
        rr.hel.target = V3(c.target), rr.hel.cruise = c.cruise, rr.hel.waypoint = c.waypoint, rr.hel.disp = V3(c.disp);
        rr.hel.forcePass = c.byte0 != 0;
        rr.heliRecord();
        const bool landed = c.outState == 4;
        check(c.id, "0x620460 waypoint / pass / landed", {float(rr.hel.waypoint), float(rr.hel.forcePass), float(rr.hel.forcePass1), float(rr.heliNext), float(rr.heliLanded)},
              {float(c.outWaypoint), float(c.outBytes & 0xff), float(c.outBytes >> 8), float(c.outPassed), float(landed)});
        // A finished record keeps its previous position and its target wraps to gate 0 (0x5c1ca0 with passed % (n + 1));
        // the port stops flying at the landing, so those two are compared only while it is in the race.
        if (!landed)
            check(c.id, "0x620460 target / previous position", {rr.hel.target[0], rr.hel.target[1], rr.hel.target[2], rr.heliPrev[0], rr.heliPrev[1], rr.heliPrev[2]},
                  {c.outTarget[0], c.outTarget[1], c.outTarget[2], c.outPrev[0], c.outPrev[1], c.outPrev[2]});
    }
    const float dt = 1.0f / 60;
    for (const HeliDiv& c : kHeliDiv) {
        RaceHelicopter h;
        h.target = V3(c.target), h.pos = V3(c.pos), h.dir = V3(c.dir), h.phase = c.phase, h.timeLeft = c.timeLeft;
        RaceHelicopter::V3 d30{};
        for (int f = 1; f <= c.frames; ++f) {
            h.wobble(dt);
            h.velocity(dt, {0, 0, 0});
            if (f == 30) d30 = h.disp;
        }
        // 2e-3: the feedback amplifies the last-bit differences of the float rounding as much as the offset itself
        // (frame 30 matches to 1e-6).
        check(c.id, "0x620370/0x620670 after time A", {d30[0], d30[1], d30[2], h.disp[0], h.disp[1], h.disp[2], h.dir[0], h.dir[1], h.dir[2], h.phase},
              {c.disp30[0], c.disp30[1], c.disp30[2], c.dispEnd[0], c.dispEnd[1], c.dispEnd[2], c.dirEnd[0], c.dirEnd[1], c.dirEnd[2], c.phaseEnd}, 2e-3f);
        // The port's clamp: the same frames through RaceRules::heliStep keep the display within the wobble of a unit
        // direction (|cos t R1 d + cos 0.8t R2 d| <= 2; the velocity is 0 here).
        fuel::RaceRules rr;
        course(rr, false);
        rr.hel.target = V3(c.target), rr.hel.pos = V3(c.pos), rr.hel.dir = V3(c.dir), rr.hel.phase = c.phase, rr.hel.timeLeft = c.timeLeft;
        rr.hel.disp = rr.hel.pos;
        float worst = 0;
        for (int f = 0; f < 600; ++f) {
            rr.heliStep(dt, Vec3{0, 0, 0});
            const float ox = rr.hel.disp[0] - rr.hel.pos[0], oy = rr.hel.disp[1] - rr.hel.pos[1], oz = rr.hel.disp[2] - rr.hel.pos[2];
            worst = std::max(worst, std::sqrt(ox * ox + oy * oy + oz * oz));
        }
        const std::string id = std::string(c.id) + "c";
        std::printf("%-6s port clamp: display - position <= %.3f m over 600 frames after time A\n", id.c_str(), worst);
        check(id.c_str(), "port: heliStep keeps the unit wobble", {worst <= 2.0f ? 1.0f : 0.0f}, {1.0f});
    }
}

#include "fuelphysvec_heli.inc"
void heliVectors() {
    using fuel::RaceHelicopter;
    auto V3 = [](const float (&a)[3]) { return RaceHelicopter::V3{a[0], a[1], a[2]}; };
    for (const HeliVel& c : kHeliVel) {
        RaceHelicopter h;
        h.target = V3(c.target), h.disp = V3(c.disp), h.pos = V3(c.pos), h.timeLeft = c.timeLeft, h.vel = V3(c.vel), h.cruise = c.cruise;
        h.speed = c.speed, h.prevSpeed = 20.0f, h.missionStart = V3(c.mstart), h.missionEnd = V3(c.mend);
        h.velocity(c.dt, V3(c.player));
        check(c.id, "0x620670 steering / speed", {h.dir[0], h.dir[1], h.dir[2], h.prevSpeed, h.speed, h.vel[0], h.vel[1], h.vel[2]},
              {c.dir[0], c.dir[1], c.dir[2], c.prev, c.spd, c.outVel[0], c.outVel[1], c.outVel[2]});
    }
    for (const HeliCruise& c : kHeliCruise) {
        RaceHelicopter h;
        h.target = V3(c.target), h.pos = V3(c.pos), h.requiredSpeed = c.v0;
        int n = 0;
        const float g1 = c.g1, g2 = c.g2;
        h.ground = [&](float, float) { return n++ == 0 ? g1 : g2; };
        h.cruiseHeight();
        check(c.id, "0x620530 cruise height", {h.cruise}, {c.want});
    }
    for (const HeliWob& c : kHeliWob) {
        RaceHelicopter h;
        h.dir = V3(c.dir), h.phase = c.phase, h.pos = V3(c.pos);
        h.wobble(c.dt);
        check(c.id, "0x620370 wobble", {h.disp[0], h.disp[1], h.disp[2], h.phase}, {c.disp[0], c.disp[1], c.disp[2], c.outPhase});
    }
    for (const HeliAtt& c : kHeliAtt) {
        RaceHelicopter h;
        h.dir = V3(c.dir), h.vel = V3(c.vel), h.pitchRaw = c.pitch, h.phase = c.phase;
        h.quat = {c.quat[0], c.quat[1], c.quat[2], c.quat[3]};
        h.attitude(c.dt);
        // a start quaternion of w = 0 / -w are the same rotation: compare up to sign
        const float s = (h.quat[0] * c.outQuat[0] + h.quat[1] * c.outQuat[1] + h.quat[2] * c.outQuat[2] + h.quat[3] * c.outQuat[3]) < 0 ? -1.0f : 1.0f;
        check(c.id, "0x620980 attitude", {h.pitchRaw, h.rollRaw, s * h.quat[0], s * h.quat[1], s * h.quat[2], s * h.quat[3]},
              {c.outPitch, c.outRoll, c.outQuat[0], c.outQuat[1], c.outQuat[2], c.outQuat[3]}, 1e-4f);
    }
    heliRecordVectors();
}

// ---- rain-ai: an AI racer passes its gate when its path cursor runs past the gate's checkpoint block
// (IAPathBlockCheckpoint::vf04 0x624210 -> racer +0x35 / +0x38; 0x6182b0's fifth frame -> race record byte +0 when the
// record's target is within 5 m; 0x61b630 -> 0x5c1d40 passes on the byte). EmuRun of FUEL.exe (fuelphysvec_aigate.inc)
// against fuel::aiPassCheckpointBlocks / AiCheckpointFlag / RaceRules::aiCheckpointBlock + update.
#include "fuelphysvec_aigate.inc"
void checkV(const char* id, const char* what, const std::vector<float>& got, const std::vector<float>& want, float rel);
void aiGateVectors() {
    // CB: the port's cursor rule on a straight line whose point 3 is the block's centre (101, 0): FUEL's block (100 -> 102)
    // is passed when the point projects past its exit, however far aside (CB3: 30 m).
    fuel::RaceLine line;
    line.build({0, 0, 0}, {{40, 0, 0}, {70, 0, 0}, {101, 0, 0}, {300, 0, 0}});
    for (const AiGateBlock& c : kAiGateBlock) {
        fuel::AiCheckpointFlag flag;
        flag.index = c.last;
        const float s = line.project(c.pos[0], c.pos[1], 0.0f, 400.0f);
        const size_t cursor = fuel::aiPassCheckpointBlocks(line, 3, s, flag);
        check(c.id, "0x624210 block passed / flag / index", {float(cursor > 3), float(flag.up), float(flag.index)},
              {float(c.ret == 1), float(c.flag), float(c.index)});
    }
    // A race record heading for `target` (gate 0) and racer 1's path records (0 start, 1 = gate 0 ...).
    using Vec3 = fuel::RaceRules::Vec3;
    auto course = [](fuel::RaceRules& rr, const std::vector<std::array<float, 7>>& g) {  // centre x y z, ax az bx bz
        rr.gates.clear(), rr.route.clear();
        for (const auto& v : g) {
            fuel::RaceRules::Gate o;
            o.centre = {v[0], v[1], v[2]}, o.ax = v[3], o.az = v[4], o.bx = v[5], o.bz = v[6], o.half = std::hypot(v[5] - v[3], v[6] - v[4]) * 0.5f;
            rr.gates.push_back(o), rr.route.push_back(o.centre);
        }
        rr.gateRadius = 10.0f;  // record +0x14c of ag.py
        rr.racers.assign(2, fuel::RaceRules::Racer{});
        rr.state = fuel::RaceRules::Running, rr.clock = 12.5;
    };
    for (const AiGateFrames& c : kAiGateFrames) {
        fuel::RaceRules rr;
        course(rr, {{c.target[0], c.target[1], c.target[2], c.target[0], c.target[2] - 9, c.target[0], c.target[2] + 9}});
        const Vec3 recs[3] = {{0, 0, 0}, {100, 0, 0}, {c.rec2[0], 0, c.rec2[1]}};
        fuel::AiCheckpointFlag flag;
        flag.up = c.flag != 0, flag.index = c.index, flag.frames = c.count;
        std::vector<float> got, want;
        for (int f = 0; f < c.frames; ++f) {
            const int k = flag.frame();
            if (k >= 0 && k < 3) rr.aiCheckpointBlock(1, recs[k]);
            got.insert(got.end(), {float(rr.racers[1].forcePass), float(rr.racers[1].forcePass1), float(flag.up), float(flag.frames)});
            want.insert(want.end(), {float(c.bytes[f] & 0xff), float(c.bytes[f] >> 8), float(c.flagOut[f]), float(c.countOut[f])});
        }
        checkV(c.id, "0x6182b0 frames: bytes / flag / count", got, want, 2e-5f);
    }
    for (const AiGateChain& c : kAiGateChain) {
        fuel::RaceRules rr;
        course(rr, {{100, 0, 0, 100, -9, 100, 9}, {200, 0, 0, 200, -9, 200, 9}, {300, 0, 0, 300, -9, 300, 9}, {400, 0, 0, 400, -10, 400, 10}});
        const Vec3 recs[5] = {{0, 0, 0}, {100, 0, 0}, {200, 0, 0}, {300, 0, 0}, {400, 0, 0}};
        rr.racers[1].forcePass = c.byte0 != 0;
        rr.racers[1].prev = {100, 0, 60}, rr.racers[1].havePrev = true;
        rr.racers[0].prev = {-500, 0, -500}, rr.racers[0].havePrev = true;
        const std::vector<Vec3> pos = {{-500, 0, -500}, {100.5f, 0, 60}};  // 0x61b630 copies +0x1104 into the record
        fuel::AiCheckpointFlag flag;
        flag.up = c.flag != 0, flag.index = c.index;
        std::vector<float> got, want;
        for (int f = 0; f < c.frames; ++f) {
            const size_t before = rr.racers[1].next;
            rr.update(1.0 / 60, pos);
            const int k = flag.frame();
            if (k >= 0 && k < 5) rr.aiCheckpointBlock(1, recs[k]);
            got.insert(got.end(), {float(rr.racers[1].next > before), float(rr.racers[1].next), float(rr.racers[1].forcePass), float(rr.racers[1].forcePass1)});
            want.insert(want.end(), {float(c.ret[f]), float(c.passed[f]), float(c.bytes[f] & 0xff), float(c.bytes[f] >> 8)});
        }
        checkV(c.id, "0x61b630 + 0x6182b0 frames: gates passed", got, want, 2e-5f);
    }
}

// ---- PK27: the F14 fly-by (core/f14_flyby): EmuRun of 0x51a380 (spawn; 0x420330 / 0x421ab0 / 0x6460f0 / 0x445500 stubbed
// with the inputs), F14Move_G vf17 0x51a050 (count, spawn + fly, fly, smoothing, end of the pass, a tiny step) and the
// emitter doppler 0x4ea560 + 0x732e30 (Sound_Z rate 1e7: the frequency resolves the pitch to 1e-7) (fuelphysvec_f14.inc).
#include "fuelphysvec_f14.inc"
void checkV(const char* id, const char* what, const std::vector<float>& got, const std::vector<float>& want, float rel = 2e-5f) {
    bool ok = got.size() == want.size();
    for (size_t i = 0; ok && i < want.size(); ++i) ok = close(got[i], want[i], rel);
    std::printf("%-6s %-34s %s  got", id, what, ok ? "match   " : "MISMATCH");
    for (float x : got) std::printf(" %.8g", x);
    if (!ok) {
        std::printf("  want");
        for (float x : want) std::printf(" %.8g", x);
    }
    std::printf("\n");
    (ok ? passes : failures)++;
}
// a quaternion and its negative are one rotation (Quat_FromArc's half turn picks either axis sign)
std::vector<float> sameSign(const std::array<float, 4>& q, const float* want) {
    const float d = q[0] * want[0] + q[1] * want[1] + q[2] * want[2] + q[3] * want[3];
    const float s = d < 0 ? -1.0f : 1.0f;
    return {s * q[0], s * q[1], s * q[2], s * q[3]};
}
void f14Vectors() {
    using fuel::F14Flyby;
    for (const F14SpawnVec& c : kF14Spawn) {
        F14Flyby f;
        const float r = c.r, g = c.g;
        f.rand01 = [r] { return r; };
        f.ground = [g](float, float, float) { return g; };
        f.spawn(true, {c.p[0], c.p[1], c.p[2]}, {c.f[0], c.f[1], c.f[2]});
        checkV(c.id, "0x51a380 start / end / velocity", {f.pos[0], f.pos[1], f.pos[2], f.end[0], f.end[1], f.end[2], f.vel[0], f.vel[1], f.vel[2],
                                                          f.player[0], f.player[1], f.player[2]},
               std::vector<float>(c.out, c.out + 12), 2e-5f);
        checkV(c.id, "0x51a380 rotation (Quat_FromArc)", sameSign(f.quat, c.out + 12), std::vector<float>(c.out + 12, c.out + 16), 1e-4f);
    }
    for (const F14MoveVec& c : kF14Move) {
        F14Flyby f;
        f.timer = c.timer;
        f.pos = {c.pos[0], c.pos[1], c.pos[2]}, f.end = {c.end[0], c.end[1], c.end[2]}, f.vel = {c.vel[0], c.vel[1], c.vel[2]};
        f.nodePos = {c.npos[0], c.npos[1], c.npos[2]};
        f.quat = {0, 0, 0, 0};  // the emulated record starts zeroed
        f.shown = true;          // node +0x2c = 0x8000000 (not hidden)
        int n = 0;
        f.ground = [&c, &n](float, float, float) { const float v = c.g[std::min(n, c.ng - 1)]; ++n; return v; };
        const float sr = c.sr;
        f.rand01 = [sr] { return sr; };
        f.step(c.dt, true, true, {c.sp[0], c.sp[1], c.sp[2]}, {c.sf[0], c.sf[1], c.sf[2]});
        checkV(c.id, "0x51a050 timer / position", {f.timer, f.pos[0], f.pos[1], f.pos[2]}, std::vector<float>(c.out, c.out + 4), 2e-5f);
        if (c.spawn) checkV(c.id, "0x51a050 spawned end / velocity", {f.end[0], f.end[1], f.end[2], f.vel[0], f.vel[1], f.vel[2]},
                            std::vector<float>(c.out + 4, c.out + 10), 2e-5f);
        checkV(c.id, "0x51a050 rotation +0x54", sameSign(f.quat, c.out + 13), std::vector<float>(c.out + 13, c.out + 17), 1e-4f);
        checkV(c.id, "0x51a050 node hidden (0x40000)", {f.shown ? 0.0f : 1.0f}, {(c.nodeFlags & 0x40000) ? 1.0f : 0.0f});
    }
    for (const F14DopVec& c : kF14Dop) {
        F14Flyby f;
        f.soundOn = true;
        f.relPrev = {c.prev[0], c.prev[1], c.prev[2]};
        f.nodePos = {c.epos[0], c.epos[1], c.epos[2]};
        const float p = f.doppler(c.dt, {c.cam[0], c.cam[1], c.cam[2]});
        // 0x732e30: fild rate, fmul pitch, fistp with RC = chop (0xc00); Ghidra's pcode ignores RC and rounds to nearest,
        // so the emulated frequency is round(1e7 p) (the game's is trunc: within 1 Hz, port mixer: round).
        const double freq = std::nearbyint(1e7 * double(p));
        checkV(c.id, "0x4ea560 doppler: frequency (1e7 x p)", {float(freq)}, {float(c.freq)}, 0.0f);
        checkV(c.id, "0x4ea560 previous camera vector", {f.relPrev[0], f.relPrev[1], f.relPrev[2]}, {c.rel[0], c.rel[1], c.rel[2]});
    }
}
// ---- PK27 end

}  // namespace


// P46: FUEL's collision tests (0x763890 box-box, 0x71a9e0 sphere-box, 0x71b1b0 sphere-sphere, 0x765bc0 + 0x7667e0
// box-triangle; tri record 0x768d50), emulated on FUEL.exe with synthetic boxes and triangles
// (~/re-work/fuel/notes/p46-work/coll_gen.py, coll_cpp.py).
Obb vecBox(const Vec3 (&ax)[3], const Vec3& c, const Vec3& h) {
    Obb b;
    for (int i = 0; i < 3; ++i) b.ax[i] = ax[i];
    b.c = c, b.h = h, b.r = len(h);
    return b;
}
void listCheck(const char* id, const char* what, int n, const Vec3* got, const float* gotD, int wantN, const std::vector<Vec3>& want,
               const std::vector<float>& wantD) {
    check(id, what, {float(n)}, {float(wantN)});
    if (n != wantN) return;
    for (int i = 0; i < n; ++i) {
        check(id, "  point", {got[i].x, got[i].y, got[i].z}, {want[size_t(i)].x, want[size_t(i)].y, want[size_t(i)].z}, 2e-4f);
        check(id, "  depth", {gotD[i]}, {wantD[size_t(i)]}, 2e-4f);
    }
}
void collideVectors() {
    struct BB {
        const char* id;
        Vec3 ax1[3], c1, h1, ax2[3], c2, h2;
        int n;
        Vec3 normal;
        std::vector<Vec3> pts;
        std::vector<float> dep;
    } bbs[] = {
{"BB1", {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, {2.0f, 0.5f, 2.0f}, {{0.8775825618903728f, 0.0f, -0.479425538604203f}, {0.0f, 1.0f, 0.0f}, {0.479425538604203f, 0.0f, 0.8775825618903728f}}, {0.3f, 0.9f, 0.2f}, {1.0f, 0.5f, 0.5f}, 4, {0.0f, 1.0f, 0.0f}, {{-0.8172953f, 0.45f, 0.24063428f}, {-0.33786976f, 0.45f, 1.1182168f}, {1.4172953f, 0.45f, 0.15936573f}, {0.9378698f, 0.45f, -0.71821684f}}, {0.100000024f, 0.100000024f, 0.100000024f, 0.100000024f}},
    {"BB2", {{0.9800665778412416f, 0.0f, -0.19866933079506122f}, {0.0f, 1.0f, 0.0f}, {0.19866933079506122f, 0.0f, 0.9800665778412416f}}, {0.0f, 0.0f, 0.0f}, {3.0f, 0.5f, 3.0f}, {{0.7911893306268348f, 0.15892528220895627f, -0.5905609179221606f}, {0.08831869757745694f, 0.9258314362328857f, 0.36747239262449893f}, {0.605160516524734f, -0.34289780745545134f, 0.718471880369553f}}, {0.4f, 1.2f, -0.3f}, {1.2f, 0.4f, 0.6f}, 1, {0.0f, 1.0f, 0.0f}, {{-0.22165841f, 0.4666092f, 0.6927673f}}, {0.06678158f}},
    {"BB3", {{0.7071067811865476f, 0.7071067811865475f, 0.0f}, {-0.7071067811865475f, 0.7071067811865476f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, {{0.9987502603949663f, 0.03534060950936697f, 0.03534060950936696f}, {-0.04997916927067833f, 0.7062230818371108f, 0.7062230818371107f}, {0.0f, -0.7071067811865475f, 0.7071067811865476f}}, {0.1f, 2.75f, 0.2f}, {1.0f, 1.0f, 1.0f}, 1, {-0.035362724f, 0.9993745f, 0.0f}, {{-4.0960906e-08f, 1.4142137f, 0.19568029f}}, {0.08279866f}},
    {"BB4", {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, {2.0f, 0.5f, 2.0f}, {{0.4535961214255773f, 0.0f, -0.8912073600614354f}, {0.0f, 1.0f, 0.0f}, {0.8912073600614354f, 0.0f, 0.4535961214255773f}}, {6.0f, 0.2f, 0.0f}, {1.0f, 0.5f, 0.5f}, 0, {0.0f, 0.0f, 0.0f}, {}, {}},
    {"BB5", {{0.955336489125606f, 0.0f, -0.29552020666133955f}, {0.029502791919178272f, 0.9950041652780258f, 0.09537450575679464f}, {0.29404383655185584f, -0.09983341664682815f, 0.9505637859220634f}}, {5.0f, -2.0f, 1.0f}, {4.0f, 1.0f, 2.0f}, {{0.9027862393524789f, 0.28962947762551555f, 0.3179493225754375f}, {-0.19828206785239938f, 0.9362933635841992f, -0.2898947375796307f}, {-0.3816559020950483f, 0.19866933079506122f, 0.90270109637546f}}, {5.5f, -0.6f, 2.4f}, {0.9f, 0.6f, 1.4f}, 2, {0.029502792f, 0.9950042f, 0.0953745f}, {{5.3509545f, -1.3574363f, 1.0568924f}, {6.966805f, -1.1451788f, 1.5995753f}}, {0.6897325f, 0.06847775f}},
    };
    for (const BB& t : bbs) {
        Vec3 n, pts[20];
        float dep[20];
        const int k = boxBox(vecBox(t.ax1, t.c1, t.h1), vecBox(t.ax2, t.c2, t.h2), n, pts, dep);
        if (k && t.n) check(t.id, "0x763890 normal", {n.x, n.y, n.z}, {t.normal.x, t.normal.y, t.normal.z}, 2e-4f);
        listCheck(t.id, "0x763890 contacts", k, pts, dep, t.n, t.pts, t.dep);
    }
    struct SO {
        const char* id;
        Vec3 ax[3], c, h, sc;
        float r;
        int hit;
        Vec3 p, n;
        float depth;
    } sos[] = {
    {"SO1", {{0.9210609940028851f, 0.0f, -0.3894183423086505f}, {0.0f, 1.0f, 0.0f}, {0.3894183423086505f, 0.0f, 0.9210609940028851f}}, {0.0f, 0.0f, 0.0f}, {1.0f, 0.5f, 2.0f}, {0.2f, 0.8f, 0.3f}, 0.4f, 1, {0.2f, 0.45000002f, 0.3f}, {0.0f, 1.0f, 0.0f}, 0.1f},
    {"SO2", {{0.9210609940028851f, 0.0f, -0.3894183423086505f}, {0.11508098899676866f, 0.955336489125606f, 0.2721921352954314f}, {0.3720255519422596f, -0.29552020666133955f, 0.879923176281257f}}, {1.0f, 2.0f, 3.0f}, {1.0f, 0.5f, 2.0f}, {2.3f, 2.6f, 4.9f}, 0.9f, 1, {2.205635f, 1.8166373f, 4.6768064f}, {0.11508099f, 0.9553365f, 0.27219212f}, 0.16002786f},
    {"SO3", {{0.9210609940028851f, 0.0f, -0.3894183423086505f}, {0.0f, 1.0f, 0.0f}, {0.3894183423086505f, 0.0f, 0.9210609940028851f}}, {0.0f, 0.0f, 0.0f}, {1.0f, 0.5f, 2.0f}, {0.3f, 0.2f, -0.5f}, 0.3f, 1, {0.3f, 0.2f, -0.5f}, {0.0f, 1.0f, 0.0f}, 0.6f},
    };
    for (const SO& t : sos) {
        Vec3 p, n;
        float d = 0;
        const bool hit = sphereObb(vecBox(t.ax, t.c, t.h), t.sc, t.r, p, n, d);
        check(t.id, "0x71a9e0 p", {p.x, p.y, p.z}, {t.p.x, t.p.y, t.p.z}, 1e-4f);
        check(t.id, "0x71a9e0 n / depth / hit", {n.x, n.y, n.z, d, float(hit)}, {t.n.x, t.n.y, t.n.z, t.depth, float(t.hit)}, 1e-4f);
    }
    struct SS {
        const char* id;
        Vec3 cs;
        float rs;
        Vec3 c;
        float r;
        Vec3 p, n;
        float depth;
    } sss[] = {
    {"SS", {1.0f, 2.0f, 3.0f}, 0.8f, {1.5f, 2.9f, 3.2f}, 0.5f, {1.6784903f, 3.2212827f, 3.2713962f}, {0.47673127f, 0.8581164f, 0.19069254f}, 0.25119105f},
    };
    for (const SS& t : sss) {
        Vec3 p, n;
        float d = 0;
        sphereSphere(t.cs, t.rs, t.c, t.r, p, n, d);
        check(t.id, "0x71b1b0 p / n / depth", {p.x, p.y, p.z, n.x, n.y, n.z, d}, {t.p.x, t.p.y, t.p.z, t.n.x, t.n.y, t.n.z, t.depth}, 1e-4f);
    }
    struct PO {
        const char* id;
        float frame[12];
        float half[3];
        float r;
        float world[16];
        float out[16];  // 0x743ce0: rows (axis, centre) x 3, half, radius
    } pos[] = {
    {"PO1", {0.9564250858492325f, 0.09784339500725571f, -0.2750958473182437f, 1.0f, -0.036957013524625076f, 0.975170327201816f, 0.21835066314633442f, 2.0f, 0.28962947762551555f, -0.19866933079506122f, 0.9362933635841992f, -0.5f}, {0.7f, 0.4f, 1.2f}, 1.5f, {0.5802849005919296f, 0.0642433441963515f, -1.1615258184265573f, 0.0f, -0.20238956412042927f, 1.2837959830275238f, -0.030105452953517492f, 0.0f, 1.145560080694224f, 0.19426957221567898f, 0.5830535437084488f, 0.0f, 100.0f, 20.0f, -50.0f, 1.0f}, {0.16927518f, 0.10277858f, -0.9801951f, 99.60272f, 0.024095304f, 0.9938187f, 0.10836823f, 22.5347f, 0.9852742f, -0.041962158f, 0.16575234f, -51.513264f, 0.91f, 0.52000004f, 1.5600001f, 1.95f}},
    };
    for (const PO& t : pos) {
        const Obb b = placeObb(t.frame, t.half, t.r, t.world);
        check(t.id, "0x743ce0 axes", {b.ax[0].x, b.ax[0].y, b.ax[0].z, b.ax[1].x, b.ax[1].y, b.ax[1].z, b.ax[2].x, b.ax[2].y, b.ax[2].z},
              {t.out[0], t.out[1], t.out[2], t.out[4], t.out[5], t.out[6], t.out[8], t.out[9], t.out[10]}, 1e-5f);
        check(t.id, "0x743ce0 centre / half / r", {b.c.x, b.c.y, b.c.z, b.h.x, b.h.y, b.h.z, b.r},
              {t.out[3], t.out[7], t.out[11], t.out[12], t.out[13], t.out[14], t.out[15]}, 1e-5f);
    }
    struct BT {
        const char* id;
        Vec3 ax[3], c, h, v0, v1, v2;
        int code;
        float best;
        int n;
        std::vector<Vec3> pts;
        std::vector<float> dep;
    } bts[] = {
        // (R27 of coll_gen.py is left out: an edge axis is perpendicular to the box axis whose sign picks the
        // corner, so the corner is decided by the last bits of the x87 rounding.)
    {"BT1", {{0.9534613708504311f, 0.07981482141109528f, -0.29075936541946806f}, {-0.061622807066763666f, 0.9955559637318878f, 0.07121062229109201f}, {0.2951508833549871f, -0.04997916927067833f, 0.9541425672790118f}}, {0.5f, 0.45f, 0.2f}, {1.0f, 0.5f, 0.6f}, {-5.0f, 0.0f, -5.0f}, {-5.0f, 0.0f, 5.0f}, {5.0f, 0.0f, -5.0f}, 1, 0.1575803f, 3, {{-0.43032932f, -0.12629245f, 0.43032932f}, {-0.5997405f, -0.0976053f, -0.117331505f}, {-0.02571249f, -0.0723059f, 0.02571249f}}, {0.12629245f, 0.0976053f, 0.0723059f}},
    {"BT2", {{0.9800665778412416f, 0.0f, -0.19866933079506122f}, {0.0f, 1.0f, 0.0f}, {0.19866933079506122f, 0.0f, 0.9800665778412416f}}, {0.0f, 0.0f, 0.0f}, {2.0f, 1.0f, 2.0f}, {1.6f, -0.3f, -0.2f}, {2.6f, 0.4f, 0.1f}, {1.7f, 0.5f, 0.6f}, 2, 0.45308828f, 4, {{1.7f, 0.5f, 0.6f}, {1.6f, -0.3f, -0.2f}, {2.0260446f, -0.0017686919f, -0.07218658f}, {2.1155102f, 0.45383218f, 0.36916095f}}, {0.45308834f, 0.39215955f, 9.576492e-08f, 4.9970165e-08f}},
    {"BT3", {{0.8253356149096783f, 0.5646424733950354f, 0.0f}, {-0.5646424733950354f, 0.8253356149096783f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.9f, 0.0f}, {1.0f, 0.5f, 0.5f}, {-0.3f, 0.0f, -3.0f}, {-0.25f, 0.1f, 3.0f}, {0.2f, 3.0f, 0.1f}, 3, 1.0734093f, 4, {{-0.0490966f, 1.4722253f, -0.5f}, {-0.27916667f, 0.041666687f, -0.5f}, {-0.27083334f, 0.058333337f, 0.5f}, {-0.042744823f, 1.4765708f, 0.5f}}, {2.3586459e-08f, 1.0507836f, 1.0417335f, 2.9779748e-08f}},
    {"BT4", {{0.9828939875470718f, 0.18298657129998708f, -0.02086441870006303f}, {-0.15957479896450788f, 0.90270109637546f, 0.3995830503637253f}, {0.09195266597143172f, -0.3894183423086505f, 0.9164595255079895f}}, {0.2f, 0.3f, -0.1f}, {0.8f, 0.4f, 1.0f}, {-2.0f, 0.2f, -2.0f}, {-1.8f, 0.1f, 3.0f}, {3.0f, -0.1f, -1.5f}, 1, 0.65703845f, 7, {{-0.43053257f, -0.596888f, 0.6733179f}, {-0.5950699f, 0.09992534f, -0.96656704f}, {-0.22774649f, 0.07494569f, -0.76344347f}, {0.9306464f, -0.24559797f, 0.42330718f}, {1.104758f, -0.14597599f, 0.26778245f}, {1.073487f, -0.013544083f, -0.04388368f}, {0.3860185f, 0.03320688f, -0.42404187f}}, {0.65703845f, 0.0f, 0.0f, 0.23168772f, 0.12487254f, 0.0f, 3.9683767e-10f}},
    {"BT5", {{0.7648421872844885f, 0.0f, -0.644217687237691f}, {0.12798629680985413f, 0.9800665778412416f, 0.1519506855116402f}, {0.6313762241158432f, -0.19866933079506122f, 0.7495962650805187f}}, {0.0f, 1.0f, 0.0f}, {1.5f, 0.5f, 0.8f}, {-1.0f, 0.6f, 0.2f}, {0.4f, 2.0f, 0.9f}, {1.2f, 0.3f, -0.8f}, 3, 1.1540236f, 4, {{1.2f, 0.3f, -0.8f}, {-1.0f, 0.6f, 0.2f}, {-0.16418988f, 1.4358101f, 0.617905f}, {0.69330496f, 1.3767271f, 0.27672702f}}, {1.1540236f, 0.9896228f, -0.0f, 1.2271454e-08f}},
    {"R0", {{-0.8008164929625798f, 0.4184500283892208f, -0.42847697527425516f}, {-0.004324094352901375f, 0.7113698837633033f, 0.7028045181148252f}, {0.5988941866533198f, 0.5646702243041019f, -0.5678673181097974f}}, {0.21587200412565682f, -0.1628764779812396f, 0.009430370414111877f}, {1.0004765826883273f, 1.436639426372075f, 0.3386176678275213f}, {-2.155f, -0.9f, 1.625f}, {1.716f, -1.823f, 2.098f}, {0.139f, 2.339f, 2.173f}, 6, 0.24565029f, 0, {}, {}},
    {"R1", {{-0.7353622340626903f, 0.1216788581635205f, 0.6666608134504015f}, {-0.28318561697603867f, -0.9489134487931493f, -0.13917317642919355f}, {0.6156689784715724f, -0.29113145171127247f, 0.732252816158639f}}, {-0.20633331624840162f, -0.02063489184991446f, -0.25540815684384793f}, {1.1986494245808905f, 0.9533070589840265f, 0.4312582203685975f}, {-1.368f, 0.033f, 1.392f}, {-1.125f, -1.967f, -0.726f}, {-2.155f, -0.822f, 1.859f}, 11, 0.015684988f, 0, {}, {}},
    {"R2", {{0.8989772399895107f, 0.37116613681935795f, 0.23254165403091803f}, {0.26016895658586414f, -0.025406603851303826f, -0.9652287907588365f}, {-0.3523521477298872f, 0.9282188337667667f, -0.11940586514405044f}}, {-0.17780898289806207f, -0.23109246082624668f, -0.15110475274585713f}, {1.0323378835616757f, 1.1540777984724997f, 1.1120799003316588f}, {-2.177f, -1.08f, -1.921f}, {0.301f, -0.671f, 1.256f}, {0.343f, 0.411f, -1.642f}, 1, 1.4075336f, 5, {{-0.41375828f, -1.6758361f, -1.3723266f}, {-0.5768806f, -1.6599064f, -0.7671417f}, {-0.04397416f, -1.3376727f, 0.16524696f}, {0.9073081f, -1.0636289f, -0.5115514f}, {0.5920057f, -1.0150009f, 0.7968836f}}, {1.2202333f, 0.946004f, 0.68341684f, 1.1468723f, 0.561598f}},
    {"R3", {{-0.6230267634672534f, 0.5831713485171507f, 0.5212953388168813f}, {0.751396378362035f, 0.6313922352601555f, 0.19169592545905081f}, {-0.21735025785109288f, 0.5111311216526269f, -0.8315671000527805f}}, {0.2149548739367934f, -0.1954634313052653f, -0.29939448830607507f}, {1.1590399493639916f, 0.5225948888114361f, 0.6610113448276993f}, {0.373f, 2.2f, -2.384f}, {0.747f, -2.325f, 1.374f}, {-0.672f, -2.071f, -0.54f}, 2, 1.3708441f, 4, {{0.5263167f, 0.34503168f, -0.84345406f}, {0.61590177f, -0.73885137f, 0.0567078f}, {0.25606623f, -1.1348466f, -0.0926429f}, {-0.1248308f, -0.37154937f, -1.1137147f}}, {0.9966388f, 0.77798545f, 0.69338375f, 0.843546f}},
    {"R4", {{0.8879335723746664f, 0.377443722510582f, -0.26288820321063794f}, {-0.08006556122737368f, -0.4359779087569384f, -0.8963887376474965f}, {-0.45294975102687557f, 0.8169819455858749f, -0.3568991785244259f}}, {0.16833925926108023f, 0.19189413910992054f, 0.13671313969055654f}, {0.8651787970420788f, 1.145939710636384f, 0.47874146810219687f}, {-0.583f, -0.942f, -1.84f}, {-2.295f, -2.236f, 2.379f}, {-0.612f, -0.903f, 0.781f}, 6, 0.14370133f, 0, {}, {}},
    {"R5", {{0.24332753566839369f, -0.45182184721389085f, 0.8582824294868061f}, {-0.6609322175423065f, 0.5704054736525425f, 0.48765377004779553f}, {-0.7099016229028893f, -0.6859260995234445f, -0.15982825717750201f}}, {-0.2657971572635272f, -0.1516011389840166f, -0.0664785258674529f}, {0.66760407948916f, 0.9671802256713995f, 0.5895209223309665f}, {1.994f, -0.859f, 1.97f}, {2.128f, 0.805f, -1.139f}, {-2.216f, 2.473f, 2.114f}, 2, 1.4378672f, 0, {}, {}},
    {"R6", {{0.0305157456565139f, -0.8734395422308986f, -0.4859754678319734f}, {-0.9980960330184987f, -0.0005545162974653679f, -0.06167658700359252f}, {0.05360128860170085f, 0.48693229362936513f, -0.8717934636604492f}}, {-0.12381367993557715f, -0.03792645090326918f, 0.28792794466800936f}, {1.4014045269516486f, 1.317782850429556f, 0.3897135091845119f}, {0.763f, 0.525f, 1.083f}, {-2.42f, 2.063f, -1.656f}, {-2.493f, 0.786f, 1.186f}, 2, 0.5503986f, 5, {{-1.493731f, 0.73771834f, 1.0839545f}, {0.3307911f, 0.5596457f, 1.0966724f}, {0.763f, 0.525f, 1.083f}, {-0.053866565f, 0.9197033f, 0.38007894f}, {-1.4498111f, 1.055947f, 0.37034842f}}, {0.2952723f, 0.500304f, 0.5503986f, 0.5223242f, 0.3654541f}},
    {"R7", {{0.5859736521494621f, 0.7826970601927602f, 0.20980989240794162f}, {0.29083164063647043f, 0.03853109371866824f, -0.9559980709298226f}, {-0.7563410842941961f, 0.6212090363013516f, -0.20505486442956147f}}, {0.027208525478861156f, 0.0632648678148433f, 0.1437838360198947f}, {0.31734734245210944f, 1.084223843937136f, 0.5567385887252403f}, {-0.793f, -1.723f, -0.018f}, {-1.224f, 0.631f, -1.072f}, {-1.905f, -0.855f, -1.014f}, 1, 1.0495479f, 0, {}, {}},
    {"R8", {{0.2012367961321023f, -0.5122227769293513f, 0.834944057214177f}, {-0.31593880227855364f, -0.8407684183886611f, -0.4396488824676144f}, {0.9271927658784227f, -0.17531769287500132f, -0.3310245934484431f}}, {0.2186553311356358f, 0.08522051129146846f, 0.2895406056199041f}, {0.8883390928318404f, 0.870327754736002f, 0.6251665108629987f}, {-1.3f, 2.042f, -0.095f}, {0.771f, 2.158f, 0.151f}, {-1.6f, 1.042f, 1.476f}, 0, 3e+38f, 0, {}, {}},
    {"R9", {{0.6958927481944752f, -0.7165346345287315f, 0.04807702706197603f}, {-0.5491620028824993f, -0.5740910403670221f, -0.6073224612678114f}, {0.46276816830938416f, 0.3962292201378991f, -0.7929993868273101f}}, {0.10940547762899033f, 0.17306077223535704f, -0.2736834243476576f}, {1.2734298938414492f, 1.304454693209173f, 0.38280667830641846f}, {0.592f, -1.968f, 0.093f}, {-1.849f, 0.373f, -1.106f}, {-0.058f, 0.607f, 1.411f}, 1, 0.11341619f, 3, {{-1.362402f, 0.15353596f, -0.6596046f}, {-1.6702728f, 0.18496239f, -0.8235653f}, {-1.4747458f, 0.38936532f, -0.60733044f}}, {0.0f, 0.11341614f, 0.0f}},
    {"R10", {{0.3520525036988871f, 0.436367734283123f, 0.8280351653860838f}, {0.9289967021567604f, -0.2707748436368741f, -0.25228180956083585f}, {0.11412345079434176f, 0.8580583806070775f, -0.5007111477176749f}}, {-0.048658542416461226f, -0.09531198732493998f, -0.17619463576933098f}, {1.4790991274985958f, 0.7195264546580502f, 0.30187313806358845f}, {-1.27f, 1.364f, 1.109f}, {-0.138f, -2.26f, -0.82f}, {2.067f, -1.798f, -2.039f}, 4, 0.77115303f, 4, {{-0.47307667f, 0.60886914f, 0.3572125f}, {-0.70656186f, -0.1037481f, 0.26228625f}, {0.3948722f, -0.7852361f, -0.65452397f}, {0.5378033f, -0.3489979f, -0.5964135f}}, {0.5905827f, 7.395197e-09f, 4.8395695e-08f, 0.36153314f}},
    {"R11", {{-0.706108422514524f, -0.5967166709539136f, -0.381234980372643f}, {-0.6533878678193988f, 0.7565777756368366f, 0.02596851167927189f}, {0.2729380696064043f, 0.2674308957807522f, -0.9241133729913414f}}, {0.19676138260877024f, -0.20582340280724262f, 0.2242785045819356f}, {1.4738124567768531f, 1.2190993662739038f, 1.2280387791685246f}, {-1.901f, -1.313f, -0.126f}, {-1.562f, 1.676f, -0.548f}, {0.716f, -0.519f, 2.353f}, 1, 0.85415816f, 7, {{-0.045582652f, 0.15093231f, 1.5848273f}, {0.1295867f, 0.04604709f, 1.6062108f}, {-0.30224144f, -0.530414f, 1.3118467f}, {-1.9756333f, -0.49134386f, 0.828915f}, {-1.5094924f, -1.0311027f, 0.8103885f}, {-0.59066844f, -0.91544425f, 1.1152349f}, {-0.10161054f, -0.26258528f, 1.4486108f}}, {0.12729092f, 0.0f, 0.0f, 0.85415816f, 0.41747442f, 0.0f, 0.0f}},
    {"R12", {{-0.8728598078770664f, -0.440437523388101f, -0.21007271070885591f}, {0.05211230257447874f, 0.3439046115694057f, -0.9375574254741317f}, {0.4851804444876119f, -0.8293035669361686f, -0.27722829969855384f}}, {0.08037901535545466f, -0.2558571283182271f, 0.26212222615714015f}, {0.5016747245937234f, 0.39721902559226757f, 1.3270797254457367f}, {-1.584f, -2.338f, 1.832f}, {2.117f, -0.757f, -0.712f}, {1.142f, 1.128f, -1.079f}, 1, 0.89369947f, 8, {{0.22170508f, -1.6388395f, 0.18636131f}, {-0.294304f, -0.75684154f, 0.48120487f}, {-0.3110808f, -0.71953404f, 0.47269404f}, {-0.262527f, -0.6145879f, 0.050922036f}, {0.30706048f, -1.4407601f, -0.58358574f}, {-0.22444463f, -0.5322752f, -0.2798879f}, {-0.27477694f, -0.64106536f, 0.15733302f}, {0.22827792f, -1.6397574f, 0.16097498f}}, {0.36313525f, 1.5087238e-08f, 8.791839e-08f, 0.29123428f, 0.89369947f, 0.5196591f, 0.21775739f, 0.38187274f}},
    {"R13", {{-0.5469211053758329f, 0.5979341936690451f, -0.5859624600056098f}, {-0.7855300389773815f, -0.12447313689559586f, 0.6061757138450564f}, {0.28951660117974326f, 0.7918214055156119f, 0.5377722560792909f}}, {-0.2882699532642589f, 0.1372193978539895f, -0.06989075047733251f}, {0.9426699059094081f, 0.6029656068564835f, 0.751629834805154f}, {1.008f, 1.893f, 1.646f}, {0.062f, 1.709f, -1.008f}, {0.619f, -1.072f, 1.97f}, 10, 0.17406465f, 0, {}, {}},
    {"R14", {{0.16514958389497558f, -0.931829466796802f, -0.3231400002297828f}, {-0.8049717597876234f, 0.061957920542232425f, -0.5900692179960746f}, {0.5698649672367421f, 0.3575682604640267f, -0.739864080912837f}}, {-0.21023322647320536f, 0.2117244554750845f, 0.19684340714298343f}, {1.3504581468367418f, 0.36731323758159173f, 0.670554813465799f}, {-2.27f, -2.327f, -0.333f}, {0.47f, -2.316f, 0.89f}, {-0.684f, -1.798f, 0.659f}, 1, 2.2979167f, 0, {}, {}},
    {"R15", {{-0.7219567202903312f, 0.2717951950460646f, -0.636322140096901f}, {-0.6400937657399656f, -0.6116000387482528f, 0.4650003910363579f}, {-0.2627897735614126f, 0.7430159921246698f, 0.6155231680111919f}}, {0.04574142942594145f, -0.07731201524548809f, 0.02635080596946815f}, {1.4239408403573273f, 0.49726600185058034f, 1.1243220291044107f}, {2.263f, 0.852f, -1.963f}, {1.473f, 0.111f, -1.566f}, {1.02f, 1.881f, 1.155f}, 2, 1.4579523f, 0, {}, {}},
    {"R16", {{0.8532962777227109f, -0.0726922405071778f, 0.5163344851882476f}, {0.48216632486712774f, 0.48696505101234683f, -0.7282723901512881f}, {-0.1984970971795142f, 0.8703912208096938f, 0.4505752158615889f}}, {0.217395018122097f, -0.08709738499452283f, 0.02251119711275923f}, {1.3032045138107675f, 1.2017959176022828f, 0.9125619865795132f}, {-1.527f, 0.002f, 2.135f}, {1.868f, -1.264f, -2.242f}, {-0.367f, 0.539f, 2.168f}, 2, 1.7074163f, 5, {{1.0523484f, -0.60600454f, -0.6325935f}, {0.03255856f, 0.21667156f, 1.3796092f}, {-0.31750566f, 0.102728106f, 1.4454998f}, {-0.61181504f, -0.3392737f, 0.9550987f}, {0.5457034f, -0.7709139f, -0.53723073f}}, {0.8912746f, 0.78229f, 1.0386941f, 1.510908f, 1.262366f}},
    {"R17", {{-0.4031590574471756f, -0.9086806327041101f, 0.10845405546480552f}, {-0.7022921896242142f, 0.23123033119774225f, -0.6732891015953022f}, {0.5867268996655358f, -0.347609035674622f, -0.7313819135897659f}}, {0.17417623090827894f, 0.03850539689241206f, -0.2455825898527575f}, {0.8429878445460122f, 1.0294244866491227f, 0.8055574188204009f}, {0.352f, 1.384f, 2.053f}, {0.341f, -1.8f, 2.375f}, {1.501f, -0.125f, 0.034f}, 2, 1.8880134f, 0, {}, {}},
    {"R18", {{-0.736227571105652f, -0.6726103743956131f, 0.0745938858034981f}, {0.5905923689493232f, -0.5847809818288755f, 0.5560861956838012f}, {-0.330408258496513f, 0.45346056889949055f, 0.8277704362748477f}}, {-0.0017417382407931559f, 0.2930258937829267f, -0.19485988823899952f}, {1.4071642865691445f, 0.5192239932503191f, 0.7324036200400386f}, {1.761f, -0.514f, -1.5f}, {-1.035f, -0.256f, -1.953f}, {2.317f, -0.03f, -0.825f}, 3, 1.7271423f, 0, {}, {}},
    {"R19", {{-0.34231823842776077f, -0.8791662089541271f, -0.33148906569137815f}, {0.322579237936332f, 0.22139196446648063f, -0.9202870385494343f}, {0.8824742822836992f, -0.4219625280790945f, 0.20781425842556925f}}, {0.2928622157540219f, 0.10086871194019392f, -0.09941638862274199f}, {1.483477689131668f, 1.1565459170680001f, 0.5696937139552429f}, {1.759f, 0.05f, -1.286f}, {-1.829f, 1.748f, -1.732f}, {1.051f, -0.067f, -1.488f}, 0, 3e+38f, 0, {}, {}},
    {"R20", {{0.9806118994872569f, 0.18376903375093617f, 0.06803855391056698f}, {0.19567147312388333f, -0.9370858462685266f, -0.28910688564738246f}, {0.010629072844008559f, 0.29681485636242433f, -0.9548758892406018f}}, {-0.28164982493748797f, -0.05073729896707341f, -0.2843380480697317f}, {1.2943560111281935f, 1.2164349948735742f, 0.4543143672860758f}, {2.292f, 1.579f, -1.617f}, {-0.148f, 0.443f, 1.948f}, {-1.695f, 1.46f, -0.407f}, 1, 0.29208186f, 4, {{0.7447605f, 1.192182f, 0.5892216f}, {-0.22983694f, 1.11278f, 0.18696761f}, {-0.18084335f, 1.1882842f, -0.024605632f}, {0.7544184f, 1.4618765f, -0.27840602f}}, {0.29208183f, 0.0f, 0.0f, 0.23976363f}},
    {"R21", {{-0.21215043238536646f, -0.344566498482642f, -0.9144758729250944f}, {-0.9545653404219135f, -0.12733177766860493f, 0.26942833789516135f}, {-0.2092778175151354f, 0.9300863113276985f, -0.30189774523327806f}}, {0.2355939612978279f, 0.22057079848742717f, -0.23516598596437827f}, {1.4557761945975338f, 1.205441642062404f, 1.2339908684157752f}, {1.518f, 1.625f, 2.083f}, {0.098f, 2.25f, 2.316f}, {1.911f, -1.018f, 1.891f}, 1, 4.3651648f, 0, {}, {}},
    {"R22", {{-0.773671340249644f, 0.6095527918007068f, -0.1728526866562504f}, {-0.5963330101692775f, -0.6083879275871508f, 0.5236898610329019f}, {0.21405512901766224f, 0.5082415996100437f, 0.8341887545196567f}}, {-0.2209153466182334f, -0.021354245531065408f, -0.16454206783050282f}, {1.3549079109693158f, 1.4880802292744515f, 1.1901317363637063f}, {1.021f, 0.572f, -0.442f}, {0.063f, 0.821f, 2.048f}, {-1.925f, 0.033f, -1.869f}, 1, 1.8060458f, 5, {{-1.1942009f, 0.9036602f, -0.38472772f}, {-0.12702346f, 2.3147383f, -0.18523952f}, {-0.39971507f, 1.6672732f, -1.2479391f}, {-0.89826226f, 0.98068655f, -1.4376913f}, {-1.2400743f, 0.8183514f, -0.4802384f}}, {0.5652301f, 1.8060459f, 1.3497094f, 0.7567522f, 0.49992502f}},
    {"R23", {{-0.8496948472479235f, -0.38065874491277685f, 0.3648528285237175f}, {-0.32977881949349647f, -0.1562558313536692f, -0.9310370805620201f}, {0.4114177885967057f, -0.9114181450297131f, 0.007236583226441701f}}, {-0.23272602205788337f, 0.09644719625204984f, -0.25508214460464307f}, {0.34248554364645106f, 1.3291676137523196f, 1.0841504076356245f}, {0.059f, 2.369f, 0.541f}, {0.68f, -0.4f, 0.683f}, {2.219f, 1.229f, 1.438f}, 2, 0.09818588f, 3, {{0.7714526f, -0.30319932f, 0.7278647f}, {0.57454485f, 0.070217885f, 0.6588863f}, {0.68f, -0.4f, 0.683f}}, {-0.0f, 1.3117383e-08f, 0.098185815f}},
    {"R24", {{-0.02712777479428099f, 0.9333097295485644f, 0.3580461318946153f}, {-0.9821945634176361f, 0.041722819493099146f, -0.18317490528559188f}, {-0.18589761544342107f, -0.35664009177815637f, 0.915559895096393f}}, {0.19623067462884108f, 0.2963274801982491f, 0.08829752100923816f}, {0.40550430241754326f, 1.388736418049097f, 0.5896024791283734f}, {1.465f, -2.044f, 0.175f}, {1.994f, 0.571f, -1.523f}, {1.081f, 1.135f, 1.841f}, 3, 0.23366141f, 5, {{1.355536f, -0.34584457f, 0.7175194f}, {1.6583166f, 0.07115728f, -0.34652808f}, {1.5993339f, 0.8148025f, -0.068830304f}, {1.5910503f, 0.81991965f, -0.038309045f}, {1.2953715f, 0.41269886f, 1.0007805f}}, {0.10802211f, 0.022937918f, 0.061030146f, 0.06378904f, 0.14687754f}},
    {"R25", {{-0.4079309474876829f, -0.8924264688679094f, -0.1927883288629152f}, {0.21062370801197833f, 0.11347665114389817f, -0.9709586516780471f}, {0.888386174872778f, -0.4366898754369286f, 0.1416755341821358f}}, {-0.2248345933792882f, -0.2472427370860359f, -0.2981309147344587f}, {1.149892938498579f, 0.6546324863018855f, 0.8100737261030038f}, {1.247f, -0.597f, -1.201f}, {-0.848f, -0.464f, 1.129f}, {-0.044f, -0.626f, 2.253f}, 5, 0.46710205f, 0, {}, {}},
    {"R26", {{0.6341792595568985f, 0.6106577333439319f, -0.47425077696838347f}, {-0.7016015026687659f, 0.19673909507276635f, -0.6848715645454108f}, {-0.32491844853869684f, 0.7670663994578307f, 0.5532062369695464f}}, {-0.06394043902662513f, 0.09490024411165943f, 0.29343234146548863f}, {0.5529921482614288f, 1.3307197281482546f, 1.0338641866310148f}, {-0.427f, -1.665f, 1.795f}, {-0.411f, 2.085f, 1.837f}, {-2.104f, 0.84f, -0.4f}, 8, 0.1471424f, 0, {}, {}},
    {"R28", {{0.44677447559522077f, -0.7141495062486377f, -0.5388720169775403f}, {-0.8260524690432612f, -0.5605942517963991f, 0.058063785963090336f}, {-0.3435547792480082f, 0.4191951425978864f, -0.8403841657707444f}}, {0.20506251520022983f, 0.28015318358764624f, 0.2661334574898107f}, {1.41006223655733f, 1.256525304808286f, 0.33414816479421494f}, {1.406f, -0.578f, -0.824f}, {-1.962f, -0.371f, -0.914f}, {-1.264f, 1.625f, -0.154f}, 9, 0.05260168f, 1, {{1.1060907f, -0.33067924f, -0.7755469f}}, {0.05260168f}},
    {"R29", {{-0.1506805214349965f, -0.46237220439708393f, -0.8737890621088471f}, {-0.8342159243335028f, -0.4148006941960405f, 0.3633513116570277f}, {-0.5304518564850887f, 0.7836787152242072f, -0.32321587098417476f}}, {0.14835713892750158f, -0.10590734268177646f, -0.2253397870359628f}, {0.3420809939633022f, 0.8835421598425672f, 0.6726728438209888f}, {-2.215f, 0.924f, 1.291f}, {0.425f, -1.537f, -1.19f}, {-0.57f, -1.85f, 0.476f}, 11, 0.055570472f, 1, {{-0.64321405f, -0.5779034f, -0.21205159f}}, {0.055570472f}},
    {"R30", {{0.17746474465629836f, -0.8329604209236157f, 0.5241022816004791f}, {-0.9758549391252921f, -0.08003772823792456f, 0.20322672029849942f}, {-0.12733185849914863f, -0.5475133781317724f, -0.8270524158588062f}}, {-0.0816713456855378f, 0.2049825211105633f, -0.1471989505478474f}, {0.9942941786817578f, 0.6218442693601696f, 1.1858854498849771f}, {2.183f, 1.104f, -1.304f}, {-1.594f, 1.871f, -1.591f}, {-1.047f, -0.528f, -1.802f}, 1, 0.19994369f, 3, {{-0.44249177f, 0.38116527f, -1.6421512f}, {0.1977055f, 0.43367302f, -1.7754755f}, {0.087141514f, 0.20812029f, -1.6091361f}}, {0.0f, 0.1999437f, 1.0016867e-08f}},
    {"R31", {{-0.003641551161002489f, 0.9765428859024142f, 0.21529219934434696f}, {0.9549043092388035f, 0.06731505834825108f, -0.28918236999640007f}, {-0.2968913931091723f, 0.2045303765042146f, -0.9327501411335938f}}, {0.16796954981799556f, 0.07876608921923833f, 0.228039793870438f}, {0.6351118591388221f, 0.3717651338395551f, 0.939333981170607f}, {-1.328f, -2.012f, -0.468f}, {-2.037f, -0.916f, -2.028f}, {-0.954f, -0.628f, 2.083f}, 0, 3e+38f, 0, {}, {}},
    {"R32", {{-0.23903920688710317f, -0.4728214461674231f, -0.8481156392939181f}, {0.96450016496435f, -0.014647981542213173f, -0.26367568795867513f}, {0.11224833786977134f, -0.8810365013328821f, 0.4595378047175097f}}, {0.29293066924218086f, 0.17250893793126088f, -0.18002769038822253f}, {0.3153999869904555f, 0.477394440762427f, 0.755195113120644f}, {1.071f, -1.289f, -2.283f}, {-1.77f, -0.302f, 1.585f}, {0.731f, 0.646f, 2.265f}, 1, 0.17351185f, 4, {{-0.15813994f, -0.63498056f, 0.025394678f}, {0.91354007f, -0.3507105f, 0.30863118f}, {-0.007354021f, -0.33672482f, 0.56038594f}, {0.76275414f, -0.64896625f, -0.22636008f}}, {0.049258776f, 0.1544872f, 0.030234192f, 0.17351177f}},
    {"R33", {{0.9063337253196563f, 0.08320258015076931f, -0.4142903679841576f}, {-0.42027596581801757f, 0.2793496677755026f, -0.8633260540893276f}, {0.04390092144735044f, 0.9565778033019432f, 0.2881520697931952f}}, {-0.2608682853294994f, -0.2825206780238267f, 0.19135125964323263f}, {1.3751307591921773f, 1.3385318342704124f, 0.8497032955608297f}, {-0.823f, -1.179f, 1.278f}, {-1.75f, -2.34f, -2.053f}, {0.47f, 1.093f, -0.608f}, 4, 1.9672469f, 5, {{0.47f, 1.093f, -0.6079999f}, {-0.8230001f, -1.179f, 1.278f}, {-0.94620955f, -1.333311f, 0.8352698f}, {-0.68829536f, -0.7055805f, -1.2878999f}, {-0.5683765f, -0.5127416f, -1.28388f}}, {1.9672469f, 0.28059304f, -0.0f, 2.4156135e-08f, 0.19088824f}},
    {"R34", {{-0.9723641936219692f, -0.23346090314868895f, 0.001970193618925295f}, {-0.00787868031306746f, 0.024378384718700068f, -0.99967175650562f}, {0.23333624098802583f, -0.9720605439269231f, -0.025544032220313456f}}, {-0.1101385975169916f, 0.0231090348756281f, 0.11396448379566904f}, {0.736123588484151f, 0.32509194100714606f, 0.5011614295578894f}, {-2.039f, -1.515f, 1.422f}, {2.465f, 1.231f, 1.32f}, {-0.254f, -1.076f, 1.809f}, 0, 3e+38f, 0, {}, {}},
    {"R35", {{0.2082994501866816f, -0.7857742583821192f, -0.5823831676104287f}, {0.10280826846014282f, -0.5745550158396059f, 0.81198337034066f}, {-0.9726468007391755f, -0.2290094946450495f, -0.03889540299657149f}}, {-0.14301032633046087f, -0.2851273893277088f, 0.09766267336942808f}, {1.041598671443774f, 0.9093304271353111f, 1.2384184010609252f}, {-0.575f, -0.141f, 1.107f}, {0.75f, -0.153f, -1.727f}, {-1.174f, -0.337f, -2.211f}, 3, 1.6016743f, 3, {{0.4083985f, -0.14990625f, -0.9963596f}, {-0.9379798f, -0.25977138f, -0.9036294f}, {-0.575f, -0.141f, 1.107f}}, {-0.0f, 3.802663e-08f, 1.6016743f}},
    {"R36", {{0.9863470493192293f, -0.16234127913697863f, 0.027655151191406963f}, {0.08928574963019592f, 0.6682881494742096f, 0.7385248839310092f}, {-0.13837468414498355f, -0.7259726292089159f, 0.6736588071325647f}}, {-0.0072206121127394995f, 0.21470596289092886f, 0.006363732462969507f}, {1.0250190799804704f, 1.257571143659819f, 1.1935732879588639f}, {0.981f, 0.725f, -1.01f}, {1.587f, -1.222f, -0.543f}, {2.185f, 0.319f, 1.796f}, 2, 0.16124004f, 3, {{1.0864381f, 0.38624132f, -0.92874664f}, {1.126847f, 0.67581904f, -0.67009395f}, {0.981f, 0.725f, -1.01f}}, {-0.0f, 2.9093117e-08f, 0.16124006f}},
    {"R37", {{0.2930504783298913f, -0.06981415422843719f, -0.953544650774147f}, {-0.955932551981044f, -0.002899614080897224f, -0.2935720495912215f}, {0.01773057285117101f, 0.9975558009993977f, -0.06758735590936718f}}, {-0.04355164857120086f, 0.1258157913469934f, -0.24251299469267829f}, {1.2242814054646494f, 1.2667976098380636f, 0.4849405085921536f}, {-0.549f, 0.755f, -0.446f}, {-1.665f, -2.122f, -1.33f}, {-1.908f, -0.745f, 2.393f}, 3, 0.7257094f, 5, {{-0.9896326f, -0.38093197f, -0.79503155f}, {-1.2901206f, -0.2751341f, 0.6876645f}, {-1.1043794f, 0.14199853f, 0.71420735f}, {-0.66606325f, 0.62579113f, -0.2014507f}, {-0.6071648f, 0.6050537f, -0.49207318f}}, {0.19873469f, 0.34707335f, 0.53363115f, 0.685223f, 0.6561472f}},
    {"R38", {{-0.1252661978852907f, 0.8886146925375716f, 0.4412168490365256f}, {0.6411808466236558f, -0.2668592910963606f, 0.7194951290165346f}, {0.7570967584069654f, 0.37302821201874925f, -0.5363342721174547f}}, {0.060525903754054367f, 0.11194547302738767f, -0.1515121322301738f}, {0.5394724942384685f, 1.338828816096701f, 1.025844906453972f}, {-1.564f, 0.953f, -1.955f}, {1.631f, -1.717f, -1.859f}, {-0.779f, -0.614f, 1.392f}, 2, 0.6946149f, 5, {{-1.0904778f, 0.0077654123f, 0.063953504f}, {-1.275233f, 0.3765697f, -0.72378606f}, {-0.63416594f, 0.037220955f, -1.4209392f}, {-0.17519839f, -0.89034575f, 0.5774942f}, {-0.23818566f, -0.86151797f, 0.66246164f}}, {0.6861455f, 0.68945f, -0.0f, -0.0f, 0.07099602f}},
    {"R39", {{-0.1051570266061075f, 0.4485101647096179f, -0.8875700715478828f}, {0.3961942451214731f, -0.7997322379980237f, -0.4510637068522564f}, {-0.9121250571538907f, -0.3990826727118375f, -0.09359968190751457f}}, {-0.22944532794142636f, 0.05105855274424165f, -0.15543075224708128f}, {0.608995204963692f, 1.2306627765389309f, 1.2971238599323356f}, {-2.479f, -2.472f, -0.802f}, {1.202f, 0.594f, 0.286f}, {2.065f, 2.172f, 0.987f}, 2, 0.3101827f, 4, {{0.7759328f, 0.8545643f, 0.47948658f}, {-1.1360433f, -1.0994886f, -0.2732699f}, {-1.0466911f, -1.2789927f, -0.37864977f}, {0.9924966f, 0.41949922f, 0.22407667f}}, {0.30011952f, 0.29288873f, 0.29651535f, 0.30890936f}},
    };
    for (const BT& t : bts) {
        const Tri tri = makeTri(t.v0, t.v1, t.v2);
        Vec3 pts[8];
        float dep[8];
        Vec3 axis;
        const int k = boxTriangle(vecBox(t.ax, t.c, t.h), tri, pts, dep, &axis);
        listCheck(t.id, "0x765bc0/0x7667e0 contacts", k, pts, dep, t.n, t.pts, t.dep);
    }
}


// P46: FUEL's damage records and crash meter, emulated on FUEL.exe (notes/p46-work/dmg_gen.py): 0x420180 slot add,
// 0x51fd50 directional damage (with the direction table read from the player's FUEL.exe: FUEL_EXE or argv[1]),
// 0x41ff30 largest slot, 0x4200c0 repair, 0x51ee90 crash meter.
void damageVectors(const char* exe) {
    {
        DamageRecord r{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5f, 2.0f};
        damageAdd(r, 9, 0.7f);
        damageAdd(r, 9, -0.3f);
        check("D1", "0x420180 slot 9 / total", {r[9], r[10]}, {0.7f, 2.4f});
    }
    {
        DamageRecord r{0.1f, 0.2f, 0.9f, 0, 0.95f, 0, 0, 0.3f, 0, 0.8f, 1.5f};
        check("D3", "0x41ff30 largest slot", {damageMax(r)}, {0.95f});
        damageLower(r, 1.0f, 0.0f);
        check("D3", "0x4200c0 repair (1, 0)", {damageMax(r), r[10]}, {0.0f, 1.5f});
    }
    const std::vector<float> dirs = exe ? fuel::readEngineFloats(std::string(exe), 0xa057a8, 30) : std::vector<float>{};
    if (dirs.size() == 30) {
        uint32_t h = 2166136261u;
        for (float f : dirs) {
            uint8_t b[4];
            std::memcpy(b, &f, 4);
            for (uint8_t x : b) h = (h ^ x) * 16777619u;
        }
        check("D2", "0xa057a8 table hash (FNV-1a)", {float(h == 0x44162c08u)}, {1.0f});
        DamageRecord r{0.1f, 0.2f, 0.9f, 0, 0.5f, 0, 0, 0.3f, 0, 0.8f, 1.5f};
        damageDirectional(r, dirs.data(), {0.6f, 0.3f, 0.74f}, 0.4f);
        check("D2", "0x51fd50 slots 0-4", {r[0], r[1], r[2], r[3], r[4]}, {0.47895202f, 0.49600002f, 0.93959194f, 0, 0.5f});
        check("D2", "0x51fd50 slots 5-9 / total", {r[5], r[6], r[7], r[8], r[9], r[10]}, {0, 0, 0.54f, 0.120000005f, 0.8f, 1.9f});
    } else {
        std::printf("D2     0x51fd50 skipped (no FUEL.exe: pass its path or set FUEL_EXE)\n");
    }
    struct CM {
        const char* id;
        Quat q;
        Vec3 pos, p, n, rel;
        float approach, tangential, depth;
        float impact, scrape;
        Vec3 dir;
    };
    auto quat = [](float yaw, float pitch) {
        const float cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        return Quat{cy * sp, sy * cp, -sy * sp, cy * cp};
    };
    const CM cms[] = {
        {"CM1", quat(0.3f, 0.05f), {10, 2, -4}, {11.0f, 1.8f, -3.5f}, {-0.9f, 0.2f, -0.38f}, {8.0f, -1.0f, 3.0f}, 7.5f, 5.0f, 0.05f, 339.57993f, 58.08f,
         {0.74750507f, -0.16831335f, 0.63820577f}},
        {"CM2", quat(-1.1f, 0.0f), {0, 1, 0}, {0.2f, 0.4f, 0.3f}, {0, 1, 0}, {6.0f, -2.0f, 9.0f}, 2.0f, 10.8f, 0.03f, 0.0f, 127.68001f, {0, 0, 0}},
        {"CM3", quat(0.7f, -0.2f), {5, 5, 5}, {5.5f, 5.6f, 4.2f}, {0.1f, -0.6f, 0.79f}, {-3.0f, 12.0f, -20.0f}, 18.0f, 9.0f, 0.1f, 731.5814f, 106.08f,
         {0.80301243f, 0.57744867f, -0.14739037f}},
    };
    float P[9] = {50.0f, 2.0f, 1.2f, 1.5f, 0.9f, 3.0f, 0.4f, 0.08f, 0.6f};  // P585..P593
    for (const CM& t : cms) {
        RigidBody b;
        b.S.q = t.q, b.S.x = t.pos;
        b.crashP = P;
        b.massP3 = 350;
        Contact c;
        c.p = t.p, c.n = t.n, c.approach = t.approach, c.tangential = t.tangential, c.depth = t.depth, c.a = &b;
        crashMeter(c, b, t.rel);
        check(t.id, "0x51ee90 +0x138c / +0x1390", {b.impactMeter, b.scrapeMeter}, {t.impact, t.scrape}, 1e-4f);
        check(t.id, "0x51ee90 +0x1060 (body frame)", {b.impactDirL.x, b.impactDirL.y, b.impactDirL.z}, {t.dir.x, t.dir.y, t.dir.z}, 1e-4f);
    }
    // P87: Vehicle_ImpactFx 0x51e9e0's crash-debris trigger, emulated (notes/p87-work/ifx_gen.py: synthetic records,
    // run to 0x51ed3d where EAX = the 0x52ee60 call count; skipped cases return before it). `want` = pieces (0 = none).
    struct IF {
        const char* id;
        bool driver = true;
        int state = 5, cls = 0;
        float timer = -0.5f;
        Vec3 rel{21, 0, 0}, n{0, 1, 0};
        bool wheel = false;
        float depth = 0.05f, bottom = 1e11f;
        Vec3 hub{0, 0.4f, 0};
        float slipY = 0;
        Quat node{0, 0, 0, 1}, q{0, 0, 0, 1};
        int want = 10;
    };
    const float s2 = 0.70710678f;
    std::vector<IF> ifs(18);
    for (size_t k = 0; k < ifs.size(); ++k) ifs[k].id = nullptr;
    ifs[0].id = "IFX1";
    ifs[1].id = "IFX2", ifs[1].driver = false, ifs[1].want = 0;
    ifs[2].id = "IFX3", ifs[2].state = 6, ifs[2].want = 0;
    ifs[3].id = "IFX4", ifs[3].state = 7, ifs[3].want = 0;
    ifs[4].id = "IFX5", ifs[4].state = 4;
    ifs[5].id = "IFX6", ifs[5].wheel = true, ifs[5].depth = 0.1f, ifs[5].bottom = 0.2f, ifs[5].want = 0;
    ifs[6].id = "IFX7", ifs[6].wheel = true, ifs[6].depth = 0.25f, ifs[6].bottom = 0.2f, ifs[6].want = 0;
    ifs[7].id = "IFX8", ifs[7].wheel = true, ifs[7].depth = 0.25f, ifs[7].bottom = 0.2f, ifs[7].slipY = -13;
    ifs[8].id = "IFX9", ifs[8].wheel = true, ifs[8].depth = 0.25f, ifs[8].bottom = 0.2f, ifs[8].hub = {0.4f, 0.1f, 0};
    ifs[9].id = "IFX10", ifs[9].cls = 3, ifs[9].want = 0;
    ifs[10].id = "IFX11", ifs[10].cls = 3, ifs[10].n = {1, 0, 0};
    ifs[11].id = "IFX12", ifs[11].cls = 1, ifs[11].node = {0, 0, s2, s2};
    ifs[12].id = "IFX13";
    ifs[13].id = "IFX14", ifs[13].timer = 0.5f, ifs[13].want = 0;
    ifs[14].id = "IFX15", ifs[14].rel = {4.9f, 0, 0}, ifs[14].want = 0;
    ifs[15].id = "IFX16", ifs[15].rel = {30, 40, 0}, ifs[15].want = 20;
    ifs[16].id = "IFX17", ifs[16].rel = {7.99f, 0, 0}, ifs[16].want = 3;
    ifs[17].id = "IFX18", ifs[17].wheel = true, ifs[17].depth = 0.25f, ifs[17].bottom = 0.2f, ifs[17].hub = {-0.4f, 0, 0}, ifs[17].q = {0, 0, s2, s2},
    ifs[17].want = 0;
    for (const IF& t : ifs) {
        RigidBody b;
        b.S.q = t.q;
        b.crashP = P;
        b.fxDriver = t.driver, b.fxRaceState = t.state, b.fxClass = t.cls, b.fxTimer = t.timer, b.fxWheelDrop = 12;
        b.fxProbeNormal = {0, 1, 0};
        // [port] the body's up stands for the node's (vt+0xb4(0) +0x10): the emulation turns only the node.
        if (t.cls == 1 || t.cls == 3) b.S.q = t.node;
        WheelRec w;
        w.hubW = t.hub, w.slipGather = {0, t.slipY, 0};
        Contact c;
        c.p = {0, 0, 0}, c.n = t.n, c.depth = t.depth, c.m.maxDepth = t.bottom, c.a = &b;
        if (t.wheel) c.wheel = &w;
        impactDebris(c, b, t.rel);
        const int got = b.debrisSerial ? b.debrisPieces : 0;
        check(t.id, "0x51e9e0 -> 0x52ee60 calls; FX timer", {float(got), b.fxTimer}, {float(t.want), t.want ? 1.0f : t.timer});
    }
}


// P46: 0x5259f0, the in-gear rpm target of the display/sound rpm, emulated (notes/p46-work/rpm_gen.py).
void rpmVectors() {
    struct RP {
        const char* id;
        int wheels;
        float f, speed;
        int gear;
        float load[4], fn[4], den[4], T[3], t, p46, p677, want;
    } rps[] = {
        {"RP1", 4, 5200, 14, 2, {1, 1, 1, 1}, {0.3f, 0.5f, 0.2f, 0.7f}, {1, 1, 1, 1}, {0.1f, 0.1f, 0.2f}, 5, 0, 0, 4697.20355f},
        {"RP2", 4, 6100, 3, 1, {1, 1, 0, 1}, {0.3f, 1.8f, 0.2f, 0.9f}, {1, 1, 1, 1}, {0.1f, 0.1f, 0.2f}, 0.25f, 1, 0, 6449.97883f},
        {"RP3", 2, 4000, 22, 3, {1, 1, 0, 0}, {0.2f, 0.4f, 0, 0}, {1, 1, 0, 0}, {0.1f, 0.1f, 0.2f}, 5, 0, 0, 4688.10553f},
        {"RP4", 4, 3000, 6, -1, {1, 1, 1, 1}, {0.1f, 0.1f, 0.1f, 0.1f}, {1, 1, 1, 1}, {0.1f, 0.1f, 0.2f}, 5, 0, 0, 1500},
        {"RP5", 4, 3000, 6, -1, {1, 1, 1, 1}, {0.1f, 0.1f, 0.1f, 0.1f}, {1, 1, 1, 1}, {0.1f, 0.1f, 0.2f}, 5, 0, 1, 1500},
        {"RP6", 4, 2500, 9, 4, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0.1f, 0.1f, 0.2f}, 5, 0, 0, 2852.87966f},
    };
    for (const RP& t : rps) {
        std::vector<std::pair<int, float>> p = {{46, t.p46}, {47, 1}, {49, 0.31f}, {50, 0.33f}, {324, 9000}, {325, 1500}, {350, 4.2f},
                                                {351, -3.1f}, {352, 3.0f}, {353, 2.1f}, {354, 1.6f}, {355, 1.25f}, {367, 0.85f}, {677, t.p677}};
        FuelVehicle v = synthetic(p, t.wheels);
        FuelVehicleTest::rpmState(v, t.f, t.speed, t.gear, t.load, t.fn, t.den, t.T, t.t);
        check(t.id, "0x5259f0 rpm target", {v.rpmTarget()}, {t.want}, 2e-5f);
    }
}

// BC: respawn breadcrumbs (P70b, physics/breadcrumbs.*): EmuRun of FUEL.exe's 0x5281c0 (safety), 0x5283b0 (record,
// with 0x5281c0 / 0x528690 / 0x421dc0 / 0x5309e0 stubbed by the values the port's callbacks give here), 0x528830
// (tick: 0x528780 and 0x5283b0 run for real, the same stubs) and 0x5288d0 (pick, with 0x527af0 / 0x52f6f0 for real),
// synthetic states (command files: scratchpad bc/gen.py, gensafe.py; notes p70-port-decoded.md). FUEL's crumb
// quaternion is a pure yaw (Quat_FromArc((0,0,1), horizontal dir)): compared as yaw = 2 atan2(qy, qw).
float qyaw(float qy, float qw) { return 2.0f * std::atan2(qy, qw); }

void breadcrumbVectors() {
    using fuel::fv::Breadcrumbs;
    using fuel::fv::CrumbVehicle;
    using fuel::fv::CrumbWorld;
    {  // 0x5281c0: base vehicle at (100, 50, 200), point (105, 60, 200); FUEL's AL per case
        struct S {
            const char* id;
            double pos[3], p[3];
            float up, vy, ny, fy, mw, water, h;
            bool loaded, blocked, tutorial, want;
        };
        const S cs[] = {
            {"BCS1", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, false, false, true},
            {"BCS2", {70000, 50, 200}, {70005, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, false, false, false},
            {"BCS3", {100, 50, 200}, {105, 60, 200}, -0.1f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, false, false, false},
            {"BCS4", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, false, false, false, false},
            {"BCS5", {100, 50, 200}, {105, 60, 200}, 0.9f, -31, 0.95f, 0.1f, 0, 0.05f, 45, true, false, false, false},
            {"BCS6", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.818f, 0.1f, 0, 0.05f, 45, true, false, false, false},
            {"BCS7", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.51f, 0, 0.05f, 45, true, false, false, false},
            {"BCS8", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, -0.71f, 0, 0.05f, 45, true, false, false, false},
            {"BCS9", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0.01f, 0.05f, 45, true, false, false, false},
            {"BCS10", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, true, false, false},
            {"BCS11", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.11f, 45, true, false, false, false},
            {"BCS12", {100, 50, 200}, {105, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 9, true, false, false, false},
            {"BCS13", {100, 50, 200}, {100.5, 60, 200}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 9, true, false, false, true},
            {"BCS14", {-50000, 50, -59000}, {-50005, 60, -59000}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, false, true, true},
            {"BCS15", {-53600, 50, -59000}, {-53605, 60, -59000}, 0.9f, 0, 0.95f, 0.1f, 0, 0.05f, 45, true, false, true, false},
            {"BCS16", {100, 50, 200}, {105, 60, 200}, 0.9f, -30, 0.819f, 0.5f, 0, 0.05f, 45, true, false, false, true},
        };
        for (const S& c : cs) {
            CrumbVehicle v;
            for (int k = 0; k < 3; ++k) v.pos[k] = c.pos[k];
            v.upY = c.up, v.velY = c.vy, v.probeNormalY = c.ny, v.fwd[1] = c.fy, v.maxWaterDepth = c.mw, v.allWheelsLoaded = c.loaded;
            CrumbWorld w;
            w.tutorialBounds = c.tutorial;
            w.blocked = [&c](const double*, float) { return c.blocked; };
            w.waterDepth = [&c](double, double) { return c.water; };
            w.baseHeight = [&c](double, double) { return c.h; };
            check(c.id, "0x5281c0 safe", {Breadcrumbs::safe(c.p, v, w) ? 1.0f : 0.0f}, {c.want ? 1.0f : 0.0f});
        }
    }
    {  // 0x5283b0: one record call; FUEL's list after it (count, then the new crumb's position, yaw, road flag)
        struct R {
            const char* id;
            double pos[3], p[3];
            float fwd[3];
            int road;  // 0 none (zero point), 1 the given point + dir
            double rp[2];
            float rdir[3];
            int race, gps;
            double tgt[2];      // race: T - p (the stub's out); gps: the point
            int prior;          // a crumb already in the list at `prev`
            double prev[3];
            bool unsafe;
            int count;          // FUEL's count after
            double cp[3];
            float qy, qw, flag;
        };
        const R cs[] = {
            {"BCR1", {1000, 50, 2000}, {1000, 50, 2000}, {0, 0, 1}, 1, {1003, 2004}, {3, 0, 4}, 1, 0, {30, -40}, 0, {}, false, 1, {1000, 50, 2000}, 0.94868326f, 0.31622776f, 1},
            {"BCR2", {1000, 50, 2000}, {1000, 50, 2000}, {0.6f, 0, 0.8f}, 1, {1003, 2004}, {-3, 0, -4}, 0, 0, {}, 0, {}, false, 1, {1000, 50, 2000}, 0.3162278f, 0.94868326f, 1},
            {"BCR3", {1000, 50, 2000}, {1000, 50, 2000}, {0, 0, 1}, 1, {1030, 2000}, {1, 0, 0}, 0, 1, {1500, 1500}, 0, {}, false, 1, {1000, 50, 2000}, 0.92387956f, 0.38268343f, 0},
            {"BCR4", {1000, 50, 2000}, {1004.8, 50, 1998.6}, {0.28f, 0, 0.96f}, 0, {}, {}, 0, 0, {}, 0, {}, false, 1, {1004.8, 50, 1998.6}, 0.16610327f, 0.98610836f, 0},
            {"BCR5", {1000, 50, 2000}, {1000, 50, 2000}, {0, 0, 1}, 0, {}, {}, 0, 0, {}, 1, {1003, 50, 2004}, false, 1, {1003, 50, 2004}, 0, 1, 0},
            {"BCR6", {1000, 50, 2000}, {1000, 50, 2000}, {0, 0, 1}, 0, {}, {}, 0, 0, {}, 1, {1009, 50, 2012}, true, 1, {1009, 50, 2012}, 0, 1, 0},
            {"BCR7", {1000, 50, 2000}, {1000, 50, 2000}, {-1, 0, 0}, 1, {1000, 2010}, {5, 0, 0}, 0, 0, {}, 1, {1009, 50, 2012}, false, 2, {1000, 50, 2000}, -0.70710677f, 0.70710677f, 1},
            {"BCR8", {10, 5, 12}, {10, 5, 12}, {0, 0, 1}, 0, {}, {}, 0, 0, {}, 0, {}, false, 1, {10, 5, 12}, 0, 1, 1},
        };
        for (const R& c : cs) {
            Breadcrumbs b;
            if (c.prior) b.list().push_back({c.prev[0], c.prev[1], c.prev[2], 0.0f, false});
            CrumbVehicle v;
            for (int k = 0; k < 3; ++k) v.pos[k] = c.pos[k], v.fwd[k] = c.fwd[k];
            if (c.unsafe) v.upY = -1;
            CrumbWorld w;
            if (c.road)
                w.roadCentreline = [&c](double, double, double& px, double& pz, float d[3]) {
                    px = c.rp[0], pz = c.rp[1];
                    for (int k = 0; k < 3; ++k) d[k] = c.rdir[k];
                    return true;
                };
            else
                w.roadCentreline = [](double, double, double&, double&, float*) { return false; };
            if (c.race) w.raceTarget = [&c](double& x, double& z) { return x = c.p[0] + c.tgt[0], z = c.p[2] + c.tgt[1], true; };
            if (c.gps) w.gpsTarget = [&c](double& x, double& z) { return x = c.tgt[0], z = c.tgt[1], true; };
            b.recordAt(c.p, v, w);
            const Breadcrumbs::Crumb& t = b.list().back();
            check(c.id, "0x5283b0 count / crumb x y z", {float(b.list().size()), float(t.x), float(t.y), float(t.z)},
                  {float(c.count), float(c.cp[0]), float(c.cp[1]), float(c.cp[2])});
            if (c.count > c.prior) check(c.id, "0x5283b0 crumb yaw / road flag", {t.yaw, t.onRoad ? 1.0f : 0.0f}, {qyaw(c.qy, c.qw), c.flag}, 1e-5f);
        }
    }
    {  // 0x528830: four ticks of one vehicle (identity orientation, lateral (1, 0, 0)); road point (2000, -2960)
        Breadcrumbs b;
        CrumbWorld w;
        w.roadCentreline = [](double, double, double& px, double& pz, float d[3]) { return px = 2000, pz = -2960, d[0] = 0, d[1] = 0, d[2] = 1, true; };
        w.baseHeight = [](double, double) { return 70.0f; };
        struct T {
            const char* id;
            double pos[3];
            float dt, timer;
            int count;
        };
        const T ts[] = {{"BCT1", {2000, 80, -3000}, 0.016f, 0, 1}, {"BCT2", {2000, 80, -2988}, 0.016f, 1, 2}, {"BCT3", {2000, 80, -2980}, 0.5f, 0.5f, 2},
                        {"BCT4", {2008.5, 80, -2988}, 0.6f, 1, 4}};
        for (const T& t : ts) {
            CrumbVehicle v;
            for (int k = 0; k < 3; ++k) v.pos[k] = t.pos[k];
            b.tick(t.dt, v, w);
            check(t.id, "0x528830 timer / count", {b.timer(), float(b.list().size())}, {t.timer, float(t.count)});
        }
        const double cp[4][3] = {{2000, 80, -3000}, {2000, 80, -2988}, {2013.5, 80, -2988}, {2003.5, 80, -2988}};
        const float q[4][2] = {{0, 1}, {0, 1}, {0.0249766f, 0.999688f}, {-0.0249766f, 0.999688f}};
        for (size_t k = 0; k < 4 && k < b.list().size(); ++k) {
            const Breadcrumbs::Crumb& c = b.list()[k];
            check("BCT4", "0x528780 crumb x z yaw flag", {float(c.x), float(c.z), c.yaw, c.onRoad ? 1.0f : 0.0f},
                  {float(cp[k][0]), float(cp[k][2]), qyaw(q[k][0], q[k][1]), 0}, 1e-4f);
        }
    }
    {  // 0x5288d0: list (100,10,0) 10 deg, (120,11,0) 20 deg road, (130,12,0) 30 deg, (136,13,0) 40 deg (tail)
        struct P {
            const char* id;
            float minDist;
            double x;
            int list;       // 0 empty, 1 single crumb, 4 the four (2: crumb 2 without its road flag)
            bool inPlace;
            double cp[3];
            float qy, qw;
            int count;
        };
        const P ps[] = {{"BCP1", 10, 150, 4, false, {136, 13, 0}, 0.34202015f, 0.9396926f, 4}, {"BCP2", 10, 138, 4, false, {120, 11, 0}, 0.17364818f, 0.9848077f, 2},
                        {"BCP3", 30, 138, 4, false, {120, 11, 0}, 0.17364818f, 0.9848077f, 2}, {"BCP4", 60, 138, 2, false, {100, 10, 0}, 0.087155744f, 0.9961947f, 1},
                        {"BCP5", 10, 104, 1, true, {104, 0, 0}, 0.3826834f, 0.9238795f, 1},    {"BCP6", 10, 104, 0, true, {104, 0, 0}, 0.3826834f, 0.9238795f, 0}};
        const float deg = 0.017453292f;
        for (const P& c : ps) {
            Breadcrumbs b;
            if (c.list == 1) b.list().push_back({100, 10, 0, 10 * deg, false});
            if (c.list >= 2) {
                b.list().push_back({100, 10, 0, 10 * deg, false});
                b.list().push_back({120, 11, 0, 20 * deg, c.list == 4});
                b.list().push_back({130, 12, 0, 30 * deg, false});
                b.list().push_back({136, 13, 0, 40 * deg, false});
            }
            CrumbVehicle v;
            v.pos[0] = c.x, v.yaw = 45 * deg;
            Breadcrumbs::Crumb out;
            const bool got = b.pick(c.minDist, v, out);
            if (!got) out.x = v.pos[0], out.y = v.pos[1], out.z = v.pos[2], out.yaw = v.yaw;
            check(c.id, "0x5288d0 in place / x y z / yaw / count", {got ? 0.0f : 1.0f, float(out.x), float(out.y), float(out.z), out.yaw, float(b.list().size())},
                  {c.inPlace ? 1.0f : 0.0f, float(c.cp[0]), float(c.cp[1]), float(c.cp[2]), qyaw(c.qy, c.qw), float(c.count)}, 1e-5f);
        }
    }
    {  // VehMove_GroundHeightProbe 0x529600 at (100, 50, 200): 0x639d30 stubbed (hit distance at result +0x20),
       // 0x40fe70 = the terrain (bc/probe.cmd); the port's ray gives the hit height y + 0.25 - distance.
        const float g1 = fuel::fv::groundHeightProbe(50.25f - 3, 40), g2 = fuel::fv::groundHeightProbe(std::nanf(""), 40);
        const float g3 = fuel::fv::groundHeightProbe(50.25f - 20, 40), g4 = fuel::fv::groundHeightProbe(50.25f, 50.25f);
        check("BCG1", "0x529600 ground height x4", {g1, g2, g3, g4}, {47.25f, 40.0f, 40.0f, 50.25f});
    }
    {  // 0x527f50's road query: Segment_Covers 0x6f0d10 and 0x6fd3a0 (0x6fb6f0 stubbed with the candidate list) on
       // synthetic segments S1 (0,0)-(10,0) w 3, S2 (0,5)-(10,5) w 4, S3 (2,1)-(9,7) w 2.5 (bc/road.cmd)
        auto seg = [](float ax, float az, float bx, float bz, float w) {
            fuel::RoadSampler::Segment s{};
            const float dx = bx - ax, dz = bz - az, L = std::sqrt(dx * dx + dz * dz);
            s.ax = ax, s.az = az, s.bx = bx, s.bz = bz, s.dx = dx / L, s.dz = dz / L, s.tx = s.dx / L, s.tz = s.dz / L;
            s.c30 = s.dz * ax - s.dx * az, s.c34 = s.tx * ax + s.tz * az, s.len = L, s.w = w;
            return s;
        };
        const fuel::RoadSampler::Segment S1 = seg(0, 0, 10, 0, 3), S2 = seg(0, 5, 10, 5, 4), S3 = seg(2, 1, 9, 7, 2.5f);
        struct C {
            const fuel::RoadSampler::Segment* s;
            float x, z;
            bool want;
        };
        const C cs[] = {{&S1, 5, 2, true},     {&S1, 5, 4, false},    {&S1, -2, 1, true}, {&S1, -3, 1, false}, {&S1, 12, 0, true},
                        {&S1, 13.5f, 0, false}, {&S1, 5, -3, true},    {&S1, 5, -3.01f, false}, {&S3, 4, 8.5f, false}, {&S3, 6, 2, true}};
        float got[10], want[10];
        for (int k = 0; k < 10; ++k) got[k] = fuel::RoadSampler::covers(*cs[k].s, cs[k].x, cs[k].z) ? 1.0f : 0.0f, want[k] = cs[k].want ? 1.0f : 0.0f;
        check("BCC1", "0x6f0d10 covers x10", {got[0], got[1], got[2], got[3], got[4], got[5], got[6], got[7], got[8], got[9]},
              {want[0], want[1], want[2], want[3], want[4], want[5], want[6], want[7], want[8], want[9]});
        struct N {
            const char* id;
            std::vector<const fuel::RoadSampler::Segment*> segs;
            float x, z, px, pz, dx, dz;
        };
        const N ns[] = {{"BCC2", {&S1, &S2}, 5, 2, 5, 0, 10, 0}, {"BCC3", {&S1}, 12, 1, 10, 0, 10, 0}, {"BCC4", {&S2, &S1}, 5, 2.5f, 5, 5, 10, 0},
                        {"BCC5", {&S3, &S2}, 6, 5, 6, 5, 10, 0}};
        for (const N& n : ns) {
            double px = 0, pz = 0;
            float dx = 0, dz = 0;
            fuel::RoadSampler::nearestCentreline(n.segs, n.x, n.z, px, pz, dx, dz);
            check(n.id, "0x6fd3a0 point x z / dir x z", {float(px), float(pz), dx, dz}, {n.px, n.pz, n.dx, n.dz});
        }
    }
}

// P81: the wheel hub offset 0x5d7a90 (fastcall ECX = vehicle, EDX = wheel, EAX = out), emulated with veh+0x6b4..0x6c8 =
// VehiclePP 57..62 = 0.11 .. 0.66 (scratchpad emu/hub.cmd): modes 0..5. 0x52def0 / 0x5c8620 add it to the proxy
// sphere centre (rec+0xe0 = sphere + offset - CoM); the port does not apply it (FuelVehicle::hubRest, notes p81 W2).
void hubOffsetVectors() {
    FuelVehicle v = synthetic({{57, 0.11f}, {58, 0.22f}, {59, 0.33f}, {60, 0.44f}, {61, 0.55f}, {62, 0.66f}});
    const float want[6][3] = {{0.11f, 0.22f, 0.33f}, {0.44f, 0.55f, 0.66f}, {-0.11f, 0.22f, 0.33f}, {-0.44f, 0.55f, 0.66f}, {-0.44f, 0.55f, 0.66f}, {-0.44f, 0.55f, 0.66f}};
    const char* ids[6] = {"HUB0", "HUB1", "HUB2", "HUB3", "HUB4", "HUB5"};
    for (int w = 0; w < 6; ++w) {
        const Vec3 o = v.hubOffset(w);
        check(ids[w], "0x5d7a90 hub offset mode w", {o.x, o.y, o.z}, {want[w][0], want[w][1], want[w][2]});
    }
}

// P81: 0x70da10, the world inverse inertia of set 2 (+0x7b0, the impulse pass), emulated with +0x5f0 = R (Ry(0.5) Rx(0.3)),
// +0x5b0 = R^T, +0x730 = I2^-1 local {{2, .1, 0}, {.1, 3, .2}, {0, .2, 4}} (scratchpad emu/inert.cmd): R I R^T. The port
// computes it eagerly every substep (RigidBody::updateWorldInertia) where FUEL caches it behind +0xec9.
void worldInertiaVector() {
    fuel::fv::RigidBody b;
    b.S.q = mul(axisAngle({0, 1, 0}, 0.5f), axisAngle({1, 0, 0}, 0.3f));
    b.I2inv.m[0][0] = 2, b.I2inv.m[0][1] = 0.1f, b.I2inv.m[0][2] = 0;
    b.I2inv.m[1][0] = 0.1f, b.I2inv.m[1][1] = 3, b.I2inv.m[1][2] = 0.2f;
    b.I2inv.m[2][0] = 0, b.I2inv.m[2][1] = 0.2f, b.I2inv.m[2][2] = 4;
    b.updateWorldInertia();
    const auto& m = b.I2invW.m;
    check("WI1", "0x70da10 world I2^-1 (+0x7b0)", {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]},
          {2.490448f, 0.027624048f, 0.8682072f, 0.027624028f, 2.9744039f, -0.14870146f, 0.8682073f, -0.14870146f, 3.5351484f}, 1e-5f);
}

// P81: Contacts_ReducePerWheel 0x71b7e0 (+ its sort 0x725a80), emulated on synthetic contact lists (body +0xd98 count,
// +0xd9c records 0x18c: n +0x10, depth +0xd8, wheel +0xe4, c +0xf0; scratchpad emu/red.cmd). RED1: deepest per wheel
// wins; RED2: a contact zeroed by the deepest still zeroes later ones, the last contact is never a candidate; RED3:
// c = 1 is not a candidate, non-wheel penalty contacts (wheel 0) are reduced among themselves.
void reductionVectors() {
    using fuel::fv::Contact;
    using fuel::fv::WheelRec;
    struct In {
        int wheel;
        float c, depth;
        Vec3 n;
    };
    WheelRec recs[4];
    auto run = [&](const char* id, std::vector<In> in, std::initializer_list<float> want) {
        std::vector<Contact> cs(in.size());
        for (size_t i = 0; i < in.size(); ++i) {
            cs[i].wheel = in[i].wheel ? &recs[in[i].wheel] : nullptr;
            cs[i].m.c = in[i].c, cs[i].depth = in[i].depth, cs[i].n = in[i].n;
        }
        fuel::fv::reducePerWheel(cs);
        std::vector<float> got;
        for (const Contact& c : cs) got.push_back(c.depth);
        std::initializer_list<float> g = {got[0], got[1], got[2], got[3], got[4], got.size() > 5 ? got[5] : 0.0f};
        check(id, "0x71b7e0 depths after reduction", g, want);
    };
    run("RED1", {{1, 0.05f, 0.05f, {0, 1, 0}}, {1, 0.05f, 0.08f, {0, 0.9f, 0.436f}}, {2, 0.05f, 0.03f, {0, 1, 0}}, {1, 0.05f, 0.02f, {0, -0.2f, 0.98f}},
                 {0, 0, 0.1f, {0, 1, 0}}, {2, 0.05f, 0.06f, {0, 1, 0}}},
        {0, 0.08f, 0.03f, 0, 0.1f, 0.06f});
    run("RED2", {{1, 0.05f, 0.1f, {1, 0, 0}}, {1, 0.05f, 0.05f, {0.6f, 0.8f, 0}}, {1, 0.05f, 0.02f, {-0.6f, 0.8f, 0}}, {3, 0.05f, 0.01f, {0, 1, 0}},
                 {1, 0.05f, 0.3f, {1, 0, 0}}},
        {0.1f, 0, 0, 0.01f, 0.3f, 0});
    run("RED3", {{1, 1, 0.2f, {0, 1, 0}}, {1, 0.05f, 0.1f, {0, 1, 0}}, {0, 0.05f, 0.07f, {0, 1, 0}}, {0, 0.05f, 0.04f, {0, 0.8f, 0.6f}},
                 {1, 0.05f, 0.05f, {0, 1, 0}}, {1, 0.05f, 0.01f, {0, 1, 0}}},
        {0.2f, 0.1f, 0.07f, 0, 0, 0.01f});
}


// P85: the visual damage, emulated on FUEL.exe (scratchpad emu85/sph.cmd, md.cmd): 0x5c7760's slot sphere (callees
// 0x5c6560 / 0x4297d0 / 0x712410 / 0x5c79a0 stubbed; bounds (-0.6, 0.05, -1.1)..(0.62, 1.2, 0.9), identity body at the
// origin) and 0x7cb550 on a synthetic 3-point, 5-vertex mesh (renderer VB calls stubbed).
void visualDamageVectors(const char* exe) {
    const fuel::damage::Tables t = exe ? fuel::damage::loadTables(std::string(exe)) : fuel::damage::Tables{};
    if (!t.ok) {
        std::printf("SPH    0x5c7760 skipped (no FUEL.exe: pass its path or set FUEL_EXE)\n");
    } else {
        const std::array<float, 3> mn{-0.6f, 0.05f, -1.1f}, mx{0.62f, 1.2f, 0.9f};
        struct V { const char* id; int slot; float v; float want[4]; };
        const V vs[] = {{"SPH1", 0, 0.5f, {0.44127002f, 0.625f, 0.607f, 0.4025f}},
                        {"SPH2", 3, 1.0f, {-0.6f, 0.625f, -0.100000024f, 1.0f}},
                        {"SPH3", 8, 0.25f, {0.00999999f, 1.2f, -0.100000024f, 0.20125f}},
                        {"SPH4", 9, 0.7f, {0.00999999f, 0.625f, -0.100000024f, 0.5635f}},
                        {"SPH5", 5, 0.35f, {0.00999999f, 0.625f, -1.1f, 0.21350001f}}};
        for (const V& v : vs) {
            const fuel::damage::Sphere s = fuel::damage::slotSphere(t, v.slot, mn, mx, v.v);
            check(v.id, "0x5c7760 slot sphere (centre, radius)", {s.c[0], s.c[1], s.c[2], s.r}, {v.want[0], v.want[1], v.want[2], v.want[3]}, 1e-5f);
        }
    }
    fuel::Mesh m;
    m.collisionVerts = {-512, 0, 0, -1000, 500, 0, 700, 0, 300};
    auto node = [&](std::array<float, 3> lo, std::array<float, 3> hi, uint16_t c1, uint16_t c2, uint16_t start, uint16_t n) {
        uint8_t rec[0x20]{};
        std::memcpy(rec, lo.data(), 12);
        std::memcpy(rec + 0x10, hi.data(), 12);
        const uint16_t ch[2] = {c1, c2}, rg[2] = {start, n};
        std::memcpy(rec + 0xc, ch, 4);
        std::memcpy(rec + 0x1c, rg, 4);
        m.morphNodes.insert(m.morphNodes.end(), rec, rec + 0x20);
    };
    node({-1, -1, -1}, {1, 1, 1}, 2, 3, 0, 0);
    node({-1, -1, -1}, {0, 1, 1}, 0, 0, 0, 2);
    node({0, -1, -1}, {1, 1, 1}, 0, 0, 2, 1);
    m.morphPairs = {0, 0, 1, 4, 2, 7};
    m.morphGroups = {0, 2, 0, 3, 1, 1, 1, 2, 2, 2, 4};
    fuel::Mesh::MorphTarget tg;
    tg.vertexDelta.assign(5, 0);
    tg.deltas = {409, -82, 41, 0, -100, 200, 300, 1, 4095, -4096, 7, 2};
    m.morphs.push_back(tg);
    struct M { const char* id; float c[3], r, ret; std::vector<float> vb; };
    const float a0 = 0.099853516f, a1 = -0.020019531f, a2 = 0.010009766f, b0 = -0.024414062f, b1 = 0.048828125f, b2 = 0.07324219f,
                c0 = 0.99975586f, c1 = -1.0f, c2 = 0.0017089844f;
    const M ms[] = {{"MD1", {-0.4f, 0.1f, 0}, 0.3f, 1, {a0, a1, a2, 0, 0, 0, 0, 0, 0, a0, a1, a2, 0, 0, 0}},
                    {"MD2", {0.2f, 0, 0.1f}, 0.6f, 1, {0, 0, 0, 0, 0, 0, c0, c1, c2, 0, 0, 0, c0, c1, c2}},
                    {"MD3", {5, 5, 5}, 0.5f, 0, {}},
                    {"MD4", {0, 0, 0}, 3.0f, 1, {a0, a1, a2, b0, b1, b2, c0, c1, c2, a0, a1, a2, c0, c1, c2}}};
    for (const M& t2 : ms) {
        std::vector<float> vb;
        const float ret = fuel::damage::morphDamage(m, {{t2.c[0], t2.c[1], t2.c[2]}, t2.r}, vb);
        check(t2.id, "0x7cb550 result / VB created", {ret, float(vb.size())}, {t2.ret, float(t2.vb.size())});
        bool same = vb.size() == t2.vb.size();
        for (size_t k = 0; same && k < vb.size(); ++k) same = std::fabs(vb[k] - t2.vb[k]) < 1e-7f;
        check(t2.id, "0x7cb550 morph VB (15 floats)", {float(same)}, {1.0f});
    }
    fuel::Mesh none;
    std::vector<float> vb;
    check("MD5", "0x7cb550 no morph target: -1 [0xa05224]", {fuel::damage::morphDamage(none, {{0, 0, 0}, 1}, vb)}, {-1.0f});
    const float a[10] = {0, 0.5f, 0, 0, 0, 0, 0, 0, 0, 0}, b[10] = {0, 0.51f, 0, 0, 0, 0, 0, 0, 0, 0}, c[10] = {0, 0.5125f, 0, 0, 0, 0, 0, 0, 0, 0};
    check("MD6", "0x5c6560 changed (0.01 / 0.0125)", {float(fuel::damage::slotsChanged(a, b)), float(fuel::damage::slotsChanged(a, c))}, {0, 1});
}

// ---- PK22: the vehicle visual links (core/vehicle_links.hpp), emulated on FUEL.exe (scratchpad emu22/v.cmd):
// 0x75fbc0 (OBB clamp), 0x5cc370 (body velocity at a point), 0x75fd20 (sphere / box distance), 0x40f8b0 (axis angle);
// the lamp rule (0x5cc780), the dirt block (0x529ce0) and the V:DYN pose at rest checked against the decode.
void linkVectors() {
    namespace vl = fuel::vlinks;
    const std::array<float, 16> box{0.8f, 0, 0.6f, 1, 0, 1, 0, 2, -0.6f, 0, 0.8f, 3, 0.5f, 0.25f, 1, 1.2f};
    struct C { const char* id; vl::V3 p; float ret; vl::V3 out; };
    const C cs[] = {{"OBB1", {1.1f, 2.1f, 3.0f}, 0, {0, 0, 0}}, {"OBB2", {3, 2, 3}, 1, {2.0f, 2.0f, 2.5f}},
                    {"OBB3", {1, 2.5f, 1}, 1, {1.2f, 2.25f, 1.9f}}, {"OBB4", {0, 0, 0}, 1, {1.2f, 1.75f, 1.9f}}};
    for (const C& c : cs) {
        vl::V3 o{0, 0, 0};
        const bool r = vl::obbClamp(box, c.p, o);
        check(c.id, "0x75fbc0 clamped / point", {float(r), o[0], o[1], o[2]}, {c.ret, c.out[0], c.out[1], c.out[2]}, 1e-5f);
    }
    vl::Motion m1;
    m1.pos = {1, 2, 3}, m1.vel = {4, -1, 0.5f}, m1.angVel = {0.3f, -0.7f, 1.1f};
    const vl::V3 p1 = vl::pointVelocity(m1, {1.5f, 2.5f, 2});
    check("PV1", "0x5cc370 point velocity", {p1[0], p1[1], p1[2]}, {3.85f, -1.85f, 0.0f}, 1e-5f);
    vl::Motion m2;
    m2.pos = {-100, 5, 40}, m2.vel = {0, 0, 10}, m2.angVel = {0, 2, 0};
    const vl::V3 p2 = vl::pointVelocity(m2, {-99, 5.5f, 41});
    check("PV2", "0x5cc370 point velocity", {p2[0], p2[1], p2[2]}, {-2.0f, 0.0f, 12.0f}, 1e-5f);
    float d = 0;
    check("SB1", "0x75fd20 inside: penetration + r", {float(vl::sphereBox(box, {1.2f, 2.1f, 3.1f}, 0.4f, d)), d}, {1, 0.5500001f}, 1e-5f);
    check("SB2", "0x75fd20 outside: distance", {float(vl::sphereBox(box, {2, 2, 3}, 0.8f, d)), d}, {1, 0.3f}, 1e-5f);
    check("SB3", "0x75fd20 no contact", {float(vl::sphereBox(box, {5, 5, 5}, 0.5f, d))}, {0});
    const vl::Q4 a1 = vl::axisAngle({0, 1, 0}, 0.7f), a2 = vl::axisAngle({1, 0, 0}, -2.5f);
    check("AA1", "0x40f8b0 axis angle", {a1[0], a1[1], a1[2], a1[3]}, {0, 0.3428978f, 0, 0.9393727f}, 1e-6f);
    check("AA2", "0x40f8b0 axis angle", {a2[0], a2[1], a2[2], a2[3]}, {-0.9489846f, 0, 0, 0.31532237f}, 1e-6f);
    // 0x5cc780 lamps: reverse (gear -1) BACK, brake value > 0.1 STOP, headlights POSITION / HEAD with I = 0.9.
    vl::LampInputs li;
    li.gear = -1, li.brakeValue = 0.1f, li.headlights = false;
    auto la = vl::lampAlphas(li);
    check("LAMP1", "0x5cc780 reverse, brake 0.1", {la[0], la[1], la[2], la[3]}, {1, 0, 0, 0});
    li.gear = 2, li.brakeValue = 0.11f, li.headlights = true;
    la = vl::lampAlphas(li);
    check("LAMP2", "0x5cc780 brake 0.11, headlights", {la[0], la[1], la[2], la[3]}, {0, 0.9f, 0.9f, 0.9f});
    li.physics = false, li.gear = -1;
    la = vl::lampAlphas(li);
    check("LAMP3", "0x5cc780 no physics body: no BACK", {la[0]}, {0});
    // 0x529ce0 dirt: P594 5, P595 0.01, P596 0.002, P597 0.5, P598 0.1.
    vl::DirtParams pp;
    pp.p594 = 5, pp.p595 = 0.01f, pp.p596 = 0.002f, pp.p597 = 0.5f, pp.p598 = 0.1f;
    float s1[10]{}, tot = 0;
    vl::stepDirt(s1, tot, 0.5f, true, 0.8f, 25, 0, 0, pp);  // + 0.8 x 0.01 x 20 x 0.5 = 0.08, - 0.001 (above 0.05)
    check("DIRT1", "0x529ce0 dirt rise + dry wash", {s1[0], s1[9], tot}, {0.079f, 0.079f, 0.08f}, 1e-6f);
    vl::stepDirt(s1, tot, 0.5f, true, 0.8f, 25, 0.2f, 0, pp);  // water: - 0.25 down to 0.05
    check("DIRT2", "0x529ce0 water wash to 0.05", {s1[0]}, {0.05f}, 1e-6f);
    float s3[10] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.04f};
    vl::stepDirt(s3, tot, 1.0f, false, 0, 0, 0, 0.5f, pp);  // rain 0.5: - 0.05, airborne: nothing else; 0.04 stays
    check("DIRT3", "0x529ce0 rain wash, slot under the floor kept", {s3[0], s3[9], vl::dirtMean(s3)}, {0.45f, 0.04f, 0.409f}, 1e-6f);
    // 0x6a0200 V:DYN at rest: the rotation is the bone's; a point moved +x about a Y-only record turns about Y.
    vl::DynRecord r;
    r.autostart = true, r.axe = {0, 1, 0};
    vl::SpringPoint sp;
    sp.rest = {0, 0, -1};
    r.points.push_back(sp);
    vl::reset(r);
    vl::DynPose ps = vl::pose(r, {0, 0, 0}, {0, 0, 0, 1});
    check("DYN1", "0x6a0200 V:DYN at rest", {float(ps.state), ps.rot[0], ps.rot[1], ps.rot[2], ps.rot[3]}, {1, 0, 0, 0, 1}, 1e-6f);
    r.points[0].pos = {-1, 0, -1};  // atan2(x, z): rest -pi, now -3 pi / 4: +pi/4 about Y
    ps = vl::pose(r, {0, 0, 0}, {0, 0, 0, 1});
    const float sgn = ps.rot[3] < 0 ? -1.0f : 1.0f;  // q and -q: the same rotation (FUEL's atan2 wrap gives -q here)
    check("DYN2", "0x6a0200 V:DYN swing about Y (+pi/4)", {sgn * ps.rot[1], sgn * ps.rot[3]}, {0.38268343f, 0.9238795f}, 1e-5f);
    vl::addDamage(r, 1, 0.9f, 0.8f);  // 1.7 > 1.6: loose, +0x34 = 0
    check("DYN3", "0x6a2480 break loose above 1.6", {float(r.loose), r.damage, r.damageTotal}, {1, 0, 1.7f}, 1e-6f);
}

// [nitro] The port's nitrous (physics/nitro.hpp, ai_nitro.hpp): the tank (drain, idle recharge, start / dry rules), the
// bonus fills (air, powerslide, near miss, overtakes), the boost's numbers and its push on a vehicle, the AI's choice.
// Hand vectors from the stated rules (no FUEL code: FUEL has no nitrous); the other vectors run with the nitro idle.
void nitroVectors() {
    namespace nz = fuel::nitro;
    // Fixed numbers (the defaults may be retuned): torque gain 0.5, push 3 m/s^2, lift 0.15, FUEL's P92 blend on, power 1;
    // the tank: 3 s, 0.12 s of boost a second (25 s empty -> full) after 1.5 s, minStart 0.18 s, ramps 0.12 / 0.25 s; the
    // fills in seconds of boost: air 0.12 at 0.6 s + 0.3 a second (0.9 at most), slide 0.3 a second after 0.4 s,
    // overtake 0.45, near miss 0.24. The vehicle reads the globals.
    nz::Params& G = nz::params();
    const nz::Params saved = G;
    G.torqueGain = 0.5f, G.thrust = 3.0f, G.topSpeedLift = 0.15f, G.dragRelease = 1.0f, G.power = 1.0f;
    G.tankSeconds = 3.0f, G.rechargeRate = 0.12f, G.rechargeDelay = 1.5f, G.minStart = 0.18f, G.rampUp = 0.12f, G.rampDown = 0.25f;
    G.airMin = 0.6f, G.airBase = 0.12f, G.airRate = 0.3f, G.airMax = 0.9f, G.slideHold = 0.4f, G.slideRate = 0.3f;
    G.slideMinDeg = 15, G.slideMaxDeg = 70, G.slideMinSpeed = 11, G.overtake = 0.45f, G.overtakeHold = 1.0f;
    G.nearMiss = 0.24f, G.nearMissGap = 1.2f, G.nearMissSpeed = 6.0f, G.nearMissCooldown = 2.0f;
    const nz::Params P = G;
    const float dt = 1.0f / 60;
    check("NIT1", "torque x (amount 0 / 0.5 / 1)", {nz::torqueMultiplier(0, P), nz::torqueMultiplier(0.5f, P), nz::torqueMultiplier(1, P)}, {1, 1.25f, 1.5f});
    check("NIT2", "0x520fd0 drag blend P91 -> P92", {nz::wheelDragBlend(0, P), nz::wheelDragBlend(0.4f, P), nz::wheelDragBlend(1.5f, P)}, {0, 0.4f, 1});
    check("NIT3", "push fade (top 40 m/s, vmax 46)",
          {nz::thrustFade(30, 40, P), nz::thrustFade(39.1f, 40, P), nz::thrustFade(44, 40, P), nz::thrustFade(46, 40, P), nz::thrustFade(50, 40, P)},
          {1, 1, 2.0f / 6.9f, 0, 0}, 1e-4f);
    check("NIT4", "push 3 m/s^2 x half the wheels", {nz::thrustAccel(1, 30, 40, 0.5f, P), nz::thrustAccel(0, 30, 40, 1, P)}, {1.5f, 0});
    check("NIT5", "rev range stretch / P619 -> m/s", {nz::rpmStretch(0, P), nz::rpmStretch(1, P), nz::topSpeedMs(188)}, {1, 1.15f, 188 * 0.86956525f * 0.2777778f});
    {  // the vehicle: FUEL's torque curve untouched at amount 0, stretched by 1.15 at 1; the push along the heading
        std::vector<std::pair<int, float>> p = {{324, 8000}, {327, 1000}};
        for (int i = 0; i <= 10; ++i) p.push_back({338 + i, 0.1f * float(i)});
        FuelVehicle v = synthetic(p);
        v.setNitro(0, 40);
        const float t0 = v.engineTorque(4400);
        v.setNitro(1, 40);
        check("NIT6", "0x524c70 curve: amount 0 = FUEL, 1 stretched", {t0, v.engineTorque(8000)}, {550.000042f, 1000.0f * 8000.0f / 9200.0f}, 1e-5f);
        v.setNitro(0, 40);
        const float m = FuelVehicleTest::mass(v);
        const Vec3 f1 = FuelVehicleTest::nitroPush(v, 1, 40, 10, 1, 4), fh = FuelVehicleTest::nitroPush(v, 1, 40, 10, 1, 1);
        const Vec3 fr = FuelVehicleTest::nitroPush(v, 1, 40, 10, -1, 4), fa = FuelVehicleTest::nitroPush(v, 1, 40, 10, 1, 0);
        check("NIT7", "push = m x 3 along +z (4 / 1 wheel loaded)", {f1.x, f1.y, f1.z / m, fh.z / m}, {0, 0, 3, 3}, 1e-5f);
        check("NIT8", "no push in reverse or in the air", {fr.z, fa.z}, {0, 0});
        // the torque gain (0.5 here) with every wheel gripping; spinning: cut fully (tractionCut 1) with a 0.05 s rate, back
        // over 0.3 s; amount 0: exactly 1
        G.tractionCut = 1.0f;
        p.push_back({46, 1}), p.push_back({47, 1});  // four driven wheels
        FuelVehicle d = synthetic(p);
        const float grip = FuelVehicleTest::nitroTorque(d, 1, false, 10, dt);
        const float spin1 = FuelVehicleTest::nitroTorque(d, 1, true, 1, dt);  // one step: grip 1 -> 1 - (1/60) / 0.05
        const float spin = FuelVehicleTest::nitroTorque(d, 1, true, 30, dt);
        const float back = FuelVehicleTest::nitroTorque(d, 1, false, 18, dt);  // 0.3 s gripping again
        const float off = FuelVehicleTest::nitroTorque(d, 0, true, 5, dt);
        check("NIT9", "boost torque traction limiter: grip / 1 step spinning / spinning / 0.3 s back / amount 0",
              {grip, spin1, spin, back, off}, {1.5f, 1.0f + 0.5f * (1.0f - (1.0f / 60) / 0.05f), 1.0f, 1.0f + 0.5f * (1.0f - std::pow(1.0f - (1.0f / 60) / 0.3f, 18.0f)), 1.0f},
              1e-4f);
    }
    {  // the tank
        nz::Tank t;
        t.reset(P);
        for (int i = 0; i < 90; ++i) t.step(dt, true, true, true, P);  // 1.5 s held on the ground
        check("TANK1", "1.5 s held: half, full push, 1 start", {t.level, t.amount, float(t.active), float(t.starts)}, {0.5f, 1, 1, 1}, 1e-4f);
        for (int i = 0; i < 100; ++i) t.step(dt, true, true, true, P);  // run dry while held
        check("TANK2", "dry: stops, flash, waits for a release", {t.level, float(t.active), float(t.needRelease), float(t.emptyFlash > 0)}, {0, 0, 1, 1});
        for (int i = 0; i < 60; ++i) t.step(dt, true, true, true, P);  // still held: nothing, the push ramps out (0.25 s)
        check("TANK3", "held after dry: no restart, push 0", {float(t.active), t.amount, float(t.starts)}, {0, 0, 1});
        for (int i = 0; i < 840; ++i) t.step(dt, false, true, true, P);  // released
        // idle since the tank ran dry at 3 s (step 180 of 190): 10 + 60 + 840 steps; recharge after 1.5 s at 0.12 s / s
        check("TANK4", "idle recharge: 15.17 s since dry -> 0.547", {t.level}, {(910.0f / 60.0f - 1.5f) * 0.12f / 3.0f}, 2e-3f);
        nz::Tank a;
        a.reset(P);
        for (int i = 0; i < 60; ++i) a.step(dt, true, false, true, P);  // held in the air
        check("TANK5", "airborne: engaged, no drain, no push", {float(a.active), a.level, a.amount}, {1, 1, 0});
        nz::Tank e;
        e.reset(P);
        e.level = 0.05f;  // 0.15 s < minStart 0.18 s
        e.step(dt, true, true, true, P);
        check("TANK6", "below minStart: no start, the flash", {float(e.active), e.emptyFlash}, {0, 0.6f - dt}, 1e-4f);
        nz::Tank c;
        c.reset(P);
        c.step(dt, true, true, false, P);
        check("TANK7", "not allowed (countdown, wrecked): no start", {float(c.active), c.level}, {0, 1});
        c.level = 0.9f;
        const float got = c.add(0.9f, nz::KindAir, P);  // 0.9 s into a tank 0.3 s short of full
        check("TANK8", "a bonus caps at a full tank (seconds)", {c.level, got, c.earned[nz::KindAir], c.bonusGlow}, {1, 0.3f, 0.9f, 0.8f}, 1e-5f);
        nz::Params big = P;
        big.tankSeconds = 4.8f;  // the tank stage 3: the same bonus seconds fill a smaller fraction
        nz::Tank b4;
        b4.reset(big);
        b4.level = 0.5f;
        b4.add(0.96f, nz::KindSlide, big);
        check("TANK9", "4.8 s tank: 0.96 s bonus = 0.2 of it; recharge 0.12 s/s", {b4.level, big.fraction(big.rechargeRate)}, {0.7f, 0.025f}, 1e-5f);
    }
    {  // the fills
        check("NAIR1", "air fill: 0.5 / 0.6 / 1.6 / 5 s", {nz::airBonus(0.5f, P), nz::airBonus(0.6f, P), nz::airBonus(1.6f, P), nz::airBonus(5, P)},
              {0, 0.12f, 0.42f, 0.9f}, 1e-5f);
        nz::Detector d;
        float sum = 0;
        for (int i = 1; i <= 96; ++i) sum += d.air(float(i) * dt, true, false, false, P);  // 1.6 s up
        sum += d.air(0, true, false, false, P);                                            // landed upright
        nz::Detector r;
        float res = 0;
        for (int i = 1; i <= 96; ++i) res += r.air(float(i) * dt, true, false, false, P);
        res += r.air(0, true, false, true, P);  // a respawn ends it
        nz::Detector u;
        float ups = 0;
        for (int i = 1; i <= 96; ++i) ups += u.air(float(i) * dt, true, false, false, P);
        ups += u.air(0, false, false, false, P);  // landed on its roof
        check("NAIR2", "landing 1.6 s upright / respawned / on its roof", {sum, res, ups}, {0.42f, 0, 0}, 1e-3f);
        check("SLIDE1", "slip angle 45 deg; vertical speed ignored",
              {nz::slipAngleDeg({1, 0, 1}, {0, 0, 1}, {0, 1, 0}), nz::slipAngleDeg({0, 5, 1}, {0, 0, 1}, {0, 1, 0})}, {45, 0}, 1e-4f);
        nz::Detector s1, s2, s3;
        float a = 0, b = 0, c = 0;
        for (int i = 0; i < 84; ++i) {  // 1.4 s
            a += s1.powerslide(dt, 30, 15, true, false, P);
            b += s2.powerslide(dt, 30, 8, true, false, P);
            c += s3.powerslide(dt, 80, 15, true, false, P);
        }
        check("SLIDE2", "1.4 s at 30 deg 15 m/s / too slow / spun", {a, b, c}, {0.3f, 0, 0}, 1e-3f);
        auto pass = [&](nz::NearMissPair& pr, float lat, float closing) {  // j from 6 m ahead to 6 m behind at `closing`
            float got = 0;
            for (float lon = 6; lon >= -6; lon -= closing * dt) got += pr.step(dt, lat, lon, 1.0f, 2.0f, closing, P);
            return got;
        };
        nz::NearMissPair p1, p2, p3;
        const float clean = pass(p1, 1.8f, 10), again = pass(p1, 1.8f, 10), touch = pass(p2, 0.95f, 10), slow = pass(p3, 1.8f, 3);
        check("NEAR1", "near miss 0.8 m / at once again / touching / slow", {clean, again, touch, slow}, {0.24f, 0, 0, 0});
        nz::Overtakes o;
        std::vector<float> g;
        const std::vector<bool> racing = {true, true};
        o.step(dt, {3, 2}, racing, g, P);
        float won = 0, lost = 0;
        for (int i = 0; i < 70; ++i) {
            o.step(dt, {2, 3}, racing, g, P);
            won += g[0], lost += g[1];
        }
        float flick = 0;
        for (int i = 0; i < 40; ++i) o.step(dt, {3, 2}, racing, g, P), flick += g[1];
        for (int i = 0; i < 40; ++i) o.step(dt, {2, 3}, racing, g, P), flick += g[1] + g[0];
        check("OVT1", "a place held 1 s / the one passed / a 0.67 s swap", {won, lost, flick}, {0.45f, 0, 0}, 1e-5f);
    }
    {  // the AI's button
        fuel::RaceLine line;
        line.build({0, 0, 0}, {{0, 0, 200}, {200, 0, 200}});  // 200 m north, then east
        check("NAI1", "path turn: straight / the 90 deg corner ahead",
              {fuel::nitro::pathTurn(line, 20, 0, 100), fuel::nitro::pathTurn(line, 150, 0, 100)}, {0, 90}, 1e-3f);
        nz::AiView v;
        v.level = 1, v.speed = 25, v.trail = 100;
        nz::AiNitro n1, n2, n3, n4;
        const bool straight = n1.decide(dt, v, P);
        nz::AiView tight = v;
        tight.turnNear = 40;
        const bool corner = n2.decide(dt, tight, P);
        nz::AiView air = v;
        air.airborne = true;
        const bool flying = n3.decide(dt, air, P);
        nz::AiView lead = v;
        lead.trail = -150, lead.level = 0.4f;
        const bool reserve = n4.decide(dt, lead, P);
        check("NAI2", "AI: straight / tight bend / airborne / leader's reserve", {float(straight), float(corner), float(flying), float(reserve)}, {1, 0, 0, 0});
    }
    G = saved;
}


// [upgrades U1] The port's vehicle upgrades (core/upgrades.hpp; notes upgrades-plan.md 2.2 / 2.3 / 3.2, upg1.md) and the
// traction governor's step (truck-power). Hand vectors from the plan's stated tables (FUEL has no upgrades). UPGB: every
// VehiclePP block of the player's data (FUEL_DATA, or three folders above the FUEL.exe given) with its physics proxy:
// stock stages, and any stages with `upgrades = off`, leave the whole setup bit for bit.
void upgradeVectors(const char* exe) {
    namespace u = fuel::upg;
    const bool wasOn = u::enabled();
    u::setEnabled(true);
    {
        u::Stages a, b, c, d;
        const bool ok1 = u::parseStages("3,2,1,0,1,2", a), ok2 = u::parseStages("max", b), bad = !u::parseStages("1,4", c), bad2 = !u::parseStages("1,1,1,1,1,1,1", d);
        u::Stages e;
        u::parseStages("2", e);
        check("UPG1", "parse e,s,t,w,np,nt / max / bad", {float(ok1), float(a[u::Engine]), float(a[u::Suspension]), float(a[u::Tires]), float(a[u::Weight]),
              float(a[u::NitroPower]), float(a[u::NitroTank]), float(ok2 && b[u::NitroTank] == 3 && b[u::Engine] == 3), float(bad), float(bad2),
              float(e[u::Engine] == 2 && e[u::Suspension] == 0), float(u::formatStages(a) == "3,2,1,0,1,2")}, {1, 3, 2, 1, 0, 1, 2, 1, 1, 1, 1, 1});
    }
    check("UPG2", "tables: engine / damp (FUEL's) / travel / roll / bump", {u::engineGain(0), u::engineGain(1), u::engineGain(2), u::engineGain(3), u::dampScale(0),
          u::dampScale(3), u::travelScale(1), u::travelScale(3), u::antiRollScale(3), u::bumpScale(3)}, {1, 1.25f, 1.5f, 1.8f, 1, 1, 1.33f, 2.0f, 1.5f, 1.3f});
    check("UPG3", "tables: tyre / mass / nitro power / tank / rate", {u::tyreScale(3), u::effectiveMass(1), u::effectiveMass(3), u::nitroPowerScale(3),
          u::nitroTankScale(1) * 3.0f, u::nitroTankScale(3) * 3.0f, u::nitroRechargeScale(3)}, {1.12f, 0.96f, 0.88f, 1.45f, 3.6f, 4.8f, 1.3f});
    check("UPG4", "damping compliance x rate", {u::dampCompliance(0.5f, 2.0f), u::dampCompliance(0.8f, 1.33f), u::dampCompliance(1.0f, 2.0f),
          u::dampCompliance(1.3f, 2.0f), u::dampCompliance(0.0f, 2.0f), u::dampCompliance(0.37f, 1.0f)},
          {1.0f / 3.0f, 1.0f / (1.33f * 0.25f + 1.0f), 1.0f, 1.3f, 0.0f, 0.37f});
    {
        u::Stages e3w3, e3, w1, st;
        e3w3[u::Engine] = 3, e3w3[u::Weight] = 3, e3[u::Engine] = 3, w1[u::Weight] = 1;
        bool c1 = false, c2 = false, c3 = false, c4 = false;
        const float q1 = u::torqueScale(e3w3, u::Quad, 1.0f, &c1), q2 = u::torqueScale(e3w3, u::Quad, 2.0f, &c2), q3 = u::torqueScale(e3, u::Quad, 1.25f, &c3);
        const float q4 = u::torqueScale(e3, u::Quad, 2.0f, &c4);
        check("UPG5", "torque scale: caps 2.0 / 2.6, clamped flags", {q1, float(c1), q2, float(c2), q3, float(c3), q4, float(c4)},
              {2.0f, 1, 1.3f, 1, 1.8f, 0, 1.3f, 1});
        check("UPG6", "torque scale: class k, weight, stock, none", {u::torqueScale(e3, u::Bike, 1), u::torqueScale(e3, u::Dragster, 1), u::torqueScale(e3, u::Truck, 1),
              u::torqueScale(w1, u::Muscle, 1), u::torqueScale(st, u::Suv, 1.25f), u::torqueScale(e3, -1, 1)}, {1.6f, 1.4f, 1.8f, 1.0f / 0.96f, 1, 1});
    }
    check("UPG7", "widening min(ht L / h, cap)", {u::widening(0.6f, 0.1f, 0.06f), u::widening(0.5f, 1.0f, 0.08f), u::widening(0.5f, 0.01f, 0.03f),
          u::widening(0.5f, 0.3f, 0.0f), u::widening(0.9f, 0.45f, 0.12f, 0.3f), u::widening(0.9f, 0.1f, 0.12f, 0.3f)}, {0.06f, 0.04f, 0.03f, 0, 0.24f, 0.3f});
    {  // the VehiclePP rewrite on a synthetic block: every touched value 0.5 (compliances, grips, drag, inertia), travel 0.2
        std::vector<float> P0(680, 0.5f);
        P0[71] = P0[72] = 0.2f;
        auto run = [&](int cls, int cat, int stage) {
            std::vector<float> P = P0;
            u::Stages st;
            st[cat] = stage;
            u::transformParams(P, st, cls);
            return P;
        };
        const auto s3 = run(u::Quad, u::Suspension, 3), s1b = run(u::Bike, u::Suspension, 1), tb = run(u::Bike, u::Tires, 3);
        check("UPG19", "tyres S3 bike: lateral only", {tb[108], tb[109], tb[116], tb[132], tb[140], tb[110], tb[111], tb[119], tb[135], tb[143]},
              {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.56f, 0.56f, 0.56f, 0.56f, 0.56f});
        check("UPG8", "suspension S3 quad: damp kept / roll / bump / travel x2", {s3[79], s3[86], s3[87], s3[90], s3[71], s3[72], s3[73], s3[75]},
              {0.5f, 0.5f, 0.75f, 0.65f, 0.4f, 0.4f, 0.5f, 0.5f});
        check("UPG9", "suspension S1 bike: no anti-roll, travel x1.33", {s1b[79], s1b[87], s1b[89], s1b[71]}, {0.5f, 0.5f, 0.55f, 0.266f});
        const auto t3 = run(u::Suv, u::Tires, 3), t3q = run(u::Quad, u::Tires, 2);
        check("UPG10", "tyres S3 SUV long / lat (k 0.5) / S2 quad", {t3[108], t3[109], t3[110], t3[111], t3[116], t3[118], t3[132], t3[134], t3[140], t3[143], t3[112], t3q[110]},
              {0.56f, 0.56f, 0.53f, 0.53f, 0.56f, 0.53f, 0.56f, 0.53f, 0.56f, 0.53f, 0.5f, 0.54f});
        const auto w3 = run(u::Muscle, u::Weight, 3), w3q = run(u::Quad, u::Weight, 3);
        check("UPG11", "weight S3: drag kept / inertia / quad half / mass kept", {w3[93], w3[94], w3[4], w3[6], w3q[5], w3[2], w3[3], w3[7]},
              {0.5f, 0.5f, 0.44f, 0.44f, 0.47f, 0.5f, 0.5f, 0.5f});
        const auto e3 = run(u::Truck, u::Engine, 3), n3 = run(u::Truck, u::NitroTank, 3), none = run(-1, u::Tires, 3);
        bool same = true;
        for (size_t i = 0; i < P0.size(); ++i) same = same && e3[i] == P0[i] && n3[i] == P0[i] && none[i] == P0[i];
        check("UPG12", "engine / nitro / no class: VehiclePP as is", {float(same)}, {1});
    }
    {  // the classes and the nitro multipliers through apply() on a synthetic setup (two axles, CoM 0.5 m over the tyres)
        check("UPG13", "classOf: prefix / gifts / traffic / test", {float(u::classOf("QUAD_ROAD_2")), float(u::classOf("TRUCK_ALL_4")), float(u::classOf("BIKE_OFF_3")),
              float(u::classOf("BONUS_ROAD_3")), float(u::classOf("BONUS_OFF_1")), float(u::classOf("BONUS_ALL_2")), float(u::classOf("FLOTTE_JEEP_1")),
              float(u::classOf("TRAFFIC_TRUCK")), float(u::classOf("VEH_TEST_1"))}, {3, 5, 1, 7, 6, 5, -1, -1, -1});
        fuel::VehicleSetup s;
        s.name = "SUV_TEST_1";
        s.params.assign(680, 0.5f);
        s.params[11] = 0.4f;  // CoM y
        for (int k = 0; k < 4; ++k) {
            fuel::VehicleSetup::Wheel w;
            w.hub = {k % 2 ? -0.9f : 0.9f, 0.25f, k < 2 ? 1.4f : -1.4f};
            w.radius = 0.35f, w.front = k < 2;
            s.wheels.push_back(w);
        }
        u::Stages st;
        st.s.fill(3);
        fuel::VehicleSetup a = s;
        const u::Result r = u::apply(a, st, 1.25f);
        // CoM over the tyres 0.4 - (0.25 - 0.35) = 0.5; widening (SUV: the full formula, at most 0.3) 0.9 x 0.12 / 0.5 = 0.216
        check("UPG14", "apply S3 SUV: torque / lift / widen / CoM / hubs", {r.torqueScale, a.torqueScale, a.lift, a.widenFront, a.widenRear, r.comHeight,
              a.wheels[0].hub[0], a.wheels[1].hub[0], a.wheels[2].hub[1]}, {2.0f, 2.0f, 0.12f, 0.216f, 0.216f, 0.5f, 1.116f, -1.116f, 0.13f});
        check("UPG15", "apply S3 SUV: nitro power / tank / recharge", {a.nitroPower, a.nitroTank, a.nitroRecharge}, {1.45f, 1.6f, 1.3f});
        fuel::VehicleSetup b = s;
        b.name = "BIKE_TEST_1";
        b.bike = true;
        b.wheels.resize(2);
        b.wheels[1].front = false;
        const u::Result rb = u::apply(b, st, 1.0f);
        check("UPG16", "apply S3 bike: k 0.75, fork 3 cm, no widening", {rb.torqueScale, b.lift, b.widenFront, b.widenRear, b.nitroPower},
              {std::min(1.6f / 0.88f, 2.0f), 0.03f, 0, 0, 1.3375f});
        // off: nothing at all
        u::setEnabled(false);
        fuel::VehicleSetup c = s;
        const u::Result rc = u::apply(c, st, 1.0f);
        u::setEnabled(true);
        check("UPG17", "upgrades off: identity", {float(rc.applied), c.torqueScale, c.lift, c.widenFront, c.nitroPower, float(c.params == s.params), c.wheels[0].hub[0]},
              {0, 1, 0, 0, 1, 1, 0.9f});
    }
    check("UPG18", "prices: quad L3 engine / tires, L5 tank S3, L1 susp", {float(u::price(3, u::Engine, 1)), float(u::price(3, u::Engine, 2)),
          float(u::price(3, u::Engine, 3)), float(u::price(3, u::Tires, 3)), float(u::price(5, u::NitroTank, 3)), float(u::price(6, u::Weight, 1)),
          float(u::price(1, u::Suspension, 1)), float(u::starsNeeded(1)), float(u::starsNeeded(2)), float(u::starsNeeded(3))},
          {1700, 5600, 14000, 10000, 21250, 3300, 500, 0, 20, 45});
    // [upgrades U4] The opponents' stages (aiDraw: hub tier base, +-1 per category, rivals, SplitMix64 seeded per race and grid
    // slot) and their ghost factors (aiFactors). Known answers: the fields the game drew for CR2-Yell-CP (tier 0) and
    // CR60-Crmo-RD (tier 8) with seed 0, kept so a later build draws the same fields (notes upg4.md).
    {
        check("AID1", "base stage by hub tier", {float(u::aiBaseStage(-1)), float(u::aiBaseStage(0)), float(u::aiBaseStage(2)), float(u::aiBaseStage(3)),
              float(u::aiBaseStage(4)), float(u::aiBaseStage(5)), float(u::aiBaseStage(6)), float(u::aiBaseStage(7)), float(u::aiBaseStage(9))},
              {0, 0, 0, 1, 1, 2, 2, 3, 3});
        uint64_t sm = 0;
        const uint64_t s1 = u::splitmix64(sm), s2 = u::splitmix64(sm);
        check("AID2", "SplitMix64 (seed 0) / FNV-1a 64", {float(s1 == 0xE220A8397B1DCDAFull), float(s2 == 0x6E789E6AA1B965F4ull),
              float(u::fnv1a64("") == 0xcbf29ce484222325ull), float(u::fnv1a64("a") == 0xaf63dc4c8601ec8cull)}, {1, 1, 1, 1});
        static const char* kCr2[7] = {"0,0,1,0,0,0", "0,0,1,0,0,1", "0,0,0,1,0,1", "0,0,0,0,0,1", "0,0,0,0,0,1", "0,0,1,1,0,0", "0,1,0,0,0,1"};
        int cr2 = 0;
        for (int k = 0; k < 7; ++k) cr2 += u::formatStages(u::aiDraw("CR2-Yell-CP", k, 0, 0).stages) == kCr2[k];
        const u::AiDraw c0 = u::aiDraw("CR60-Crmo-RD", 0, 8, 0), c3 = u::aiDraw("CR60-Crmo-RD", 3, 8, 0), again = u::aiDraw("CR60-Crmo-RD", 3, 8, 0);
        check("AID3", "known fields CR2 (t0) / CR60 (t8)", {float(cr2), float(u::aiDraw("CR2-Yell-CP", 0, 0, 0).seed == 0x787ca8aa4390bd90ull),
              float(u::formatStages(c0.stages) == "3,3,2,3,3,3"), float(c3.rival), float(c3.seed == 0x4651a20efd6e66f9ull),
              float(again.stages == c3.stages && again.seed == c3.seed)}, {7, 1, 1, -1, 1, 1});
        int seedDiff = 0, slotDiff = 0;
        for (int k = 0; k < 11; ++k) {
            seedDiff += !(u::aiDraw("CR64-Val4-CP", k, 8, 0).stages == u::aiDraw("CR64-Val4-CP", k, 8, 12345).stages);
            slotDiff += u::aiDraw("CR64-Val4-CP", k, 8, 0).seed != u::aiDraw("CR64-Val4-CP", k + 1, 8, 0).seed;
        }
        const u::AiDraw none = u::aiDraw("CR2-Yell-CP", 0, -1, 0);
        check("AID4", "seed / slot change the draw, no hub", {float(seedDiff > 0), float(slotDiff), float(none.stages.stock()), float(none.rival), float(none.base)},
              {1, 11, 1, 0, 0});
        // Rates over 200,000 draws per tier: rivals (stock / max) and the category steps among the others.
        auto rates = [](int tier, double out[6]) {
            long n = 0, stock = 0, max = 0, cats = 0, lo = 0, mid = 0, hi = 0;
            const int b = u::aiBaseStage(tier);
            for (int r = 0; r < 20000; ++r)
                for (int k = 0; k < 10; ++k) {
                    const u::AiDraw d = u::aiDraw("R" + std::to_string(r), k, tier, 0);
                    ++n, stock += d.rival < 0, max += d.rival > 0;
                    if (d.rival) continue;
                    for (int c = 0; c < u::kCategories; ++c) ++cats, lo += d.stages[c] < b, mid += d.stages[c] == b, hi += d.stages[c] > b;
                }
            out[0] = double(stock) / n, out[1] = double(max) / n, out[2] = double(lo) / cats, out[3] = double(mid) / cats, out[4] = double(hi) / cats;
        };
        double t0[6], t3[6], t5[6], t9[6];
        rates(0, t0), rates(3, t3), rates(5, t5), rates(9, t9);
        auto near = [](double x, double want) { return std::fabs(x - want) < 0.006 ? 1.0f : 0.0f; };
        check("AID5", "rates t0: rivals 0, steps 0 / 75 / 25 %", {float(t0[0]), float(t0[1]), near(t0[2], 0), near(t0[3], 0.75), near(t0[4], 0.25)}, {0, 0, 1, 1, 1});
        check("AID6", "rates t3 / t5 / t9: rivals 5 / 3, 6, 10 %", {near(t3[0], 0.05), near(t3[1], 0.03), near(t5[0], 0.05), near(t5[1], 0.06), near(t9[0], 0.05),
              near(t9[1], 0.10)}, {1, 1, 1, 1, 1, 1});
        check("AID7", "steps t3 / t5 25/50/25, t9 25/75", {near(t3[2], 0.25), near(t3[3], 0.5), near(t3[4], 0.25), near(t5[2], 0.25), near(t5[3], 0.5),
              near(t5[4], 0.25), near(t9[2], 0.25), near(t9[3], 0.75), near(t9[4], 0)}, {1, 1, 1, 1, 1, 1, 1, 1, 1});
        int t = 0;
        const bool p1 = u::parseAiTier("7", t) && t == 7, p2 = u::parseAiTier("none", t) && t == u::kAiTierNone, p3 = !u::parseAiTier("10", t),
                   p4 = !u::parseAiTier("x", t), p5 = u::parseAiTier("auto", t) && t == u::kAiTierAuto;
        check("AID8", "--ai-upgrade-tier parse", {float(p1), float(p2), float(p3), float(p4), float(p5)}, {1, 1, 1, 1, 1});
        u::Stages s3;
        s3.s.fill(3);
        const u::AiFactors q = u::aiFactors(s3, u::Quad, 1.0f), m = u::aiFactors(s3, u::Muscle, 1.0f), sv = u::aiFactors(s3, u::Suv, 1.0f);
        check("AID9", "ghost factors S3 quad / muscle / SUV", {q.time, q.corner, q.accel, q.top, m.time, m.accel, sv.corner, sv.top},
              {0.95148889f, 1.0583005f, 1.4339552f, 1.023f, 0.9734f, 1.0792282f, 1.0295630f, 1.074f});
        u::Stages e1;
        e1[u::Engine] = 1;
        const u::AiFactors b1 = u::aiFactors(e1, u::Bike, 1.0f);
        u::setEnabled(false);
        const bool offId = u::aiFactors(s3, u::Quad, 1.0f).identity();
        u::setEnabled(true);
        check("AID10", "factors: stock / no class / off / bike E1", {float(u::aiFactors(u::Stages{}, u::Quad, 1).identity()), float(u::aiFactors(s3, -1, 1).identity()),
              float(offId), b1.time, b1.accel, b1.top, b1.corner}, {1, 1, 1, 0.991f, 1.0991290f, 1, 1});
    }
    // The governor's frame step and FUEL's torque at s == 1 (truck-power; plan 2.6).
    {
        const float dt = 1.0f / 60;
        float g = 1;
        for (int i = 0; i < 6; ++i) g = FuelVehicle::governorNext(g, 0.95f, false, dt);  // 0.1 s over the threshold: 1 - 0.8
        float h = 0;
        for (int i = 0; i < 15; ++i) h = FuelVehicle::governorNext(h, 0.5f, false, dt);  // 0.25 s under it: 0.5
        check("GOV1", "g: down 8/s over 0.8, up 2/s under", {g, h, FuelVehicle::governorNext(0.9f, 0.5f, true, dt), FuelVehicle::governorNext(0.05f, 2.0f, false, dt),
              FuelVehicle::governorNext(0.99f, 0.8f, false, dt)}, {0.2f, 0.5f, 0, 0, 1}, 1e-4f);
        std::vector<std::pair<int, float>> p = {{324, 8000}, {327, 1000}};
        for (int i = 0; i <= 10; ++i) p.push_back({338 + i, 0.1f * float(i)});
        FuelVehicle v = synthetic(p);
        check("GOV2", "torque at s 1 (FUEL) / s 1.8 at g 1", {v.governedTorque(4000, 1.0f), v.governedTorque(6100, 1.0f), v.governedTorque(4000, 1.8f)},
              {500, 762.5f, 900});
    }
    // UPGB: the player's data (all 83 VehiclePP blocks with their proxies).
    std::string data = std::getenv("FUEL_DATA") ? std::getenv("FUEL_DATA") : "";
    if (data.empty() && exe) data = std::filesystem::path(exe).parent_path().parent_path().string();
    const std::string tsc = data + "/CommonComponent/Param/VehiclesParam.tsc";
    if (data.empty() || !std::filesystem::exists(tsc)) {
        std::printf("UPGB   stock identity over the data skipped (no data: pass FUEL.exe's path or set FUEL_DATA)\n");
        if (!wasOn) u::setEnabled(false);
        return;
    }
    fuel::custom::setDataDir(data);
    const auto blocks = fuel::parseVehiclesParam(fuel::custom::readScript(tsc));
    int n = 0, same = 0, offSame = 0, staged = 0, unknown = 0, geo = 0;
    std::string neverNames;
    std::map<std::string, std::unique_ptr<fuel::DpcFile>> dpcs;
    u::Stages max;
    max.s.fill(3);
    for (const auto& [mesh, P] : blocks) {
        const std::string name = mesh.rfind("MESH_ID_", 0) == 0 ? mesh.substr(8) : mesh;
        std::string file;
        const int cls = u::classOf(name);
        if (const fuel::custom::Vehicle* cv = fuel::custom::find(name)) file = cv->dpc;
        else {
            const std::string pre = name.substr(0, name.find('_'));
            file = data + "/CommonComponent/DATAS/" + (pre == "BONUS" || pre == "FLOTTE" || pre == "TRAFFIC" || pre == "VEH" ? std::string("BONUS1") : pre) + ".DPC";
        }
        fuel::VehicleSetup base;
        try {
            auto& d = dpcs[file];
            if (!d) d = std::make_unique<fuel::DpcFile>(file);
            base = fuel::makeVehicleSetup(*d, name, P);
        } catch (const std::exception&) {
            base = fuel::VehicleSetup{};  // no proxy (test vehicles): the params alone
            base.name = name, base.params = P;
        }
        ++n;
        auto equal = [](const fuel::VehicleSetup& a, const fuel::VehicleSetup& b) {
            if (a.params.size() != b.params.size() || std::memcmp(a.params.data(), b.params.data(), a.params.size() * sizeof(float)) != 0) return false;
            if (a.wheels.size() != b.wheels.size()) return false;
            for (size_t i = 0; i < a.wheels.size(); ++i)
                if (std::memcmp(a.wheels[i].hub.data(), b.wheels[i].hub.data(), sizeof(float) * 3) != 0) return false;
            return a.torqueScale == 1 && a.lift == 0 && a.widenFront == 0 && a.widenRear == 0 && a.nitroPower == 1 && a.nitroTank == 1 &&
                   a.nitroRecharge == 1 && !a.upgraded && b.torqueScale == 1;
        };
        fuel::VehicleSetup s0 = base;
        u::apply(s0, u::Stages{}, 1.25f);
        same += equal(s0, base);
        u::setEnabled(false);
        fuel::VehicleSetup s1 = base;
        u::apply(s1, max, 1.25f);
        u::setEnabled(true);
        offSame += equal(s1, base);
        fuel::VehicleSetup s3 = base;
        const u::Result r = u::apply(s3, max, 1.25f);
        if (cls < 0) unknown += !r.applied && equal(s3, base), neverNames += " " + name;
        else {
            staged += r.applied && r.torqueScale > 1 && s3.params != base.params;
            // geometry: the lift of the class and the widening within 0..lift (bikes none)
            const float L = u::factors(cls).lift[2];
            const float W = (cls == u::Suv || cls == u::Truck) ? 0.3f : L;
            geo += r.lift == L && r.widenFront >= 0 && r.widenFront <= W + 1e-6f && r.widenRear <= W + 1e-6f && (!base.bike || r.widenFront == 0);
        }
    }
    int withClass = 0;
    for (const auto& [mesh, P] : blocks) withClass += u::classOf(mesh.rfind("MESH_ID_", 0) == 0 ? mesh.substr(8) : mesh) >= 0;
    std::printf("UPGB   %d VehiclePP blocks (%d with a class), stock identity %d, off identity %d, S3 changed %d, S3 geometry %d, never upgraded %d:%s\n", n, withClass,
                same, offSame, staged, geo, unknown, neverNames.c_str());
    check("UPGB", "all blocks: stock / off identity, S3", {float(same), float(offSame), float(staged), float(geo), float(unknown)},
          {float(n), float(n), float(withClass), float(withClass), float(n - withClass)});
    if (!wasOn) u::setEnabled(false);
}

// [suspanim] The suspension / steering parts every frame (core/susp_anim; notes/suspanim.md). SUSP1-3: 0x6a0200's hinge
// angle (sign by d . (0,0,1) [0xa06350] / d . (1,0,0) [0xa06338], acos of the y components) and the steer turn; SUSP4-5:
// the port's segment map (re-aimed parts: C fixed, B -> B'; coils: radius kept, pitch scaled); SUSP6-8 a synthetic rig:
// identity at rest, a Z-hinged arm following its hub, an upright (V:LINKM) at the arm's end steering with the wheel, a
// skinned shock reaching the arm point; SUSP9 a plan's re-aimed part at the rest hub = the static U5 adjust (no pop).
void suspVectors() {
    namespace s = fuel::susp;
    const float d10 = 10.0f * float(M_PI) / 180.0f;
    check("SUSP1", "hinge: rest 0, X arm up 10, Z arm up 10, rear X arm up 10",
          {s::hingeAngle({0, 0, 2}, {0, 0, 1}, 0), s::hingeAngle({0, std::sin(d10), std::cos(d10)}, {0, 0, 1}, 0),
           s::hingeAngle({std::cos(d10), std::sin(d10), 0}, {1, 0, 0}, 2), s::hingeAngle({0, std::sin(d10), -std::cos(d10)}, {0, 0, -1}, 0),
           s::hingeAngle({-std::cos(d10), std::sin(d10), 0}, {-1, 0, 0}, 2)},
          {0, -d10, d10, d10, -d10}, 1e-5f);
    {  // the turned LOCAL_LOOKAT points along d (in the hinge plane)
        auto rot = [](const std::array<float, 9>& m, s::V3 p) {
            return s::V3{m[0] * p[0] + m[1] * p[1] + m[2] * p[2], m[3] * p[0] + m[4] * p[1] + m[5] * p[2], m[6] * p[0] + m[7] * p[1] + m[8] * p[2]};
        };
        const s::V3 dx{0, -0.3f, 0.6f}, dz{-0.5f, 0.2f, 0};
        const float ax = s::hingeAngle(dx, {0, 0, 1}, 0), az = s::hingeAngle(dz, {-1, 0, 0}, 2);
        const s::V3 lx = rot(s::rotation({1, 0, 0}, ax), {0, 0, 1}), lz = rot(s::rotation({0, 0, 1}, az), {-1, 0, 0});
        const float nx = std::sqrt(0.09f + 0.36f), nz = std::sqrt(0.25f + 0.04f);
        check("SUSP2", "hinge turns LOOKAT onto d (X, Z)", {lx[1], lx[2], lz[0], lz[1]}, {dx[1] / nx, dx[2] / nx, dz[0] / nz, dz[1] / nz}, 1e-5f);
        const float a20 = 20.0f * float(M_PI) / 180.0f;
        const s::V3 t = rot(s::turnAbout({0, 1, 0}, {0, 0, 1}, {std::sin(a20), 0, std::cos(a20)}), {0, 0, 1});
        const s::V3 tl = rot(s::turnAbout({0, 0.8f, -0.6f}, {0, 0, 1}, {0, 0, 1}), {0.3f, 0.1f, 0.2f});
        check("SUSP3", "steer turn about up 20 deg; straight = identity", {t[0], t[1], t[2], tl[0], tl[1], tl[2]},
              {std::sin(a20), 0, std::cos(a20), 0.3f, 0.1f, 0.2f}, 1e-5f);
    }
    {
        const s::Affine a = s::segment({0, 1, 0}, {1, 0.5f, 0}, {1, 0.4f, 0}, 1.2f);
        const s::V3 c = a.apply({0, 1, 0}), b = a.apply({1, 0.5f, 0});
        check("SUSP4", "segment: C fixed, B -> B'", {c[0], c[1], c[2], b[0], b[1], b[2]}, {0, 1, 0, 1, 0.4f, 0}, 1e-5f);
        const s::Affine k = s::segment({0, 0, 0}, {0, 1, 0}, {0, 0.8f, 0}, 1.0f);  // a coil compressed to 80 %
        const s::V3 q = k.apply({0.05f, 0.5f, 0}), r = k.apply({0, 0.25f, -0.05f});
        check("SUSP5", "coil: radius kept, pitch x0.8", {q[0], q[1], q[2], r[0], r[1], r[2]}, {0.05f, 0.4f, 0, 0, 0.2f, -0.05f}, 1e-5f);
    }
    {  // a synthetic quad corner: wheel bone 1 at the hub, arm on bone 2 (AxeZ) aimed at bone 1, a shock (skinned LINKB) on
       // bone 3 aimed at the arm's point, an upright (LINKM) at the arm's end aimed at the wheel bone
        fuel::AssembledVehicle av;
        av.bonePos = {{0, 0, 0}, {0.45f, 0.3f, 0.8f}, {0.1f, 0.35f, 0.8f}, {0.12f, 0.75f, 0.7f}};
        auto at = [](s::V3 p) {
            fuel::Mat4 m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            m[12] = p[0], m[13] = p[1], m[14] = p[2];
            return m;
        };
        fuel::PlacedPart wheel, arm, shock, up;
        wheel.meshId = 1, wheel.placement = fuel::Placement::Wheel, wheel.bone = 1, wheel.world = at(av.bonePos[1]), wheel.objectId = 11;
        arm.meshId = 2, arm.placement = fuel::Placement::Link, arm.bone = 2, arm.world = at(av.bonePos[2]), arm.objectId = 12;
        arm.aim.bone = 1, arm.aim.axis = 2, arm.aim.hasLook = true, arm.aim.look = {0.35f, -0.05f, 0};
        shock.meshId = 3, shock.placement = fuel::Placement::LinkB, shock.bone = 3, shock.world = at(av.bonePos[3]), shock.objectId = 13;
        shock.shape = fuel::PartShape::SkinBindPose;
        shock.aim.object = 12, shock.aim.point = {0.3f, 0.33f, 0.8f}, shock.aim.hasLook = true, shock.aim.look = {0.18f, -0.42f, 0.1f};
        up.meshId = 4, up.placement = fuel::Placement::LinkM, up.bone = 1, up.world = at({0.42f, 0.3f, 0.8f}), up.objectId = 14;
        up.aim.atObject = 12, up.aim.bone = 1, up.aim.hasLook = true, up.aim.look = {0, 0, 1};
        av.parts = {arm, shock, up, wheel};
        const std::vector<s::V3> hubs{{0.45f, 0.3f, 0.8f}};
        const s::Rig rig = s::build(av, hubs, nullptr);
        s::Output o;
        std::vector<s::WheelLive> w(1);
        w[0].hub = hubs[0];
        s::evaluate(rig, w, o);
        float worst = 0;
        for (const s::Affine& a : o.node) {
            const s::V3 p = a.apply({0.3f, 0.2f, 0.9f});
            worst = std::max({worst, std::fabs(p[0] - 0.3f), std::fabs(p[1] - 0.2f), std::fabs(p[2] - 0.9f)});
        }
        check("SUSP6", "rig: 3 live nodes, identity at rest", {float(rig.nodes.size()), worst}, {3, 0}, 1e-5f);
        const float a5 = 5.0f * float(M_PI) / 180.0f;  // the wheel up 3 cm and steered 5 deg
        w[0].hub = {0.45f, 0.33f, 0.8f}, w[0].forward = {std::sin(a5), 0, std::cos(a5)};
        s::evaluate(rig, w, o);
        const s::Affine& A = o.node[size_t(rig.partNode[0])];
        const s::V3 end = A.apply({0.45f, 0.3f, 0.8f});  // the arm's authored end (LOCAL_LOOKAT ends at the bone)
        const s::V3 sp = o.node[size_t(rig.partNode[1])].apply({0.3f, 0.75f - 0.42f, 0.8f});  // the shock's end
        const s::V3 tgt = A.apply({0.3f, 0.33f, 0.8f});
        const s::Affine& U = o.node[size_t(rig.partNode[2])];
        const s::V3 u0 = U.apply({0.42f, 0.3f, 0.8f}), uf = U.apply({0.42f, 0.3f, 1.8f});
        check("SUSP7", "arm end at the raised hub (y), upright at the arm's end, steered 5 deg",
              {end[1], u0[0] - A.apply({0.42f, 0.3f, 0.8f})[0], u0[1] - A.apply({0.42f, 0.3f, 0.8f})[1], (uf[0] - u0[0]) / (uf[2] - u0[2])},
              {0.33f, 0, 0, std::tan(a5)}, 2e-3f);
        const float dist = std::sqrt((sp[0] - tgt[0]) * (sp[0] - tgt[0]) + (sp[1] - tgt[1]) * (sp[1] - tgt[1]) + (sp[2] - tgt[2]) * (sp[2] - tgt[2]));
        check("SUSP8", "skinned shock: its end on the arm's moving point (m)", {dist}, {0}, 1e-3f);
        // SUSP9: the plan's re-aimed arm (a Live segment) at the rest hub = the static adjust, raised 3 cm = B + 3 cm
        fuel::upgvis::Plan plan;
        plan.active = true;
        plan.parts.resize(av.parts.size());
        fuel::upgvis::Live& L = plan.parts[0].live;
        L.kind = fuel::upgvis::Live::Segment, L.wheel = 0, L.C = {0.1f, 0.35f, 0.8f}, L.B = {0.45f, 0.3f, 0.8f}, L.D = {0, -0.06f, 0}, L.sB = 1, L.thick = 1.12f;
        plan.parts[0].moved = true;
        plan.parts[0].adjust = fuel::upgvis::segmentMap(L.C, L.B, {0.45f, 0.24f, 0.8f}, 1.12f);
        const s::Rig r2 = s::build(av, hubs, &plan);
        w[0].hub = hubs[0], w[0].forward = {0, 0, 1};
        s::evaluate(r2, w, o);
        const s::Affine& P = o.node[size_t(r2.partNode[0])];
        const s::V3 q = P.apply({0.3f, 0.4f, 0.75f}), qs = fuel::upgvis::transformPoint(plan.parts[0].adjust, {0.3f, 0.4f, 0.75f});
        w[0].hub = {0.45f, 0.33f, 0.8f};
        s::evaluate(r2, w, o);
        const s::V3 b = o.node[size_t(r2.partNode[0])].apply(L.B);
        check("SUSP9", "plan arm: rest = U5 adjust, hub +3 cm -> B y", {q[0] - qs[0], q[1] - qs[1], q[2] - qs[2], b[1]}, {0, 0, 0, 0.27f}, 1e-5f);
    }
}

int main(int argc, char** argv) {
    std::printf("FUEL vehicle physics test vectors (emulated on FUEL.exe, synthetic inputs)\n");
    penaltyVectors();
    tyreVectors();
    integratorVectors();
    torqueVectors();
    steerAccelVectors();
    steerAngleVectors();
    channelVectors();
    airVector();
    drivelineVectors();
    bodyForceVectors();
    guidedForceVectors();
    derivedVectors();
    collideVectors();
    damageVectors(argc > 1 ? argv[1] : std::getenv("FUEL_EXE"));
    visualDamageVectors(argc > 1 ? argv[1] : std::getenv("FUEL_EXE"));
    rpmVectors();
    breadcrumbVectors();
    startHoldVectors();  // PK01
    heliVectors();       // PK07
    aiGateVectors();     // rain-ai
    f14Vectors();        // PK27
    hubOffsetVectors();
    worldInertiaVector();
    reductionVectors();
    linkVectors();  // PK22
    nitroVectors();  // [nitro]
    upgradeVectors(argc > 1 ? argv[1] : std::getenv("FUEL_EXE"));  // [upgrades U1]
    suspVectors();  // [suspanim]
    std::printf("%d match, %d mismatch\n", passes, failures);
    return failures ? 1 : 0;
}
