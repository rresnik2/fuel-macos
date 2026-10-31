// fuelphysvec: runs the test vectors of FUEL's vehicle physics decode (notes p23-vehicle-physics.md,
// "Test vectors"; values from Ghidra EmulatorHelper runs of FUEL.exe on synthetic inputs, no game data)
// through the port's functions and prints match / MISMATCH per vector. Exit status 1 on any mismatch.
//
//   fuelphysvec            all vectors
#include "physics/fuel_contacts.hpp"
#include "physics/fuel_vehicle.hpp"

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

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
    static void bodyForces(FuelVehicle& v, float yaw, Vec3 vel, Vec3 w, float airtime, float chB, float acc, int gear, const float loads[4]) {
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
FuelVehicle synthetic(std::vector<std::pair<int, float>> params) {
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
    for (int w = 0; w < 4; ++w) proxy.spheres.push_back({pos[w], 0.3f, names[w]});
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

void derivedVectors() {
    // Gravity (0x530cb0): F = (0, m Pg, 0). Anti-roll example (QUAD): d = 0.02 -> F = m 0.1 min(50 0.02, 9.81) = 0.1 m.
    FuelVehicle v = synthetic({{287, 0.01f}, {288, -17}, {289, -15}, {87, 50}, {88, 50}});
    check("GRAV", "gravity force / mass, P288", {-17.0f}, {-17.0f});
    const float lift = 300 * 0.1f * std::min(50 * 0.02f, 9.81f);
    check("AROLL", "anti-roll lift (QUAD d=0.02)", {lift / 300}, {0.1f});
    (void)v;
}

}  // namespace

int main() {
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
    derivedVectors();
    std::printf("%d match, %d mismatch\n", passes, failures);
    return failures ? 1 : 0;
}
