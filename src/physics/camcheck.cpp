#include "camcheck.hpp"

#include "fuel_camera.hpp"

#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace fuel {

namespace {

using cam::Vec3;

bool near(float a, float b) { return std::fabs(a - b) <= 2e-6f + 2e-5f * std::fabs(b); }
bool nearV(const Vec3& a, const Vec3& b) { return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z); }
std::string fmt(const Vec3& v) {
    char b[96];
    std::snprintf(b, sizeof b, "(%.8g, %.8g, %.8g)", v.x, v.y, v.z);
    return b;
}
std::string fmt(float v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.8g", v);
    return b;
}
Vec3 n3(float x, float y, float z) { return fv::normalize(Vec3{x, y, z}); }

}  // namespace

CamCheckResult runCameraVectors(const std::vector<std::function<std::vector<CamCheckItem>()>>& extra) {
    std::vector<CamCheckItem> items;
    auto add = [&](const std::string& name, bool ok, const std::string& detail) { items.push_back({name, ok, detail}); };
    auto vec = [&](const std::string& name, const Vec3& got, const Vec3& want) {
        add(name, nearV(got, want), "got " + fmt(got) + " want " + fmt(want));
    };
    auto num = [&](const std::string& name, float got, float want) { add(name, near(got, want), "got " + fmt(got) + " want " + fmt(want)); };

    // Smooth1 0x4100b0 / SmoothV 0x40fb70
    num("Smooth1 0x4100b0", cam::smooth1(0, 1, 1.05f, 1.0f / 60), 0.024340354f);
    vec("SmoothV 0x40fb70", cam::smoothV({}, {1, 2, -1}, 1.05f, 1.0f / 60), {0.0243404f, 0.0486807f, -0.0243404f});
    // Lift 0x404040
    vec("Lift 0x404040 #0", cam::ChaseCamera::liftOffset(n3(0.1f, 1, 0.05f), n3(1, 0, 0.1f), n3(0, 0.1f, 1), 0.68f, 0.3f),
        {-0.00403641f, 1.0639697f, 0.48813084f});
    vec("Lift 0x404040 #1", cam::ChaseCamera::liftOffset(n3(0.3f, 0.9f, -0.2f), n3(0.9f, 0, 0.4f), n3(-0.4f, 0.2f, 0.9f), 0.68f, 0.3f),
        {-0.04630065f, 1.0557175f, 0.10417644f});
    // SnapOffset 0x409790 (D = 1.91 * 1.5)
    vec("SnapOffset 0x409790 #0", cam::ChaseCamera::snapOffset({0, 0, 1}, -0.1f, 1.91f * 1.5f), {0, 0.28602275f, -2.850687f});
    vec("SnapOffset 0x409790 #1", cam::ChaseCamera::snapOffset({0.6f, 0.2f, 0.774597f}, 0.05f, 1.91f * 1.5f), {-1.6993141f, -0.71258104f, -2.193806f});
    // Distance 0x405800 (D 2.865): {O, lock in, speed, want O, want lock}
    struct DCase {
        Vec3 O;
        bool lock;
        float speed;
        Vec3 want;
        bool wantLock;
    };
    const DCase dc[] = {{{0.3f, 1.2f, -4}, false, 5, {0.20528391f, 0.82113564f, -2.7371187f}, true},
                        {{0.3f, 1.2f, -2}, false, 5, {0.3f, 1.2f, -2}, true},
                        {{0.3f, 1.2f, -2}, true, 5, {0.36549658f, 1.4619863f, -2.4366438f}, true},
                        {{0.3f, -0.5f, -2}, true, 5, {0.4249954f, -0.7083256f, -2.8333025f}, true},
                        {{0.3f, 1.2f, -2}, false, 0.5f, {0.3f, 1.2f, -2}, false}};
    for (int i = 0; i < 5; ++i) {
        cam::ChaseCamera c;
        c.yaw() = c.pitch() = 0;
        c.lock() = dc[i].lock;
        const Vec3 o = c.distance(dc[i].O, 2.865f, dc[i].speed, false);
        add("Distance 0x405800 #" + std::to_string(i), nearV(o, dc[i].want) && c.lock() == dc[i].wantLock,
            "got " + fmt(o) + " lock " + std::to_string(c.lock()) + " want " + fmt(dc[i].want) + " lock " + std::to_string(dc[i].wantLock));
    }
    // Manual orbit 0x404c90 (O (0.3,1.2,-2.8), ctx30 1.91, speed 10)
    struct OCase {
        float sx, sy, y0, p0, dt;
        Vec3 want;
        float wy, wp, w18dc, w18e8;
    };
    const OCase oc[] = {{0, 0, 0, 0, 1.0f / 60, {0.3f, 1.2f, -2.8f}, 0, 0, -0.016666668f, 1},
                        {1, 0, 0, 0, 1.0f / 60, {0.40874088f, 1.2f, -2.7883492f}, -0.29536432f, 0, 0.98333335f, 0},
                        {0, -1, 0.2f, 0.1f, 1.0f / 60, {0.2542379f, 1.0651597f, -2.8626919f}, 0.14092714f, 0.36582786f, 0.98333335f, 0},
                        {0, 0, 0.5f, -0.3f, 1.0f / 30, {0.11222026f, 1.3098055f, -2.7730598f}, 0.24825573f, -0.14895344f, 0.96666664f, 0}};
    for (int i = 0; i < 4; ++i) {
        cam::ChaseCamera c;
        c.yaw() = oc[i].y0, c.pitch() = oc[i].p0;
        c.pauseTimer() = 0, c.autoFollow() = 0, c.autoFollowTarget() = 0;
        const Vec3 o = c.orbit({0.3f, 1.2f, -2.8f}, oc[i].sx, oc[i].sy, 10, 1.91f, oc[i].dt, false);
        const bool ok = nearV(o, oc[i].want) && near(c.yaw(), oc[i].wy) && near(c.pitch(), oc[i].wp) && near(c.pauseTimer(), oc[i].w18dc) &&
                        near(c.autoFollow(), oc[i].w18e8);
        add("Orbit 0x404c90 #" + std::to_string(i), ok,
            "got " + fmt(o) + " angles " + fmt(c.yaw()) + "/" + fmt(c.pitch()) + " 18dc " + fmt(c.pauseTimer()) + " 18e8 " + fmt(c.autoFollow()) +
                " want " + fmt(oc[i].want) + " " + fmt(oc[i].wy) + "/" + fmt(oc[i].wp));
    }
    // Follow spring 0x405440
    struct FCase {
        Vec3 O, Dd;
        float g, tg, dt;
        Vec3 want;
        float wg;
    };
    const FCase fc[] = {{{1, 1, -2.5f}, n3(0, 0.3f, -1), 0.3f, 0.5f, 1.0f / 60, {0.95123446f, 0.99888223f, -2.5199535f}, 0.30939928f},
                        {{0.5f, 1, 2.5f}, n3(0, 0.3f, -1), 0.3f, 0.5f, 1.0f / 60, {0.6875338f, 1.00483f, 2.4605613f}, 0.30939928f},
                        {{-2, 0.4f, -1}, n3(0.2f, 0.25f, -1), 0.8f, 0.2f, 1.0f / 30, {-1.9356371f, 0.4056461f, -1.1264675f}, 0.2f}};
    for (int i = 0; i < 3; ++i) {
        cam::ChaseCamera c;
        c.gain() = fc[i].g, c.autoFollowTarget() = 1;
        const Vec3 o = c.followSpring(fc[i].O, fc[i].Dd, fc[i].tg, fc[i].dt);
        add("FollowSpring 0x405440 #" + std::to_string(i), nearV(o, fc[i].want) && near(c.gain(), fc[i].wg),
            "got " + fmt(o) + " g " + fmt(c.gain()) + " want " + fmt(fc[i].want) + " g " + fmt(fc[i].wg));
    }
    // Lag 0x4091e0 (ctx48 0.33, side (1,0,0))
    struct LCase {
        Vec3 O, v;
        float pitch, g2, a18e8;
        bool d197;
        float dt;
        Vec3 want;
        float wg2;
        bool wd197;
    };
    const LCase lc[] = {
        {{0.3f, 1.2f, -2.8f}, {1, 0.2f, 12}, -0.1f, 0.5f, 1, true, 1.0f / 60, {0.2828924f, 1.1688763f, -2.8151717f}, 0.49586213f, true},
        {{0.3f, 1.2f, -2.8f}, {1, 0.2f, -6}, -0.1f, 0.5f, 1, true, 1.0f / 60, {0.27759245f, 1.2f, -2.7559247f}, 0.49586213f, true},
        {{-1.5f, 0.8f, -2}, {4, -1, 8}, 0.05f, 0.2f, 0, false, 1.0f / 30, {-1.495522f, 0.78148586f, -2.0107641f}, 0.19038235f, false},
        {{0, 1, -3}, {0.02f, 0, -0.03f}, -0.1f, 0.9f, 0, true, 1.0f / 60, {0, 1, -3}, 0.87809366f, false}};
    for (int i = 0; i < 4; ++i) {
        cam::ChaseCamera c;
        c.gain2() = lc[i].g2, c.autoFollow() = lc[i].a18e8, c.flag197d() = lc[i].d197;
        const Vec3 o = c.lag(lc[i].O, lc[i].v, lc[i].pitch, {1, 0, 0}, lc[i].dt, false);
        add("Lag 0x4091e0 #" + std::to_string(i), nearV(o, lc[i].want) && near(c.gain2(), lc[i].wg2) && c.flag197d() == lc[i].wd197,
            "got " + fmt(o) + " g2 " + fmt(c.gain2()) + " want " + fmt(lc[i].want) + " g2 " + fmt(lc[i].wg2));
    }
    // First-person head spring 0x408d30: frame 1 snap, frame 2 rolled 0.1 rad
    {
        cam::ChaseCamera c;
        Vec3 eye{0, 1.2f, 0.3f}, tgt{0, 1.2f, 100.3f};
        float roll = 0;
        c.headSpring({0, 1, 0}, {0, 0, 1}, eye, tgt, 1.0f / 60, true, roll, 1.0f);
        const bool ok1 = nearV(eye, {0, 1.2f, 0.3f}) && nearV(tgt, {0, 1.2f, 100.3f}) && near(roll, 0);
        add("HeadSpring 0x408d30 frame 1", ok1, "got " + fmt(eye) + " roll " + fmt(roll));
        eye = {0.11980011f, 1.194005f, 0.3f}, tgt = {0.11980011f, 1.194005f, 100.3f}, roll = 0;
        c.headSpring({0.09983342f, 0.99500417f, 0}, {0, 0, 1}, eye, tgt, 1.0f / 60, false, roll, 1.0f);
        const auto& h = c.head();
        const bool ok2 = nearV(eye, {0.12480001f, 1.1934649f, 0.3f}) && nearV(tgt, {0.12480001f, 1.1934649f, 100.3f}) && near(roll, 0.015239519f) &&
                         nearV(h.prev, {0.03294503f, 0.32835138f, 0}) && nearV(h.p, {0.0050332686f, -2.5187276e-4f, 0}) &&
                         nearV(h.v, {0.3019961f, -0.015112365f, 0});
        add("HeadSpring 0x408d30 frame 2", ok2, "got eye " + fmt(eye) + " roll " + fmt(roll) + " p " + fmt(h.p) + " v " + fmt(h.v));
    }
    for (const auto& f : extra)
        for (CamCheckItem& it : f()) items.push_back(std::move(it));

    CamCheckResult r;
    for (const CamCheckItem& it : items) {
        ++r.total;
        r.matched += it.ok;
        r.report += std::string(it.ok ? "  match  " : "  MISS   ") + it.name + ": " + it.detail + "\n";
    }
    return r;
}

}  // namespace fuel
