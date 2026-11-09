#include "f14_view.h"

#include "core/assembly.hpp"
#include "core/resources.hpp"
#include "physics/fuel_math.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

bool F14View::load(const fuel::DpcFile& world) {
    meshes_.clear();
    const uint32_t crc = fuel::asoboCrc("F14");
    for (uint32_t id : world.ids(fuel::cls::Lod)) {
        const std::vector<uint8_t> link = world.link(*world.find(id));
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name != crc) continue;
        const auto body = world.body(*world.find(id));
        if (!body) return false;
        try {
            const fuel::Lod lod = fuel::parseLodBody(*body);
            // Lod_Z::Draw at level 0 (base distance 1, no fade / cull distances in the shipped Lod): its first object.
            if (!lod.objects.empty()) {
                const auto* e = world.find(lod.objects[0]);
                if (e && e->classCrc == fuel::cls::Mesh) meshes_.push_back(lod.objects[0]);
            }
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "fuelview: F14: %s\n", ex.what());
        }
        return !meshes_.empty();
    }
    return false;
}

simd_float4x4 F14View::world(const fuel::F14Flyby& f) {
    const fuel::fv::Quat q{f.nodeQuat[0], f.nodeQuat[1], f.nodeQuat[2], f.nodeQuat[3]};
    const fuel::fv::Vec3 ax = fuel::fv::rotate(q, {1, 0, 0}), ay = fuel::fv::rotate(q, {0, 1, 0}), az = fuel::fv::rotate(q, {0, 0, 1});
    simd_float4x4 w = matrix_identity_float4x4;
    w.columns[0] = simd_make_float4(ax.x, ax.y, ax.z, 0);
    w.columns[1] = simd_make_float4(ay.x, ay.y, ay.z, 0);
    w.columns[2] = simd_make_float4(az.x, az.y, az.z, 0);
    w.columns[3] = simd_make_float4(f.nodePos[0], f.nodePos[1], f.nodePos[2], 1);
    return w;
}

void F14View::frame(const fuel::F14Flyby* f, fuel::MeshRenderer& r) {
    if (meshes_.empty()) return;
    if (f == nullptr || !f->shown) {
        if (shown_) r.setF14Instances({});
        shown_ = false;
        return;
    }
    const simd_float4x4 w = world(*f);
    std::vector<fuel::MeshInstance> list;
    for (uint32_t m : meshes_) list.push_back({m, w});
    r.setF14Instances(list);
    shown_ = true;
}

bool F14View::debugLook(fuel::OrbitCamera& cam, const fuel::F14Flyby& f, simd_float3 player) {
    const char* e = std::getenv("FUEL_F14_LOOK");
    if (!e || !f.shown) return false;
    if (e[0] == 'p') {  // "player": from 2 m above the player, looking at the jet
        cam.eye = player + simd_make_float3(0, 2, 0);
        cam.target = simd_make_float3(f.nodePos[0], f.nodePos[1], f.nodePos[2]);
        return true;
    }
    float d = 60, up = 10, a = 0;
    std::sscanf(e, "%f,%f,%f", &d, &up, &a);
    float hx = f.vel[0], hz = f.vel[2];
    const float l = std::hypot(hx, hz);
    if (l < 1e-3f) hx = 0, hz = 1;
    else hx /= l, hz /= l;
    const float r = a * 3.14159265f / 180.0f, c = std::cos(r), s = std::sin(r);
    const float bx = -(hx * c - hz * s), bz = -(hx * s + hz * c);  // behind, turned by a
    cam.eye = simd_make_float3(f.nodePos[0] + bx * d, f.nodePos[1] + up, f.nodePos[2] + bz * d);
    cam.target = simd_make_float3(f.nodePos[0], f.nodePos[1], f.nodePos[2]);
    return true;
}
