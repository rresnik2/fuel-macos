#include "heli_view.h"

#include "core/resources.hpp"
#include "physics/fuel_math.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

bool HeliView::load(const std::vector<const fuel::DpcFile*>& files, const fuel::DpcFile& world, fuel::MeshRenderer& r) {
    keys_.clear();
    const uint32_t crc = fuel::asoboCrc("IT_HELICO");
    for (uint32_t id : world.ids(fuel::cls::Lod)) {
        const std::vector<uint8_t> link = world.link(*world.find(id));
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name != crc) continue;
        std::string err;
        if (!model_.load(files, id, &err)) {
            std::fprintf(stderr, "fuelview: IT_HELICO: %s\n", err.c_str());
            return false;
        }
        clip_ = model_.clipForSlot(0);
        for (size_t p = 0; p < model_.parts.size(); ++p)
            if (r.addBakedMesh(0xF2000000u + uint32_t(p), model_.parts[p].mesh, &err)) keys_.push_back(0xF2000000u + uint32_t(p));
        return !keys_.empty();
    }
    return false;
}

void HeliView::frame(float dt, const fuel::RaceHelicopter* h, fuel::MeshRenderer& r) {
    if (keys_.empty()) return;
    if (h == nullptr) {
        if (shown_) r.setHeliInstances({});
        shown_ = false;
        return;
    }
    if (clip_) {  // PlayAnim_Z: t += dt, looped
        t_ += dt;
        if (clip_->loops())
            while (t_ > clip_->duration) t_ -= clip_->duration;
    }
    const fuel::fv::Quat q{h->quat[0], h->quat[1], h->quat[2], h->quat[3]};
    const fuel::fv::Vec3 ax = fuel::fv::rotate(q, {1, 0, 0}), ay = fuel::fv::rotate(q, {0, 1, 0}), az = fuel::fv::rotate(q, {0, 0, 1});
    simd_float4x4 w = matrix_identity_float4x4;
    w.columns[0] = simd_make_float4(ax.x, ax.y, ax.z, 0);
    w.columns[1] = simd_make_float4(ay.x, ay.y, ay.z, 0);
    w.columns[2] = simd_make_float4(az.x, az.y, az.z, 0);
    w.columns[3] = simd_make_float4(h->disp[0], h->disp[1], h->disp[2], 1);
    const auto bones = model_.pose(clip_, t_);
    std::vector<fuel::MeshInstance> list;
    std::vector<uint8_t> v;
    for (size_t p = 0; p < keys_.size(); ++p) {
        model_.skin(model_.parts[p], bones, v);
        r.updateBakedVertices(keys_[p], v);
        list.push_back({keys_[p], w});
    }
    r.setHeliInstances(list);
    shown_ = true;
}

void HeliView::debugLook(fuel::OrbitCamera& cam, const fuel::RaceHelicopter& h) {
    const char* e = std::getenv("FUEL_HELI_LOOK");
    if (!e) return;
    float d = 25, up = 4, a = 0;
    std::sscanf(e, "%f,%f,%f", &d, &up, &a);
    float hx = h.vel[0], hz = h.vel[2];
    const float l = std::hypot(hx, hz);
    if (l < 1e-3f) hx = 0, hz = 1;
    else hx /= l, hz /= l;
    const float r = a * 3.14159265f / 180.0f, c = std::cos(r), s = std::sin(r);
    const float bx = -(hx * c - hz * s), bz = -(hx * s + hz * c);  // behind, turned by a
    cam.eye = simd_make_float3(h.disp[0] + bx * d, h.disp[1] + up, h.disp[2] + bz * d);
    cam.target = simd_make_float3(h.disp[0], h.disp[1], h.disp[2]);
}
