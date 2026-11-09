// PK20: FUEL's 3-D garage preview, the 3-D side (notes/pk20.md). Tags: [O] observed in FUEL.exe, [I] inferred,
// [port] port choice.
//
// FUEL (InGamePauseHUD, garage pages):
//   InGame3DVehicle +0xf0 (ctor 0x4d0d90, vf00 0x4d10f0, vf01 0x4d0df0, set-up 0x4d1020, set vehicle 0x4d1110)
//   InGame3DPerso   +0x214 (ctor 0x4d0460, vf00 0x4d0a00, vf01 0x4d04d0, set-up 0x4d0720, framing 0x4d0ad0)
//   each with an InGameObjects_Z scene (ctor 0x68f910 through 0x4cfd50 / 0x4362c0, draw 0x845ac0 -> 0x7ca0a0,
//   lights 0x68ffd0 x 0x4cfec0, shown / hidden by 0x436020).
// The menu (ui/garage.mm) queues the calls the garage page makes into them; run() executes them in that order and
// draw() renders the scenes that are on with a MeshRenderer of its own (renderer.h PreviewScene).
#include "viewer/garage_preview.h"

#include "core/assembly.hpp"
#include "core/atmosphere.hpp"
#include "core/dpc.hpp"
#include "core/material_anim.hpp"
#include "core/resources.hpp"
#include "core/rider.hpp"
#include "core/custom_rider.hpp"  // PHYSIQUE
#include "core/vehicle.hpp"
#include "core/vehicle_params.hpp"
#include "core/custom_vehicles.hpp"  // [upgrades U5] the packs' VehiclesParam lines
#include "core/upgrade_visuals.hpp"  // [upgrades U5]
#include "core/upgrades.hpp"         // [upgrades U5]
#include "physics/fuel_vehicle.hpp"  // [upgrades U5] powerScale
#include "physics/vehicle_physics.hpp"  // [upgrades U5] makeVehicleSetup
#include "physics/fuel_camera.hpp"
#include "ui/hud_math.hpp"
#include "viewer/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <future>
#include <map>

namespace fs = std::filesystem;

namespace fuel {
namespace {

using fv::Quat;
using fv::Vec3;

constexpr int kRiderSlot = 0x11;              // 0x4d0720: 0x667a00 with ECX 0x11 (PILOT.DPC: a 19.33 s loop) [O]
constexpr uint32_t kPilotLodId = 0x9A9C3E61;  // the IT_PILOT_SKIN Lod_Z (rider.cpp)
constexpr uint32_t kRiderKey = 0x504b3230;    // the preview rider's baked mesh key ("PK20"; not a Mesh_Z id)
constexpr NSUInteger kSamples = 4;            // as the session's scene pass

float P(const std::vector<float>& pp, int i) { return size_t(i) < pp.size() ? pp[size_t(i)] : 0.0f; }

// Object_Z +0x18 (the "size" 0x4d0ad0 / 0x4d1110 read: the bounding sphere radius, link bytes 8..23 = centre, radius).
float objectRadius(const DpcFile& f, const DpcFile::Entry& e) {
    const std::vector<uint8_t> l = f.link(e);
    float r = 0;
    if (l.size() >= 24) std::memcpy(&r, l.data() + 20, 4);
    return r;
}

// The physics agent of a vehicle (vehicle record +0x74) is the "<variant>_DYN" Lod_Z (vehicle_params.cpp).
const DpcFile::Entry* dynLod(const DpcFile& dpc, const std::string& variant) {
    const uint32_t want = asoboCrc(variant + "_DYN");
    for (uint32_t id : dpc.ids(cls::Lod)) {
        const DpcFile::Entry* e = dpc.find(id);
        const std::vector<uint8_t> link = dpc.link(*e);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name == want) return e;
    }
    return nullptr;
}

simd_float4x4 nodeMatrix(const Vec3& pos, const Quat& q) {  // Node_Z local (quat, pos), no scale; column vectors
    simd_float4x4 m = simd_matrix4x4(simd_quaternion(q.x, q.y, q.z, q.w));
    m.columns[3] = simd_make_float4(pos.x, pos.y, pos.z, 1);
    return m;
}

simd_float4x4 rowsToSimd(const Mat4& w) {  // a row-vector (p * M) matrix as simd's column-vector form
    simd_float4x4 m;
    for (int c = 0; c < 4; ++c) m.columns[c] = simd_make_float4(float(w[size_t(c * 4)]), float(w[size_t(c * 4 + 1)]), float(w[size_t(c * 4 + 2)]), float(w[size_t(c * 4 + 3)]));
    return m;
}

}  // namespace

struct GaragePreview::Impl {
    id<MTLDevice> device;
    std::string dataDir;
    std::function<std::string(const std::string&)> dpcOf;
    std::vector<float> pp;  // pp001.tsc (array *0xa6749c)

    ResourceLibrary library;  // SHARED.DPC (CUBEMAP_GARAGE), PILOT.DPC, the vehicle DPCs as they are needed
    std::map<std::string, const DpcFile*> files;
    std::map<const DpcFile*, std::unique_ptr<VehicleAssembler>> assemblers;
    const DpcFile* pilot = nullptr;
    std::unique_ptr<Rider> rider;

    std::future<std::unique_ptr<MeshRenderer>> building;
    std::unique_ptr<MeshRenderer> r;
    bool sceneLook = false;  // the renderer's lights / cube are set
    int bound = 0;           // what the renderer's instances are: 0 none, 1 the vehicle, 2 the rider
    bool vehicleDirty = true, riderDirty = true;
    std::array<int, 6> upgrades{};  // [upgrades U3] setUpgradeStages (U5 draws them)
    int upgradesSet = 0;

    // An InGameObjects_Z scene (0x4cfd50 -> 0x4362c0): its viewport rectangle (from / to, normalised, before the
    // letterbox correction), its camera (FOV +0x818, near +0x728, far +0x72c / +0x730) and whether it is registered
    // for drawing (0x436020 on / off).
    struct Scene {
        bool live = false, on = false;
        float x0 = 0, y0 = 0, x1 = 1, y1 = 1;
        float fov = 1.0f, zn = 0.1f, zf = 100.0f;
        Vec3 camTarget{0, 0, 1};  // Camera_Z +0x820 (0x4d0270); the camera node stays at the origin
    };

    // InGame3DVehicle (+0xf0)
    struct Vehicle {
        Scene scene;
        bool visible = true;      // +0x40 (ctor 0x4d0d90)
        bool keepHidden = false;  // +0x39 (0x4cfd50: 0)
        std::string shown;        // +0x4c (the table index; the label here), empty = -1
        float angle = 0;          // +0x48 (ctor only: kept across vehicles and visits)
        Vec3 orbit, target;       // +0x64 (current), +0x70 (wanted)
        float pivotZ = 0;         // +0x44: the mean z of the physics spheres
        int applied = 0, pending = 0;  // livery holder +0x144 / +0x148 (ctor: 0)
        bool force = false;            // +0x14c
        simd_float3 paint = {0, 0, 0};
        Quat q;
        Vec3 pos;
        // the drawn vehicle (record +0x78 item): LOD1 parts in vehicle space, its DPC and name
        const DpcFile* dpc = nullptr;
        uint32_t nameCrc = 0;
        std::vector<MeshInstance> parts;
        // [upgrades U5] the assembled LOD1 record (the visible upgrades rebuild `parts` from it) and FUEL's framing inputs
        std::shared_ptr<AssembledVehicle> av;
        float s = 0, minY = 0;
        std::array<int, 6> shownStages{};  // the stages `parts` show
    } veh;
    std::map<std::string, std::vector<float>> vehicleParams;  // [upgrades U5] VehiclesParam.tsc (+ packs), read at first use

    // InGame3DPerso (+0x214)
    struct Perso {
        Scene scene;
        bool visible = true;  // +0x40 (ctor 0x4d0460)
        bool live = false;    // the IT_PILOT_SKIN agent +0x58
        float s = 0;          // its node object's +0x18
        Vec3 pos, target;     // node +0x20.., +0x48..+0x50
        float yaw = 0;        // +0x54 (ctor only)
        int mode = 0;         // +0x44
        PilotLook look;       // the outfit record +0x5c (page +0x270)
        RiderOutfit outfit{0, true};  // PHYSIQUE (port deviation): the custom rider's outfit (0 = FUEL's pilot parts)
        bool lookDirty = true;
        double animT = 0;     // slot 17's clock
        std::unique_ptr<Rider::SkinTemplate> tmpl;
        std::vector<uint8_t> vertices;
    } perso;

    Impl(id<MTLDevice> dev, std::string data, std::function<std::string(const std::string&)> f)
        : device(dev), dataDir(std::move(data)), dpcOf(std::move(f)) {
        pp = hud::loadPp((fs::path(dataDir) / "CommonComponent/Param/pp001.tsc").string());
        addFile("CommonComponent/DATAS/SHARED.DPC");
        pilot = addFile("CommonComponent/DATAS/PILOT.DPC");
        ResourceLibrary* lib = &library;
        building = std::async(std::launch::async, [dev, lib] {
            @autoreleasepool {
                auto m = std::make_unique<MeshRenderer>(dev, kSamples, *lib);
                m->showGround = false;  // no ground plane under single meshes
                m->dirt = 0.0f;         // 0x5dd980 / 0x5dd300: dirt 0 on both previews
                return m;
            }
        });
    }

    const DpcFile* addFile(const std::string& rel) {
        if (auto it = files.find(rel); it != files.end()) return it->second;
        const DpcFile* f = nullptr;
        try {
            auto d = std::make_unique<DpcFile>((fs::path(dataDir) / rel).string());
            f = d.get();
            library.add(std::move(d));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "garage preview: %s: %s\n", rel.c_str(), e.what());
        }
        files[rel] = f;
        return f;
    }

    bool ready(bool wait) {
        if (r) return true;
        if (!building.valid()) return false;
        if (!wait && building.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
        try {
            r = building.get();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "garage preview: renderer: %s\n", e.what());
        }
        return r != nullptr;
    }

    // ---- InGameObjects_Z (0x4cfd50): a new scene with the rectangle from (x0, y0) to (x1, y1), FOV 1.0 (desc +0x18),
    // near 0.1 (0x99648c), far 100 (0x9dbfc0), mode 6 (+0x34), lights 0x4cfec0 [O]. Unregistered until shown.
    static void sceneSetup(Scene& s, float x0, float y0, float x1, float y1) {
        s = Scene{};
        s.live = true, s.x0 = x0, s.y0 = y0, s.x1 = x1, s.y1 = y1;
    }

    // 0x4cfec0 after 0x68ffd0(PP63 x 3600): the scene's Atmo at that time (sun, Preetham colours, scatter from
    // Scatter_InitParams' defaults seen from the camera node at the origin); light block 1 (+0x268): DLIGHT_DIR,
    // DLIGHT_COLOR and DLIGHT_AMBIENT x 0.3 x PP64, +0x44 = 1.0; block 2 (+0x2b0): DSKY_DIR, DSKY_COLOR x 0.3 x PP65,
    // ambient 0 [O]. The colours of the port's shaders are those rows (sunColour = DLIGHT_COLOR x SC1.z).
    void setLights() {
        AtmosphereInput in;
        in.timeOfDay = double(P(pp, 63)) * 3600.0;
        in.weather.apply = false;  // the scene's own scatter parameters: Scatter_InitParams (ctor 0x68f910)
        in.camera = {0, 0, 0};
        const Atmosphere a = computeAtmosphere(in);
        const float k1 = 0.3f * P(pp, 64), k2 = 0.3f * P(pp, 65);
        auto f4 = [](const F3& v, float k, float w) { return simd_make_float4(v[0] * k, v[1] * k, v[2] * k, w); };
        MeshRenderer::PreviewScene& p = r->preview;
        p.lightDir = f4(a.lightDir, 1.0f, 1.0f);  // +0x2ac = 1.0: DLIGHT_INVINTENSITY
        p.skyDir = f4(a.skyDir, 1.0f, 0.0f);
        p.sunColour = f4(a.sunColour, k1, 1.0f);
        p.dlight = f4(a.dlightColor, k1, a.blueShift[3]);
        p.skyColour = f4(a.dskyColor, k2, 0.0f);
        p.skyAmbient = simd_make_float4(0, 0, 0, 0);  // +0x2e8..+0x2f0 = 0 (x PP65)
        for (size_t i = 0; i < 11; ++i) p.scat[i] = simd_make_float4(a.c[i][0], a.c[i][1], a.c[i][2], a.c[i][3]);
        // Mode 6: the environment cube is the MaterialAnim_Z named CUBEMAP_GARAGE (SHARED.DPC), its Material_Z's cube
        // (0x669160 "CUBEMAP_GARAGE" -> 0x67a6c0 -> material -> vt+0xb0); none: the cube flag off (+0xd28 = 0) [O].
        p.envCube = nil;
        for (const auto& f : library.files()) {
            for (uint32_t id : f->ids(cls::MaterialAnimZ)) {
                const DpcFile::Entry* e = f->find(id);
                const std::vector<uint8_t> link = f->link(*e);
                uint32_t name = 0;
                if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                if (name != asoboCrc("CUBEMAP_GARAGE")) continue;
                try {
                    const MaterialAnim ma = parseMaterialAnimBody(f->body(*e).value());
                    if (auto hit = library.find(ma.material, cls::Material)) {
                        const Material m = parseMaterialBody(hit.file->body(*hit.entry).value());
                        for (uint32_t bmp : m.textures)
                            if (bmp && !p.envCube) p.envCube = r->cubeTexture(bmp);
                    }
                } catch (const std::exception& ex) {
                    std::fprintf(stderr, "garage preview: CUBEMAP_GARAGE: %s\n", ex.what());
                }
            }
            if (p.envCube) break;
        }
        if (!p.envCube) std::fprintf(stderr, "garage preview: no CUBEMAP_GARAGE cube (reflections off)\n");
        sceneLook = true;
    }

    // ---- InGame3DVehicle

    // 0x4d1020 (from 0x4781c0): the orbit vectors +0x64 / +0x70 = 0 (0xa66208 is never written: zero), the scene with
    // (PP68, PP70)-(PP69, PP71) = (0.4, 0)-(1.0, 0.9), its camera FOV +0x818 = PP66 (1.65 rad) [O].
    void vehicleSetup() {
        veh.orbit = veh.target = Vec3{};
        sceneSetup(veh.scene, P(pp, 68), P(pp, 70), P(pp, 69), P(pp, 71));
        veh.scene.fov = P(pp, 66);
        veh.keepHidden = false;
    }

    // 0x4d1110(index): the shown vehicle. A new one: the old item goes back to the pool, the vehicle record's item
    // +0x78 is taken (the drawn vehicle, LOD flag 0x200000: always LOD1 in its own viewport), attached to the scene;
    // the record's physics item +0x74 (the "<variant>_DYN" Lod_Z) gives the framing: s = its object +0x18, pivot
    // +0x44 = the mean z of its spheres (+0x6c / +0x70, 0x18 bytes: centre, radius, ...), minY = min(y - r) (from
    // 10000, 0xa2a4e0); +0x70 = (0, -PP62 s / PP67 - minY, s / PP67) (0xa06350 = (0, 0, 1)), the new node at the old
    // +0x70; +0x64 = +0x70 when it was still zero (the first vehicle snaps, the next ones glide) [O].
    void setVehicle(const std::string& label) {
        if (label == veh.shown) return;
        veh.shown.clear(), veh.parts.clear(), veh.dpc = nullptr;  // 0x4f1b60: back to the pool
        vehicleDirty = true;
        if (label.empty()) return;
        veh.shown = label;
        const DpcFile* dpc = addFile(dpcOf(label));
        if (!dpc) return;
        auto& as = assemblers[dpc];
        if (!as) as = std::make_unique<VehicleAssembler>(*dpc);
        const VehicleVariant* v = as->find(label);
        if (!v) {
            std::fprintf(stderr, "garage preview: no variant %s in %s\n", label.c_str(), dpc->path().c_str());
            return;
        }
        veh.av.reset(), veh.shownStages = {};
        try {
            const AssembledVehicle av = as->assemble(*v, 1);
            for (const PlacedPart& p : av.parts)
                if (p.drawn()) veh.parts.push_back({p.meshId, rowsToSimd(p.world)});
            veh.av = std::make_shared<AssembledVehicle>(av);  // [upgrades U5]
        } catch (const std::exception& e) {
            std::fprintf(stderr, "garage preview: %s: %s\n", label.c_str(), e.what());
        }
        veh.dpc = dpc, veh.nameCrc = v->name;
        float s = 0, minY = 10000.0f, sumZ = 0;
        int n = 0;
        if (const DpcFile::Entry* e = dynLod(*dpc, label)) {
            s = objectRadius(*dpc, *e);
            try {
                for (const ProxySphere& sp : findVehicleProxy(*dpc, label).spheres) {
                    sumZ += sp.centre[2], ++n;
                    minY = std::min(minY, sp.centre[1] - sp.radius);
                }
            } catch (const std::exception&) {
            }
        } else {
            std::fprintf(stderr, "garage preview: %s has no %s_DYN\n", label.c_str(), label.c_str());
        }
        veh.pivotZ = n > 0 ? sumZ / float(n) : 0.0f;  // (FUEL divides by the count unguarded)
        veh.s = s, veh.minY = minY;  // [upgrades U5]
        const float k = 1.0f / P(pp, 67);
        const Vec3 old = veh.target;
        veh.target = Vec3{0.0f * s * k, (-P(pp, 62) * (1.0f * s * k) - minY) + 0.0f * s * k, 1.0f * s * k};
        veh.pos = old;  // node +0x20.. = the old +0x70
        if (std::fabs(veh.orbit.x) < 1e-6f && std::fabs(veh.orbit.y) < 1e-6f && std::fabs(veh.orbit.z) < 1e-6f) veh.orbit = veh.target;
        showUpgrades();  // [upgrades U5] the stages already set (the menu sends them after LoadVehicle too)
    }

    // ---- [upgrades U5] the shown vehicle with the stages (core/upgrade_visuals): the parts at the physics' rest stance (the
    // body `lift` higher, the wheels out by the widening) with the procedural parts and the carbon; the framing grows by
    // the lift / widening (the bound radius s of FUEL's +0x70). Stock stages, upgrades off or no setup: FUEL's parts.
    void showUpgrades() {
        if (!veh.av || veh.dpc == nullptr) return;
        if (veh.shownStages == upgrades && !veh.parts.empty() && veh.parts.size() != 0 && upgradesShownFor == veh.shown) return;
        upgradesShownFor = veh.shown;
        veh.shownStages = upgrades;
        const AssembledVehicle& av = *veh.av;
        std::vector<MeshInstance> parts;
        float grow = 0;
        bool any = false;
        for (int x : upgrades) any = any || x > 0;
        if (any && upg::enabled() && upgvis::enabled()) try {
                if (vehicleParams.empty())
                    vehicleParams = parseVehiclesParam(custom::readScript((std::filesystem::path(dataDir) / "CommonComponent/Param/VehiclesParam.tsc").string()));
                const auto it = vehicleParams.find(veh.shown);
                if (it != vehicleParams.end()) {
                    std::vector<std::array<float, 3>> hubs;
                    for (const PlacedPart& p : av.parts)
                        if (p.drawn() && (p.placement == Placement::Wheel || p.placement == Placement::WheelFlipped))
                            hubs.push_back({float(p.world[12]), float(p.world[13]), float(p.world[14])});
                    VehicleSetup setup = makeVehicleSetup(*veh.dpc, veh.shown, it->second, hubs);
                    upg::Stages st;
                    for (int c = 0; c < upg::kCategories; ++c) st[c] = upgrades[size_t(c)];
                    upg::apply(setup, st, fv::FuelVehicle::powerScale);
                    const upgvis::Plan plan = upgvis::makePlan(av, setup);
                    if (plan.active) {
                        for (const upgvis::ScenePart& sp : upgvis::restScene(av, setup, plan)) parts.push_back({sp.mesh, rowsToSimd(sp.world)});
                        grow = std::max({plan.lift, plan.widenFront, plan.widenRear});
                    }
                    if (std::getenv("FUEL_PK20_TRACE") || std::getenv("FUEL_U5_TRACE"))
                        std::printf("garage preview: %s stages %s: %s\n", veh.shown.c_str(), upg::formatStages(st).c_str(), upgvis::describe(plan).c_str());
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "garage preview: upgrades of %s: %s\n", veh.shown.c_str(), e.what());
                parts.clear();
            }
        if (parts.empty())
            for (const PlacedPart& p : av.parts)
                if (p.drawn()) parts.push_back({p.meshId, rowsToSimd(p.world)});
        veh.parts = std::move(parts);
        vehicleDirty = true;
        const float s = veh.s + grow, k = 1.0f / P(pp, 67);  // FUEL's framing on the widened bound
        veh.target = Vec3{0.0f, -P(pp, 62) * (s * k) - veh.minY, s * k};
    }
    std::string upgradesShownFor;
    // ---- [upgrades U5] end

    bool liveryAvailable(int k) const {  // 0x5dd6f0: Vehicle_G::BindLivery lookup on the drawn object
        if (!veh.dpc || veh.parts.empty() || k < 0) return false;
        return findLiveryTextures(*veh.dpc, veh.nameCrc, k + 1)[0] != 0;
    }
    // 0x5dd790: a request; the holder keeps its livery when the vehicle has no such livery [O].
    void requestLivery(int k, bool force) {
        if (liveryAvailable(k)) veh.pending = k, veh.force = force;
        else veh.pending = veh.applied;
    }
    // 0x5dd7c0: commit when pending != applied or forced: BindLivery + 0x6a2c30 (the LIVERY_XX_LOD<n> materials take
    // the livery's textures), applied = pending, force = 0; 1 when there is nothing to do, 0 on failure [O].
    bool commitLivery() {
        if (veh.pending == veh.applied && !veh.force) return true;
        if (!liveryAvailable(veh.pending)) return false;
        veh.applied = veh.pending, veh.force = false;
        vehicleDirty = true;
        return true;
    }

    // InGame3DVehicle vf01 0x4d0df0: with a live item and the livery committed: the scene on (0x436020) unless +0x39;
    // +0x48 += dt; q = axis-angle(X, -PP72) x axis-angle(Y, +0x48) (Quat_Mul 0x410260 = Hamilton(2nd, 1st)); +0x64
    // smoothed to +0x70 (Cam_SmoothV3, 1.1); node = (+0x64 - q (0, 0, +0x44), q); visible by +0x40; the camera target
    // +0x820 = axis-angle(Y, PP73) (0, 0, 1) (0x4d0270) [O].
    void vehicleTick(float dt) {
        if (veh.shown.empty() || veh.parts.empty()) return;
        if (!commitLivery()) return;
        if (!veh.keepHidden) veh.scene.on = true;
        veh.angle += dt;
        veh.q = fv::mul(fv::axisAngle({1, 0, 0}, -P(pp, 72)), fv::axisAngle({0, 1, 0}, veh.angle));
        veh.orbit = cam::smoothV(veh.orbit, veh.target, 1.1f, dt);
        veh.pos = veh.orbit - fv::rotate(veh.q, Vec3{0.0f * veh.pivotZ, 0.0f * veh.pivotZ, 1.0f * veh.pivotZ});
        veh.scene.camTarget = fv::rotate(fv::axisAngle({0, 1, 0}, P(pp, 73)), Vec3{0, 0, 1});
    }

    // vf00 0x4d10f0: 0x4d1110(-1) then InGame3DBase vf00 0x4d01b0 (the scene off and released) [O].
    void vehicleRelease() {
        setVehicle("");
        veh.scene = Scene{};
    }

    // ---- InGame3DPerso

    // 0x4d0720: the scene with (0.2, 0)-(1, 1) (0x9dbfa8); the pooled IT_PILOT_SKIN agent -> +0x58; when live: its
    // node into the scene at (0, -0.45 s, 1.5 s) = +0x48..+0x50 (s = its object +0x18), the scene on, its
    // manipulator registered, slot 0x11 played (0x667a00), the outfit record defaults (0x5dc2b0 / 0x5dc9b0), dirt 0.
    // Then 0x471010: mode 0 with framing PP300-302 (0x4d0ad0), the scene off, the outfit 0x471570, vf01(0) [O].
    void riderSetup(const PilotLook& look, const RiderOutfit& outfit = {0, true}) {
        sceneSetup(perso.scene, 0.2f, 0.0f, 1.0f, 1.0f);
        if (!rider && pilot) try {
                rider = std::make_unique<Rider>(*pilot);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "garage preview: rider: %s\n", e.what());
            }
        perso.live = rider != nullptr && pilot != nullptr && pilot->find(kPilotLodId) != nullptr;
        if (!perso.live) return;
        perso.s = objectRadius(*pilot, *pilot->find(kPilotLodId));
        perso.pos = perso.target = Vec3{perso.s * 0.0f, perso.s * -0.45000002f, perso.s * 1.5f};
        perso.animT = 0;
        perso.scene.on = true;   // 0x436020 AL 1 (0x4d0720) ..
        setMode(0);              // 0x471010: +0x44 = 0, 0x4d0ad0(300, 301, 302)
        perso.scene.on = false;  // .. AL 0 (0x471010)
        setLook(look, outfit);   // 0x471570
        riderTick(0);
    }
    // 0x4d0ad0(a, b, c) through the mode setters 0x4d0bb0 (0: PP300-302), 0x4d0bf0 (1: 306-308), 0x4d0c10 (2: 309-311),
    // 0x4d0bd0 (3: 303-305), 0x4d0b90 (4: 312-314): +0x48..+0x50 = PP[a..c] x s [O].
    void setMode(int m) {
        perso.mode = m;
        static const int kFirst[5] = {300, 306, 309, 303, 312};
        if (m < 0 || m > 4 || !perso.live) return;
        const int a = kFirst[m];
        perso.target = Vec3{P(pp, a) * perso.s, P(pp, a + 1) * perso.s, P(pp, a + 2) * perso.s};
    }
    void setLook(const PilotLook& look, const RiderOutfit& outfit) {
        bool same = outfit == perso.outfit;  // PHYSIQUE
        for (int k = 0; k < 8; ++k) same = same && look.at(k) == perso.look.at(k);
        if (same && perso.tmpl) return;
        perso.look = look;
        perso.outfit = outfit;
        perso.lookDirty = true;
    }
    // vf01 0x4d04d0: when the agent is live: the mode's framing again, PilotLivery_ApplyAll (0x5dcfe0), the agent's
    // manipulator step 1/60 (+0x24 = 0x3c888889), +0x54 += dt, node quat = axis-angle(Y, +0x54), node pos smoothed
    // to +0x48 (Cam_SmoothV3, 1.1), visible by +0x40 [O].
    void riderTick(float dt) {
        if (!perso.live) return;
        setMode(perso.mode);
        perso.yaw += dt;
        perso.pos = cam::smoothV(perso.pos, perso.target, 1.1f, dt);
    }
    void riderRelease() {  // vf00 0x4d0a00: the agent back to the pool, InGame3DBase vf00 (scene off)
        perso.live = false;
        perso.scene = Scene{};
    }

    // ---- drawing

    // 0x7c9eb0 with the rectangle 0x4cfd50 made: x0, y0 + lb, x1 - x0, (y1 - y0) - 2 lb, lb = the letterbox bar over
    // the screen height (Menu_CanvasRect 0x49bec0: canvas height min(H, W x 9 / 16) centred); must fit in 0..1; pixels
    // by truncation (_ftol), w / h at least 1. Tangents: t = tan(FOV / 2), a = 0.75 w / h; w > h: x a t, y 0.75 t;
    // else x 2 a t, y 0.75 t (the renderer aspect [0xa2e8f4] / +0x1260 taken as 1) [O].
    static bool sceneRect(const Scene& s, NSUInteger W, NSUInteger H, MTLRegion& out, float& tanX, float& tanY) {
        const float ch = std::min(float(H), float(W) * 9.0f / 16.0f), bar = (float(H) - ch) * 0.5f;
        const float lb = bar / (bar * 2.0f + ch);
        float rx = s.x0, ry = s.y0 + lb, rw = s.x1 - s.x0, rh = (s.y1 - s.y0) - lb * 2.0f;
        // [port] a target wider than 16:9: the port's menus are pillarboxed (UiRenderer::begin; FUEL stretches them to
        // the full width), so the scene keeps its place in the 16:9 canvas.
        if (float(H) < float(W) * 0.5625f) {
            const float cw = float(H) / 0.5625f, off = (float(W) - cw) * 0.5f;
            rx = (off + rx * cw) / float(W), rw = rw * cw / float(W);
        }
        if (!(rx + rw <= 1.0f && ry + rh <= 1.0f && rx >= 0.0f && ry >= 0.0f)) return false;
        const int x = int(float(W) * rx), y = int(float(H) * ry);
        const int w = std::max(1, int(float(W) * rw)), h = std::max(1, int(float(H) * rh));
        if (x + w > int(W) || y + h > int(H)) return false;
        out = MTLRegionMake2D(NSUInteger(x), NSUInteger(y), NSUInteger(w), NSUInteger(h));
        const float t = std::tan(s.fov * 0.5f), a = float(w) / float(h) * 0.75f;
        tanX = w > h ? a * t : a * t * 2.0f;
        tanY = t * 0.75f;
        return true;
    }

    void drawScene(id<MTLCommandBuffer> cb, id<MTLTexture> target, const Scene& s) {
        MTLRegion rect;
        float tx = 1, ty = 1;
        if (!sceneRect(s, target.width, target.height, rect, tx, ty)) return;
        r->preview.on = true;
        r->preview.tanX = tx, r->preview.tanY = ty;
        r->preview.zNear = s.zn, r->preview.zFar = s.zf;
        OrbitCamera cam;
        cam.explicitPose = true;
        cam.eye = simd_make_float3(0, 0, 0);  // InGameCameraNode at the origin (0x68fe30: identity)
        cam.target = simd_make_float3(s.camTarget.x, s.camTarget.y, s.camTarget.z);
        cam.zNear = s.zn;
        r->renderPreview(cb, target, rect, cam);
        if (std::getenv("FUEL_PK20_TRACE"))
            std::printf("pk20: scene rect %lu,%lu %lux%lu tan %.3f %.3f | veh pos %.3f %.3f %.3f pivot %.3f angle %.3f | rider pos %.3f %.3f %.3f s %.3f yaw %.3f mode %d\n",
                        (unsigned long)rect.origin.x, (unsigned long)rect.origin.y, (unsigned long)rect.size.width, (unsigned long)rect.size.height, tx, ty,
                        veh.pos.x, veh.pos.y, veh.pos.z, veh.pivotZ, veh.angle, perso.pos.x, perso.pos.y, perso.pos.z, perso.s, perso.yaw, perso.mode);
    }

    void bindVehicle() {
        if (bound == 1 && !vehicleDirty) return;
        Paint p;
        p.color = veh.paint;
        p.livery = findLiveryTextures(*veh.dpc, veh.nameCrc, veh.applied + 1);
        r->setPaint(p);
        std::vector<MeshInstance> inst = veh.parts;
        std::string err;
        r->setScene(inst, &err);
        if (!err.empty()) std::fprintf(stderr, "garage preview: %s: %s\n", veh.shown.c_str(), err.c_str());
        bound = 1, vehicleDirty = false;
    }
    void bindRider() {
        if (perso.lookDirty || !perso.tmpl) {
            Rider::Parts parts;
            int* f = &parts.head;
            for (int k = 0; k < 8; ++k) f[k] = std::max(1, perso.look.at(k));
            try {
                const CustomRider* cr = perso.outfit.outfit > 0 ? CustomRider::shared() : nullptr;  // PHYSIQUE
                perso.tmpl = std::make_unique<Rider::SkinTemplate>(cr ? cr->prepare(*rider, perso.outfit, parts, 0) : rider->prepareSkin(parts));
            } catch (const std::exception& e) {  // a part the DPC lacks: the default rider (as RiderView)
                std::fprintf(stderr, "garage preview: rider parts: %s\n", e.what());
                perso.tmpl = std::make_unique<Rider::SkinTemplate>(rider->prepareSkin(Rider::Parts{}));
            }
            BakedMesh mesh = perso.tmpl->mesh;
            rider->skin(*perso.tmpl, pose(), mesh.vertices);
            std::string err;
            if (!r->addBakedMesh(kRiderKey, mesh, &err)) std::fprintf(stderr, "garage preview: rider: %s\n", err.c_str());
            perso.vertices = mesh.vertices;
            perso.lookDirty = false;
            riderDirty = true;
        } else {
            rider->skin(*perso.tmpl, pose(), perso.vertices);
            r->updateBakedVertices(kRiderKey, perso.vertices);
        }
        if (bound != 2 || riderDirty) {
            std::string err;
            r->setScene({MeshInstance{kRiderKey, matrix_identity_float4x4}}, &err);
            bound = 2, riderDirty = false;
        }
    }
    std::vector<Mat4> pose() const {
        double t = perso.animT;
        if (rider->hasSlot(kRiderSlot)) {
            const double d = rider->duration(kRiderSlot);
            if (rider->loops(kRiderSlot) && d > 0) t = std::fmod(t, d);
            return rider->pose(kRiderSlot, t);
        }
        return rider->pose(rider->ridingSlot("QUAD"), 0.0);
    }
};

GaragePreview::GaragePreview(id<MTLDevice> device, const std::string& dataDir, std::function<std::string(const std::string&)> dpcOf)
    : d_(std::make_unique<Impl>(device, dataDir, std::move(dpcOf))) {}

GaragePreview::~GaragePreview() {
    if (d_ && d_->building.valid()) d_->building.wait();
}

void GaragePreview::waitReady() { d_->ready(true); }

// [upgrades U3] a stub: the stages are stored for U5 (visible upgrade parts), which draws them on the preview vehicle.
void GaragePreview::setUpgradeStages(const std::array<int, 6>& stages) {
    d_->upgrades = stages;
    ++d_->upgradesSet;
    // [upgrades U5] FUEL_GARAGE_UPGRADES=e,s,t,w,np,nt forces the shown stages (shots of any garage page)
    if (const char* e = std::getenv("FUEL_GARAGE_UPGRADES")) {
        upg::Stages st;
        if (upg::parseStages(e, st))
            for (int c = 0; c < upg::kCategories; ++c) d_->upgrades[size_t(c)] = st[c];
    }
    for (int& x : d_->upgrades) x = std::clamp(x, 0, 3);
    d_->showUpgrades();  // [upgrades U5] the visible upgrades on the preview vehicle
}
const std::array<int, 6>& GaragePreview::upgradeStages() const { return d_->upgrades; }
int GaragePreview::upgradeStagesSet() const { return d_->upgradesSet; }

int GaragePreview::scenesOn() const {
    return (d_->veh.scene.live && d_->veh.scene.on ? 1 : 0) | (d_->perso.scene.live && d_->perso.scene.on ? 2 : 0);
}

void GaragePreview::run(const std::vector<ui::Menu::Garage3DCmd>& cmds) {
    using C = ui::Menu::Garage3DCmd;
    Impl& d = *d_;
    for (const C& c : cmds) {
        switch (c.kind) {
            case C::Setup:
                d.vehicleSetup();
                d.riderSetup(c.look, c.outfit);
                break;
            case C::Release:
                d.riderRelease();
                d.vehicleRelease();
                break;
            case C::Restart:  // 0x478ec0: 0x436020 AL 0 on +0xf4 and +0x218
                d.veh.scene.on = false;
                d.perso.scene.on = false;
                break;
            case C::LoadVehicle:
                d.setVehicle(c.vehicle);  // 0x4d1110 (0x4cfec0's lights are the scene's, set once)
                d.requestLivery(c.livery, true);
                if (!d.veh.parts.empty()) d.veh.paint = simd_make_float3(c.rgb[0], c.rgb[1], c.rgb[2]), d.vehicleDirty = true;
                break;
            case C::ShowRider: d.perso.scene.on = d.perso.scene.live; break;
            case C::Livery: d.requestLivery(c.livery, true); break;  // 0x4799c0
            case C::Paint:  // 0x479a00 -> 0x5dd980: on the holder's live object only
                if (!d.veh.parts.empty()) d.veh.paint = simd_make_float3(c.rgb[0], c.rgb[1], c.rgb[2]), d.vehicleDirty = true;
                break;
            case C::Look: d.setLook(c.look, c.outfit); break;
            case C::PoseMode: d.setMode(c.mode); break;
            case C::TickVehicle: d.vehicleTick(c.dt); break;
            case C::TickRider: d.riderTick(c.dt); break;
            case C::Upgrades: setUpgradeStages(c.stages); break;  // [upgrades U3]
        }
    }
    // The rider agent's manipulator runs every frame while it is registered: one 1/60 s step (+0x24) [O].
    if (d.perso.live) d.perso.animT += 1.0 / 60.0;
}

void GaragePreview::draw(id<MTLCommandBuffer> cb, id<MTLTexture> target) {
    Impl& d = *d_;
    const bool vOn = d.veh.scene.live && d.veh.scene.on && d.veh.visible && !d.veh.parts.empty();
    const bool rOn = d.perso.scene.live && d.perso.scene.on && d.perso.live && d.perso.visible;
    if (!vOn && !rOn) return;
    if (!d.ready(false)) return;
    if (!d.sceneLook) d.setLights();
    if (vOn) {
        d.bindVehicle();
        std::vector<simd_float4x4> worlds;  // node x part (setInstanceWorlds takes each instance's full world)
        worlds.reserve(d.veh.parts.size());
        const simd_float4x4 node = nodeMatrix(d.veh.pos, d.veh.q);
        for (const MeshInstance& p : d.veh.parts) worlds.push_back(simd_mul(node, p.world));
        d.r->setInstanceWorlds(worlds);
        d.drawScene(cb, target, d.veh.scene);
    }
    if (rOn) {
        d.bindRider();
        d.r->setInstanceWorlds({nodeMatrix(d.perso.pos, fv::axisAngle({0, 1, 0}, d.perso.yaw))});
        d.drawScene(cb, target, d.perso.scene);
    }
}

// ---- PK20: the MeshRenderer side of the preview (declared in renderer.h; kept here with the rest of the preview)

id<MTLTexture> MeshRenderer::cubeTexture(uint32_t bitmapId) {
    auto hit = library_.find(bitmapId, cls::Bitmap);
    if (!hit) return nil;
    try {
        const BitmapInfo bm = parseBitmapLink(hit.file->link(*hit.entry));
        const auto body = hit.file->body(*hit.entry);
        if (!body || bm.type != BitmapInfo::Cube || !checkBitmap(bm, uint32_t(body->size())).empty() || bm.width != bm.height) return nil;
        MTLPixelFormat pf = MTLPixelFormatBGRA8Unorm;  // sampled as stored (the shaders light in gamma space)
        if (bm.format == PixelFormat::DXT1) pf = MTLPixelFormatBC1_RGBA;
        else if (bm.format == PixelFormat::DXT3) pf = MTLPixelFormatBC2_RGBA;
        else if (bm.format == PixelFormat::DXT5) pf = MTLPixelFormatBC3_RGBA;
        MTLTextureDescriptor* td = [MTLTextureDescriptor textureCubeDescriptorWithPixelFormat:pf size:bm.width mipmapped:bm.mips > 1];
        td.mipmapLevelCount = bm.mips;
        id<MTLTexture> t = [device_ newTextureWithDescriptor:td];
        const std::vector<uint32_t> sizes = bm.mipSizes();
        size_t offset = 0;
        for (NSUInteger face = 0; face < 6; ++face) {  // D3D face order +X -X +Y -Y +Z -Z = Metal's slices
            uint32_t w = bm.width;
            for (uint32_t level = 0; level < bm.mips; ++level) {
                const uint8_t* src = body->data() + offset;
                if (bm.isBlockCompressed()) {
                    const uint32_t blockBytes = bm.format == PixelFormat::DXT1 ? 8 : 16;
                    const NSUInteger row = std::max(1u, (w + 3) / 4) * blockBytes;
                    [t replaceRegion:MTLRegionMake2D(0, 0, w, w) mipmapLevel:level slice:face withBytes:src bytesPerRow:row bytesPerImage:row * std::max(1u, (w + 3) / 4)];
                } else if (bm.format == PixelFormat::A8R8G8B8) {  // B, G, R, A in memory
                    [t replaceRegion:MTLRegionMake2D(0, 0, w, w) mipmapLevel:level slice:face withBytes:src bytesPerRow:w * 4 bytesPerImage:w * w * 4];
                } else {
                    return nil;
                }
                offset += sizes[level];
                w = std::max(1u, w >> 1);
            }
        }
        return t;
    } catch (const std::exception&) {
        return nil;
    }
}

namespace {
const char* kPreviewComposite = R"(
#include <metal_stdlib>
using namespace metal;
struct PvOut { float4 position [[position]]; };
vertex PvOut vsPk20Composite(uint vid [[vertex_id]]) {
    const float2 p[3] = {float2(-1, -1), float2(3, -1), float2(-1, 3)};
    PvOut o;
    o.position = float4(p[vid], 0.0, 1.0);
    return o;
}
// The scene (premultiplied over a clear 0 colour) over the menu target: the shader colours clamped as an 8-bit back
// buffer keeps them, through the inverse sRGB curve when the target encodes sRGB (the UI does the same).
fragment float4 fsPk20Composite(PvOut in [[stage_in]], texture2d<float> scene [[texture(0)]], constant float4& u [[buffer(0)]]) {
    const float4 c = scene.read(uint2(in.position.xy - u.xy));
    if (c.a <= 0.0) discard_fragment();
    float3 v = saturate(c.rgb / c.a);
    if (u.z > 0.0) v = select(pow((v + 0.055) / 1.055, 2.4), v / 12.92, v <= 0.04045);
    return float4(v, saturate(c.a));
}
)";
}  // namespace

void MeshRenderer::renderPreview(id<MTLCommandBuffer> cb, id<MTLTexture> target, MTLRegion rect, const OrbitCamera& cam) {
    if (!preview.on || instances_.empty() || rect.size.width == 0 || rect.size.height == 0) return;
    if (!toOptimize_.empty()) {
        id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
        for (id<MTLTexture> t : toOptimize_) [blit optimizeContentsForGPUAccess:t];
        [blit endEncoding];
        toOptimize_.clear();
    }
    if (frame_ == 0) epoch_ = std::chrono::steady_clock::now();
    cpu_ = CpuTimes{};
    const NSUInteger w = rect.size.width, h = rect.size.height;
    ensureTargets(w, h);
    scaledFrame_ = false, cutFrame_ = false, lastScale_ = 1.0f;
    viewW_ = w, viewH_ = h;
    // The scene pass: colour cleared to 0 with alpha 0 (only the depth is cleared over the menu: the composite below
    // leaves the uncovered pixels as they are), depth 1.0.
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdrMsaa_;
    rp.colorAttachments[0].resolveTexture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionClear;
    rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 0);
    rp.colorAttachments[0].storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.texture = depthMsaa_;
    rp.depthAttachment.resolveTexture = depth_;
    rp.depthAttachment.loadAction = MTLLoadActionClear;
    rp.depthAttachment.clearDepth = 1.0;
    rp.depthAttachment.storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.depthResolveFilter = MTLMultisampleDepthResolveFilterMin;
    lastFrame_.reset();
    frameCb_ = cb;
    frameRp_ = rp;
    encode(nil, float(w) / float(h), cam);
    frameCb_ = nil;
    frameRp_ = nil;
    fxEnd();
    ++frame_;

    // The composite into the target's rectangle.
    const NSUInteger fmt = NSUInteger(target.pixelFormat);
    id<MTLRenderPipelineState> ps = previewComposite_[fmt];
    if (ps == nil) {
        NSError* err = nil;
        if (previewLib_ == nil) previewLib_ = [device_ newLibraryWithSource:@(kPreviewComposite) options:nil error:&err];
        if (previewLib_ == nil) {
            std::fprintf(stderr, "garage preview: composite shader: %s\n", err.localizedDescription.UTF8String);
            return;
        }
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
        pd.vertexFunction = [previewLib_ newFunctionWithName:@"vsPk20Composite"];
        pd.fragmentFunction = [previewLib_ newFunctionWithName:@"fsPk20Composite"];
        pd.colorAttachments[0].pixelFormat = target.pixelFormat;
        pd.colorAttachments[0].blendingEnabled = YES;
        pd.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        pd.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        pd.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
        pd.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
        if (ps == nil) {
            std::fprintf(stderr, "garage preview: composite: %s\n", err.localizedDescription.UTF8String);
            return;
        }
        previewComposite_[fmt] = ps;
    }
    MTLRenderPassDescriptor* cp = [MTLRenderPassDescriptor renderPassDescriptor];
    cp.colorAttachments[0].texture = target;
    cp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    cp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:cp];
    [enc setViewport:MTLViewport{double(rect.origin.x), double(rect.origin.y), double(w), double(h), 0.0, 1.0}];
    [enc setScissorRect:MTLScissorRect{rect.origin.x, rect.origin.y, w, h}];
    [enc setRenderPipelineState:ps];
    [enc setFragmentTexture:hdr_ atIndex:0];
    const bool srgb = target.pixelFormat == MTLPixelFormatBGRA8Unorm_sRGB || target.pixelFormat == MTLPixelFormatRGBA8Unorm_sRGB;
    const simd_float4 u = simd_make_float4(float(rect.origin.x), float(rect.origin.y), srgb ? 1.0f : 0.0f, 0.0f);
    [enc setFragmentBytes:&u length:sizeof u atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc endEncoding];
}

}  // namespace fuel
