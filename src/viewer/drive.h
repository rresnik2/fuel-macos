// Drive mode: assembled vehicles on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "core/upgrade_visuals.hpp"  // [upgrades U5]
#include "core/susp_anim.hpp"        // [suspanim]
#include "core/automesh.hpp"
#include "core/roads.hpp"
#include "core/vegetation.hpp"
#include "core/world.hpp"
#include "core/dyn_object.hpp"
#include "core/anim_trigger.hpp"
#include "physics/fuel_camera.hpp"
#include "physics/vehicle_physics.hpp"
#include "core/sfx.hpp"
#include "core/blob_shadow.hpp"
#include "renderer.h"

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <future>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <deque>
#include <vector>

class RiderView;
// The riders' animation and skinning (rider_view.h); set by the viewer, null = static riders.
extern RiderView* gRiderView;

// ---- P104: FUEL's vehicle LOD (notes p104-vehicle-lod.md). A non-player vehicle draws its Lod_Z level by distance
// (Lod_Z::Draw 0x884170) with LodData flag 0x400000 (never LOD1: AI racers, TRAFFIC_TM) or none (the FLOTTE fleet
// traffic, 0x5c7fe0); its rider by the pilot Lod_Z's rule. The player (no VehicleLods) stays at LOD1 (flag 0x20000).
struct VehicleLods {
    std::vector<fuel::AssembledVehicle> lower;  // the variant at LOD2 .. LODn (VehicleAssembler::assemble(v, 2..))
    int minLevel = 1;                           // 1: LodData 0x400000; 0: the FLOTTE exception
    std::vector<uint32_t> riderLower;           // the rider's LOD2.. mesh keys (RiderView::lowerLevels)
    int riderLevels = 1;
    float riderD = 1, riderView = 0;            // the pilot Lod_Z's D and view distance
    std::array<double, 3> seat{};               // the rider node (riderSeatPoint), vehicle space
};
// FUEL_VEHICLE_LOD=0: every vehicle at LOD1 (the pre-P104 picture; perf A/B).
bool vehicleLodEnabled();
// ---- P104 end

// The drawn parts of one vehicle and how they follow its physics state.
class VehicleRig {
public:
    // `riders`: extra meshes (the rider, in vehicle space) that follow the chassis; animated when gRiderView
    // knows them (RiderView::animated). `lods` (P104): the lower levels of a non-player vehicle.
    VehicleRig(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders,
               const VehicleLods* lods = nullptr);
    // The parts in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts() const { return parts_; }
    const fuel::VehicleSetup& setup() const { return setup_; }
    // The rider for this frame (FUEL's rider state machine, rider_anim.hpp).
    void animate(float dt, const fuel::DriveInput& input, const fuel::VehicleState& st, const fuel::fv::FuelVehicle* model, bool player,
                 int raceState, const fuel::Heightfield& terrain);
    // Skins the animated rider into the renderer's mesh.
    void uploadRider() const;
    void riderGesture(int base);  // PK31: an overtake one-shot (0x5c2370) on the rider
    void riderKillMission();      // PK14: 0x62baa0's +0x240 / +0x244 = 47
    // World matrices for the parts, in parts() order.
    std::vector<simd_float4x4> partWorlds(const fuel::VehicleState& st) const;
    // P85: part k rides on the chassis (a V:LINK* part of the skeleton: not a wheel, not the rider).
    bool chassisPart(size_t k) const { return k < links_.size() && links_[k].wheel < 0 && !links_[k].rider; }
    // P104: 0x6a2480 dents only LOD record 0's parts (each record clones its node with fresh object data).
    bool damageable(size_t k) const { return chassisPart(k) && links_[k].level <= 0; }
    // ---- P104: this frame's levels from the camera eye (Lod_Z::Draw 0x884170 + ComputeFade 0x672950), and each
    // part's alpha (0 = not drawn: another level, or faded out), in parts() order.
    void updateLod(const fuel::VehicleState& st, simd_float3 eye);
    std::vector<float> partAlphas(bool lod1 = false) const;  // lod1: as without P104 (every part of LOD1, the LOD1 rider)
    bool lodSwitched() const { return lodOn_; }
    int level() const { return level_; }
    int riderLevel() const { return riderLevel_; }
    float alpha() const { return alpha_; }
    float riderAlpha() const { return riderAlpha_; }
    simd_float3 lodAnchor() const { return anchor_; }
    simd_float3 riderAnchor() const { return riderAnchor_; }
    // ---- P104 end
    // P85: a rider rides it (has_driver +0x142b: crash debris), and the Engine bone (damage smoke), vehicle space.
    bool hasRider() const { return std::any_of(links_.begin(), links_.end(), [](const PartLink& l) { return l.rider; }); }
    const std::array<float, 3>* engine() const { return hasEngine_ ? &engine_ : nullptr; }
    simd_float3 focus(const fuel::VehicleState& st) const;  // a point on the body, for the camera
    // P74: the vehicle node's world (vehicle space -> world) and its car occlusion box (0x5cc410).
    simd_float4x4 nodeWorld(const fuel::VehicleState& st) const;
    const std::array<float, 3>& occlusionExtents() const { return occlusionExtents_; }
    // P77: the body blob's sizes (+0x2ea0 / +0x2ea4, 0x5ca7a0) and the setup wheel in each engine wheel slot
    // (0 / 2 front, 1 / 3 rear: fronts and rears interleaved in setup order).
    const std::array<float, 2>& blobBodySize() const { return blobBodySize_; }
    const std::vector<int>& engineWheels() const { return engineWheels_; }
    // P77: the headlights in vehicle space (AssembledVehicle::lights).
    const std::vector<fuel::VehicleLight>& lights() const { return lights_; }
    // ---- PK22: the V:DYN records (AssembledVehicle::dyn, at rest), part k's record (-1 none), its link bone's
    // vehicle-space pivot and +0x60 rotation (xyzw) and its vehicle-space local; the skeleton's V:HEADLIGHT records.
    const std::vector<fuel::vlinks::DynRecord>& dynRecords() const { return dyn_; }
    int partDyn(size_t k) const { return k < links_.size() ? links_[k].dyn : -1; }
    const std::array<float, 3>& partDynPivot(size_t k) const { return links_[k].dynPivot; }
    const std::array<float, 4>& partDynBoneQ(size_t k) const { return links_[k].dynBoneQ; }
    const simd_float4x4& partLocal(size_t k) const { return links_[k].local; }
    bool headlightRecords() const { return headlightRecords_; }
    // ---- PK22 end
    // ---- PK23: the exhaust slots (vehicle space; core/vehicle_fx findExhausts on the LOD record 0x5cf170 searches)
    const std::vector<fuel::AssembledVehicle::Exhaust>& exhausts() const { return exhausts_; }
    // ---- PK23 end
    // ---- [upgrades U5] the visible upgrades (core/upgrade_visuals): the plan of the vehicle's first drawn record (LOD1 for
    // the player, else the record the exhaust FX use), inactive without stages; the nitro flame's scale and outlets.
    const fuel::upgvis::Plan& upgrades() const { return upgPlan_; }
    // ---- [upgrades U5] end

private:
    struct PartLink {
        int wheel = -1;              // physics wheel driving this part, -1 = chassis
        simd_float4x4 local;         // vehicle space (chassis parts) or wheel-model space (rotation only)
        bool rider = false;          // the rider (follows the chassis, or its own fall when ejected)
        uint32_t key = 0;            // rider mesh key
        int level = -1;              // P104: the vehicle LOD it belongs to (0 = LOD1), -1 = every level (no LOD chain)
        int dyn = -1;                // PK22: PlacedPart::dyn
        std::array<float, 3> dynPivot{};
        std::array<float, 4> dynBoneQ{0, 0, 0, 1};
        // [suspanim] the per-frame links (core/susp_anim): the record's rig (susp_), the part's node (-1 rigid) and its
        // vehicle-space rest (without the plan's static adjust); wheels: the rig whose fork may shift them.
        int susp = -1, suspNode = -1;
        simd_float4x4 rest = matrix_identity_float4x4;
    };
    void addPart(const fuel::PlacedPart& p, int level, const fuel::upgvis::PartEdit* e = nullptr, int susp = -1, int suspNode = -1);  // P104 ([upgrades U5] + the plan's edit)
    void addExtras(const fuel::upgvis::Plan& plan, int level, int susp = -1);  // [upgrades U5] ([suspanim] + the live extras)
    int addRecord(const fuel::AssembledVehicle& av, const fuel::upgvis::Plan& plan, int level);  // [suspanim] parts + extras of one LOD record
    // P104 (VehicleLods)
    bool lodOn_ = false;
    int levels_ = 1, minLevel_ = 0, riderLevels_ = 1;
    float lodD_ = 1, lodView_ = 0, lodScale_ = 1, riderD_ = 1, riderView_ = 0;
    std::array<double, 3> seat_{};
    int level_ = 0, riderLevel_ = 0;
    float alpha_ = 1, riderAlpha_ = 1;
    simd_float3 anchor_{}, riderAnchor_{};
    fuel::VehicleSetup setup_;
    std::vector<fuel::MeshInstance> parts_;
    std::vector<PartLink> links_;
    float focusHeight_ = 0.5f;  // where the camera looks, above the vehicle origin
    std::array<float, 3> occlusionExtents_{0.75f, 0.75f, 0.75f};  // P74: AssembledVehicle::occlusionExtents
    std::array<float, 2> blobBodySize_{};
    std::vector<int> engineWheels_;
    std::vector<fuel::VehicleLight> lights_;
    bool hasEngine_ = false;  // P85
    std::array<float, 3> engine_{};
    std::vector<fuel::vlinks::DynRecord> dyn_;  // PK22
    bool headlightRecords_ = false;              // PK22
    std::vector<fuel::AssembledVehicle::Exhaust> exhausts_;  // PK23
    fuel::upgvis::Plan upgPlan_;  // [upgrades U5]
    std::vector<fuel::susp::Rig> susp_;                    // [suspanim] one per LOD record
    mutable std::vector<fuel::susp::Output> suspOut_;      // [suspanim] this frame's maps
    mutable std::vector<fuel::susp::WheelLive> suspWheels_;
};

// Drive mode: assembled vehicles on the terrain, moved by the physics. Vehicle 0
// is the player; the others (race opponents) are added with addVehicle and
// driven through setInput.
class DriveSession {
public:
    // Builds the physics vehicle for `vehicle`'s parts and places it at (x, z).
    // `riders`: extra meshes (the baked rider, in vehicle space) that follow the chassis.
    DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain, double x,
                 double z, float yaw, const std::vector<uint32_t>& riders = {});

    // Adds another vehicle at (x, z) facing `yaw`; returns its index.
    int addVehicle(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, double x, double z, float yaw,
                   const std::vector<uint32_t>& riders = {}, const VehicleLods* lods = nullptr);
    // P104: every vehicle's LOD levels for this frame's camera eye (before uploadRiders and the renderer's alphas).
    void updateLods(simd_float3 eye) {
        for (size_t i = 0; i < rigs_.size(); ++i) rigs_[i].updateLod(world_.state(int(i)), eye);
    }
    int vehicleCount() const { return int(rigs_.size()); }
    // PK31: Rider_ProximityReaction's one-shot (base 21/25/29/33, 4 variants) on vehicle i's rider.
    void riderGesture(int i, int base) {
        if (i >= 0 && size_t(i) < rigs_.size()) rigs_[size_t(i)].riderGesture(base);
    }
    // ---- PK14: KillMission's one-shot reset on the player's rider (0x62baa0, player list 0xa66260[0] +0x154).
    void riderKillMission(int i = 0) {
        if (i >= 0 && size_t(i) < rigs_.size()) rigs_[size_t(i)].riderKillMission();
    }
    // 0x632570 (mission states 6 / 7 ENDING_WON / LOST, every frame) with a player vehicle: Rider_Restore 0x5c8c30 (the
    // ragdoll destroyed: the rider is back on the vehicle) and Damage_Lower(1.0, 0) 0x4200c0 on its impact record +0x144.
    void endingRestore(int i = 0) { world_.endingRestore(i); }
    // ---- end PK14
    // Swaps the player's vehicle for another where it stands (the world and other vehicles stay).
    void replacePlayer(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders = {});
    // Input for vehicle `i` > 0 from the next update on (the player's comes with update).
    void setInput(int i, const fuel::DriveInput& input);
    const fuel::DriveInput& input(int i) const { return inputs_[size_t(i)]; }

    // The parts of vehicle `i` in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts(int i = 0) const { return rigs_[size_t(i)].parts(); }
    // P74: vehicle i's car occlusion omni: its node world and box half extents (MeshRenderer::setCarOcclusion).
    std::pair<simd_float4x4, std::array<float, 3>> carOcclusion(int i) const {
        return {rigs_[size_t(i)].nodeWorld(world_.state(i)), rigs_[size_t(i)].occlusionExtents()};
    }
    const fuel::VehicleSetup& setup(int i = 0) const { return rigs_[size_t(i)].setup(); }
    const VehicleRig& rig(int i = 0) const { return rigs_[size_t(i)]; }
    int raceState() const { return raceState_; }
    // Advances every vehicle; `input` drives the player.
    void update(float dt, const fuel::DriveInput& input);
    void reset(int i = 0) { world_.reset(i); }
    // Puts vehicle `i` on its wheels at (x, z) facing `yaw`.
    void teleport(double x, double z, float yaw, int i = 0) { world_.placeVehicle(i, x, z, yaw); }
    // World matrices for the parts of vehicle `i`, in parts(i) order.
    std::vector<simd_float4x4> partWorlds(int i = 0) const { return rigs_[size_t(i)].partWorlds(world_.state(i)); }
    fuel::VehicleState state(int i = 0) const { return world_.state(i); }
    simd_float3 position(int i = 0) const { return rigs_[size_t(i)].focus(world_.state(i)); }  // a point on the body, for the camera
    // CARPOLISH: a vehicle-space point (x left, y up, z forward) in the world (the vehicle node's world, P74)
    simd_float3 vehiclePoint(simd_float3 local, int i = 0) const {
        return simd_mul(rigs_[size_t(i)].nodeWorld(world_.state(i)), simd_make_float4(local, 1.0f)).xyz;
    }
    fuel::DriveWorld& physics() { return world_; }
    const fuel::DriveWorld& physics() const { return world_; }
    const fuel::Heightfield& terrain() const { return terrain_; }
    float heading(int i = 0) const;  // radians; 0 = +z
    // Race state for the riders (FUEL's race-state +0x48: 4 start, 6 finished, 0 otherwise).
    void setRaceState(int s) {
        if (s != raceState_) world_.setRaceState(s);  // P87: 0x51e9e0 reads it too
        raceState_ = s;
    }
    // Skins the animated riders (call before drawing a frame).
    void uploadRiders() const {
        for (const VehicleRig& r : rigs_) r.uploadRider();
    }

private:
    const fuel::Heightfield& terrain_;
    int raceState_ = 0;
    fuel::DriveWorld world_;
    std::vector<VehicleRig> rigs_;
    std::vector<fuel::DriveInput> inputs_;
};

// FUEL's player camera (CameraMove_G, physics/fuel_camera.hpp) on the player's vehicle of a drive session:
// near chase (view 5, default), first person (1) and far chase (4), cycled with the CAMERA button, plus the
// race intro (8), finish (9) and wreck/drown (10/11) views. The camera collides with the terrain and the
// static objects (sphere push-out) and with the other vehicles.
class ChaseCamera {
public:
    fuel::OrbitCamera update(float dt, const DriveSession& drive, const fuel::cam::CameraControls& in = {},
                             fuel::cam::Override ov = fuel::cam::Override::None);
    simd_float3 focus() const { return focus_; }  // for MeshRenderer::setFocus (the vehicle)
    float speedEffect() const { return blur_; }   // 0..1 of FUEL's speed blur (P572..P573), for MeshRenderer::speedBlur
    int view() const { return cam_.view(); }
    void setView(int v) { cam_.setPlayerView(v); }
    const fuel::cam::CameraPose& pose() const { return pose_; }

private:
    fuel::cam::ChaseCamera cam_;
    fuel::cam::CameraPose pose_;
    simd_float3 focus_ = {0, 0, 0};
    float blur_ = 0;
    double prev_[3] = {0, 0, 0};
    bool havePrev_ = false;
};

// FUEL's wheel effects (core/sfx.hpp: SpecialEffectsWheelEmitter_G per wheel, fed like 0x5ceb60) for every
// vehicle near the camera: dust, stones, mud/water projections and tyre smoke, drawn by the renderer's
// billboard_soft pass. Tuning from pp001.tsc and FUEL.exe (setDataDir).
class DustSystem {
public:
    // The FUEL install (CommonComponent/ and XFinalComponent/ below it); without it nothing is emitted.
    static void setDataDir(const std::string& dir);
    // The effects tables (pp001 rows, FX atlas) once the first update loaded them; null before / without the game.
    static const fuel::sfx::Tables* tables();
    // FUEL's point colour of the ground at (x, z), rgba 0..1 (0x6b76a0 colour; the viewer's ground map).
    static std::function<std::array<float, 4>(double x, double z)> groundColour;
    // `look`: the camera's target (FUEL's +0x10a4 camera distance is tripled behind the camera).
    void update(float dt, const DriveSession& drive, simd_float3 eye, simd_float3 look = simd_make_float3(0, 0, 0));
    const fuel::SfxFrame& particles() const { return frame_; }
    // PK24: the player vehicle's feed for the renderer's lens projections (rain drops, wheel splashes, mud, dirt strip).
    const fuel::wfx::LensVehicle& lens() const { return lens_; }
    // ---- PK23: the player's backfire flash +0x2ebc (DriveAudio::playerFlash; < 0 or unset: the effects run their own copy
    // of 0x5d9d60's pop rule). Set by the viewer.
    static std::function<float()> playerFlash;
    const fuel::vfx::VehicleFx* vehicleFx() const { return vfx_.get(); }
    // ---- PK23 end

private:
    std::unique_ptr<fuel::sfx::System> sys_;
    fuel::SfxFrame frame_;
    fuel::wfx::LensVehicle lens_;  // PK24
    fuel::blob::Producer blobs_;  // P77: FakeShadows_G producers 0x5d86b0 / 0x5d8420
    // ---- PK23
    std::unique_ptr<fuel::vfx::VehicleFx> vfx_;
    struct Backfire {  // the effects' copy of 0x5d9d60's pop rule (only without the player's sounds)
        bool started = false;
        float prevRpm = 0, timer = 0, flash = 0;
        uint32_t pm = 0x2468ace1u;
    } backfire_;
    void updateVehicleFx(float dt, const DriveSession& drive, simd_float3 eye, simd_float3 look);
    // ---- PK23 end
};

// Streams the static world around the player: every object the sub-worlds
// place, drawn with FUEL's per-object LOD and fade (renderer setLodObjects) and solid within kCollideRadius.
class WorldStreamer {
public:
    WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup);
    ~WorldStreamer() {
        if (job_.valid()) job_.wait();
        if (grassJob_.valid()) grassJob_.wait();
        if (tileJob_.valid()) tileJob_.wait();
    }
    size_t placementCount() const { return placed_.size(); }
    // USA1.DPC's own drawn placements (collectWorldRootMeshes, p36); always loaded like FUEL's
    // resident groups. Call once after construction.
    void addResident(const fuel::DpcFile& usa1);
    // FUEL's sub-world streaming (p35, decode P31 out-F): with the WorldRef table, only the two
    // nearest regions within 5000 m and the course of the running mission are drawn and solid;
    // without it every sub-world is. `mission` = the event id ("" = free roam: no course).
    void setSubWorlds(std::vector<fuel::SubWorldRef> refs);
    void setMission(const std::string& mission);
    // PK28: whether the sub-world file `name` is attached now (true without the WorldRef table or for unknown names).
    bool subWorldActive(const std::string& name) const {
        for (size_t i = 0; i < subFiles_.size(); ++i)
            if (subFiles_[i] == name) return subActive_.empty() || i >= subActive_.size() || subActive_[i] != 0;
        return true;
    }
    // M_OPEN / M_NOTOPEN / M_CLOSED and M_WON / M_WONCLOSED answers (P39: the career; see drive.mm).
    void setMissionState(std::function<bool(const std::string&)> open, std::function<bool(const std::string&)> won);
    // P67: an RTC starts or ends (Play 0x535000 / Start 0x535260 / End 0x536230 re-evaluate every condition, 0x512920).
    void setRtcRunning(bool on);
    // P107: headless checks run frames faster than real time and share the machine, so the background job (plants,
    // AutoMesh, water) can land any number of frames late. With n > 0 every job lands exactly n frames after its launch
    // (waited for then if need be): the check sees the port's streaming at a fixed latency (n / 60 s of game time), the
    // same on every run, not the machine's load. 0 (the window): a job lands when it is ready.
    void setMaxJobFrames(int n) { maxJobFrames_ = n; }
    // Procedural trees and bushes (drawn; tree trunks solid within kCollideRadius).
    void setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain);
    // Surface types for ground clutter (grass tufts, stones); must be thread-safe.
    void setSurfaces(fuel::SurfaceQuery q) { surfaces_ = std::move(q); }
    // The road network (plant filters near roads). Roads are drawn by the terrain (decals).
    void setRoads(const fuel::RoadNetwork* roads) { roads_ = roads; }
    // P50: clone.tsc's classes by node name (fuel::loadCloneClasses): which placements are game objects.
    void setGameClasses(std::unordered_map<uint32_t, fuel::GameClass> c) { gameClasses_ = std::move(c); }
    // Lake water meshes around the player (needs setGround).
    void setLakes(const fuel::Lakes* lakes) { lakes_ = lakes; }
    // FUEL's AutoMesh / RoadMesh objects (core/automesh.hpp), generated with the plants on the
    // worker (needs setVegetation, setGround and setRoads first). FUEL_NO_AUTOMESH=1 disables.
    void setAutoMesh(std::unique_ptr<fuel::AutoMesh> a) { autoMesh_ = std::move(a); }
    // P44: the fuel barrels (AutoMesh IT_Fuel01 / IT_Fuel02) of the last applied generation, and the ones the
    // player took (their meshes are left out of the next generations while their cell stays; the free-roam
    // rules pick them up on contact, so they are not solid).
    struct Barrel {
        float x = 0, y = 0, z = 0, amount = 0, radius = 0;
    };
    const std::vector<Barrel>& barrels() const { return barrels_; }
    // P49: the decor objects with a sound of their own (burnt-forest fires) of the last applied generation.
    const std::vector<fuel::AutoMesh::DecorSound>& decorSounds() const { return decor_; }
    uint64_t barrelGeneration() const { return barrelGen_; }
    void setTakenBarrels(std::vector<std::array<float, 2>> taken) {
        std::lock_guard<std::mutex> l(takenMutex_);
        taken_ = std::move(taken);
    }
    // Call every frame; refreshes after the player has moved a little.
    void update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force = false);
    // Waits for every worker job, then rebuilds everything (objects, plants, roads, grass,
    // fine ground colour) for `player` on this thread: the world as it stands when streaming
    // has fully caught up (--capture-spot).
    void settle(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    // PK15: no background job queued or running (objects and plants, grass, fine ground colour): the pool streamer's
    // busy test 0x748ef0 (requests pending or a read in flight) negated; the teleport (0x57e0f0 state 2) waits for it.
    bool idle() const { return !job_.valid() && !rerun_ && !grassJob_.valid() && !tileJob_.valid(); }
    // Placed objects within `radius` of (x, z) whose base is more than `minGap` metres off the
    // port's terrain (authored heights; the engine seats no static placement, p35).
    struct GroundGap {
        uint32_t meshId = 0;
        simd_float3 position = {0, 0, 0};  // the placement's origin
        float gap = 0;                     // origin y - terrain height, metres (+ floats, - buried)
    };
    std::vector<GroundGap> groundGaps(double x, double z, double radius, float minGap) const;
    // Knock-aside props: small objects near the player are dynamic bodies.
    struct PropStats {
        size_t tracked = 0;  // props with state (a body near the player, or knocked away)
        size_t bodies = 0;   // props currently simulated
        size_t moved = 0;    // props displaced from where they were placed
        float maxDisplacement = 0;  // metres, the furthest of them
        simd_float3 where = {0, 0, 0};  // current position of that one
    };
    PropStats propStats() const;
    // P50: FUEL's break-through rule for the player (0x51edb0 in the contact hook 0x51f6c0): a breakable decor
    // object (IT_DECORDYNMESH_G, +0x60 & 0x800000) that the vehicle is about to touch with an approach speed of at
    // least P575 m/s breaks (0x515cc0 hides it; its IT_DYNOBJ_G pieces are not spawned by the port) and the contact
    // is dropped; slower contacts meet it as a static object. Call every frame before the physics step.
    // P55: every vehicle of the world breaks through (FUEL runs 0x51f6c0 for each), and a dynamic object hitting a
    // breakable faster than 5 m/s breaks it too (IT_DYNOBJMove_G vt[141] 0x531c00); broken decor spawns its IT_DYNOBJ_G
    // pieces (0x515cc0).
    void breakThrough(const DriveSession& drive, float dt, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    size_t piecesSpawned() const { return piecesSpawned_; }
    // P55: IT_ANIMATED objects (Lod ids) whose STARTANIMATED volume the player entered (event slot 334, 0x518170), and
    // those a RESET_ rule re-armed (slot 0 again, 0x517f80), since the last call.
    std::vector<uint32_t> takeAnimStarts() { return std::exchange(animStarts_, {}); }
    std::vector<uint32_t> takeAnimResets() { return std::exchange(animResets_, {}); }
    // P61: 0x5181f0 (the tornado manipulators): every shown IT_ANIMATED placement within `r` m of (x, y, z) starts.
    void startAnimatedNear(double x, double y, double z, float r);
    // PK29: the shown placements of Lod `lod` (one per node; row-vector world matrices): where the world animation's
    // event sounds (0x24) and FX (0x20) of that class play.
    void placementsOf(uint32_t lod, std::vector<std::array<float, 16>>& out) const;
    // PK06: 0x516570 (the tornado manipulators' vf18 within 200 m of the camera): every intact IT_DECORDYNMESH_G whose
    // node lies within `r` m (3-D) of (x, y, z) is broken (0x515cc0: hidden, its IT_DYNOBJ_G pieces spawn). Returns how
    // many broke (placed and AutoMesh decor alike).
    size_t breakNear(double x, double y, double z, float r, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    // P92: what throwDebris loads at its first call (the Lod name table: every Lod link of the world, ~25 ms) and the
    // items' meshes (with their textures), done at session set-up instead of at the first crash or tornado.
    void prewarmDebris(const std::vector<const char*>& items, fuel::MeshRenderer& renderer);
    // P65: the tornado debris spawner (0x5bd590 -> 0x4f16f0 + 0x533d70): an IT_Debris item (USA1 Lod named `item`) at
    // `pos`, turned `yaw` about y, launched at `vel`. [port] at most kMaxThrown at once: the oldest goes.
    bool throwDebris(const char* item, const std::array<double, 3>& pos, const std::array<float, 3>& vel, float yaw, fuel::DriveWorld& physics,
                     const std::array<float, 3>* angular = nullptr, uint32_t* id = nullptr,  // P85: + w, the prop id
                     bool noContact = false);  // P87: +0xeb2 kept (the crash debris): touches nothing
    size_t debrisThrown() const { return debrisThrown_; }
    // P85: a thrown debris prop's position (false when gone) and its removal (IT_DebrisMove vt+0x1d8, 0x533ec0).
    bool thrownPosition(uint32_t id, std::array<double, 3>& out) const;
    void dropThrown(uint32_t id, fuel::DriveWorld& physics);
    // P61: impact sounds of the props (IT_DYNOBJMove_G vt[139] 0x531710, IT_DebrisMove vt[139] 0x5341f0): after a
    // frame's physics, each prop's peak approach speed plays its Lod's bank symbol through 0x636ee0.
    struct ObjectSound {
        uint32_t lodName = 0;  // the object's name CRC (its Lod_Z sound bank)
        int symbol = 0;        // 0x83 / 0x84 (0x86: debris below 1 m/s)
        std::array<double, 3> pos{};
        float pitch = 1, volume = 1, minDist = 50, maxDist = 500;
    };
    std::vector<ObjectSound> updateImpacts(float dt, fuel::DriveWorld& physics);
    size_t brokenCount() const { return brokenCount_; }
    // The nearest breakable solid to (x, z) (placed or AutoMesh); false when none is near.
    bool nearestBreakable(double x, double z, simd_float3& out, const std::vector<simd_float3>& skip = {}) const {
        double best = 1e30;
        for (const auto* list : {&breakables_, &autoBreakables_})
            for (const Breakable& b : *list) {
                const simd_float3 c = simd_make_float3(float(b.m[12]), float(b.m[13]), float(b.m[14]));
                if (std::any_of(skip.begin(), skip.end(), [&](simd_float3 t) { return simd_distance(t, c) < 1.0f; })) continue;
                if (const double d = std::hypot(b.m[12] - x, b.m[14] - z); d < best) best = d, out = c;
            }
        return best < 1e30;
    }
    // ---- PK14: KillMission 0x62baa0 on the world objects (retry: TTMN 0x54a910 runs KillMission before the TLP; the
    // other in-world callers, free ride `KillMission 1` and the save load 0x5539f0, restart the port's session):
    //   0x515be0  every IT_DECORDYNMESH_G vf20 0x515c80: messages dropped, +0x6c = 0, its break pieces back to the
    //             object factory (0x5161d0 -> 0x4f1b60: gone). [O] vf20 does not clear the decor node's 0x40000: a
    //             broken decor object stays hidden (until its sub-world is attached again, below).
    //   0x51df10  every IT_DYNOBJ_G 0x51dea0: vf20 0x51b930 + mover reset (vt+0x38, 0x70e160 node re-sync): placed and
    //             AutoMesh props back on their placement.
    //   SetState(0) (0x632c10 case 0: the M_END event 0x512980, UNSTREAM_M_END) detaches the course sub-world; the retry's
    //             STREAM_M_TELEPORT attaches it fresh (WorldRef_Z::Detach 0x740420 / Attach 0x740690: 0x55ed70 / 0x55ed20
    //             re-register its CONDITION_OBJECT records, 0x72fde0 binds new agents) -> its decor whole, conditions anew.
    //   0x62c850  every node of the game world with +0xd8 bit 0 (set by IT_ANIMATED's start 0x518170) -> agent vf20
    //             (IT_ANIMATED::Reset 0x517f80: +0x74 = 0, the animation re-armed).
    //   0x512920(0) every object condition again (M_RUNNING off).
    struct KillReport {
        size_t pieces = 0, props = 0, autoProps = 0, courseFresh = 0, anims = 0, endResets = 0;
    };
    KillReport killMission(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    // For --retry-check: the props (placed or AutoMesh, not pieces / debris) within `r` m of (x, z): where each is, its
    // placement and how far it is from it.
    struct PropPlace {
        simd_float3 pos, origin;
        float displacement = 0;
    };
    std::vector<PropPlace> propsNear(double x, double z, float r) const;
    size_t piecesNow() const;  // break pieces alive
    // ---- end PK14
    // P50 --collision-check: the collision world around the last update, by class.
    struct CollisionClassCount {
        size_t now = 0;   // what this build makes solid
        size_t main = 0;  // what main 1efc828 made solid (render radius 0.4..3 m -> prop, else node flag 0x8000000)
    };
    struct CollisionStats {
        std::map<std::string, CollisionClassCount> classes;
        size_t outsideMesh = 0;    // solid with collision reaching > 1 m outside the drawn mesh's bounding sphere
        size_t invisibleNear = 0;  // solid whose mesh is not drawn at 10 m (fade alpha < 0.01) or has no mesh
        std::vector<std::string> examples;  // a few of those
    };
    CollisionStats collisionStats() const;
    // Positions of the placements whose node is a game object of class `c` (all sub-worlds, active or not).
    std::vector<simd_float3> placementsOfClass(fuel::GameClass c) const {
        std::vector<simd_float3> out;
        for (const Placed& p : placed_)
            if (p.collisionFirst && gameClass(p.nodeName) == c) out.push_back(simd_make_float3(p.world[12], p.world[13], p.world[14]));
        return out;
    }
    // Solid bodies within `radius` of the point (labels "class (where)"): placed static, AutoMesh, trunks, props.
    std::vector<std::string> solidsAt(double x, double y, double z, double radius) const;
    // True when (x, z) is within `clearance` metres (at most 5) of a placed object's ground-plane
    // bounding circle: the spot is inside, or touching, a building, rock or prop.
    bool nearObject(double x, double z, float clearance) const { return nearObstacle(x, z, clearance); }
    // Position of the movable prop nearest to `from` (diagnostics); false when there is none.
    bool nearestProp(simd_float3 from, const std::vector<simd_float3>& skip, simd_float3& out) const;

private:
    static constexpr float kCell = 256.0f, kCollideRadius = 250.0f;
    // Decor draw distances follow FUEL's decor quadtree (see decorChunkSize in drive.mm):
    // FUEL's trees end with the 384 m chunks (camera within 2.0625 x 768 m of the 768 m
    // chunk centre, so at most ~2130 m away), bushes with the 96 m chunks, full meshes
    // with the 48 m chunks (~200 m). kPlantRadius only bounds the scatter.
    static constexpr float kPlantRadius = 600.0f;  // every plant within this; only tree cards beyond (the 96 m chunks end within ~540 m)
    static constexpr double kTreeChunkMax = 600.0, kBushChunkMax = 100.0, kFullMeshChunk = 60.0;
    // P94: what can show before the next rebuild (40 m on, a worker job late: 80 m of slack) under FUEL's decor fades
    // (vegetation.hpp decorAtt): meshes fade out by 162.5 m, bush cards by 300 m, tree cards by 1300 m.
    static constexpr float kMeshRadius = 162.5f + 80.0f, kBushCardRadius = 300.0f + 80.0f, kCardRadius = 1300.0f + 80.0f;
    static constexpr float kLegacyCardRadius = 2150.0f;  // FUEL_POP_LEGACY: tree cards out to the last 384 m chunks
    static constexpr float kPropRadius = 120.0f, kPropForget = 300.0f;  // dynamic within, state reset beyond
    // P94: AutoMesh generation slack: the 40 m rebuild step, the worker's latency and the chase camera's offset.
    static constexpr double kAutoSlack = 100.0;
    const fuel::Vegetation* vegetation_ = nullptr;
    fuel::SurfaceQuery surfaces_;
    static constexpr float kClutterRadius = 150.0f;
    const fuel::RoadNetwork* roads_ = nullptr;
    const fuel::Lakes* lakes_ = nullptr;
    // Plants are rebuilt on a worker thread (they took ~6 ms
    // per update on the main thread); the renderer gets them when ready.
    struct Background {
        std::vector<fuel::MeshRenderer::WaterVertex> water;
        std::vector<uint32_t> waterIndices;
        bool haveWater = false;
        std::vector<fuel::MeshInstance> plants;
        std::vector<std::array<double, 16>> trunks;  // solid cylinders for nearby trees and bushes (row-vector matrices)
        size_t treeTrunks = 0, bushTrunks = 0;
        bool haveAuto = false;
        std::vector<fuel::PlacedMesh> autoObjects;   // AutoMesh / RoadMesh spawns (renderer set 1)
        size_t autoRoadFrom = 0;                     // road furniture from here on (not solid)
        std::vector<Barrel> barrels;                 // P44: fuel barrels among the spawns
        std::vector<fuel::AutoMesh::DecorSound> decor;  // P49: spawns with a sound (decor fires)
        std::vector<uint8_t> autoCollect;            // per autoObjects entry: part of a barrel (not solid)
    };
    std::vector<Barrel> barrels_;
    std::vector<fuel::AutoMesh::DecorSound> decor_;
    uint64_t barrelGen_ = 0;
    mutable std::mutex takenMutex_;
    std::vector<std::array<float, 2>> taken_;
    std::unique_ptr<fuel::AutoMesh> autoMesh_;
    std::vector<std::pair<uint32_t, std::array<double, 16>>> autoSolid_;  // big AutoMesh objects near the player
    Background buildBackground(simd_float3 player) const;
    void buildWater(simd_float3 player, Background& bg) const;
    void applyBackground(Background&& bg, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void pollBackground(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void applySolids(fuel::DriveWorld& physics);
    void ensureCollision(fuel::DriveWorld& physics, uint32_t key);
    // P50: FUEL's collision world (see solidKind in drive.mm).
    enum class Solid : uint8_t { None, Static, Movable };
    Solid solidKind(const fuel::PlacedMesh& p) const;
    fuel::GameClass gameClass(uint32_t nodeName) const;
    const fuel::StaticCollision& collision(uint32_t objectId) const;
    mutable std::mutex collisionMutex_;
    mutable std::unordered_map<uint32_t, std::shared_ptr<fuel::StaticCollision>> collision_;
    std::unordered_map<uint32_t, fuel::GameClass> gameClasses_;
    std::vector<fuel::PlacedMesh> autoObjects_;  // the last applied AutoMesh generation (diagnostics)
    size_t autoRoadFrom_ = 0;
    size_t treeTrunks_ = 0, bushTrunks_ = 0;
    // Breakable solids near the player: placement index (or -1 for AutoMesh) with its object and matrix.
    struct Breakable {
        int64_t placed = -1;
        uint32_t objectId = 0;
        std::array<double, 16> m{};
    };
    std::vector<Breakable> breakables_, autoBreakables_;
    std::vector<std::array<float, 2>> brokenAuto_;  // AutoMesh breakables broken (x, z), dropped by later generations
    size_t brokenCount_ = 0, piecesSpawned_ = 0;
    static constexpr size_t kMaxThrown = 24;
    std::deque<uint32_t> thrown_;  // P65: debris prop ids, oldest first
    size_t debrisThrown_ = 0;
    void refreshHidden(fuel::MeshRenderer& renderer);
    std::vector<std::pair<uint32_t, std::array<double, 16>>> cellSolid_;  // placed objects near the player
    std::vector<std::array<double, 16>> trunks_;
    std::future<Background> job_;
    int jobFrames_ = 0, maxJobFrames_ = 0;  // P107: frames the background job has run; the bound (0: none)
    bool rerun_ = false;      // the player moved on while the job ran
    std::vector<fuel::MeshInstance> cellDraw_, plantDraw_;  // the static world's draw list, in two halves
    const fuel::Heightfield* terrain_ = nullptr;
    struct Placed : fuel::PlacedMesh {  // with its level-of-detail data (p36)
        uint16_t sub = 0;  // index into subFiles_
        bool hidden = false;  // its CONDITION_OBJECT hides it (p38, fuel::evaluateObjectCondition)
        bool broken = false;  // P50: a breakable decor node the player broke through (0x51edb0 -> 0x515cc0)
    };
    fuel::ObjectGameState objects_;  // what the object conditions ask (running mission, p38)
    void evaluateConditions();
    void addPlacement(const fuel::PlacedMesh& pm, uint16_t sub);
    // The renderer's object list (setLodObjects): placement index -> list index, or -1.
    std::vector<int32_t> lodIndex_;
    bool lodDirty_ = true;  // the active set changed: hand the renderer a new list
    std::vector<std::string> subFiles_;  // the sub-world DPC file names, in constructor order
    std::vector<fuel::SubWorldRef> refs_;
    std::vector<int> subRef_;            // per sub-world file: index into refs_ (-1 = not streamed)
    std::vector<char> subActive_;        // per sub-world file
    std::string mission_;
    bool streamed_ = false;              // refs_ set: stream regions / courses like FUEL
    bool refreshActive(simd_float3 player);  // true when the loaded set changed
    bool active(const Placed& p) const { return !p.hidden && !p.broken && (subActive_.empty() || subActive_[p.sub]); }
    // Movable props (IT_DYNOBJ_G, IT_Debris): keyed by placement index for placed objects, kAutoPropId | n for AutoMesh
    // spawns and the pieces of broken decor (P55). A prop keeps its state until it is far away.
    struct Prop {
        std::array<float, 16> world;  // current transform (row-vector convention)
        float scale[3];
        bool moved = false, body = false;
        // P55
        uint32_t objectId = 0;             // its object (collision shapes, UserDefine)
        std::vector<uint32_t> meshes;      // drawn meshes (the object's first level)
        // P94: the same entries with their Lod data: FUEL's dynamic objects keep drawing through their Lod_Z
        // (0x884170: level by distance, ComputeFade), so the props are the renderer's LOD set 3 at their pose.
        std::vector<fuel::PlacedMesh> parts;
        float mass = 0;                    // FUEL's mass (fuel::dynObjMass), kg
        bool debris = false;               // P61: IT_Debris (IT_DebrisMove mass 100 kg/m^3, drag 0x533790)
        bool piece = false;                // spawned by a break (0x515cc0)
        float radius = 0;                  // P61: body radius +0x408 (physics bounds)
        float impactPeak = 0, impactTimer = 0;  // P61: vt[141] peak (+0xef0 / +0xefc), vt[139] timer (+0xeec / +0xef8)
        uint64_t autoKey = 0;              // AutoMesh spawn: its origin key (autoPropKey), else 0
        std::array<float, 3> origin{};     // where it was placed or spawned
    };
    static constexpr uint32_t kAutoPropId = 0x80000000u;
    uint32_t nextPropId_ = 0;
    std::unordered_map<uint32_t, Prop> props_;
    std::unordered_map<uint32_t, uint32_t> objectNames_;  // P61: object id -> its link name CRC (sound bank)
    std::unordered_map<uint64_t, uint32_t> autoProps_;  // AutoMesh spawn origin key -> prop id
    static uint64_t autoPropKey(const fuel::PlacedMesh& p);
    void updateProps(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool dirty);
    // P55: FUEL's dynamic objects (core/dyn_object.hpp).
    fuel::DynObjParams dynParams_;
    bool dynParamsLoaded_ = false;
    mutable std::unordered_map<uint32_t, fuel::DynObjUserDefine> dynUserDefine_;  // Lod object -> its UserDefine
    const fuel::DynObjUserDefine& dynUserDefine(uint32_t objectId) const;
    std::unordered_map<uint32_t, uint32_t> lodByName_;  // Lod link name CRC -> object id (the break pieces)
    float propMass(uint32_t objectId, float scale) const;
    bool makeProp(Prop& prop, const fuel::PlacedMesh& p, fuel::DriveWorld& physics);
    // 0x515cc0: the IT_DYNOBJ_G pieces of a broken decor object (its Lod's UserDefine), at the decor's pose.
    void spawnPieces(uint32_t decorObject, const std::array<double, 16>& m, fuel::DriveWorld& physics);
    struct AnimVolume : fuel::AnimStartVolume {
        uint16_t sub = 0;
        bool fired = false;
        std::vector<uint32_t> started;  // PK12: EvenementVolAgent_G +0x84: the STARTFX nodes it started (0x5714f0)
    };
    // ---- PK12: STARTFX actions for the renderer (fired in checkAnimVolumes, handed over in update), the hidden state of
    // the started nodes and the releases of volumes whose sub-world went away (vf25 0x570780).
    std::vector<fuel::sand::Fire> startFxFires_;
    void pushStartFx(fuel::MeshRenderer& renderer, bool dropHidden);
    bool sandColourSet_ = false;
    // ---- PK12 end
    std::vector<AnimVolume> animVolumes_;
    std::vector<uint32_t> animStarts_, animResets_;
    std::vector<uint32_t> animated_;  // P61: placements of IT_ANIMATED classes
    size_t animatedScanned_ = 0;
    std::vector<uint32_t> decorBreak_;  // PK06: placements of IT_DECORDYNMESH_G (breakable decor)
    size_t decorScanned_ = 0;
    void checkAnimVolumes(simd_float3 player);
    // ---- PK14
    std::vector<uint32_t> startedAnims_;  // IT_ANIMATED objects started (0x518170 sets their node's +0xd8 = 1)
    void noteAnimStarts(const std::vector<uint32_t>& objects);
    // A sub-world attached again (fresh nodes, 0x740690): its decor whole, conditions from their defaults, volumes re-armed.
    size_t freshSub(size_t f);
    // The RESET_ rules a mission event fires (0x512980 -> 0x513630 action 4) go to resetPending_.
    size_t fireResets(int kind, const std::string& mission);
    // ---- end PK14
    std::vector<uint32_t> resetPending_;  // placements a RESET_ rule fired for (0x513ca0 -> agent reset 0x51b930)
    void applyResets(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void pushAutoDraw(fuel::MeshRenderer& renderer);

    // ---- Grass and the ground generator (p14) ----
public:
    // The decoded ground generator: grass tufts near the player and the trees' ground tests.
    void setGround(const fuel::GroundGenerator* g) { ground_ = g; }

private:
    const fuel::GroundGenerator* ground_ = nullptr;
    // FUEL builds grass for decor chunks of at most 30 m (W+0x5af98), i.e. the 24 m leaves
    // of the decor quadtree (decorChunkSize): they exist while the camera is within
    // 2.0625 x 48 m of their parent's centre, so grass reaches 65..133 m depending on
    // where the chunks fall. Rebuilt on a worker after kGrassStep of travel.
    static constexpr float kGrassReach = 2.0625f * 48.0f + 24.0f * 1.4143f, kGrassStep = 4.0f;
    static constexpr int kGrassChunk = 24;  // metres, FUEL's smallest decor chunk; tufts are cached per chunk
    using GrassCache = std::unordered_map<uint64_t, std::vector<fuel::GrassTuft>>;
    struct GrassJob {
        std::vector<fuel::GrassTuft> tufts;
        GrassCache cache;
    };
    GrassJob buildGrass(simd_float3 player, GrassCache cache) const;
    void updateGrass(simd_float3 player, fuel::MeshRenderer& renderer, bool force);
    // Fine ground colour around the player (MeshRenderer::setGroundTile): FUEL colours its
    // terrain per vertex, 1 m apart near the camera, finer than the 60 m colour map.
    struct GroundTile {
        int level = 0;
        double originX = 0, originZ = 0;
        float extent = 0;
        std::vector<uint8_t> rgba;
        std::vector<uint16_t> types;  // 4 per texel: surface type | weight byte << 8 (bit 21, 0xff = none)
        std::vector<float> decal;     // 4 per texel: road decal u (across), v (along), weight, slot (-1 none)
    };
    // Texels sit on FUEL's vertex lattice of the 24, 48 and 96 m chunks (1, 2 and 4 m).
    static constexpr int kTileLevels = 3;
    static constexpr float kTileSpacing[kTileLevels] = {1.0f, 2.0f, 4.0f}, kTileStep[kTileLevels] = {48.0f, 96.0f, 192.0f};  // rebuilt after this much travel
    GroundTile buildGroundTile(int level, simd_float3 player) const;
    void updateGroundTiles(simd_float3 player, fuel::MeshRenderer& renderer, bool force);
    std::future<GroundTile> tileJob_;
    simd_float2 tileAt_[kTileLevels] = {{1e30f, 1e30f}, {1e30f, 1e30f}, {1e30f, 1e30f}};
    GrassCache grassCache_;  // handed to the worker and back, never shared
    std::future<GrassJob> grassJob_;
    simd_float3 grassAt_ = {1e30f, 0, 1e30f};
    std::vector<const fuel::DpcFile*> lookup_;
    std::vector<Placed> placed_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    // Ground-plane bounding circles of the placements: FUEL plants no tree or bush
    // within 5 m of a placed object.
    struct Obstacle {
        float x, z, r;
    };
    static constexpr float kObstacleCell = 64.0f, kPlantClearance = 5.0f, kMaxObstacle = 60.0f;
    std::vector<Obstacle> obstacles_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> obstacleCells_;
    std::unordered_map<uint32_t, std::array<float, 4>> sphereOf_;  // mesh bounding spheres (load time only)
    void addObstacle(const fuel::PlacedMesh& pm);
    bool nearObstacle(double x, double z, float clearance) const;
    simd_float3 last_ = {1e30f, 0, 0};
};

// P74: the player's car occlusion omni (vehicle 0; FUEL gives it to the player's vehicle only) to the renderer.
void setCarOcclusion(const DriveSession& drive, fuel::MeshRenderer& renderer);

// The assembled wheel centres in vehicle space (for makeVehicleSetup).
std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle);
