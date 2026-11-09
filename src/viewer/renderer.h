// Metal renderer for FUEL meshes and assembled vehicles.
//
// A forward renderer driven by the game's material data, lit the way FUEL lights its world (P28 port of
// the P26 renderer decode, ~/re-work/fuel/notes/p26-renderer.md): the sun path and Preetham colours, the
// warm sky light, the atmospheric scattering that is FUEL's fog, the per-material lighting of
// scattering_light.h, the 4-slice cascaded sun shadow, the sky inscatter with the sun / moon disc, and
// the HDR post chain (log-luminance auto exposure, bright pass, gaussian bloom and star, blue shift,
// the x+x^2+x^3+x^4 tone curve, tint, radial speed blur). Like the game everything is in gamma space:
// textures are read without sRGB decoding and the tone-mapped value is the display value (written
// through the inverse sRGB curve, so the sRGB output target stores it unchanged).
#pragma once

#import <Metal/Metal.h>
#include <simd/simd.h>

#include "core/atmosphere.hpp"
#include "core/weather.hpp"
#include "core/blob_shadow.hpp"
#include "core/clouds.hpp"
#include "core/sfx.hpp"
#include "core/vehicle_fx.hpp"  // PK23
#include "core/vehicle_lights.hpp"  // PK28: omni records / RTC switch
#include "core/rotshape.hpp"        // PK28: RotShape_Z halos / beams
#include "core/sand_tank.hpp"   // PK12
#include "core/weather_fx.hpp"
#include "core/tornado_fx.hpp"
#include "core/wind_tank.hpp"
#include "core/dpc.hpp"
#include "core/material_anim.hpp"
#include "core/resources.hpp"
#include "core/ground.hpp"
#include "core/ground_point.hpp"
#include "core/rider.hpp"
#include "core/custom_rider.hpp"
#include "core/terrain.hpp"
#include "core/terrain_mesh.hpp"
#include "core/vehicle.hpp"

#include <cstdlib>
#include "core/world.hpp"
#include "viewer/dynres.h"  // P93

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuel {

// ---- PK28: the Metal blend of a 0x856160 blend kind (shared by the mesh and RotShape pipelines; --rendercheck).
struct MetalBlend {
    MTLBlendOperation op = MTLBlendOperationAdd;
    MTLBlendFactor src = MTLBlendFactorOne, dst = MTLBlendFactorZero, srcAlpha = MTLBlendFactorOne, dstAlpha = MTLBlendFactorZero;
};
MetalBlend metalBlend(BlendKind kind);
// ---- PK28 end

// The set of open DPC files a viewer resolves resource IDs against.
class ResourceLibrary {
public:
    void add(std::unique_ptr<DpcFile> file) { files_.push_back(std::move(file)); }
    const std::vector<std::unique_ptr<DpcFile>>& files() const { return files_; }

    struct Hit {
        const DpcFile* file = nullptr;
        const DpcFile::Entry* entry = nullptr;
        explicit operator bool() const { return entry != nullptr; }
    };
    Hit find(uint32_t id, uint32_t classCrc) const;

private:
    std::vector<std::unique_ptr<DpcFile>> files_;
};

struct OrbitCamera {
    float yaw = 0.65f;     // radians around +y; 0 looks at the model from +z
    float pitch = 0.30f;   // radians above the horizon
    float distance = 2.1f; // in model radii
    float fovY = 0.70f;    // vertical field of view, radians
    float aim = 0;         // the camera looks this many metres above the focus
    // FUEL's cameras (physics/fuel_camera.hpp) give the pose directly: eye and target (world), roll about the
    // view axis, near plane and the horizontal FOV of a 4:3 frame (Hor+ for wider frames, A10).
    bool explicitPose = false;
    simd_float3 eye = {0, 0, 0}, target = {0, 0, 1};
    float roll = 0;      // radians
    float zNear = 0.25f;
    float fovH43 = 0;    // radians; > 0 replaces fovY
};

// Vehicle paint: the colour mixed in where the diffuse alpha is 0, and the
// livery texture (per LOD 1..3) that replaces slot 0 of LIVERY_XX_LOD<k>
// materials. Without a livery those materials show plain paint.
struct Paint {
    simd_float3 color = {60.0f / 255, 60.0f / 255, 60.0f / 255};  // gamma-space (as stored), like the game
    std::array<uint32_t, 3> livery{};
};

// One placed copy of a mesh.
struct MeshInstance {
    uint32_t meshId = 0;
    simd_float4x4 world = matrix_identity_float4x4;
};

// The scene renders into offscreen HDR targets of these formats; MeshRenderer::render
// writes the finished frame into a single-sample kOutputColorFormat texture.
inline constexpr MTLPixelFormat kRenderColorFormat = MTLPixelFormatRGBA16Float;
inline constexpr MTLPixelFormat kRenderDepthFormat = MTLPixelFormatDepth32Float;
inline constexpr MTLPixelFormat kOutputColorFormat = MTLPixelFormatBGRA8Unorm_sRGB;

// One effects billboard (core/sfx.hpp Particle) for FUEL's billboard_soft shaders (sfx.vhl/phl):
// GPU-animated ones (SubCode 0xe, vh_bboard_soft_static) carry their spawn state, CPU ones (0xb,
// vh_billboard_soft) their current position, alpha and size. 128 bytes, matches the shader.
struct SfxSprite {
    simd_float4 posAngle;  // start (GPU) or current (CPU) position, w = angle (radians)
    simd_float4 uv;        // u at the -x corners, v top, u at the +x corners, v bottom (FX atlas)
    simd_float4 colour;    // vertex rgb (gamma, as the game stores it), a = vertex alpha
    simd_float4 vel;       // xyz velocity (GPU), w = corner offset in metres (the quad spans +-w)
    simd_float4 acc;       // xyz acceleration (GPU), w = rotation speed (GPU)
    simd_float4 life;      // x spawn time (FX clock), y life, z = 1 GPU-animated / 0 CPU
    simd_float4 alphaP;    // GetParameterValue packs of the alpha and size codes (0x5fab60), GPU
    simd_float4 scaleP;
};
// The effects of one frame: sprites, the FX clock (C_USER0.x) and the camera fade (C_USER1.xy = PP5530, 5531).
struct SfxFrame {
    std::vector<SfxSprite> sprites;
    std::vector<sfx::DecalVertex> decals;   // tyre tracks: 0x5df2c0 strips, decalCounts vertices each
    std::vector<uint32_t> decalCounts;
    std::vector<blob::Record> blobs;        // P77: FakeShadows_G records of the frame (0x509630 draws them)
    float clock = 0;
    float fadeTo = 2.0f, fadeOut = 1.25f;
    // ---- PK23: the vehicle effects (core/vehicle_fx; viewer/vehicle_fx_render.mm): exhaust smoke (SubCode 0x15, atlas 6),
    // exhaust flames (0x15, atlas 0x30), scrape spark streaks (0x15) and exhaust embers (0x16, atlas 0 FX_SPARKLE)
    std::vector<vfx::SmokeSprite> exhaustSmoke;
    std::vector<vfx::FlameQuad> exhaustFlames;
    std::vector<vfx::Streak> streaks;
    // ---- PK23 end
};

struct FrameUniforms;  // renderer.mm
struct ViewCull;       // renderer.mm (P92: the scene pass's view culling)

// P65: one tornado's SpecialEffectsTornado_G draws (core/tornado_fx.hpp): the funnel (positions relative to `base`),
// its C_USER0.yz uv scroll, the base smoke and the two cloud-top discs.
struct TornadoDraw {
    std::array<double, 3> base{};
    std::array<float, 2> scroll{};
    std::vector<storm::TornadoVertex> funnel;
    std::vector<storm::TornadoSmoke> smoke;
    std::array<storm::TornadoCloudTop, 2> clouds{};
};

// p57: one face of the global cube map in Metal's cube convention (+X, -X, +Y, -Y, +Z, -Z): the direction it
// looks along (F) and its screen right (R) and up (U) in world axes; the face's view rows are R, U, -F.
void cubeFaceBasis(int face, simd_float3& F, simd_float3& R, simd_float3& U);

class MeshRenderer {
public:
    // Optional: fraction of the segment from the camera target to the eye that is
    // clear of world objects; the chase camera moves in front of what blocks it.
    std::function<float(simd_float3 from, simd_float3 to)> cameraClearance;
    MeshRenderer(id<MTLDevice> device, NSUInteger sampleCount, const ResourceLibrary& library);

    // Registers CPU-built geometry (the baked rider) under `key`, which then works
    // as a meshId in setScene. Materials are drawn like a Mesh_Z's, with their
    // livery bitmaps substituted. Replaces an earlier mesh with the same key;
    // pick keys that are not Mesh_Z IDs. Returns false (and says why) on bad data.
    bool addBakedMesh(uint32_t key, const BakedMesh& mesh, std::string* error);
    // New vertices (same count and layout) for a baked mesh: the animated riders, skinned every frame.
    void updateBakedVertices(uint32_t key, const std::vector<uint8_t>& vertices);
    // PK29: PlayAnim_Z material tracks (0x813e50) write an animated skin instance's own material clones (Skel_Z vf18
    // 0x66c0e0), which its batches draw with (by material name): material `slot` (index into BakedMesh::materials) of
    // baked mesh `key` takes m's diffuse, emissive and UV transform (the cube-lit path keeps its white diffuse).
    void setBakedMaterial(uint32_t key, size_t slot, const Material& m);

    // P92: loads (and caches) a Mesh_Z's GPU mesh and textures now, so its first draw does not (hitch prewarming).
    bool preloadMesh(uint32_t meshId, std::string* error) { return loadMesh(meshId, error) != nullptr; }
    // Shows one Mesh_Z at the origin; returns false with a reason on failure.
    bool setMesh(uint32_t meshId, std::string* error);
    // Shows several placed meshes. Meshes that fail to load are skipped and
    // reported in `error`; returns false only when nothing could be shown.
    bool setScene(const std::vector<MeshInstance>& instances, std::string* error);
    uint32_t triangleCount() const { return triangles_; }
    // Moves the instances of the last setScene (same order as its input;
    // `placement` is not applied) without reloading anything.
    void setInstanceWorlds(const std::vector<simd_float4x4>& worlds);
    // World objects drawn with the scene (not part of its bounds); meshes
    // load on first use. Returns the number of instances that could be shown.
    size_t setWorldInstances(const std::vector<MeshInstance>& instances);
    // Placed objects with FUEL's per-object level of detail and fade (p36): each frame picks the
    // level (Lod_Z::Draw 0x884170) and the fade alpha (ComputeFade 0x672950) from the camera
    // distance; faded-out objects are not drawn. Returns the number of objects kept.
    // set 0: placed objects (sub-worlds, USA1), set 1: AutoMesh / RoadMesh spawns, set 2: race markers (P69),
    // set 3: the props (P94: FUEL draws its dynamic objects with their Lod_Z too; set again whenever they move).
    size_t setLodObjects(const std::vector<PlacedMesh>& objects, int set = 0);
    // P92: whether the next frame can draw baked mesh `key` (addBakedMesh) as a placed LOD object, seen from `cam`:
    // some object using it is in range at a level that is this mesh, with fade alpha >= 0.01, as evaluateLodObjects
    // will decide (the same eye: explicit-pose cameras only; otherwise, and for meshes also used by props or cut-scene
    // objects, true). The world animations skip skinning classes nobody can see (their pose depends only on the clock).
    bool bakedMeshDrawable(uint32_t key, const OrbitCamera& cam);
    // Objects (indexes into the last setLodObjects list) not to draw (e.g. those that became props).
    void setLodHidden(const std::vector<uint32_t>& indexes) {
        lodHidden_.assign(lodObjects_[0].size(), 0);
        for (uint32_t i : indexes)
            if (i < lodHidden_.size()) lodHidden_[i] = 1;
    }
    fuel::LodView lodView;  // FUEL's main view factors (world.hpp)
    // ---- P94: the pop probe (viewer/pop_check.h). This frame's draw decision for every world object the
    // renderer holds, seen from `eye`, without drawing: placed / AutoMesh / marker / prop LOD objects (level mesh
    // and fade alpha as evaluateLodObjects decides), props of the plain prop list, and the plants (world
    // instances, with FUEL's decor fade when they carry one).
    struct PopSample {
        enum Kind : uint8_t { Placed, AutoMesh, Marker, Prop, PlantMesh, PlantCard, Vehicle /* P104: vehicles, riders */, kKinds };
        Kind kind;
        const void* mesh0;   // the object's level-0 mesh (its identity, with the position; plants: by position only)
        const void* level;   // the mesh drawn this frame
        simd_float3 pos;     // origin
        float alpha;         // fade alpha (1 = opaque; plants: FUEL's decor att)
        float radius;        // bounding radius (metres)
        uint32_t object = 0; // LOD objects: the node's object (diagnostics)
        float view = 0;      // LOD objects: the Lod view distance (link +88; 0 = none); plants: their decor leaf size
        bool gated = false;  // an AutoMesh spawn FUEL does not hold for this camera (alpha 0; autoCellHolds)
        bool lodRule = false;  // its mesh is the Lod_Z::Draw level (a LOD set), not a fixed mesh
    };
    void popProbe(simd_float3 eye, std::vector<PopSample>& out) const;
    // ---- P94 end
    // ---- PK15: the teleport's world regeneration (0x640fe0: the generated ground around the destination is built
    // before the player is released): the next frame builds every terrain chunk around its eye before drawing, as a
    // jump of more than 2 km does; terrainIdle() = no chunk queued or building (0x57f050).
    void settleTerrain() { terrainSettle_ = true; }
    bool terrainIdle() const;
    // ---- end PK15
    // Objects drawn / fading in the last frame (FUEL_STREAM_TRACE).
    size_t lodDrawn() const { return lodDrawn_; }
    size_t lodFading() const { return lodFading_; }
    // A small second list for props the physics moves; cheap to call every frame.
    size_t setPropInstances(const std::vector<MeshInstance>& instances);
    // P47: the objects of a playing cut-scene (RTC), drawn like the props; replaced every frame.
    size_t setRtcInstances(const std::vector<MeshInstance>& instances);
    // PK07: the race helicopter (heli_view.mm), drawn like the props; replaced every frame (a list of its own: the RTC
    // and the physics props each replace theirs).
    size_t setHeliInstances(const std::vector<MeshInstance>& instances);
    // ---- PK27: the F14 fly-by jet (f14_view.mm), drawn like the props while a pass flies; an empty list (the jet hidden
    // between passes) is a no-op once the last pass has been cleared, so the jet costs nothing while it is not flying.
    size_t setF14Instances(const std::vector<MeshInstance>& instances);
    // ---- PK27 end
    // Race beacons: xyz = ground position, w = kind (0 next checkpoint,
    // 1 the one after, 2 finish). Drawn as tall glowing columns.
    void setMarkers(const std::vector<simd_float4>& markers) { markers_ = markers; }
    // Debug: tree / bush trunk cylinders (P52, FUEL_P52_TRUNKS), drawn like the markers (see vsMarker).
    void setDebugTrunks(const std::vector<simd_float4>& trunks) { debugTrunks_ = trunks; }
    // Points the camera at `centre` instead of the scene's centre.
    void setFocus(simd_float3 centre) { center_ = centre; }
    // Other vehicles (race opponents), drawn with the scene: each its own parts (vehicle
    // space, like setScene) and paint; their meshes load with that vehicle's livery.
    // Replaces the earlier list. Vehicles far from the focus are not drawn.
    void setVehicles(const std::vector<std::vector<MeshInstance>>& parts, const std::vector<Paint>& paints);
    // Moves vehicle `k`'s parts (same order as its setVehicles input).
    void setVehicleWorlds(size_t k, const std::vector<simd_float4x4>& worlds);
    // ---- P104: FUEL's vehicle LOD (viewer/drive.h VehicleLods, notes p104-vehicle-lod.md). Vehicle `k`'s per-part
    // alpha this frame (setVehicles input order; 0 = not drawn: another Lod_Z level or faded out; < 1 = the Lod fade)
    // and, for the pop probe, its Lod_Z levels at the vehicle and rider nodes (-1: no LOD chain).
    struct VehicleLodState {
        simd_float3 anchor{}, riderAnchor{};
        int level = -1, riderLevel = -1;
        float alpha = 1, riderAlpha = 1;
    };
    // `lod1Alphas`: the pre-P104 picture (every vehicle at LOD1), drawn instead while vehicleLodOff (--perf-alternate vlod).
    void setVehicleLod(size_t k, const std::vector<float>& partAlphas, const VehicleLodState& st, const std::vector<float>& lod1Alphas = {});
    bool vehicleLodOff = false;
    void applyVehicleLodSwitch() {  // re-picks every vehicle's drawn alphas after vehicleLodOff changed
        for (OtherVehicle& ov : vehicles_)
            if (!ov.alphaLod.empty()) ov.alpha = vehicleLodOff ? ov.alphaLod1 : ov.alphaLod;
    }
    // ---- P104 end
    // ---- P85: FUEL's visual damage (core/vehicle_damage.hpp): the per-part morph VB of 0x7cb550 (3 floats per
    // vertex of the part mesh's morph VB; empty = undamaged), drawn by vsMain on the vertex-type-8 batches with
    // weight 1 (MeshPc_Z::Draw 0x848880). `vehicle` -1 = the player's scene (setScene input index), else the
    // setVehicles index. setScene / setVehicles drop the morphs they replace and bump morphEpoch().
    void setPartMorph(int vehicle, size_t part, const std::vector<float>& deltas);
    uint32_t morphEpoch() const { return morphEpoch_; }
    // ---- PK22: the vehicle visual links (core/vehicle_links.hpp). `vehicle` as setPartMorph (-1 the player).
    //  - setPartAdjust: the part's V:DYN pose as a right factor of its world (world x m; nullptr: none), applied by
    //    setInstanceWorlds / setVehicleWorlds;
    //  - setVehicleLamps: the lamp groups' clone alphas (0: the LIGHT_* batches are not drawn) for this frame;
    //  - setVehicleDirt: C_USER0.x (0x5dd8c0 / 0x5dd300) of the vehicle and its rider (< 0: the global `dirt`).
    void setPartAdjust(int vehicle, size_t part, const simd_float4x4* m);
    void setVehicleLamps(int vehicle, const std::array<float, 4>& alpha);
    void setVehicleDirt(int vehicle, float dirt);
    bool linksOff = false;  // --perf-alternate links: A = the lamps as stored (no switching)
    // For the checks: the lamp clone emissive of a vehicle (0x69da00: the first part's material colour x 5).
    std::array<simd_float3, 4> vehicleLampEmissive(int vehicle);
    // ---- PK22 end
    const ResourceLibrary& library() const { return library_; }  // P85: the meshes' morph tables
    // ---- P74: FUEL's car occlusion omnis (Vehicle_CarOcclusion 0x5cc410; notes p74-lighting.md 1): per vehicle
    // its node world (vehicle space -> world) and the box half extents (AssembledVehicle::occlusionExtents). The
    // ground and the static / vehicle meshes inside a box lose sky light (bOmni of ground.phl / scat_mesh.phl).
    struct CarOcclusion {
        simd_float4x4 world = matrix_identity_float4x4;
        std::array<float, 3> extents{0.75f, 0.75f, 0.75f};
    };
    void setCarOcclusion(std::vector<CarOcclusion> boxes) { carOcclusion_ = std::move(boxes); }
    // P77: the vehicles' headlight omnis this frame (world space; the renderer applies the day-factor switch).
    struct VehicleOmni {
        simd_float3 pos = {0, 0, 0}, dir = {0, 0, 1};  // dir = gDOmniDir (the spot's axis, pointing back at the light)
        simd_float4 value = {0, 0, 0, 0};              // gDOmniValue
        simd_float3 colour = {0, 0, 0};                // gDOmniColor
        float radius = 250.0f;  // PK28: the Omni_Z bound sphere x node scale (0x821830's test in 0x6a0200)
        uint32_t flags = 0x28400028u;  // PK28: Omni_Z +0x60 (0x20000000 spot: the deferred pass 0x7fdd80 draws only spots)
    };
    void setVehicleLights(std::vector<VehicleOmni> lights) { vehicleLights_ = std::move(lights); }
    // ---- PK28: the placed RotShape_Z objects (core/rotshape.hpp; RotShapePc_Z::vf16 0x835a00) and the sub-world test
    // (a course's shapes are drawn only while its sub-world is attached; "" = USA1 resident). FUEL_NO_ROTSHAPES=1 off.
    // Loads every placed shape's materials and makes their pipelines now (no compile or upload in a race frame).
    void setRotShapes(std::vector<rotshape::Placement> list, std::function<bool(const std::string&)> subActive);
    size_t rotShapeCount() const { return rotShapes_.size(); }
    // ---- PK28 end
    bool carOcclusionOn = std::getenv("FUEL_NO_CAR_OCCLUSION") == nullptr;  // off: before / after shots
    // ---- PK28: A / B switches (perf_check --perf-alternate omni=N / rotshapes): the light record cap (FUEL's 64;
    // FUEL_OMNI_MAX=n lowers it) and the RotShapes (FUEL_NO_ROTSHAPES=1 off).
    int omniCap = std::getenv("FUEL_OMNI_MAX") ? std::clamp(std::atoi(std::getenv("FUEL_OMNI_MAX")), 0, omni::kMaxLightRecords) : omni::kMaxLightRecords;
    bool rotShapesOn = std::getenv("FUEL_NO_ROTSHAPES") == nullptr;
    // ---- PK28 end
    // ---- PK28: RTC messages 0xc6 / 0xc7 (0x7aabe0, omni::rtcSwitch) on the Omni_Z flags of the world's omnis: the car
    // occlusion omni OMBREVOITURES (+0x60 = 0x08400028) leaves 0x7fd670's gather while its 0x400000 is clear. The
    // headlights (0x28400028) switch too, but their deferred list (0x6a0200) does not test the flag.
    void setRtcOmnis(bool on) { carOccOmniFlags_ = omni::rtcSwitch(carOccOmniFlags_, on); }
    uint32_t carOccOmniFlags_ = 0x08400028u;
    bool carOcclusionLoaded() const { return carSpot_ != nil; }
    // Half-size in metres of the area around the focus that gets sun shadows
    // (0: the scene's radius). Larger shows the shadows of vehicles nearby.
    float shadowRadius = 0;

    // Takes effect for meshes loaded afterwards (the colour applies at once).
    void setPaint(const Paint& paint);

    // ---- PK20: the garage preview scene (InGameObjects_Z ctor 0x68f910, draws 0x7ca0a0 / InGameObjectsPc_Z 0x845ac0;
    // viewer/garage_preview.mm, notes/pk20.md). A MeshRenderer of its own (never the session's) with `preview.on` draws
    // its instances into a rectangle of a menu target: depth cleared only (Renderer_QueueClear flags 5 -> ZBUFFER |
    // STENCIL), its own projection (0x7c9eb0 tangents, near / far of its camera), its own lights (0x68ffd0 x 0x4cfec0's
    // PP64 / PP65) and the CUBEMAP_GARAGE cube (mode 6), no sky, ground, shadow slices, cube faces or post chain: the
    // scene is in the viewport's object list with the HUDs, drawn after the world into the list's target (0x666905:
    // vf14 0x84fc80 switches the target), preset +0xd21 = 2 before the HUDs' 4 (the colours go out as the shaders
    // write them).
    struct PreviewScene {
        bool on = false;
        float tanX = 1, tanY = 1;      // view x / z and y / z at the rectangle's edges (InGameObjects_Z +0x50 / +0x4c)
        float zNear = 0.1f, zFar = 100.0f;
        simd_float4 lightDir, skyDir, sunColour, dlight, skyColour, skyAmbient;  // FrameUniforms' light rows
        std::array<simd_float4, 11> scat{};
        id<MTLTexture> envCube = nil;  // a game cube (Bitmap_Z type 2): sampled with GetCubeMap's z flip (hlsl_math.h)
    } preview;
    // Draws the scene seen from `cam` (explicit pose) into `rect` (pixels) of `target`, over what it holds.
    void renderPreview(id<MTLCommandBuffer> cb, id<MTLTexture> target, MTLRegion rect, const OrbitCamera& cam);
    id<MTLTexture> cubeTexture(uint32_t bitmapId);  // a Bitmap_Z cube from the library (nil: none)
    // ---- PK20 end

    // Applied to every instance of the next setScene/setMesh (e.g. to stand a
    // vehicle on the terrain).
    simd_float4x4 placement = matrix_identity_float4x4;

    bool cullBackFaces = true;
    bool wireframe = false;
    bool showGround = true;
    float dirt = 0.0f;  // 0 clean .. 1 fully dirty (vehicles with dirt maps)

    // Draws the world heightfield under the scene (with sky and haze) instead
    // of the ground plane.
    void setTerrain(const Heightfield& hf);
    // Ground colour, surface types and FUEL's ground detail textures (looked up in `world`).
    void setGround(const GroundMaps& maps, const DpcFile& world);
    bool showTerrain = true;

    // FUEL's wheel effects drawn with the next frames (replaces the earlier list).
    void setDust(const SfxFrame& frame) { dust_ = frame; }
    // P65: the tornadoes drawn with the next frames (tornado_render.mm).
    void setTornadoes(std::vector<TornadoDraw> t) { tornadoes_ = std::move(t); }
    // P65: the race weather (AddMissionParam start / end, 0x632250) on the camera's weather (0x5b89e0 overrides).
    std::function<void(WeatherAmounts&)> weatherOverride;
    // P86: Navigator3dHUD's GPS arrows (emitter 0x4bbba0, SpecialEffectsRibbon_G): each a rope of points (head first),
    // its bank vector (+0xbc) and alpha (+0x64); drawn in the scene with the next frames (tornado_render.mm).
    struct GpsRibbonDraw {
        std::vector<std::array<float, 3>> points;
        std::array<float, 3> up{0, 1, 0};
        float alpha = 0;
    };
    std::vector<GpsRibbonDraw> gpsRibbons;
    // PK19: Navigator3dHUD's FX_NAVIGATOR3D_LINE beams (0x4bf820, SpecialEffectsRibbon_G with draw flags 0): each a rope of
    // points (ground first), the particle colour (rgb x 8 half floats, alpha) and half width (+0x58); drawn camera-facing
    // with the next frames (tornado_render.mm encodeNavBeams).
    struct NavBeamDraw {
        std::vector<std::array<float, 3>> points;
        std::array<float, 4> colour{1, 1, 1, 1};
        float halfWidth = 0;
    };
    std::vector<NavBeamDraw> navBeams;
    // The FX atlas texture (FX_ATLAS_0 of DATAS/SHARED.DPC); setGround loads it next to the world.
    bool sfxAtlasLoaded() const { return sfxAtlas_ != nil; }
    // ---- p48: the splash and thunder tanks (WeatherManager +0x13c / +0x140) and placed engine particles.
    // Lightning: P49's thunder (DriveAudio::setLightningHook, its port of the 0x5e32a0 timer) calls this with each
    // bolt's ground position; the first call switches the thunder tank's own timer off so sound and flash stay in
    // step. Without audio the tank keeps its own timer. Safe from any thread.
    void lightningStrike(const std::array<float, 3>& groundPos);
    const wfx::ThunderTank* thunderTank() const { return thunder_.get(); }
    const wfx::SplashTank* splashTank() const { return splash_.get(); }
    // ---- PK24: the lens projections (rain drops, wheel splashes, mud, dirt strip; lens_render.mm). The drive loop feeds the
    // player's vehicle every frame (DustSystem::lens()); the entries live in core/weather_fx ScreenFx.
    void setLensVehicle(const wfx::LensVehicle& v) { lensVeh_ = v, lensVehFrame_ = frame_; }
    const wfx::ScreenFx* screenFx() const { return lens_.get(); }
    // ---- PK24 end
    // ---- PK12: STARTFX records (WeatherManager +0x130) and the sand-storm walls (sand tank +0x14c; sand_render.mm).
    // The world streamer hands over the STARTFX actions of its event volumes (0x570ae0), the hidden state of their
    // CollisionVol_Z nodes (0x5c08b0's dead test) and the releases of unloaded volumes (vf25 0x570780); the records and
    // the tank step with the effects; the tank's fog floor reaches the weather in fillAtmosphere (0x5bba30).
    void startFxFire(const sand::Fire& f);
    void startFxRelease(uint32_t node);
    void setStartFxHidden(std::unordered_map<uint32_t, bool> hidden);
    void startFxDropHidden(std::vector<uint32_t> nodes);  // PK14's retry: these nodes' records go at once (FUEL: detach / next frames)
    void setSandGroundColour(std::function<std::array<float, 4>(float, float, float)> c);  // 0x640760 (point context +0x70)
    const sand::StartFx* startFx() const;
    // ---- PK12 end
    // Placed Particles_Z quads of this frame, grouped by Material_Z (vh_particle).
    struct EngineParticle {
        simd_float4 pos;   // render position, w angle
        simd_float4 size;  // x, y; z orientation (0 camera-facing, 1 vertical (flags 4), 2 ground (flags 2))
        simd_float4 rgba;  // colour (c88 x c94, c64 alpha) x the distance fade
    };
    struct EngineBatch {
        uint32_t material = 0;
        uint32_t first = 0, count = 0;
    };
    void setEngineParticles(std::vector<EngineParticle> quads, std::vector<EngineBatch> batches) {
        engineQuads_ = std::move(quads), engineBatches_ = std::move(batches);
    }
    // Speed effects for this frame, 0..1: FUEL's radial blur (ph_motionblur) at speedBlur x speedBlurMax
    // (viewport +0x1dc = VehiclePP 574 x the speed fraction; 0.015 for most vehicles).
    float speedBlur = 0;
    float speedBlurMax = 0.015f;
    float speedBlurAbsolute = -1;  // >= 0: FUEL's blur strength (viewport +0x1dc) from the camera, used as is
    bool postEffects = true;  // false: tone mapping and auto exposure only (no bloom, star, radial blur)
    // ---- P102 [DEVIATION, the user's request]: film grain after the tone map, before the HUD (FUEL has none:
    // notes/p102-post-look.md). Amplitude in display units at the luminance weight's peak; 0 = off. The game sets it
    // from settings.txt `film_grain = off|light|strong` (default light, applySettings); the renderer's own default is
    // off so captures, films and --rendercheck stay deterministic. FUEL_FILM_GRAIN overrides it for one run.
    static constexpr float kFilmGrainLight = 0.035f, kFilmGrainStrong = 0.08f;
    float filmGrain = 0;
    // "off" | "light" | "strong" | a number -> amplitude; anything else -> `fallback`.
    static float filmGrainAmplitude(const std::string& v, float fallback);
    // Soften [DEVIATION, the user's request]: the HDR scene through a 3 x 3 [1 2 1] tent before the tone map, mixed in
    // by this amount (0 off, light 0.5, strong 1). settings.txt `soften = off|light|strong` (default off); FUEL_SOFTEN.
    static constexpr float kSoftenLight = 0.5f, kSoftenStrong = 1.0f;
    float soften = 0;
    static float softenAmount(const std::string& v, float fallback);
    // The grain's noise tile: texel (x, y) of the 256 x 256 R8 texture (a fixed integer hash; rendercheck mirrors it).
    static uint8_t grainNoiseByte(uint32_t x, uint32_t y) {
        uint32_t h = (x * 0x8da6b343u) ^ (y * 0xd8163841u) ^ 0x2545f491u;
        h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
        return uint8_t(h >> 24);
    }
    // ---- end P102
    // P67: the screen fader (BaseInGameDatas_G 0x438be0: black or white quad, rgb + alpha) over the final image, and
    // a tint replacing gTintLerp while w >= 0 (the race-end grade, MissionHUD 0x4b6050).
    simd_float4 screenFade = {0, 0, 0, 0};
    simd_float4 tintOverride = {0, 0, 0, -1};

    // ---- Time of day and weather (p26-renderer.md L1-L5)
    // The game's clock: 08:00 + elapsed seconds x PP371 (96 in pp001.tsc), or ForceDayTime.
    double forcedTimeOfDay = -1;   // seconds of the day (--time HH.MM / capture spots); < 0: the running clock
    bool timeFromCommandLine = false;  // --time given: capture spots keep it
    double gameClock = -1;         // elapsed game seconds; < 0: wall time since the renderer started
    float clockScale = 96.0f;      // PP371, read from pp001.tsc by setGround when present
    Weather weather;               // --weather: fixed inputs of the atmosphere hook 0x6409d0
    bool weatherFromWorld = true;  // the world's weather fields at the camera (core/weather.hpp) replace `weather`
    // P105: sample W+0xbc (0x77a240, the lowest ground near the camera) for the haze layer P7 = W+0xbc + 1000 and the
    // cloud layer heights; FUEL_P105_NO_GROUND_HAZE=1 restores the pre-P105 P7 = 500 (A/B captures only).
    bool terrainLowestGround = std::getenv("FUEL_P105_NO_GROUND_HAZE") == nullptr;
    const WeatherAmounts& weatherAmounts() const { return weatherNow_; }  // of the last frame
    float brightness = 1.0f;       // the Brightness option (0.5..2), default 1 (U2)
    // Seconds the auto exposure adapts over in the next frame (< 0: wall time since the last frame).
    float exposureDt = -1;
    double timeOfDay() const;      // the time used for the next frame, seconds
    // ---- PK04: SetDayClock 0x46ab10 and the race's weather / time of day (0x631bd0 + 0x631aa0, 0x632250, 0x5b89e0).
    // setDayClock(t): the game clock restarts at t seconds (40-bit ms, setDayClockMs) and runs on from there; the
    // time of day is 08:00 + clock x PP371. PK03 calls it with 43200 (08:00) on the first arrival at a hub.
    void setDayClock(double t);
    double dayClockSeconds() const;  // the game clock (s) the next frame uses (after any setDayClock)
    // ---- PK17: the pause menu (notes/pk17.md). setPauseBackdrop(true) = InGameMenuHUD vf07 0x46aca0 shown with the Pause
    // mode: the console "Pause" freezes the world (GameManager +0x40: no MaterialAnim advance 0x660aa0; LevelManipulator's
    // day clock stops; the weather manager (manipulator priority 0x12) and the effect tanks skip their update
    // (ManipulatorMgr 0x7b0cf0 runs only buckets 0..3, 20, 22, 24, 26..29 while paused)), 0x46abd0 saves the day clock
    // and shows the world at 20:00 (SetDayClock(43200 / PP371), World_SetTimeAndCloud(72000, cloud 0)) and the world's
    // sway factor W+0xfb4 / W+0x5c040 = 0. setPauseBackdrop(false) = vf08 0x46ade0 + "UnPause": the saved clock is put
    // back (the raw 40-bit value, so the time of day carries on where it was) and the world runs again.
    void setPauseBackdrop(bool on);
    bool pauseBackdrop() const { return pauseSky_; }
    float tintLerpA() const;  // gTintLerp.a this frame would use (renderer +0x1b8): the override, else the default
    // ---- end PK17
    // Race start (0x62e7ce / 0x62e7d6): the mission's AddMissionParam weather (hubinfos.tsc, by name) and the
    // scenario `flags` (mission +0x54) set the override block; every frame then lerps it by the player's progress
    // `player()` between `start` and `finish` (x z). No AddMissionParam line: the overrides are cleared.
    void beginRaceWeather(const std::string& missionId, const std::array<float, 3>& start, const std::array<float, 3>& finish,
                          std::function<std::array<double, 3>()> player, uint32_t flags = 0);
    void beginRaceWeather(const MissionWeather& m, const std::array<float, 3>& start, const std::array<float, 3>& finish,
                          std::function<std::array<double, 3>()> player, uint32_t flags = 0);
    void endRaceWeather();  // KillMission's vf14 0x5b9760: the flags cleared, the clock runs again
    const WeatherOverrides& weatherOverrides() const { return weatherOverrides_; }
    const RaceWeather& raceWeather() const { return raceWeather_; }
    // ---- end PK04
    const Atmosphere& atmosphere() const { return atmo_; }
    // CPU-vs-GPU check of the shader maths (--rendercheck): evaluates the MSL scattering, lighting and tone
    // curve on `inputs` (float4 each, see rendercheck.mm) for the given atmosphere.
    std::vector<simd_float4> shaderCheck(const Atmosphere& a, simd_float3 eye, const std::vector<simd_float4>& inputs,
                                         const std::vector<CarOcclusion>& boxes = {});
    bool grass = true;        // false: no grass tufts (settings file; FUEL_NO_GRASS=1 also disables)
    // ---- P93: dynamic resolution (settings.txt `dynamic_resolution = on|off`, `dynamic_resolution_min`; viewer/dynres.h).
    // When on, the 3-D scene, its effects and the post chain render at a scale of the output size picked from the GPU
    // time of the last frames (below 1 only when a frame would miss the budget), then upscale into the output; what the
    // caller draws over the output afterwards (HUD, menus) stays at the output size. Off by default here (captures,
    // --rendercheck): the game turns it on from the settings.
    bool dynamicResolution = false;
    DynamicResolution::Params dynamicResolutionParams;
    float fixedRenderScale = 0;  // > 0: every frame at this scale (quality comparisons), the controller unused
    // The upscaler: the sharpened bilinear (one pass, 0.6 ms at 3420x2224 on an M2) by default; MetalFX spatial
    // (settings.txt dynamic_resolution_upscaler = metalfx) costs 3.3 ms there, more than rendering at 0.8 saves (2.7 ms),
    // so it is an option only (P93 notes). Without MetalFX on the device the sharpened bilinear is used either way.
    enum class Upscaler { MetalFX, Sharpen, MetalFXTemporal };
    Upscaler upscaler = Upscaler::Sharpen;
    float lastRenderScale() const { return lastScale_; }  // the scale of the last render()
    bool metalFxAvailable() const;
    // ---- TAA [port performance option, not a FUEL behaviour] (taa_render.mm, notes ~/re-work/fuel/notes/taa.md):
    // settings.txt `dynamic_resolution_upscaler = metalfx-temporal`. While dynamic resolution (or a fixed --render-scale)
    // is on, every frame (native ones too: the history must not break when the scale reaches 1) is drawn with a
    // Halton(2, 3) sub-pixel jitter, writes motion vectors and a reactive mask, and its HDR scene goes through
    // MTLFXTemporalScaler to the output size before the tone map. Off (any other upscaler, or neither dynamic
    // resolution nor a fixed scale): nothing of this runs and the picture is the one without it, bit for bit.
    bool metalFxTemporalAvailable() const;
    static const char* upscalerName(Upscaler u);  // "sharpen" | "metalfx" | "metalfx-temporal" (settings.txt spelling)
    static Upscaler upscalerFromName(const std::string& v, Upscaler fallback);
    void resetTemporalHistory() { taaReset_ = true, ++cutSerial_; }  // a camera cut the renderer cannot see (teleport, respawn)
    // PHYSIQUE: counts camera cuts (seen or reported); the rider's glute springs restart on a change.
    uint32_t cutSerial() const { return cutSerial_; }
    // --rendercheck: kTaaCheck (taa_render.mm) on the GPU for a camera pair (viewProj without / with the jitter, eye, zn,
    // q = far / (near - far)) on a W x H render area. in: per point (ndc x, ndc y, depth, 0) in the jittered projection,
    // (world now, 1), (world in the last frame, 1); out: per point (camera motion uv, view distance, 0), (object motion uv,
    // clip w, 0).
    struct TaaCheckCamera {
        simd_float4x4 vp = matrix_identity_float4x4, jitteredVp = matrix_identity_float4x4;
        simd_float3 eye = {0, 0, 0};
        float zn = 0.25f, q = -1.0f;
    };
    std::vector<simd_float4> taaCheck(const TaaCheckCamera& cur, const TaaCheckCamera& prev, float W, float H, const std::vector<simd_float4>& in);
    bool temporalActive() const { return temporalFrame_; }  // the last render() went through the temporal scaler
    // The temporal scaler's GPU cost the dynamic-resolution controller assumes (ms at the output size): measured on the
    // M2 Air at 3420x2224, MetalFX temporal 8.8-9.9 ms + velocity 0.6 + output-size tone map ~1 (taa.md 8): so much that the
    // controller holds the lowest scale there. FUEL_DYNRES_UPSCALE overrides the controller's figure for either upscaler.
    float temporalUpscaleMs = 10.5f;
    // ---- TAA end
    // P93: settings.txt `shadow_filter = 2x2|3x3`: the sun shadow's edge filter. false (3x3) is FUEL's PixelShadow4
    // (four bilinear PCF lookups over 3 x 3 texels); true is one bilinear PCF lookup over 2 x 2 texels (cheaper,
    // harder edges). The game's default is 2x2 (applySettings); the renderer's own default keeps FUEL's look.
    bool shadow2x2 = false;
    // P90 [DEVIATION]: settings.txt `wind_debris` (smooth default / fuel) for the tornado-debris tank's motion
    // (core/wind_tank.hpp DebrisMotion); FUEL_WIND_DEBRIS=smooth|fuel overrides it.
    storm::DebrisMotion windDebris = storm::DebrisMotion::Smooth;
    float sceneRadius() const { return radius_; }
    // GPU time of the last completed frame, in milliseconds (0 until one finished).
    double lastGpuMs() const { return gpuMs_; }
    // ---- P92: the CPU side of the last render() (milliseconds) and the one-off work it did (pipeline compiles,
    // texture uploads, mesh builds, terrain chunk buffers): the causes --perf-check gives for slow frames.
    struct CpuTimes {
        double total = 0, terrain = 0, lod = 0, shadows = 0, cube = 0, scene = 0, effects = 0, post = 0;
        int pipelines = 0, textures = 0, meshes = 0, chunks = 0;
        // Work counts of the frame (statistics for the report).
        double shadowCasters = 0, shadowTriangles = 0, shadowChunks = 0, sceneDraws = 0, sceneTriangles = 0, sceneChunks = 0, grassTufts = 0;
        float renderScale = 1;  // P93: the frame's render scale (dynamic resolution)
    };
    const CpuTimes& cpuTimes() const { return cpu_; }
    // P92 profiling only (--perf-check --perf-breakdown): parts of the scene pass to leave out, to measure what each
    // costs (bits of PerfSkip). Always 0 otherwise.
    enum PerfSkip : uint32_t {
        SkipSky = 1u << 0, SkipTerrain = 1u << 1, SkipOpaque = 1u << 2, SkipCutout = 1u << 3, SkipGrass = 1u << 4,
        SkipWater = 1u << 5, SkipBlend = 1u << 6, SkipShadowLookup = 1u << 7, SkipOmnis = 1u << 8,
        // P95: the other fixed-cost passes' parts (shadow casters, the cube face).
        SkipShadowTerrain = 1u << 9, SkipShadowObjects = 1u << 10, SkipCube = 1u << 11,
        // (Toggles inside the shaders were tried for fsTerrain's parts: they change the shader's code generation and
        // with it the image, so the breakdown stays at draws and uniforms.)
    };
    uint32_t perfSkip = 0;
    // P95: the fixed-cost optimisations (bits of P95Opt), all on by default. A set bit turns one off: for --perf-alternate
    // opt=MASK (A/B on alternate frames) and FUEL_P95_OFF=MASK (image comparisons). Every one is image-identical.
    uint32_t p95Off = 0;
    enum P95Opt : uint32_t {
        // (bits 0, 2, 4, 5 were indexed grass, effect early-outs, uv-less shadow casters, 16-bit terrain indices:
        // measured without a gain, removed; P95 notes)
        P95LumCompute = 1u << 1,    // post: shrink + log luminance in one pass, the 2 x 2 reductions in one compute pass
        P95SlimVaryings = 1u << 3,  // vsMain / fsMain variants without the vertex outputs a material never reads
        P95CullInstances = 1u << 6, // scene pass: world / prop instances and opponents' vehicles wholly off screen left out
    };

    // ---- Grass tufts (p14): FUEL's 1 m-grid grass cards (core/ground_point.hpp).
    void setGrassAtlas(uint32_t bitmapId);                 // the Grass material's 4x4 atlas
    // Tree and bush meshes: their blended materials are drawn as cut-outs (alpha to coverage).
    void setFoliage(std::unordered_set<uint32_t> meshIds) { foliage_ = std::move(meshIds); }
    void setGrass(const std::vector<GrassTuft>& tufts);    // replaces the list
    // Fine ground colour around the player (levels 0, 1, 2: 1, 2, 4 m texels), kGroundTile
    // texels per side, gamma-space RGBA8 like the colour map (alpha = the ground's sky occlusion); the
    // terrain blends it over the 60 m map. `originX/Z` = world position of texel (0, 0)'s corner.
    static constexpr uint32_t kGroundTile = 256;
    // Also per texel the four surface types (type | weight byte << 8) and the road decal
    // (u across, v along, weight, decal slot or -1): the inputs of ground.phl's vertices.
    void setGroundTile(int level, double originX, double originZ, float extent, const std::vector<uint8_t>& rgba,
                       const std::vector<uint16_t>& types, const std::vector<float>& decal);
    static constexpr int kGroundTileLevels = 3;
    // Lake water (Paysage_BuildWaterMesh 0x7fbc10 rules, ground_water.phl ph_water): position,
    // colour (BGRA bytes as the engine packs them, alpha 0/10/40), uv = world xz, and the
    // depth of water above the ground at the vertex (port: stands in for the depth buffer read).
    struct WaterVertex {  // 32 bytes, matches the shader's packed layout
        float position[3];
        uint32_t colour;
        float uv[2];
        float depth;
        float pad = 0;
    };
    void setWater(const std::vector<WaterVertex>& vertices, const std::vector<uint32_t>& indices);
    // The water constants (WaterConstant_Z, 8 float4 from FUEL.exe .data 0xa26880) and the
    // water material's foam (slot 0) and normal (slot 5) maps.
    void setWaterLook(const std::array<simd_float4, 8>& constants, const DpcFile& world);
    // Road decal textures by slot: the GenWorld texture enums the road table uses (T[+0xa8]):
    // 0x2a..0x2e Road01..05 -> 0..4, 0x34 Dirt01 -> 5; -1 for none.
    static constexpr int kDecalSlots = 6;
    static int decalSlot(uint16_t textureEnum) {
        if (textureEnum >= 0x2a && textureEnum <= 0x2e) return int(textureEnum) - 0x2a;
        return textureEnum == 0x34 ? 5 : -1;
    }
    float grassRadius = 60.0f;                             // tufts shrink away towards it

    // Renders a frame into `target` (kOutputColorFormat, single-sample, any size):
    // shadows, the scene in HDR with MSAA, dust, then the post chain.
    void render(id<MTLCommandBuffer> cb, id<MTLTexture> target, const OrbitCamera& cam);

private:
    // Renders the sun's shadow map.
    void encodeShadows(id<MTLCommandBuffer> cb, FrameUniforms& fu, const std::vector<std::shared_ptr<const TerrainChunk>>& chunks);
    id<MTLCommandBuffer> frameCb_;          // render(): the scene pass encoder is made inside encode(), after the shadows
    MTLRenderPassDescriptor* frameRp_ = nil;
    // ---- PK20: the preview's composite into the menu target (garage_preview.mm), one pipeline per target format
    id<MTLLibrary> previewLib_ = nil;
    std::map<NSUInteger, id<MTLRenderPipelineState>> previewComposite_;
    // ---- PK20 end
    void fillAtmosphere(FrameUniforms& fu, simd_float3 eye, const simd_float4x4& viewRH, float fovY, float aspect, float zn);
    // Draws the scene into a pass with kRenderColorFormat/kRenderDepthFormat.
    void encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam);
    void ensureTargets(NSUInteger width, NSUInteger height);
    void encodeDust(id<MTLCommandBuffer> cb);
    // P92: the one render pass all the effects share (made at the first effect that draws, closed by fxEnd).
    id<MTLRenderCommandEncoder> fxEncoder(id<MTLCommandBuffer> cb, const char* name);
    void fxEnd();
    id<MTLRenderCommandEncoder> fxEnc_ = nil;
    void encodeTanks(id<MTLCommandBuffer> cb);  // rain / snow (weather tanks)
    void encodeBlobs(id<MTLCommandBuffer> cb);   // P77: blob shadows (FakeShadows_G::vf19 0x509630)
    void encodeDecals(id<MTLCommandBuffer> cb);  // tyre tracks (p48)
    void encodeWeatherFx(id<MTLCommandBuffer> cb, float dt);  // splashes, lightning bolts, the thunder flash (p48)
    void encodeEngineParticles(id<MTLCommandBuffer> cb);  // Particles_Z (p48)
    id<MTLRenderPipelineState> decalPipeline_, fxPipeline_, particlePipeline_, particleAddPipeline_;
    id<MTLRenderPipelineState> blobPipeline_;  // P77: blob shadows (FakeShadows_G)
    id<MTLTexture> blobTex_ = nil;             // P77: FAKESHADOW
    id<MTLTexture> pneuDiffuse_ = nil, pneuNormal_ = nil;  // FX_PNEU (atlas 15) slots 0 and 3
    bool pneuCompressed_ = false;                           // DXT5 normal map: decal.phl's bCompression (wyz)
    id<MTLTexture> splashTex_ = nil, boltTex_ = nil;        // atlas 9 (splash tank) and 0x19 (thunder) materials
    // ---- PK23: the vehicle effects (vehicle_fx_render.mm): smoke.vhl vh_smoke + smoke.phl ph_smoke, one triangle list
    void createVehicleFxPipeline();
    void encodeVehicleFx(id<MTLCommandBuffer> cb);
    id<MTLRenderPipelineState> vfxPipeline_ = nil;
    id<MTLTexture> sparkleTex_ = nil;  // FX atlas 0 (FX_SPARKLE)
    // ---- PK23 end
    // ---- PK12: sand-storm walls (sand_render.mm)
    void makeSandPipeline();
    void updateSandFx(float dt);
    void encodeSandWalls(id<MTLCommandBuffer> cb);
    std::unique_ptr<sand::StartFx> startFx_;
    sand::Params sandParams_;
    std::mutex sandMu_;
    struct SandOp {
        int kind = 0;  // 0 fire, 1 release, 2 drop (the nodes hidden when it was queued)
        sand::Fire fire;
        std::vector<uint32_t> nodes;
    };
    std::vector<SandOp> sandQueue_;
    std::unordered_map<uint32_t, bool> sandHidden_;
    std::function<std::array<float, 4>(float, float, float)> sandColour_;
    std::unordered_map<uint64_t, std::array<float, 4>> sandColourCache_;
    double sandAcc_ = 0;
    float sandFog_ = 0;
    bool sandFogSet_ = false;
    id<MTLRenderPipelineState> sandPipeline_ = nil;
    std::array<id<MTLTexture>, 2> sandTex_{};  // FX_SANDSTORM01 slots 0 / 3
    bool sandNormalBC3_ = false;
    id<MTLBuffer> sandIdx_ = nil;
    size_t sandIdxCount_ = 0;
    int sandIdxRings_ = -1, sandIdxCols_ = -1;
    // ---- PK12 end
    // ---- PK24: lens projections
    void createLensPipeline();
    void updateLens(float dt);
    void encodeLens(id<MTLCommandBuffer> cb);
    struct LensTex {
        id<MTLTexture> diffuse = nil, normal = nil;  // the atlas entry's material, slots 0 and 3
        bool compressed = false;                     // DXT5 normal map (wyz)
    };
    std::array<LensTex, 12> lensTex_;  // FX atlas entries 30..41 (dirt, mud, splash, rain drops)
    std::unique_ptr<wfx::ScreenFx> lens_;
    wfx::LensVehicle lensVeh_;
    uint32_t lensVehFrame_ = 0;
    bool lensWarmed_ = false;
    std::unique_ptr<sfx::Well512> lensWell_;
    id<MTLRenderPipelineState> lensPipeline_ = nil;
    id<MTLTexture> lensCopy_ = nil;  // the scene colour for ph_screenwater
    // ---- PK24 end
    std::unique_ptr<wfx::SplashTank> splash_;
    std::unique_ptr<wfx::ThunderTank> thunder_;
    std::unique_ptr<sfx::Well512> wfxWell_;               // the tanks' WELL512 draws (port: own state from FUEL's seed)
    uint32_t wfxGame_ = 0x1234567u;                       // 0x6460f0 stand-in
    float flashAlpha_ = 0, flashTime_ = 1;                 // HUD flash (white, 1 s)
    float fxClock_ = 0;
    std::mutex strikeMu_;
    std::vector<std::array<float, 3>> strikes_;
    bool externalThunder_ = false;
    std::chrono::steady_clock::time_point fxWall_;
    bool haveFxWall_ = false;
    void updateWeatherFx(float dt);
    std::vector<EngineParticle> engineQuads_;
    std::vector<EngineBatch> engineBatches_;
    id<MTLRenderPipelineState> tankPipeline_;
    void encodeClouds(id<MTLCommandBuffer> cb);  // billboard clouds (cloud tank, p37)
    id<MTLRenderPipelineState> cloudPipeline_;
    id<MTLTexture> cloudTex_ = nil;               // FX_BILLBOARD_CLOUDS (FX atlas entry 14)
    std::unique_ptr<clouds::Tank> cloudTank_;
    // P65 (tornado_render.mm): SubCodes 0x23 / 0x25 / 0x26 and their textures (TORNADE_CYLINDRE slots 0 / 3,
    // FX_TORNADEHAUT slots 0 / 3, the FX atlas entry 6 material).
    void makeTornadoPipelines();
    void encodeTornadoes(id<MTLCommandBuffer> cb);
    std::vector<TornadoDraw> tornadoes_;
    id<MTLRenderPipelineState> tornadoPipeline_ = nil, tornadoSmokePipeline_ = nil, tornadoCloudPipeline_ = nil;
    std::array<id<MTLTexture>, 5> tornadoTex_{};
    std::array<bool, 2> tornadoNormalBC3_{};
    // P65 (tornado_render.mm): the wind (+0x148, SubCode 0x19) and tornado debris (+0x154, 0x1a) tanks.
    void encodeWindTanks(id<MTLCommandBuffer> cb, float dt);
    std::unique_ptr<storm::WindTank> windTank_, debrisTank_;
    id<MTLBuffer> windTankBuf_ = nil, debrisTankBuf_ = nil;
    id<MTLRenderPipelineState> windTankPipeline_ = nil;
    std::array<id<MTLTexture>, 2> windTankTex_{};
    // P86 (tornado_render.mm): the GPS arrows, SubCode 0x1e (sfx.vhl vh_billboard / sfx.phl ph_billboard), FX atlas 0x15.
    void encodeGpsRibbons(id<MTLCommandBuffer> cb);
    id<MTLRenderPipelineState> gpsRibbonPipeline_ = nil;
    id<MTLTexture> gpsArrowTex_ = nil;
    // PK19 (tornado_render.mm): the navigator beams, the same SubCode 0x1e billboard ribbons with FX atlas 0x1d.
    void encodeNavBeams(id<MTLCommandBuffer> cb);
    id<MTLRenderPipelineState> navBeamPipeline_ = nil;
    id<MTLTexture> navLineTex_ = nil;
    id<MTLBuffer> cloudBuf_ = nil;
    uint32_t cloudRng_ = 0x6d2b79f5u;
    std::chrono::steady_clock::time_point cloudClock_;
    bool haveCloudClock_ = false;
    WindGusts gusts_;                             // 0x5b8680 gusts on the weather wind
    simd_float3 wind_ = {0, 0, 0};                // WeatherManager +0xc4 after the gusts
    double windClock_ = 0;
    id<MTLBuffer> rainDrops_ = nil, snowDrops_ = nil;
    simd_float3 rainSmooth_ = {0, 0, 0}, rainShift_ = {0, 0, 0}, snowShift_ = {0, 0, 0}, tankEye_ = {0, 0, 0};
    bool haveTankEye_ = false, haveTankClock_ = false;
    std::chrono::steady_clock::time_point tankClock_;
    std::shared_ptr<const std::vector<sfx::AtlasEntry>> sfxTables_;  // the FX atlas (0x8e6ae0 replay)
    void encodePost(id<MTLCommandBuffer> cb, id<MTLTexture> target);
    // ---- Grass tufts (p14)
    void createGrassPipeline();
    void encodeGrass(id<MTLRenderCommandEncoder> enc, const ViewCull& cull);
    id<MTLRenderPipelineState> grassPipeline_;
    id<MTLBuffer> grassBuffer_;
    id<MTLTexture> grassAtlas_;
    size_t grassCount_ = 0;
    // P92: the tufts in runs of one 24 m grass chunk (the streamer appends them chunk by chunk), each with a box that
    // holds its cards at full size and sway; runs outside the view are not drawn (the others keep their order).
    struct GrassRun {
        uint32_t first = 0, count = 0;
        simd_float3 lo = {0, 0, 0}, hi = {0, 0, 0};
    };
    std::vector<GrassRun> grassRuns_;
    std::chrono::steady_clock::time_point grassEpoch_;

    enum class AlphaMode : uint32_t { Opaque = 0, Test = 1, Blend = 2 };
    enum Feature : uint32_t {
        PaintOnly = 1u << 0,
        HasOcclusion = 1u << 1,
        HasNormal = 1u << 2,
        HasSpecular = 1u << 3,
        HasNormalLocal = 1u << 4,
        HasDirt = 1u << 5,
        Reflective = 1u << 6,
        GroundTint = 1u << 7,  // grey detail diffuse: multiplied by the local ground colour
        Foliage = 1u << 8,     // tree / bush cut-out: fades out right at the camera
        TwoSided = 1u << 9,    // CULLMODE none (flag 0x8000 or blended): back faces keep the stored normal
        NoCarOcclusion = 1u << 10,  // P74: set per draw for skinned batches (scat_skin: no car occlusion omni)
        // PHYSIQUE (port deviation): the player's custom rider: skin (wrap + subsurface tint + sheen), hair (strand
        // highlight along the bitangent), eyes (wet highlight).
        SkinShade = 1u << 11,
        HairShade = 1u << 12,
        EyeShade = 1u << 13,
        // [upgrades U5] the weight stage's carbon weave on the paint (codes 17 / 19) of a derived vehicle mesh
        // (core/upgrade_visuals): lodBias2.z = level 1..3, lodBias2.w = the front z (mesh space); vsMain / fsMain variant
        // bit 16 (function constant 16).
        Carbon = 1u << 14,
    };
    enum Slot { kDiffuse = 0, kOcclusion = 2, kNormal = 3, kSpecular = 4, kNormalLocal = 5, kDirt = 6 };

    struct GpuMaterial {
        std::array<id<MTLTexture>, 7> textures;
        simd_float4 diffuse = {0.7f, 0.7f, 0.7f, 1.0f};
        simd_float4 specular = {0.5f, 0.5f, 0.5f, 20.0f};
        simd_float4 emissive = {0, 0, 0, 0};
        simd_float2 fresnel = {0, 0};
        AlphaMode alpha = AlphaMode::Opaque;
        bool lit = true;
        bool skip = false;
        uint32_t samplerIndex = 0;  // bit 0 clamp U, bit 1 clamp V
        uint32_t code = 0;          // MatCodeId (flags & 0x1f)
        uint32_t features = 0;
        // p57: UV transform (MaterialMat, hlsl_math.h: u' = (u, v, 1).c0, v' = (u, v, 1).c1; BindMaterial 0x84f470).
        simd_float4 uv0 = {1, 0, 0, 1};  // (+0x2c, +0x30, +0x3c, +0x40): u' = u.x + v.z, v' = u.y + v.w
        simd_float2 uv1 = {0, 0};        // (+0x4c, +0x50)
        float alphaRef = 0;              // ALPHAREF: 0, or 128/255 with material flag 0x100000 (0x848880, 0x858810)
        bool twoSided = false;           // CULLMODE none: flag 0x8000 or a blended draw (0x848880 pass bits)
        int anim = -1;                   // index into materialAnims_ (an auto-playing MaterialAnim_Z), or -1
        bool fxAdditive = false;         // p48's particle rule: an opaque bitmap (alpha kind 0), opacity 1, no 0x180000
        uint32_t flags = 0;              // P72: Material_Z flags (blended phase, fade pre-pass rules of 0x848880)
        int lamp = -1;                   // PK22: a LIGHT_BACK / STOP / POSITION / HEAD material (vlinks::Lamp), -1 none
        uint32_t nameCrc = 0;            // [upgrades U5] the Material_Z's name (a derived mesh leaves batches out by name)
        simd_float4 lodBias = {0, 0, 0, 0};   // PK28: Bitmap_Z +0x24 of texture slots 0 / 2 / 3 / 4 (0x84ca80)
        simd_float4 lodBias2 = {0, 0, 0, 0};  // PK28: slots 5 / 6
    };
    struct Draw {
        uint32_t vb = 0;
        uint32_t firstIndex = 0;
        uint32_t indexCount = 0;
        uint32_t material = 0;
        bool sprite = false;  // camera-facing sprite batch
        bool skinned = false;  // P74: Vertex1Blend / 4Blend (+ morph) batches and baked riders: scat_skin, omni = 0
        bool morph = false;    // P85: vertex type 8 (VertexND_Z + a morph stream): takes the instance's damage morph
    };
    // GPU copy of one Mesh_Z, cached by ID.
    struct GpuMesh {
        std::vector<id<MTLBuffer>> vbs;
        std::vector<uint32_t> vbStrides;
        id<MTLBuffer> indices;
        std::vector<GpuMaterial> materials;  // [0] = no material
        std::vector<Draw> draws[3];          // by AlphaMode
        simd_float3 boundsMin = {0, 0, 0}, boundsMax = {0, 0, 0};
        uint32_t triangles = 0;
        float spriteReach = 0;  // P92: the largest sprite corner offset (metres, before the instance scale)
    };
    struct Instance {
        const GpuMesh* mesh = nullptr;
        simd_float4x4 world = matrix_identity_float4x4;
        id<MTLBuffer> morph = nil;  // P85: the part's damage morph VB (float3 per vertex), nil = undamaged
    };

    const GpuMesh* loadMesh(uint32_t meshId, std::string* error);
    // The same with another livery than the current paint's (cached separately).
    const GpuMesh* loadMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>& livery);
    std::unique_ptr<GpuMesh> buildMesh(uint32_t meshId, std::string* error);
    // ---- [upgrades U5] the visible upgrades' meshes (core/upgrade_visuals): a procedural key is added as a baked mesh at
    // first use, a derived key is a copy of its base mesh without some batches and / or with the carbon paint.
    const GpuMesh* loadUpgradeMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>* livery, bool* handled);
    std::unique_ptr<GpuMesh> deriveMesh(const GpuMesh& base, uint32_t key);
    id<MTLTexture> solidTexture(const std::array<float, 4>& rgba);
    std::unordered_map<uint32_t, id<MTLTexture>> solidTextures_;
    // ---- [upgrades U5] end
    float shadowHalfSize() const { return std::max(radius_ * 1.05f, shadowRadius); }
    id<MTLTexture> texture(uint32_t bitmapId, bool srgb, uint8_t* alphaKind);
    id<MTLTexture> uploadBitmap(const DpcFile& file, const DpcFile::Entry& entry, bool srgb, uint8_t* alphaKind);
    struct TextureStats {
        simd_float3 mean = {0.5f, 0.5f, 0.5f};  // of the DXT colour endpoints, 0..1
        bool greyDetail = false;  // pure grey around 0.5: takes the ground colour
    };
    const TextureStats& textureStats(uint32_t bitmapId);
    float bitmapLodBias(uint32_t bitmapId);  // PK28: the bitmap's own mip LOD bias (+0x24, 0x84ca80), 0 when unknown
    std::unordered_map<uint32_t, TextureStats> textureStats_;
    // P95: `variant` = vsVariant(material, sprite) (15 = every vertex output).
    id<MTLRenderPipelineState> pipeline(uint32_t stride, AlphaMode mode, uint32_t variant = 15u);
    // P72: depth-only twin of pipeline() for the fade pre-pass (MeshPc_Z::Draw 0x848880 pass bits 0x83 / 0x03).
    // PK28: 0x856160's blend; warmOnly starts the specialised variant's async compile and returns nil.
    id<MTLRenderPipelineState> blendPipeline(uint32_t stride, uint32_t flags, uint32_t variant, bool warmOnly = false);
    id<MTLRenderPipelineState> prepassPipeline(uint32_t stride, uint32_t variant = 15u);
    uint32_t vsVariant(const GpuMaterial& m, bool sprite) const;
    id<MTLRenderPipelineState> mainPipeline(uint64_t key, uint32_t variant, MTLRenderPipelineDescriptor* (^describe)(void));
    std::mutex asyncPipeMutex_;
    std::unordered_map<uint64_t, id<MTLRenderPipelineState>> asyncPipes_;  // P95: background compiles (nil while running)
    // ---- p57: MaterialAnim_Z (core/material_anim.hpp): auto-playing anims (state 0x40) by target material.
    struct AnimatedMaterial {
        MaterialAnim anim;
        MaterialAnimClock clock;
        Material base;           // the target Material_Z as loaded
        simd_float4 uv0 = {1, 0, 0, 1};
        simd_float2 uv1 = {0, 0};
        simd_float4 diffuse = {1, 1, 1, 1}, specular = {0, 0, 0, 0}, emissive = {0, 0, 0, 0};
        id<MTLTexture> texture = nil;  // flip-book frame, nil = the material's own
        bool evaluated = false;
    };
    std::vector<AnimatedMaterial> materialAnims_;
    std::unordered_map<uint32_t, int> animByMaterial_;
    size_t animFilesScanned_ = 0;
    std::chrono::steady_clock::time_point animClock_;
    bool haveAnimClock_ = false;
    void scanMaterialAnims();
    void updateMaterialAnims(float dt);
    // ---- p57: FUEL's global cube map (sGlobalCube = gCubeMapZ, 0x837950): 128 x 128, one face per frame
    // round robin (0x757c20), the sky dome and the ground only (GenWorld kinds 3..8 + 24, scene objects off).
    void encodeCube(id<MTLCommandBuffer> cb, const FrameUniforms& fu, const std::vector<std::shared_ptr<const TerrainChunk>>& chunks,
                    bool allFaces);
    id<MTLTexture> globalCube_ = nil, cubeDepth_ = nil;
    id<MTLRenderPipelineState> cubeSkyPipeline_, cubeTerrainPipeline_;
    int cubeFace_ = -1;     // scene +0x194
    bool cubeFresh_ = true; // all six faces on the first frame and after a jump
    id<MTLRenderPipelineState> shadowPipeline(uint32_t stride, bool cut);
    GpuMaterial makeMaterial(uint32_t materialId, uint32_t diffuseOverride = 0, uint32_t specularOverride = 0);
    // ---- PHYSIQUE: a custom rider material (core/custom_rider.hpp) from its DDS textures (cached by path).
    GpuMaterial makeCustomMaterial(const CustomMaterial& c);
    id<MTLTexture> ddsTexture(const std::string& path, bool* hasAlpha = nullptr);
    std::unordered_map<std::string, id<MTLTexture>> ddsTextures_;
    // ---- PHYSIQUE end
    std::unordered_map<uint32_t, GpuMaterial> particleMats_;  // Particles_Z materials (p48)
    simd_float4x4 sunViewProj() const;

    id<MTLDevice> device_;
    NSUInteger sampleCount_;
    const ResourceLibrary& library_;
    id<MTLLibrary> shaders_;
    id<MTLDepthStencilState> depthWrite_, depthNoWrite_;
    id<MTLSamplerState> samplers_[4];
    id<MTLTexture> white_, shadowMap_;  // shadowMap_: the 4-slice atlas, W x 4W depth (reversed)
    // ---- FUEL's cascaded shadow (shadows.md): settings from FUEL.exe, per-slice light matrices
    ShadowSettings shadowSettings_;
    PostDefaults postDefaults_;
    bool engineSettingsRead_ = false;
    int shadowCounter_ = -1;                            // scene+0x190
    std::array<std::array<double, 16>, 4> sliceLvp_{};  // world -> light clip of each slice (row-major)
    std::array<bool, 4> sliceValid_{};
    std::array<float, 4> sliceBias_{};
    id<MTLRenderPipelineState> terrainShadowPipeline_;
    id<MTLDepthStencilState> shadowDepth_;
    struct ChunkBounds { float minY = 0, maxY = 0; };
    std::unordered_map<uint64_t, ChunkBounds> chunkBounds_;
    Atmosphere atmo_;
    std::chrono::steady_clock::time_point epoch_ = std::chrono::steady_clock::now(), lastRender_{};
    bool haveLastRender_ = false;
    id<MTLRenderPipelineState> groundPipeline_, terrainPipeline_, skyPipeline_, markerPipeline_;
    std::vector<simd_float4> markers_, debugTrunks_;
    id<MTLDepthStencilState> depthAlways_;
    id<MTLTexture> heightTex_, groundColour_, groundSurface_, groundDetail_;
    id<MTLTexture> groundFine_;  // 3-slice arrays (setGroundTile): colour, surface types, road decal
    id<MTLTexture> groundTypes_, groundDecal_;
    // P92: the near bands' per-triangle draw sets (kGroundDrawSet), RGBA32Uint, slices level * 2 + half (levels 0, 1),
    // remade when a level-0 / level-1 fine tile is uploaded; fsTerrain reads it at texture 5.
    id<MTLTexture> groundDrawSet_ = nil;
    id<MTLComputePipelineState> drawSetPipeline_ = nil;
    id<MTLRenderPipelineState> waterPipeline_;
    id<MTLBuffer> waterVertices_, waterIndices_;
    uint32_t waterIndexCount_ = 0;
    std::array<simd_float4, 8> waterConst_{};
    bool haveWaterLook_ = false;
    id<MTLTexture> waterFoam_, waterNormal_;
    id<MTLTexture> decalTex_[kDecalSlots] = {};
    float decalLodBias_[kDecalSlots] = {};  // PK28: Bitmap_Z +0x24 of each decal slot (0x84ca80)
    simd_float4 groundFineRect_[kGroundTileLevels] = {};  // origin x, z, extent, 1 when loaded
    // Fine tiles waiting for the next frame: setGroundTile stages them, render() uploads them and
    // builds the mips in the frame's own command buffer before it draws, then switches the rect.
    // (Writing the textures in place, with mips on a second queue, let frames still in flight or the
    // next frame sample half-updated tiles: a one-frame dark flicker of the ground.)
    struct PendingTile {
        bool set = false;
        id<MTLBuffer> rgba = nil, types = nil, decal = nil;
        simd_float4 rect{};
    };
    std::array<PendingTile, kGroundTileLevels> pendingTiles_{};
    // P92: reused staging buffers for the fine tiles (setGroundTile), a ring per level.
    static constexpr size_t kTileRing = 4;
    struct TileStaging {
        id<MTLBuffer> rgba = nil, types = nil, decal = nil;
    };
    std::array<std::array<TileStaging, kTileRing>, kGroundTileLevels> tileStaging_{};
    std::array<size_t, kGroundTileLevels> tileStagingNext_{};
    std::mutex pendingTilesMutex_;
    void flushGroundTiles(id<MTLCommandBuffer> cb);
    simd_float4 groundFar_ = {0, 0, 0, 0};  // p16: far-band forest colour, w = decoded colour map
    id<MTLBuffer> terrainIndices_;
    uint32_t terrainIndexCount_ = 0;
    Heightfield terrainData_;
    // FUEL's ground mesh (terrain_mesh.hpp): quadtree chunks built on worker threads.
    std::unique_ptr<TerrainLod> terrainLod_;
    struct ChunkGpu {
        id<MTLBuffer> vb;
        uint64_t lastFrame = 0;
        simd_float3 lo = {0, 0, 0}, hi = {0, 0, 0};  // P92: bounds of every vertex and its morph target (view culling)
    };
    std::unordered_map<uint64_t, ChunkGpu> chunkGpu_;
    uint64_t chunkFrame_ = 0;
    simd_float3 lastTerrainEye_ = {1e9f, 0, 1e9f};
    bool terrainSettle_ = false;  // PK15
    simd_float4 terrainInfo_ = {0, 0, 1, 0};  // origin x, origin z, spacing, metres for a texel value of 1.0
    std::unordered_map<uint64_t, id<MTLRenderPipelineState>> pipelines_;
    std::unordered_map<uint32_t, id<MTLRenderPipelineState>> shadowPipelines_;
    std::unordered_map<uint64_t, id<MTLTexture>> textures_;  // key: id << 1 | srgb
    std::unordered_map<uint32_t, uint8_t> textureAlpha_;
    std::unordered_map<uint32_t, float> textureBias_;  // PK28: Bitmap_Z +0x24 by bitmap ID (bitmapLodBias)
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> meshes_;
    std::unordered_set<uint32_t> foliage_;
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> bakedMeshes_;  // addBakedMesh; survive setPaint
    // P92: updateBakedVertices's vertex buffers, a ring per baked mesh (reused instead of one new buffer a frame).
    static constexpr size_t kBakedRing = 5;
    struct BakedRing {
        std::vector<id<MTLBuffer>> buffers;
        size_t next = 0;
    };
    std::unordered_map<uint32_t, BakedRing> bakedRing_;

    Paint paint_;
    simd_float3 sunDir_;
    std::vector<Instance> instances_;
    std::vector<Instance> worldInstances_;  // flat list (shadow casters)
    struct WorldGroup {
        const GpuMesh* mesh;
        id<MTLBuffer> matrices;
        uint32_t count;
        uint32_t offset = 0;  // bytes into `matrices`
    };
    std::vector<WorldGroup> worldGroups_;   // instanced draws
    // P95: the same groups with each instance's bounding sphere (xyz centre, w radius: the mesh box through the instance's
    // scale plus the sprite corner reach), and this frame's groups without the instances wholly off screen (the scene
    // pass draws those; the shadow slices keep every caster).
    struct CullGroup {
        const GpuMesh* mesh;
        std::vector<simd_float4x4> worlds;
        std::vector<simd_float4> spheres;
    };
    std::vector<CullGroup> worldCull_, propCull_;
    std::vector<WorldGroup> worldVis_, propVis_;
    static CullGroup cullGroupOf(const GpuMesh* gm, const std::vector<simd_float4x4>& worlds);
    void cullGroups(const std::vector<CullGroup>& in, std::vector<WorldGroup>& out, const ViewCull& cull);
    bool vehicleOffScreen(size_t v, const ViewCull& cull) const;  // every part of opponent v wholly off screen
    std::vector<Instance> propInstances_;   // movable props (shadow casters)
    struct LodObject {
        PlacedMesh p;
        simd_float4x4 world;
        std::vector<const GpuMesh*> levels;  // nullptr = not drawable
    };
    static constexpr size_t kLodSets = 4;  // P94: set 3 = the props (drawn with their Lod_Z like FUEL's dynamic objects)
    std::array<std::vector<LodObject>, kLodSets> lodObjects_;
    std::array<std::unordered_map<const GpuMesh*, std::vector<uint32_t>>, kLodSets> lodIndex_;  // P92: objects by level mesh
    std::array<bool, kLodSets> lodIndexDirty_{{true, true, true, true}};
    std::vector<char> lodHidden_;
    std::vector<Instance> lodVisible_;      // this frame's levels (shadow casters; fade in column 0 w)
    std::vector<WorldGroup> lodGroups_;     // this frame's instanced draws (objects at full alpha)
    std::vector<Instance> lodFadingObjs_;   // P72: this frame's fading objects (alpha < 1, fade in column 0 w):
                                            // drawn in the sorted blended phase, not instanced
    size_t lodDrawn_ = 0, lodFading_ = 0, lodSetDrawn_[kLodSets] = {0, 0, 0, 0};
    void evaluateLodObjects(simd_float3 eye, const ViewCull& cull);
    // P94: one LOD object's draw decision this frame (evaluateLodObjects and popProbe): the level mesh and the fade
    // alpha; false = not drawn.
    bool lodDecide(size_t set, size_t i, simd_float3 eye, const GpuMesh*& gm, float& alpha) const;
    std::vector<WorldGroup> propGroups_;
    std::vector<MeshInstance> propInputs_, rtcInputs_, heliInputs_;  // P47: the lists behind propInstances_ (PK07: heliInputs_)
    std::vector<MeshInstance> f14Inputs_;  // PK27
    size_t rebuildProps();
    std::vector<int> inputToInstance_;  // setScene input index -> instance, or -1
    struct OtherVehicle {
        std::vector<Instance> instances;
        std::vector<int> inputToInstance;  // setVehicles part index -> instance, or -1
        simd_float3 paint;                 // sRGB
        // P104: per instance: this frame's alpha (setVehicleLod; 0 = not drawn) and whether it is a rider mesh.
        std::vector<float> alpha, alphaLod, alphaLod1;  // alpha = the one drawn (alphaLod, or alphaLod1 with vehicleLodOff)
        std::vector<char> rider;
        VehicleLodState lod;
        bool drawn(size_t i) const { return i >= alpha.size() || alpha[i] > 0; }
        float fadeOf(size_t i) const { return i < alpha.size() && alpha[i] < 1 ? 1.0f - alpha[i] : 0.0f; }  // world col 0 w
    };
    std::vector<OtherVehicle> vehicles_;
    // ---- PK22: per vehicle (index vehicle + 1): V:DYN part factors by input part, lamp clone state, dirt
    struct VehicleVisual {
        std::vector<simd_float4x4> adjust;
        std::vector<char> adjusted;
        bool lampsSet = false, emissiveFound = false;
        std::array<float, 4> lampAlpha{};
        std::array<simd_float3, 4> lampEmissive{};
        float dirt = -1;
    };
    std::vector<VehicleVisual> visuals_;
    VehicleVisual& visual(int vehicle) {
        if (visuals_.size() < size_t(vehicle + 2)) visuals_.resize(size_t(vehicle + 2));
        return visuals_[size_t(vehicle + 1)];
    }
    const VehicleVisual* lampState(int vehicle) const {  // nullptr: the lamps as stored
        return !linksOff && size_t(vehicle + 1) < visuals_.size() && visuals_[size_t(vehicle + 1)].lampsSet ? &visuals_[size_t(vehicle + 1)] : nullptr;
    }
    void findLampEmissive(int vehicle);
    // ---- PK22 end
    uint32_t morphEpoch_ = 0;    // P85
    id<MTLBuffer> noMorph_ = nil;  // P85: bound at buffer 4 when a draw has no morph
    // P74: car occlusion omnis and the OMBREVOITURES spot map (sProjSpotMap0), loaded by setGround from the world.
    static constexpr size_t kMaxCarOcclusion = 16;  // FrameUniforms.carOcc holds 3 rows each
    std::vector<CarOcclusion> carOcclusion_;
    std::vector<VehicleOmni> vehicleLights_;  // P77
    mutable std::vector<simd_float4> omniRows_;  // PK28: this frame's gathered lights, 4 rows each (shader buffer 6)
    // ---- PK28: RotShape_Z records of this frame (0x835a00: one per drawn point)
    struct RotRecord {
        simd_float4 pivot;      // world pivot, w node scale (RSHAPE_SCALE)
        simd_float4 upLeft;     // RSHAPE_UPLEFT, w 1 = vh_front (code 0xc), 0 = vh_normal (0xb)
        simd_float4 downRight;  // RSHAPE_DWRIGHT, w the view alpha (GOURAUD_DFLT.w)
        simd_float4 extra;      // x MTL_ADDSOUS_NULL (0 for 0x1000 / 0x2000 materials: SetUnlit 0x84f060), yzw unused
        uint32_t material = 0;  // Material_Z ID (rotMats_)
        float dist = 0;         // the pivot's distance from the eye (blended sort)
        bool fading = false;    // view alpha < 1 (0x84ec60)
    };
    std::vector<rotshape::Placement> rotShapes_;
    std::function<bool(const std::string&)> rotSubActive_;
    std::vector<RotRecord> rotRecords_;
    std::unordered_map<uint32_t, GpuMaterial> rotMats_;
    std::unordered_map<uint32_t, id<MTLRenderPipelineState>> rotPipes_;
    size_t rotDrawn_ = 0;
    const GpuMaterial& rotMaterial(uint32_t id);
    id<MTLRenderPipelineState> rotPipeline(uint32_t kind);  // 0 opaque, 1 + BlendKind blended, 0xff depth only
    void gatherRotShapes(simd_float3 eye, const ViewCull& cull);
    void drawRotRecord(id<MTLRenderCommandEncoder> enc, const RotRecord& r, bool prepass);
    // ---- PK28 end
    mutable int omniGathered_ = 0;               // PK28: records gathered before the 64 cap (FUEL_OMNI_TRACE)
    const void* omniBytes(NSUInteger* length) const;  // PK28: omniRows_ for setBytes (one zero row when empty)
    id<MTLTexture> carSpot_ = nil;
    id<MTLTexture> sunTex_ = nil;  // P74: the sun / moon sprite (GenWorld material 0x36), fsSky
    void loadCarSpot(const DpcFile& world);
    void fillCarOcclusion(FrameUniforms& fu, simd_float3 eye) const;
    void fillVehicleOmnis(FrameUniforms& fu, simd_float3 eye) const;  // P77
    id<MTLBuffer> encodeOmniClusters(id<MTLCommandBuffer> cb, FrameUniforms& fu, float fovY, float aspect);  // P92 (kOmniClusters)
    id<MTLComputePipelineState> omniClusterPipeline_ = nil;
    std::map<std::pair<uint32_t, std::array<uint32_t, 3>>, std::unique_ptr<GpuMesh>> liveryMeshes_;
    // Offscreen targets (rebuilt when the output size changes) and the HDR post chain (post.md P1/P3).
    NSUInteger targetWidth_ = 0, targetHeight_ = 0;
    id<MTLTexture> hdrMsaa_, depthMsaa_, hdr_, depth_;
    id<MTLTexture> shrink_;                       // 128 x 128 FP16 scene copy
    std::array<id<MTLTexture>, 8> lum_;           // log luminance 128 .. 1 (R32F)
    std::array<id<MTLTexture>, 2> adapted_;       // exposure S, 1 x 1 R32F ping-pong
    id<MTLTexture> lumOne_;                       // P95: kLumReduce's 1 x 1 mean log luminance
    id<MTLRenderPipelineState> shrinkLumPipeline_;
    id<MTLComputePipelineState> lumReducePipeline_;
    int adaptedIndex_ = 0;
    simd_float4 grainNow_ = {0, 0, 1, 0};  // P102: this frame's film grain uniforms (encodePost -> encodeUpscale)
    id<MTLTexture> grainTex_ = nil;        // P102: the grain's 256 x 256 R8 noise tile
    uint32_t grainFrame_ = 0;              // P102: the grain's frame seed
    bool adaptInit_ = true;
    std::array<id<MTLTexture>, 4> bloomRt_;       // 128 x 128 8-bit: bloom, scratch, star A, star B
    id<MTLTexture> postFx_;                       // tone-mapped frame before the radial blur
    id<MTLRenderPipelineState> dustPipeline_, shrinkPipeline_, lumLogPipeline_, lumAvgPipeline_, adaptPipeline_, brightPipeline_,
        gaussPipeline_, toneMapPipeline_, toneMapFxPipeline_, radialPipeline_;
    // ---- P93: dynamic resolution. The targets above keep the output size; a scaled frame draws into their top-left
    // viewW_ x viewH_ (the passes' render area and viewport), the post chain reads only that part, and the tone-mapped
    // frame (lowOut_, display values) is upscaled into upOut_ by MetalFX, then copied into the output (present pass),
    // or upscaled straight into the output by the sharpened bilinear.
    NSUInteger viewW_ = 0, viewH_ = 0;            // this frame's render size (= the output size when native)
    bool scaledFrame_ = false;
    float lastScale_ = 1;
    id<MTLTexture> lowOut_ = nil, upOut_ = nil;   // RGBA8Unorm, output size
    id fxScaler_ = nil;                           // id<MTLFXSpatialScaler> (renderer.mm imports MetalFX)
    bool fxScalerTried_ = false;
    Upscaler builtUpscaler_ = Upscaler::Sharpen;  // P101: the upscaler the targets were made for (a menu change rebuilds them)
    id<MTLRenderPipelineState> radialLowPipeline_ = nil, presentPipeline_ = nil, sharpenPipeline_ = nil;
    DynamicResolution dynres_;
    std::mutex dynresMutex_;
    struct DynresSample { double ms; float scale; uint32_t epoch; };
    std::vector<DynresSample> dynresDone_;  // completed frames for the controller (epoch: cuts so far when encoded)
    uint32_t cutEpoch_ = 0;
    double dynresLastEnd_ = 0;                          // GPUEndTime of the last completed frame (completion handler)
    float chooseScale(NSUInteger width, NSUInteger height);
    // P93: camera cuts. The first frame after a cut costs up to twice a normal one on the GPU (P92: the TELEPORT_HELICO
    // cuts at 3.0 and 12.3 s, 22-28 ms; the tiler's first pass over a new view), and what the controller measured belongs
    // to the old view (the intro's helicopter shot costs 5 ms, the start line 15+). With dynamic resolution on, the cut's
    // frame renders at the lowest scale and is not reported; the controller restarts from the lowest scale and takes the
    // new view's scale from its first frames (DynamicResolution::cut; reports of frames encoded before the cut dropped).
    simd_float3 cutEye_ = {0, 0, 0}, cutFwd_ = {0, 0, 1};
    bool haveCutCam_ = false, cutFrame_ = false;
    bool cameraCut(const OrbitCamera& cam);
    void ensureUpscaler(NSUInteger width, NSUInteger height, bool allowTemporal = true);  // TAA: false = the display-value path
    void applyView(id<MTLRenderCommandEncoder> enc) const;  // the scaled viewport (no-op at native)
    void encodeUpscale(id<MTLCommandBuffer> cb, id<MTLTexture> target);
    // ---- TAA: MetalFX temporal upscaling (taa_render.mm). Pass order of a temporal frame: shadows, cube face, clusters
    // (unjittered), the scene (jittered; its last draw keeps each pixel's luminance in the HDR alpha, which no effect
    // writes), the effects, the velocity pass (camera motion from the resolved depth, then the moving objects; the
    // reactive mask from the effects' luminance change), the exposure and bloom from the render-size HDR, MetalFX
    // temporal (HDR, render size -> output size), and the tone map / radial blur / grain at the output size.
    bool temporalFrame_ = false;        // this frame goes through the temporal scaler
    bool taaReset_ = true;              // the next temporal frame drops the scaler's history
    uint32_t cutSerial_ = 0;            // PHYSIQUE: cutSerial()
    uint32_t taaIndex_ = 0;             // temporal frames so far (the jitter phase)
    simd_float2 taaJitterPx_ = {0, 0};  // this frame's jitter in render pixels (x right, y down)
    float texBiasBase_ = -1.0f;         // PK28's kTexBias (renderer +0x1104 + +0x1100; FUEL_LOD_BIAS)
    id fxTemporal_ = nil;               // id<MTLFXTemporalScaler>
    bool fxTemporalTried_ = false, fxTemporalFailed_ = false;
    NSUInteger temporalW_ = 0, temporalH_ = 0;
    float taaMinScale_ = 0.5f;          // the lowest render scale the scaler was made for
    MTLTextureUsage targetExtraUsage_ = 0;  // usage bits the scaler asks of hdr_ / depth_ (ensureTargets adds them)
    id<MTLTexture> vel_ = nil, react_ = nil, upHdr_ = nil, expo16_ = nil, taaMark_ = nil;
    id<MTLRenderPipelineState> taaLumPipeline_ = nil, taaCameraPipeline_ = nil, taaMeshPipeline_ = nil, taaGrassPipeline_ = nil,
                               taaWaterPipeline_ = nil, taaRotPipeline_ = nil, taaExposurePipeline_ = nil;
    struct TaaCam {
        simd_float4x4 vp = matrix_identity_float4x4;  // viewProj without the jitter (fuelClip form: applied to w - eye)
        simd_float3 eye = {0, 0, 0}, right = {1, 0, 0}, R = {1, 0, 0}, U = {0, 1, 0}, F = {0, 0, 1};
        float zn = 0.25f, q = -1.0f;
        float grassT = 0, grassAmp = 0, waterT = 0;
        bool valid = false;
    };
    TaaCam taaCur_, taaPrev_;
    simd_float4x4 taaJitteredVp_ = matrix_identity_float4x4;
    float grassClock_ = 0, grassAmp_ = 0, waterClock_ = 0;  // the clocks encode() gave the grass / water this frame
    std::vector<std::pair<uint32_t, uint32_t>> taaGrassDraws_;  // this frame's grass draws (first tuft, count)
    bool taaWaterDrawn_ = false;
    std::vector<char> taaVehicleOff_;   // encode()'s off-screen opponents of this frame
    std::vector<Instance> taaPrevPlayer_, taaPrevProps_;  // the last frame's instances (index, mesh and nearby origin match)
    std::vector<std::vector<Instance>> taaPrevVehicles_;
    std::vector<simd_float4x4> taaPrevLod3_;
    std::unordered_map<const GpuMesh*, id<MTLBuffer>> taaPrevBaked_;  // skinned meshes: the vertex buffer the last frame drew
    std::shared_ptr<FrameUniforms> stableFrame_;  // lastFrame_ without the jitter: GPS arrows / nav beams (screen-stable)
    const FrameUniforms* stableFrame() const;
    void taaFrameUniforms(FrameUniforms& fu, float scale);  // the jitter, the LOD bias and the unjittered camera
    void makeTaaPipelines();
    bool ensureTemporal(NSUInteger width, NSUInteger height);
    void encodeTaaLumCapture(id<MTLRenderCommandEncoder> enc);
    void encodeTaaVelocity(id<MTLCommandBuffer> cb);
    void encodeTemporal(id<MTLCommandBuffer> cb, id<MTLTexture> S);
    void taaEndFrame();
    // ---- TAA end
    SfxFrame dust_;
    id<MTLTexture> sfxAtlas_ = nil;
    std::unique_ptr<DpcFile> sfxShared_;
    std::shared_ptr<const WeatherFields> weatherFields_;
    WeatherAmounts weatherNow_;
    // ---- PK04
    WeatherOverrides weatherOverrides_;  // WeatherManager_G +0xe0..+0x11b
    RaceWeather raceWeather_;            // race +0x2d8..+0x33c
    bool raceWeatherOn_ = false;         // 0x632250 runs (a race)
    std::array<float, 3> raceStart_{}, raceFinish_{};
    std::function<std::array<double, 3>()> racePlayer_;
    bool cloudRepopulate_ = false;       // 0x5b7bb0: the cloud tank's vf+0x10 at the next cloud pass
    double dayClockBase_ = 0, dayClockAt_ = 0;  // setDayClock: clock = base + (elapsed - at)
    std::string gameRoot_;               // the game folder (hubinfos.tsc), from setGround's world path
    double elapsedClock() const;         // gameClock, or the wall time since the first frame
    void stepRaceWeather();
    // ---- end PK04
    void loadSfxAtlas(const DpcFile& world);
    std::shared_ptr<FrameUniforms> lastFrame_;  // the uniforms of the last encode, for dust and the post pass
    std::atomic<double> gpuMs_{0};
    CpuTimes cpu_;  // P92
    // P92: textures filled by the CPU (replaceRegion, managed storage) to be re-laid out for the GPU
    // (optimizeContentsForGPUAccess) by the next frame's command buffer: same texels, faster sampling.
    std::vector<id<MTLTexture>> toOptimize_;
    const bool fixedClock_ = std::getenv("FUEL_FIXED_CLOCK") != nullptr;  // P92: repeatable shots (animSeconds)
    float animSeconds() const;
    // ---- PK17: the pause backdrop state (setPauseBackdrop)
    bool pauseSky_ = false;            // 0x46abd0 .. 0x46ade0
    double pauseSavedClock_ = 0;       // InGameMenuHUD +0x70 / +0x74 (the day clock, seconds)
    double pausedAt_ = -1;             // rawElapsed() when the world stopped, -1 = running
    double pausedTotal_ = 0;           // the time the world spent paused (subtracted from the clocks)
    float animPausedAt_ = -1, animPausedTotal_ = 0;  // the same for animSeconds()
    Weather pauseHook_{};              // the weather hook's output of the last running frame (0x6409d0 is not re-run)
    bool havePauseHook_ = false;
    double rawElapsed() const;
    // ---- end PK17
    struct OneOff { int pipelines = 0, textures = 0, meshes = 0, chunks = 0; } oneOff_, oneOffSeen_;  // P92: running counts
    uint32_t frame_ = 0;
    simd_float3 center_ = {0, 0, 0};
    simd_float3 sceneMin_ = {0, 0, 0}, sceneMax_ = {0, 0, 0};
    float radius_ = 1.0f;
    uint32_t triangles_ = 0;
};

}  // namespace fuel
