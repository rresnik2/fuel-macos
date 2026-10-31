// Metal renderer for FUEL meshes and assembled vehicles.
//
// A modern forward renderer driven by the game's material data: physically
// based shading of the diffuse, normal, specular, occlusion and dirt texture
// slots, car paint and liveries, a sun with a shadow map, sky lighting and
// reflections, a ground plane and soft dust particles, drawn in linear HDR and
// finished by a post pass (bloom, sun glare and shafts, speed blur, filmic tone
// mapping and a warm grade, vignette) into an sRGB target.
#pragma once

#import <Metal/Metal.h>
#include <simd/simd.h>

#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/ground.hpp"
#include "core/ground_point.hpp"
#include "core/rider.hpp"
#include "core/terrain.hpp"
#include "core/terrain_mesh.hpp"
#include "core/vehicle.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuel {

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
    simd_float3 color = {60.0f / 255, 60.0f / 255, 60.0f / 255};  // sRGB
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

// One dust particle, a soft sprite facing the camera.
struct DustParticle {
    simd_float4 position;  // xyz, w radius in metres
    simd_float4 colour;    // linear albedo, a opacity
    simd_float4 extra;     // x rotation (radians), y noise seed 0..1
};

struct FrameUniforms;  // renderer.mm

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
    // A small second list for props the physics moves; cheap to call every frame.
    size_t setPropInstances(const std::vector<MeshInstance>& instances);
    // Race beacons: xyz = ground position, w = kind (0 next checkpoint,
    // 1 the one after, 2 finish). Drawn as tall glowing columns.
    void setMarkers(const std::vector<simd_float4>& markers) { markers_ = markers; }
    // Points the camera at `centre` instead of the scene's centre.
    void setFocus(simd_float3 centre) { center_ = centre; }
    // Other vehicles (race opponents), drawn with the scene: each its own parts (vehicle
    // space, like setScene) and paint; their meshes load with that vehicle's livery.
    // Replaces the earlier list. Vehicles far from the focus are not drawn.
    void setVehicles(const std::vector<std::vector<MeshInstance>>& parts, const std::vector<Paint>& paints);
    // Moves vehicle `k`'s parts (same order as its setVehicles input).
    void setVehicleWorlds(size_t k, const std::vector<simd_float4x4>& worlds);
    // Half-size in metres of the area around the focus that gets sun shadows
    // (0: the scene's radius). Larger shows the shadows of vehicles nearby.
    float shadowRadius = 0;

    // Takes effect for meshes loaded afterwards (the colour applies at once).
    void setPaint(const Paint& paint);

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

    // Dust drawn with the next frames (replaces the earlier list).
    void setDust(const std::vector<DustParticle>& particles) { dust_ = particles; }
    // Speed effects for this frame, 0..1: radial blur towards the screen edges.
    float speedBlur = 0;
    bool postEffects = true;  // false: plain tone mapping (no bloom, glare, blur, grade)
    bool grass = true;        // false: no grass tufts (settings file; FUEL_NO_GRASS=1 also disables)
    float sceneRadius() const { return radius_; }
    // GPU time of the last completed frame, in milliseconds (0 until one finished).
    double lastGpuMs() const { return gpuMs_; }

    // ---- Grass tufts (p14): FUEL's 1 m-grid grass cards (core/ground_point.hpp).
    void setGrassAtlas(uint32_t bitmapId);                 // the Grass material's 4x4 atlas
    // Tree and bush meshes: their blended materials are drawn as cut-outs (alpha to coverage).
    void setFoliage(std::unordered_set<uint32_t> meshIds) { foliage_ = std::move(meshIds); }
    void setGrass(const std::vector<GrassTuft>& tufts);    // replaces the list
    // Fine ground colour around the player (levels 0, 1, 2: 1, 2, 4 m texels), kGroundTile
    // texels per side, sRGB RGBA8 like the colour map (alpha = far-band forest weight); the
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
    void encodeShadows(id<MTLCommandBuffer> cb);
    // Draws the scene into a pass with kRenderColorFormat/kRenderDepthFormat.
    void encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam);
    void ensureTargets(NSUInteger width, NSUInteger height);
    void encodeDust(id<MTLCommandBuffer> cb);
    void encodePost(id<MTLCommandBuffer> cb, id<MTLTexture> target);
    // ---- Grass tufts (p14)
    void createGrassPipeline();
    void encodeGrass(id<MTLRenderCommandEncoder> enc);
    id<MTLRenderPipelineState> grassPipeline_;
    id<MTLBuffer> grassBuffer_;
    id<MTLTexture> grassAtlas_;
    size_t grassCount_ = 0;
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
    };
    struct Draw {
        uint32_t vb = 0;
        uint32_t firstIndex = 0;
        uint32_t indexCount = 0;
        uint32_t material = 0;
        bool sprite = false;  // camera-facing sprite batch
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
    };
    struct Instance {
        const GpuMesh* mesh = nullptr;
        simd_float4x4 world = matrix_identity_float4x4;
    };

    const GpuMesh* loadMesh(uint32_t meshId, std::string* error);
    // The same with another livery than the current paint's (cached separately).
    const GpuMesh* loadMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>& livery);
    std::unique_ptr<GpuMesh> buildMesh(uint32_t meshId, std::string* error);
    float shadowHalfSize() const { return std::max(radius_ * 1.05f, shadowRadius); }
    id<MTLTexture> texture(uint32_t bitmapId, bool srgb, uint8_t* alphaKind);
    id<MTLTexture> uploadBitmap(const DpcFile& file, const DpcFile::Entry& entry, bool srgb, uint8_t* alphaKind);
    struct TextureStats {
        simd_float3 mean = {0.5f, 0.5f, 0.5f};  // of the DXT colour endpoints, 0..1
        bool greyDetail = false;  // pure grey around 0.5: takes the ground colour
    };
    const TextureStats& textureStats(uint32_t bitmapId);
    std::unordered_map<uint32_t, TextureStats> textureStats_;
    id<MTLRenderPipelineState> pipeline(uint32_t stride, AlphaMode mode);
    id<MTLRenderPipelineState> shadowPipeline(uint32_t stride);
    GpuMaterial makeMaterial(uint32_t materialId, uint32_t diffuseOverride = 0, uint32_t specularOverride = 0);
    simd_float4x4 sunViewProj() const;

    id<MTLDevice> device_;
    NSUInteger sampleCount_;
    const ResourceLibrary& library_;
    id<MTLLibrary> shaders_;
    id<MTLDepthStencilState> depthWrite_, depthNoWrite_;
    id<MTLSamplerState> samplers_[4];
    id<MTLTexture> white_, shadowMap_;
    id<MTLRenderPipelineState> groundPipeline_, terrainPipeline_, skyPipeline_, markerPipeline_;
    std::vector<simd_float4> markers_;
    id<MTLDepthStencilState> depthAlways_;
    id<MTLTexture> heightTex_, groundColour_, groundSurface_, groundDetail_;
    id<MTLTexture> groundFine_;  // 3-slice arrays (setGroundTile): colour, surface types, road decal
    id<MTLTexture> groundTypes_, groundDecal_;
    id<MTLRenderPipelineState> waterPipeline_;
    id<MTLBuffer> waterVertices_, waterIndices_;
    uint32_t waterIndexCount_ = 0;
    std::array<simd_float4, 8> waterConst_{};
    bool haveWaterLook_ = false;
    id<MTLTexture> waterFoam_, waterNormal_;
    id<MTLTexture> decalTex_[kDecalSlots] = {};
    id<MTLCommandQueue> blitQueue_;  // mipmaps for the fine tiles
    simd_float4 groundFineRect_[kGroundTileLevels] = {};  // origin x, z, extent, 1 when loaded
    simd_float4 groundFar_ = {0, 0, 0, 0};  // p16: far-band forest colour, w = decoded colour map
    id<MTLBuffer> terrainIndices_;
    uint32_t terrainIndexCount_ = 0;
    Heightfield terrainData_;
    // FUEL's ground mesh (terrain_mesh.hpp): quadtree chunks built on worker threads.
    std::unique_ptr<TerrainLod> terrainLod_;
    struct ChunkGpu {
        id<MTLBuffer> vb;
        uint64_t lastFrame = 0;
    };
    std::unordered_map<uint64_t, ChunkGpu> chunkGpu_;
    uint64_t chunkFrame_ = 0;
    simd_float3 lastTerrainEye_ = {1e9f, 0, 1e9f};
    simd_float4 terrainInfo_ = {0, 0, 1, 0};  // origin x, origin z, spacing, metres for a texel value of 1.0
    std::unordered_map<uint64_t, id<MTLRenderPipelineState>> pipelines_;
    std::unordered_map<uint32_t, id<MTLRenderPipelineState>> shadowPipelines_;
    std::unordered_map<uint64_t, id<MTLTexture>> textures_;  // key: id << 1 | srgb
    std::unordered_map<uint32_t, uint8_t> textureAlpha_;
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> meshes_;
    std::unordered_set<uint32_t> foliage_;
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> bakedMeshes_;  // addBakedMesh; survive setPaint

    Paint paint_;
    simd_float3 sunDir_;
    std::vector<Instance> instances_;
    std::vector<Instance> worldInstances_;  // flat list (shadow casters)
    struct WorldGroup {
        const GpuMesh* mesh;
        id<MTLBuffer> matrices;
        uint32_t count;
    };
    std::vector<WorldGroup> worldGroups_;   // instanced draws
    std::vector<Instance> propInstances_;   // movable props (shadow casters)
    std::vector<WorldGroup> propGroups_;
    std::vector<int> inputToInstance_;  // setScene input index -> instance, or -1
    struct OtherVehicle {
        std::vector<Instance> instances;
        std::vector<int> inputToInstance;  // setVehicles part index -> instance, or -1
        simd_float3 paint;                 // sRGB
    };
    std::vector<OtherVehicle> vehicles_;
    std::map<std::pair<uint32_t, std::array<uint32_t, 3>>, std::unique_ptr<GpuMesh>> liveryMeshes_;
    // Offscreen targets (rebuilt when the output size changes) and post pipelines.
    static constexpr int kBloomLevels = 5;  // 1/2 .. 1/32 of the output
    NSUInteger targetWidth_ = 0, targetHeight_ = 0;
    id<MTLTexture> hdrMsaa_, depthMsaa_, hdr_, depth_, shafts_;
    std::array<id<MTLTexture>, kBloomLevels> bloom_;
    id<MTLRenderPipelineState> dustPipeline_, brightPipeline_, downPipeline_, upPipeline_, shaftsPipeline_, compositePipeline_;
    std::vector<DustParticle> dust_;
    std::shared_ptr<FrameUniforms> lastFrame_;  // the uniforms of the last encode, for dust and the post pass
    std::atomic<double> gpuMs_{0};
    uint32_t frame_ = 0;
    simd_float3 center_ = {0, 0, 0};
    simd_float3 sceneMin_ = {0, 0, 0}, sceneMax_ = {0, 0, 0};
    float radius_ = 1.0f;
    uint32_t triangles_ = 0;
};

}  // namespace fuel
