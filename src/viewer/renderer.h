// Metal renderer for FUEL meshes and assembled vehicles.
//
// A modern forward renderer driven by the game's material data: physically
// based shading of the diffuse, normal, specular, occlusion and dirt texture
// slots, car paint and liveries, a sun with a shadow map, sky lighting and
// reflections, a ground plane, and HDR tone mapping into an sRGB target.
#pragma once

#import <Metal/Metal.h>
#include <simd/simd.h>

#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/ground.hpp"
#include "core/rider.hpp"
#include "core/terrain.hpp"
#include "core/vehicle.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
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

// Render target formats the renderer's pipelines are built for.
inline constexpr MTLPixelFormat kRenderColorFormat = MTLPixelFormatBGRA8Unorm_sRGB;
inline constexpr MTLPixelFormat kRenderDepthFormat = MTLPixelFormatDepth32Float;

class MeshRenderer {
public:
    MeshRenderer(id<MTLDevice> device, NSUInteger sampleCount, const ResourceLibrary& library);

    // Registers CPU-built geometry (the baked rider) under `key`, which then works
    // as a meshId in setScene. Materials are drawn like a Mesh_Z's, with their
    // livery bitmaps substituted. Replaces an earlier mesh with the same key;
    // pick keys that are not Mesh_Z IDs. Returns false (and says why) on bad data.
    bool addBakedMesh(uint32_t key, const BakedMesh& mesh, std::string* error);

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
    // Road ribbons draped over the terrain: vertices (xyz, w = opacity; uv
    // across/along) and batches of triangles sharing a Bitmap_Z (0 = dirt colour).
    struct RoadVertex {
        simd_float4 position;
        simd_float4 uv;
    };
    struct RoadBatch {
        uint32_t texture, firstIndex, indexCount;
        bool dirt;  // tint the texture with the dirt colour
    };
    void setRoads(const std::vector<RoadVertex>& vertices, const std::vector<uint32_t>& indices, const std::vector<RoadBatch>& batches);
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

    // Call before the main pass each frame: renders the sun's shadow map.
    void encodeShadows(id<MTLCommandBuffer> cb);
    // Draws the scene into a pass with kRenderColorFormat/kRenderDepthFormat.
    void encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam);

private:
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
    id<MTLRenderPipelineState> roadPipeline_;
    id<MTLBuffer> roadVertices_, roadIndices_;
    std::vector<std::pair<id<MTLTexture>, RoadBatch>> roadBatches_;
    id<MTLDepthStencilState> depthAlways_;
    id<MTLTexture> heightTex_, groundColour_, groundSurface_, groundDetail_;
    id<MTLBuffer> terrainIndices_;
    uint32_t terrainIndexCount_ = 0;
    Heightfield terrainData_;
    simd_float4 terrainInfo_ = {0, 0, 1, 0};  // origin x, origin z, spacing, metres for a texel value of 1.0
    std::unordered_map<uint64_t, id<MTLRenderPipelineState>> pipelines_;
    std::unordered_map<uint32_t, id<MTLRenderPipelineState>> shadowPipelines_;
    std::unordered_map<uint64_t, id<MTLTexture>> textures_;  // key: id << 1 | srgb
    std::unordered_map<uint32_t, uint8_t> textureAlpha_;
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> meshes_;
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
    simd_float3 center_ = {0, 0, 0};
    simd_float3 sceneMin_ = {0, 0, 0}, sceneMax_ = {0, 0, 0};
    float radius_ = 1.0f;
    uint32_t triangles_ = 0;
};

}  // namespace fuel
