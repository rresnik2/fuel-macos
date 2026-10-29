// Metal renderer for one FUEL mesh at a time: uploads a Mesh_Z with its
// materials and textures and draws it lit by a single sun.
#pragma once

#import <Metal/Metal.h>
#include <simd/simd.h>

#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/vehicle.hpp"

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
    simd_float3 color = {60.0f / 255, 60.0f / 255, 60.0f / 255};
    std::array<uint32_t, 3> livery{};
};

// One placed copy of a mesh.
struct MeshInstance {
    uint32_t meshId = 0;
    simd_float4x4 world = matrix_identity_float4x4;
};

class MeshRenderer {
public:
    MeshRenderer(id<MTLDevice> device, MTLPixelFormat colorFormat, MTLPixelFormat depthFormat, NSUInteger sampleCount,
                 const ResourceLibrary& library);

    // Shows one Mesh_Z at the origin; returns false with a reason on failure.
    bool setMesh(uint32_t meshId, std::string* error);
    // Shows several placed meshes. Meshes that fail to load are skipped and
    // reported in `error`; returns false only when nothing could be shown.
    bool setScene(const std::vector<MeshInstance>& instances, std::string* error);
    uint32_t triangleCount() const { return triangles_; }

    // Takes effect for meshes loaded afterwards (the colour applies at once).
    void setPaint(const Paint& paint);

    bool cullBackFaces = true;
    bool wireframe = false;

    void encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam);

private:
    enum class AlphaMode : uint32_t { Opaque = 0, Test = 1, Blend = 2 };

    struct GpuMaterial {
        id<MTLTexture> texture;
        simd_float4 diffuse = {0.7f, 0.7f, 0.7f, 1.0f};
        AlphaMode alpha = AlphaMode::Opaque;
        bool lit = true;
        bool skip = false;
        uint32_t samplerIndex = 0;  // bit 0 clamp U, bit 1 clamp V
        uint32_t code = 0;          // MatCodeId (flags & 0x1f)
        bool paintOnly = false;     // livery material without a livery texture
    };
    struct Draw {
        uint32_t vb = 0;
        uint32_t firstIndex = 0;
        uint32_t indexCount = 0;
        uint32_t material = 0;
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
    id<MTLTexture> texture(uint32_t bitmapId, uint8_t* alphaKind);
    id<MTLRenderPipelineState> pipeline(uint32_t stride, AlphaMode mode);
    GpuMaterial makeMaterial(uint32_t materialId);

    id<MTLDevice> device_;
    MTLPixelFormat colorFormat_, depthFormat_;
    NSUInteger sampleCount_;
    const ResourceLibrary& library_;
    id<MTLLibrary> shaders_;
    id<MTLDepthStencilState> depthWrite_, depthNoWrite_;
    id<MTLSamplerState> samplers_[4];
    id<MTLTexture> white_;
    std::unordered_map<uint64_t, id<MTLRenderPipelineState>> pipelines_;
    std::unordered_map<uint32_t, id<MTLTexture>> textures_;
    std::unordered_map<uint32_t, uint8_t> textureAlpha_;
    std::unordered_map<uint32_t, std::unique_ptr<GpuMesh>> meshes_;

    Paint paint_;
    std::vector<Instance> instances_;
    simd_float3 center_ = {0, 0, 0};
    float radius_ = 1.0f;
    uint32_t triangles_ = 0;
};

}  // namespace fuel
