// Metal renderer for one FUEL mesh at a time: uploads a Mesh_Z with its
// materials and textures and draws it lit by a single sun.
#pragma once

#import <Metal/Metal.h>
#include <simd/simd.h>

#include "core/dpc.hpp"
#include "core/resources.hpp"

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

class MeshRenderer {
public:
    MeshRenderer(id<MTLDevice> device, MTLPixelFormat colorFormat, MTLPixelFormat depthFormat, NSUInteger sampleCount,
                 const ResourceLibrary& library);

    // Uploads one Mesh_Z; returns false with a reason on failure.
    bool setMesh(uint32_t meshId, std::string* error);
    uint32_t triangleCount() const { return triangles_; }

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
    };
    struct Draw {
        uint32_t vb = 0;
        uint32_t firstIndex = 0;
        uint32_t indexCount = 0;
        uint32_t material = 0;
    };

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

    std::vector<id<MTLBuffer>> vbs_;
    std::vector<uint32_t> vbStrides_;
    id<MTLBuffer> indices_;
    std::vector<GpuMaterial> materials_;
    std::vector<Draw> draws_[3];  // by AlphaMode
    simd_float3 center_ = {0, 0, 0};
    float radius_ = 1.0f;
    uint32_t triangles_ = 0;
};

}  // namespace fuel
