// Decoders for the render resources stored in DPC files: Bitmap_Z (textures),
// Mesh_Z (GPU vertex/index buffers and draw batches) and Material_Z.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

// Resource IDs and class IDs are this CRC of a string (class name or asset
// path): an MSB-first CRC-32 table (poly 0x04C11DB7) driven with a reflected
// shift, over bytes with A-Z folded to lower case, init 0, no final xor.
uint32_t asoboCrc(const std::string& s);

namespace cls {
inline constexpr uint32_t Bitmap = 0x57B1F99E;
inline constexpr uint32_t Mesh = 0x52B12EB5;
inline constexpr uint32_t MeshData = 0xD8220A84;
inline constexpr uint32_t Material = 0x8362982B;
}  // namespace cls

// ---- Bitmap_Z ---------------------------------------------------------------

enum class PixelFormat { Unknown, A1R5G5B5, R5G6B5, A4R4G4B4, A8R8G8B8, R8G8B8, DXT1, DXT3, DXT5, R32F, RGBA16F, RGBA32F, L8 };

const char* pixelFormatName(PixelFormat f);

struct BitmapInfo {
    enum Type : uint16_t { Tex2D = 0, Cube = 2 };
    uint16_t type = 0;
    uint32_t width = 0, height = 0;
    uint32_t size = 0;  // body size, 0 if not recorded
    uint32_t mips = 1;
    uint8_t formatCode = 0;
    PixelFormat format = PixelFormat::Unknown;
    uint8_t alphaKind = 0;  // 0 opaque, 1 alpha test, 2 alpha blend

    bool isBlockCompressed() const;
    uint32_t faces() const { return type == Cube ? 6 : 1; }
    // Byte size of each mip level of one face.
    std::vector<uint32_t> mipSizes() const;
    uint32_t expectedBodySize() const;
};

// The 32-byte Bitmap_Z link header holds the whole texture description; the
// body is the raw, linear (unswizzled) mip chain, face by face for cubes.
BitmapInfo parseBitmapLink(const std::vector<uint8_t>& link);  // throws
// Empty when the bitmap is consistent with a body of `bodySize` bytes.
std::vector<std::string> checkBitmap(const BitmapInfo& bm, uint32_t bodySize);

// ---- Material_Z -------------------------------------------------------------

struct Material {
    std::array<float, 3> diffuse{};
    float alpha = 1.0f;
    std::array<float, 3> emissive{};
    std::array<float, 4> specular{};  // rgb, power
    std::array<float, 2> fresnel{};   // reflectance at normal and grazing incidence (0 = none)
    uint16_t textureMask = 0;  // bit i = texture slot i is used
    uint32_t flags = 0;
    // Slot 0 diffuse, 1 envmap, 2 occlusion, 3 normal, 4 specular,
    // 5 normal-local, 6 dirt, 7 add-normal-local, 8 unused.
    std::array<uint32_t, 9> textures{};

    bool skipDraw() const { return flags & 0x80; }
    bool clampU() const { return flags & 0x100; }
    bool clampV() const { return flags & 0x200; }
};

Material parseMaterialBody(const std::vector<uint8_t>& body);  // throws

// ---- Mesh_Z -----------------------------------------------------------------

struct MeshLink {
    uint32_t dataId = 0;  // MeshData_Z
    std::array<float, 4> sphere{};  // centre xyz, radius
    uint32_t flags = 0;
};

struct VertexBuffer {
    uint32_t count = 0;
    uint32_t stride = 0;
    uint32_t flags = 0;
    std::vector<uint8_t> data;
};

struct IndexBuffer {
    uint32_t count = 0;
    uint32_t flags = 0;
    std::vector<uint16_t> data;
};

// One Direct3D 9 DrawIndexedPrimitive call.
struct Batch {
    uint32_t vb = 0, ib = 0;
    uint16_t primitiveType = 0;  // 4 = triangle list, 5 = triangle strip
    uint16_t vertexType = 0;     // 0 static, 2 one-bone skin, 4 four-bone skin, 7 water, 8 morph
    uint16_t minVertex = 0;
    uint32_t numVertices = 0;
    uint32_t startIndex = 0;
    uint32_t primitiveCount = 0;
    uint32_t baseVertex = 0;
    uint32_t startVertex = 0;  // non-indexed draws
    uint16_t stride = 0;
    uint16_t material = 0;  // index into Mesh::materials, 0xFFFF = none
};

struct Mesh {
    std::vector<uint32_t> materials;  // Material_Z IDs
    std::vector<VertexBuffer> vbs;
    std::vector<IndexBuffer> ibs;
    std::vector<Batch> batches;
    std::vector<std::string> morphTargets;
};

MeshLink parseMeshLink(const std::vector<uint8_t>& link);  // throws
Mesh parseMeshBody(const std::vector<uint8_t>& body, uint32_t linkFlags);  // throws

// Vertex indices (into the batch's VB) of each triangle; throws on
// out-of-range data.
std::vector<uint32_t> batchTriangles(const Mesh& m, const Batch& b);

struct MeshStats {
    uint32_t triangles = 0;
    std::array<float, 3> boundsMin{}, boundsMax{};
};
// Empty problem list when every batch draws inside its buffers.
std::vector<std::string> checkMesh(const Mesh& m, MeshStats* stats = nullptr);

}  // namespace fuel
