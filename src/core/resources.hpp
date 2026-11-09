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
    // ---- PK28: obj +0x24 (link bytes 22..25, float): the bitmap's own mip LOD bias, added by 0x84ca80 to the renderer's
    // global bias (+0x1104 -1.0, +0x1100 0.0) as D3DSAMP_MIPMAPLODBIAS. 0 in 8,525 shipped bitmaps, -1 in 349, -2 in 18.
    float lodBias = 0;

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
    // UV transform (Material_Z::UpdateUVMatrix 0x695ee0; object = body + 0xc). The body stores the
    // precomputed 3 x 2 matrix uv' = u * row0 + v * row1 + row2 (rows at body 0x20 / 0x30 / 0x40, two
    // floats each; the cells between them hold other fields), and its inputs: angle (body 0x4c), offset
    // (0x70, 0x74), scale (0x78, 0x7c). Rotation and scale are about the texture centre (0.5, 0.5).
    std::array<float, 6> uvMatrix{1, 0, 0, 1, 0, 0};
    float uvAngle = 0;
    std::array<float, 2> uvOffset{}, uvScale{1, 1};
    std::array<float, 4> params{};  // body 0x60 (MtlConstant Params, gMaterial c3)
    int32_t field28 = -1, field2c = 7;  // body 0x28 / 0x2c, ints (ctor 0x695c50); 0x2c has a MaterialAnim step track
    bool identityUv() const {
        return uvMatrix[0] == 1 && uvMatrix[1] == 0 && uvMatrix[2] == 0 && uvMatrix[3] == 1 && uvMatrix[4] == 0 && uvMatrix[5] == 0;
    }
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
    // P85: the morph tables of a vehicle mesh (Mesh_Z +0xfc..+0x118, read by the visual damage 0x7cb550):
    // a point tree (0x20-byte nodes: min xyz, u16 children (1-based) or 0, max xyz, u16 first pair, u16 count),
    // (u16 point, u16 group offset) pairs (points = collisionVerts, int16 xyz / 1024), the groups (u16 delta,
    // u16 n, n vertex indices) and per target the vertex count of its stride-12 VB and the deltas (int16 x4, xyz / 4096).
    struct MorphTarget {
        uint32_t flags = 0;    // the VB flags (+0x10)
        uint16_t vb = 0;       // its stride-12 VB (+0x14)
        std::vector<uint16_t> vertexDelta;  // per vertex: index into deltas (+0x18 count = the VB's vertex count)
        std::vector<int16_t> deltas;        // 4 per delta (+0x8 / +0xc)
    };
    std::vector<MorphTarget> morphs;
    std::vector<uint8_t> morphNodes;
    std::vector<uint16_t> morphPairs;
    std::vector<uint16_t> morphGroups;
    // Static collision of the mesh (P46, read by Mesh_Z vtable +0x88 0x80b1d0 / 0x817cf0): BVH nodes (0x20 bytes:
    // min xyz, u16 children (1-based) or 0, max xyz, u16 first triangle, u16 count), triangles (u16 a, b, c,
    // material index) and vertices (int16 xyz, metres x 1024).
    std::vector<uint8_t> collisionNodes;
    std::vector<uint16_t> collisionTris;
    std::vector<int16_t> collisionVerts;
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
