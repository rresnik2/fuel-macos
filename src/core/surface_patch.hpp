// Surface_Z / SurfacePc_Z: FUEL's bicubic Bezier patch surfaces (P73, notes p73-mesh-material.md). Twelve
// world placements (USA1 2, sub-worlds 10: CR19, CR40, the Drown City / Mountain 2 / Tanker hubs, the CR1 dressing)
// and the menu scene. Decoded from FUEL.exe:
//   load        Surface_Z::vf03 0x80f220 (SurfacePc_Z::vf03 0x848710 adds the per-patch cache slots)
//   resolve     vf04 0x80f610 / vf05 0x80f6d0: patch +0xac MaterialAnim_Z ID -> patch +0xc = that anim's material
//   net         0x80ce20: the 16 control points of a patch from its four edges (inner points by the parallelogram rule)
//   draw        SurfacePc_Z::vf16 0x872df0 -> 0x846f00 -> 0x846f80: (N+1)^2 vertices by forward differences, N = 8
//               when the material's draw code (flags & 0x1f) is 7, else 3; normal, uv0, uv1 and colour bilinear
//               over the patch corners
//   collision   Surface_Z::vf34 0x812120 (the triangle slot +0x88) -> 0x811bc0 -> 0x7cf0a0: a (nu+1) x (nv+1) grid,
//               nu = ((flags >> 7) & 3) + 1, nv = ((flags >> 9) & 3) + 1, two triangles per cell, kept when the
//               patch material's collision flags (Material +0x38) pass the caller's masks
//               (verified against EmuRun of 0x846f80 / 0x7cf0a0: fuelsurface --vectors). FUEL's ray / sphere / sweep
//               queries (vf29/30/31) use a fixed 3 x 3 grid instead (ColSurfaceCache 0x7cffa0); the port gives Jolt
//               only the vf34 triangles. In the shipped data every world surface but one (buried) is invisible
//               (material flag 0x80): collision-only.
// No game data here: bodies are parsed from the player's DPC files at run time.
#pragma once

#include "dpc.hpp"
#include "rider.hpp"  // BakedMesh

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t Surface = 0x65B38A8D;       // Surface_Z
inline constexpr uint32_t SurfaceDatas = 0xDF6328C1;  // SurfaceDatas_Z (body: u32 1; nothing the patches use)
}  // namespace cls

// One 0xb0-byte patch record (Surface +0x90).
struct SurfacePatch {
    uint16_t flags = 0;            // +0x00: bits 1..4 reverse edge 0..3; bits 7..8 / 9..10 collision grid nu-1 / nv-1
    std::array<uint16_t, 4> edges{};  // +0x04: edge indices (Surface +0x98), going round the patch
    std::array<float, 4> sphere{};    // +0x10: bounding sphere (centre, radius), object space
    std::array<float, 12> frame{};    // +0x40: 3 x 4 box frame (axis i in [4i..4i+2], centre in column 3)
    std::array<float, 4> half{};      // +0x70: box half extents, radius
    std::array<uint16_t, 4> corner16{};  // +0x80: per-corner float4 records (Surface +0x78)
    std::array<uint16_t, 4> colour{}, normal{}, uv0{}, uv1{};  // +0x88 / +0x90 / +0x98 / +0xa0: corner indices
    uint32_t materialAnim = 0;     // +0xac: MaterialAnim_Z ID (its material is what the patch draws with)

    int collisionNu() const { return ((flags >> 7) & 3) + 1; }
    int collisionNv() const { return ((flags >> 9) & 3) + 1; }
};

struct SurfaceBody {
    std::vector<std::array<float, 3>> points;   // Surface +0x70: control points
    std::vector<std::array<float, 4>> corner16; // +0x78
    std::vector<SurfacePatch> patches;          // +0x90
    std::vector<std::array<uint16_t, 4>> edges; // +0x98: corner a, corner b, handle near a, handle near b
    std::vector<std::array<float, 3>> normals;  // +0xa0
    std::vector<std::array<float, 3>> colours;  // +0xa8 (rgb)
    std::vector<std::array<float, 2>> uvs;      // +0xb0
    std::vector<uint16_t> list16;               // +0xb8
    bool hasGrid = false;                       // a ColSurfaceCache grid follows (0x818350 / 0x81a1e0)
};
SurfaceBody parseSurfaceBody(const std::vector<uint8_t>& body);  // throws on malformed data

// 0x80ce20: the 4 x 4 control net of patch `p`, row-major (r * 4 + c), x y z w (w = 1). Row 0 runs along edge 0,
// column 3 along edge 1, row 3 backwards along edge 2, column 0 backwards along edge 3; inner points
// P11 = P01 + P10 - P00 and so on.
std::array<float, 64> patchControlNet(const SurfaceBody& s, const SurfacePatch& p);  // throws on bad indices

// 0x846f80 output: one 0x28-byte vertex, (N+1)^2 of them, row r (v, along column 0) of N+1 vertices along u.
struct SurfaceVertex {
    std::array<float, 3> pos{};
    std::array<uint8_t, 3> normal{};  // bytes at +0x10 / +0x11 / +0x12 = z, y, x (D3DCOLOR order), floor(n 127 + 128)
    std::array<float, 2> uv0{}, uv1{};
    std::array<float, 3> colour{};    // packed by PackColorBGRA at +0x24
};
std::vector<SurfaceVertex> tessellatePatch(const SurfaceBody& s, const SurfacePatch& p, int n);
// The draw level N of 0x872df0 for a patch material with draw flags `materialFlags` (Material +0x48).
inline int patchDrawLevel(uint32_t materialFlags) { return (materialFlags & 0x1f) == 7 ? 8 : 3; }

// 0x7cf0a0: the collision grid, nv + 1 rows of nu + 1 points (x y z), object space.
std::vector<std::array<float, 3>> patchCollisionGrid(const SurfaceBody& s, const SurfacePatch& p, int nu, int nv);
// 0x811bc0's cell split: for cell (row r, column j) of a grid with nu + 1 points per row, triangles
// (P[r+1][j], P[r+1][j+1], P[r][j]) and (P[r+1][j+1], P[r][j+1], P[r][j]). Indices into the grid.
std::vector<uint32_t> patchGridTriangles(int nu, int nv);

// The patch's material: patch +0xac (a MaterialAnim_Z) -> that anim's Material_Z (vf05 0x80f6d0); 0 when missing.
uint32_t patchMaterial(const std::vector<const DpcFile*>& lookup, const SurfacePatch& p);

// The whole Surface_Z `surfaceId` as one static mesh in object space, as SurfacePc_Z::vf16 draws it (each patch at
// its draw level, one batch per patch). False (with `error`) when the object is missing or malformed.
bool bakeSurface(const std::vector<const DpcFile*>& lookup, uint32_t surfaceId, BakedMesh& out, std::string* error = nullptr);

}  // namespace fuel
