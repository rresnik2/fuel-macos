// PK28: FUEL's RotShape_Z objects (camera-facing quads around a pivot: light beams, halos, glows; class 33a508e6),
// decoded from FUEL.exe (notes pk28.md section 4):
//   load      RotShape_Z::vf03 0x87c8c0 (arrays 0x777d50 / 0x813cc0 / 0x6dd8b0 / 0x740b80)
//   fade      RotShape_Z::vf16 0x8834f0 (ComputeFade 0x672950 with the object's view distance +0x5c and near fade +0x9c)
//   draw      RotShapePc_Z::vf16 0x835a00: per point one record of the quad 0xa26a80 (0x859bc0) with RotShapeConstant
//             slot 0x12 {pivot, UPLEFT, scale = node +0x44, DWRIGHT}, vertex code 0xb (vh_normal) or 0xc (vh_front,
//             +0xa0 != 0), the material's draw state (opaque / blended phases, the type-4 occlusion flare)
//   shaders   scat_rotshape.vhl vh_front / vh_normal + scat_rotshape.phl ph_base (outdoor light types 6 / 7)
// No game data here: bodies are parsed from the player's DPC files at run time.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

class DpcFile;

namespace rotshape {

inline constexpr uint32_t kClass = 0x33A508E6;  // RotShape_Z

struct Point {
    std::array<float, 3> pos{};        // +0x6c array (12 B): the pivot in object space (RSHAPE_PIVOT)
    std::array<float, 3> upLeft{};     // +0x84 array, entry 2i (RSHAPE_UPLEFT)
    std::array<float, 3> downRight{};  // +0x84 array, entry 2i + 1 (RSHAPE_DWRIGHT)
    uint32_t material = 0;             // +0x7c array: index into `materials`
};

struct Shape {
    uint32_t id = 0;
    uint32_t flags = 0;        // Object_Z +0x60 (link byte 92)
    float viewDist = 0;        // Object_Z +0x5c (link byte 88): ComputeFade's view distance
    float nearFade = 0;        // +0x9c (body, after the handle list): ComputeFade's near fade
    bool front = false;        // +0xa0 != 0: vertex code 0xc = vh_front (else 0xb = vh_normal)
    std::array<float, 4> sphere{};  // Object_Z bound sphere (link 8..23)
    std::vector<Point> points;
    std::vector<uint32_t> materials;  // +0x98: Material_Z handles (IDs)
    // Arrays read but not used by the draw: +0x74 (16 B records) and +0x8c (8 B: the quad's uv corners in the data).
    size_t unused74 = 0, unused8c = 0;
};
// Parses a RotShape_Z body with its 98-byte link; throws on a malformed body (the grammar must end at the body end).
Shape parse(uint32_t id, const std::vector<uint8_t>& link, const std::vector<uint8_t>& body);

// 0x8834f0: the view alpha (view +0x160) of a RotShape at camera distance d. Starts from 1 (0x752bf0 sets it before each
// object); left at 1 when the shape has no view distance and no near fade (and the node's +0x5c >= 1); else
// ComputeFade(+0x5c, +0x9c, 0, K' = 10, s = node scale x view scale, d, node +0x5c). nodeAlpha = node +0x5c (1).
float fade(const Shape& s, float d, float nodeScale, float viewScale = 0.39720f, float Kfade = 10.0f, float nodeAlpha = 1.0f);

// 0x859bc0's quad 0xa26a80 (stride 0x28, triangle strip): corner weights P (xyz) and uv. The vertex shader takes
// pos = (1 - P) x DWRIGHT + P x UPLEFT component by component.
struct QuadVertex { std::array<float, 3> P; std::array<float, 2> uv; };
const std::array<QuadVertex, 4>& quad();

// scat_rotshape.vhl vh_front / vh_normal in view space (+z forward, y up): the corner of quad vertex k for the pivot
// in view space `pv`, world up in view space `up` (the view matrix's y column), node scale s and the alpha
// (GOURAUD_DFLT.w: at or below 1/256 the quad collapses to the pivot).
std::array<float, 3> corner(const Shape& s, const Point& p, int k, const std::array<float, 3>& pv, const std::array<float, 3>& up, float scale,
                            float alpha);

// A RotShape node of a world DPC.
struct Placement {
    uint32_t nodeId = 0;
    std::array<float, 16> world{};  // row vectors (as assembly Node)
    float lodScale = 1;             // node +0x44
    std::shared_ptr<const Shape> shape;
    std::string sub;                // the sub-world file name ("" = USA1 resident)
};
// Every RotShape_Z node of `dpc` (worldRootOnly: only the nodes under the World_Z root, as the particles); nodes within
// 50 m of the origin (library copies) are skipped. `sub` is stored in each placement.
std::vector<Placement> collect(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, bool worldRootOnly, const std::string& sub);

}  // namespace rotshape
}  // namespace fuel
