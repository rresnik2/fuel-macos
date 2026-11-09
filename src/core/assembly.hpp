// Vehicle assembly: which meshes make up one drivable vehicle variant, and
// where each one sits at rest.
//
// A vehicle DPC holds one GameObj_Z per variant (for example QUAD_ROAD_2):
// named groups of Node_Z, an un-suffixed group (COMMON) and <V>_LOD1..LOD4.
// The vehicle's Node_Z matrices are all identity; placement comes from the
// vehicle skeleton instead. A COMMON-group Lod_Z leads (Skin_Z, then its
// Skel_Z) to that skeleton, and its bones carry UserDefine_Z text with `V:`
// commands that name each part and say where it goes. See docs/formats.md,
// "Vehicle assembly".
#pragma once

#include "vehicle_lights.hpp"
#include "vehicle_links.hpp"  // PK22
#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t Node = 0x85D02528;
inline constexpr uint32_t Lod = 0x73DC6A13;
inline constexpr uint32_t Skin = 0x53415807;
inline constexpr uint32_t Skel = 0xD73B85EC;
inline constexpr uint32_t GameObj = 0xF42D99BD;
inline constexpr uint32_t UserDefine = 0x52F79F96;
inline constexpr uint32_t Omni = 0x20C0683D;
}  // namespace cls

// 4x4 matrix, 16 values row-major in the Direct3D row-vector convention:
// p_world = [x y z 1] * M, translation in elements 12..14.
using Mat4 = std::array<double, 16>;

// ---- scene-graph resources --------------------------------------------------

// Node_Z body (234 bytes): ten resource links, a 0x82-byte transform block
// and the node's world matrix.
struct Node {
    uint32_t parent = 0, child = 0, next = 0, prev = 0;
    uint32_t object = 0;      // Mesh_Z, Lod_Z, Omni_Z, ...
    uint32_t objectData = 0;  // MeshData_Z, LodData_Z, ...
    uint32_t userDefine = 0;
    uint32_t light = 0;
    // node+0x2c (transform block +0x1c): 0x8000000 = in the physics scene (gathered by 0x6431d0 for 0x71dd80),
    // 0x40000 = excluded from it, 0x4000 Lod node, 0x80000000 light node.
    uint32_t flags = 0;
    float lodScale = 1;  // node+0x44 (transform block +0x34): the scale Lod_Z::Draw / ComputeFade use
    std::array<float, 16> world{};  // row-vector convention
};
Node parseNodeBody(const std::vector<uint8_t>& body);  // throws

// The 98-byte link header shared by object classes (Mesh_Z, Lod_Z, Skin_Z,
// Omni_Z, ...): +0 name CRC, +4 data resource (a Skin_Z's Skel_Z).
struct ObjectLink {
    uint32_t name = 0;
    uint32_t dataId = 0;
};
ObjectLink parseObjectLink(const std::vector<uint8_t>& link);  // throws

// A named local point of an object (attachment points such as LOCAL_LINK
// and LOCAL_LOOKAT), in the object's own space.
struct NamedSphere {
    std::array<float, 3> centre{};
    float radius = 0;
    uint32_t type = 0;
    uint32_t name = 0;  // CRC of the point name
};
// The named spheres listed after a Mesh_Z link's dependency list.
std::vector<NamedSphere> parseMeshLinkSpheres(const std::vector<uint8_t>& link);  // throws
// The named boxes that follow them (72 bytes: three rows {r0 r1 r2 t} of the local frame, half extents, radius,
// type, name). The rider's IK_LEFTHAND / IK_RIGHTHAND / IK_LEFTFOOT / IK_RIGHTFOOT targets are such boxes (P59b).
struct NamedBox {
    std::array<float, 12> rows{};  // row k = (axis k x, y, z, translation k)
    std::array<float, 3> half{};
    float radius = 0;
    uint32_t type = 0;
    uint32_t name = 0;
    std::array<float, 3> centre() const { return {rows[3], rows[7], rows[11]}; }
};
std::vector<NamedBox> parseMeshLinkBoxes(const std::vector<uint8_t>& link);  // throws

// GameObj_Z body: named groups of Node_Z IDs.
struct NodeGroup {
    std::string name;
    uint32_t value = 0;
    std::vector<uint32_t> nodes;
};
std::vector<NodeGroup> parseGameObjBody(const std::vector<uint8_t>& body);  // throws

// Lod_Z body: named points, then one object per LOD level.
struct Lod {
    std::vector<NamedSphere> spheres;
    std::vector<NamedBox> boxes;  // P74: e.g. DURL, CAR_OCCLUSION (vehicle Lod_Z)
    // Lod+0x7c near fade, +0x80 hard cull, +0x84 base distance D (metres; Lod_Z::Draw 0x884170,
    // ComputeFade 0x672950, p6 W7/W8): level k is drawn from D * 2^(k-1) on (scaled, see world.hpp).
    float nearFade = 0, hardCull = 0, baseDistance = 1;
    std::vector<uint32_t> objects;  // [0] = LOD1, ...
    // (animation slot, Animation_Z id) pairs; the rider's Lod_Z lists its poses here.
    std::vector<std::pair<uint32_t, uint32_t>> animations;
    uint32_t userDefine = 0;  // UserDefine_Z (e.g. a vegetation prototype's "GenWorld Arbre" script)
};
Lod parseLodBody(const std::vector<uint8_t>& body);  // throws

// Skin_Z body; only the mesh list matters for placement.
// One per-batch record of a skinned part (0x60 bytes): [0] material name CRC,
// [1..21] the bone-name palette (0xffffffff = keep the previous batch's slot,
// 0xfffffffe = end of list), the rest is runtime state.
using SkinRecord = std::array<uint32_t, 24>;
struct Skin {
    std::vector<uint32_t> meshes;
    std::vector<std::vector<SkinRecord>> sets;  // one set per mesh, one record per batch
};
Skin parseSkinBody(const std::vector<uint8_t>& body);  // throws

// Skel_Z bone. The file stores the local TRS (relative to the parent) and
// the world bind matrix; assembly uses the world matrix directly.
struct Bone {
    uint32_t name = 0;        // CRC of the bone name ("Hips", "LeftHand", ...)
    uint32_t userDefine = 0;  // UserDefine_Z with the bone's commands, 0 = none
    std::array<float, 4> restQuat{};  // second rotation copy (bone +0x60), xyzw
    std::array<float, 4> localQuat{};   // local rotation (xyzw), scale and translation
    std::array<float, 3> localScale{}, localT{};
    std::array<float, 16> world{};    // world bind matrix, row-vector convention
    int32_t firstChild = -1, parent = -1, nextSibling = -1, prevSibling = -1;
};
struct Skel {
    std::vector<Bone> bones;
    // P96: the per-mesh bone-name lists after the Material_Z / MeshData_Z lists (Skel_Z::Load 0x7b4830 registers each
    // name as a global bone id, 0x748260, in this order).
    std::vector<std::vector<uint32_t>> palettes;
    // ---- PK22: the named spheres and boxes of the body tail (Skel_Z +0x40 / +0x48: 0x66bd60 / 0x66bdf0), the
    // V:DYN records' POINT= / BOX= / LIMB= (core/vehicle_links.hpp)
    std::vector<vlinks::NamedPoint> points;
    std::vector<vlinks::NamedObb> boxes;
    // ---- PK22 end
};
Skel parseSkelBody(const std::vector<uint8_t>& body);  // throws

// UserDefine_Z body: a text whose `[` `]` are dropped and whose `;` separate
// lines. Returns the non-empty lines, trimmed.
std::vector<std::string> parseUserDefineBody(const std::vector<uint8_t>& body);  // throws

// One bone command, e.g. `V:LINKB "A+LOCAL_LINK" "B" AxeZ LOD1`.
struct BoneCommand {
    std::string keyword;            // first token, upper case
    std::vector<std::string> args;  // quoted tokens
    std::vector<std::string> tags;  // bare tokens (LOD1, AxeZ, LIGHT:1, ...)
    std::string text;               // the whole line

    // A line applies to LOD n when one of its LOD tags contains the digit n;
    // untagged lines apply to LOD1 only.
    bool appliesToLod(int lod) const;
};
std::optional<BoneCommand> parseBoneCommand(const std::string& line);

// ---- names -------------------------------------------------------------------

// Resources refer to names by CRC only. This table recovers the strings that
// occur in the DPC (UserDefine_Z commands, GameObj_Z group names) plus the
// biped bone names the vehicle rigs use, for display.
class NameTable {
public:
    explicit NameTable(const DpcFile& dpc);
    const std::string* find(uint32_t crc) const;
    // The name, or "?<crc in hex>" when it is not known.
    std::string operator()(uint32_t crc) const;

private:
    std::unordered_map<uint32_t, std::string> names_;
};

// ---- assembly ---------------------------------------------------------------

enum class Placement { Link, LinkB, LinkR, LinkM, LinkMM, Wheel, WheelFlipped, Unplaced };
enum class PartShape { Mesh, SkinBindPose, Light };

struct PlacedPart {
    uint32_t meshId = 0;  // Mesh_Z to draw; 0 for lights
    Mat4 world{};
    Placement placement = Placement::Unplaced;
    PartShape shape = PartShape::Mesh;
    // The part has no LOCAL_LOOKAT point, so the game leaves it at its node matrix.
    bool keptNodeMatrix = false;
    int bone = -1;  // index of the commanding bone in the skeleton; -1 = unplaced
    uint32_t boneName = 0;
    // Where it came from.
    uint32_t nodeId = 0, objectId = 0, objectClass = 0, objectName = 0;
    std::string group;    // GameObj_Z group that holds the node
    std::string command;  // the V: line that placed it
    uint32_t skinId = 0, skelId = 0;  // SkinBindPose parts
    std::string hub;  // wheels: "PART+POINT" when a V:LINKWHEEL point gives the hub, else "bone"
    // ---- PK22: a V:LINK with a DYN:n tag (0x69e020 link record +0x24): the index into AssembledVehicle::dyn, -1 none;
    // the link bone's skeleton-space translation and +0x60 rotation (0x6a0200's V:DYN pivot and boneQ).
    int dyn = -1;
    std::array<float, 3> dynPivot{};
    std::array<float, 4> dynBoneQ{0, 0, 0, 1};
    // ---- PK22 end
    // ---- [suspanim] what Skel_Z::UpdateVehicleLinks 0x6a0200 aims the record at every frame (spec S18; core/susp_anim):
    // V:LINK A / V:LINKR A [C] -> bones, V:LINKB A+P / V:LINKMM .. C+P2 -> a point of another part (vehicle space, rest),
    // V:LINKM / V:LINKMM -> placed at a point of part `atObject`; the hinge axis (X, Z with AxeZ), the part's LOCAL_LOOKAT,
    // and a V:LINKR part's LOCAL_ATTACH / LOCAL_LIMIT (the fork slider).
    struct LinkAim {
        int bone = -1, bone2 = -1;  // aim bones (skeleton indices), -1 none
        uint32_t object = 0;        // aim part (object id), 0 none
        std::array<float, 3> point{};
        uint32_t atObject = 0;      // V:LINKM / LINKMM: the object A whose point places the part
        int axis = 0;               // 0: X (record +4), 2: Z (AxeZ, record +0xc)
        bool hasLook = false;
        std::array<float, 3> look{};
        bool hasAttach = false;     // LOCAL_ATTACH + LOCAL_LIMIT found
        std::array<float, 3> attach{}, limit{};
        float limitRadius = 0;
    } aim;
    // ---- [suspanim] end

    // What the game draws: placed meshes (unplaced group nodes and lights are listed for inspection).
    bool drawn() const { return meshId != 0 && placement != Placement::Unplaced; }
};

// The part's kind as reverse-engineering tools/scene_dump.py names it, e.g.
// "mesh_link", "skin_bindpose_linkb", "mesh_wheel_flipped", "light_unplaced".
std::string partKind(const PlacedPart& p);

struct VehicleVariant {
    uint32_t gameObjId = 0;
    uint32_t name = 0;  // CRC of the variant name (the GameObj_Z link)
    std::string label;  // the name when known, else "?<crc>"
    std::vector<std::string> groups;
};

// P77: a headlight (V:HEADLIGHT:n + V:LINK ... LIGHT:n, notes p77-shadows-lights.md 2).
struct VehicleLight {
    std::string key;      // "LIGHT:1"
    uint32_t omniId = 0;  // the Omni_Z
    int bone = -1;        // the bone it hangs from
    Mat4 world{};         // vehicle space (the bone's bind matrix), row vectors
    omni::Params params;
    omni::ObjectInfo info;  // PK28: Omni_Z flags, bound sphere, spot angles (light gathers 0x6a0200 / 0x7984c0)
};

struct AssembledVehicle {
    uint32_t gameObjId = 0;
    uint32_t name = 0;
    int lod = 1;
    std::vector<std::string> groups;   // groups used: un-suffixed ones and <V>_LOD<lod>
    std::optional<uint32_t> skeleton;  // the vehicle Skel_Z
    std::vector<PlacedPart> parts;
    std::vector<std::string> problems;  // commands that name missing parts or points
    std::vector<std::pair<int, std::string>> otherCommands;  // (bone, line): lights, dynamics, ...
    // P74: the half extents of the vehicle's CAR_OCCLUSION omni (Vehicle_CarOcclusion 0x5cc410, carOcclusionExtents).
    std::array<float, 3> occlusionExtents{0.75f, 0.75f, 0.75f};
    uint32_t occlusionLod = 0;      // the Lod_Z the box came from (0: none, the 0.75 default)
    uint32_t occlusionBoxName = 0;  // CAR_OCCLUSION, DURL or 0
    // P77: min / max of every box of that Lod_Z (0x5ca7a0: Obb_PlaceByNode 0x743ce0 + merge 0x744910), as half
    // extents and centre in vehicle space; the body blob's size when the wheels do not give it. Zero: no boxes.
    std::array<float, 3> boxHalf{}, boxCentre{};
    // P104: the vehicle Lod_Z's switch data (Lod_Z::Draw 0x884170, notes p104-vehicle-lod.md 1.1): its level count
    // (the GameObj's LOD chain; 1 when none), base distance D (Lod+0x84), view distance (link +88: the Lod fade, 0 =
    // none) and its node's scale (node+0x44).
    int lodLevels = 1;
    float lodBaseDistance = 1, lodViewDist = 0, lodNodeScale = 1;
    std::vector<VehicleLight> lights;  // P77
    // P85: the "Engine" bone's position (vehicle space, bind pose): where 0x5d00c0 / 0x5cfeb0 hang the damage smoke.
    bool hasEngine = false;
    std::array<float, 3> engine{};
    // ---- PK22: the V:DYN records of the skeleton (0x69e020, in command order; reset) and whether it has V:HEADLIGHT
    // records (0x69df30: the lamp groups POSITION / HEAD need them).
    std::vector<vlinks::DynRecord> dyn;
    bool headlightRecords = false;
    // ---- PK22 end
    // ---- PK23: the exhaust slots (0x5cf170: named boxes SFX_EXHAUST1..16 of the parts, vehicle space; core/vehicle_fx)
    struct Exhaust {
        std::array<float, 3> pos{}, dir{0, 0, -1};
        int index = 0, dyn = -1;
    };
    std::vector<Exhaust> exhausts;
    // ---- PK23 end
    // [upgrades U5] where it came from (set by VehicleAssembler::assemble; valid while the assembler lives): the visible
    // upgrades (core/upgrade_visuals) read the parts' meshes and names through them.
    const DpcFile* dpc = nullptr;
    const NameTable* names = nullptr;
    // [suspanim] the skeleton's bones at rest (vehicle space translation of each bind matrix), for the per-frame links.
    std::vector<std::array<float, 3>> bonePos;
};

// P74: FUEL's car occlusion omni (Vehicle_CarOcclusion 0x5cc410, notes p74-lighting.md 1): an Omni_Z instanced from
// the world's OMBREVOITURES agent as a child of the vehicle node, of half extents e = the vehicle Lod_Z's box
// CAR_OCCLUSION (+0x30 of the 0x48-byte box, Lod_Z::vf24 0x87ee90), else its box DURL + 0.75, else 0.75 each.
// The vehicle Lod_Z is the object of the vehicle agent's root node: the root (parent-less) node of the variant whose
// Lod_Z carries the LOD chain (most levels). Returns the extents, the Lod and the box name used.
// `lod` = that Lod_Z (nullptr: none).
std::array<float, 3> carOcclusionExtents(const Lod* lod, uint32_t* boxName = nullptr);

// Assembles the variants of one vehicle DPC; parsed resources are cached.
class VehicleAssembler {
public:
    explicit VehicleAssembler(const DpcFile& dpc);
    ~VehicleAssembler();
    VehicleAssembler(const VehicleAssembler&) = delete;
    VehicleAssembler& operator=(const VehicleAssembler&) = delete;

    const DpcFile& dpc() const { return dpc_; }
    const NameTable& names() const { return names_; }
    // GameObj_Z variants in file order.
    const std::vector<VehicleVariant>& variants() const { return variants_; }
    // By variant name, GameObj_Z ID in hex, or the start of a group name
    // ("QUAD_ROAD_2" matches "QUAD_ROAD_2_LOD1"); nullptr if none matches.
    const VehicleVariant* find(const std::string& selector) const;

    // The parts for LOD `lod` (1 = highest detail), undamaged, at rest.
    // Throws on malformed resources.
    AssembledVehicle assemble(const VehicleVariant& variant, int lod = 1) const;

private:
    struct Resources;

    const DpcFile& dpc_;
    NameTable names_;
    std::vector<VehicleVariant> variants_;
    std::unique_ptr<Resources> res_;
};

// The variants of a vehicle DPC (name, GameObj_Z ID, groups).
std::vector<VehicleVariant> listVehicleVariants(const DpcFile& dpc);

}  // namespace fuel
