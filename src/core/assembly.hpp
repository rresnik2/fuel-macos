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
    std::vector<uint32_t> objects;  // [0] = LOD1, ...
    // (animation slot, Animation_Z id) pairs; the rider's Lod_Z lists its poses here.
    std::vector<std::pair<uint32_t, uint32_t>> animations;
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

struct AssembledVehicle {
    uint32_t gameObjId = 0;
    uint32_t name = 0;
    int lod = 1;
    std::vector<std::string> groups;   // groups used: un-suffixed ones and <V>_LOD<lod>
    std::optional<uint32_t> skeleton;  // the vehicle Skel_Z
    std::vector<PlacedPart> parts;
    std::vector<std::string> problems;  // commands that name missing parts or points
    std::vector<std::pair<int, std::string>> otherCommands;  // (bone, line): lights, dynamics, ...
};

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
