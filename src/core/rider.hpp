// The rider: FUEL's one pilot model (PILOT.DPC), skinned into the riding pose
// of a vehicle archetype and baked into a static mesh in vehicle space.
// See docs/formats.md, "Skinning and the rider".
#pragma once

#include "anim_clip.hpp"
#include "rider_jiggle.hpp"
#include "assembly.hpp"
#include "dpc.hpp"
#include "vehicle.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t Animation = 0x46107D89;
}

struct CustomMaterial;  // custom_rider.hpp (PHYSIQUE)

// A material of a baked mesh: a Material_Z, with its diffuse (slot 0) and
// specular (slot 4) bitmaps replaced when non-zero (livery).
struct BakedMaterial {
    uint32_t materialId = 0;
    uint32_t diffuseTex = 0, specularTex = 0;
    // PHYSIQUE (port deviation): a material of the player's custom rider, drawn from its own textures (materialId 0).
    const CustomMaterial* custom = nullptr;
};

struct BakedBatch {
    uint32_t material = 0;  // index into BakedMesh::materials
    uint32_t firstIndex = 0, indexCount = 0;
};

// Static geometry with interleaved 36-byte vertices: float3 position, D3DCOLOR
// tangent @12, D3DCOLOR normal @16, float2 uv @20, float2 uv1 @28.
struct BakedMesh {
    static constexpr uint32_t kStride = 36;
    std::vector<uint8_t> vertices;
    std::vector<uint32_t> indices;  // triangle lists, relative to the first vertex
    std::vector<BakedMaterial> materials;
    std::vector<BakedBatch> batches;
    // Vertices are stored batch after batch; each batch holds the vertices its
    // triangles use, in source order.

    uint32_t vertexCount() const { return uint32_t(vertices.size() / kStride); }
    std::array<float, 3> position(uint32_t i) const;
};

// The AddVehicleInfo archetype (last token) of a vehicle variant such as
// "QUAD_ROAD_2" in VehiclesDef.tsc; empty if there is none.
std::string vehicleArchetype(const std::string& tscText, const std::string& variant);

// P66: names (CRC) of the pilot DPC's MaterialAnim_Z (the liveries FUEL looks up with 0x67a760).
std::set<uint32_t> riderLiveryNames(const DpcFile& pilot);
// 0x5dc850: how many of "LIVERY_%02d_<part>" k = 1, 2, ... exist (stopping at the first missing one) or, with
// `types`, of "LIVERY_01_<part><k>" (the skin page's body types, 0x4750e0).
int riderLiveryCount(const std::set<uint32_t>& names, const std::string& part, bool types = false);

// Where FUEL seats the rider (P59): Rider_PlaceRootNode 0x5ca410 (called every frame by 0x5d7e00) puts the
// pilot object at node 61 of the vehicle visual's node array (veh+0x90 -> +0x68 -> +0x6c, +0x24 node pointers)
// plus the sway offset, rotated by that node's rotation times the vehicle's. Node arrays are indexed by the
// engine's bone-name registry (the rider animations' bone records carry the same indices: root 15, Hips 25,
// LeftHand 27, Head 61, ...), so node 61 is the vehicle skeleton's "Head" bone. The riding poses are authored
// relative to it. Returns its translation in vehicle space; nullopt when the skeleton has no Head bone (FUEL
// then leaves the pilot where it was). `rotationIdentity`: the bone has no rotation (true for every shipped
// vehicle; the port applies the translation only).
struct SeatPoint {
    std::array<double, 3> t{};
    bool rotationIdentity = true;
};
std::optional<SeatPoint> riderSeatPoint(const DpcFile& vehicleDpc, const AssembledVehicle& av);

// The rider's IK targets on a vehicle (P59b): IT_PilotMove 0x604630 -> 0x603270 / 0x6022b0 look up IK_LEFTHAND,
// IK_RIGHTHAND, IK_LEFTFOOT, IK_RIGHTFOOT by name CRC in the vehicle object (veh+0x90 = pm+0x390) with 0x6a23b0:
// the first part of the current LOD whose resource knows the named point (vslot 0x60), placed by the part's node
// (Obb_PlaceByNode). In the data they are named BOXES of the parts' Mesh_Z links (the hull's). Vehicle space, in
// that order; NaN x when the vehicle has none.
std::array<std::array<float, 3>, 4> riderIkPoints(const DpcFile& vehicleDpc, const AssembledVehicle& av);
// PK31: the hand boxes' orientations (IK_LEFTHAND, IK_RIGHTHAND) in vehicle space as row-vector rotations (rows =
// images of the box axes), for 0x604410 (Quat_FromMatrix 0x644c10 of the placed box). NaN [0] when the hand has no
// box (a sphere point has no orientation).
std::array<std::array<double, 9>, 2> riderIkHandFrames(const DpcFile& vehicleDpc, const AssembledVehicle& av);

class Rider {
public:
    explicit Rider(const DpcFile& pilot);  // throws on malformed resources

    // Animation slot of the neutral seated pose of an archetype (BUGGY,
    // BUGGY_ROAD, MBIKE, MBIKE_SPORT, MBIKE_CHOPPER, MBIKE_TRAIL, MUSCLE,
    // QUAD, QUAD_TRAIL, SUV, TRUCK); -1 for an unknown archetype.
    int ridingSlot(const std::string& archetype) const;

    // The Head<head>, Chest<chest> and Legs<legs> parts skinned to animation
    // `slot` at `time` seconds, with liveries `livery` (1..) applied.
    BakedMesh bake(int slot, double time = 0, int head = 1, int chest = 1, int legs = 1, int livery = 1) const;

    // Bone world matrices of a pose (row-vector, model space), for inspection.
    std::vector<Mat4> pose(int slot, double time) const;
    size_t boneCount() const { return skel_.bones.size(); }

    // ---- per-frame animation (rider_anim.*) ----
    struct BoneTRS {
        std::array<double, 4> q{0, 0, 0, 1};
        std::array<double, 3> t{}, s{1, 1, 1};
    };
    bool hasSlot(int slot) const { return animSlots_.count(uint32_t(slot)) != 0; }
    float duration(int slot) const;  // seconds
    bool loops(int slot) const;      // Animation_Z flags bit 0 (header u16 at +8)
    // Local TRS of every bone at `time` (bones without a track keep the bind TRS); `has[b]` = the animation
    // has a track for bone b (the layer's per-bone mask).
    void sampleLocal(int slot, double time, std::vector<BoneTRS>& out, std::vector<uint8_t>* has) const;
    std::vector<BoneTRS> bindLocal() const;
    // P76: animation events of `slot` in the window (Clip::events) on every track: appends (bone name CRC, kind).
    void events(int slot, double from, double to, bool closed, std::vector<std::pair<uint32_t, uint32_t>>& out) const;
    // World (model space) matrices from local TRS (FK, p9 S9).
    std::vector<Mat4> worlds(const std::vector<BoneTRS>& local) const;
    int bone(const std::string& name) const;  // -1 when absent
    int parent(int b) const { return skel_.bones[size_t(b)].parent; }

    // Skinning prepared once for a choice of parts and livery; skin() writes BakedMesh vertices for a pose.
    struct SkinTemplate {
        BakedMesh mesh;  // indices, batches, materials; vertices of the bind
        struct V {
            float pos[3], nrm[3], tan[3];
            int bone[4];
            float w[4];
            uint8_t jig = 0;  // PHYSIQUE: 1 + the glute helper bone (RiderJiggle) moving this vertex, 0 none
            float jw = 0;     // its weight
        };
        std::vector<V> verts;
        std::array<JiggleBone, RiderJiggle::kBones> jiggleBones{};  // PHYSIQUE: the helper bones (custom rider only)
        bool hasJiggle = false;
    };
    SkinTemplate prepareSkin(int head = 1, int chest = 1, int legs = 1, int livery = 1) const;
    // P66: the garage's choice (game +0x33d0..+0x33ec, 0x42b420): part types, a texture livery per part
    // (LIVERY_<k>_HEAD<n>, ..._CHEST<n>, ..._LEGS<n>) and the skin = body type with its livery (LIVERY_<k>_BODY<skin>).
    struct Parts {
        int head = 1, chest = 1, legs = 1, headLivery = 1, chestLivery = 1, legsLivery = 1, bodyLivery = 1, skin = 1;
    };
    // P104: `level` = the rider Lod_Z's level (0 = LOD1; its Skin_Z objects[level], the same parts at less detail).
    SkinTemplate prepareSkin(const Parts& parts, int level = 0) const;
    // ---- PHYSIQUE: only some of the three parts (bit 0 head, 1 chest, 2 legs); the material a FUEL part batch with
    // material name `nameCrc` (record [0]) draws with under `parts`' liveries, `preferred` = the mesh's own Material_Z ids;
    // a bone by its name CRC (-1 when absent).
    SkinTemplate prepareSkin(const Parts& parts, int level, unsigned partMask) const;
    BakedMaterial partMaterial(uint32_t nameCrc, const std::vector<uint32_t>& preferred, const Parts& parts) const;
    int boneByCrc(uint32_t crc) const {
        const auto it = boneByName_.find(crc);
        return it == boneByName_.end() ? -1 : int(it->second);
    }
    // ---- PHYSIQUE end
    // P104: the rider Lod_Z 9a9c3e61's switch data (Lod_Z::Draw 0x884170): levels (3), D (2.0), view distance (15).
    int lodLevels() const { return int(lodSkins_.size()); }
    float lodBaseDistance() const { return lodD_; }
    float lodViewDist() const { return lodView_; }
    // PHYSIQUE: `jiggle` (the player's custom rider, active) adds the glute helper bones' offsets on top of the skinning.
    void skin(const SkinTemplate& t, const std::vector<Mat4>& world, std::vector<uint8_t>& vertices, const RiderJiggle* jiggle = nullptr) const;
    // PHYSIQUE: bone b's skin matrix (bind -> the space of `world`, row-vector).
    Mat4 skinMatrix(int b, const std::vector<Mat4>& world) const;

private:
    // P47: FUEL's Animation_Z and samplers (anim_clip.hpp: slerp, Hermite with tangents, Euler tracks; the
    // port sampled the keys linearly before).
    struct Animation {
        anim::Clip clip;
        std::map<uint32_t, int> tracks;  // bone name CRC -> clip track
        float duration = 0;
        bool loop = false;
    };
    const Animation& animation(int slot) const;
    std::vector<Mat4> poseWorlds(const Animation* a, double time) const;
    uint32_t materialNamed(uint32_t nameCrc, const std::vector<uint32_t>& preferred) const;
    uint32_t liveryMaterial(const std::string& name) const;

    const DpcFile& dpc_;
    Skel skel_;
    Skin skin_;
    std::vector<Skin> lodSkins_;  // P104: [0] = skin_
    float lodD_ = 1, lodView_ = 0;  // P104
    std::map<uint32_t, uint32_t> animSlots_;  // slot -> Animation_Z id
    std::map<uint32_t, size_t> boneByName_;
    std::map<uint32_t, uint32_t> materialByName_;  // Material_Z by name CRC (first)
    std::map<uint32_t, uint32_t> liveryByName_;    // MaterialAnim_Z name CRC -> Material_Z
    std::vector<Mat4> invBind_;                    // inverse bind world per bone
    mutable std::map<uint32_t, Animation> anims_;
};

}  // namespace fuel
