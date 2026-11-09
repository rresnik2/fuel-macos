// FUEL's static collision geometry of a placed object (P46; notes p46-contacts-damage.md part S), read from the
// player's game data at run time:
//   - a node takes part when its flags (node+0x2c) have 0x8000000 and not 0x40000 (0x6431d0 from 0x71dd80);
//   - its object's collision comes from the object's vtable slots +0x84 boxes, +0x88 triangles, +0x8c spheres:
//       Lod_Z (0x884dc0 / 0x885110 / 0x884f50): the Lod body's spheres and boxes; with the Lod flag 0x400000
//         (link +0x5c = Lod+0x60) also its first mesh's boxes and spheres, else the triangles of its first
//         (loaded) level mesh;
//       Mesh_Z (0x80af40 / 0x80b1d0 / 0x80b070): the link's spheres and boxes and the body's triangle tree;
//   - shapes and triangles are kept by type: boxes and spheres with type & 2, triangles whose Material_Z flags
//     (Material+0x38 = body +0x2c) have 2 and neither 0x800 nor 8 (the vehicle masks of 0x71dd80);
//   - triangle vertices are int16 / 1024 m, transformed by the node's world matrix (0x817cf0).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel {

struct StaticCollision {
    struct Sphere {
        std::array<float, 3> c{};
        float r = 0;
        uint32_t type = 0;
    };
    struct Box {
        std::array<float, 12> frame{};  // 3 x 4 rows: axis i in [4i..4i+2], centre in column 3
        std::array<float, 3> half{};
        float r = 0;
        uint32_t type = 0;
    };
    std::vector<Sphere> spheres;
    std::vector<Box> boxes;
    // P58: the boxes the AI racers steer round (0x619d70): kind = type & 0x1c000000 not 0x10000000, and type & 7
    // or kind 0x8000000 or kind 0x4000000 (the last are jump ramps and the like: types 2/3 of 0x6189b0).
    std::vector<Box> aiBoxes;
    // Triangles (object space, metres) that collide with vehicles, with their material flags.
    std::vector<std::array<float, 9>> tris;
    std::vector<uint32_t> triFlags;
    float radius = 0;  // bounding radius around the object origin
    bool empty() const { return spheres.empty() && boxes.empty() && tris.empty(); }
};

// The collision of object `objectId` (Lod_Z, Mesh_Z or Surface_Z: its patch grids, surface_patch.hpp); empty when
// it has none.
StaticCollision loadStaticCollision(const std::vector<const DpcFile*>& lookup, uint32_t objectId);

// Node flags test of 0x6431d0 (mask 0x8000000, exclusion 0x40000).
inline bool inPhysicsScene(uint32_t nodeFlags) { return (nodeFlags & 0x8000000u) != 0 && (nodeFlags & 0x40000u) == 0; }

// P50 (notes p50-collision-world.md): a node attached to the world goes into the collision hash World_Z +0xac
// (0x65ef40, from Node_SetWorld 0x65e910, the transform walk 0x65f880 and the AutoMesh spawn 0x778650) unless its
// flags have 0x10000 or its object's flags (Object_Z +0x60) have 8; 0x639ba0 gathers only hashed nodes. Hiding a
// node (0x40000, e.g. a CONDITION_OBJECT through 0x513fd0 -> 0x65eed0 -> 0x6430c0) also takes it out of the hash.
inline bool inCollisionHash(uint32_t nodeFlags, uint32_t objectFlags) { return (nodeFlags & 0x10000u) == 0 && (objectFlags & 8u) == 0; }
inline bool staticSolid(uint32_t nodeFlags, uint32_t objectFlags) { return inPhysicsScene(nodeFlags) && inCollisionHash(nodeFlags, objectFlags); }

// The game-object class a world node becomes (P50): sub-world nodes are named after clone.tsc classes
// (`cloneclass <name> <base>`, CommonComponent/clone.tsc, read at run time; node link u32 0 = AsoboCRC(name)):
// IT_DECORMESH_G 3,294 nodes (static decor), IT_DECORDYNMESH_G 1,239 (breakable decor: static until hit, then its
// IT_DYNOBJ_G pieces take over; 0x51b930 hides the decor node), IT_DYNOBJ_G 1,015 (dynamic objects), Agent_Z 90,
// IT_ANIMATED 21. Others (and unnamed nodes) are plain scene nodes.
enum class GameClass : uint8_t { None, DecorMesh, DecorDynMesh, DynObj, Debris, Collect, Animated, Agent, Other };
const char* gameClassName(GameClass c);
// name CRC -> class, resolving clones of clones to their RTTI base. Empty when the file is missing.
std::unordered_map<uint32_t, GameClass> loadCloneClasses(const std::string& cloneTscPath);
// Points for a convex hull of the collision (box corners, sphere extremes, triangle vertices), object space.
std::vector<float> collisionHullPoints(const StaticCollision& c);

}  // namespace fuel
