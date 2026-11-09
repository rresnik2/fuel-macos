// FUEL's narrow-phase collision tests between the vehicle's shapes and the static world (PhysicWorld_Z,
// FUEL.exe 0x71xxxx-0x76xxxx), decoded in ~/re-work/fuel/notes/p46-contacts-damage.md (part C):
//   - the oriented box record (0xac bytes; 0x742b80 from a 3x4 frame, 0x743ce0 with the node matrix),
//   - sphere vs box 0x71a9e0 and sphere vs sphere 0x71b1b0,
//   - the box culls 0x7266a0 / 0x726160 and box vs box 0x763890 (ODE's dBoxBox with FUEL's 0.001 m
//     separation tolerance, up to 20 contacts, every point moved by n * depth / 2),
//   - box vs triangle: separating axes 0x765bc0 (0x765800 face axes, 0x765920 edge axes, edges weighted 1.5)
//     and the contact clipper 0x7667e0 (0x765a50 polygon clip, 0x766740 closest approach of two lines).
#pragma once

#include "fuel_contacts.hpp"

#include <cstdint>

namespace fuel::fv {

// The engine's box record: axes = the rows of the rotation (+0x00, +0x10, +0x20; their transpose at +0x40),
// centre +0x80, half extents +0x90, bounding radius +0xa0, type flags +0xa4.
struct Obb {
    Vec3 ax[3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Vec3 c;
    Vec3 h;
    float r = 0;
    uint32_t type = 0;
};

// A collision box of a Lod_Z / Mesh_Z (72 bytes: 3x4 frame with the centre in column 3, half extents, radius)
// placed by a node's world matrix (row-vector 4x4, scale in the rows): 0x743ce0 then 0x742b80. The half extents
// and the radius are scaled by |row 0| of the node matrix.
Obb placeObb(const float frame34[12], const float half[3], float radius, const float world[16]);

// 0x71a9e0: sphere (c, r) vs box. On a hit: n from the box towards the sphere, depth, p = c - n (r - depth / 2).
bool sphereObb(const Obb& b, const Vec3& c, float r, Vec3& p, Vec3& n, float& depth);
// 0x71b1b0: sphere (c, r) vs a static sphere (cs, rs). n from the static sphere towards c, p = c + n (r - depth / 2)
// (sic: the far side of the query sphere).
bool sphereSphere(const Vec3& cs, float rs, const Vec3& c, float r, Vec3& p, Vec3& n, float& depth);

// 0x7266a0: a sphere (the body's bound) touches the box. 0x726160: two boxes' bounding spheres touch the other box.
bool obbSphereCull(const Obb& b, const Vec3& c, float r);
bool obbObbCull(const Obb& a, const Obb& b);

// 0x763890: box `a` (the static box, box 1) vs box `b` (the body's box, box 2). Returns the number of contacts
// (0 = apart); n points from a towards b; pts / depth hold up to 20 contacts.
int boxBox(const Obb& a, const Obb& b, Vec3& n, Vec3 pts[20], float depth[20]);

// 0x765bc0 + 0x7667e0: box vs triangle. Returns the number of contacts (up to 8); the contact normal the engine
// uses is the triangle's unit normal (0x724120), the clipper's axis is returned in `axis` for the record.
int boxTriangle(const Obb& box, const Tri& t, Vec3 pts[8], float depth[8], Vec3* axis = nullptr);

}  // namespace fuel::fv
