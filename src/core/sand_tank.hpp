// PK12: FUEL's STARTFX effect records (WeatherManager_G +0x130) and the sand-storm walls (SpecialEffectsSandTank_G,
// weather +0x14c, and its SpecialEffectsSandMesh_G slots), decoded from FUEL.exe; notes ~/re-work/fuel/notes/pk12.md
// (on top of p98 b014 / b016 / b029 / b060). Nothing of the game is compiled in: the tuning is the player's pp001.tsc
// (rows 5380-5399), the volumes come from the course DPCs at run time.
//
//   - EvenementVolAgent_G STARTFX (0x570ae0): for each child C of the volume node whose object is a Spline_Z (type 2) and
//     whose first child G holds a CollisionVol_Z (type 0xe): 0x5bc340(G) finds or appends a 0x70-B record, 0x5c0710
//     fills it (TIME from C's UserDefine, default 120 s; type 1 when G's UserDefine has SAND; G's OBB 0x7375d0) and a
//     SAND record takes a sand-tank slot (0x5c0850 -> 0x5ec1d0: a SandMesh).
//   - Every weather frame 0x5bc3b0 drops the records flagged dead and steps the others (0x5c08b0): elapsed += dt, G
//     hidden -> dead; the spline position at clamp01(elapsed / TIME) goes to the record's +0x64 (read by nothing);
//     a SAND record moves its slot to state 1 (first step) then 2 and copies its OBB into the slot matrix.
//   - SandTank vf05 0x5ecb20 (FX manager, every frame): per live slot the wall's scale (OBB extents x 0.8) and
//     position (OBB centre, y 0), state 1 samples every ring point's ground height and colour and rebuilds the wall,
//     state 2 refreshes one ring point per frame; the camera's distance to the nearest wall edge (or inside) gives the
//     walls' alpha and a fog floor for the weather (+0x120 / +0x11c, applied by 0x5bba30 as max(fog, it)).
//   - SandMesh 0x5ea170: a closed cylinder of PP5380 rings x PP5381 columns (0x5ea2e0), vertices 0x5ea870, scrolls
//     0x5eb590, drawn by 0x5ea5f0 with SubCode 0x14 (sand.vhl vh_sandstorm / sand.phl ph_sandstorm, FX_SANDSTORM01).
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace fuel::sand {

using V3 = std::array<float, 3>;
using V4 = std::array<float, 4>;
// FUEL's OBB (0x743210 output, 0x40 B): rows 0..2 = the unit axes with the centre in the 4th column ([3], [7], [11]),
// row 3 = the half extents and their length ([12..14], [15]).
using Obb = std::array<float, 16>;

// 0x7375d0 Node_CollisionBoxObb: C = box (CollisionVol_Z box 0, row vectors) x the node's world matrix (0x5857b0), then
// 0x743210 with the unit OBB (identity axes, centre 0, extents 0.5): axes = C's normalised rows, centre = C's
// translation, extents = 0.5 x C's row lengths.
Obb collisionBoxObb(const std::array<float, 16>& box, const std::array<float, 16>& nodeWorld);
bool obbContains(const Obb& o, double x, double y, double z);  // 0x75fb20 point-in-OBB (|local_i| <= h_i)

// pp001.tsc rows the sand tank and its meshes read (g_pp001_params[PP n]).
struct Params {
    float rings = 8, columns = 60;          // PP5380 / 5381 (ftol in 0x5ea2e0)
    float bulge = 0.15f, bulgeStart = 0.4f;  // PP5382 / 5383: radius x (1 + bulge sin(pi (start + (1 - start) f)))
    float uvDiffuse[2] = {5, 1.1f};          // PP5384 / 5385: C_USER0.xy
    float scroll0 = 0.01f;                   // PP5386: mesh +0x8c += it dt (C_USER0.z)
    float colourBase = 0, colourSlope = 1.5f;  // PP5387 / 5388: vertex colour x (base + slope f)
    float height = 180, sink = 20;           // PP5389 / 5390: y = ground + height f - sink
    float fogFar = 250, fogNear = 20;        // PP5392 / 5393: fog floor ramp (edge distance)
    float uvNormal[2] = {4, 0.6f};           // PP5394 / 5395: C_USER1.xy
    float scroll1 = -0.05f;                  // PP5396: mesh +0x90 += it dt (C_USER1.w)
    float alphaFar = 100, alphaNear = 10;    // PP5397 / 5398: wall alpha ramp (edge distance)
    float fogMax = 1;                        // PP5399: the fog floor inside / at the edge
    static Params fromPP(const std::vector<float>& pp);
};

// SandStormVertex_Z (0x4c B): position (w 1), normal + u, tangent + v, colour, binormal.
struct Vertex {
    float pos[4];
    float normal[3];
    float u;
    float tangent[3];
    float v;
    float colour[4];
    float binormal[3];
};
static_assert(sizeof(Vertex) == 0x4c, "SandStormVertex_Z is 0x4c bytes");

// The world 0x5ecb20 samples (G2 +0x4b8 loaded): the ground height 0x63eb10 + 0x63ea40 / 0x6407e0 at (x, z), and the
// ground colour band 0x640760 -> 0x63ecf0 (point context +0x70, rgba).
struct Ground {
    bool loaded = false;
    std::function<float(float x, float z)> height;
    std::function<V4(float x, float y, float z)> colour;
};

// SpecialEffectsSandMesh_G (0xc4 B).
class Mesh {
public:
    explicit Mesh(const Params& p);  // ctor 0x5ea170 + vf02 0x5ea2e0 (index list, buffers)
    // 0x5ea870(a, b): rebuilds the vertices of columns a..columns-1 (all rings), refreshes the ring-0 points' x / z of
    // those columns, and "uploads" columns [a, a + b) (the drawn copy).
    void fill(int a, int b);
    void update(float dt);  // vf03 0x5eb590: the two uv scrolls; the first call fills every column
    int rings() const { return rings_; }
    int columns() const { return cols_; }
    const std::vector<Vertex>& vertices() const { return drawn_; }  // the vertex buffer (column-major: column x rings + ring)
    const std::vector<Vertex>& cpu() const { return cpu_; }         // +0x38
    static std::vector<uint16_t> indices(int rings, int columns);    // 0x5ea2e0: 6 per cell, the columns wrap
    std::vector<V3> points;          // +0x84: ring-0 positions (x, z local) and their ground height (y)
    std::vector<V4> colours;         // +0x88: the columns' ground colours
    Obb matrix{};                    // +0x44: the slot OBB with the extents x 0.8; +0x74..+0x80 = matrix[12..15]
    float scroll0 = 0, scroll1 = 0;  // +0x8c, +0x90
    float edge = 0;                  // +0x94: camera distance to the wall's edge (10000 inside); the draw's sort key
    float alpha = 0;                 // +0x98: C_USER3.x
    bool first = false;              // +0xb8
    int next = 0;                    // +0xbc: the ring point state 2 refreshes next
    std::vector<V3> instances;       // +0x14 / +0x24: one position per live slot each frame (0x5de5b0 / 0x5de470)

private:
    Params p_;
    int rings_ = 0, cols_ = 0;
    std::vector<Vertex> cpu_, drawn_;
};

// SpecialEffectsSandTank_G (weather +0x14c, ctor 0x5ec020): the wall slots of the STARTFX SAND records.
class Tank {
public:
    explicit Tank(const Params& p) : p_(p) {}
    int addSlot();                    // 0x5ec1d0: count++, appends a slot (new SandMesh, state 0, identity matrix)
    void removeSlot(int slot);        // 0x5ec290: count--, state 0, its mesh deleted; count 0 empties the slot arrays
    // vf05 0x5ecb20 then the meshes' vf03 0x5eb590 (the FX manager's order, 0x5f62c0). `camera` = the tank position
    // (+0x14..+0x1c).
    void update(float dt, const V3& camera, const Ground& ground);
    int count() const { return count_; }
    size_t slots() const { return meshes_.size(); }
    int& state(int slot) { return states_[size_t(slot)]; }      // +0xa4[slot]
    int state(int slot) const { return states_[size_t(slot)]; }
    Obb& matrix(int slot) { return matrices_[size_t(slot)]; }   // +0x9c[slot] (0x5ed740: rows 0-2 identity, row 3 = 1)
    Mesh* mesh(int slot) const { return size_t(slot) < meshes_.size() ? meshes_[size_t(slot)].get() : nullptr; }
    // The weather's fog floor (+0x120) and its flag (+0x11c): set every frame while a slot exists, cleared by the reader.
    float fog = 0;
    bool fogSet = false;
    float minEdge = 10000;  // the frame's D (camera to the nearest wall edge, 0 inside)
    bool inside = false;

    // 0x5ec9a0 / 0x5eca40 (exposed for the checks).
    static bool insidePolygon(const V3& camera, const std::vector<V3>& pts, int n, const Obb& m);
    static float segmentDistance(const V3& p, const V3& a, const V3& b);

private:
    Params p_;
    int count_ = 0;                          // +0x94
    std::vector<Obb> matrices_;              // +0x98 / +0x9c
    std::vector<int> states_;                // +0xa0 / +0xa4
    std::vector<std::unique_ptr<Mesh>> meshes_;  // +0xa8 / +0xac
};

// One STARTFX record (0x70 B, WeatherManager +0x130 / +0x134).
struct Record {
    int type = 0;            // +0x00: 1 = SAND
    int slot = -1;           // +0x04: the sand-tank slot
    bool dead = false;       // +0x08
    Obb obb{};               // +0x0c: G's OBB (0x7375d0)
    float elapsed = 0;       // +0x4c
    float duration = 120;    // +0x50: TIME of the spline node (atof), default 120 s
    uint32_t node = 0;       // +0x54: G
    bool spline = false;     // +0x5c: the spline handle triple resolved (C holds a Spline_Z)
    V3 centre{};             // +0x64: the OBB centre (the spline position at elapsed / duration is written here and never read)
};

// What a STARTFX action hands over for one Spline_Z child C (0x570ae0 at 0x570e50..0x570ecf).
struct Fire {
    uint32_t node = 0;       // G (the CollisionVol_Z node below C)
    bool spline = true;      // 0x51b4e0 found C's Spline_Z (always true on this path)
    bool hasTime = false;    // C's UserDefine has TIME
    float time = 0;          // its atof value
    bool sand = false;       // G's UserDefine has SAND
    Obb obb{};               // 0x7375d0 on G
};

class StartFx {
public:
    explicit StartFx(const Params& p) : tank_(p) {}
    void fire(const Fire& f);  // 0x5bc340 (find by node, else append 0x5bc520) + 0x5c0710 + 0x5c0850
    void release(uint32_t node);  // 0x5bc290 (EvenementVolAgent_G vf25 0x570780): the first record of `node`
    // 0x5bc3b0: drop the dead records (from the back), then step each (0x5c08b0). `hidden(node)` = G's node flag 0x40000.
    void step(float dt, const std::function<bool(uint32_t)>& hidden);
    void dropHidden(const std::function<bool(uint32_t)>& hidden);  // [port] the first half of 0x5c08b0 + 0x5bc3b0 at once
    const std::vector<Record>& records() const { return records_; }
    Tank& tank() { return tank_; }
    const Tank& tank() const { return tank_; }

private:
    void removeAt(size_t i);  // 0x5c0a00 (SAND: slot removed 0x5ec290) + 0x5bc4a0
    std::vector<Record> records_;
    Tank tank_;
};

}  // namespace fuel::sand
