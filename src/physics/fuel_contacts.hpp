// FUEL's engine-side rigid body and contacts (PhysicObj_Z / PhysicWorld_Z, FUEL.exe 0x68xxxx-0x72xxxx),
// decoded in ~/re-work/fuel/notes/p23-vehicle-physics.md (Part E) and p24-physics-port.md:
//   - the rigid body state and force accumulators (0x70fbb0, 0x70dc20, 0x70d690),
//   - contact records and their material block R[0x3c..0x5d] (E4),
//   - the penalty contact force 0x721d50 (E7), the tyre friction solver 0x720cc0 (7.2) and the generic
//     friction 0x7216c0 (E8),
//   - the wheel attach 0x722e60, the rigid impulse pass 0x683ee0 / 0x720b10 / 0x7204d0 (E9),
//   - sphere-vs-triangle 0x769560 with its prefilter 0x7260a0, the triangle record 0x768d50,
//   - the terrain collision triangles GenWorld_Z::CollectTriangles 0x77f510 (4 m cells of 3 x 3 altitude
//     samples, 2 or up to 8 triangles: U1 of the decode, resolved in P24).
#pragma once

#include "fuel_math.hpp"

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace fuel::fv {

// Contact material: R[0x3c..0x5d] of the contact record (copied from the wheel record +0x100..0x187 or the
// chassis block body+0x374).
struct ContactMat {
    float c = 0;            // 0x3c compliance: 0 rigid (impulse), 1 no normal force, else k = 1/c - 1
    float bias = 0.5f;      // 0x3d penetration bias gain (1/s)
    float maxDepth = 1e11f; // 0x3e beyond: bottom-out impulse
    float preload = 0;      // 0x3f
    float dampC = 1, dampR = 1, quadC = 1, quadR = 1;  // 0x40..0x43 (0x40 is also the impulse restitution)
    float set1[8] = {};     // 0x44..0x4b friction set 1 (longitudinal) c0..c7
    float set2[8] = {};     // 0x4c..0x53 friction set 2 (lateral)
    float maxN = 50;        // 0x54 max normal acceleration
    float loadFloor = 0, loadBlend = 0;  // 0x55, 0x56
    float extraDepth = 0;   // 0x57 added at attach
    float loadExp = 1;      // 0x58
    bool minFriction = true;
};

// One wheel record (0x1b0 bytes at [phys+0xd54] + 0x1b0 w), the fields the engine and the vehicle use.
struct WheelRec {
    // Body frame (relative to the centre of mass), written by the vehicle's wheel update 0x5213c0.
    Vec3 rimVelL, rimAccL, dirL, angL;  // +0x20, +0x40, +0x60, +0x80
    float invR = 1;                     // +0x8c
    // World frame (0x71c5f0).
    Vec3 rimVelW, rimAccW, dirW, angW;  // +0x30, +0x50, +0x70, +0x90
    Vec3 contactRimVel;                 // +0x10 rim surface velocity at the contact (0x722e60)
    Vec3 nDepth, fric, up, slip;        // +0xa0, +0xb0, +0xc0 (body up), +0xd0
    Vec3 hubL;                          // +0xe0 hub relative to the CoM, body frame
    float radius = 0.3f;                // +0xec sphere radius = proxy radius + P55|P56
    Vec3 hubW;                          // +0xf0
    ContactMat mat;                     // +0x100..+0x187
    float maxDepthSeen = 0;             // +0x188
    float load = 0;                     // +0x18c max projected compression ("wheel load")
    float util = 0;                     // +0x194 grip utilisation sumFn/sumDen
    float compRatio = 0;                // +0x198
    float sumFn = 0, sumDen = 0;        // +0x19c, +0x1a0
    bool tyreAxis = true;               // +0x1a5 friction along the wheel axis
    bool slidingDriven = false;         // +0x1a6
    uint8_t onRoad = 0;                 // +0x1a8
    bool active = true;                 // +0x1ac
    bool useTyre = true;                // +0x1ae tyre solver 0x720cc0
    int surface = -1;                   // last contact's surface (port bookkeeping)
};

struct BodyState {
    Quat q;
    Vec3 x, xComp, v, w, a, alpha;
    int nAux = 0;
    float auxPos[8] = {}, auxVel[8] = {}, auxDead[8] = {}, auxAcc[8] = {};
};

// The engine part of IT_Dyn_VehicleMove_G (PhysicObj_Z fields the contact code reads).
struct RigidBody {
    BodyState S, P, N;           // +0x7f0 state, +0x9d0 previous, +0xac0 new
    float mass = 1, invMass = 1; // +0xe20, +0xe1c
    Mat3 I1, I1inv, I2, I2inv;   // body-frame inertia sets 1 (forces) and 2 (impulses)
    Mat3 I1invW, I2invW;         // world (0x70d690)
    Vec3 F, T;                   // +0xbb0, +0xbc0 accumulators
    Vec3 avgV, avgW;             // +0x524.. averaged velocities of the last substep
    float maxSpeed = 83.34f;     // +0x558 (|v| and |w| clamp)
    float dv2 = 0.0025f, dw2 = 0.0025f;  // +0xe8c, +0xe88 integration deadbands
    float impulseThr = 0.001f;   // +0xea0
    float timeScale = 1;         // +0x548
    float boundR = 1;            // +0x408 bounding radius
    Vec3 meanNormal{0, 1, 0};    // +0x480
    int slidingContacts = 0;     // +0xde8
    float impactSpeed = 0;       // +0xe40 (max approach speed this frame)
    float scrapeSpeed = 0;       // port: max tangential speed of a chassis contact this frame

    void updateWorldInertia() {  // 0x70d690
        const Mat3 R = Mat3::fromQuat(S.q), Rt = R.transposed();
        I1invW = R * I1inv * Rt;
        I2invW = R * I2inv * Rt;
    }
    Vec3 pointVel(const Vec3& p) const { return S.v + cross(S.w, p - S.x); }  // 0x70ad50
    void addForceAt(const Vec3& f, const Vec3& p) {  // 0x70fbb0
        F += f;
        T += cross(p - S.x, f);
    }
};

// Contact record (0x18c bytes); indices of R are noted.
struct Contact {
    Vec3 p, n;                 // 0..7 (n towards body A)
    Vec3 surfVel;              // 8..0xb surface velocity of the other object
    Vec3 surfVelWheel;         // 0xc..0xf surface vel + rim velocity at the contact (wheel)
    float depth = 0;           // 0x36
    RigidBody* a = nullptr;    // 0x37
    RigidBody* b = nullptr;    // 0x38 (null = static world)
    WheelRec* wheel = nullptr; // 0x39
    int surface = 0;           // 0x3a
    ContactMat m;              // 0x3c..0x5d
    bool fromTriangle = false; // port bookkeeping
    // impulse cache (0x70ed40)
    Vec3 ra, rb, raXn, IraXn, rbXn, IrbXn;
    float effMass = 0, biasVel = 0, thr = 0.001f;
    bool impN = false, impF = false;
    float approach = 0, tangential = 0;  // 0x33, 0x34
};

// ---- integrator pieces (mode 3) ----------------------------------------------------------
// 0x7150a0: v += a h, w += alpha h, aux velocities += aux accelerations h.
void integrateVelocity(BodyState& S, float h);
// 0x714d20: x += v h with Kahan compensation when |v|^2 > dv2; q = dq (x) q (world-frame w, Hamilton)
// when |w|^2 > dw2; aux positions += vel h outside their deadbands.
void integratePosition(BodyState& S, float h, float dv2, float dw2);

// ---- friction and penalty forces --------------------------------------------------------

// Tyre friction 0x720cc0 (7.2): `u` is the tangential slip (in: original; the friction is accumulated into
// `friction`), `axis` the wheel forward. Returns the grip ratio (> 1 = sliding). Writes the wheel record's
// accumulators. `rigid` = R[0x3c] == 0.
float tyreFriction(const Vec3& axis, float Nf, const float set1[8], const float set2[8], bool rigid, WheelRec* rec, float depth,
                   const Vec3& u, Vec3& friction);
// Generic friction 0x7216c0 (E8).
float genericFriction(const Vec3& axisIn, float Nf, const float set1[8], const float set2[8], bool rigid, WheelRec* rec, float depth,
                      const Vec3& u, Vec3& friction);
// Penalty contact force 0x721d50 (E7): adds the spring/damper normal force and the friction to body A (and
// the opposite to B). No-op for rigid contacts (R[0x3c] == 0).
void penaltyForce(Contact& c);

// Wheel attach 0x722e60 (the "wheel" path, rec+0x1ad == 0, with rec+0x1a5 set): copies the tyre material,
// sets the rim velocity at the contact, moves the point to hub - n (r - depth), adds the extra depth and
// updates the wheel's depth/load accumulators.
void attachWheel(Contact& c, WheelRec& rec, const Vec3& bodyUp);

// ---- impulse pass (E9) -------------------------------------------------------------------
// Prepares a contact for the impulse pass (0x70ed40); returns true when it takes part.
bool prepareImpulse(Contact& c);
// One friction (0x7204d0) and normal (0x720b10) impulse; returns |dv| applied (for convergence).
void applyImpulses(Contact& c);

// ---- geometry ----------------------------------------------------------------------------
struct Tri {
    Vec3 v0, v1, v2;  // 0x4c, 0x5c, 0x6c
    Vec3 n;           // unit normal (v1-v0) x (v2-v0)
    float d = 0;      // n . v0
    Vec3 centroid;    // 0x3c..
    float boundR = 0; // max distance of a vertex from the centroid
    Vec3 e0, e1;      // v1-v0, v2-v0
    float a00 = 0, a01 = 0, a11 = 0;
    bool degenerate = false;
    int surface = 2;  // terrain triangles carry 2 (0x71dd80 call)
};
Tri makeTri(const Vec3& v0, const Vec3& v1, const Vec3& v2, int surface = 2);  // 0x768d50
// Prefilter 0x7260a0 + sphere-vs-triangle 0x769560: closest point on the triangle (two-sided); on a hit,
// depth = r - distance, n from the closest point to the centre, p = c - n r.
bool sphereTriangle(const Tri& t, const Vec3& c, float r, float& depth, Vec3& n, Vec3& p);

// The terrain's collision triangles, GenWorld_Z slot 0x88 = 0x77f510: cells of 4 m aligned to the grid;
// each cell samples the altitude at its 3 x 3 points (2 m apart, 0x63ea40 = the ground at integer
// coordinates); a border midpoint is "straight" when the angle test of the engine passes (cos > 0.9997);
// with all four borders straight the cell is 2 triangles (0,6,8), (0,8,2), else each border gives (a, c, b)
// or (a, c, m) + (m, c, b) around the centre c. Cached per cell like the engine's triangle cache 0x77e8d0.
class TerrainTriangles {
public:
    // `height(x, z)` at integer world coordinates (metres); `ox`, `oz` = the simulation's local origin.
    using HeightFn = std::function<float(double x, double z)>;
    explicit TerrainTriangles(HeightFn h) : height_(std::move(h)) {}
    void setOrigin(double ox, double oz) {
        ox_ = ox, oz_ = oz;
        cells_.clear();  // (the height samples are keyed by world coordinates and survive)
    }
    // Triangles of the cells overlapping the square around (cx, cz) (local coordinates) of half-size r.
    void query(float cx, float cz, float r, std::vector<const Tri*>& out);
    // Height of the triangulated surface under (x, z) (local); also the normal. False outside cached cells
    // (then it builds the cell).
    float surfaceHeight(float x, float z, Vec3* normal = nullptr);
    size_t cachedCells() const { return cells_.size(); }
    // Counters for the cost report: cells built and heights evaluated since construction.
    uint64_t cellsBuilt = 0, heightsEvaluated = 0;
    void trim(size_t maxCells);  // drops the oldest cells beyond maxCells

private:
    struct Cell {
        std::vector<Tri> tris;
        uint64_t used = 0;
    };
    const Cell& cell(int64_t ix, int64_t iz);
    float sample(int64_t x, int64_t z);
    HeightFn height_;
    double ox_ = 0, oz_ = 0;
    std::unordered_map<uint64_t, Cell> cells_;
    std::unordered_map<uint64_t, float> samples_;
    uint64_t tick_ = 0;
};

}  // namespace fuel::fv
