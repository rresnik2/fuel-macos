// FUEL's engine particles (Particles_Z: placed emitters for smoke, fire, dust, waterfalls...), decoded from
// FUEL.exe (notes p26-work/particles.md section 1, p48-effects2.md): the body grammar of 0x80bf20 / 0x80be10,
// the curve readers (0x7d20d0 and its 8-, 12- and 16-byte siblings), spawn 0x7d2fa0, update 0x7d2560, the
// distance fade 0x80c540 and the draw 0x841660 (particle.vhl vh_particle). The emitters and their curves come
// from the player's DPC files at run time.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>

#include "dpc.hpp"
#include <string>
#include <vector>

namespace fuel::particles {

using V3 = std::array<float, 3>;

// A curve: keys of (t, values...) with t normalised 0..1; flags bit 0x8 = step (no interpolation).
struct Curve {
    uint16_t flags = 0;
    int width = 1;               // values per key (after t)
    std::vector<float> keys;     // n x (1 + width)
    int count() const { return width > 0 ? int(keys.size()) / (1 + width) : 0; }
    // 0x7d20d0 (and siblings): from the cached 1-based segment index `idx` (only moves forward), linear between
    // the keys around t; past the last key, or before the segment's start, the last key; returns the new index.
    int eval(int idx, float t, float* out) const;
};

// One emitter element (0xa8 B in memory, 0x4c B fixed + 7 curves + Material_Z id in the file).
struct Element {
    uint16_t maxParticles = 0;   // +0x00
    int16_t rate = 0;            // +0x02 particles per second
    float duration = 10;         // +0x04
    float speed = 0;             // +0x08
    float spin = 0;              // +0x0c
    float offAxis = 0, offAxisSpread = 0;    // +0x10, +0x14
    float offPlane = 0, offPlaneSpread = 0;  // +0x18, +0x1c
    float unk20 = 0;             // +0x20
    float life = 1;              // +0x24
    float sizeX = 1, sizeY = 1;  // +0x28, +0x2c
    float wander = 0, wanderPeriod = 1, variation = 0;  // +0x30, +0x34, +0x38
    float gravity = 0;           // +0x3c (positive = down)
    float bounce = 1, friction = 1;  // +0x40, +0x44
    uint32_t flags = 0;          // +0x48: 1 loop, 2 / 4 orientation, 8 spacing, 0x40 / 0x80 shape, 0x200 local, 0x400 collide, 0x4000 wind
    Curve c4c, c58, c64, c70, c88, c94, c7c;
    uint32_t material = 0;       // Material_Z id (+0xa4)
};

struct Emitter {
    std::vector<Element> elements;
    float viewDist = 0, nearFade = 0;  // the fade of 0x80c540: view distance = link offset 88 (I), near fade 0
};
// Parses a Particles_Z body (throws on a short or malformed body). `link` = the object's link header.
Emitter parseParticles(const std::vector<uint8_t>& body, const std::vector<uint8_t>& link);

// One live particle (0x9c B record).
struct Particle {
    V3 pos{};           // [0..2]
    V3 vel{};           // [4..6]
    std::array<float, 4> wanderAxis{};  // [8..11]
    std::array<float, 3> colour0{1, 1, 1};  // [12..14]
    std::array<float, 4> colour{1, 1, 1, 1};  // [16..19] (+0x40)
    float gravity = 0, fall = 0;  // [20], [21]
    float angle = 0, spin = 0;    // [22], [23]
    float age = 0, life = 1;      // [24], [25]
    float sizeX = 1, sizeY = 1;   // [27], [28] base size
    float wanderAmp = 0, wanderFreq = 0;  // [29], [30]
    V3 render{};        // [31..33] (+0x7c) drawn position
    float drawX = 0, drawY = 0;   // [34], [35] (+0x88, +0x8c)
    int k64 = 1, k70 = 1, k7c = 1, k94 = 1;  // +0x90.. cached curve keys
};

// The running state of one element of a placed emitter (0x7d2560's param).
struct ElementState {
    float accum = 0;      // +8 birth accumulator
    float age = 0;        // +0xc
    float wind = 1;       // +0x18 wind factor (1 without wind)
    bool stopped = false; // +0x1c
    int k58 = 1, k4c = 1, k88 = 1;
    std::vector<Particle> live;
};

using Rnd = std::function<float()>;

// A placed instance: the node's world matrix (row-vector convention, translation in 12..14) and the scale.
struct Instance {
    const Emitter* emitter = nullptr;
    std::array<float, 16> world{};
    float scale = 1;
    std::vector<ElementState> state;
    bool warm = false;
    void reset() { state.assign(emitter ? emitter->elements.size() : 0, ElementState{}); }
};

// 0x7d2fa0 spawn + 0x7d2560 update of every element (world-space emitters; 0x200 local ones are kept in the
// emitter's space and moved with the matrix when drawn).
void step(Instance& inst, float dt, const Rnd& rnd);
// The direction 0x7d24f0: (cos b sin(-a), cos(-a), sin(-a) sin b).
V3 direction(float a, float b);

// ---- Placed emitters (Node_Z objects of class Particles_Z c56963de) ----------------------------------------
struct Placement {
    uint32_t nodeId = 0;
    std::array<float, 16> world{};
    float lodScale = 1;  // node +0x44 (the fade's scale)
    std::shared_ptr<const Emitter> emitter;
};
// Every Particles_Z node of `dpc` (USA1: only the nodes under the World_Z root, as collectWorldRootMeshes);
// nodes within 50 m of the origin (library copies) are skipped.
std::vector<Placement> collectPlacedParticles(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, bool worldRootOnly);

// What the renderer draws (vh_particle): one quad per live particle.
struct Quad {
    V3 pos{};
    float angle = 0;
    float sx = 0, sy = 0;
    int orient = 0;  // 0 camera-facing, 1 vertical (flags 4), 2 ground (flags 2)
    std::array<float, 4> rgba{};
};
struct Batch {
    uint32_t material = 0;
    uint32_t first = 0, count = 0;
};

class PlacedSystem {
public:
    void add(std::vector<Placement> p);
    size_t placements() const { return placed_.size(); }
    const Placement& placement(size_t i) const { return placed_[i].p; }
    size_t active() const { return active_; }
    size_t neverSleeping() const {  // PK29: placements whose emitter never sleeps (element flag 0x100)
        size_t n = 0;
        for (const Live& l : placed_) n += l.neverSleep;
        return n;
    }
    // Steps the emitters whose fade 0x80c540 (lodFade of the object's +0x20 / +0x24 at the node scale) is visible.
    void update(float dt, const V3& eye, float viewScale, float kFade);
    void collect(std::vector<Quad>& quads, std::vector<Batch>& batches) const;

private:
    struct Live {
        Placement p;
        Instance inst;
        float fade = 0;
        bool on = false;
        bool neverSleep = false;  // PK29: an element with flag 0x100 (0x677710 / 0x677520)
    };
    std::vector<Live> placed_;
    size_t active_ = 0;
    uint32_t rng_ = 0x2468ace1u;
};

}  // namespace fuel::particles
