// P94: the headless pop detector (see pop_check.h).
#include "viewer/pop_check.h"

#include <simd/simd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

namespace fuel {

namespace {

using Sample = MeshRenderer::PopSample;

enum Event : int { Appear, Vanish, Jump, LodSwitch, CellRule, DecorChunk, PathSwitch, MeshToCard, kEvents };
const char* const kEventNames[kEvents] = {"appear", "vanish", "alpha jump", "Lod_Z level", "FUEL cell", "decor chunk", "path switch", "mesh<->card"};
// FUEL pops these itself: Lod_Z::Draw switches levels without a blend (0x884170), and an AutoMesh spawn arrives when
// the camera's decor leaf over its cell first generates it (0x78cbf0: small objects only in 24 m leaves, objects
// without a view distance whenever their chunk passes), at whatever alpha its distance gives.
// A plant whose decor leaf changes size switches kinds at once too (7847b0 / 7880b0 rebuild the chunk's lists).
bool fuelOwn(int event) { return event == LodSwitch || event == CellRule || event == DecorChunk; }
const char* const kKindNames[Sample::kKinds] = {"placed", "automesh", "marker", "prop", "plant mesh", "plant card", "vehicle"};  // P104: vehicle
constexpr int kBins = 6;
const float kBinEdge[kBins] = {50, 100, 200, 400, 800, 1e9f};
const char* const kBinNames[kBins] = {"<50 m", "50-100", "100-200", "200-400", "400-800", ">=800"};

// One object across frames: LOD objects and props by their level-0 mesh and position (0.25 m), plants by position.
struct Key {
    const void* mesh0;
    int32_t x, y, z;
    bool operator==(const Key& o) const { return mesh0 == o.mesh0 && x == o.x && y == o.y && z == o.z; }
};
struct KeyHash {
    size_t operator()(const Key& k) const {
        size_t h = std::hash<const void*>()(k.mesh0);
        h ^= size_t(uint32_t(k.x)) * 0x9E3779B97F4A7C15ull;
        h ^= size_t(uint32_t(k.y)) * 0xC2B2AE3D27D4EB4Full + (h << 6);
        h ^= size_t(uint32_t(k.z)) * 0x165667B19E3779F9ull + (h >> 2);
        return h;
    }
};
struct State {
    Sample::Kind kind = Sample::Placed;
    const void* level = nullptr;
    float alpha = 0;       // LOD objects / props; plants: the mesh
    float cardAlpha = 0;   // plants: the card
    simd_float3 pos{};
    float radius = 0;
    bool plant = false;
    uint32_t object = 0;
    float view = 0;
    bool gated = false, lodRule = false;
};

// Props and race markers come and go with the game (debris thrown, a prop forgotten 300 m away, the next checkpoint's
// beacon): FUEL creates and deletes those agents too, so their appearing and vanishing is not counted.
bool isSpawn(int kind, int event) { return (kind == Sample::Prop || kind == Sample::Marker) && (event == Appear || event == Vanish); }

}  // namespace

struct PopCheck::Impl {
    Options o;
    std::unordered_map<Key, State, KeyHash> prev;
    bool havePrev = false;
    simd_float3 prevEye{};
    std::vector<Sample> samples;
    long frames = 0, cuts = 0, broken = 0;
    long counts[Sample::kKinds][kEvents] = {};
    long bins[kEvents][kBins] = {};
    FILE* csv = nullptr;
    struct Example {
        double t;
        int kind, event;
        float d, px, a0, a1;
    };
    std::vector<Example> examples;
};

PopCheck::PopCheck(const Options& o) : impl_(std::make_unique<Impl>()) {
    impl_->o = o;
    if (!o.csv.empty()) {
        impl_->csv = std::fopen(o.csv.c_str(), "w");
        if (impl_->csv) std::fprintf(impl_->csv, "t,kind,event,distance,pixels,alpha_before,alpha_after,x,y,z,object,view\n");
    }
}

PopCheck::~PopCheck() {
    if (impl_->csv) std::fclose(impl_->csv);
}

void PopCheck::frame(const MeshRenderer& renderer, const OrbitCamera& cam, double t) {
    Impl& m = *impl_;
    const simd_float3 eye = cam.eye;
    renderer.popProbe(eye, m.samples);
    std::unordered_map<Key, State, KeyHash> now;
    now.reserve(m.samples.size());
    for (const Sample& s : m.samples) {
        const bool plant = s.kind == Sample::PlantMesh || s.kind == Sample::PlantCard;
        const float q = plant ? 10.0f : 4.0f;
        Key k{plant ? nullptr : s.mesh0, int32_t(std::lround(s.pos.x * q)), int32_t(std::lround(s.pos.y * q)), int32_t(std::lround(s.pos.z * q))};
        if (s.kind == Sample::Vehicle) k.x = k.y = k.z = 0;  // P104: a vehicle / rider is one object wherever it drives
        State& st = now[k];
        st.pos = s.pos, st.plant = plant;
        if (plant) st.view = std::max(st.view, s.view);
        st.radius = std::max(st.radius, s.radius);
        if (plant) {
            (s.kind == Sample::PlantCard ? st.cardAlpha : st.alpha) = std::max(s.kind == Sample::PlantCard ? st.cardAlpha : st.alpha, s.alpha);
            if (s.kind == Sample::PlantMesh || st.level == nullptr) st.kind = s.kind, st.level = s.level;
        } else {
            st.kind = s.kind, st.level = s.level, st.alpha = std::max(st.alpha, s.alpha);
            st.object = s.object, st.view = s.view, st.gated = s.gated, st.lodRule = s.lodRule;
        }
    }
    static const char* dbg = std::getenv("FUEL_POP_DEBUG");  // "x,z,t0,t1": every sample within 6 m of (x, z) in that time
    if (dbg) {
        float dx = 0, dz = 0, t0 = 0, t1 = 0;
        if (std::sscanf(dbg, "%f,%f,%f,%f", &dx, &dz, &t0, &t1) == 4 && t >= t0 && t <= t1)
            for (const Sample& s : m.samples)
                if (std::hypot(s.pos.x - dx, s.pos.z - dz) < 6.0f)
                    std::printf("pop debug t %.3f kind %d mesh0 %p level %p pos (%.2f, %.2f, %.2f) alpha %.3f gated %d lod %d\n", t, int(s.kind), s.mesh0, s.level,
                                s.pos.x, s.pos.y, s.pos.z, s.alpha, int(s.gated), int(s.lodRule));
    }
    ++m.frames;
    // A camera cut (or the first frame): nothing to compare with.
    const bool cut = !m.havePrev || simd_distance(eye, m.prevEye) > 5.0f;
    m.cuts += cut && m.havePrev;
    if (!cut && t >= 1.0) {
        simd_float3 fwd = simd_normalize(cam.target - cam.eye);
        simd_float3 right = simd_normalize(simd_cross(fwd, simd_make_float3(0, 1, 0)));
        simd_float3 up = simd_cross(right, fwd);
        const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
        const float tanY = std::tan(fovY * 0.5f), tanX = tanY * float(m.o.width) / float(m.o.height);
        const float pxPerUnit = float(m.o.height) * 0.5f / tanY;
        auto inView = [&](const State& s, float& d, float& px) {
            const simd_float3 v = s.pos - eye;
            d = simd_length(v);
            const float z = simd_dot(v, fwd);
            if (z < -s.radius) return false;
            const float zz = std::max(z, 0.1f);
            if (std::fabs(simd_dot(v, right)) > zz * tanX + s.radius * 1.5f) return false;
            if (std::fabs(simd_dot(v, up)) > zz * tanY + s.radius * 1.5f) return false;
            px = 2.0f * s.radius / std::max(zz, 0.1f) * pxPerUnit;
            return px >= m.o.minPixels;
        };
        auto record = [&](const State& s, int event, float a0, float a1) {
            float d = 0, px = 0;
            if (!inView(s, d, px)) return;
            m.counts[s.kind][event]++;
            int b = 0;
            while (b < kBins - 1 && d >= kBinEdge[b]) ++b;
            m.bins[event][b]++;
            if (m.csv)
                std::fprintf(m.csv, "%.3f,%s,%s,%.1f,%.1f,%.3f,%.3f,%.1f,%.1f,%.1f,%08x,%.1f\n", t, kKindNames[s.kind], kEventNames[event], d, px, a0, a1, s.pos.x,
                             s.pos.y, s.pos.z, s.object, s.view);
            if (!fuelOwn(event) && !isSpawn(s.kind, event) && m.examples.size() < 40) m.examples.push_back({t, int(s.kind), event, d, px, a0, a1});
        };
        constexpr float kOn = 0.25f, kOff = 0.01f, kJump = 0.25f;
        // Props move: one that is gone from its key may be at a nearby position with the same mesh.
        // (World animations move placed objects too.) Only the few unmatched keys search.
        // P107: AutoMesh spawns never move (one that is knocked becomes a prop, set 3): no search. CR9-ISLA-SR has spawns of
        // one mesh 1.2 m apart (22baecf6); a new one fading in at alpha 0.01 was compared with its drawn neighbour (0.44)
        // and counted as an "alpha jump" (12 in the race).
        auto movedProp = [&](const Key& k, const State& s, const std::unordered_map<Key, State, KeyHash>& other) {
            if (s.kind == Sample::AutoMesh) return static_cast<const State*>(nullptr);
            for (const auto& [k2, s2] : other)
                if (k2.mesh0 == k.mesh0 && !(k2 == k) && !s2.plant && simd_distance(s2.pos, s.pos) < 3.0f) return &s2;
            return static_cast<const State*>(nullptr);
        };
        for (const auto& [k, s] : now) {
            const auto it = m.prev.find(k);
            const State* p = it != m.prev.end() ? &it->second : (!s.plant ? movedProp(k, s, m.prev) : nullptr);
            if (s.plant) {
                const float pm = p ? p->alpha : 0.0f, pc = p ? p->cardAlpha : 0.0f;
                if (p && p->view != s.view && s.view > 0 && (std::fabs(s.alpha - pm) >= kJump || std::fabs(s.cardAlpha - pc) >= kJump)) {
                    record(s, DecorChunk, std::max(pm, pc), std::max(s.alpha, s.cardAlpha));
                    continue;
                }
                const bool meshOn = pm < kOff && s.alpha >= kOn, meshOff = pm >= kOn && s.alpha < kOff;
                const bool cardOn = pc < kOff && s.cardAlpha >= kOn, cardOff = pc >= kOn && s.cardAlpha < kOff;
                if ((meshOff && cardOn) || (meshOn && cardOff)) record(s, MeshToCard, meshOff ? pm : pc, meshOff ? s.cardAlpha : s.alpha);
                else if (meshOn || cardOn) record(s, Appear, 0, std::max(s.alpha, s.cardAlpha));
                else if (meshOff || cardOff) record(s, Vanish, std::max(pm, pc), 0);
                else if (std::fabs(s.alpha - pm) >= kJump || std::fabs(s.cardAlpha - pc) >= kJump) record(s, Jump, std::max(pm, pc), std::max(s.alpha, s.cardAlpha));
                continue;
            }
            if (p == nullptr) {
                if (s.alpha >= kOn) record(s, Appear, 0, s.alpha);
                continue;
            }
            const bool emptyNow = s.lodRule && s.level == nullptr, emptyBefore = p->lodRule && p->level == nullptr;  // a level with no mesh
            if (p->alpha < kOff && s.alpha >= kOn) record(s, p->gated ? CellRule : emptyBefore ? LodSwitch : Appear, p->alpha, s.alpha);
            else if (s.gated && p->alpha >= kOn) record(s, CellRule, p->alpha, s.alpha);
            else if (emptyNow && p->alpha >= kOn) record(s, LodSwitch, p->alpha, s.alpha);
            else if (std::fabs(s.alpha - p->alpha) >= kJump) record(s, Jump, p->alpha, s.alpha);
            // A mesh change by Lod_Z::Draw's rule on both frames is FUEL's; any other (a placed object turned into a
            // prop drawn at a fixed level, say) is the port's.
            if (p->level != s.level && p->alpha >= kOn && s.alpha >= kOn) record(s, p->lodRule && s.lodRule ? LodSwitch : PathSwitch, p->alpha, s.alpha);
        }
        for (const auto& [k, p] : m.prev) {
            if (now.count(k)) continue;
            if (!p.plant && movedProp(k, p, now)) continue;
            // Breakable decor broken by a vehicle (0x51edb0 -> 0x515cc0): its pieces, props of other meshes, start here.
            if (!p.plant && p.kind != Sample::Prop) {
                bool broke = false;
                for (const auto& [k2, s2] : now)
                    if (s2.kind == Sample::Prop && !m.prev.count(k2) && simd_distance(s2.pos, p.pos) < 3.0f) broke = true;
                if (broke) {
                    ++m.broken;
                    continue;
                }
            }
            if (p.plant) {
                if (p.alpha >= kOn || p.cardAlpha >= kOn) record(p, Vanish, std::max(p.alpha, p.cardAlpha), 0);
            } else if (p.alpha >= kOn) {
                record(p, Vanish, p.alpha, 0);
            }
        }
    }
    m.prev = std::move(now);
    m.prevEye = eye;
    m.havePrev = true;
}

void PopCheck::covered() {  // PK15
    impl_->havePrev = false;
    impl_->prev.clear();
    ++impl_->frames;
}

long PopCheck::portPops() const {  // PK15
    long port = 0;
    for (int k = 0; k < Sample::kKinds; ++k)
        for (int e = 0; e < kEvents; ++e)
            if (!fuelOwn(e) && !isSpawn(k, e)) port += impl_->counts[k][e];
    return port;
}

bool PopCheck::finish() {
    Impl& m = *impl_;
    std::printf("\npop check: %s, %ld frames (%ld camera cuts), objects >= %.0f px tall on %dx%d\n", m.o.label.c_str(), m.frames, m.cuts, m.o.minPixels,
                m.o.width, m.o.height);
    std::printf("  %-12s", "kind");
    for (int e = 0; e < kEvents; ++e) std::printf(" %12s", kEventNames[e]);
    std::printf("\n");
    long port = 0, own = 0, spawns = 0;
    for (int k = 0; k < Sample::kKinds; ++k) {
        std::printf("  %-12s", kKindNames[k]);
        for (int e = 0; e < kEvents; ++e) {
            std::printf(" %12ld", m.counts[k][e]);
            (fuelOwn(e) ? own : isSpawn(k, e) ? spawns : port) += m.counts[k][e];
        }
        std::printf("\n");
    }
    std::printf("  by distance  ");
    for (int b = 0; b < kBins; ++b) std::printf(" %9s", kBinNames[b]);
    std::printf("\n");
    for (int e = 0; e < kEvents; ++e) {
        std::printf("  %-12s", kEventNames[e]);
        for (int b = 0; b < kBins; ++b) std::printf(" %9ld", m.bins[e][b]);
        std::printf("\n");
    }
    for (const auto& x : m.examples)
        std::printf("  e.g. t %.2f s %s %s at %.0f m (%.0f px), alpha %.2f -> %.2f\n", x.t, kKindNames[x.kind], kEventNames[x.event], x.d, x.px, x.a0, x.a1);
    const bool ok = port == 0;
    if (m.broken > 0) std::printf("  (%ld decor objects broken into pieces: not counted)\n", m.broken);
    std::printf("pop check: %ld pops FUEL would not have, %ld by FUEL's own rules (Lod_Z levels, AutoMesh cells), %ld props / markers spawned or "
                "removed by the game: %s\n", port, own, spawns, ok ? "ok" : "FAILED");
    return ok;
}

}  // namespace fuel
