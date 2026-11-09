// P85: FUEL's visual vehicle damage on the drive session's vehicles (notes p85-damage.md):
//  - the morph damage of 0x5c7760 -> 0x6a2480 -> 0x7cb550 (core/vehicle_damage.hpp), rebuilt when an impact slot of a
//    vehicle moved by more than 0.01 and handed to the renderer as per-part morph VBs (MeshRenderer::setPartMorph);
//  - the engine smoke of 0x5cfda0 (FX_CAR_Damage01 from the largest impact slot 0.6, FX_CAR_Damage03 above 0.85).
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "core/vehicle_damage.hpp"
#include "core/vehicle_links.hpp"  // PK22

#include "core/particles.hpp"

class DriveSession;
class WorldStreamer;
namespace fuel {
class MeshRenderer;
struct Mesh;
}  // namespace fuel

class DamageView {
public:
    // Once per drawn frame, after the parts moved (main.mm showOpponents).
    void update(const DriveSession& drive, fuel::MeshRenderer& renderer);
    // P92: loads now what the first impact would load (FUEL.exe's damage tables, the vehicles' chassis meshes with
    // their morph tables): done at the first crash it stalled a frame ~55 ms. Call when the session's vehicles are set.
    // Also the crash debris items' Lod table and meshes (WorldStreamer::prewarmDebris).
    void prewarm(const DriveSession& drive, fuel::MeshRenderer& renderer, WorldStreamer* streamer);
    // For the checks: parts carrying a morph VB, vertices moved, and the last rebuild's vehicle.
    struct Stats {
        size_t rebuilds = 0, morphedParts = 0, movedVertices = 0;
        // PK22: V:DYN parts posed this frame, records broken loose (0x6a2480 > 1.6) and asking their IT_DEBRIS
        size_t dynPosed = 0, dynLoose = 0, dynDetached = 0, dirtPushes = 0;
    };
    const Stats& stats() const { return stats_; }
    // 0x5cfda0: the engine smoke level of vehicle i (0 none, 1 FX_CAR_Damage01, 2 FX_CAR_Damage03).
    int smokeLevel(int i) const { return size_t(i) < vehicles_.size() ? vehicles_[size_t(i)].smoke : 0; }
    // Every simulated frame (main.mm next to gStorms.update): the crash debris of 0x51e9e0 -> 0x52ee60 (pieces thrown
    // through WorldStreamer::throwDebris, removed by IT_DebrisMove's lifetime 0x533ec0) and the damage smoke's
    // emitters (Particles_Z FX_CAR_Damage01 / 03 of USA1 at the Engine bone).
    void step(float dt, const DriveSession& drive, WorldStreamer* streamer, const fuel::MeshRenderer& renderer);
    // Appends the damage smoke's particles to the placed ones (main.mm updatePlacedFx).
    void appendSmoke(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches) const;
    bool smoking() const {
        return std::any_of(vehicles_.begin(), vehicles_.end(), [](const Vehicle& v) { return v.smokeFx.emitter != nullptr; });
    }
    size_t debrisSpawned() const { return debrisSpawned_; }
    size_t debrisLive() const { return pieces_.size(); }
    size_t debrisRequests() const { return debrisRequests_; }
    std::vector<uint32_t> pieceIds() const {  // P87: the live crash-debris props (for the checks)
        std::vector<uint32_t> ids;
        for (const Piece& q : pieces_) ids.push_back(q.id);
        return ids;
    }  // P87: 0x51e9e0 -> 0x52ee60 calls (each 2..20 pieces)

private:
    struct Vehicle {
        std::array<float, 10> applied{};  // the copy of 0x5c7760 (+0x19c, impact slots)
        bool valid = false;
        int smoke = 0;  // +0x1b68
        uint32_t debrisSerial = 0;
        int smokeShown = 0;                 // the emitter running (level), 0 none
        fuel::particles::Instance smokeFx;  // stopped emitters keep stepping until their particles die
        // ---- PK22: the vehicle visual links (core/vehicle_links.hpp)
        bool dynInit = false;
        std::vector<fuel::vlinks::DynRecord> dyn;  // the skeleton's V:DYN records (Skel +0x148)
        float phase = 0;                           // veh+0x1084 (0x5cb8c0)
        float dirtPushed = 0;                      // veh+0x10a0 (0x5c9090)
        bool dirtSet = false;
        std::array<float, 4> lamps{-1, -1, -1, -1};
        // ---- PK22 end
    };
    // ---- PK22
    void stepLinks(float dt, const DriveSession& drive, const fuel::MeshRenderer& renderer);
    void updateLinks(const DriveSession& drive, fuel::MeshRenderer& renderer);
    void initLinks(Vehicle& v, const DriveSession& drive, int i);
    // The DYN_DAMAGE box of a part mesh (0x6a2480: vt+0x60 by CRC), cached; nullptr: none.
    const std::array<float, 16>* dynDamageBox(const fuel::MeshRenderer& renderer, uint32_t meshId);
    std::unordered_map<uint32_t, std::unique_ptr<std::array<float, 16>>> dynBoxes_;
    // ---- PK22 end
    struct Piece {  // a thrown IT_SmallDebris (0x533ec0 fields)
        uint32_t id = 0;
        float limit = 2.0f;  // +0xf40 (0x52ee60 sets 2)
        float age = 0;       // +0xeec
        float tick = 0;      // +0xf44
    };
    std::vector<Piece> pieces_;
    size_t debrisSpawned_ = 0;
    size_t debrisRequests_ = 0;
    uint32_t rng_ = 0x2545f491u;  // [port] Park-Miller stand-in for Rand01_ParkMillerShuffle 0x6460f0
    float rand01();
    std::shared_ptr<fuel::particles::Emitter> smokeEmitters_[2];
    bool smokeTried_ = false;
    const fuel::Mesh* mesh(const fuel::MeshRenderer& renderer, uint32_t id);
    fuel::damage::Tables tables_;
    bool tablesTried_ = false;
    uint32_t epoch_ = ~0u;
    std::vector<Vehicle> vehicles_;
    std::unordered_map<uint32_t, std::unique_ptr<fuel::Mesh>> meshes_;  // null: no morph tables
    Stats stats_;
};

extern DamageView gDamageView;
