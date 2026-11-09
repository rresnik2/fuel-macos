// FUEL's free roam, the per-frame rules (P44, decoded in ~/re-work/fuel/notes/p44-freeroam.md):
//  - the world's points of interest (POI manager list, 0x4c-byte records): vista points (type 3, IT_DISCOVERY),
//    vehicle liveries (type 2, IT_LIVERY) and teleports / heliports (type 4), from the scripts (0x54f760, 0x5756e0);
//  - pickup by distance 0x4ed0a0 (a vista within 4 m is visited), livery pickup 0x4fcb60 (contact with the spawned
//    livery), fuel barrels (AutoMesh IT_Fuel01 / IT_Fuel02 = IT_DYNCOLLECT1/2_G, pickup 0x51d600);
//  - the free-roam screen icons 0x4c0260: vistas and liveries within 5000 m and on screen are revealed, the current
//    hub's challenges within 5000 m are discovered once the hub is opened;
//  - statistics: distance 0x569460, jumps / falls / air time 0x4c8450, tricks (the rider's big jump, 0x5d4a80),
//    race no-crash / no-GPS flags 0x632b50;
//  - MessagesHUD's queue (0x4af9d0 push, 0x4afb20 update, 0x4afe00 fade).
// The career state these change lives in Career (career.hpp).
#pragma once

#include "career.hpp"
#include "race_markers.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace fuel {

// MessagesHUD (vtable 0x9a5564): kinds 1..12 (pages 126..135, see notes F6), 5 s each, one at a time.
class MessageQueue {
public:
    struct Message {
        int kind = 0;
        int param = -1;
        float duration = 5.0f;  // 0x9dbf8c
    };
    void push(int kind, int param, float duration = 5.0f) { queue_.push_back({kind, param, duration}); }  // 0x4af9d0 / 0x4b05c0
    void update(float dt);   // 0x4afb20
    bool showing() const { return left_ > 0; }
    const Message& current() const { return cur_; }
    float alpha() const;     // 0x4afe00
    size_t pending() const { return queue_.size(); }
    // PK08 0x4af920: a MENU_VALID press while the shown message is in its steady phase (0.1 T < left < 0.9 T) jumps to its
    // fade-out (left = 0.1 T). True when it did.
    bool skip();
    void clear() { queue_.clear(), left_ = 0, cur_ = {}; }

private:
    std::deque<Message> queue_;
    Message cur_;
    float left_ = 0;  // +0x4ec (time left), cur_.duration = +0x4f0
};

// One fuel barrel AutoMesh spawned near the player (an IT_DYNCOLLECT agent).
struct FuelBarrel {
    float x = 0, y = 0, z = 0;
    float amount = 100;   // +0xa0: IT_DYNCOLLECT1_G 100, IT_DYNCOLLECT2_G 400
    float radius = 1;     // the template's bounding radius: the barrel body in the contact test [port: one sphere]
    bool taken = false;   // +0xa4
};

// PK09: a vehicle body the barrels can touch (0x531310: the contact node's +0xd8 & 0x20c): its collision shapes in
// world space. flags 0x4 = the player's vehicle (credits the fuel), 0x8 = an AI vehicle (takes the barrel without credit).
struct ContactBody {
    int flags = 0x4;
    std::vector<std::array<float, 4>> spheres;  // centre x, y, z, radius
    struct Box {
        std::array<float, 3> c{}, h{};
        std::array<std::array<float, 3>, 3> ax{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    };
    std::vector<Box> boxes;
};

struct FreeRoamFrame {
    float dt = 0;
    std::array<float, 3> pos{}, vel{};  // player vehicle (vehicle+0x820 velocity)
    bool onGround = true;     // some wheel with contact weight (0x52e620)
    float heightAbove = 0;    // the vehicle's height above the ground (rider R4)
    float airtime = 0;        // veh+0x19ec
    bool mission = false;     // a race object exists with state not 0/1 (race +0x48)
    bool pageShown = false;   // a full-screen page is up (IGP+0x6b84 page set +0x28 > 1e-6)
    bool offRoad = false;     // 0x6d1d70(pos) < 0
    bool wrecked = false;     // player phys +0x1420
    bool gpsShown = true;     // the GPS option ([0xa6536e] == 0)
    int vehicleClass = -1;    // 0x56a580 class of the player's vehicle (BIKE 1, QUAD 3 count tricks)
    // On-screen test of a world point (0x4ca890 projection + 0x4c10c0); null = never on screen.
    std::function<bool(float x, float y, float z)> onScreen;
    // minimap-fix: 0x4c0260's reference point, the camera (0x421980), and the hub whose zone holds it (-1 none: no
    // discovery, no reveal). Unset: the vehicle's position and the career's current hub (game +0x33c0), as before.
    bool haveEye = false;
    std::array<float, 3> eye{};
    int zoneHub = -2;
    // PK09: the vehicles' bodies this frame (the player's with flags 0x4, AI racers 0x8) for the barrel contacts.
    std::vector<ContactBody> bodies;
};

class FreeRoam {
public:
    // A POI record (0x4c bytes in FUEL): what the port needs of it.
    struct Poi {
        int type = 0;          // 2 vehicle livery, 3 vista point, 4 teleport (heliport)
        int hub = -1;          // +0x3c (uid = CareerRules hub index)
        int index = -1;        // the hub's livery / vista / teleport index (FUEL: +0x48 key)
        int order = -1;        // PK09: the record's place in the script append order (before the sort)
        float x = 0, z = 0;
        bool taken = false;    // +4
        std::string name;
    };

    // `startOf`: a mission's start (x, z) (the challenge record +0xc / +0x14); used for discovery.
    using StartOf = std::function<bool(const std::string& mission, float& x, float& z)>;
    FreeRoam(Career& career, StartOf startOf);

    // The barrels spawned around the player now. Barrels that were already spawned keep their taken flag;
    // others are new agents (0x51d540: taken = 0), as FUEL recreates them with their AutoMesh cell.
    void setBarrels(std::vector<FuelBarrel> now);
    const std::vector<FuelBarrel>& barrels() const { return barrels_; }
    // PK09: the POI list in FUEL's index order (append order, then 0x735ac0's sort): pois()[i] = FUEL's POI index i.
    const std::vector<Poi>& pois() const { return pois_; }

    void update(const FreeRoamFrame& f);

    MessageQueue& messages() { return messages_; }
    // P61: the player caught special traffic vehicle k of `hub` (0x5c53b0): the career reward, "SaveGameData" and the
    // message (8 vehicle, 9 liveries, 10 vistas, 11 challenges) for 5 s (0x4afa70(5.0)). False when already caught.
    bool catchTraffic(int hub, int k);
    // "SaveGameData" was issued since the last call.
    bool takeSaveRequest() { const bool s = save_; save_ = false; return s; }
    // Barrels picked up since the last call (indices into barrels()).
    std::vector<size_t> takePicked() { auto p = std::move(picked_); picked_.clear(); return p; }
    // Achievements awarded by free-roam progress since the last call.
    std::vector<int> takeTrophies() { auto t = std::move(newTrophies_); newTrophies_.clear(); return t; }

    // Jump tracker (0x4c8450, +0x38..+0x48) for the HUD / checks.
    struct Jump {
        bool tracking = false;
        float airTime = 0, length = 0, fall = 0, height = 0;
    };
    const Jump& jump() const { return jump_; }

    static constexpr float kPickupRadius = 4.0f;    // 0x4ed0a0: d^2 < 16 (0x9dbf50)
    static constexpr float kRevealRadius = 5000.0f; // 0x4c0260: d^2 < 2.5e7 (0x9dcfd0)

private:
    Career& career_;
    StartOf startOf_;
    std::vector<Poi> pois_;
    PoiGrid grid_;  // 0x4ecf00's grid over pois_
    std::vector<FuelBarrel> barrels_;
    std::vector<size_t> picked_;
    MessageQueue messages_;
    Jump jump_;
    bool trickArmed_ = true;  // the rider's +0x254 (one big jump per flight)
    bool save_ = false;
    bool mission_ = false;
    float trophyClock_ = 0;
    std::vector<int> newTrophies_;
};

}  // namespace fuel
