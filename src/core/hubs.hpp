// The 19 hub regions for free roam: camp spawn, teleports and vista points
// (hubinfos.tsc SetHubTargetInfos, poi.tsc AddTypePointOfInterest, the
// USA1_REGION_HUB_* worlds). Shared contract between the hub loader (P18) and
// the menus (P19); see ~/re-work/fuel/notes/p18-hubs.md. Heights are not stored:
// callers take y from the terrain.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fuel {

struct HubPoint {
    std::string name;   // e.g. "Yellowstone_Teleport_3"
    float x = 0, z = 0;
    float yaw = 0;      // radians, forward = (sin, cos) in x, z
};

// PK03: one box of a hub volume: CollisionVol_Z box k x the volume node's world matrix (row vectors; OBB = normalised
// rows, extents = row lengths, centre = translation, as 0x7375d0 / 0x743210 build it).
struct HubBox {
    std::array<float, 3> centre{};
    std::array<std::array<float, 3>, 3> axis{};  // unit axes (world)
    std::array<float, 3> half{};                 // extents along them
    bool contains(float x, float y, float z) const;
};

struct Hub {
    std::string id;           // e.g. "YELLOWSTONE" (as hubinfos.tsc spells it, upper case)
    uint32_t nameText = 0;    // localized text id of the hub's name (0 = none; show `id`)
    int tier = 0;             // unlock order in the career (0 = first)
    HubPoint camp;            // where free roam in this hub starts
    std::vector<HubPoint> teleports;  // helicopter teleport targets
    std::vector<HubPoint> vistas;     // vista points
    // PK03: the rest of the hub record HubVolume_Init 0x577630 fills from the hub volume (USA1.DPC):
    std::vector<HubBox> volume;                    // the CollisionVol_Z: HubVolumeAgent_G enter vf46 0x577bf0 / leave vf44 0x577c90
    std::vector<std::array<float, 2>> boundary;    // H+0x40/+0x44: child HUB_BOUNDARIES' Spline_Z, 8 points per segment (x, z)
    HubPoint end1;                                 // H+0x18c/+0x194: child END1 (the camp exit, 0x47f060's TLP target)
    bool hasEnd1 = false;
};

// The hubs in career order. `dataDir` is the extracted game folder.
std::vector<Hub> loadHubs(const std::string& dataDir);

// The hub named `name` (compared without case, spaces or punctuation: "mount rainier",
// "MOUNT_RAINIER", "Mountrainier"); nullptr when none matches.
const Hub* findHub(const std::vector<Hub>& hubs, const std::string& name);
// The hub whose camp, teleports or vistas are nearest to (x, z).
const Hub* nearestHub(const std::vector<Hub>& hubs, float x, float z);
// Looks for a spot where `isClear` holds: (x, z) itself, then rings of growing radius
// (4 m apart, up to `maxRadius`). Returns false when there is none; otherwise moves x, z.
bool nudgeToClearSpot(float& x, float& z, const std::function<bool(float, float)>& isClear, float maxRadius = 120.0f);

// PK03 0x481a40: (x, z) inside the hub's HUB_BOUNDARIES polygon (0x7652b0 on H+0x40, the crossings of a ray toward -x,
// edges half-open in z). False when the hub has no boundary.
bool insideBoundary(const Hub& hub, float x, float z);

// PK03: the hub presence and arrival of FUEL's pause HUD (InGamePauseHUD, "player data block" 0x46dd80): +0x954 the
// pending hub record, +0x958 the hub-volume request, +0x944 the Map tab's state (0xc3 idle, 0xc4 StartHubHUD wait,
// 0xc5 opening a hub, 0xc7 TT_PAGE_HUB_ALERT), +0x94 the pause mode the menu was opened in (vf22 <- 0x46e650), and
// MissionStatus_G +0x374 (0x62f680) the zone the player is in. Hubs are indices into the `hubs` vector it was made with.
// Notes: ~/re-work/fuel/notes/pk03.md (b031, b033 sec. 6).
class HubPresence {
public:
    enum State { Idle = 0xc3, Wait = 0xc4, Open = 0xc5, Alert = 0xc7 };
    // What the host must do after update().
    struct Action {
        enum Kind { None, TeleportToHub, OpenMenu, OpenAlert } kind = None;
        int hub = -1;      // TeleportToHub: the hub (`hubs` index)
        int pauseMode = 1; // OpenMenu / OpenAlert: vf22's mode (vf09 = 1 offline)
    };
    // The career side: hub `hubs` indices.
    struct Hooks {
        std::function<bool(int)> unlocked;           // 0x579720
        std::function<bool(int)> opened;             // hub +0x248
        std::function<void(int)> arrive;             // 0x481d70: game+0x33c0 = uid, travel history 0x56ca20
        std::function<void(int)> open;               // 0x481cd0: +0x248 = 1, 0x57a100, 0x57a240, save flag +0x80
        std::function<void(double)> setDayClock;     // SetDayClock 0x46ab10 (PK04)
    };
    HubPresence(const std::vector<Hub>& hubs, Hooks hooks) : hubs_(hubs), career_(std::move(hooks)) {}

    // The player at (x, y, z) this frame: the hub volumes' enter (vf46 0x577bf0) / leave (vf44 0x577c90) events, then
    // MissionStatus_G vf17 0x62f680's zone (+0x374 = Zone_FindContaining 0x6ee810; `zone` is the caller's lookup).
    // `race`: a race runs (MissionStatus +0x48 not 0/1).
    void volumes(float x, float y, float z, bool race, int zone = -1);
    // 0x481b30, the Map tab's per-frame step (InGamePauseHUD vf02 0x46ea50 runs it while the menu is closed).
    // `movie`: a Bink plays (0x694290(0); always false: the port plays none); `key`: MENU_BACK / MENU_VALID this frame.
    Action update(float x, float z, bool race, bool movie = false, bool key = false);
    // StartHubHUD <uid> (0x5492d0), queued by TeleportToHub 0x5495a0 after the teleport: +0x954 = hub, +0x94 = 2,
    // 0x47d970 ("SetHudsVisibleState OFF", state 0xc4).
    void startHubHUD(int hub);
    // 0x46e650: the pause mode now (3 / 4 race offline / online, 2 in a pending hub's boundary (0x481a40), else 1).
    int pauseMode(float x, float z, bool race) const;
    // vf22 0x46e750: the menu opens; +0x94 = 0x46e650().
    void menuOpened(float x, float z, bool race) { menuMode_ = pauseMode(x, z, race); }
    // The menu closed. `back`: by MENU_BACK / START (0x46fd60, 0x46ea50): with +0x94 == 2 that is 0x47f060 (drop the
    // pending hub: TLP to its END1, returns true with the target); otherwise the close resets the tab (0x47ee80).
    bool menuClosed(bool back, float& tx, float& tz);
    // TT_PAGE_HUB_ALERT OK (0x4ce850 -> 0x47f060): the END1 target of the pending hub; false when there is none.
    bool end1(float& tx, float& tz) const;

    int pending() const { return pending_; }   // +0x954
    bool request() const { return request_; }  // +0x958
    int state() const { return state_; }       // +0x944
    int zone() const { return zone_; }         // MissionStatus +0x374
    bool inside(int hub) const { return hub >= 0 && size_t(hub) < inside_.size() && inside_[size_t(hub)]; }

private:
    bool inPendingBoundary(float x, float z) const;  // 0x481a40
    void drop();                                      // 0x47ee80 / 0x47f060's resets
    const std::vector<Hub>& hubs_;
    Hooks career_;
    std::vector<bool> inside_;  // per hub: the player overlaps its volume (the collision system's enter/leave state)
    int pending_ = -1;
    bool request_ = false;
    int state_ = Idle;
    int menuMode_ = 1;
    int zone_ = -1;
};

}  // namespace fuel
