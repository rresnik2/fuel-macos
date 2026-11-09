// P44: FUEL's free-roam rules (core/freeroam.hpp) on the viewer's drive session: the frame inputs from the
// vehicle and camera, the AutoMesh fuel barrels from the world streamer, saving, the map's free-roam GPS.
#pragma once

#include "core/freeroam.hpp"
#include "core/hubs.hpp"
#include "drive.h"
#include "ui/menu.h"

#include <memory>
#include <string>

class FreeRoamView {
public:
    // `event`: the race being run ("" = free roam), `hub` / `vehicle`: the session's. Does nothing without a career.
    FreeRoamView(fuel::ui::Menu& menu, const std::string& event, const std::string& hub, const std::string& vehicle);
    ~FreeRoamView();
    // One frame. Returns true when barrels were taken (the caller refreshes the streamer so their meshes go).
    bool step(float dt, DriveSession& d, WorldStreamer* streamer, bool mission, const fuel::OrbitCamera& cam);
    fuel::FreeRoam* rules() { return freeRoam_.get(); }
    // P61: the career's uncaught AddHubTrafficVehicle entries become the traffic's special vehicles (0x5c5290); a catch
    // runs FreeRoam::catchTraffic (reward, save, message).
    void attachTraffic(class TrafficView& traffic);
    // P78: the special vehicles for the minimap icons (HudState::specials, 0x5c57d0 / 0x4ac1a0).
    void fillSpecials(fuel::ui::HudState& h) const;

    // ---- PK03: hub presence and arrival (fuel::HubPresence on the menu's hubs; notes pk03.md). step() runs the hub
    // volumes, the player's zone and the Map tab's per-frame step 0x481b30; the host acts on takeHubAction().
    void setZones(const fuel::HubZones* z) { zones_ = z; }
    fuel::HubPresence::Action takeHubAction() { auto a = hubAction_; hubAction_ = {}; return a; }
    // TeleportToHub 0x5495a0 put the player on `hub`'s camp: its queued StartHubHUD <uid> runs (0x5492d0).
    void hubTeleported(int hub);
    // A session started by TeleportToHub (free ride in a hub, not from a race end): StartHubHUD for the session's hub.
    void startSessionHub(const std::string& hub);
    void hubMenuOpened(float x, float z, bool race) { if (presence_) presence_->menuOpened(x, z, race); }
    // The menu closed (`back`: by Back / START); true with the END1 target when FUEL teleports there (0x47f060).
    bool hubMenuClosed(bool back, float& tx, float& tz) { return presence_ && presence_->menuClosed(back, tx, tz); }
    bool hubEnd1(float& tx, float& tz) const { return presence_ && presence_->end1(tx, tz); }
    const fuel::HubPresence* presence() const { return presence_.get(); }
    // SetDayClock 0x46ab10 (PK04 provides it; the host sets this).
    std::function<void(double)> setDayClock;

private:
    std::unique_ptr<fuel::HubPresence> presence_;
    fuel::HubPresence::Action hubAction_;
    const fuel::HubZones* zones_ = nullptr;
    fuel::ui::Menu& menu_;
    std::unique_ptr<fuel::FreeRoam> freeRoam_;
    int vehicleClass_ = -1;
    uint64_t barrelGen_ = ~0ull;
    class TrafficView* traffic_ = nullptr;
};

// The map target's free-roam GPS (MapHUD +0x60 -> 0x4a2250): trail and ribbons to the target, the route on the
// minimap, arrival within 20 m. Port: a straight route (FUEL snaps both ends to the highways, gps.md 1).
void freeRoamGps(fuel::ui::HudState& h, const DriveSession& d, float dt, const fuel::OrbitCamera& cam, fuel::ui::Menu* menu);
