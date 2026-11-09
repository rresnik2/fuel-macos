// Free roam in the hubs: putting the player on a hub point (nudged to a clear spot when the
// point lies in an object or on a cliff) and the --hub-check report.
#pragma once

#include "core/hub_zones.hpp"
#include "core/hubs.hpp"
#include "core/terrain.hpp"
#include "drive.h"

struct HubSpawn {
    float x = 0, z = 0, yaw = 0;
    bool nudged = false;
    float moved = 0;           // metres from the point
    std::string problem;       // why the point itself failed ("" = it was fine); also set when no spot was found
    bool found = true;         // false: nothing usable within the search radius, the point is used as it is
};

// Why the spawn at (x, z) is no good ("" = fine): under the terrain, on a cliff, inside a placed object, or the
// vehicle does not settle (wheels off the ground, tipped over, sliding) in 1.5 s of physics. Leaves the player there.
std::string spawnProblem(DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain, float x,
                         float z, float yaw);

// Puts the player on `p`, or on the nearest clear spot when `p` is not usable, and streams the world there.
HubSpawn placeAtHubPoint(DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain,
                         const fuel::HubPoint& p);

// `fuelview --hub-check`: every hub's camp, teleports and vistas, after the hub-presence case. Returns the exit code.
int runHubCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                const fuel::Heightfield& terrain, const fuel::HubZones* zones = nullptr);

// PK03: the hub presence and arrival (fuel::HubPresence) for every hub, headless: the hub record (volume, boundary,
// END1), a drive from outside into the camp (enter vf46 -> TeleportToHub -> StartHubHUD -> 0x481d70) for an opened, a
// first-time and a locked hub, the camp menu's Back (0x47f060 -> END1), leave / re-enter, and the zones' hub names.
// Returns the number of failures.
int runHubPresenceCheck(const std::vector<fuel::Hub>& hubs, const fuel::HubZones* zones);

// `fuelview --collision-check` (P50): the collision world by class at `spots` and the hub points inside solid bodies.
int runCollisionCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                      const fuel::Heightfield& terrain, const std::vector<std::array<float, 2>>& spots);
