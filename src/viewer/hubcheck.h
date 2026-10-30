// Free roam in the hubs: putting the player on a hub point (nudged to a clear spot when the
// point lies in an object or on a cliff) and the --hub-check report.
#pragma once

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

// `fuelview --hub-check`: every hub's camp, teleports and vistas. Returns the exit code.
int runHubCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                const fuel::Heightfield& terrain);
