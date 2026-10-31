// Reference captures: named spots rendered the same way every time, to set beside
// screenshots of the real game taken at the same spot (fuelview --capture-spot NAME,
// --capture-all DIR, --capture-find X,Z). The spot list is a TSV kept with the
// reverse-engineering notes (~/re-work/fuel/notes/capture-spots.tsv), not in the repo;
// the procedure for the real game is ~/re-work/fuel/notes/capture-checklist.md.
#pragma once

#include "core/roads.hpp"
#include "core/terrain.hpp"
#include "drive.h"
#include "renderer.h"

#include <string>
#include <vector>

struct CaptureSpot {
    std::string name;
    double x = 0, z = 0;    // where the vehicle stands (FUEL's own x, z in metres)
    float headingDeg = 0;   // vehicle heading: 0 faces +z, 90 faces +x (FUEL's TeLePort third argument, in degrees)
    float pitchDeg = -1;    // camera pitch above the horizon; < 0 ("chase"): the port's chase camera at rest
    float distance = -1;    // camera distance from the vehicle in metres; < 0 ("chase"): the chase camera
    std::string timeOfDay;  // HH.MM as FUEL's ForceDayTime takes it; the port's sun is fixed, so it is only reported
    std::string notes;
};

// $FUEL_CAPTURE_SPOTS, else ~/re-work/fuel/notes/capture-spots.tsv.
std::string defaultCaptureSpotsPath();
// Tab-separated: name, x, z, yaw, pitch, distance, time of day, notes; '#' lines and a
// header line starting with "name" are skipped. Throws std::runtime_error on a bad line.
std::vector<CaptureSpot> loadCaptureSpots(const std::string& path);
// Case-insensitive; nullptr when there is none.
const CaptureSpot* findCaptureSpot(const std::vector<CaptureSpot>& spots, const std::string& name);

struct CaptureView {
    fuel::OrbitCamera cam;
    simd_float3 focus = {0, 0, 0};    // MeshRenderer::setFocus
    simd_float3 vehicle = {0, 0, 0};  // where the vehicle came to rest
    float ground = 0;                 // port terrain height under it
    std::string report;               // one line for the log
};

// Puts the vehicle on the spot, lets it settle for 2 s with the brakes on, waits for every
// streaming job and rebuilds the world there, and returns the camera to render with
// (vehicle part transforms are already handed to the renderer).
CaptureView prepareCapture(const CaptureSpot& spot, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                           const fuel::Heightfield& terrain);

// --capture-find X,Z: helps pick spots near (x, z): the nearest drawn roads (point, direction,
// type) and placed objects whose base is off the port's terrain.
void printCaptureCandidates(double x, double z, double radius, const WorldStreamer& streamer, const fuel::RoadNetwork* roads,
                            const fuel::Heightfield& terrain);
