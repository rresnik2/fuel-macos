// --camcheck: FUEL's camera and rider functions against test vectors from emulated runs of the original
// code (notes p25-camera-rider.md §P4, p27-camera-rider-port.md). The expected numbers are outputs of
// FUEL.exe's functions on made-up inputs (no game data).
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace fuel {

struct CamCheckResult {
    int total = 0, matched = 0;
    std::string report;
};

// Camera vectors; `extra` lets the caller add checks (the rider slot table needs the player's FUEL.exe):
// each returns {name, ok, detail}.
struct CamCheckItem {
    std::string name;
    bool ok = false;
    std::string detail;
};
CamCheckResult runCameraVectors(const std::vector<std::function<std::vector<CamCheckItem>()>>& extra = {});

}  // namespace fuel
