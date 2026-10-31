#include "capture.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <strings.h>

namespace {

std::vector<std::string> splitTabs(const std::string& line) {
    std::vector<std::string> out;
    std::string cell;
    std::istringstream in(line);
    while (std::getline(in, cell, '\t')) out.push_back(cell);
    return out;
}

std::string trim(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

// A number, or -1 for "chase", "-" or an empty cell.
float numberOrChase(const std::string& s, const std::string& what, int line) {
    const std::string t = trim(s);
    if (t.empty() || t == "-" || strcasecmp(t.c_str(), "chase") == 0) return -1;
    char* end = nullptr;
    const float v = std::strtof(t.c_str(), &end);
    if (end == t.c_str() || *end != '\0') throw std::runtime_error("capture spots line " + std::to_string(line) + ": bad " + what + " '" + t + "'");
    return v;
}

double number(const std::string& s, const std::string& what, int line) {
    const std::string t = trim(s);
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (t.empty() || end == t.c_str() || *end != '\0') throw std::runtime_error("capture spots line " + std::to_string(line) + ": bad " + what + " '" + t + "'");
    return v;
}

float slopeDeg(const fuel::Heightfield& t, double x, double z) {
    const float dx = (t.smoothHeightAt(x + 2, z) - t.smoothHeightAt(x - 2, z)) / 4.0f;
    const float dz = (t.smoothHeightAt(x, z + 2) - t.smoothHeightAt(x, z - 2)) / 4.0f;
    return std::atan(std::hypot(dx, dz)) * 57.29578f;
}

}  // namespace

std::string defaultCaptureSpotsPath() {
    if (const char* env = std::getenv("FUEL_CAPTURE_SPOTS"); env && *env) return env;
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "") + "/re-work/fuel/notes/capture-spots.tsv";
}

std::vector<CaptureSpot> loadCaptureSpots(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot read " + path);
    std::vector<CaptureSpot> spots;
    std::string line;
    for (int n = 1; std::getline(f, line); ++n) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trim(line).empty() || trim(line)[0] == '#') continue;
        const auto cells = splitTabs(line);
        if (strcasecmp(trim(cells[0]).c_str(), "name") == 0) continue;  // header
        if (cells.size() < 3) throw std::runtime_error("capture spots line " + std::to_string(n) + ": needs at least name, x, z");
        CaptureSpot s;
        s.name = trim(cells[0]);
        s.x = number(cells[1], "x", n);
        s.z = number(cells[2], "z", n);
        if (cells.size() > 3) s.headingDeg = float(number(cells[3], "yaw", n));
        if (cells.size() > 4) s.pitchDeg = numberOrChase(cells[4], "pitch", n);
        if (cells.size() > 5) s.distance = numberOrChase(cells[5], "distance", n);
        if (cells.size() > 6) s.timeOfDay = trim(cells[6]);
        if (cells.size() > 7) s.notes = trim(cells[7]);
        if ((s.pitchDeg < 0) != (s.distance < 0))
            throw std::runtime_error("capture spots line " + std::to_string(n) + ": give both pitch and distance, or 'chase' for both");
        spots.push_back(std::move(s));
    }
    return spots;
}

const CaptureSpot* findCaptureSpot(const std::vector<CaptureSpot>& spots, const std::string& name) {
    for (const CaptureSpot& s : spots)
        if (strcasecmp(s.name.c_str(), name.c_str()) == 0) return &s;
    return nullptr;
}

CaptureView prepareCapture(const CaptureSpot& spot, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                           const fuel::Heightfield& terrain) {
    const float heading = spot.headingDeg * float(M_PI) / 180.0f;
    drive.teleport(spot.x, spot.z, heading);
    streamer.update(drive.position(), renderer, drive.physics(), true);
    // 2 s on the handbrake (the brake pedal reverses once stopped): the vehicle drops onto the
    // ground and its suspension comes to rest.
    const fuel::DriveInput hold{0.0f, 0.0f, 0.0f, true};
    for (int i = 0; i < 120; ++i) {
        drive.update(1.0f / 60, hold);
        if (i % 10 == 0) streamer.update(drive.position(), renderer, drive.physics());
    }
    streamer.settle(drive.position(), renderer, drive.physics());
    renderer.setInstanceWorlds(drive.partWorlds());

    CaptureView v;
    v.vehicle = drive.position();
    v.ground = terrain.smoothHeightAt(v.vehicle.x, v.vehicle.z);
    ChaseCamera chase;  // the window's chase camera, at rest
    v.cam = chase.update(1.0f / 60, drive, renderer.sceneRadius());
    v.focus = chase.focus();
    if (spot.distance >= 0) {  // an explicit orbit behind the vehicle
        const float radius = std::max(renderer.sceneRadius(), 0.05f);
        v.cam.yaw = drive.heading() + float(M_PI);
        v.cam.pitch = spot.pitchDeg * float(M_PI) / 180.0f;
        v.cam.distance = spot.distance / radius;
    }
    const float dist = v.cam.distance * std::max(renderer.sceneRadius(), 0.05f);
    const simd_float3 eye = v.focus + dist * simd_make_float3(std::cos(v.cam.pitch) * std::sin(v.cam.yaw), std::sin(v.cam.pitch),
                                                              std::cos(v.cam.pitch) * std::cos(v.cam.yaw));
    char line[512];
    std::snprintf(line, sizeof line,
                  "capture %s: vehicle (%.1f, %.1f, %.1f) heading %.0f deg, %.2f m above the port terrain (%.1f, slope %.0f deg), %.1f km/h; "
                  "camera %s pitch %.1f deg, %.1f m, fov %.0f deg, eye ~(%.1f, %.1f, %.1f)%s",
                  spot.name.c_str(), v.vehicle.x, v.vehicle.y, v.vehicle.z, drive.heading() * 57.29578f, v.vehicle.y - v.ground, v.ground,
                  slopeDeg(terrain, v.vehicle.x, v.vehicle.z), drive.state().speedKmh, spot.distance >= 0 ? "orbit" : "chase", v.cam.pitch * 57.29578f,
                  dist, v.cam.fovY * 57.29578f, eye.x, eye.y, eye.z,
                  spot.timeOfDay.empty() ? "" : ("; time of day " + spot.timeOfDay + " not applied (the port's sun is fixed)").c_str());
    v.report = line;
    return v;
}

void printCaptureCandidates(double x, double z, double radius, const WorldStreamer& streamer, const fuel::RoadNetwork* roads,
                            const fuel::Heightfield& terrain) {
    std::printf("capture-find around (%.0f, %.0f), %.0f m: terrain %.1f m, slope %.0f deg\n", x, z, radius, terrain.smoothHeightAt(x, z),
                slopeDeg(terrain, x, z));
    if (roads) {
        struct Near {
            double d;
            size_t line;
            double px, pz, headingDeg;
        };
        std::vector<Near> near;
        const int32_t r = int32_t(std::ceil(radius / fuel::RoadNetwork::kCell));
        const int32_t cx = int32_t(std::floor(x / fuel::RoadNetwork::kCell)), cz = int32_t(std::floor(z / fuel::RoadNetwork::kCell));
        std::vector<uint32_t> seen;
        for (int32_t dz = -r; dz <= r; ++dz)
            for (int32_t dx = -r; dx <= r; ++dx) {
                auto it = roads->cells.find(fuel::RoadNetwork::cellKey(cx + dx, cz + dz));
                if (it == roads->cells.end()) continue;
                for (uint32_t li : it->second) {
                    if (std::find(seen.begin(), seen.end(), li) != seen.end()) continue;
                    seen.push_back(li);
                    const auto& L = roads->lines[li];
                    Near best{1e30, li, 0, 0, 0};
                    for (size_t k = 0; k + 1 < L.points.size(); ++k) {
                        const double ax = L.points[k][0], az = L.points[k][1], bx = L.points[k + 1][0], bz = L.points[k + 1][1];
                        const double ex = bx - ax, ez = bz - az, len2 = ex * ex + ez * ez;
                        const double t = len2 > 0 ? std::clamp(((x - ax) * ex + (z - az) * ez) / len2, 0.0, 1.0) : 0.0;
                        const double px = ax + ex * t, pz = az + ez * t, d = std::hypot(px - x, pz - z);
                        if (d < best.d) best = {d, li, px, pz, std::atan2(ex, ez) * 57.29578};
                    }
                    if (best.d <= radius) near.push_back(best);
                }
            }
        std::sort(near.begin(), near.end(), [](const Near& a, const Near& b) { return a.d < b.d; });
        if (near.size() > 8) near.resize(8);
        for (const Near& n : near) {
            const auto& L = roads->lines[n.line];
            const float width = roads->styles[L.type & 31].width;
            // The ground 8 m to either side of the road: towards heading h + 90 and h - 90
            // (heading h faces (sin h, cos h) in x, z).
            const double h = n.headingDeg / 57.29578, sx = std::cos(h), sz = -std::sin(h);  // direction of h + 90
            const float y0 = terrain.smoothHeightAt(n.px, n.pz);
            const float yp = terrain.smoothHeightAt(n.px + 8 * sx, n.pz + 8 * sz), ym = terrain.smoothHeightAt(n.px - 8 * sx, n.pz - 8 * sz);
            std::printf("  road line %zu type %d (%s, %.1f m wide): nearest (%.0f, %.0f) %.0f m away, runs at heading %.0f deg; "
                        "ground 8 m towards %.0f deg %+.1f m, towards %.0f deg %+.1f m\n",
                        n.line, int(L.type), fuel::isTarmacRoad(L.type) ? "tarmac" : "dirt", width, n.px, n.pz, n.d, n.headingDeg,
                        std::remainder(n.headingDeg + 90, 360.0), yp - y0, std::remainder(n.headingDeg - 90, 360.0), ym - y0);
        }
    }
    const auto gaps = streamer.groundGaps(x, z, radius, 0.5f);
    size_t floating = 0;
    for (const auto& g : gaps) floating += g.gap > 0;
    std::printf("  %zu placed objects more than 0.5 m off the port terrain (%zu above it, %zu below); highest above:\n", gaps.size(), floating,
                gaps.size() - floating);
    for (size_t i = 0; i < gaps.size() && i < 12 && gaps[i].gap > 0; ++i) {
        const auto& g = gaps[i];
        std::printf("    mesh %08x at (%.1f, %.1f, %.1f), %.1f m away: %+.2f m\n", g.meshId, g.position.x, g.position.y, g.position.z,
                    std::hypot(g.position.x - x, g.position.z - z), g.gap);
    }
}
