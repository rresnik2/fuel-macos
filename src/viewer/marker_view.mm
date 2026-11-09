// P69: FUEL's race markers in the viewer (see marker_view.h).
#include "marker_view.h"

#include "core/assembly.hpp"
#include "core/race_rules.hpp"
#include "core/resources.hpp"
#include "core/road_graph.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

RaceMarkerView gRaceMarkers;

namespace {
std::string readText(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}
}  // namespace

void RaceMarkerView::init(const std::vector<const fuel::DpcFile*>& lookup, const std::string& dataDir, const fuel::RoadGraph* graph,
                          const std::array<fuel::RoadTypeParams, 21>* types, std::function<float(float, float)> ground) {
    lookup_ = lookup;
    ground_ = std::move(ground);
    for (const fuel::DpcFile* f : lookup_)  // the marker classes are USA1 Lod_Z objects named so (link name CRC)
        for (uint32_t id : f->ids(fuel::cls::Lod)) try {
                if (const fuel::DpcFile::Entry* e = f->find(id)) lodByName_.emplace(fuel::parseObjectLink(f->link(*e)).name, id);
            } catch (const std::exception&) {
            }
    namespace fs = std::filesystem;
    const fs::path story = fs::path(dataDir) / "LocalizedComponent/GameTsc/Story";
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"})
        for (fuel::Race& r : fuel::parseMissions(readText(story / file)))
            if (r.showStartEnd && (r.start[0] != 0 || r.start[2] != 0)) freeRoam_.push_back(std::move(r));
    // PK09: the whole POI list in FUEL's index order (hubinfos liveries + poi.tsc, sorted 0x735ac0): record key = index.
    if (!std::getenv("FUEL_NO_POI_MARKERS")) pois_ = fuel::worldPoiList(readText(story / "hubinfos.tsc"), readText(story / "poi.tsc"));
    ctx_.minHalf = fuel::loadMarkerMinHalf((fs::path(dataDir) / "CommonComponent/Param/pp001.tsc").string());
    if (graph && types) {
        ctx_.gateWidth = [graph, types](float x, float z) { return fuel::missionGateWidth(*graph, types->data(), types->size(), x, z); };
        ctx_.flagAt = [graph, types](int flag, float x, float z) {
            int rec = -1, k = 0;
            float qx = 0, qz = 0;
            int t = -2;  // none within 1000 m
            if (graph->nearestRoad(x, z, 1000.0f, rec, k, qx, qz) >= 0 && rec >= 0) t = int(graph->rec(size_t(rec)).type);
            const bool at = t == -2 && graph->roadAt(x, z, [types](int ty) { return ty >= 0 && ty < 21 ? (*types)[size_t(ty)].flat : 0.0f; }) >= 0;
            return fuel::checkpointMarkerFlag(flag, 0, t, at);
        };
    }
    ready_ = !std::getenv("FUEL_NO_RACE_MARKERS");
    if (std::getenv("FUEL_MARKER_TRACE"))
        std::printf("race markers: %zu free-roam missions with banners, %zu points of interest, PP360 %.1f, %s\n", freeRoam_.size(), pois_.size(),
                    ctx_.minHalf, lodByName_.count(fuel::asoboCrc("MISSIONSTARTMIDDLE")) ? "classes found" : "MISSIONSTARTMIDDLE not found");
}

const std::vector<fuel::PlacedMesh>& RaceMarkerView::classMeshes(const char* cls) {
    auto it = meshes_.find(cls);
    if (it != meshes_.end()) return it->second;
    std::vector<fuel::PlacedMesh> m;
    if (auto lod = lodByName_.find(fuel::asoboCrc(cls)); lod != lodByName_.end()) m = fuel::collectObjectMeshes(lookup_, lod->second);
    if (std::getenv("FUEL_MARKER_TRACE")) {
        std::printf("race markers: class %s: %zu meshes\n", cls, m.size());
        for (const fuel::PlacedMesh& p : m) std::printf("  mesh %08x (%zu levels)\n", p.meshId, p.levels.size());
    }
    return meshes_.emplace(cls, std::move(m)).first->second;
}

void RaceMarkerView::update(const fuel::Race* race, int target, simd_float3 player, fuel::MeshRenderer& r) {
    if (!ready_) return;
    // VisibleObjectMgr_G 0x4fc9d0: the records are rebuilt every frame; the port rebuilds when the set changes.
    std::string key;
    std::vector<size_t> near;
    if (race) {
        key = "race " + race->name + " " + std::to_string(target);
    } else {
        // 0x4f83d0: the missions in the grid cells within [0xa2a4e0] = 10000 m of the player (0x4edc70 square query).
        key = "free";
        for (size_t i = 0; i < freeRoam_.size(); ++i)
            if (std::fabs(freeRoam_[i].start[0] - player.x) <= 10000.0f && std::fabs(freeRoam_[i].start[2] - player.z) <= 10000.0f) {
                near.push_back(i);
                key += " " + std::to_string(i);
            }
    }
    // P79: DiscoveryPointVisibleObject 0x4ed2d0 (in races too): the points within 1000 m.
    std::vector<fuel::RaceMarker> poi;
    fuel::poiMarkers(pois_, player.x, player.z, poi);
    for (const fuel::RaceMarker& m : poi) key += " p" + std::to_string(m.index);
    if (key == key_) return;
    key_ = key;
    markers_.clear();
    if (race) {
        fuel::RaceMarkerContext c = ctx_;
        const int scenario = race->scenario;
        if (ctx_.flagAt)
            c.flagAt = [f = ctx_.flagAt, scenario](int flag, float x, float z) { return scenario == 11 ? 0 : f(flag, x, z); };
        else
            c.flagAt = [scenario](int flag, float, float) { return scenario == 11 ? 0 : flag; };
        fuel::missionMarkers(*race, 0, target, c, markers_);
    } else {
        for (size_t i : near) fuel::startBanners(freeRoam_[i], int(i), ctx_, markers_);
    }
    markers_.insert(markers_.end(), poi.begin(), poi.end());
    std::vector<fuel::PlacedMesh> draw;
    shown_ = 0;
    for (const fuel::RaceMarker& m : markers_) {
        // CHECKPOINT_FLAG_LIGHT: a light (Omni) and a glow mesh whose additive material the port's LOD path draws as an
        // opaque grey slab; kept as records, drawn only with FUEL_MARKER_LIGHTS=1 (approximation, notes M5).
        static const bool lights = std::getenv("FUEL_MARKER_LIGHTS") != nullptr;
        if ((m.type == 14 || m.type == 15) && !lights) continue;
        const char* cls = m.cls ? m.cls : fuel::raceMarkerClass(m.type);
        if (!cls) continue;
        const std::vector<fuel::PlacedMesh>& cm = classMeshes(cls);
        if (cm.empty()) continue;
        const float g = ground_ ? ground_(m.pos[0], m.pos[2]) : m.pos[1];
        std::array<float, 3> n{0, 1, 0};
        if (!m.upright && ground_) {  // the ground normal from the height field (central differences over 1 m)
            const float dx = ground_(m.pos[0] + 1, m.pos[2]) - ground_(m.pos[0] - 1, m.pos[2]);
            const float dz = ground_(m.pos[0], m.pos[2] + 1) - ground_(m.pos[0], m.pos[2] - 1);
            n = {-dx * 0.5f, 1.0f, -dz * 0.5f};
        }
        const std::array<float, 16> w = fuel::raceMarkerWorld(m, g, n);
        if (std::getenv("FUEL_MARKER_TRACE") && std::atoi(std::getenv("FUEL_MARKER_TRACE")) > 1)
            std::printf("  marker %2d %-24s #%d at (%.1f, %.1f, %.1f) heading (%.2f, %.2f)\n", m.type, cls, m.index, w[12], w[13],
                        w[14], w[8], w[10]);
        for (fuel::PlacedMesh p : cm) {
            p.world = w;
            draw.push_back(std::move(p));
        }
        ++shown_;
    }
    r.setLodObjects(draw, 2);
    if (std::getenv("FUEL_MARKER_TRACE"))
        std::printf("race markers: %s: %zu records, %zu objects, %zu meshes\n", race ? race->name.c_str() : "free roam", markers_.size(), shown_,
                    draw.size());
}
