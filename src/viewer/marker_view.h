// P69: FUEL's race markers in the viewer (core/race_markers.hpp; notes ~/re-work/fuel/notes/p69-objects-scripting.md M):
// the start / finish banners and the checkpoint flags VisibleObjectMgr_G (0x4fc9d0) keeps as game objects, drawn as
// their USA1 Lod_Z (renderer LOD set 2). The CHECKPOINT_FLAG classes share their Skin_Z mesh with the placed flags, so
// WorldAnimation's slot-0 loop animates them too (BaseSharingPlayAnimAgent_G, p47).
// P79: also the points of interest within 1000 m (DiscoveryPointVisibleObject 0x4ed2d0, notes p79-objects-move.md P3):
// the vista objects (IT_DISCOVERY) and the teleport / heliport markers (CAMIONHUB_RESPAWN). FUEL_NO_POI_MARKERS=1 hides them.
#pragma once

#include "core/dpc.hpp"
#include "core/race_markers.hpp"
#include "core/road_types.hpp"
#include "core/world.hpp"
#include "renderer.h"

#include <simd/simd.h>

#include <array>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel { class RoadGraph; }

class RaceMarkerView {
public:
    // lookup: where the world resolves ids (USA1 holds the marker Lods); the free-roam missions come from the scripts
    // (LoadMissionsTsc: miss_official.tsc, missChallenge.tsc); graph / types: gate widths and flag kinds (may be null).
    void init(const std::vector<const fuel::DpcFile*>& lookup, const std::string& dataDir, const fuel::RoadGraph* graph,
              const std::array<fuel::RoadTypeParams, 21>* types, std::function<float(float, float)> ground);
    bool ready() const { return ready_; }
    // A frame: `race` = the running mission (null in free roam), `target` = the player's gate in the lap.
    void update(const fuel::Race* race, int target, simd_float3 player, fuel::MeshRenderer& r);
    size_t shown() const { return shown_; }
    const std::vector<fuel::RaceMarker>& markers() const { return markers_; }

private:
    const std::vector<fuel::PlacedMesh>& classMeshes(const char* cls);
    std::vector<const fuel::DpcFile*> lookup_;
    std::unordered_map<uint32_t, uint32_t> lodByName_;
    std::unordered_map<std::string, std::vector<fuel::PlacedMesh>> meshes_;
    std::vector<fuel::PoiPoint> pois_;  // P79: poi.tsc's points (DiscoveryPointVisibleObject 0x4ed2d0)
    std::vector<fuel::Race> freeRoam_;  // missions with banners on (+0)
    fuel::RaceMarkerContext ctx_;
    std::function<float(float, float)> ground_;
    std::vector<fuel::RaceMarker> markers_;
    std::string key_;
    size_t shown_ = 0;
    bool ready_ = false;
};

extern RaceMarkerView gRaceMarkers;
