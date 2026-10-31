// The road-type fields the height, AI and race code use, taken from the one emulation of
// AGW_TerraRoad::Init 0x6f24a0 in road_table.* (via loadRoadTypeTable: the player's FUEL.exe,
// later-init values). p32 unified the two readers: this file used to run its own x86/x87
// interpreter over the same code; both produced bit-identical fields (checked on the user's exe).
#include "road_types.hpp"
#include "road_segments.hpp"
#include "genworld_noise.hpp"

#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace fuel {

std::optional<std::array<RoadTypeParams, 21>> loadRoadTypes(const DpcFile& world, std::string* error) {
    static std::mutex mu;
    static std::unordered_map<std::string, std::pair<std::optional<std::array<RoadTypeParams, 21>>, std::string>> cache;
    std::lock_guard<std::mutex> lk(mu);
    const std::string exe = engineExePath(world);
    auto it = cache.find(exe);
    if (it == cache.end()) {
        std::pair<std::optional<std::array<RoadTypeParams, 21>>, std::string> res;
        try {
            const RoadTypeTable T = loadRoadTypeTable(world);
            if (!T.valid) throw std::runtime_error("cannot rebuild the table from " + exe);
            std::array<RoadTypeParams, 21> t{};
            for (uint32_t k = 0; k < 21; ++k) {
                const RoadType& R = T.types[k];
                RoadTypeParams& q = t[k];
                q.reach = R.at(0x00), q.crown = R.at(0x24), q.verge = R.at(0x28), q.noise = R.at(0x34);
                q.fade = R.at(0x4c), q.ditch = R.at(0x50), q.ditchDepth = R.at(0x54), q.stretch = R.at(0x5c);
                q.bumpAmp = R.at(0x74), q.bumpLen = R.at(0x78), q.waveAmp = R.at(0x7c), q.waveLen = R.at(0x80);
                q.band = R.at(0xbc), q.core = R.at(0xc4), q.flat = R.at(0xcc);
                q.aiSpeedBlend = R.at(0xb4);
                q.smooth = R.at(0x1f0) * R.at(0x1e4), q.smoothMin = R.at(0x1e8);
                // Sanity: plausible metres for every road type (a different FUEL.exe fails here).
                auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
                if (!(in(q.reach, 1, 200) && in(q.band, 1, 200) && q.band <= q.reach && in(q.fade, 0.5f, 100) && in(q.flat, -50, 100) &&
                      in(q.crown, -5, 5) && in(q.verge, -5, 5) && in(q.noise, -5, 5) && in(q.ditch, 0, 50) && in(q.ditchDepth, 0, 10) &&
                      in(q.stretch, 0, 100) && in(q.waveLen, 0, 1000) && in(q.smooth, 0, 1000) && in(q.smoothMin, 0, 1000))) {
                    char m[200];
                    std::snprintf(m, sizeof m, "road type %u: implausible values (reach %g band %g fade %g flat %g)", k, q.reach, q.band, q.fade,
                                  q.flat);
                    throw std::runtime_error(m);
                }
            }
            res.first = t;
        } catch (const std::exception& e) {
            res.second = std::string("road table from FUEL.exe: ") + e.what();
        }
        it = cache.emplace(exe, std::move(res)).first;
    }
    if (error) *error = it->second.second;
    return it->second.first;
}

}  // namespace fuel
