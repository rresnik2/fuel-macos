// The rider in the viewer: bakes the pilot for a vehicle's archetype, registers
// it with the renderer and hands back the mesh to place on the vehicle.
#pragma once

#include "core/rider.hpp"
#include "renderer.h"

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

class RiderView {
public:
    // `pilot` (PILOT.DPC) must be in the renderer's resource library.
    RiderView(const fuel::DpcFile& pilot, std::string vehiclesDefText, fuel::MeshRenderer& renderer)
        : rider_(pilot), tsc_(std::move(vehiclesDefText)), renderer_(renderer) {}

    // The rider's mesh IDs for `variant` (one, in vehicle space; none when it
    // has no archetype or the rider cannot be built). Archetypes are baked once.
    std::vector<uint32_t> parts(const std::string& variant) {
        const std::string arch = fuel::vehicleArchetype(tsc_, variant);
        if (arch.empty()) return {};
        if (auto it = keys_.find(arch); it != keys_.end()) return it->second ? std::vector<uint32_t>{it->second} : std::vector<uint32_t>{};
        const uint32_t key = fuel::asoboCrc("RIDER_" + arch);  // not a Mesh_Z ID
        uint32_t result = 0;
        try {
            const int slot = rider_.ridingSlot(arch);
            if (slot < 0) throw std::runtime_error("no riding pose for archetype " + arch);
            std::string err;
            if (renderer_.addBakedMesh(key, rider_.bake(slot), &err))
                result = key;
            else
                std::fprintf(stderr, "fuelview: rider: %s\n", err.c_str());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: rider for %s: %s\n", variant.c_str(), e.what());
        }
        keys_[arch] = result;
        return result ? std::vector<uint32_t>{result} : std::vector<uint32_t>{};
    }

private:
    fuel::Rider rider_;
    std::string tsc_;
    fuel::MeshRenderer& renderer_;
    std::map<std::string, uint32_t> keys_;  // archetype -> mesh key (0 = failed)
};
