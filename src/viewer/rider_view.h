// The rider in the viewer. Static scenes get the riding pose baked once per archetype (parts()); driven
// vehicles get their own animated rider (animated()): FUEL's rider state machine, layer mixer, sway spring,
// head levelling and IK (core/rider_anim.hpp), skinned on the CPU every frame into a per-vehicle mesh.
#pragma once

#include "core/assembly.hpp"
#include "core/rider.hpp"
#include "core/rider_anim.hpp"
#include "core/rider_tables.hpp"
#include "renderer.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class RiderView {
public:
    // `pilot` (PILOT.DPC) must be in the renderer's resource library; `exePath` = the player's FUEL.exe (the
    // slot table is replayed from it; without it the riders keep the static pose).
    RiderView(const fuel::DpcFile& pilot, std::string vehiclesDefText, fuel::MeshRenderer& renderer, const std::string& exePath = {})
        : rider_(pilot), tsc_(std::move(vehiclesDefText)), renderer_(renderer) {
        if (!exePath.empty()) {
            std::string err;
            if (auto t = fuel::loadRiderTables(exePath, &err)) tables_ = std::make_unique<fuel::RiderTables>(*t);
            else std::fprintf(stderr, "fuelview: rider animation off: %s\n", err.c_str());
        }
    }

    // The rider's mesh IDs for `variant` (one, in vehicle space; none when it has no archetype or the rider
    // cannot be built), in the static riding pose. Archetypes are baked once.
    std::vector<uint32_t> parts(const std::string& variant) {
        const std::string arch = fuel::vehicleArchetype(tsc_, variant);
        if (arch.empty()) return {};
        if (auto it = keys_.find(arch); it != keys_.end()) return it->second ? std::vector<uint32_t>{it->second} : std::vector<uint32_t>{};
        const uint32_t key = fuel::asoboCrc("RIDER_" + arch);  // not a Mesh_Z ID
        uint32_t result = 0;
        try {
            const int slot = staticSlot(arch);
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

    // A rider of its own for one driven vehicle (animated when the tables loaded, else the static pose).
    // `av`/`dpc`: the assembled vehicle and its DPC, for the IK_* points of its parts; `P` its VehiclePP block.
    std::vector<uint32_t> animated(const std::string& variant, const fuel::AssembledVehicle& av, const fuel::DpcFile& dpc,
                                   const std::vector<float>& P) {
        if (!tables_) return parts(variant);
        const std::string arch = fuel::vehicleArchetype(tsc_, variant);
        if (arch.empty() || fuel::riderArchetypeIndex(arch) < 0) return {};
        try {
            auto inst = std::make_unique<Instance>();
            inst->tmpl = &skinTemplate();
            inst->anim = std::make_unique<fuel::RiderAnimator>(rider_, *tables_, arch, vehicleClass(variant), P, ikPoints(av, dpc));
            if (!inst->anim->valid()) return parts(variant);
            const uint32_t key = fuel::asoboCrc("RIDER_" + arch + "_" + std::to_string(++serial_));
            fuel::BakedMesh mesh = inst->tmpl->mesh;
            rider_.skin(*inst->tmpl, inst->anim->boneWorlds(), mesh.vertices);
            std::string err;
            if (!renderer_.addBakedMesh(key, mesh, &err)) {
                std::fprintf(stderr, "fuelview: rider: %s\n", err.c_str());
                return {};
            }
            instances_[key] = std::move(inst);
            return {key};
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: rider for %s: %s\n", variant.c_str(), e.what());
            return parts(variant);
        }
    }

    fuel::RiderAnimator* animator(uint32_t key) {
        auto it = instances_.find(key);
        return it == instances_.end() ? nullptr : it->second->anim.get();
    }
    // Skins the rider `key` in its current pose and hands the vertices to the renderer.
    void upload(uint32_t key) {
        auto it = instances_.find(key);
        if (it == instances_.end()) return;
        Instance& in = *it->second;
        rider_.skin(*in.tmpl, in.anim->boneWorlds(), in.vertices);
        renderer_.updateBakedVertices(key, in.vertices);
    }

private:
    struct Instance {
        const fuel::Rider::SkinTemplate* tmpl = nullptr;
        std::unique_ptr<fuel::RiderAnimator> anim;
        std::vector<uint8_t> vertices;
    };
    int staticSlot(const std::string& arch) const {
        if (tables_) {  // the base riding loop (action 0) from FUEL's slot table
            const int a = fuel::riderArchetypeIndex(arch);
            if (a >= 0 && rider_.hasSlot(tables_->slots[size_t(a)][0])) return tables_->slots[size_t(a)][0];
        }
        return rider_.ridingSlot(arch);
    }
    const fuel::Rider::SkinTemplate& skinTemplate() {
        if (!tmpl_) tmpl_ = std::make_unique<fuel::Rider::SkinTemplate>(rider_.prepareSkin());
        return *tmpl_;
    }
    // VehicleInfo+0x38 class from AddVehicleInfo's second token (parser 0x56a580).
    int vehicleClass(const std::string& variant) const {
        std::istringstream in(tsc_);
        std::string line;
        const std::string want = "MESH_ID_" + variant;
        static const char* const kClasses[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK", "HYDRO", "DRAGSTER"};
        while (std::getline(in, line)) {
            std::istringstream ls(line);
            std::string a, b, c;
            ls >> a >> b >> c;
            if (a == "AddVehicleInfo" && b == want)
                for (int k = 0; k < 8; ++k)
                    if (c == kClasses[k]) return k;
        }
        return 0;
    }
    // IK_LEFTHAND / IK_RIGHTHAND / IK_LEFTFOOT / IK_RIGHTFOOT: named points of the vehicle's parts (Mesh_Z link
    // spheres), in vehicle space; NaN when the vehicle has none.
    static std::array<std::array<float, 3>, 4> ikPoints(const fuel::AssembledVehicle& av, const fuel::DpcFile& dpc) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        std::array<std::array<float, 3>, 4> out;
        for (auto& p : out) p = {nan, nan, nan};
        const uint32_t crc[4] = {fuel::asoboCrc("IK_LEFTHAND"), fuel::asoboCrc("IK_RIGHTHAND"), fuel::asoboCrc("IK_LEFTFOOT"),
                                 fuel::asoboCrc("IK_RIGHTFOOT")};
        for (const fuel::PlacedPart& part : av.parts) {
            if (!part.drawn() || part.shape != fuel::PartShape::Mesh) continue;
            const fuel::DpcFile::Entry* e = dpc.find(part.meshId);
            if (e == nullptr) continue;
            std::vector<fuel::NamedSphere> spheres;
            try {
                spheres = fuel::parseMeshLinkSpheres(dpc.link(*e));
            } catch (const std::exception&) {
                continue;
            }
            for (const fuel::NamedSphere& s : spheres)
                for (int k = 0; k < 4; ++k)
                    if (s.name == crc[k] && std::isnan(out[size_t(k)][0])) {
                        const auto& m = part.world;
                        const double x = s.centre[0], y = s.centre[1], z = s.centre[2];
                        out[size_t(k)] = {float(x * m[0] + y * m[4] + z * m[8] + m[12]), float(x * m[1] + y * m[5] + z * m[9] + m[13]),
                                          float(x * m[2] + y * m[6] + z * m[10] + m[14])};
                    }
        }
        return out;
    }

    fuel::Rider rider_;
    std::string tsc_;
    fuel::MeshRenderer& renderer_;
    std::unique_ptr<fuel::RiderTables> tables_;
    std::unique_ptr<fuel::Rider::SkinTemplate> tmpl_;
    std::map<std::string, uint32_t> keys_;  // archetype -> mesh key (0 = failed)
    std::map<uint32_t, std::unique_ptr<Instance>> instances_;
    int serial_ = 0;
};
