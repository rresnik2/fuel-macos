// The rider in the viewer. Static scenes get the riding pose baked once per archetype (parts()); driven
// vehicles get their own animated rider (animated()): FUEL's rider state machine, layer mixer, sway spring,
// head levelling and IK (core/rider_anim.hpp), skinned on the CPU every frame into a per-vehicle mesh.
#pragma once

#include "core/assembly.hpp"
#include "core/custom_rider.hpp"
#include "core/rider.hpp"
#include "core/rider_anim.hpp"
#include "core/rider_tables.hpp"
#include "renderer.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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

    // P66: the garage's rider (parts and liveries) for every rider built after this call [port: AI riders too].
    void setParts(const fuel::Rider::Parts& p) { parts_ = p, tmpl_.reset(), tmplLevels_.clear(), playerTmpl_.reset(); }
    // PHYSIQUE (port deviation): the player's outfit (core/custom_rider.hpp); the player's rider (animated(..., true)) wears
    // the custom body when the assets are there and the outfit is not 0. AI riders keep FUEL's.
    void setOutfit(const fuel::RiderOutfit& o) { outfit_ = o, playerTmpl_.reset(); }
    // PHYSIQUE (glutes): settings.txt `rider_jiggle` (fuel::riderJiggleSetting: off 0, subtle 0.5, full 1); the player's
    // springs scale it by the outfit's share. 0 = the plain skinning.
    void setJiggle(double setting) {
        jiggleSetting_ = setting;
        for (auto& [k, in] : instances_)
            if (in->player) in->jiggle.setAmplitude(jiggleAmplitude());
    }
    double jiggleAmplitude() const { return customPlayer() ? jiggleSetting_ * fuel::riderJiggleOutfitScale(outfit_.outfit) : 0.0; }
    // One step of the player's glute springs: `vehicleWorld` = the rider mesh's vehicle -> world (row-vector), dt seconds.
    void stepJiggle(uint32_t key, const fuel::Mat4& vehicleWorld, double dt, bool seated) {
        auto it = instances_.find(key);
        if (it == instances_.end() || !it->second->player || !it->second->jiggle.active() || !it->second->tmpl->hasJiggle) return;
        Instance& in = *it->second;
        if (renderer_.cutSerial() != in.cutSerial) in.cutSerial = renderer_.cutSerial(), in.jiggle.reset();  // cuts, teleports, respawns
        const int hips = in.tmpl->jiggleBones[0].base;
        if (hips < 0) return;
        const fuel::Mat4 skinV = rider_.skinMatrix(hips, in.anim->boneWorlds());
        in.jiggle.step(fuel::anim::mul(skinV, vehicleWorld), dt, seated);
        static const bool trace = std::getenv("FUEL_JIGGLE_TRACE") != nullptr;  // films: the springs' offsets, 10 times a second
        if (trace && (in.traceT += dt) >= 0.1) {
            in.traceT = 0;
            const auto o = in.jiggle.offset(0);
            std::printf("jiggle: amp %.2f L offset %+.4f %+.4f %+.4f m%s\n", in.jiggle.amplitude(), o.d[0], o.d[1], o.d[2], seated ? "" : " (not seated)");
        }
    }
    void resetJiggle() {
        for (auto& [k, in] : instances_) in->jiggle.reset();
    }
    const fuel::RiderJiggle* jiggle(uint32_t key) const {
        auto it = instances_.find(key);
        return it == instances_.end() ? nullptr : &it->second->jiggle;
    }
    bool customPlayer() const { return outfit_.outfit > 0 && fuel::CustomRider::shared() != nullptr; }

    // The rider's mesh IDs for `variant` (one, in vehicle space; none when it has no archetype or the rider
    // cannot be built), in the static riding pose, seated at `seat` (riderSeatPoint, FUEL's node 61 of 0x5ca410).
    // Baked once per variant.
    std::vector<uint32_t> parts(const std::string& variant, const std::array<double, 3>& seat = {}) {
        const std::string arch = fuel::vehicleArchetype(tsc_, variant);
        if (arch.empty()) return {};
        if (auto it = keys_.find(variant); it != keys_.end()) return it->second ? std::vector<uint32_t>{it->second} : std::vector<uint32_t>{};
        const uint32_t key = fuel::asoboCrc("RIDER_" + variant);  // not a Mesh_Z ID
        uint32_t result = 0;
        try {
            const int slot = staticSlot(arch);
            if (slot < 0) throw std::runtime_error("no riding pose for archetype " + arch);
            std::vector<fuel::Mat4> world = rider_.pose(slot, 0.0);
            for (fuel::Mat4& m : world)
                for (int k = 0; k < 3; ++k) m[size_t(12 + k)] += seat[size_t(k)];
            fuel::BakedMesh mesh = skinTemplate().mesh;
            rider_.skin(skinTemplate(), world, mesh.vertices);
            std::string err;
            if (renderer_.addBakedMesh(key, mesh, &err))
                result = key;
            else
                std::fprintf(stderr, "fuelview: rider: %s\n", err.c_str());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: rider for %s: %s\n", variant.c_str(), e.what());
        }
        keys_[variant] = result;
        return result ? std::vector<uint32_t>{result} : std::vector<uint32_t>{};
    }
    // FUEL's seat point for an assembled vehicle (zero when its skeleton has no Head bone).
    static std::array<double, 3> seat(const fuel::AssembledVehicle& av, const fuel::DpcFile& dpc) {
        try {
            if (auto s = fuel::riderSeatPoint(dpc, av)) return s->t;
        } catch (const std::exception&) {
        }
        return {0, 0, 0};
    }

    // A rider of its own for one driven vehicle (animated when the tables loaded, else the static pose).
    // `av`/`dpc`: the assembled vehicle and its DPC, for the IK_* points of its parts; `P` its VehiclePP block.
    std::vector<uint32_t> animated(const std::string& variant, const fuel::AssembledVehicle& av, const fuel::DpcFile& dpc,
                                   const std::vector<float>& P, bool player = false) {
        const std::array<double, 3> seatAt = seat(av, dpc);
        if (!tables_) return parts(variant, seatAt);
        const std::string arch = fuel::vehicleArchetype(tsc_, variant);
        if (arch.empty() || fuel::riderArchetypeIndex(arch) < 0) return {};
        try {
            auto inst = std::make_unique<Instance>();
            inst->tmpl = player ? &playerTemplate() : &skinTemplate();  // PHYSIQUE: the player's outfit
            inst->player = player;
            if (player && inst->tmpl->hasJiggle) inst->jiggle.setBones(inst->tmpl->jiggleBones), inst->jiggle.setAmplitude(jiggleAmplitude());
            inst->anim = std::make_unique<fuel::RiderAnimator>(rider_, *tables_, arch, vehicleClass(variant), P, ikPoints(av, dpc), seatAt);
            if (!inst->anim->valid()) return parts(variant, seatAt);
            inst->anim->setHandFrames(fuel::riderIkHandFrames(dpc, av));  // PK31: 0x604410 grip orientation
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
            return parts(variant, seatAt);
        }
    }

    // ---- P104: FUEL's rider LOD (notes p104-vehicle-lod.md 1.3). The rider Lod_Z's lower levels of the animated rider
    // `key` (animated's result): one mesh key per level 1.. (LOD2, LOD3), sharing its animator; empty when `key` is not
    // an animated rider. The rule's data: lodLevels / lodBaseDistance / lodViewDist.
    std::vector<uint32_t> lowerLevels(uint32_t key) {
        auto it = instances_.find(key);
        if (it == instances_.end()) return {};
        std::vector<uint32_t> out;
        for (int level = 1; level < rider_.lodLevels(); ++level) {
            try {
                auto inst = std::make_unique<Instance>();
                inst->tmpl = &skinTemplate(level);
                inst->anim = it->second->anim;
                const uint32_t k = fuel::asoboCrc("RIDER_LOD" + std::to_string(level + 1) + "_" + std::to_string(++serial_));
                fuel::BakedMesh mesh = inst->tmpl->mesh;
                rider_.skin(*inst->tmpl, inst->anim->boneWorlds(), mesh.vertices);
                std::string err;
                if (!renderer_.addBakedMesh(k, mesh, &err)) {
                    std::fprintf(stderr, "fuelview: rider LOD%d: %s\n", level + 1, err.c_str());
                    break;
                }
                instances_[k] = std::move(inst);
                out.push_back(k);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "fuelview: rider LOD%d: %s\n", level + 1, e.what());
                break;
            }
        }
        return out;
    }
    int lodLevels() const { return rider_.lodLevels(); }
    float lodBaseDistance() const { return rider_.lodBaseDistance(); }
    float lodViewDist() const { return rider_.lodViewDist(); }
    // ---- P104 end

    fuel::RiderAnimator* animator(uint32_t key) {
        auto it = instances_.find(key);
        return it == instances_.end() ? nullptr : it->second->anim.get();
    }
    // Skins the rider `key` in its current pose and hands the vertices to the renderer.
    void upload(uint32_t key) {
        auto it = instances_.find(key);
        if (it == instances_.end()) return;
        Instance& in = *it->second;
        rider_.skin(*in.tmpl, in.anim->boneWorlds(), in.vertices, in.jiggle.active() ? &in.jiggle : nullptr);  // PHYSIQUE: glutes
        renderer_.updateBakedVertices(key, in.vertices);
        // PHYSIQUE (rider4) debug: FUEL_RIDER_POSE_DUMP=FILE writes the player's bone worlds (vehicle space, row-vector,
        // 16 per line in skeleton order) after every upload; the last one is the shot's pose (hand close-ups, grip fit).
        static const char* dump = std::getenv("FUEL_RIDER_POSE_DUMP");
        if (dump && in.player) {
            if (FILE* f = std::fopen(dump, "w")) {
                for (const fuel::Mat4& m : in.anim->boneWorlds()) {
                    for (double v : m) std::fprintf(f, "%.6f ", v);
                    std::fprintf(f, "\n");
                }
                std::fclose(f);
            }
        }
    }

private:
    struct Instance {
        const fuel::Rider::SkinTemplate* tmpl = nullptr;
        std::shared_ptr<fuel::RiderAnimator> anim;  // P104: shared by the rider's LOD keys
        std::vector<uint8_t> vertices;
        bool player = false;         // PHYSIQUE: the player's rider (custom body, glute springs)
        fuel::RiderJiggle jiggle;    // PHYSIQUE
        uint32_t cutSerial = 0;      // PHYSIQUE: the renderer's cut count when the springs last ran
        double traceT = 0;
    };
    int staticSlot(const std::string& arch) const {
        if (tables_) {  // the base riding loop (action 0) from FUEL's slot table
            const int a = fuel::riderArchetypeIndex(arch);
            if (a >= 0 && rider_.hasSlot(tables_->slots[size_t(a)][0])) return tables_->slots[size_t(a)][0];
        }
        return rider_.ridingSlot(arch);
    }
    const fuel::Rider::SkinTemplate& skinTemplate(int level) {  // P104: the rider Lod_Z's level `level` (> 0)
        if (level <= 0) return skinTemplate();
        auto& t = tmplLevels_[level];
        if (!t) {
            try {
                t = std::make_unique<fuel::Rider::SkinTemplate>(rider_.prepareSkin(parts_, level));
            } catch (const std::exception& e) {  // P66: a part the DPC lacks: the default rider
                std::fprintf(stderr, "fuelview: rider parts (LOD%d): %s\n", level + 1, e.what());
                t = std::make_unique<fuel::Rider::SkinTemplate>(rider_.prepareSkin(fuel::Rider::Parts{}, level));
            }
        }
        return *t;
    }
    // PHYSIQUE: the player's template: the custom rider in the chosen outfit, else FUEL's parts.
    const fuel::Rider::SkinTemplate& playerTemplate() {
        if (!customPlayer()) return skinTemplate();
        if (!playerTmpl_) {
            try {
                playerTmpl_ = std::make_unique<fuel::Rider::SkinTemplate>(fuel::CustomRider::shared()->prepare(rider_, outfit_, parts_, 0));
                std::printf("fuelview: custom rider: %s, %zu vertices\n", fuel::riderOutfitName(outfit_), playerTmpl_->verts.size());
            } catch (const std::exception& e) {
                std::fprintf(stderr, "fuelview: custom rider: %s; FUEL's rider is used\n", e.what());
                return skinTemplate();
            }
        }
        return *playerTmpl_;
    }
    const fuel::Rider::SkinTemplate& skinTemplate() {
        if (!tmpl_) {
            try {
                tmpl_ = std::make_unique<fuel::Rider::SkinTemplate>(rider_.prepareSkin(parts_));
            } catch (const std::exception& e) {  // P66: a part the DPC lacks: the default rider
                std::fprintf(stderr, "fuelview: rider parts: %s\n", e.what());
                tmpl_ = std::make_unique<fuel::Rider::SkinTemplate>(rider_.prepareSkin());
            }
        }
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
    // IK_LEFTHAND / IK_RIGHTHAND / IK_LEFTFOOT / IK_RIGHTFOOT (named boxes of the vehicle's parts), vehicle space.
    static std::array<std::array<float, 3>, 4> ikPoints(const fuel::AssembledVehicle& av, const fuel::DpcFile& dpc) {
        return fuel::riderIkPoints(dpc, av);
    }

    fuel::Rider rider_;
    std::string tsc_;
    fuel::MeshRenderer& renderer_;
    std::unique_ptr<fuel::RiderTables> tables_;
    std::unique_ptr<fuel::Rider::SkinTemplate> tmpl_;
    std::map<int, std::unique_ptr<fuel::Rider::SkinTemplate>> tmplLevels_;  // P104
    fuel::Rider::Parts parts_;  // P66
    fuel::RiderOutfit outfit_{0, true};  // PHYSIQUE: 0 until setOutfit (the viewer's checks keep FUEL's rider)
    double jiggleSetting_ = 0.5;          // PHYSIQUE: rider_jiggle (default subtle)
    std::unique_ptr<fuel::Rider::SkinTemplate> playerTmpl_;
    std::map<std::string, uint32_t> keys_;  // variant -> mesh key (0 = failed)
    std::map<uint32_t, std::unique_ptr<Instance>> instances_;
    int serial_ = 0;
};
