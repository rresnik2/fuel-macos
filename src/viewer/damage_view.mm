// P85: FUEL's visual vehicle damage on the drive session's vehicles (damage_view.h).
#import "damage_view.h"

#include "drive.h"
#include "renderer.h"

#include "core/assembly.hpp"
#include "core/resources.hpp"
#include "core/upgrade_visuals.hpp"  // [upgrades U5]
#include "core/vehicle_lights.hpp"
#include "physics/fuel_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

DamageView gDamageView;

const fuel::Mesh* DamageView::mesh(const fuel::MeshRenderer& renderer, uint32_t id) {
    auto it = meshes_.find(id);
    if (it != meshes_.end()) return it->second.get();
    std::unique_ptr<fuel::Mesh> m;
    const uint32_t src = fuel::upgvis::baseMesh(id);  // [upgrades U5] a derived mesh dents as its base (same vertices)
    if (auto hit = renderer.library().find(src, fuel::cls::Mesh)) try {
            const fuel::MeshLink link = fuel::parseMeshLink(hit.file->link(*hit.entry));
            if (const auto body = hit.file->body(*hit.entry)) {
                m = std::make_unique<fuel::Mesh>(fuel::parseMeshBody(*body, link.flags));
                if (m->morphs.empty()) m.reset();  // 0x7cb550 returns -1: nothing to deform
            }
        } catch (const std::exception&) {
            m.reset();
        }
    if (!tablesTried_ && renderer.library().find(src, fuel::cls::Mesh)) {
        tablesTried_ = true;
        tables_ = fuel::damage::loadTables(*renderer.library().find(src, fuel::cls::Mesh).file);
        if (!tables_.ok) std::fprintf(stderr, "fuelview: FUEL.exe damage tables (0xa057a8, 0xa060e8) not read: no visual damage\n");
    }
    return meshes_.emplace(id, std::move(m)).first->second.get();
}

void DamageView::prewarm(const DriveSession& drive, fuel::MeshRenderer& renderer, WorldStreamer* streamer) {
    if (streamer != nullptr)  // the items step() throws (kItems below)
        streamer->prewarmDebris({"IT_DYN_GrosBoulon", "IT_DYN_GrosseTige", "IT_DYN_PetiteTige", "IT_DYN_PetitePlaque"}, renderer);
    static const bool off = std::getenv("FUEL_NO_DAMAGE_VISUAL") != nullptr;
    if (off) return;
    for (int i = 0; i < drive.vehicleCount(); ++i) {
        if (drive.physics().model(i) == nullptr) continue;
        const VehicleRig& rig = drive.rig(i);
        for (size_t p = 0; p < rig.parts().size(); ++p)
            if (rig.damageable(p)) mesh(renderer, rig.parts()[p].meshId);  // P104: LOD1 parts  // cached (and the tables read) as update would
    }
}

void DamageView::update(const DriveSession& drive, fuel::MeshRenderer& renderer) {
    static const bool off = std::getenv("FUEL_NO_DAMAGE_VISUAL") != nullptr;
    const int n = drive.vehicleCount();
    if (renderer.morphEpoch() != epoch_ || vehicles_.size() != size_t(n)) {
        epoch_ = renderer.morphEpoch();
        vehicles_.assign(size_t(n), Vehicle{});
    }
    if (off) {
        updateLinks(drive, renderer);  // PK22 (the lamps, V:DYN motion and dirt are not damage visuals)
        return;
    }
    for (int i = 0; i < n; ++i) {
        const fuel::fv::FuelVehicle* model = drive.physics().model(i);
        if (model == nullptr) continue;
        Vehicle& v = vehicles_[size_t(i)];
        const auto& impact = model->damageRecord(1);   // pm+0x144
        const auto& scrape = model->damageRecord(0);   // pm+0x118
        float cur[10];
        for (int k = 0; k < 10; ++k) cur[k] = impact[size_t(k)];
        // 0x5cfda0: the largest impact slot picks the engine smoke (>= 0.6: FX_CAR_Damage01, > 0.85: FX_CAR_Damage03).
        const float mx = *std::max_element(cur, cur + 10);
        v.smoke = mx > 0.85f ? 2 : mx >= 0.6f ? 1 : 0;
        // 0x5c7760: nothing to do while no impact slot moved by more than 0.01 (0x5c6560).
        if (v.valid && !fuel::damage::slotsChanged(v.applied.data(), cur)) continue;
        const bool firstPass = !v.valid;
        v.valid = true;
        std::copy(cur, cur + 10, v.applied.begin());
        const bool any = std::any_of(cur, cur + 10, [](float x) { return x > 0; });
        if (firstPass && !any) continue;  // nothing drawn deformed yet (0x6a27b0 on a clean vehicle)
        // ---- PK22: 0x6a27b0 -> 0x66f390: every V:DYN record back to rest (loose, damage, spring points)
        initLinks(v, drive, i);
        for (fuel::vlinks::DynRecord& r : v.dyn) fuel::vlinks::reset(r);
        // ---- PK22 end
        ++stats_.rebuilds;
        // The body's local AABB (0x712410): proxy spheres (the wheels included) and boxes, in vehicle space.
        const fuel::VehicleSetup& setup = drive.setup(i);
        std::array<float, 3> lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        for (const auto& s : setup.proxy.spheres)
            for (int k = 0; k < 3; ++k) lo[size_t(k)] = std::min(lo[size_t(k)], s.centre[size_t(k)] - s.radius), hi[size_t(k)] = std::max(hi[size_t(k)], s.centre[size_t(k)] + s.radius);
        for (const auto& b : setup.proxy.boxes)
            for (int k = 0; k < 3; ++k)
                lo[size_t(k)] = std::min(lo[size_t(k)], b.centre[size_t(k)] - b.halfExtents[size_t(k)]),
                hi[size_t(k)] = std::max(hi[size_t(k)], b.centre[size_t(k)] + b.halfExtents[size_t(k)]);
        if (lo[0] > hi[0]) continue;
        const VehicleRig& rig = drive.rig(i);
        const auto& parts = rig.parts();
        for (size_t p = 0; p < parts.size(); ++p) {
            if (!rig.damageable(p)) continue;  // P104: 0x6a2480 dents LOD record 0 only
            const fuel::Mesh* m = mesh(renderer, parts[p].meshId);
            // ---- PK22: a V:DYN part without morph target (0x7cb550 -1): its DYN_DAMAGE box against each sphere
            // (0x75fd20), f = r / the distance
            if (m == nullptr && tables_.ok && rig.partDyn(p) >= 0 && size_t(rig.partDyn(p)) < v.dyn.size())
                if (const std::array<float, 16>* box = dynDamageBox(renderer, parts[p].meshId)) {
                    const simd_float4x4 inv = simd_inverse(parts[p].world);
                    fuel::vlinks::DynRecord& r = v.dyn[size_t(rig.partDyn(p))];
                    for (int k = 0; k < 10; ++k) {
                        if (!(cur[k] > 0)) continue;
                        const fuel::damage::Sphere s = fuel::damage::slotSphere(tables_, k, lo, hi, cur[k]);
                        const simd_float4 c = simd_mul(inv, simd_make_float4(s.c[0], s.c[1], s.c[2], 1));
                        float dist = 1e10f;
                        if (!fuel::vlinks::sphereBox(*box, {c.x, c.y, c.z}, s.r, dist) || !(dist > 0)) continue;
                        const bool was = r.loose;
                        fuel::vlinks::addDamage(r, s.r / dist, cur[k], std::max(0.0f, scrape[size_t(k)]));
                        if (r.loose && !was) ++stats_.dynLoose;
                    }
                }
            // ---- PK22 end
            if (m == nullptr || !tables_.ok) continue;
            // 0x6a27b0 freed the part's morph VB; each slot with an impact re-applies its sphere (0x6a2480 -> 0x7cb550)
            // in the part's local space (the node's inverse world; the radius as is).
            std::vector<float> vb;
            const simd_float4x4 inv = simd_inverse(parts[p].world);
            const int dynIdx = rig.partDyn(p);  // PK22
            for (int k = 0; k < 10; ++k) {
                if (!(cur[k] > 0)) continue;
                const fuel::damage::Sphere s = fuel::damage::slotSphere(tables_, k, lo, hi, cur[k]);
                const simd_float4 c = simd_mul(inv, simd_make_float4(s.c[0], s.c[1], s.c[2], 1));
                const float f = fuel::damage::morphDamage(*m, {{c.x, c.y, c.z}, s.r}, vb);
                // ---- PK22: 0x6a2480's V:DYN part: f (0 / 1) x (v + s) into the record (the scrape slot pm+0x118)
                if (dynIdx >= 0 && size_t(dynIdx) < v.dyn.size()) {
                    fuel::vlinks::DynRecord& r = v.dyn[size_t(dynIdx)];
                    const bool was = r.loose;
                    fuel::vlinks::addDamage(r, f, cur[k], std::max(0.0f, scrape[size_t(k)]));
                    if (r.loose && !was) ++stats_.dynLoose;
                }
                // ---- PK22 end
            }
            renderer.setPartMorph(i == 0 ? -1 : i - 1, p, vb);
            if (!vb.empty()) {
                ++stats_.morphedParts;
                for (size_t q = 0; q + 2 < vb.size(); q += 3) stats_.movedVertices += (vb[q] != 0 || vb[q + 1] != 0 || vb[q + 2] != 0);
            }
        }
        static const bool trace = std::getenv("FUEL_P85_TRACE") != nullptr;
        if (trace)
            std::printf("damage visual: vehicle %d rebuilt (impact max %.2f), %zu parts morphed so far, %zu vertices moved so far\n", i, mx,
                        stats_.morphedParts, stats_.movedVertices);
    }
    updateLinks(drive, renderer);  // PK22
}

// ---- Crash debris (0x51e9e0 -> 0x52ee60) and damage smoke (0x5cfda0 -> 0x5cfeb0) ------------------------------

float DamageView::rand01() {  // [port] a Park-Miller draw (FUEL's 0x6460f0 shares one sequence with the whole game)
    rng_ = uint32_t((uint64_t(rng_) * 16807u) % 2147483647u);
    return float(rng_) / 2147483647.0f;
}

void DamageView::step(float dt, const DriveSession& drive, WorldStreamer* streamer, const fuel::MeshRenderer& renderer) {
    static const bool off = std::getenv("FUEL_NO_DAMAGE_VISUAL") != nullptr;
    if (dt <= 0) return;
    const int n = drive.vehicleCount();
    if (vehicles_.size() != size_t(n)) vehicles_.resize(size_t(n));
    stepLinks(dt, drive, renderer);  // PK22
    if (off) return;
    if (!smokeTried_) {  // the Particles_Z FX_CAR_Damage01 / 03 (USA1; 0x5d00c0 hashes the names into +0x1a74 / +0x1a78)
        smokeTried_ = true;
        const uint32_t want[2] = {fuel::asoboCrc("FX_CAR_Damage01"), fuel::asoboCrc("FX_CAR_Damage03")};
        for (const auto& f : renderer.library().files())
            for (uint32_t id : f->ids(0xc56963deu)) try {  // Particles_Z
                    const fuel::DpcFile::Entry* e = f->find(id);
                    if (e == nullptr) continue;
                    const std::vector<uint8_t> link = f->link(*e);
                    uint32_t name = 0;
                    if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                    for (int k = 0; k < 2; ++k)
                        if (name == want[k] && !smokeEmitters_[k])
                            if (const auto body = f->body(*e)) smokeEmitters_[k] = std::make_shared<fuel::particles::Emitter>(fuel::particles::parseParticles(*body, link));
                } catch (const std::exception&) {
                }
    }
    const fuel::fv::FuelVehicle* player = drive.physics().model(0);
    for (int i = 0; i < n; ++i) {
        const fuel::fv::FuelVehicle* model = drive.physics().model(i);
        if (model == nullptr) continue;
        Vehicle& v = vehicles_[size_t(i)];
        // 0x52ee60: the debris request of the contact hook 0x51e9e0 (body().debris*; the player only, fuel_vehicle.cpp).
        const fuel::fv::RigidBody& body = model->body();
        // The physics works about a moving origin: its model origin vs the drawn chassis gives the shift.
        const fuel::fv::Vec3 mo = model->modelOrigin();
        const auto& cp = drive.state(i).chassis.position;
        const std::array<double, 3> shift{cp[0] - mo.x, cp[1] - mo.y, cp[2] - mo.z};
        if (body.debrisSerial != v.debrisSerial) {
            v.debrisSerial = body.debrisSerial;
            static const char* const kItems[4] = {"IT_DYN_GrosBoulon", "IT_DYN_GrosseTige", "IT_DYN_PetiteTige", "IT_DYN_PetitePlaque"};
            if (std::getenv("FUEL_P85_TRACE")) std::printf("crash debris: streamer %d\n", streamer != nullptr);
            ++debrisRequests_;
            if (streamer != nullptr && !std::getenv("FUEL_NO_CRASH_DEBRIS"))
                for (int k = 0; k < body.debrisPieces; ++k) {
                    const int item = std::clamp(int(std::floor(rand01() * 3.99f)), 0, 3);  // ROUND under RC_DOWN
                    const float up = rand01() * 3.0f + 1.0f, x = rand01() * 4.0f - 2.0f, z = rand01() * 4.0f - 2.0f;
                    const std::array<float, 3> a{x, up, z};  // up (0xa06320) x (1 + 3 r) + X (0xa06338) x .. + Z (0xa06350) x ..
                    const std::array<float, 3> vel{a[0] * 3, a[1] * 3, a[2] * 3};
                    const std::array<float, 3> w{(a[0] - a[2]) * 3, a[1] * 3, (a[0] + a[2]) * 3};  // 3 (a x up + a)
                    uint32_t id = 0;
                    // 0x533d70: the node turned about up (0xa06320) by 0x40f550(v.x, v.z) = fpatan = atan2(v.x, v.z),
                    // placed at the contact point (Node_Z 0x65feb0), +0x820 = v, +0x830 = w; the body keeps +0xeb2 = 1
                    // (no contacts at all: it cannot be trapped in its vehicle).
                    const float yaw = std::atan2(vel[0], vel[2]);
                    if (streamer->throwDebris(kItems[item], {body.debrisAt.x + shift[0], body.debrisAt.y + shift[1], body.debrisAt.z + shift[2]}, vel, yaw,
                                              const_cast<DriveSession&>(drive).physics(), &w, &id, true)) {
                        pieces_.push_back({id});
                        ++debrisSpawned_;
                    }
                }
            if (std::getenv("FUEL_P85_TRACE"))
                std::printf("crash debris: vehicle %d asks %d pieces at (%.1f, %.1f, %.1f); %zu spawned so far, %zu live\n", i, body.debrisPieces,
                            body.debrisAt.x + shift[0], body.debrisAt.y + shift[1], body.debrisAt.z + shift[2], debrisSpawned_, pieces_.size());
        }
        {  // 0x5cfda0 (every frame): the largest impact slot picks the level (> 0.85: 2, >= 0.6: 1)
            const auto& r1 = model->damageRecord(1);
            const float mx = *std::max_element(r1.begin(), r1.begin() + 10);
            v.smoke = mx > 0.85f ? 2 : mx >= 0.6f ? 1 : 0;
        }
        // 0x5cfda0: the smoke level change stops the running FX (0x5cfff0) and starts the new one (0x5cfeb0, restart 1).
        if (v.smoke != v.smokeShown) {
            for (auto& s : v.smokeFx.state) s.stopped = true;
            if (v.smoke > 0 && smokeEmitters_[v.smoke - 1]) {
                v.smokeFx = fuel::particles::Instance{};
                v.smokeFx.emitter = smokeEmitters_[v.smoke - 1].get();
                v.smokeFx.reset();
            }
            v.smokeShown = v.smoke;
            if (std::getenv("FUEL_P85_TRACE"))
                std::printf("damage smoke: vehicle %d level %d (FX_CAR_Damage01 %s, 03 %s)\n", i, v.smoke, smokeEmitters_[0] ? "found" : "missing",
                            smokeEmitters_[1] ? "found" : "missing");
        }
        if (v.smokeFx.emitter != nullptr) {
            const std::array<float, 3>* eng = drive.rig(i).engine();
            const simd_float4x4 node = drive.rig(i).nodeWorld(drive.state(i));
            const simd_float4 p = simd_mul(node, eng ? simd_make_float4((*eng)[0], (*eng)[1], (*eng)[2], 1) : simd_make_float4(0, 0.5f, 0, 1));
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) v.smokeFx.world[size_t(r * 4 + c)] = node.columns[r][c];  // row-vector rows = columns
            v.smokeFx.world[12] = p.x, v.smokeFx.world[13] = p.y, v.smokeFx.world[14] = p.z, v.smokeFx.world[15] = 1;
            fuel::particles::step(v.smokeFx, dt, [this] { return rand01(); });
            static int traceN = 0;
            if (std::getenv("FUEL_P85_TRACE") && ++traceN % 30 == 0) {
                size_t live = 0;
                for (const auto& st : v.smokeFx.state) live += st.live.size();
                std::printf("damage smoke: vehicle %d, %zu elements, %zu live particles at (%.1f, %.1f, %.1f)", i, v.smokeFx.state.size(), live,
                            v.smokeFx.world[12], v.smokeFx.world[13], v.smokeFx.world[14]);
                for (const auto& E : v.smokeFx.emitter->elements)
                    std::printf(" [rate %d dur %.1f life %.1f flags %x max %d]", E.rate, E.duration, E.life, E.flags, E.maxParticles);
                std::printf("\n");
            }
            if (v.smokeShown == 0 && std::all_of(v.smokeFx.state.begin(), v.smokeFx.state.end(), [](const auto& s) { return s.live.empty(); }))
                v.smokeFx = fuel::particles::Instance{};
        }
    }
    // IT_DebrisMove vt[140] 0x533ec0: every second the limit is refreshed from the player distance (small bodies or
    // >= 100 m: min(limit, (2 - clamp((100 - d) / 100, 0, 1)) x 5); else 30), the age grows, past the limit or 20 m below
    // the ground the piece leaves (vt+0x1d8).
    if (streamer == nullptr) return;
    const simd_float3 pp = player ? simd_make_float3(player->body().S.x.x, player->body().S.x.y, player->body().S.x.z) : drive.position();
    for (size_t k = 0; k < pieces_.size();) {
        Piece& q = pieces_[k];
        std::array<double, 3> pos{};
        bool gone = !streamer->thrownPosition(q.id, pos);
        q.tick += dt;
        if (!gone && 1.0f <= q.tick) {
            const float d = simd_distance(pp, simd_make_float3(float(pos[0]), float(pos[1]), float(pos[2])));
            const float radius = 0.25f;  // [port] the pieces are bolts, rods and a small plate: +0x408 <= 0.5
            if (radius <= 0.5f || 100.0f <= d) q.limit = std::min(q.limit, ((1.0f - std::clamp((100.0f - d) * 0.01f, 0.0f, 1.0f)) + 1.0f) * 5.0f);
            else q.limit = 30.0f;
            q.age += q.tick;
            if (q.limit < q.age) gone = true;
            q.tick = 0;
            if (!gone && float(pos[1]) < drive.terrain().smoothHeightAt(pos[0], pos[2]) - 20.0f) gone = true;
            if (gone) streamer->dropThrown(q.id, const_cast<DriveSession&>(drive).physics());
        }
        if (gone) pieces_.erase(pieces_.begin() + std::ptrdiff_t(k));
        else ++k;
    }
}

void DamageView::appendSmoke(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches) const {
    for (const Vehicle& v : vehicles_) {  // as PlacedSystem::collect, full fade
        if (v.smokeFx.emitter == nullptr) continue;
        for (size_t e = 0; e < v.smokeFx.state.size() && e < v.smokeFx.emitter->elements.size(); ++e) {
            const fuel::particles::Element& E = v.smokeFx.emitter->elements[e];
            const auto& s = v.smokeFx.state[e];
            fuel::particles::Batch b;
            b.material = E.material, b.first = uint32_t(quads.size());
            const int orient = (E.flags & 2) ? 2 : (E.flags & 4) ? 1 : 0;
            for (const fuel::particles::Particle& p : s.live) {
                if (p.age <= 0) continue;
                fuel::particles::Quad q;
                q.pos = p.render, q.angle = p.angle, q.sx = p.drawX, q.sy = p.drawY, q.orient = orient;
                q.rgba = p.colour;
                static const bool dbg = std::getenv("FUEL_P85_SMOKE_DEBUG") != nullptr;  // red markers: is it drawn at all
                if (dbg) q.rgba = {4, 0, 0, 1}, q.sx = q.sy = 3;
                quads.push_back(q);
            }
            b.count = uint32_t(quads.size()) - b.first;
            if (b.count) batches.push_back(b);
            if (b.count && std::getenv("FUEL_P85_TRACE")) {
                const auto& q = quads[b.first];
                float amax = 0, smax = 0;
                for (uint32_t k = b.first; k < b.first + b.count; ++k) amax = std::max(amax, quads[k].rgba[3]), smax = std::max(smax, quads[k].sx);
                std::printf("damage smoke draw: alpha max %.2f size max %.2f; ", amax, smax);
                std::printf("damage smoke draw: %u quads, first at (%.1f, %.1f, %.1f) size %.2f %.2f rgba %.2f %.2f %.2f %.2f orient %d\n", b.count, q.pos[0],
                            q.pos[1], q.pos[2], q.sx, q.sy, q.rgba[0], q.rgba[1], q.rgba[2], q.rgba[3], q.orient);
            }
        }
    }
}

// ---- PK22: the vehicle visual links (core/vehicle_links.hpp, notes pk22.md) ------------------------------------------

void DamageView::initLinks(Vehicle& v, const DriveSession& drive, int i) {
    if (v.dynInit) return;
    v.dynInit = true;
    v.dyn = drive.rig(i).dynRecords();  // reset at assembly (0x66f490)
}

const std::array<float, 16>* DamageView::dynDamageBox(const fuel::MeshRenderer& renderer, uint32_t meshId) {
    auto it = dynBoxes_.find(meshId);
    if (it != dynBoxes_.end()) return it->second.get();
    std::unique_ptr<std::array<float, 16>> box;
    if (auto hit = renderer.library().find(fuel::upgvis::baseMesh(meshId), fuel::cls::Mesh)) try {  // [upgrades U5]
            const uint32_t want = fuel::asoboCrc("DYN_DAMAGE");
            for (const fuel::NamedBox& b : fuel::parseMeshLinkBoxes(hit.file->link(*hit.entry)))
                if (b.name == want) {
                    box = std::make_unique<std::array<float, 16>>();
                    for (int k = 0; k < 12; ++k) (*box)[size_t(k)] = b.rows[size_t(k)];
                    for (int k = 0; k < 3; ++k) (*box)[size_t(12 + k)] = b.half[size_t(k)];
                    (*box)[15] = b.radius;
                    break;
                }
        } catch (const std::exception&) {
            box.reset();
        }
    return dynBoxes_.emplace(meshId, std::move(box)).first->second.get();
}

void DamageView::stepLinks(float dt, const DriveSession& drive, const fuel::MeshRenderer& renderer) {
    static const bool noDirt = std::getenv("FUEL_NO_LIVE_DIRT") != nullptr;  // A/B: the record stays clean
    const float dtc = std::min(dt, 0.1f);  // 0x5cb8c0 clamps the frame time to 0.1 before 0x5cc780
    for (int i = 0; i < drive.vehicleCount() && size_t(i) < vehicles_.size(); ++i) {
        const fuel::fv::FuelVehicle* model = drive.physics().model(i);
        if (model == nullptr) continue;
        Vehicle& v = vehicles_[size_t(i)];
        initLinks(v, drive, i);
        // 0x529ce0's dirt block (record 2): the weather's rain is the game +0x3330 object's +0xb8.
        if (!noDirt) const_cast<fuel::fv::FuelVehicle*>(model)->stepDirt(dt, renderer.weatherAmounts().rain);
        // 0x5cb8c0: the shake phase, then 0x5cc780's V:DYN step.
        v.phase = fuel::vlinks::advancePhase(v.phase, model->soundRpm(), dtc);
        if (v.dyn.empty()) continue;
        const fuel::VehicleState st = drive.state(i);
        fuel::vlinks::Motion m;
        m.pos = {float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2])};
        m.q = {st.chassis.rotation[0], st.chassis.rotation[1], st.chassis.rotation[2], st.chassis.rotation[3]};
        const fuel::fv::RigidBody& body = model->body();
        m.vel = {body.avgV.x, body.avgV.y, body.avgV.z};
        m.angVel = {body.avgW.x, body.avgW.y, body.avgW.z};
        fuel::vlinks::Engine e;
        e.rpm = model->soundRpm();
        e.phase = v.phase;
        e.throttle = std::clamp(model->engineThrottle(), 0.0f, 1.0f);  // 0x5cf2d0's clamp of +0x1088
        e.gear = model->gear();
        for (fuel::vlinks::DynRecord& r : v.dyn)
            if (fuel::vlinks::step(r, m, e, dtc)) {
                ++stats_.dynDetached;
                if (std::getenv("FUEL_PK22_TRACE")) std::printf("pk22: vehicle %d V:DYN part leaves (damage total %.2f > %.2f)\n", i, r.damageTotal, r.ldyn);
            }
    }
}

void DamageView::updateLinks(const DriveSession& drive, fuel::MeshRenderer& renderer) {
    static const bool noLinks = std::getenv("FUEL_NO_VEHICLE_LINKS") != nullptr;  // A/B: lamps as stored, no V:DYN, global dirt
    if (noLinks) return;
    static const bool trace = std::getenv("FUEL_PK22_TRACE") != nullptr;
    stats_.dynPosed = 0;
    const double tod = renderer.timeOfDay();
    for (int i = 0; i < drive.vehicleCount() && size_t(i) < vehicles_.size(); ++i) {
        const fuel::fv::FuelVehicle* model = drive.physics().model(i);
        Vehicle& v = vehicles_[size_t(i)];
        initLinks(v, drive, i);
        const VehicleRig& rig = drive.rig(i);
        const int rv = i == 0 ? -1 : i - 1;  // the renderer's vehicle index (setPartMorph)
        // Lamp groups (0x5cc780): BACK in reverse, STOP above 0.1 of 0x5cd8f0, POSITION / HEAD with the headlights.
        fuel::vlinks::LampInputs li;
        li.physics = model != nullptr;
        li.gear = model ? model->gear() : 1;
        li.brakeValue = model ? model->slipPedal() : 0.0f;
        li.headlights = rig.headlightRecords() && fuel::omni::headlightsOn(tod);
        const std::array<float, 4> lamps = fuel::vlinks::lampAlphas(li);
        renderer.setVehicleLamps(rv, lamps);
        if (trace && lamps != v.lamps)
            std::printf("pk22: vehicle %d lamps back %.1f stop %.1f position %.1f head %.1f (gear %d, brake %.2f)\n", i, lamps[0], lamps[1], lamps[2],
                        lamps[3], li.gear, li.brakeValue);
        v.lamps = lamps;
        // V:DYN parts (0x6a0200): a moved record's rotation (and LIMB pivot) as the factor local^-1 x local'.
        for (size_t p = 0; p < rig.parts().size(); ++p) {
            const int d = rig.partDyn(p);
            if (d < 0 || size_t(d) >= v.dyn.size()) continue;
            const fuel::vlinks::DynPose ps = fuel::vlinks::pose(v.dyn[size_t(d)], rig.partDynPivot(p), rig.partDynBoneQ(p));
            if (ps.state == 0) {
                renderer.setPartAdjust(rv, p, nullptr);
                continue;
            }
            simd_float4x4 adj;
            if (ps.state == 2) {  // [port] the part left with its IT_DEBRIS (not thrown in the port): not drawn
                adj = simd_float4x4{};
                adj.columns[3] = simd_make_float4(0, 0, 0, 1);
            } else {
                simd_float4x4 local2 = simd_matrix4x4(simd_quaternion(ps.rot[0], ps.rot[1], ps.rot[2], ps.rot[3]));
                local2.columns[3] = simd_make_float4(ps.pos[0], ps.pos[1], ps.pos[2], 1);
                adj = simd_mul(simd_inverse(rig.partLocal(p)), local2);
                ++stats_.dynPosed;
            }
            renderer.setPartAdjust(rv, p, &adj);
        }
        // Dirt (0x5c9090): the mean of record 2 pushed to C_USER0 of the vehicle and rider when it moved > 0.025.
        if (model != nullptr) {
            const auto& r2 = model->damageRecord(2);
            const float mean = fuel::vlinks::dirtMean(r2.data());
            if (!v.dirtSet || fuel::vlinks::dirtPush(mean, v.dirtPushed)) {
                if (v.dirtSet) {
                    v.dirtPushed = mean;
                    ++stats_.dirtPushes;
                    if (trace) std::printf("pk22: vehicle %d dirt %.3f\n", i, mean);
                }
                v.dirtSet = true;
                renderer.setVehicleDirt(rv, v.dirtPushed);
            }
        }
    }
}
// ---- PK22 end
