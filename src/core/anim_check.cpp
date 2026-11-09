// --animcheck (P47): FUEL's key samplers against vectors emulated on the player's FUEL.exe (helper notes
// p47-work/anim-objects.md §4, rtc.md §2.3), the shipped RTC files parsed exactly, the RTC playback timing
// (TR_DEATH's respawn event, VI_BIKE's loop, TELEPORT_HELICO's end) and the world's animated skins.
#include "anim_check.hpp"

#include "anim_clip.hpp"
#include "world_anim.hpp"
#include "dyn_object.hpp"
#include "race_markers.hpp"
#include "assembly.hpp"
#include "dpc.hpp"
#include "reader.hpp"
#include "resources.hpp"
#include "rider.hpp"
#include "custom_rider.hpp"
#include "rider_anim.hpp"
#include "rider_tables.hpp"
#include "rtc.hpp"
#include "rtc_fade.hpp"
#include "particles.hpp"
#include "vehicle_params.hpp"
#include "skinned_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <tuple>

namespace fuel::anim {
namespace {

struct Tally {
    int ok = 0, total = 0;
    void check(bool good, const std::string& what, double got, double want) {
        ++total;
        if (good) ++ok;
        std::printf("  %-52s %s (got %.8g, want %.8g)\n", what.c_str(), good ? "ok" : "MISMATCH", got, want);
    }
    void near(const std::string& what, double got, double want, double tol = 2e-6) { check(std::fabs(got - want) <= tol, what, got, want); }
    void vec(const std::string& what, const double* got, std::initializer_list<double> want, double tol = 2e-6) {
        size_t i = 0;
        double err = 0;
        for (double w : want) err = std::max(err, std::fabs(got[i++] - w));
        check(err <= tol, what, got[0], *want.begin());
    }
};

// A synthetic Animation_Z body (grammar 0x7c4980) with one bone track.
struct Body {
    std::vector<uint8_t> b;
    template <class T>
    void put(T v) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
        b.insert(b.end(), p, p + sizeof v);
    }
};
std::vector<uint8_t> clipBody(const std::vector<std::array<float, 1>>& at, const std::vector<std::array<int16_t, 4>>& aq,
                              const std::vector<std::array<float, 10>>& bk, uint16_t dFlags,
                              const std::vector<std::pair<float, std::array<int16_t, 9>>>& dk) {
    Body o;
    o.put(2.0f), o.put(uint32_t(0x3f800000)), o.put(uint16_t(1)), o.put(uint16_t(0xffff));
    o.put(uint32_t(at.size()));
    for (size_t i = 0; i < at.size(); ++i) {
        o.put(at[i][0]);
        for (int16_t v : aq[i]) o.put(v);
    }
    o.put(uint32_t(bk.size()));
    for (const auto& k : bk)
        for (float v : k) o.put(v);
    o.put(uint16_t(0)), o.put(uint32_t(0));  // C
    o.put(dFlags), o.put(uint32_t(dk.size()));
    for (const auto& [t, v] : dk) {
        o.put(t);
        for (int16_t x : v) o.put(x);
        o.put(uint16_t(0xffff));
    }
    o.put(uint32_t(0));  // E
    for (int i = 0; i < 7; ++i) o.put(uint16_t(0)), o.put(uint32_t(0));
    o.put(uint32_t(1));  // M: one track
    o.put(uint32_t(0x1234)), o.put(uint16_t(0)), o.put(uint16_t(0));
    o.put(uint16_t(0)), o.put(uint16_t(dk.size()));
    o.put(uint16_t(0)), o.put(uint16_t(at.size()));
    o.put(uint16_t(0)), o.put(uint16_t(bk.size()));
    o.put(uint16_t(0)), o.put(uint16_t(0));
    o.put(uint16_t(0)), o.put(uint16_t(0));
    o.put(uint32_t(0)), o.put(uint32_t(0)), o.put(uint32_t(0));  // N O P
    return o.b;
}

std::unique_ptr<DpcFile> openDpc(const std::string& p) {
    try {
        return std::make_unique<DpcFile>(p);
    } catch (const std::exception&) {
        return nullptr;
    }
}

bool rtcOf(const DpcFile& f, rtc::Rtc& out, std::string* err) {
    for (uint32_t id : f.ids(0x2A11CEE8))  // Rtc_Z
        if (auto b = f.body(*f.find(id))) return rtc::parse(*b, out, err);
    if (err) *err = "no Rtc_Z";
    return false;
}


// ---- rider seat (P59) ----

std::string readText(const std::string& p) {
    std::ifstream f(p);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

// Bone-name registry index of each bone track in the rider animations (Animation_Z bone record +4, grammar
// 0x7c4980), with the number of animations that disagree.
std::map<uint32_t, int> trackRegistry(const DpcFile& pilot, int* conflicts) {
    std::map<uint32_t, int> out;
    *conflicts = 0;
    for (uint32_t id : pilot.ids(cls::Animation)) {
        const auto body = pilot.body(*pilot.find(id));
        if (!body) continue;
        try {
            ByteReader r(*body);
            r.f32(), r.u32(), r.u16(), r.u16();
            r.skipArray(12);
            r.skipArray(40);
            r.u16(), r.skipArray(24);
            r.u16(), r.skipArray(24);
            for (uint32_t i = 0, n = r.count(); i < n; ++i) r.u32(), r.skipArray(20);
            for (size_t sz : {size_t(12), size_t(12), size_t(0x18), size_t(0x18), size_t(12), size_t(12), size_t(12)}) r.u16(), r.skipArray(sz);
            const uint32_t n = r.count();
            const uint8_t* p = r.raw(size_t(n) * 0x1c);
            for (uint32_t i = 0; i < n; ++i) {
                uint32_t crc;
                uint16_t idx;
                std::memcpy(&crc, p + i * 0x1c, 4);
                std::memcpy(&idx, p + i * 0x1c + 4, 2);
                auto [it, fresh] = out.emplace(crc, idx);
                if (!fresh && it->second != idx) ++*conflicts;
            }
        } catch (const std::exception&) {
            ++*conflicts;
        }
    }
    return out;
}

// The vehicle's drawn triangles in vehicle space, and the highest surface at (x, z) not above y (-inf if none).
struct VehicleSurface {
    std::vector<std::array<double, 9>> tris;
    VehicleSurface(const DpcFile& dpc, const AssembledVehicle& av) {
        for (const PlacedPart& part : av.parts) {
            if (!part.drawn() || part.shape != PartShape::Mesh) continue;
            const DpcFile::Entry* e = dpc.find(part.meshId);
            if (e == nullptr) continue;
            try {
                const Mesh m = parseMeshBody(dpc.body(*e).value(), parseMeshLink(dpc.link(*e)).flags);
                for (const Batch& b : m.batches) {
                    const VertexBuffer& vb = m.vbs.at(b.vb);
                    if (vb.stride < 12) continue;
                    const std::vector<uint32_t> idx = batchTriangles(m, b);
                    for (size_t k = 0; k + 2 < idx.size(); k += 3) {
                        std::array<double, 9> t;
                        for (size_t c = 0; c < 3; ++c) {
                            float q[3];
                            std::memcpy(q, &vb.data[size_t(idx[k + c]) * vb.stride], 12);
                            const auto& w = part.world;
                            for (size_t j = 0; j < 3; ++j) t[c * 3 + j] = q[0] * w[j] + q[1] * w[4 + j] + q[2] * w[8 + j] + w[12 + j];
                        }
                        tris.push_back(t);
                    }
                }
            } catch (const std::exception&) {
            }
        }
    }
    double below(double x, double y, double z) const {
        double best = -HUGE_VAL;
        for (const auto& t : tris) {
            const double d = (t[5] - t[8]) * (t[0] - t[6]) + (t[6] - t[3]) * (t[2] - t[8]);
            if (std::fabs(d) < 1e-12) continue;
            const double l1 = ((t[5] - t[8]) * (x - t[6]) + (t[6] - t[3]) * (z - t[8])) / d;
            const double l2 = ((t[8] - t[2]) * (x - t[6]) + (t[0] - t[6]) * (z - t[8])) / d;
            const double l3 = 1 - l1 - l2;
            if (l1 < 0 || l2 < 0 || l3 < 0) continue;
            const double h = l1 * t[1] + l2 * t[4] + l3 * t[7];
            if (h <= y && h > best) best = h;
        }
        return best;
    }
};

// Pelvis against the seat for a pose: hips bone height above the surface under it, and the lowest skin vertex
// within 0.1 m (x, z) of the hips above the surface under that vertex (negative = sunk into the seat).
struct SeatGap {
    double hips = HUGE_VAL, pelvis = HUGE_VAL;
};
SeatGap seatGap(const Rider& rider, const Rider::SkinTemplate& tmpl, const std::vector<Mat4>& world, const VehicleSurface& vs) {
    SeatGap g;
    const Mat4& H = world[size_t(rider.bone("Hips"))];
    const double s = vs.below(H[12], H[13], H[14]);
    if (s == -HUGE_VAL) return g;
    g.hips = H[13] - s;
    std::vector<uint8_t> v;
    rider.skin(tmpl, world, v);
    for (size_t i = 0; i < tmpl.verts.size(); ++i) {
        float p[3];
        std::memcpy(p, &v[i * BakedMesh::kStride], 12);
        if (std::hypot(p[0] - H[12], p[2] - H[14]) > 0.1 || p[1] > H[13]) continue;
        const double u = vs.below(p[0], H[13], p[2]);
        if (u != -HUGE_VAL) g.pelvis = std::min(g.pelvis, double(p[1]) - u);
    }
    return g;
}

// The vehicle seat check: FUEL's rule (0x5ca410 node 61 = the vehicle skeleton's Head bone) applied to every
// drivable garage variant, and the pelvis-to-seat distance per vehicle class, standing and driving.
void riderSeatCheck(const std::string& dataDir, Tally& t) {
    std::printf("rider seat (0x5ca410: pilot at the vehicle's node 61 = its Head bone):\n");
    auto pilot = openDpc(dataDir + "/CommonComponent/DATAS/PILOT.DPC");
    std::string err;
    const auto tables = loadRiderTables(dataDir + "/XFinalComponent/FUEL.exe", &err);
    if (!pilot || !tables) {
        t.check(false, "PILOT.DPC and FUEL.exe rider tables", 0, 1);
        return;
    }
    int conflicts = 0;
    const auto reg = trackRegistry(*pilot, &conflicts);
    auto idx = [&](const char* n) {
        const auto it = reg.find(asoboCrc(n));
        return it == reg.end() ? -1 : it->second;
    };
    t.check(conflicts == 0, "bone registry indices agree over the 256 anims", conflicts, 0);
    t.check(idx("Head") == 61, "registry: Head = node 61 (0x5ca410)", idx("Head"), 61);
    t.check(idx("Hips") == 25, "registry: Hips = node 25 (ragdoll body 0)", idx("Hips"), 25);
    t.check(idx("LeftHand") == 27 && idx("LeftForeArm") == 56 && idx("LeftArm") == 55 && idx("LeftShoulder") == 54,
            "registry: left-hand IK chain 27/56/55/54", idx("LeftHand"), 27);

    const Rider rider(*pilot);
    const Rider::SkinTemplate tmpl = rider.prepareSkin();
    {  // P76: per-limb IK switches in the rider animations (0x7c0480 / 0x64f3f0): kinds 0x10 off / 0x11 on, limb roots only
        const std::set<uint32_t> limbs = {asoboCrc("LeftArm"), asoboCrc("RightArm"), asoboCrc("LeftUpLeg"), asoboCrc("RightUpLeg")};
        int off = 0, on = 0, elsewhere = 0, clips = 0, reOn = 0;
        for (uint32_t id : pilot->ids(cls::Animation)) {
            Clip c;
            if (!c.parse(pilot->body(*pilot->find(id)).value_or(std::vector<uint8_t>{}))) continue;
            ++clips;
            for (size_t tr = 0; tr < c.tracks.size(); ++tr) {
                std::vector<std::pair<float, uint32_t>> ev;
                c.events(int(tr), -1e30, 1e30, true, ev);
                bool wasOff = false;
                for (const auto& [time, kind] : ev) {
                    if (kind != 0x10 && kind != 0x11) continue;
                    (kind == 0x10 ? off : on) += 1;
                    elsewhere += limbs.count(c.tracks[tr].bone) == 0;
                    if (kind == 0x10) wasOff = true;
                    else if (wasOff && time > 0.8 && time < 1.31) ++reOn, wasOff = false;
                }
            }
        }
        t.check(off == 195 && on == 801 && elsewhere == 0, "rider anims: 195 IK-off (0x10) / 801 IK-on (0x11) events, all on LeftArm/RightArm/LeftUpLeg/RightUpLeg",
                off * 1000 + on, 195801);
        t.check(reOn > 0, "some one-shots give a limb back to the IK between 0.83 and 1.3 s", reOn, 1);
        (void)clips;
    }
    const std::string defs = readText(dataDir + "/LocalizedComponent/GameTsc/Story/VehiclesDef.tsc");
    const auto params = parseVehiclesParam(readText(dataDir + "/CommonComponent/Param/VehiclesParam.tsc"));
    auto vclassOf = [&](const std::string& variant) {
        std::istringstream in(defs);
        std::string line;
        static const char* const kClasses[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK", "HYDRO", "DRAGSTER"};
        while (std::getline(in, line)) {
            std::istringstream ls(line);
            std::string a, b, c;
            ls >> a >> b >> c;
            if (a == "AddVehicleInfo" && b == "MESH_ID_" + variant)
                for (int k = 0; k < 8; ++k)
                    if (c == kClasses[k]) return k;
        }
        return 0;
    };
    const int hips = rider.bone("Hips");
    std::array<int, 4> freeStill{}, freeDrive{};  // P76: limbs left to the animation (0x10) after 2 s
    int variants = 0, placed = 0, ruleOk = 0, noHead = 0, rotated = 0, ikAll = 0;
    double ikWorst = 0;
    std::vector<double> gripAngle, gripAngleT;  // PK31
    double gripWorst = 0;
    struct Row { std::string label; double still, drive, pelvis; };
    std::map<std::string, std::vector<Row>> byClass;
    std::printf("  %-18s %-14s %-24s %9s %9s %9s %3s %7s %7s\n", "variant", "archetype", "seat (Head bone)", "hips-seat", "driving", "pelvis", "IK", "ikStill", "ikDrive");
    for (const char* name : {"QUAD", "BIKE", "BUGGY", "MUSCLE", "SUV", "TRUCK", "VEH", "BONUS1"}) {
        auto dpc = openDpc(dataDir + "/CommonComponent/DATAS/" + name + ".DPC");
        if (!dpc) continue;
        const VehicleAssembler as(*dpc);
        for (const VehicleVariant& v : as.variants()) {
            const auto P = params.find(v.label);
            if (P == params.end()) continue;
            const std::string arch = vehicleArchetype(defs, v.label);
            if (riderArchetypeIndex(arch) < 0) continue;
            ++variants;
            const AssembledVehicle av = as.assemble(v, 1);
            const auto seat = riderSeatPoint(*dpc, av);
            if (!seat) ++noHead;
            else if (!seat->rotationIdentity) ++rotated;
            const std::array<double, 3> at = seat ? seat->t : std::array<double, 3>{0, 0, 0};
            const std::array<std::array<float, 3>, 4> noIk = {{{NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}}};
            const auto ik = riderIkPoints(*dpc, av);
            int found = 0;
            for (const auto& p : ik) found += std::isnan(p[0]) ? 0 : 1;
            if (found == 4) ++ikAll;
            RiderAnimator still(rider, *tables, arch, vclassOf(v.label), P->second, ik, at), drive(rider, *tables, arch, vclassOf(v.label), P->second, ik, at);
            RiderDrive d;
            d.dt = 1.0f / 60;
            RiderDrive dd = d;
            dd.vel = {0, 0, 10};
            dd.accel = true;
            for (int f = 0; f < 120; ++f) still.update(d), drive.update(dd);
            for (int l = 0; l < 4; ++l) freeStill[size_t(l)] += still.limbFree(l), freeDrive[size_t(l)] += drive.limbFree(l);  // P76
            // the rule: the base riding pose's hips = the archetype pose + the seat point (no sway when standing)
            const std::vector<Mat4> base = rider.pose(tables->slots[size_t(riderArchetypeIndex(arch))][0], 0.0);
            RiderAnimator fresh(rider, *tables, arch, vclassOf(v.label), P->second, noIk, at);
            const Mat4& Hf = fresh.boneWorlds()[size_t(hips)];
            const Mat4& Hb = base[size_t(hips)];
            const double ruleErr = std::max({std::fabs(Hf[12] - Hb[12] - at[0]), std::fabs(Hf[13] - Hb[13] - at[1]), std::fabs(Hf[14] - Hb[14] - at[2])});
            if (ruleErr < 1e-4) ++ruleOk;
            const VehicleSurface vs(*dpc, av);
            const SeatGap gs = seatGap(rider, tmpl, still.boneWorlds(), vs), gd = seatGap(rider, tmpl, drive.boneWorlds(), vs);
            if (gs.hips != HUGE_VAL) ++placed;
            // IK residual: hand / foot bone to its target after 2 s (worst of the four), standing and driving
            double ikS = 0, ikD = 0;
            const int ends[4] = {rider.bone("LeftHand"), rider.bone("RightHand"), rider.bone("LeftFoot"), rider.bone("RightFoot")};
            for (int k = 0; k < 4; ++k) {
                if (std::isnan(ik[size_t(k)][0])) continue;
                auto dist = [&](const std::vector<Mat4>& w) {
                    const Mat4& m = w[size_t(ends[k])];
                    return std::sqrt(std::pow(m[12] - ik[size_t(k)][0], 2) + std::pow(m[13] - ik[size_t(k)][1], 2) + std::pow(m[14] - ik[size_t(k)][2], 2));
                };
                ikS = std::max(ikS, dist(still.boneWorlds()));
                ikD = std::max(ikD, dist(drive.boneWorlds()));
            }
            ikWorst = std::max({ikWorst, found ? ikS : 0.0, found ? ikD : 0.0});
            {  // PK31: the driving hands' animated + IK orientation against the grip boxes (0x604410 then makes them equal)
                const auto frames = riderIkHandFrames(*dpc, av);
                const int hb[2] = {rider.bone("LeftHand"), rider.bone("RightHand")};
                RiderAnimator grip(rider, *tables, arch, vclassOf(v.label), P->second, ik, at);
                grip.setHandFrames(frames);
                for (int f = 0; f < 120; ++f) grip.update(dd);
                for (int k = 0; k < 2; ++k) {
                    if (std::isnan(frames[size_t(k)][0]) || hb[k] < 0) continue;
                    const Mat4& m = drive.boneWorlds()[size_t(hb[k])];
                    double H[9];
                    for (int r = 0; r < 3; ++r) {
                        const double l = std::sqrt(m[size_t(r * 4)] * m[size_t(r * 4)] + m[size_t(r * 4 + 1)] * m[size_t(r * 4 + 1)] + m[size_t(r * 4 + 2)] * m[size_t(r * 4 + 2)]);
                        for (int c = 0; c < 3; ++c) H[r * 3 + c] = m[size_t(r * 4 + c)] / l;
                    }
                    const auto& B = frames[size_t(k)];
                    double tr = 0, trT = 0;  // trace(H B^T) and with B transposed
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c) tr += H[r * 3 + c] * B[size_t(r * 3 + c)], trT += H[r * 3 + c] * B[size_t(c * 3 + r)];
                    gripAngle.push_back(std::acos(std::clamp((tr - 1) / 2, -1.0, 1.0)) * 180 / M_PI);
                    const Mat4& g = grip.boneWorlds()[size_t(hb[k])];
                    double trg = 0;
                    for (int r = 0; r < 3; ++r) {
                        const double l = std::sqrt(g[size_t(r * 4)] * g[size_t(r * 4)] + g[size_t(r * 4 + 1)] * g[size_t(r * 4 + 1)] + g[size_t(r * 4 + 2)] * g[size_t(r * 4 + 2)]);
                        for (int c = 0; c < 3; ++c) trg += g[size_t(r * 4 + c)] / l * B[size_t(r * 3 + c)];
                    }
                    gripWorst = std::max(gripWorst, std::acos(std::clamp((trg - 1) / 2, -1.0, 1.0)) * 180 / M_PI);
                    gripAngleT.push_back(std::acos(std::clamp((trT - 1) / 2, -1.0, 1.0)) * 180 / M_PI);
                }
            }
            auto f = [](double x) {
                char b[16];
                if (x == HUGE_VAL) return std::string("     none");
                std::snprintf(b, sizeof b, "%9.3f", x);
                return std::string(b);
            };
            char seatTxt[48];
            std::snprintf(seatTxt, sizeof seatTxt, "(%.3f %.3f %.3f)%s", at[0], at[1], at[2], seat ? "" : "!");
            std::printf("  %-18s %-14s %-24s %s %s %s %3d %7.3f %7.3f\n", v.label.c_str(), arch.c_str(), seatTxt, f(gs.hips).c_str(), f(gd.hips).c_str(),
                        f(gs.pelvis).c_str(), found, ikS, ikD);
            byClass[v.label.substr(0, v.label.find('_'))].push_back({v.label, gs.hips, gd.hips, gs.pelvis});
        }
    }
    std::printf("  per class (hips above the seat surface, m; standing / driving; pelvis skin gap standing):\n");
    for (auto& [cls, rows] : byClass) {
        std::vector<double> a, b, c;
        for (const Row& r : rows) {
            if (r.still != HUGE_VAL) a.push_back(r.still);
            if (r.drive != HUGE_VAL) b.push_back(r.drive);
            if (r.pelvis != HUGE_VAL) c.push_back(r.pelvis);
        }
        auto med = [](std::vector<double> v) -> double { if (v.empty()) return NAN; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
        auto mx = [](const std::vector<double>& v) -> double { return v.empty() ? NAN : *std::max_element(v.begin(), v.end()); };
        std::printf("    %-7s %2zu variants, %2zu over a seat: standing median %.3f max %.3f; driving median %.3f max %.3f; pelvis median %.3f max %.3f\n",
                    cls.c_str(), rows.size(), a.size(), med(a), mx(a), med(b), mx(b), med(c), mx(c));
    }
    t.check(ruleOk == variants, "rider root at the vehicle's Head bone (all variants)", ruleOk, variants);
    // R6 levelling (0x5d5800, node 25 = Hips): on a 20 degree side slope the player's quad rider keeps the hips'
    // world tilt of flat ground (weight 1); a standing bike rider is not levelled (bikes level only when sliding or
    // reversing, weight 0.5).
    for (const auto& [variant, dpcName, want] : {std::tuple<const char*, const char*, double>{"QUAD_ROAD_2", "QUAD", 0.0},
                                                std::tuple<const char*, const char*, double>{"BIKE_OFF_1", "BIKE", 20.0}}) {
        auto dpc = openDpc(dataDir + "/CommonComponent/DATAS/" + dpcName + ".DPC");
        const auto P = params.find(variant);
        if (!dpc || P == params.end()) continue;
        const VehicleAssembler as(*dpc);
        const VehicleVariant* vv = as.find(variant);
        if (vv == nullptr) continue;
        const AssembledVehicle av = as.assemble(*vv, 1);
        const std::string arch = vehicleArchetype(defs, variant);
        auto hipsTilt = [&](double rollDeg) {
            RiderAnimator a(rider, *tables, arch, vclassOf(variant), P->second, riderIkPoints(*dpc, av), riderSeatPoint(*dpc, av)->t);
            RiderDrive d;
            d.dt = 1.0f / 60;
            const double h = rollDeg * M_PI / 360;
            d.q = {0, 0, float(std::sin(h)), float(std::cos(h))};
            for (int f = 0; f < 240; ++f) a.update(d);
            const Mat4& m = a.boneWorlds()[size_t(hips)];
            // hips up axis (row 1) in model space, then rolled into the world (row-vector, about z)
            const double ux = m[4], uy = m[5], uz = m[6], c = std::cos(2 * h), sn = std::sin(2 * h);
            const double wx = ux * c - uy * sn, wy = ux * sn + uy * c;
            (void)uz;
            return std::atan2(wx, wy) * 180 / M_PI;  // roll of the hips' up axis in the world x-y plane
        };
        const double flat = hipsTilt(0), slope = hipsTilt(20);
        char what[96];
        std::snprintf(what, sizeof what, "%s hips tilt change on a 20 deg slope (deg)", variant);
        t.near(what, std::fabs(slope - flat), want, 1.0);
    }
    std::printf("  IK targets (named boxes IK_LEFTHAND/RIGHTHAND/LEFTFOOT/RIGHTFOOT): all four on %d of %d variants\n", ikAll, variants);
    if (!gripAngle.empty()) {  // PK31
        std::sort(gripAngle.begin(), gripAngle.end());
        std::sort(gripAngleT.begin(), gripAngleT.end());
        std::printf("  hand vs grip box orientation before 0x604410 (%zu hands): median %.1f deg (box read transposed: %.1f deg)\n", gripAngle.size(),
                    gripAngle[gripAngle.size() / 2], gripAngleT[gripAngleT.size() / 2]);
        t.check(gripWorst < 0.05, "0x604410: driving hands take their grip box's orientation (worst deg)", gripWorst, 0);
    }
    t.check(ikAll > 0, "IK target boxes found on the vehicles", ikAll, variants);
    std::printf("  limbs left to the animation after 2 s (LH RH LF RF): standing %d %d %d %d, driving %d %d %d %d\n", freeStill[0], freeStill[1],
                freeStill[2], freeStill[3], freeDrive[0], freeDrive[1], freeDrive[2], freeDrive[3]);
    t.check(freeDrive[0] + freeDrive[1] + freeDrive[2] + freeDrive[3] == 0, "driving riders keep all four limbs on the IK (riding loops: 0x11 at 0 s)",
            freeDrive[0] + freeDrive[1] + freeDrive[2] + freeDrive[3], 0);
    t.check(noHead == 0 && rotated == 0, "every garage skeleton has an unrotated Head bone", noHead + rotated, 0);
    // BUGGY_OFF_5, BUGGY_ALL_1 and FLOTTE_JEEP_1 have no drawn mesh under the pelvis (open seat frames)
    t.check(placed >= variants - 3, "riders over a seat surface (all but 3)", placed, variants - 3);
}

// P79 (notes p79-objects-move.md): PhysAgent_Z's UserDefine table (0x740e80) and the points of interest of
// DiscoveryPointVisibleObject (0x4ed2d0) with their record placement (0x4fbf10).
void objectChecks(const std::string& dataDir, Tally& t) {
    std::printf("Objects (0x740e80 UserDefine, 0x4ed2d0 points of interest, 0x4fbf10 placement):\n");
    {
        const DynObjUserDefine u = parseDynObjUserDefine({"massevolumique 3", "Delay 250", "EXPLOFX FX_Boom", "replace Crate_Broken",
                                                          "DAMAGE", "explotouch", "MinPower 12", "BarriereBois01_DYN00"});
        t.near("MASSEVOLUMIQUE 3 (any case) -> 10", u.massVolumique, 10.0);
        t.near("DELAY 250 -> +0xe28 0.25 s", u.explodeDelay, 0.25, 1e-6);
        t.near("MINPOWER 12 -> +0xe2c / +0xe30", u.explodeDamage, 12.0);
        t.near("DAMAGE without value -> 1.0", u.damage, 1.0);
        t.check(u.explodeFx == "FX_Boom" && u.replaceWith == "Crate_Broken" && u.explodeOnTouch, "EXPLOFX / REPLACE names, EXPLOTOUCH",
                u.explodeOnTouch, 1);
        t.check(u.names.size() == 1 && u.names[0] == "BarriereBois01_DYN00", "only the piece name is left as a name", double(u.names.size()), 1);
        const DynObjUserDefine fx = parseDynObjUserDefine({"EXPLOFX FX_A"});
        t.near("EXPLOFX alone: delay 0.01 s", fx.explodeDelay, 0.01, 1e-7);
        t.near("DISAPPEAREXPLO without value: 0.5 s", parseDynObjUserDefine({"DISAPPEAREXPLO"}).explodeDelay, 0.5);
        // PK09: 0x7b1a80 (strnicmp prefix, '_'-separated %d values >= 0) and AnimatedAgent_Z +0x40 = first^2.
        const std::vector<int> d = parseDistActive({"Vautour", "distactive_120_x_300", "DISTACTIVE_-5"});
        t.check(d == std::vector<int>{120, 300}, "0x7b1a80: distactive_120_x_300 -> 120 300 (x and -5 skipped)", double(d.size()), 2);
        t.near("+0x40 = DISTACTIVE^2; none -> IT_DYNBASE's 5625 ([0xa0521c])",
               distActive2({"DISTACTIVE_1000"}) + distActive2({"HIMMOBILE"}, kDynActiveDist2), 1e6 + 5625.0);
    }
    {
        std::vector<PoiPoint> p(5);
        p[0] = {3, 1000, 0, false, "VP_EDGE"};      // d^2 == 1e6: inside
        p[1] = {4, 0, 1000.1f, false, "TP_OUT"};    // outside
        p[2] = {2, 10, 10, false, "LIVERY"};        // the painted vehicle: not drawn by the port
        p[3] = {4, -300, 400, true, "TP_TAKEN"};    // teleports ignore taken
        p[4] = {3, 5, 5, true, "VP_VISITED"};       // vistas stay
        std::vector<RaceMarker> m;
        poiMarkers(p, 0, 0, m);
        t.check(m.size() == 3 && m[0].index == 0 && m[1].index == 3 && m[2].index == 4, "0x4ed2d0: within 1000 m (edge in), no livery", double(m.size()), 3);
        if (m.size() == 3)
            t.check(m[0].type == 16 && std::string(m[0].cls) == "IT_DISCOVERY" && m[0].upright && std::string(m[1].cls) == "CAMIONHUB_RESPAWN" &&
                        !m[1].upright && m[0].text == "VP_EDGE",
                    "record type 0x10, classes, +0x6f only for the vista, name as text", m[0].type, 16);
        RaceMarker r;
        r.upright = false, r.lift = 0, r.pos = {1, 0, 2};
        const float s = std::sqrt(0.5f);
        const auto w = raceMarkerWorld(r, 10, {s, s, 0});
        t.vec("0x4fbf10 tilt: up row = ground normal (45 deg in x)", std::array<double, 3>{w[4], w[5], w[6]}.data(), {s, s, 0}, 1e-6);
        t.vec("0x4fbf10 tilt: position", std::array<double, 3>{w[12], w[13], w[14]}.data(), {1, 10, 2}, 1e-6);
    }
    std::ifstream f(dataDir + "/LocalizedComponent/GameTsc/Story/poi.tsc", std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::vector<PoiPoint> pois = parsePoiScript(ss.str());
    size_t tp = 0, vp = 0;
    for (const PoiPoint& p : pois) (p.type == 4 ? tp : vp) += 1;
    t.check(tp == 190 && vp == 95, "poi.tsc: 190 teleports, 95 vista points (p44)", double(tp * 1000 + vp), 190095);
    if (auto usa = openDpc(dataDir + "/CommonComponent/WORLD/USA1.DPC")) {
        // IT_DISCOVERY is a Lod of that name. CAMIONHUB_RESPAWN (clone.tsc: an Agent_Z class) names no resource in any
        // shipped DPC (notes P3): 0x730820 finds no template, the teleport records draw nothing.
        bool disc = false, camion = false;
        float discDist2 = -1;
        for (uint32_t id : usa->ids(cls::Lod)) try {
                const uint32_t n = parseObjectLink(usa->link(*usa->find(id))).name;
                disc |= n == asoboCrc("IT_DISCOVERY");
                if (n == asoboCrc("IT_DISCOVERY") && discDist2 < 0) {  // PK09: its UserDefine's DISTACTIVE (vf22 0x667220)
                    const Lod lod = parseLodBody(*usa->body(*usa->find(id)));
                    if (const DpcFile::Entry* u = lod.userDefine ? usa->find(lod.userDefine) : nullptr; u && u->header)
                        if (const auto body = usa->body(*u)) discDist2 = distActive2(parseUserDefineBody(*body), 0);
                }
                camion |= n == asoboCrc("CAMIONHUB_RESPAWN");
            } catch (const std::exception&) {
            }
        t.check(disc && !camion, "USA1: an IT_DISCOVERY Lod, no CAMIONHUB_RESPAWN Lod", disc + !camion, 2);
        t.near("USA1 IT_DISCOVERY: DISTACTIVE_1000 -> +0x40 = 1e6 (the 0x4ed2d0 edge)", discDist2, 1e6);
    }
}

// PK13 (notes/pk13.md): RTC attach slot easing (0x5148a0), node FX (0x539140), the 0xea overlay father, Play/End flags,
// and a census of the shipped RTC events the package decides on (POSPERSO 0x9f, SETDOF 0x95, STARTRTCFX 0xa3).

// PK31 (notes/pk31.md): the AI racer lean 0x61c1b0 (springs 0x61c0c0, Quat_Mul 0x410260, axis-angle 0x40f8b0, rotate
// 0x410510 and ExpSmooth 0x4100b0 run for real) under EmuRun (notes/pk31-work/lean.cmd): handles faked in emulator
// memory, three runs of consecutive steps; want = the racer's +0x1128..+0x113c (x0 v0 x1 v1 x2 v2) and +0x119c.
void pk31LeanChecks(Tally& t) {
    std::printf("PK31 AI lean (0x61c1b0):\n");
    struct Step {
        std::array<float, 3> acc;
        float dt;
        std::array<double, 6> xv;
        std::array<double, 4> q;
    };
    struct Run {
        const char* id;
        bool bike;
        std::array<float, 4> orient;
        std::array<float, 3> x0, v0;
        std::vector<Step> steps;
    };
    const float h = float(std::sin(0.3));
    const std::array<float, 4> q0{0.1f, h, -0.05f, float(std::sqrt(1 - 0.01 - double(h) * h - 0.0025))};
    const Run runs[] = {
        {"LN1", true, q0, {0, 0, 0}, {0, 0, 0},
         {{{3, -1, 5}, 1.0f / 60, {-0.03212548, -1.9275287, 0.0019352344, 0.11611406, -9.6299604E-4, -0.05777976}, {-2.2197256E-4, -1.8632479E-6, 1.7226518E-4, 0.99999994}},
          {{-8, 0.5f, 2}, 1.0f / 60, {-0.047710586, -0.9351064, 0.045674443, 2.6243525, -0.022723682, -1.305641}, {-1.526435E-4, -2.6567912E-4, 5.2687014E-4, 0.9999999}},
          {{-8, 0.5f, 2}, 1.0f / 60, {-0.047037292, 0.04039765, 0.13008156, 5.064427, -0.06462023, -2.5137928}, {2.0106904E-4, -7.835585E-4, 0.0010583949, 0.9999991}}}},
        {"LN2", false, q0, {1.5f, -5, 10}, {30, -200, 0},
         {{{12, 0, -30}, 0.05f, {2.0, 44.100784, -6.0, -160.88576, 10.3883295, 7.7665887}, {-0.0051596942, 0.1003437, -0.035815068, 0.99429464}},
          {{0, 0, 0}, 0.05f, {2.0, 40.614124, -6.0, -146.63512, 10.506083, 2.3550513}, {-0.0050547677, 0.10134465, -0.03563032, 0.9942003}}}},
        {"LN3", true, {0, 0, 0, 1}, {3.4f, 44, -44.9f}, {0, 1e-7f, -100},
         {{{0, -60, 40}, 1.0f / 30, {2.5093272, -26.720184, 43.534622, -13.961372, -45.0, -80.17293}, {-0.12309247, -0.36281386, 0.3503094, 0.85469157}}}},
    };
    for (const Run& r : runs) {
        AiLean L;
        L.x = r.x0, L.v = r.v0;
        int k = 0;
        for (const Step& s : r.steps) {
            L.step(s.dt, s.acc, r.orient, r.bike);
            double got[10];
            for (int i = 0; i < 3; ++i) got[2 * i] = L.x[size_t(i)], got[2 * i + 1] = L.v[size_t(i)];
            for (int i = 0; i < 4; ++i) got[6 + i] = L.q[size_t(i)];
            double err = 0;
            for (int i = 0; i < 6; ++i) err = std::max(err, std::fabs(got[i] - s.xv[size_t(i)]) / std::max(1.0, std::fabs(s.xv[size_t(i)])));
            for (int i = 0; i < 4; ++i) err = std::max(err, std::fabs(got[6 + i] - s.q[size_t(i)]));
            t.check(err <= 2e-6, std::string(r.id) + " step " + std::to_string(++k) + ": springs and lean quat (rel err)", err, 0);
        }
    }
    // 0x5c2820 / 0x5c2370 on a synthetic swap: racer 1 (x 2, heading +z) passes racer 0 (x 0, 1 m behind it, on its left).
    {
        RankReactions rr;
        std::vector<RankReactions::Rec> recs = {{true, 0, -1, 0, 1}, {true, 2, 0, 0, 1}, {true, 0, -40, 0, 1}};
        const auto g0 = rr.update(1.0f / 60, {0, 1, 2}, recs);
        const auto g1 = rr.update(1.0f / 60, {1, 0, 2}, recs);  // the list is last frame's sort: nothing yet
        const auto g2 = rr.update(1.0f / 60, {1, 0, 2}, recs);
        t.check(g0.empty() && g1.empty() && g2.size() == 2 && g2[0].racer == 1 && g2[0].base == 21 && g2[1].racer == 0 && g2[1].base == 33,
                "overtake: passer 21 (s > 0, gained), passed 33 (s <= 0, lost)", g2.size() == 2 ? g2[0].base * 100 + g2[1].base : -1, 2133);
        const auto g3 = rr.update(1.0f / 60, {0, 1, 2}, recs), g4 = rr.update(1.0f / 60, {0, 1, 2}, recs);
        t.check(g3.empty() && g4.empty(), "overtake: 1 s cooldown (+0x17c) blocks the swap back", double(g3.size() + g4.size()), 0);
        for (int k = 0; k < 60; ++k) rr.update(1.0f / 60, {0, 1, 2}, recs);
        const auto g5 = rr.update(1.0f / 60, {1, 0, 2}, recs), g6 = rr.update(1.0f / 60, {1, 0, 2}, recs);
        t.check(g5.empty() && g6.size() == 2, "overtake: gestures again after the cooldown", double(g6.size()), 2);
        recs[1].x = 30;
        for (int k = 0; k < 70; ++k) rr.update(1.0f / 60, {0, 1, 2}, recs);
        t.check(rr.update(1.0f / 60, {1, 0, 2}, recs).empty() && rr.update(1.0f / 60, {1, 0, 2}, recs).empty(), "overtake: none beyond 25 m", 0, 0);
    }
    // Quat_Mul 0x410260(ECX = a, EDX = b) = b x a (Hamilton): yaw 90 then roll 90.
    const float s2 = float(std::sqrt(0.5));
    const auto m = AiLean::quatMul({0, s2, 0, s2}, {0, 0, s2, s2});
    t.check(std::fabs(m[0] + 0.5f) < 1e-7 && std::fabs(m[1] - 0.5f) < 1e-7 && std::fabs(m[2] - 0.5f) < 1e-7 && std::fabs(m[3] - 0.5f) < 1e-7,
            "0x410260(a, b) = b x a (yaw 90, roll 90)", m[0], -0.5);
}

// PK31: IK_SolveJacobian 0x601220 under EmuRun (notes/pk31-work/ik.cmd; FK 0x66bc40 stubbed, so the effector and the
// world frames stay put and all 15 passes run; Quat_Mul / RotateVec / atan / inverse / normalise for real). Want =
// the two nodes' local quaternions and the lock bytes after the call (AL = 1 in all three).
void pk31IkChecks(Tally& t) {
    std::printf("PK31 rider IK (0x601220):\n");
    using Q4 = std::array<double, 4>;
    using V = std::array<double, 3>;
    auto nq = [](Q4 q) {
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (double& c : q) c = float(c / n);
        return q;
    };
    struct Case {
        const char* id;
        Q4 lq[2], wq[2];
        V pos[2], eff;
        V axes[3];
        bool en[3];
        double lo[3], hi[3];
        V tgt;
        Q4 want[2];
        bool lock[3];
    };
    const Case cs[] = {
        {"IK1", {nq({0.1, 0.6, 0.05, 0.79}), nq({-0.2, 0.1, 0.3, 0.92})}, {nq({0.15, -0.3, 0.1, 0.93}), nq({0.05, 0.2, -0.1, 0.97})},
         {{0.3, 0.02, 0.05}, {0, 0, 0}}, {0.55, -0.1, 0.12}, {{0, 0.8, 0.6}, {0.6, 0, 0.8}, {0, 0, 1}}, {true, true, false},
         {0.1, -10, -10}, {3, 0, 10}, {0.3, 0.35, 0.2},
         {{0.114877716, 0.626861, 0.07567901, 0.7668905}, {-0.09771366, 0.20385092, 0.8087692, 0.54294485}}, {false, true, false}},
        {"IK2 (J^T J singular: J^T e)", {nq({0, 0, 0, 1}), nq({0, 0.3, 0, 0.95})}, {nq({0, 0, 0, 1}), nq({0, 0, 0, 1})},
         {{0.3, 0, 0}, {0, 0, 0}}, {0.6, 0, 0}, {{0, 1, 0}, {0, 0, 1}, {0, 0, 1}}, {false, false, false},
         {-10, -10, -10}, {10, 10, 10}, {0.4, 0.3, 0.1},
         {{0.0, -0.10406606, 0.0, 0.9945704}, {0.28462622, 0.09832615, 0.90131634, 0.3113659}}, {false, false, false}},
        {"IK3", {nq({0.3, 0.1, 0, 0.95}), nq({0, 0, 0.4, 0.9})}, {nq({0.3, 0.1, 0, 0.95}), nq({0, 0, 0.4, 0.9})},
         {{0, -0.4, 0.1}, {0, 0, 0}}, {0.05, -0.8, 0.2}, {{1, 0, 0}, {0, 0, 1}, {0, 1, 0}}, {true, false, false},
         {0.1, -10, -10}, {3, 10, 10}, {0.3, -0.6, 0.5},
         {{0.20069179, 0.09934811, 0.010247635, 0.97455}, {-0.25523958, 0.1480522, 0.78225595, 0.54864275}}, {true, false, false}},
    };
    for (const Case& c : cs) {
        Q4 lq[2] = {c.lq[0], c.lq[1]};
        IkJoint J[3];
        for (int j = 0; j < 3; ++j) J[j].axis = {float(c.axes[j][0]), float(c.axes[j][1]), float(c.axes[j][2])}, J[j].limited = c.en[j], J[j].lo = c.lo[j], J[j].hi = c.hi[j];
        const int node[3] = {0, 1, 1};
        IkAccess io;
        io.local = [&](int j) -> Q4& { return lq[node[j]]; };
        io.worldQ = [&](int j) { return c.wq[node[j]]; };
        io.jointPos = [&](int j) { return V{float(c.pos[node[j]][0]), float(c.pos[node[j]][1]), float(c.pos[node[j]][2])}; };
        io.effector = [&] { return V{float(c.eff[0]), float(c.eff[1]), float(c.eff[2])}; };
        io.fk = [] {};
        int passes = 0;
        bool locked[3] = {};
        const bool al = ikSolveJacobian(J, 3, io, {float(c.tgt[0]), float(c.tgt[1]), float(c.tgt[2])}, &passes, locked);
        double err = 0;
        for (int k = 0; k < 2; ++k)
            for (int i = 0; i < 4; ++i) err = std::max(err, std::fabs(lq[k][size_t(i)] - c.want[k][size_t(i)]));
        const bool locks = locked[0] == c.lock[0] && locked[1] == c.lock[1] && locked[2] == c.lock[2];
        t.check(al && passes == 15 && locks && err < 2e-5, std::string(c.id) + ": 15 passes, local quats, locks", err, 0);
    }
}

void pk13Checks(const std::string& dataDir, Tally& t) {
    std::printf("PK13 RTC details:\n");
    {  // 0x5148a0: blend 0 snaps at once (TELEPORT_HELICO's 0xf1 c = 0)
        rtc::AttachSlot s;
        s.attach(0.0f);
        Vec3 p;
        Quat q;
        s.step(1.0f / 60, {1, 2, 3}, {0, 0, 0, 1}, {10, 20, 30}, {0, 1, 0, 0}, p, q);
        t.check(s.done && p[0] == 1 && p[2] == 3 && q[3] == 1, "attach blend 0: the node pose on the first step (0x5148a0)", p[0], 1);
    }
    {  // blend 1 s at 60 Hz: step 0 = the latched start, then linear in 1 - e/d, snapped once e >= d
        rtc::AttachSlot s;
        s.attach(1.0f);
        Vec3 p;
        Quat q;
        const Quat yaw90 = {0, std::sin(M_PI / 4), 0, std::cos(M_PI / 4)};
        s.step(1.0f / 60, {0, 0, 0}, {0, 0, 0, 1}, {6, 0, 0}, yaw90, p, q);
        t.check(!s.done && p[0] == 6 && std::fabs(q[1] - yaw90[1]) < 1e-9, "attach blend 1: first step = the vehicle's latched pose", p[0], 6);
        float e = 1.0f / 60;
        Vec3 mid{};
        Quat midQ{};
        for (int i = 1; i < 30; ++i) s.step(1.0f / 60, {0, 0, 0}, {0, 0, 0, 1}, {99, 99, 99}, {0, 0, 0, 1}, mid, midQ), e += 1.0f / 60;
        // elapsed before the 30th step = 29 x (1/60) as float sums
        float el = 0;
        for (int i = 0; i < 29; ++i) el += 1.0f / 60;
        const double want = double(0.0f + (6.0f - 0.0f) * (1.0f - el / 1.0f));
        t.near("attach blend 1: pos = node + (start - node)(1 - e/d) (start latched, not live)", mid[0], want, 1e-6);
        const Quat ws = slerp({0, 0, 0, 1}, yaw90, 1.0f - el);
        t.near("attach blend 1: quat = slerp(node, start, 1 - e/d) (0x431010)", midQ[1], ws[1], 1e-9);
        int steps = 30;
        while (!s.done && steps < 200) s.step(1.0f / 60, {0, 0, 0}, {0, 0, 0, 1}, {99, 99, 99}, {0, 0, 0, 1}, mid, midQ), ++steps;
        t.check(s.done && mid[0] == 0 && steps >= 60 && steps <= 62, "attach blend 1: node pose once elapsed >= 1 s (fcomp C0|C3)", steps, 61);
    }
    {  // EmuRun vector (pk13.md): 0x5148a0 with duration 1, elapsed 0.5, start (6, 2, -4) yaw 90 latched, node at the origin
        rtc::AttachSlot s;
        s.attach(1.0f);
        s.elapsed = 0.5f, s.start = {6, 2, -4}, s.startQ = {0, 0.70710678f, 0, 0.70710678f};
        Vec3 p;
        Quat q;
        s.step(0.016666668f, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0, 1}, p, q);
        t.check(p[0] == 3 && p[1] == 1 && p[2] == -2 && std::fabs(q[1] - 0.38268343) < 1e-7 && std::fabs(q[3] - 0.92387956) < 1e-7 && std::fabs(s.elapsed - 0.51666665f) < 1e-7,
                "0x5148a0 EmuRun: (3, 1, -2), quat (0, 0.38268343, 0, 0.92387956)", q[1], 0.38268343);
    }
    {  // quatOfRows round trip and the 0xea overlay projection (FOV 90 item camera, father at z = 1)
        const Quat q0 = {0.1, -0.7, 0.3, 0.64};
        const double n = std::sqrt(0.01 + 0.49 + 0.09 + 0.64 * 0.64);
        const Mat4 m = localMatrix({2.5, 2.5, 2.5}, q0, {1, 2, 3});
        const Quat q = rtc::quatOfRows(m);
        double err = 0;
        for (size_t i = 0; i < 4; ++i) err = std::max(err, std::fabs(q[i] - q0[i] / n));
        t.check(err < 1e-9, "quatOfRows: rotation of a scaled row-vector matrix", err, 0);
        const double fov = 1.2;  // a game camera's horizontal FOV (4:3)
        const Mat4 f = rtc::overlayFather({10, 20, 30}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, fov);
        const Vec3 local = {0.3, -0.2, 0.4};  // a point of the RTC under the father
        const double wx = local[0] * f[0] + local[1] * f[4] + local[2] * f[8] + f[12], wy = local[0] * f[1] + local[1] * f[5] + local[2] * f[9] + f[13],
                     wz = local[0] * f[2] + local[1] * f[6] + local[2] * f[10] + f[14];
        const double sx = (wx - 10) / ((wz - 30) * std::tan(fov / 2)), sy = (wy - 20) / ((wz - 30) * std::tan(fov / 2));
        const double ix = local[0] / ((local[2] + 1) * std::tan(rtc::kOverlayFov / 2)), iy = local[1] / ((local[2] + 1) * std::tan(rtc::kOverlayFov / 2));
        t.check(std::fabs(sx - ix) < 1e-12 && std::fabs(sy - iy) < 1e-12, "0xea overlay: game-camera projection = the FOV-90 item's (port_play)", sx, ix);
    }
    t.check(rtc::nodeFxOp(0x20) == rtc::NodeFxOp::Start && rtc::nodeFxOp(0x21) == rtc::NodeFxOp::None && rtc::nodeFxOp(0x22) == rtc::NodeFxOp::Hide,
            "node FX: 0x20 start, 0x21 no handler, 0x22 hide (vf44 0x5389d0)", 1, 1);
    t.check(rtc::PlayFlags::clearsRadialBlurAtPlay(0x43) && rtc::PlayFlags::clearsRadialBlurAtPlay(0x1fb) && !rtc::PlayFlags::clearsRadialBlurAtPlay(0x40) &&
                rtc::PlayFlags::restoresLdrSoftenAtEnd(0x1fb) && rtc::PlayFlags::restoresLdrSoftenAtEnd(0x43) && !rtc::PlayFlags::restoresLdrSoftenAtEnd(0x47),
            "Play flag 2 (TR_DEATH 0x43, TELEPORT_HELICO 0x1fb) / CAMERA_CAME 4 (0x53b6b0 / 0x53b8c0)", 1, 1);
    if (auto f = openDpc(dataDir + "/CommonComponent/RTC/TELEPORT_HELICO.DPC")) {
        rtc::Rtc r;
        if (rtcOf(*f, r, nullptr)) {
            int f1 = 0, f2 = 0, fx20 = 0, fx21 = 0;
            for (size_t n = 0; n < r.nodes.size(); ++n)
                for (const rtc::EventKey& k : r.nodes[n].events)
                    for (const rtc::Msg& m : k.msgs) {
                        if (m.type == 0xf1 && r.nodes[n].e == 307 && k.time == 0 && m.c == 0) ++f1;
                        if (m.type == 0xf2 && r.nodes[n].e == 307 && std::fabs(k.time - r.duration) < 1e-4) ++f2;
                        if (m.type == 0x20 && r.nodes[n].e == 306 && k.time == 0 && m.d == 0x4ee0b306u) ++fx20;
                        if (m.type == 0x21 && r.nodes[n].e == 306 && std::fabs(k.time - 15) < 1e-4 && m.d == 0x4ee0b306u) ++fx21;
                    }
            t.check(f1 == 1 && f2 == 1, "TELEPORT_HELICO: 0xf1 on A307 at 0 s blend c = 0, 0xf2 at the end", f1 + f2, 2);
            t.check(fx20 == 1 && fx21 == 1, "TELEPORT_HELICO: 0x20 / 0x21 on A306 (0 s / 15 s), data 4ee0b306", fx20 + fx21, 2);
        }
    }
    if (auto usa = openDpc(dataDir + "/CommonComponent/WORLD/USA1.DPC")) {  // 0x730370: the resource keyed 4ee0b306
        int found = 0;
        size_t elements = 0;
        for (uint32_t id : usa->ids(0xc56963de)) {
            const DpcFile::Entry* e = usa->find(id);
            const std::vector<uint8_t> link = usa->link(*e);
            uint32_t key = 0;
            if (link.size() >= 4) std::memcpy(&key, link.data(), 4);
            if (key != 0x4ee0b306u) continue;
            ++found;
            try {
                elements = particles::parseParticles(usa->body(*e).value(), link).elements.size();
            } catch (const std::exception&) {
            }
        }
        t.check(found == 1 && elements == 1, "USA1: one Particles_Z keyed 4ee0b306 (rotor dust), 1 element", found, 1);
    }
    {  // census over every shipped RTC: what POSPERSO_B / SETDOF / STARTRTCFX would act on
        std::map<uint32_t, int> count;
        int files = 0;
        std::error_code ec;
        for (const auto& de : std::filesystem::directory_iterator(dataDir + "/CommonComponent/RTC", ec)) {
            auto f = openDpc(de.path().string());
            rtc::Rtc r;
            if (!f || !rtcOf(*f, r, nullptr)) continue;
            ++files;
            auto add = [&](const std::vector<rtc::EventKey>& ev) {
                for (const rtc::EventKey& k : ev)
                    for (const rtc::Msg& m : k.msgs) ++count[m.type];
            };
            add(r.events);
            for (const rtc::Node& n : r.nodes) add(n.events);
        }
        std::printf("  RTC event types over %d files:", files);
        for (const auto& [k, v] : count) std::printf(" %x:%d", k, v);
        std::printf("\n");
        t.check(files == 26 && count[0x9f] == 0 && count[0x95] == 0 && count[0xa3] == 0,
                "26 RTCs: no POSPERSO 0x9f, SETDOF 0x95, STARTRTCFX 0xa3 event", count[0x9f] + count[0x95] + count[0xa3], 0);
    }
}

}  // namespace


// PK29: PlayAnim_Z's world-object passes (0x7bc150 / 0x7bc3d0 loop markers, 0x7bc5c0 events, 0x7a6ce0 visibility,
// 0x813e50 material tracks) on the shipped USA1 animations.
void pk29Checks(const std::string& dataDir, Tally& t) {
    std::printf("PK29 world animation passes (USA1.DPC):\n");
    auto usa = openDpc(dataDir + "/CommonComponent/WORLD/USA1.DPC");
    if (!usa) {
        t.check(false, "USA1.DPC", 0, 1);
        return;
    }
    std::map<uint32_t, std::unique_ptr<Clip>> clips;
    for (uint32_t id : usa->ids(cls::Animation)) {
        auto c = std::make_unique<Clip>();
        if (c->parse(usa->body(*usa->find(id)).value_or(std::vector<uint8_t>{}))) clips[id] = std::move(c);
    }
    auto clip = [&](uint32_t id) -> const Clip* {
        const auto it = clips.find(id);
        return it == clips.end() ? nullptr : it->second.get();
    };
    // Census: HIPS loop markers (0x74a0b0 reads bone id 0x19 only), event kinds, visibility / material tracks.
    int hipsMarked = 0, vis = 0, mat = 0;
    std::map<uint32_t, int> kinds;
    std::set<uint32_t> fxKeys;
    for (const auto& [id, c] : clips) {
        const int h = c->track(kHipsCrc);
        std::vector<std::pair<float, Clip::Msg>> ev;
        if (h >= 0) c->eventWindow(h, 0, c->duration, c->duration, ev);
        bool m = false;
        for (const auto& e : ev) m = m || e.second.kind == 0xc || e.second.kind == 0xd;
        hipsMarked += m;
        for (size_t ti = 0; ti < c->tracks.size(); ++ti) {
            ev.clear();
            c->eventWindow(int(ti), 0, c->duration, c->duration, ev);
            for (const auto& e : ev) {
                ++kinds[e.second.kind];
                if (e.second.kind == 0x20) fxKeys.insert(e.second.d);
            }
        }
        vis += int(c->visTracks.size()), mat += int(c->matTracks.size());
    }
    std::printf("  event kinds:");
    for (const auto& [k, n] : kinds) std::printf(" %x:%d", k, n);
    std::printf("\n");
    t.check(hipsMarked == 9, "9 USA1 anims with 0xc / 0xd on HIPS (0x7bc3d0)", hipsMarked, 9);
    int other = 0;
    for (const auto& [k, n] : kinds) other += (k != 0x20 && k != 0x24 && k != 0xc && k != 0xd && k != 0x10 && k != 0x11) ? n : 0;
    t.check(other == 0, "USA1 event kinds: only 0x20 / 0x24 act on a world object", other, 0);
    t.check(vis == 58 && mat == 152, "USA1 visibility / material tracks 58 / 152", vis * 1000 + mat, 58152);
    int resolved = 0;  // 0x20's d: the Particles_Z whose link word 0 is that key (0x4ee270 -> 0x730370)
    for (uint32_t key : fxKeys)
        for (uint32_t id : usa->ids(0xc56963de)) {
            const std::vector<uint8_t> link = usa->link(*usa->find(id));
            uint32_t k = 0;
            if (link.size() >= 4) std::memcpy(&k, link.data(), 4);
            if (k == key) {
                ++resolved;
                break;
            }
        }
    t.check(fxKeys.size() == 7 && resolved == 7, "0x20 FX keys -> USA1 Particles_Z (7 of 7)", resolved, 7);

    // 0x7bc3d0 on 63094310 (slot 334 of Lod 2154bb41): once to 6.333, then [6.0, 6.333] for ever (count 0).
    if (const Clip* c = clip(0x63094310)) {
        AnimChannel ch;
        ch.play(c);
        std::vector<AnimPass> ps;
        bool finished = false;
        double minT = 1e9, maxT = -1;
        for (int i = 0; i < 60 * 20; ++i) {
            ps.clear();
            finished = ch.advance(1.0 / 60, ps) || finished;
            if (i > 60 * 7) minT = std::min(minT, ch.time()), maxT = std::max(maxT, ch.time());
        }
        t.near("63094310 loop start (0xc at 6.0)", ch.loopStart(), 6.0, 1e-4);
        t.check(!finished && ch.running() && (ch.flags() & 6) == 6, "63094310 never ends: flags 2 | 4", ch.flags(), 6);
        t.check(minT >= 6.0 - 1e-6 && maxT <= c->duration + 1e-6 && maxT - minT > 0.25, "63094310 after 7 s: t in [6.0, 6.333]", maxT - minT, 0.333);
    } else {
        t.check(false, "63094310", 0, 1);
    }
    // Without markers a one-shot stops at its end (vt+0x5c(0)); a looping idle wraps and sees its t = 0 event again.
    if (const Clip* c = clip(0xbb68096c)) {  // markers on bone db92cd3d, not HIPS: not read
        AnimChannel ch;
        ch.play(c);
        std::vector<AnimPass> ps;
        bool fin = false;
        for (int i = 0; i < 60 * 6 && !fin; ++i) ps.clear(), fin = ch.advance(1.0 / 60, ps);
        t.check(fin && !ch.running() && std::fabs(ch.time() - c->duration) < 1e-9, "bb68096c (0xc / 0xd off HIPS) stops at 4.333", ch.time(), c->duration);
        // 0x7a6ce0 / 0x7a6de0: the group 369b41e0 (Lod 020471dc's only mesh) shown, hidden from 3.867 (hold keys)
        const bool ok = c->visTracks.size() == 1 && c->visTracks[0].name == 0x369b41e0;
        t.check(ok && c->visValue(0, 3.85) == 1 && c->visValue(0, 3.9) == 0 && c->visValue(0, 5.0) == 0 && c->visLastValue(0) == 0,
                "bb68096c hides 369b41e0 at 3.867 s (last key 0)", ok ? c->visValue(0, 3.9) : -1, 0);
    }
    if (const Clip* c = clip(0x5a1f071c)) {  // slot 0 of 2154bb41 / 65722f15: 0x20 FX 54920da3 at t = 0, loops 1 s
        AnimChannel ch;
        ch.play(c);
        std::vector<AnimPass> ps;
        int fx = 0;
        for (int i = 0; i < 60 * 3; ++i) {
            ps.clear();
            ch.advance(1.0 / 60, ps);
            for (const AnimPass& p : ps)
                for (size_t ti = 0; ti < c->tracks.size(); ++ti) {
                    std::vector<std::pair<float, Clip::Msg>> ev;
                    c->eventWindow(int(ti), p.from, p.time, p.end, ev);
                    for (const auto& e : ev) fx += e.second.kind == 0x20 && e.second.d == 0x54920da3;
                }
        }
        t.check(fx == 4, "5a1f071c 3 s: 0x20 at t 0 at the start + 3 wraps (0x73b590)", fx, 4);
    }
    // 0x72f620: hold keys (flag 8) and the 'before the first key -> last value' cursor rule.
    if (const Clip* c = clip(0x2589076d)) t.check(c->visValue(0, 0.1) == 1 && c->visLastValue(0) == 1, "2589076d one key 1.0 at 0.667", c->visValue(0, 0.1), 1);

    // 0x813e50 census: every class (slot 0 / 334) material track resolved like the instance list (the skin's materials
    // with that name) and compared with the Material_Z values it would overwrite.
    int tracks = 0, resolvedM = 0, changes = 0;
    for (uint32_t lodId : usa->ids(cls::Lod)) {
        SkinnedModel m;
        std::string err;
        if (!m.load({usa.get()}, lodId, &err)) continue;
        std::map<uint32_t, std::vector<uint32_t>> byName;  // the skin's materials by name
        for (const auto& p : m.parts)
            for (const auto& bm : p.mesh.materials) {
                const auto* e = usa->find(bm.materialId);
                if (e == nullptr) continue;
                const std::vector<uint8_t> link = usa->link(*e);
                uint32_t n = 0;
                if (link.size() >= 4) std::memcpy(&n, link.data(), 4);
                byName[n].push_back(bm.materialId);
            }
        for (const auto& [slot, animId] : m.slots()) {
            const Clip* c = clip(animId);
            if (c == nullptr) continue;
            for (size_t i = 0; i < c->matTracks.size(); ++i) {
                ++tracks;
                const auto it = byName.find(c->matTracks[i].name);
                if (it == byName.end()) continue;
                ++resolvedM;
                const Clip::MatSample s = c->sampleMaterial(i, 0);
                for (uint32_t mid : it->second) {
                    Material mt;
                    try {
                        mt = parseMaterialBody(usa->body(*usa->find(mid)).value());
                    } catch (const std::exception&) {
                        continue;
                    }
                    double d = 0;
                    for (int k = 0; k < 3; ++k) {
                        if (s.hasDiffuse) d = std::max(d, std::fabs(s.diffuse[size_t(k)] - mt.diffuse[size_t(k)]));
                        if (s.hasEmissive) d = std::max(d, std::fabs(s.emissive[size_t(k)] - mt.emissive[size_t(k)]));
                    }
                    if (s.hasU) d = std::max(d, std::fabs(s.u - mt.uvOffset[0]));
                    if (s.hasV) d = std::max(d, std::fabs(s.v - mt.uvOffset[1]));
                    if (s.hasAngle) d = std::max(d, std::fabs(s.angle - mt.uvAngle));
                    if (d > 2e-3) {
                        ++changes;
                        std::printf("  material track: Lod %08x slot %u anim %08x material %08x (%08x) differs by %.3f:", lodId, slot, animId, mid,
                                    c->matTracks[i].name, d);
                        if (s.hasDiffuse)
                            std::printf(" diffuse %.3f %.3f %.3f (was %.3f %.3f %.3f)", s.diffuse[0], s.diffuse[1], s.diffuse[2], mt.diffuse[0], mt.diffuse[1],
                                        mt.diffuse[2]);
                        if (s.hasEmissive)
                            std::printf(" emissive %.3f %.3f %.3f (was %.3f %.3f %.3f)", s.emissive[0], s.emissive[1], s.emissive[2], mt.emissive[0],
                                        mt.emissive[1], mt.emissive[2]);
                        std::printf(" uv %.3f %.3f angle %.3f (was %.3f %.3f %.3f)\n", s.u, s.v, s.angle, mt.uvOffset[0], mt.uvOffset[1], mt.uvAngle);
                    }
                }
            }
        }
    }
    std::printf("  material tracks on world classes: %d, resolved to the skin's materials %d, values changed %d\n", tracks, resolvedM, changes);
    t.check(resolvedM == 133 && changes == 12, "0x813e50: 133 tracks resolve, 12 change their material", resolvedM * 100 + changes, 13312);

    // The RTC helicopter's own clip (TELEPORT_HELICO ba33ccc9): the beacon material 04034a5f blinks red / black (75 H keys),
    // its emissive is held at 0 (the last of the two same-name tracks wins).
    if (auto heli = openDpc(dataDir + "/CommonComponent/RTC/TELEPORT_HELICO.DPC")) {
        rtc::Rtc r;
        int owners = 0;  // the RTC-only object (F) that plays it (RtcView: its own clock)
        if (rtcOf(*heli, r, nullptr))
            for (const rtc::Object& o : r.objects) owners += o.anim == 0xba33ccc9;
        t.check(owners == 1, "TELEPORT_HELICO: one F object plays ba33ccc9", owners, 1);
        Clip c;
        const auto* e = heli->find(0xba33ccc9);
        if (e && c.parse(heli->body(*e).value_or(std::vector<uint8_t>{}))) {
            int last = -1;
            for (size_t i = 0; i < c.matTracks.size(); ++i)
                if (c.matTracks[i].name == 0x04034a5f) last = int(i);
            const Clip::MatSample on = c.sampleMaterial(size_t(std::max(last, 0)), 1.8333328962326050), off = c.sampleMaterial(size_t(std::max(last, 0)), 0.7333329916000366);
            t.check(last >= 0 && on.hasDiffuse && std::fabs(on.diffuse[0] - 1) < 1e-3 && std::fabs(off.diffuse[0]) < 1e-3 && on.hasEmissive && on.emissive[0] == 0,
                    "TELEPORT_HELICO beacon 04034a5f: red at 1.833 s, black at 0.733 s", last >= 0 ? on.diffuse[0] - off.diffuse[0] : -1, 1);
        } else {
            t.check(false, "TELEPORT_HELICO ba33ccc9", 0, 1);
        }
    }
    // 0x6639d0 Camera_Z vf26: the RTC cameras look at their Camera_Z target node (+0x844, body word 2); the port's
    // rtc::Player::camera takes the RTC's C list: the same node ids in every shipped RTC.
    {
        int files = 0, same = 0;
        std::error_code ec;
        for (const auto& de : std::filesystem::directory_iterator(dataDir + "/CommonComponent/RTC", ec)) {
            auto f = openDpc(de.path().string());
            rtc::Rtc r;
            if (!f || !rtcOf(*f, r, nullptr)) continue;
            std::set<uint32_t> cams, cl;
            for (uint32_t id : f->ids(0xFCC62509)) {  // Camera_Z
                const auto b = f->body(*f->find(id));
                uint32_t tgt = 0;
                if (b && b->size() >= 12) std::memcpy(&tgt, b->data() + 8, 4);
                if (tgt) cams.insert(tgt);
            }
            for (uint32_t x : r.targets)  // C holds the bind node names; Camera_Z holds the node resources
                for (const rtc::Bind& b : r.binds)
                    if (x && b.nodeId == x) cl.insert(b.nodeRes);
            ++files;
            same += cams == cl;
        }
        t.check(files == 26 && same == 26, "RTC Camera_Z targets (+0x844) = the RTC look-at list (26 files)", same, 26);
    }
    // 0x677710: Particles_Z with an element flag 0x100 never sleep (USA1: 10 of them).
    int awake = 0;
    for (uint32_t id : usa->ids(0xc56963de)) {
        const auto* e = usa->find(id);
        try {
            const particles::Emitter em = particles::parseParticles(usa->body(*e).value(), usa->link(*e));
            bool a = false;
            for (const particles::Element& el : em.elements) a = a || (el.flags & 0x100);
            awake += a;
        } catch (const std::exception&) {
        }
    }
    t.check(awake == 10, "USA1 Particles_Z with element flag 0x100 (never sleep)", awake, 10);
}

// PHYSIQUE (port deviation): the custom rider's glute springs (rider_jiggle.cpp vectors) and the skinning hook: off is
// the plain skinning byte for byte; on, only the vertices weighted to a helper bone move, by the spring's offset.
void riderJiggleChecks(const std::string& dataDir, Tally& t) {
    std::printf("rider glute springs (PHYSIQUE):\n");
    int total = 0;
    const int bad = riderJiggleCheck(&total);
    t.ok += total - bad, t.total += total;
    auto pilot = openDpc(dataDir + "/CommonComponent/DATAS/PILOT.DPC");
    if (!pilot) {
        t.check(false, "PILOT.DPC", 0, 1);
        return;
    }
    const Rider rider(*pilot);
    Rider::SkinTemplate st = rider.prepareSkin();
    const int hips = rider.bone("Hips");
    st.jiggleBones[0].base = st.jiggleBones[1].base = hips;
    st.jiggleBones[0].centre = {0.07, 0.94, -0.10}, st.jiggleBones[1].centre = {-0.07, 0.94, -0.10};
    st.hasJiggle = hips >= 0;
    size_t marked = 0;
    for (auto& v : st.verts)
        if (v.pos[1] > 0.85f && v.pos[1] < 1.05f && v.pos[2] < -0.05f) v.jig = v.pos[0] >= 0 ? 1 : 2, v.jw = 0.6f, ++marked;
    const auto world = rider.pose(rider.ridingSlot("QUAD"), 0.0);
    std::vector<uint8_t> plain, off, on;
    rider.skin(st, world, plain);
    RiderJiggle j;
    j.setBones(st.jiggleBones);
    j.setAmplitude(0.0);
    for (int f = 0; f < 60; ++f) j.stepLocal({0, 40, 0}, RiderJiggle::kStep, false);
    rider.skin(st, world, off, &j);
    t.check(marked > 0 && plain == off, "jiggle off: the plain skinning, byte for byte", double(marked), double(marked));
    j.setAmplitude(1.0);
    rider.skin(st, world, on, &j);
    size_t moved = 0, strays = 0;
    for (size_t vi = 0; vi < st.verts.size(); ++vi) {
        const bool diff = std::memcmp(&plain[vi * BakedMesh::kStride], &on[vi * BakedMesh::kStride], 12) != 0;
        if (diff) (st.verts[vi].jig ? moved : strays) += 1;
    }
    t.check(moved > marked / 2 && strays == 0, "jiggle on: only helper-weighted vertices move", double(strays), 0);
    // the custom rider's garment masks (when its assets are installed): no body triangle culled without a garment over it,
    // every worn piece present
    if (const CustomRider* cr = CustomRider::shared()) {
        int configs = 0;
        std::vector<std::string> probs;
        const int bad = cr->coverageCheck(&configs, &probs);
        for (const std::string& p : probs) std::printf("  custom rider coverage: %s\n", p.c_str());
        t.check(bad == 0, "custom rider: culled body covered, pieces present (" + std::to_string(configs) + " outfits)", double(bad), 0);
        // rider4: every skin tone draws the custom skin with its own texture (FUEL's BODY1-3 skin choice)
        int skinMats = 0, tonedOk = 0;
        for (uint32_t m = 0; m < cr->materials().size(); ++m) {
            if (!(cr->materials()[m].flags & CustomMaterial::Skin)) continue;
            ++skinMats;
            bool ok = true;
            for (int tone = 2; tone <= CustomRider::kSkinTones; ++tone)
                ok = ok && cr->toned(m, tone).diffuse != cr->materials()[m].diffuse && cr->toned(m, tone).flags == cr->materials()[m].flags;
            tonedOk += ok;
        }
        std::printf("  custom rider skin tones: %d of %d skin materials have tones 2..%d (%d variants)\n", tonedOk, skinMats, CustomRider::kSkinTones,
                    cr->toneVariants());
        t.check(skinMats > 0 && tonedOk == skinMats, "custom rider: every skin material has a texture per FUEL skin tone", double(skinMats - tonedOk), 0);
    } else {
        std::printf("  custom rider coverage: no custom rider assets installed (skipped)\n");
    }
}

int runAnimCheck(const std::string& dataDir) {
    Tally t;
    std::printf("Animation_Z samplers (emulated 0x7d4ed0, 0x7d62d0, 0x7d6660):\n");
    {
        Clip c;
        const bool ok = c.parse(clipBody({}, {}, {}, 0,
                                         {{0.5f, {4096, -2048, 1000, 100, 200, 300, 2048, 0, -4096}},
                                          {1.5f, {8192, 2048, 0, -1024, 4096, 0, 500, 600, 700}}}));
        t.check(ok, "synthetic body parses", ok, 1);
        Pose p;
        c.sample(0, 0.75, p);
        t.vec("V1 translation Hermite t 0.75", p.t.data(), {1.2382812, -0.390625, 0.06536865});
        c.sample(0, 2.0, p);
        t.vec("V1 translation past the last key", p.t.data(), {2.0, 0.5, 0.0});
    }
    {
        Clip c;
        c.parse(clipBody({{0.0f}, {1.0f}}, {{0, 0, 0, 2000}, {0, 1414, 0, 1414}}, {}, 0, {}));
        Pose p;
        c.sample(0, 0.5, p);
        t.vec("V2 quaternion slerp t 0.5", p.q.data(), {0, 0.38263762, 0, 0.92385066});
        Clip d;
        d.parse(clipBody({{0.0f}, {0.4f}}, {{200, -400, 600, 1883}, {-900, 300, 1200, -1300}}, {}, 0, {}));
        d.sample(0, 0.1, p);
        t.vec("V2 quaternion slerp with sign flip", p.q.data(), {0.21563898, -0.20777382, 0.065806165, 0.9613697});
    }
    {
        Clip c;
        c.parse(clipBody({}, {}, {{0, .1f, .2f, .3f, .01f, .02f, .03f, .4f, .5f, .6f}, {2, 1, -.5f, .25f, .7f, .8f, .9f, .04f, .05f, .06f}}, 0, {}));
        Pose p;
        c.sample(0, 0.5, p);
        t.vec("V3 Euler Hermite, q = qz qy qx", p.q.data(), {0.11939044, 0.082156, 0.15664053, 0.97696465});
        Clip s;
        s.parse(clipBody({}, {}, {{0, .1f, .2f, .3f, 0, 0, 0, 0, 0, 0}}, 0, {}));
        s.sample(0, 0.0, p);
        t.vec("V3 Euler single key", p.q.data(), {0.034270793, 0.10602052, 0.1435722, 0.9833475});
    }
    std::printf("RTC samplers (emulated 0x7d5640, 0x72f750, 0x7d61e0):\n");
    {
        std::vector<rtc::VKey> k(3);
        std::memset(k.data(), 0, k.size() * sizeof(rtc::VKey));
        k[0].frame = 0, k[0].out[0] = 1, k[0].out[1] = 2, k[0].out[2] = 3;
        k[1].frame = 60, k[1].flags = 0x80, k[1].v[0] = 10, k[1].v[1] = 20, k[1].v[2] = 30, k[1].in[0] = 2, k[1].in[2] = -1;
        k[2].frame = 90, k[2].v[0] = k[2].v[1] = k[2].v[2] = 5;
        Vec3 v = rtc::sampleV(k, 0, 0.5);
        t.vec("V1 vec3 t 0.5", v.data(), {4.875, 10.25, 15.5});
        v = rtc::sampleV(k, 0, 1.25);
        t.vec("V1 vec3 t 1.25 (step key)", v.data(), {10, 20, 30});
        v = rtc::sampleV(k, 0, 2.0);
        t.vec("V1 vec3 t 2.0", v.data(), {5, 5, 5});
        std::vector<rtc::SKey> s = {{0, 4096, 0, 2048, 0xffff}, {2, 8192, -4096, 0, 0xffff}};
        t.near("V2 scalar t 0.5", rtc::sampleS(s, 0, 0.5), 1.2734375);
        t.near("V2 scalar t 1.5", rtc::sampleS(s, 0, 1.5), 2.0078125);
        std::vector<rtc::RKey> q = {{0, {0, 0, 0, 2000}}, {1, {0, 1414, 0, 1414}}, {2, {0, 0, 2000, 0}}};
        Quat a = rtc::sampleR(q, 0.25);
        t.vec("V3 quaternion t 0.25", a.data(), {0, 0.19506843, 0, 0.9807779});
        a = rtc::sampleR(q, 1.5);
        t.vec("V3 quaternion t 1.5", a.data(), {0, 0.4999245, 0.70710677, 0.4999245});
    }
    std::printf("shipped RTC files (grammar 0x7d6ad0, playback 0x729b30):\n");
    struct Want {
        const char* name;
        double dur;
        size_t nodes;
    };
    for (const Want& w : {Want{"TELEPORT_HELICO", 16.666664, 318}, Want{"TR_DEATH", 2.0, 15}, Want{"VI_BIKE", 34.666664, 4}}) {
        auto f = openDpc(dataDir + "/CommonComponent/RTC/" + w.name + ".DPC");
        rtc::Rtc r;
        std::string err;
        const bool ok = f && rtcOf(*f, r, &err);
        t.check(ok && std::fabs(r.duration - w.dur) < 1e-3 && r.nodes.size() == w.nodes, std::string(w.name) + " parses exactly (duration, nodes)",
                ok ? r.duration : -1, w.dur);
        if (!ok) continue;
        rtc::Player pl(r);
        pl.start();
        std::vector<rtc::Fired> fired;
        double respawnAt = -1, firstCut = -1;
        for (int i = 0; i < 60 * 60 && !pl.finished(); ++i) {
            fired.clear();
            pl.advance(1.0f / 60, fired);
            for (const rtc::Fired& e : fired) {
                if (e.msg.type == 0x10b && respawnAt < 0) respawnAt = e.time;
                if (e.msg.type == 0x30 && firstCut < 0) firstCut = e.time;
            }
        }
        if (std::string(w.name) == "TR_DEATH") {
            t.near("TR_DEATH respawn event 0x10b", respawnAt, 0.96666664, 1e-4);
            // P76: 0x10e at the respawn starts the 0.5 s pause hold (0x53c740 / 0x53c770); its release seeks past the
            // 1.0-1.5 s loop end (0x5385a0: loop end + 0.01) and the cut-scene ends at 2.0 (port had releaseLoop at 0x10b).
            rtc::Player hp(r);
            hp.start();
            rtc::PauseHold hold;
            double holdAt = -1, releasedAt = -1, afterSeek = -1, endAt = -1;
            int pausedFrames = 0;
            for (int i = 0; i < 600 && !hp.finished(); ++i) {
                fired.clear();
                hp.advance(1.0f / 60, fired);
                for (const rtc::Fired& e : fired)
                    if (e.msg.type == 0x10e && e.node < 0) hold.start(), holdAt = e.time;
                const double before = hp.time();
                if (hold.update(1.0f / 60, hp, &fired)) releasedAt = before, afterSeek = hp.time();
                pausedFrames += hold.holding;
                if (hp.finished()) endAt = hp.time();
            }
            t.near("TR_DEATH 0x10e pause hold at the respawn", holdAt, 0.96666664, 1e-4);
            t.check(releasedAt > 1.4 && releasedAt < 1.5 && std::fabs(afterSeek - 1.51) < 1e-3,
                    "TR_DEATH hold released after > 0.5 s, seek to the loop end + 0.01", afterSeek, 1.51);
            float sum = 0;  // the float timer: released on the update where sum(1/60) > 0.5
            int want = 0;
            while (!((sum += 1.0f / 60) > 0.5f)) ++want;
            t.check(pausedFrames == want && std::fabs(endAt - 2.0) < 1e-4, "TR_DEATH game paused while timer <= 0.5, ends at 2.0 s", pausedFrames, want);
            t.check(!rtc::hidesInDaylight(0.79f) && rtc::hidesInDaylight(0.8f) && rtc::hidesInDaylight(1.0f) && !rtc::hidesInDaylight(0.0f),
                    "0x10f hides its node when the day factor >= 0.8 (0x9dc154)", 0.8, 0.8);
        }
        if (std::string(w.name) == "TELEPORT_HELICO") {  // P76: one 0x10f, node level, at 0 s on a node with an F object
            int n10f = 0, withObject = 0;
            for (size_t n = 0; n < r.nodes.size(); ++n)
                for (const rtc::EventKey& k : r.nodes[n].events)
                    for (const rtc::Msg& m : k.msgs)
                        if (m.type == 0x10f && k.time == 0) {
                            ++n10f;
                            for (const rtc::Object& o : r.objects) withObject += o.e == r.nodes[n].e;
                        }
            t.check(n10f == 1 && withObject == 1, "TELEPORT_HELICO: one 0x10f at 0 s, on the node of one RTC object (A305's light mesh)", n10f, 1);
        }
        if (std::string(w.name) == "VI_BIKE")
            t.check(!pl.finished() && pl.time() >= 4.66 && pl.time() <= 34.67, "VI_BIKE loops 4.667-34.667 s (60 s played)", pl.time(), 4.667);
        if (std::string(w.name) == "TELEPORT_HELICO") {
            t.check(pl.finished() && std::fabs(pl.time() - w.dur) < 1e-4, "TELEPORT_HELICO ends at its duration", pl.time(), w.dur);
            t.check(firstCut >= 0, "TELEPORT_HELICO takes the camera (0x30)", firstCut, 0);
        }
    }
    {
        auto f = openDpc(dataDir + "/CommonComponent/DATAS/RTCSYS.DPC");
        int ok = 0, n = 0;
        if (f)
            for (uint32_t id : f->ids(0x2A11CEE8)) {
                rtc::Rtc r;
                ++n;
                ok += rtc::parse(f->body(*f->find(id)).value_or(std::vector<uint8_t>{}), r, nullptr) ? 1 : 0;
            }
        t.check(n == 19 && ok == 19, "RTCSYS: 19 hub RTCs parse", ok, 19);
    }
    // P67: the fader (0x438030 / 0x438190 / 0x4382e0, update 0x438560, draw 0x438be0), hand-evaluated from the decomp.
    std::printf("screen fader and RTC presentation (P67):\n");
    {
        rtc::Fader f;
        f.holdBlack();
        t.check(f.mode() == 3 && f.alpha() == 1.0f, "hold black (0x4382e0): mode 3, alpha 1", f.mode(), 3);
        f.fadeIn(0.3f);
        f.update(0.015f);  // the first frame of a fade from a settled value is skipped (+0xe00)
        t.near("fade in 0.3 s: skipped first frame", f.value(), 1.0);
        for (int i = 0; i < 2; ++i) f.update(0.015f);
        t.near("fade in 0.3 s: holds 1 for the first 10 % (0.03 s)", f.value(), 1.0);
        for (int i = 0; i < 9; ++i) f.update(0.015f);
        t.near("fade in 0.3 s: value at 0.165 s", f.value(), 0.5, 1e-5);
        t.near("fade in: quad alpha min(1.5 v, 1)", f.alpha(), 0.75, 1e-5);
        t.near("fade in: sound slot 1 = 1 - v (0x554200)", f.soundGain(), 0.5, 1e-5);
        f.fadeOut(1.0f);  // resumes at the same value: elapsed = 0.5
        f.update(0.1f);
        t.near("fade out resumed from 0.5: value at 0.6 s / 0.9 s", f.value(), 0.6 / 0.9, 1e-5);
        f.fadeIn(0.3f);  // value 0.667 -> elapsed (1 - v) 0.3 = 0.1
        f.update(0.1f);
        t.near("fade in resumed: 1 - (0.2 - 0.03) / 0.27", f.value(), 1.0 - 0.17 / 0.27, 1e-5);
        bool done = false;
        for (int i = 0; i < 20 && !done; ++i) done = f.update(0.02f);
        t.check(done && f.mode() == 0 && f.value() == 0, "fade in ends: mode 0, callback posted", f.mode(), 0);
        f.fadeOut(0.1f);
        f.update(0.015f);
        f.update(0.045f);
        t.near("fade out 0.1 s: value at 0.045 s", f.value(), 0.5, 1e-5);
        f.update(0.06f);
        t.check(f.mode() == 3 && f.value() == 1.0f, "fade out ends holding black (mode 3)", f.mode(), 3);
        rtc::Fader w;
        rtc::fadeMessage(w, 6, false);
        t.check(w.mode() == 6 && w.white(), "0x98 TOWHITE: 0x438190(0.1, white) mode 6", w.mode(), 6);
        rtc::fadeMessage(w, 1, true);
        t.check(w.mode() == 0 && w.value() == 0, "0x98 FROMBLACK while skipping clears the fader (0x53e1a0)", w.mode(), 0);
        rtc::fadeMessage(w, 2, true);
        t.check(w.mode() == 2 && w.value() == 1.0f, "0x98 TOBLACK while skipping: instant (0x438190(0))", w.value(), 1);
        rtc::CameraShake sh;
        sh.start();
        const auto o0 = sh.update(0.5f);
        const auto o1 = sh.update(0.1f);
        const double a = 0.5 * 0.3 * 0.5 * 0.05;
        t.check(o0[0] == 0 && std::fabs(o1[0] - a * std::cos(15.0)) < 1e-6 && std::fabs(o1[2] - a * std::cos(25.0)) < 1e-6,
                "SHAKECAM: (1-t) A t 0.05 (cos 30t, cos 40t, cos 50t) (0x4156e0)", o1[0], a * std::cos(15.0));
        rtc::EndGrade g;
        g.dir = 1;
        g.update(0.4f);
        g.update(0.8f);
        t.check(g.ramp == 1.0f && g.dir == 0, "race-end grade ramps 0 -> 1 in 1 s (0x4b6050)", g.ramp, 1);
    }
    if (auto f = openDpc(dataDir + "/CommonComponent/RTC/TELEPORT_HELICO.DPC")) {
        rtc::Rtc r;
        if (rtcOf(*f, r, nullptr)) {
            rtc::Player pl(r);
            pl.start();
            std::vector<rtc::Fired> fired;
            pl.advance(1.0f / 60, fired);
            int fadeKind = -1, sounds = 0;
            for (const rtc::Fired& e : fired) {
                if (e.msg.type == 0x98) fadeKind = int(e.msg.a);
                sounds += e.msg.type == 0x24;
            }
            t.check(fadeKind == 1, "TELEPORT_HELICO: FROMBLACK (0x98 kind 1) at 0 s", fadeKind, 1);
            t.check(sounds == 1, "TELEPORT_HELICO: rotor sound 0x24 at 0 s", sounds, 1);
            fired.clear();
            pl.seek(r.duration - 1 + 0.01f, &fired);  // the skip (SKIP_BEFORE_END) evaluates the jumped window, skipping
            uint32_t lastCut = 0;
            bool allSkipping = !fired.empty();
            for (const rtc::Fired& e : fired) {
                if (e.msg.type == 0x30) lastCut = e.msg.d;
                allSkipping = allSkipping && e.skipping;
            }
            t.check(lastCut == 0x76ffe45d && allSkipping, "skip seek fires the 12.33 s cut, skipping (0x72aeb0)", double(lastCut), double(0x76ffe45du));
            // P96: 0x103 'AMEO' (the helicopter's cable joints): four node events on A1 at 0 s, a = 1..4.
            int ameo = 0, ameoSum = 0;
            for (const rtc::EventKey& k : r.nodes[1].events)
                for (const rtc::Msg& m : k.msgs)
                    if (m.type == 0x103 && k.time == 0 && m.b == 0x4f454d41) ++ameo, ameoSum += int(m.a);
            t.check(ameo == 4 && ameoSum == 10, "TELEPORT_HELICO: 4 x 0x103 'AMEO' a = 1..4 on A1 at 0 s", ameo, 4);
            rtc::BoneRegistry reg;
            t.check(reg.size() == 56 && reg.find(asoboCrc("LeftHand")) == 27 && reg.find(asoboCrc("DYN1")) == 34 && reg.find(asoboCrc("FACE")) == 55,
                    "global bone ids: 56 fixed (LEFTHAND 27, DYN1 34, FACE 55; 0x747550)", double(reg.size()), 56);
            if (auto usa = openDpc(dataDir + "/CommonComponent/WORLD/USA1.DPC")) {  // the world's skeletons register first
                for (uint32_t id : usa->ids(cls::Skel))
                    if (auto b = usa->body(*usa->find(id))) try {
                            const Skel sk = parseSkelBody(*b);
                            for (const auto& l : sk.palettes)
                                for (uint32_t c : l) reg.id(c);
                            for (const Bone& bn : sk.bones) reg.id(bn.name);
                        } catch (const std::exception&) {
                        }
            }
            const rtc::Object* heli = nullptr;
            for (const rtc::Object& o : r.objects)
                if (o.e == r.nodes[1].e) heli = &o;
            SkinnedModel hm;
            std::string err;
            if (heli && hm.load({f.get()}, heli->res, &err)) {
                const Skel& sk = hm.skeleton();
                std::vector<uint32_t> names;
                for (const Bone& b : sk.bones) names.push_back(b.name);
                const int j1 = rtc::ameoJoint(reg, sk.palettes, names, 1, true), j2 = rtc::ameoJoint(reg, sk.palettes, names, 2, true),
                          j3 = rtc::ameoJoint(reg, sk.palettes, names, 3, true), j4 = rtc::ameoJoint(reg, sk.palettes, names, 4, true);
                // The instance (0x66c0e0, pre-order) registers LINK1 (9), Link3 (12), Link2 (15), Link4 (18) in bone order.
                t.check(j1 == 9 && j2 == 12 && j3 == 15 && j4 == 18 && names[9] == asoboCrc("LINK1") && names[12] == asoboCrc("LINK3"),
                        "AMEO FUEL order a 1..4 -> LINK1, Link3, Link2, Link4 (bones 9, 12, 15, 18)", j1 * 1000 + j2 * 100 + j3 * 10 + j4, 9000 + 1200 + 150 + 18);
                // P100 [DEVIATION]: the port pairs a with LINKa: LINK1, Link2, Link3, Link4 = bones 9, 15, 12, 18.
                const int p1 = rtc::ameoJoint(reg, sk.palettes, names, 1), p2 = rtc::ameoJoint(reg, sk.palettes, names, 2),
                          p3 = rtc::ameoJoint(reg, sk.palettes, names, 3), p4 = rtc::ameoJoint(reg, sk.palettes, names, 4);
                t.check(p1 == 9 && p2 == 15 && p3 == 12 && p4 == 18 && rtc::ameoJoint(reg, sk.palettes, names, 5) < 0,
                        "AMEO port pairing a -> LINKa (bones 9, 15, 12, 18; a 5 none)", p1 * 1000000 + p2 * 10000 + p3 * 100 + p4, 9151218);
                const anim::Vec3 at{0.5, -5.0, 0.25};
                const auto w = hm.pose(nullptr, 0, {{size_t(9), at}});
                const auto w0 = hm.pose(nullptr, 0);
                const double d = std::fabs(w[9][12] - at[0]) + std::fabs(w[9][13] - at[1]) + std::fabs(w[9][14] - at[2]);
                const double end = std::fabs((w[11][13] - w[9][13]) - (w0[11][13] - w0[9][13]));  // the cable end follows
                t.check(d < 1e-6 && end < 1e-6, "0x676480: joint origin = target; the cable end (bone 11) follows", d + end, 0);
            } else {
                t.check(false, "TELEPORT_HELICO helicopter skin", 0, 1);
            }
            int anchors = 0;
            for (int a = 1; a <= 4; ++a)
                for (const rtc::Bind& b : r.binds) anchors += b.nodeId == asoboCrc("MESH_ID_QUAD_ROAD_2_" + std::to_string(a));
            t.check(anchors == 4, "binds MESH_ID_QUAD_ROAD_2_1..4 (the cable targets)", anchors, 4);
        }
    }
    std::printf("world animation (USA1.DPC):\n");
    if (auto usa = openDpc(dataDir + "/CommonComponent/WORLD/USA1.DPC")) {
        int ok = 0, n = 0;
        for (uint32_t id : usa->ids(cls::Animation)) {
            Clip c;
            ++n;
            ok += c.parse(usa->body(*usa->find(id)).value_or(std::vector<uint8_t>{})) ? 1 : 0;
        }
        t.check(n == 36 && ok == 36, "36 Animation_Z parse", ok, 36);
        SkinnedModel flag;
        std::string err;
        const bool loaded = flag.load({usa.get()}, 0x9c719fb6, &err);  // DRAPEAU_SKIN (hub flags)
        const Clip* idle = loaded ? flag.clipForSlot(0) : nullptr;
        t.check(idle != nullptr && std::fabs(idle->duration - 3.0666666) < 1e-3 && idle->loops(), "DRAPEAU_SKIN slot 0 loops 3.067 s", idle ? idle->duration : -1,
                3.0666666);
        if (loaded) {
            const auto bind = flag.pose(nullptr, 0), mid = flag.pose(idle, 1.5);
            double moved = 0;
            for (size_t b = 0; b < bind.size(); ++b) moved = std::max(moved, std::fabs(bind[b][12] - mid[b][12]) + std::fabs(bind[b][13] - mid[b][13]));
            t.check(moved > 0.01, "DRAPEAU_SKIN bones move at t 1.5", moved, 0.01);
        }
    } else {
        t.check(false, "USA1.DPC", 0, 1);
    }
    riderSeatCheck(dataDir, t);
    riderJiggleChecks(dataDir, t);  // PHYSIQUE
    pk13Checks(dataDir, t);
    pk31LeanChecks(t);  // PK31
    pk31IkChecks(t);    // PK31
    pk29Checks(dataDir, t);
    objectChecks(dataDir, t);
    std::printf("animcheck: %d of %d vectors match\n", t.ok, t.total);
    return t.ok == t.total ? 0 : 1;
}

}  // namespace fuel::anim
