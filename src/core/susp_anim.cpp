// [suspanim] The vehicle's suspension and steering parts every frame (see susp_anim.hpp; notes/suspanim.md).
#include "susp_anim.hpp"

#include "upgrade_visuals.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <unordered_map>

namespace fuel::susp {

namespace {

V3 add(V3 a, V3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 sub(V3 a, V3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 mul(V3 a, float k) { return {a[0] * k, a[1] * k, a[2] * k}; }
float dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3 cross(V3 a, V3 b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
float len(V3 a) { return std::sqrt(dot(a, a)); }
V3 norm(V3 a) {
    const float l = len(a);
    return l > 1e-9f ? mul(a, 1.0f / l) : V3{0, 0, 0};
}
using M3 = std::array<float, 9>;
const M3 kI{1, 0, 0, 0, 1, 0, 0, 0, 1};
M3 mm(const M3& a, const M3& b) {
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[size_t(i * 3 + j)] = a[size_t(i * 3)] * b[size_t(j)] + a[size_t(i * 3 + 1)] * b[size_t(3 + j)] + a[size_t(i * 3 + 2)] * b[size_t(6 + j)];
    return r;
}
V3 mv(const M3& a, V3 p) {
    return {a[0] * p[0] + a[1] * p[1] + a[2] * p[2], a[3] * p[0] + a[4] * p[1] + a[5] * p[2], a[6] * p[0] + a[7] * p[1] + a[8] * p[2]};
}
// p' = M (p - from) + to
Affine about(const M3& M, V3 from, V3 to) {
    Affine a;
    a.m = M;
    a.t = sub(to, mv(M, from));
    return a;
}
Affine compose(const Affine& a, const Affine& b) {  // a after b
    Affine r;
    r.m = mm(a.m, b.m);
    r.t = add(mv(a.m, b.t), a.t);
    return r;
}
M3 arcFromZ(V3 f) {  // Quat_FromArc((0,0,1), f)
    f = norm(f);
    const V3 z{0, 0, 1};
    const V3 c = cross(z, f);
    const float s = len(c), co = std::clamp(dot(z, f), -1.0f, 1.0f);
    if (s < 1e-7f) return kI;
    return rotation(mul(c, 1.0f / s), std::atan2(s, co));
}
bool isWheelPart(const PlacedPart& p) { return p.placement == Placement::Wheel || p.placement == Placement::WheelFlipped; }

}  // namespace

bool enabled() {
    static const bool on = [] {
        const char* e = std::getenv("FUEL_SUSP_ANIM");
        return !(e && std::atoi(e) == 0);
    }();
    return on;
}

float hingeAngle(const V3& d, const V3& l, int axis) {
    const float ld = len(d), ll = len(l);
    if (!(ld > 1e-6f) || !(ll > 1e-6f)) return 0;
    const float dy = std::clamp(d[1] / ld, -1.0f, 1.0f), ly = std::clamp(l[1] / ll, -1.0f, 1.0f);  // Math_Clamp(-1, 1)
    const float delta = std::acos(dy) - std::acos(ly);
    const float s = axis == 2 ? d[0] : d[2];  // d . (1,0,0) [0xa06338] / d . (0,0,1) [0xa06350]
    const float sign = s > 0 ? 1.0f : s < 0 ? -1.0f : 0.0f;
    return axis == 2 ? -sign * delta : sign * delta;
}

std::array<float, 9> rotation(const V3& u, float a) {
    const float c = std::cos(a), s = std::sin(a), k = 1 - c;
    const float x = u[0], y = u[1], z = u[2];
    return {c + x * x * k, x * y * k - z * s, x * z * k + y * s, y * x * k + z * s, c + y * y * k, y * z * k - x * s,
            z * x * k - y * s, z * y * k + x * s, c + z * z * k};
}

std::array<float, 9> turnAbout(const V3& u, const V3& from, const V3& to) {
    const V3 f = norm(sub(from, mul(u, dot(u, from)))), g = norm(sub(to, mul(u, dot(u, to))));
    if (len(f) < 0.5f || len(g) < 0.5f) return kI;
    return rotation(u, std::atan2(dot(u, cross(f, g)), dot(f, g)));
}

Affine segment(const V3& C, const V3& B, const V3& B2, float thick) {
    const Mat4 m = upgvis::segmentMap(C, B, B2, thick);  // row vectors: p' = p M
    Affine a;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) a.m[size_t(r * 3 + c)] = float(m[size_t(c * 4 + r)]);
    a.t = {float(m[12]), float(m[13]), float(m[14])};
    return a;
}

Rig build(const AssembledVehicle& av, const std::vector<V3>& hubs, const upgvis::Plan* plan) {
    Rig rig;
    rig.hubRest = hubs;
    rig.partNode.assign(av.parts.size(), -1);
    if (!enabled() || hubs.empty()) return rig;
    const bool planOn = plan && plan->active && plan->parts.size() == av.parts.size();
    // The wheel bones (0x5caf40 moves their nodes with the wheels): bone -> setup wheel, nearest hub as VehicleRig.
    std::unordered_map<int, int> boneWheel;
    for (const PlacedPart& p : av.parts) {
        if (!p.drawn() || !isWheelPart(p) || p.bone < 0) continue;
        const V3 T{float(p.world[12]), float(p.world[13]), float(p.world[14])};
        int best = -1;
        float bd = 1e9f;
        for (size_t i = 0; i < hubs.size(); ++i)
            if (const float d = len(sub(hubs[i], T)); d < bd) bd = d, best = int(i);
        boneWheel[p.bone] = best;
    }
    auto wheelOf = [&](int bone) {
        const auto it = boneWheel.find(bone);
        return it == boneWheel.end() ? -1 : it->second;
    };
    auto bonePos = [&](int b) { return b >= 0 && size_t(b) < av.bonePos.size() ? av.bonePos[size_t(b)] : V3{}; };

    // One candidate node per object (a skin's meshes share it); dependencies by object.
    struct Cand {
        Node n;
        uint32_t object = 0, aimObject = 0, atObject = 0;
        bool live = false, keep = false;
        int index = -1;
    };
    std::vector<Cand> cands;
    std::unordered_map<uint32_t, int> candOf;
    std::unordered_map<uint64_t, int> recOf;
    auto fromLive = [&](const upgvis::Live& L, Node& n) {
        n.wheel = L.wheel, n.wheel2 = L.wheel2, n.C = L.C, n.B = L.B, n.D = L.D, n.pivot = L.pivot, n.sB = L.sB, n.thick = L.thick;
        n.steerWheel = L.steer ? L.wheel : -1;
        n.kind = L.kind == upgvis::Live::Segment ? Kind::Segment : Kind::Translate;
    };
    for (size_t k = 0; k < av.parts.size(); ++k) {
        const PlacedPart& t = av.parts[k];
        if (!t.drawn() || isWheelPart(t)) continue;
        // One record per (object, commanding bone): a skin's meshes share it; one object may serve two records
        // (TRUCK_ALL_4's SuspBL3 on the rear and the front left bones), each with its own placement.
        const uint64_t rec = (uint64_t(t.objectId) << 32) | uint32_t(t.bone + 1);
        if (const auto it = recOf.find(rec); it != recOf.end()) {
            rig.partNode[k] = -2 - it->second;  // resolved below
            continue;
        }
        Cand c;
        c.object = t.objectId;
        Node& n = c.n;
        n.K = {float(t.world[12]), float(t.world[13]), float(t.world[14])};
        const upgvis::PartEdit* e = planOn ? &plan->parts[k] : nullptr;
        if (e && e->live.kind != upgvis::Live::None && e->live.wheel >= 0 && size_t(e->live.wheel) < hubs.size()) {
            fromLive(e->live, n);  // [port] the upgrade visuals' re-aimed part
            c.live = c.keep = true;
        } else if (e && e->moved) {
            continue;  // grown / moved by the plan, rigid (engine, exhaust)
        } else if (t.bone >= 0 && t.dyn < 0 && !t.keptNodeMatrix) {
            const PlacedPart::LinkAim& L = t.aim;
            n.look = L.look;
            n.axis = L.axis;
            c.keep = L.hasLook;
            switch (t.placement) {
            case Placement::Link:
                if (L.bone < 0 || L.bone == t.bone) c.keep = false;  // a snap (hull on Hips): rigid
                n.kWheel = wheelOf(t.bone);
                n.aimWheel = wheelOf(L.bone);
                n.aim = bonePos(L.bone);
                break;
            case Placement::LinkB:
                n.kWheel = wheelOf(t.bone);
                c.aimObject = L.object;
                n.aim = L.point;
                n.stretch = t.shape == PartShape::SkinBindPose;
                if (L.object == 0) c.keep = false;
                break;
            case Placement::LinkR:
                n.kWheel = wheelOf(t.bone);
                if (L.bone2 >= 0) {
                    n.kind = Kind::Axle;
                    n.aimWheel = wheelOf(L.bone), n.aimWheel2 = wheelOf(L.bone2);
                    n.aim = bonePos(L.bone), n.aim2 = bonePos(L.bone2);
                } else {
                    n.kind = Kind::Steer;
                    n.steerWheel = wheelOf(L.bone);
                    if (n.steerWheel < 0) c.keep = false;
                    const V3 l = norm(L.look);
                    if (L.hasAttach && std::fabs(l[1]) > 1e-3f) {
                        n.slide = true;
                        n.attachT = dot(L.attach, l), n.limitT = dot(L.limit, l), n.limitR = L.limitRadius;
                    }
                }
                break;
            case Placement::LinkM:
                c.atObject = L.atObject;
                if (wheelOf(L.bone) >= 0) n.kind = Kind::Upright, n.steerWheel = wheelOf(L.bone);
                else n.aim = bonePos(L.bone);
                c.keep = c.keep || n.kind == Kind::Upright;
                break;
            case Placement::LinkMM:
                c.atObject = L.atObject, c.aimObject = L.object;
                n.aim = L.point;
                break;
            default:
                c.keep = false;
            }
            c.live = n.kWheel >= 0 || n.aimWheel >= 0 || n.aimWheel2 >= 0 || n.steerWheel >= 0;
        }
        if (!c.keep) continue;
        recOf[rec] = int(cands.size());
        if (!candOf.count(c.object)) candOf[c.object] = int(cands.size());  // a point of the object: its first record
        rig.partNode[k] = -2 - int(cands.size());
        cands.push_back(c);
    }
    // Liveness through the dependencies (a shock aimed at a moving arm moves).
    auto cand = [&](uint32_t obj) {
        const auto it = candOf.find(obj);
        return it == candOf.end() ? -1 : it->second;
    };
    for (bool changed = true; changed;) {
        changed = false;
        for (Cand& c : cands) {
            if (c.live) continue;
            const int a = cand(c.aimObject), b = cand(c.atObject);
            if ((a >= 0 && cands[size_t(a)].live) || (b >= 0 && cands[size_t(b)].live)) c.live = changed = true;
        }
    }
    // Order: dependencies first.
    std::vector<int> state(cands.size(), 0);
    std::function<void(int)> visit = [&](int i) {
        if (state[size_t(i)] != 0) return;
        state[size_t(i)] = 1;
        Cand& c = cands[size_t(i)];
        for (uint32_t dep : {c.aimObject, c.atObject})
            if (const int d = cand(dep); d >= 0 && cands[size_t(d)].live && state[size_t(d)] == 0) visit(d);
        c.n.aimNode = cand(c.aimObject) >= 0 ? cands[size_t(cand(c.aimObject))].index : -1;
        c.n.atNode = cand(c.atObject) >= 0 ? cands[size_t(cand(c.atObject))].index : -1;
        c.index = int(rig.nodes.size());
        rig.nodes.push_back(c.n);
        state[size_t(i)] = 2;
    };
    for (size_t i = 0; i < cands.size(); ++i)
        if (cands[i].live) visit(int(i));
    for (int& pn : rig.partNode)
        if (pn <= -2) pn = cands[size_t(-2 - pn)].live ? cands[size_t(-2 - pn)].index : -1;
    for (const Node& n : rig.nodes)
        if (n.kind == Kind::Steer && n.slide) rig.slideWheel = n.steerWheel;
    // [port] the plan's coils / coil-overs
    if (planOn)
        for (const upgvis::Extra& x : plan->liveExtras) {
            if (x.live.kind == upgvis::Live::None || x.live.wheel < 0 || size_t(x.live.wheel) >= hubs.size()) {
                rig.extraNode.push_back(-1);
                continue;
            }
            Node n;
            fromLive(x.live, n);
            rig.extraNode.push_back(int(rig.nodes.size()));
            rig.nodes.push_back(n);
        }
    if (std::getenv("FUEL_SUSP_TRACE")) {
        std::printf("susp: LOD%d %zu live nodes (slide wheel %d)\n", av.lod, rig.nodes.size(), rig.slideWheel);
        for (size_t k = 0; k < av.parts.size(); ++k)
            if (rig.partNode[k] >= 0) {
                const Node& n = rig.nodes[size_t(rig.partNode[k])];
                std::printf("susp:   %-34s kind %d K (%.3f %.3f %.3f) kW %d aimW %d aimN %d atN %d steerW %d axis %d stretch %d slide %d (%.3f %.3f %.3f)\n",
                            av.names ? (*av.names)(av.parts[k].objectName).c_str() : "?", int(n.kind), n.K[0], n.K[1], n.K[2], n.kWheel, n.aimWheel,
                            n.aimNode, n.atNode, n.steerWheel, n.axis, int(n.stretch), int(n.slide), n.attachT, n.limitT, n.limitR);
            }
    }
    return rig;
}

void evaluate(const Rig& rig, const std::vector<WheelLive>& wheels, Output& out) {
    out.node.resize(rig.nodes.size());
    out.wheelShift.assign(wheels.size(), V3{});
    auto hub = [&](int w) { return w >= 0 && size_t(w) < wheels.size() ? wheels[size_t(w)].hub : V3{}; };
    auto fwd = [&](int w) { return w >= 0 && size_t(w) < wheels.size() ? wheels[size_t(w)].forward : V3{0, 0, 1}; };
    auto delta = [&](int w) {
        return w >= 0 && size_t(w) < wheels.size() && size_t(w) < rig.hubRest.size() ? sub(wheels[size_t(w)].hub, rig.hubRest[size_t(w)]) : V3{};
    };
    for (size_t i = 0; i < rig.nodes.size(); ++i) {
        const Node& n = rig.nodes[i];
        Affine& a = out.node[i];
        const V3 Kl = n.atNode >= 0 ? out.node[size_t(n.atNode)].apply(n.K) : n.kWheel >= 0 ? hub(n.kWheel) : n.K;
        auto target = [&](int wheel, int node, const V3& rest) { return wheel >= 0 ? hub(wheel) : node >= 0 ? out.node[size_t(node)].apply(rest) : rest; };
        switch (n.kind) {
        case Kind::Hinge: {  // 0x6a0200 LAB_006a159e
            const V3 T = target(n.aimWheel, n.aimNode, n.aim);
            const V3 d = sub(T, Kl);
            M3 R = rotation(n.axis == 2 ? V3{0, 0, 1} : V3{1, 0, 0}, hingeAngle(d, n.look, n.axis));
            if (n.stretch) {  // [port] the skin's end bone at the target: the rest of the turn off the hinge plane, stretched
                const float l0 = len(sub(n.aim, n.K)), l1 = len(d);
                if (l0 > 1e-4f && l1 > 1e-4f) {
                    const V3 h = norm(mv(R, n.look)), dn = mul(d, 1.0f / l1);
                    const V3 c = cross(h, dn);
                    if (const float sn = len(c); sn > 1e-7f) R = mm(rotation(mul(c, 1.0f / sn), std::atan2(sn, dot(h, dn))), R);
                    const V3 u = norm(mv(R, n.look));
                    const float k = l1 / l0 - 1.0f;
                    M3 S = kI;
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c) S[size_t(r * 3 + c)] += k * u[size_t(r)] * u[size_t(c)];
                    R = mm(S, R);
                }
            }
            a = about(R, n.K, Kl);
            break;
        }
        case Kind::Upright:  // V:LINKM aimed at a wheel bone: its steer vector
            a = about(arcFromZ(fwd(n.steerWheel)), n.K, Kl);
            break;
        case Kind::Steer: {  // V:LINKR A: turns about LOCAL_LOOKAT by the steer angle
            const V3 l = norm(n.look);
            a = about(turnAbout(l, {0, 0, 1}, fwd(n.steerWheel)), n.K, Kl);
            if (n.slide) {  // the slider and the wheel on the axis at the hub's height, clamped to LOCAL_LIMIT
                const V3 h = hub(n.steerWheel);
                const float t = std::clamp((h[1] - Kl[1]) / l[1], n.limitT - n.limitR, n.limitT + n.limitR);
                a.t = add(a.t, mul(l, t - n.attachT));
                if (size_t(n.steerWheel) < out.wheelShift.size()) out.wheelShift[size_t(n.steerWheel)] = sub(add(Kl, mul(l, t)), h);
            }
            break;
        }
        case Kind::Axle: {  // V:LINKR A B C: aimed at the mid point, rolled with A - C
            const V3 A = target(n.aimWheel, -1, n.aim), Cc = target(n.aimWheel2, -1, n.aim2);
            const V3 d = sub(mul(add(A, Cc), 0.5f), Kl);
            const M3 Rh = rotation(n.axis == 2 ? V3{0, 0, 1} : V3{1, 0, 0}, hingeAngle(d, n.look, n.axis));
            const V3 r = sub(A, Cc), r0 = sub(n.aim, n.aim2);
            const float roll = std::atan2(r[1], r[0]) - std::atan2(r0[1], r0[0]);
            a = about(mm(rotation({0, 0, 1}, roll), Rh), n.K, Kl);
            break;
        }
        case Kind::Segment: {  // [port] C fixed, B with the hub
            const V3 B2 = add(n.B, mul(add(n.D, delta(n.wheel)), n.sB));
            a = segment(n.C, n.B, B2, n.thick);
            if (n.steerWheel >= 0) {
                const V3 u = norm(sub(B2, n.C));
                if (len(u) > 0.5f) a = compose(about(turnAbout(u, {0, 0, 1}, fwd(n.steerWheel)), n.C, n.C), a);
            }
            break;
        }
        case Kind::Translate: {  // [port] uprights / axles with the hub
            if (n.wheel2 >= 0) {
                const V3 d1 = delta(n.wheel), d2 = delta(n.wheel2);
                const V3 h1 = rig.hubRest[size_t(n.wheel)], h2 = rig.hubRest[size_t(n.wheel2)];
                const float dx = h1[0] - h2[0];
                const float roll = std::fabs(dx) > 1e-3f ? std::atan2(d1[1] - d2[1], dx) : 0.0f;
                const V3 pivot{0, n.pivot[1], n.pivot[2]};
                a = about(rotation({0, 0, 1}, roll), pivot, add(add(pivot, n.D), mul(add(d1, d2), 0.5f)));
            } else {
                const M3 R = n.steerWheel >= 0 ? arcFromZ(fwd(n.steerWheel)) : kI;
                a = about(R, n.pivot, add(add(n.pivot, n.D), delta(n.wheel)));
            }
            break;
        }
        }
    }
}

}  // namespace fuel::susp
