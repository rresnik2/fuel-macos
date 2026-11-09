// FUEL's engine particles (Particles_Z): see particles.hpp and notes p48-effects2.md.
#include "particles.hpp"

#include "assembly.hpp"
#include "world.hpp"

#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace fuel::particles {

namespace {

struct Reader {
    const std::vector<uint8_t>& b;
    size_t at = 0;
    void need(size_t n) const {
        if (at + n > b.size()) throw std::runtime_error("Particles_Z body too short");
    }
    uint32_t u32() {
        need(4);
        uint32_t v;
        std::memcpy(&v, b.data() + at, 4);
        at += 4;
        return v;
    }
    uint16_t u16() {
        need(2);
        uint16_t v;
        std::memcpy(&v, b.data() + at, 2);
        at += 2;
        return v;
    }
    float f32() {
        need(4);
        float v;
        std::memcpy(&v, b.data() + at, 4);
        at += 4;
        return v;
    }
};

Curve readCurve(Reader& r, int width) {  // { u16 flags; u32 nKeys; nKeys x (t, width floats) }
    Curve c;
    c.flags = r.u16();
    c.width = width;
    const uint32_t n = r.u32();
    if (n > 4096) throw std::runtime_error("Particles_Z curve too long");
    c.keys.resize(size_t(n) * size_t(1 + width));
    for (float& k : c.keys) k = r.f32();
    return c;
}


// Row-vector matrix: p' = p M (translation in 12..14).
V3 xformPoint(const std::array<float, 16>& m, const V3& p) {
    return {p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12], p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13],
            p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14]};
}
V3 xformDir(const std::array<float, 16>& m, const V3& p) {
    return {p[0] * m[0] + p[1] * m[4] + p[2] * m[8], p[0] * m[1] + p[1] * m[5] + p[2] * m[9], p[0] * m[2] + p[1] * m[6] + p[2] * m[10]};
}

}  // namespace

int Curve::eval(int idx, float t, float* out) const {  // 0x7d20d0 pattern
    const int n = count();
    const int stride = 1 + width;
    if (n <= 0) {
        for (int k = 0; k < width; ++k) out[k] = 1.0f;
        return idx;
    }
    idx = std::max(idx, 1);
    const float* key = keys.data();
    const float* last = nullptr;
    if (idx < n) {  // uVar11 != param_2
        float t0 = key[(idx - 1) * stride], t1 = key[idx * stride];
        while (idx < n && (t < t0 || t1 <= t)) {
            ++idx;
            if (idx == n) break;
            t0 = t1;
            t1 = key[idx * stride];
        }
        if (idx != n && !(flags & 8)) {
            const float* a = key + (idx - 1) * stride;
            const float* b = key + idx * stride;
            const float f = (t - t0) / (t1 - t0);
            for (int k = 0; k < width; ++k) out[k] = (b[1 + k] - a[1 + k]) * f + a[1 + k];
            return idx;
        }
        last = key + (idx - 1) * stride;  // pfVar9 - 5: the segment's first key (the last key past the end)
    } else {
        last = key + (std::min(idx, n) - 1) * stride;
    }
    for (int k = 0; k < width; ++k) out[k] = last[1 + k];
    return idx;
}

Emitter parseParticles(const std::vector<uint8_t>& body, const std::vector<uint8_t>& link) {  // 0x80bf20 / 0x80be10
    Emitter e;
    // The view distance: link offset 88 (Object_Z +0x5c), as for Mesh_Z / Lod_Z (p36). Link offsets 28 / 32 are 0 in
    // every shipped Particles_Z, so 0x80c540's near fade never applies (I: which link fields 0x80c540's +0x20 / +0x24 are).
    if (link.size() >= 92) {
        std::memcpy(&e.viewDist, link.data() + 88, 4);
        if (!std::isfinite(e.viewDist) || e.viewDist < 0) e.viewDist = 0;
    }
    Reader r{body};
    const uint32_t n = r.u32();
    if (n > 64) throw std::runtime_error("Particles_Z: too many elements");
    for (uint32_t i = 0; i < n; ++i) {
        Element el;
        el.maxParticles = r.u16();
        el.rate = int16_t(r.u16());
        el.duration = r.f32(), el.speed = r.f32(), el.spin = r.f32();
        el.offAxis = r.f32(), el.offAxisSpread = r.f32(), el.offPlane = r.f32(), el.offPlaneSpread = r.f32();
        el.unk20 = r.f32(), el.life = r.f32(), el.sizeX = r.f32(), el.sizeY = r.f32();
        el.wander = r.f32(), el.wanderPeriod = r.f32(), el.variation = r.f32(), el.gravity = r.f32();
        el.bounce = r.f32(), el.friction = r.f32();
        el.flags = r.u32();
        // File order c4c, c58, c64, c70, c88, c94, c7c (0x80be10 reads +0x7c last).
        el.c4c = readCurve(r, 2);
        el.c58 = readCurve(r, 4);
        el.c64 = readCurve(r, 4);
        el.c70 = readCurve(r, 1);
        el.c88 = readCurve(r, 3);
        el.c94 = readCurve(r, 3);
        el.c7c = readCurve(r, 1);
        el.material = r.u32();
        e.elements.push_back(std::move(el));
    }
    return e;
}

V3 direction(float a, float b) {  // 0x7d24f0
    const float sb = std::sin(b), cb = std::cos(b), sa = std::sin(-a), ca = std::cos(-a);
    return {cb * sa, ca, sa * sb};
}

namespace {

void spawn(const Element& E, ElementState& s, const Instance& inst, float dt, const Rnd& rnd) {  // 0x7d2fa0
    const float te = s.age * (1.0f / std::max(E.duration, 1e-6f));
    float k58[4], k4c[2], c88[3];
    s.k58 = E.c58.eval(s.k58, te, k58);  // sizeMul, spinMul, rateMul, gravityMul
    s.accum = float(int(E.rate)) * k58[2] * dt + s.accum;
    if (!(1.0f < s.accum)) return;
    const int n = int(s.accum);  // ftol
    s.accum -= float(n);
    s.k4c = E.c4c.eval(s.k4c, te, k4c);  // lifeMul, speedMul
    const float scale = inst.scale;
    const float speed = E.speed * k4c[1] * scale, life = E.life * k4c[0];
    const float sx = E.sizeX * k58[0] * scale, sy = k58[0] * E.sizeY * scale;
    const float spin = E.spin * k58[1], grav = E.gravity * k58[3];
    s.k88 = E.c88.eval(s.k88, te, c88);
    float r = (rnd() - 0.5f) * 2.0f;
    const float var = (1.0f - E.variation) + std::fabs(r) * E.variation;
    const float stepAge = dt / float(n);  // (flags & 8 keeps 0 at the first frame; port: dt / n)
    float age0 = 0;
    for (int i = 0; i < n; ++i) {
        if (int(s.live.size()) >= std::max<int>(E.maxParticles, 1)) return;
        Particle p;
        V3 pos{0, 0, 0}, dir{0, 1, 0};
        float l88 = (rnd() - 0.5f) * 2.0f;
        if ((E.flags & 0xc0) == 0) {
            const float a = E.offAxisSpread * s.wind * l88 + E.offAxis;
            const float b = ((rnd() - 0.5f) * 2.0f) * E.offPlaneSpread + E.offPlane;
            dir = direction(a, b);
        } else if ((E.flags & 0x40) == 0) {  // 0x80: a line / disc along x and z, straight up
            const float q = rnd();
            pos = {E.offPlane * l88, 0, ((q - 0.5f) * 2.0f) * E.offAxis};
            dir = {0, 1, 0};
        } else {  // 0x40: a disc of radius +0x10
            const float th = l88 * 3.1415927f;
            const float m = (rnd() - 0.5f) * 2.0f;
            const float cx = std::cos(th) * m, cz = m * std::sin(th);
            pos = {E.offAxis * cx, 0, E.offAxis * cz};
            dir = {cx * E.offPlane - pos[0], E.offAxisSpread, cz * E.offPlane - pos[2]};
            const float l = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            if (l > 0) dir = {dir[0] / l, dir[1] / l, dir[2] / l};
        }
        if ((E.flags & 0x200) == 0) {  // world space: through the emitter matrix (0x4126b0 / 0x40fa00), / scale
            pos = xformPoint(inst.world, pos);
            dir = xformDir(inst.world, dir);
            const float inv = 1.0f / scale;
            dir = {dir[0] * inv, dir[1] * inv, dir[2] * inv};
        }
        p.vel = {dir[0] * speed, dir[1] * speed, dir[2] * speed};
        p.gravity = grav, p.fall = 0, p.angle = 0;
        p.colour0 = {c88[0], c88[1], c88[2]};
        p.colour = {c88[0], c88[1], c88[2], 1};
        p.spin = spin, p.life = life, p.age = age0;
        p.pos = pos, p.render = pos;
        p.wanderAmp = E.wander * var * scale;
        p.sizeX = sx, p.sizeY = sy;
        if (0.001f < p.wanderAmp) {
            p.wanderFreq = 6.2831855f / (var * E.wanderPeriod);
            const float l = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            const V3 d = l > 0 ? V3{dir[0] / l, dir[1] / l, dir[2] / l} : V3{0, 1, 0};
            std::array<float, 4> w1{1, 0, 0, 0};
            if (!(d[2] == 0 && d[0] == 0)) {  // (d.z, 0, -d.x, 0) normalised (0x5bab70)
                const float q = std::sqrt(d[2] * d[2] + d[0] * d[0]);
                w1 = {d[2] / q, 0, -d[0] / q, 0};
            }
            const V3 w2{w1[1] * d[2] - w1[2] * d[1], w1[2] * d[0] - w1[0] * d[2], w1[0] * d[1] - d[0] * w1[1]};
            const float q2 = std::sqrt(w2[0] * w2[0] + w2[1] * w2[1] + w2[2] * w2[2] + 1.0f);  // (w2, 1) normalised
            p.wanderAxis = {w2[0] / q2 + w1[0], w2[1] / q2 + w1[1], w2[2] / q2 + w1[2], 1.0f / q2 + w1[3]};
        }
        s.live.push_back(p);
        age0 += stepAge;
    }
}

}  // namespace

void step(Instance& inst, float dt, const Rnd& rnd) {  // 0x7d2560 per element
    if (!inst.emitter) return;
    if (inst.state.size() != inst.emitter->elements.size()) inst.reset();
    for (size_t e = 0; e < inst.state.size(); ++e) {
        const Element& E = inst.emitter->elements[e];
        ElementState& s = inst.state[e];
        if (E.duration < s.age) {
            if (E.flags & 1) s.age = 0, s.k58 = s.k4c = s.k88 = 1;  // 0x717650 restart
            else s.stopped = true;
        }
        if (!s.stopped) spawn(E, s, inst, dt, rnd);
        for (size_t i = 0; i < s.live.size();) {
            Particle& p = s.live[i];
            if (!(p.age <= p.life)) {
                s.live.erase(s.live.begin() + long(i));
                continue;
            }
            const float t = p.age * (1.0f / p.life);
            float q[4], sy[1], g[1], c[3];
            p.k64 = E.c64.eval(p.k64, t, q);   // sizeX, spin, speed, alpha
            p.k70 = E.c70.eval(p.k70, t, sy);
            p.k7c = E.c7c.eval(p.k7c, t, g);
            p.k94 = E.c94.eval(p.k94, t, c);
            p.colour = {p.colour0[0] * c[0], p.colour0[1] * c[1], p.colour0[2] * c[2], q[3]};
            p.drawX = p.sizeX * q[0], p.drawY = p.sizeY * sy[0];
            const float before = p.age;
            p.age = dt + before;
            const float sp = q[2] * dt;
            for (int k = 0; k < 3; ++k) p.pos[size_t(k)] += sp * p.vel[size_t(k)];
            p.fall = p.gravity * g[0] * dt + p.fall;
            p.pos[1] -= p.fall * dt;
            p.angle = p.spin * q[1] * dt + p.angle;
            V3 rp = p.pos;
            if (0.01f < p.wanderAmp) {
                const float w = std::sin((dt + before) * p.wanderFreq) * p.wanderAmp;
                for (int k = 0; k < 3; ++k) rp[size_t(k)] += w * p.wanderAxis[size_t(k)];
            }
            if (E.flags & 0x200) rp = xformPoint(inst.world, p.pos);  // 0x7d3af0: local space
            // (0x400: collision with the emitter's plane list: no planes in the port.)
            p.render = rp;
            ++i;
        }
        s.age += dt;
    }
}

}  // namespace fuel::particles

// ---- Placed emitters ----

namespace fuel::particles {

namespace {
constexpr uint32_t kParticlesClass = 0xC56963DE, kNodeClass = 0x85D02528, kWorldClass = 0x39B67ECB;
}

std::vector<Placement> collectPlacedParticles(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, bool worldRootOnly) {
    std::vector<Placement> out;
    std::unordered_map<uint32_t, uint32_t> parent;
    std::vector<std::pair<uint32_t, Node>> nodes;
    for (uint32_t id : dpc.ids(kNodeClass)) {
        try {
            if (const auto b = dpc.body(*dpc.find(id))) {
                Node n = parseNodeBody(*b);
                parent[id] = n.parent;
                nodes.emplace_back(id, n);
            }
        } catch (const std::exception&) {
        }
    }
    uint32_t root = 0;
    if (worldRootOnly)
        for (uint32_t id : dpc.ids(kWorldClass))
            if (const auto b = dpc.body(*dpc.find(id)); b && b->size() >= 4) std::memcpy(&root, b->data(), 4);
    auto reaches = [&](uint32_t n) {
        for (int depth = 0; depth < 64 && n != 0; ++depth) {
            if (n == root) return true;
            const auto it = parent.find(n);
            if (it == parent.end()) return false;
            n = it->second;
        }
        return false;
    };
    std::unordered_map<uint32_t, std::shared_ptr<const Emitter>> cache;
    for (const auto& [id, n] : nodes) {
        if (n.object == 0) continue;
        if (std::fabs(n.world[12]) < 50 && std::fabs(n.world[14]) < 50) continue;  // library copies at the origin
        if (worldRootOnly && !reaches(id)) continue;
        std::shared_ptr<const Emitter> em;
        if (auto it = cache.find(n.object); it != cache.end()) {
            em = it->second;
        } else {
            for (const DpcFile* f : lookup) {
                const DpcFile::Entry* e = f ? f->find(n.object) : nullptr;
                if (!e) continue;
                if (e->classCrc == kParticlesClass) try {
                        if (const auto b = f->body(*e)) em = std::make_shared<const Emitter>(parseParticles(*b, f->link(*e)));
                    } catch (const std::exception&) {
                    }
                break;
            }
            cache[n.object] = em;
        }
        if (!em || em->elements.empty()) continue;
        Placement p;
        p.nodeId = id, p.world = n.world, p.lodScale = n.lodScale, p.emitter = em;
        out.push_back(std::move(p));
    }
    return out;
}

void PlacedSystem::add(std::vector<Placement> p) {
    for (Placement& q : p) {
        Live l;
        l.p = std::move(q);
        l.inst.emitter = l.p.emitter.get();
        l.inst.world = l.p.world;
        const float sx = std::sqrt(l.p.world[0] * l.p.world[0] + l.p.world[1] * l.p.world[1] + l.p.world[2] * l.p.world[2]);
        l.inst.scale = sx > 1e-4f ? sx : 1.0f;
        for (const Element& e : l.p.emitter->elements) l.neverSleep = l.neverSleep || (e.flags & 0x100);  // PK29: 0x677710
        placed_.push_back(std::move(l));
    }
}

void PlacedSystem::update(float dt, const V3& eye, float viewScale, float kFade) {
    const Rnd rnd = [this]() {  // the engine's 0x6460f0 (port: own generator)
        rng_ ^= rng_ << 13, rng_ ^= rng_ >> 17, rng_ ^= rng_ << 5;
        return float(rng_ >> 8) / 16777216.0f;
    };
    LodView view;
    view.viewScale = viewScale, view.Kfade = kFade;
    active_ = 0;
    for (Live& l : placed_) {
        const float dx = l.p.world[12] - eye[0], dy = l.p.world[13] - eye[1], dz = l.p.world[14] - eye[2];
        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
        const Emitter& e = *l.p.emitter;
        // 0x80c540: both distances 0 -> always drawn; else 0x672950 (lodFade) at s = node +0x44 x the view scale.
        l.fade = (e.viewDist == 0 && e.nearFade == 0) ? 1.0f : lodFade(e.viewDist, e.nearFade, 0, l.p.lodScale * viewScale, d, view);
        // PK29: ParticlesAgent_Z vf22 0x677710 sets +0x3c 0x4000000 when an element has flag 0x100, and vf18 0x677520
        // then never puts the emitter to sleep: out of view it keeps simulating (port: within the 1 km bound).
        const bool on = (l.fade >= 0.01f || l.neverSleep) && d < 1000.0f;  // port: no simulation beyond 1 km
        if (!on) {
            if (l.on) l.inst.reset();
            l.on = false;
            continue;
        }
        if (!l.on) {  // switched on: the emitter has been running (port: 2 s of warm-up in 0.1 s steps)
            l.inst.reset();
            int steps = 20;
            if (l.neverSleep)  // PK29: it ran since the world loaded: warm up for its longest particle life (<= 10 s)
                for (const Element& E : e.elements) steps = std::max(steps, std::min(100, int(std::ceil(E.life * 10))));
            for (int k = 0; k < steps; ++k) step(l.inst, 0.1f, rnd);
            l.on = true;
        }
        if (l.fade >= 0.01f) ++active_;
        if (dt > 0) step(l.inst, dt, rnd);
    }
}

void PlacedSystem::collect(std::vector<Quad>& quads, std::vector<Batch>& batches) const {
    quads.clear(), batches.clear();
    for (const Live& l : placed_) {
        if (!l.on || l.fade < 0.01f) continue;  // PK29: a never-sleeping emitter out of view is not drawn
        for (size_t e = 0; e < l.inst.state.size(); ++e) {
            const Element& E = l.p.emitter->elements[e];
            const ElementState& s = l.inst.state[e];
            if (s.live.empty()) continue;
            Batch b;
            b.material = E.material, b.first = uint32_t(quads.size());
            const int orient = (E.flags & 2) ? 2 : (E.flags & 4) ? 1 : 0;
            for (const Particle& p : s.live) {
                if (p.age <= 0) continue;
                Quad q;
                q.pos = p.render, q.angle = p.angle, q.sx = p.drawX, q.sy = p.drawY, q.orient = orient;
                q.rgba = {p.colour[0], p.colour[1], p.colour[2], p.colour[3] * l.fade};
                quads.push_back(q);
            }
            b.count = uint32_t(quads.size()) - b.first;
            if (b.count) batches.push_back(b);
        }
    }
}

}  // namespace fuel::particles
