// P47: animated world objects and cut-scenes in the viewer; see anim_view.h.
#include "anim_view.h"

#include "audio/ambience.hpp"
#include "audio/drive_audio.h"
#include "core/material_anim.hpp"
#include "core/sound.hpp"
#include "core/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>

namespace fs = std::filesystem;

simd_float4x4 simdFromRows(const fuel::anim::Mat4& w) {
    return simd_matrix(simd_make_float4(float(w[0]), float(w[1]), float(w[2]), float(w[3])),
                       simd_make_float4(float(w[4]), float(w[5]), float(w[6]), float(w[7])),
                       simd_make_float4(float(w[8]), float(w[9]), float(w[10]), float(w[11])),
                       simd_make_float4(float(w[12]), float(w[13]), float(w[14]), float(w[15])));
}

void attachParts(std::vector<simd_float4x4>& parts, const fuel::anim::Mat4& chassis, const fuel::anim::Mat4& node) {
    const simd_float4x4 d = simd_mul(simdFromRows(node), simd_inverse(simdFromRows(chassis)));
    for (simd_float4x4& m : parts) m = simd_mul(d, m);
}

// ---- WorldAnimation -------------------------------------------------------------------------------

namespace {

uint32_t linkName(const std::vector<const fuel::DpcFile*>& files, uint32_t id) {  // Node / Mesh link u32 0 = name CRC
    for (const fuel::DpcFile* f : files)
        if (const fuel::DpcFile::Entry* e = f->find(id)) {
            const std::vector<uint8_t> link = f->link(*e);
            uint32_t n = 0;
            if (link.size() >= 4) std::memcpy(&n, link.data(), 4);
            return n;
        }
    return 0;
}

std::array<float, 16> mulRows(const fuel::anim::Mat4& a, const std::array<float, 16>& b) {  // a then b (row vectors)
    std::array<float, 16> r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[size_t(i * 4 + k)] * double(b[size_t(k * 4 + j)]);
            r[size_t(i * 4 + j)] = float(s);
        }
    return r;
}

}  // namespace

void MaterialClones::init(const fuel::SkinnedModel& m, const std::vector<const fuel::DpcFile*>& files) {
    slots_.clear();
    for (size_t pi = 0; pi < m.parts.size(); ++pi)
        for (size_t mi = 0; mi < m.parts[pi].mesh.materials.size(); ++mi) {
            const uint32_t mid = m.parts[pi].mesh.materials[mi].materialId;
            Slot s;
            s.part = pi, s.slot = mi, s.name = linkName(files, mid);
            bool ok = false;
            for (const fuel::DpcFile* f : files)
                if (const fuel::DpcFile::Entry* e = f->find(mid)) {
                    try {
                        if (const auto b = f->body(*e)) s.cur = fuel::parseMaterialBody(*b), ok = true;
                    } catch (const std::exception&) {
                    }
                    break;
                }
            if (ok) slots_.push_back(s);
        }
}

void MaterialClones::apply(const fuel::anim::Clip& clip, double t) {
    for (size_t i = 0; i < clip.matTracks.size(); ++i) {
        bool sampled = false;
        fuel::anim::Clip::MatSample sm;
        for (Slot& s : slots_) {
            if (s.name != clip.matTracks[i].name) continue;
            if (!sampled) sm = clip.sampleMaterial(i, t), sampled = true;
            fuel::Material& m = s.cur;
            const auto d0 = m.diffuse, e0 = m.emissive;
            const auto uv0 = m.uvMatrix;
            if (sm.hasU) m.uvOffset[0] = float(sm.u);
            if (sm.hasV) m.uvOffset[1] = float(sm.v);
            if (sm.hasDiffuse)
                for (size_t k = 0; k < 3; ++k) m.diffuse[k] = float(sm.diffuse[k]);
            if (sm.hasEmissive)
                for (size_t k = 0; k < 3; ++k) m.emissive[k] = float(sm.emissive[k]);
            if (sm.hasAngle) m.uvAngle = float(sm.angle);
            fuel::updateUvMatrix(m);  // vt+0x44
            if (m.diffuse != d0 || m.emissive != e0 || m.uvMatrix != uv0) s.dirty = true;
        }
    }
}

size_t MaterialClones::push(fuel::MeshRenderer& r, const std::function<uint32_t(size_t part)>& key) {
    size_t n = 0;
    for (Slot& s : slots_)
        if (s.dirty) {
            s.dirty = false;
            if (const uint32_t k = key(s.part)) r.setBakedMaterial(k, s.slot, s.cur), ++n;
        }
    return n;
}

size_t WorldAnimation::init(const std::vector<const fuel::DpcFile*>& files, const fuel::DpcFile& usa1, fuel::MeshRenderer& r) {
    classes_.clear();
    fx_.clear();
    files_ = files;
    renderer_ = &r;
    for (uint32_t id : usa1.ids(fuel::cls::Lod)) {
        fuel::Lod lod;
        try {
            lod = fuel::parseLodBody(usa1.body(*usa1.find(id)).value());
        } catch (const std::exception&) {
            continue;
        }
        bool slot0 = false;
        for (const auto& [s, a] : lod.animations) slot0 = slot0 || s == 0;
        if (!slot0 || lod.objects.empty()) continue;
        auto c = std::make_unique<Class>();
        c->lod = id;
        std::string err;
        if (!c->model.load(files, id, &err)) continue;
        c->clip = c->idle = c->model.clipForSlot(0);
        if (c->clip == nullptr) continue;
        c->event = c->model.clipForSlot(334);
        bool any = false;
        for (const fuel::SkinnedModel::Part& p : c->model.parts) any = r.addBakedMesh(p.meshId, p.mesh, &err) || any;
        if (!any) continue;
        c->verts.resize(c->model.parts.size());
        for (const fuel::SkinnedModel::Part& p : c->model.parts) c->partName.push_back(linkName(files, p.meshId));
        c->partFlags.assign(c->model.parts.size(), 0);
        c->mats.init(c->model, files);  // PK29: the instance's material clones, by name
        play(*c, c->idle);  // AnimatedAgent_Z start: slot 0 (0x667190)
        if (std::getenv("FUEL_WORLD_ANIM_TRACE")) {
            auto info = [](const fuel::anim::Clip* k) {
                if (k == nullptr) return std::string("-");
                char b[96];
                std::snprintf(b, sizeof b, "%.2f s%s, %zu vis, %zu mat", k->duration, k->loops() ? " loop" : "", k->visTracks.size(), k->matTracks.size());
                return std::string(b);
            };
            std::printf("world animation class %08x: %zu parts, %zu materials, idle %s, event %s\n", id, c->model.parts.size(), c->mats.size(),
                        info(c->idle).c_str(), info(c->event).c_str());
        }
        classes_.push_back(std::move(c));
    }
    return classes_.size();
}

void WorldAnimation::play(Class& c, const fuel::anim::Clip* clip) {
    c.clip = clip;
    c.ch.play(clip);  // 0x7bcd40
    c.t = 0;
    c.upload = true;
    c.trackBone.clear();
    if (clip == nullptr) return;
    for (const fuel::anim::Clip::Track& tr : clip->tracks) c.trackBone.push_back(tr.en ? c.model.boneIndex(tr.bone) : -1);
    // 0x64f3f0 -> 0x7a6de0: every visibility track takes its LAST key's value at the play start (the frame's pass
    // samples the start time right after).
    for (size_t i = 0; i < clip->visTracks.size(); ++i) {
        const double v = clip->visLastValue(i);
        if (v >= 0) setGroup(c, clip->visTracks[i].name, v != 0);
    }
}

void WorldAnimation::setGroup(Class& c, uint32_t name, bool show) {  // 0x7a6ce0 / 0x7a6de0 on group 0x66ce90(name)
    for (size_t g = 0; g < c.partName.size(); ++g) {
        if (c.partName[g] != name) continue;
        uint32_t& f = c.partFlags[g];
        const uint32_t was = f;
        if (!show && (f & 6) == 0) f |= 2, c.upload = true, ++stats_.hides;  // 0x66cfa0: redraw
        else if (show && (f & 4) == 0 && (f & 2)) f &= ~2u, c.upload = true, ++stats_.shows;
        if (f != was && std::getenv("FUEL_WORLD_ANIM_TRACE"))
            std::printf("world animation %08x: group %08x %s at t %.3f\n", c.lod, name, show ? "shown" : "hidden", c.ch.time());
        return;  // the first group of that name
    }
}

void WorldAnimation::advance(float dt) {
    static std::vector<fuel::anim::AnimPass> ps;
    for (auto& c : classes_) {
        ps.clear();
        c->ch.advance(dt, ps);  // 0x7bc150 (+ 0x7bc3d0 loop markers)
        c->t = c->ch.time();
        if (!ps.empty()) passes(*c, ps), c->upload = true;
    }
    stepFx(dt);
}

void WorldAnimation::passes(Class& c, const std::vector<fuel::anim::AnimPass>& ps) {  // 0x7bc5c0 per pass
    const fuel::anim::Clip* clip = c.clip;
    if (clip == nullptr) return;
    static std::vector<std::pair<float, fuel::anim::Clip::Msg>> ev;
    for (const fuel::anim::AnimPass& p : ps) {
        for (size_t ti = 0; ti < clip->tracks.size(); ++ti) {  // every M track with an instance bone
            const int bone = ti < c.trackBone.size() ? c.trackBone[ti] : -1;
            if (bone < 0) continue;
            ev.clear();
            clip->eventWindow(int(ti), p.from, p.time, p.end, ev);
            for (const auto& [kt, m] : ev) {
                if (m.kind == 0x24) sound(c, bone, m, p.time);  // positional sound at the object's bone
                else if (m.kind == 0x20) startFx(c, bone, m.d, p.time);  // posted to IT_ANIMATED: its routine 0x517680
                // 5 / 6 / 0xe / 0xf / 0x12 / 0x25 / 0x6a / 0x6b: in no shipped world Animation_Z (census, pk29.md);
                // 0x10 / 0x11 (IK) and 0xc / 0xd (markers) do nothing to a world object here.
            }
        }
        c.mats.apply(*clip, p.time);  // 0x813e50: material tracks on the instance's material clones
        for (size_t i = 0; i < clip->visTracks.size(); ++i)  // 0x7a6ce0
            if (clip->visTracks[i].kn) setGroup(c, clip->visTracks[i].name, clip->visValue(i, p.time) != 0);
    }
    if (renderer_) stats_.materialWrites += c.mats.push(*renderer_, [&c](size_t part) { return c.model.parts[part].meshId; });
}

void WorldAnimation::sound(Class& c, int bone, const fuel::anim::Clip::Msg& m, double t) {
    ++stats_.sounds;
    if (std::getenv("FUEL_WORLD_ANIM_TRACE")) std::printf("world animation %08x: sound %08x (flags %x) at t %.3f, bone %d\n", c.lod, m.a, m.b, t, bone);
    if (audio_ == nullptr || !placements_) return;
    // SoundManager vt+0x9c(id a, flags b, node, bone): flags & 0x80 -> a 50 % chance (0x6460f0).
    auto it = samples_.find(m.a);
    if (it == samples_.end()) {
        std::shared_ptr<const fuel::snd::Pcm> pcm;
        for (const fuel::DpcFile* f : files_) {
            if (f->find(m.a) == nullptr) continue;
            try {
                pcm = fuel::snd::Pcm::from(fuel::decodeSound(*f, m.a));
            } catch (const std::exception& ex) {
                std::fprintf(stderr, "fuelview: world animation sound %08x: %s\n", m.a, ex.what());
            }
            break;
        }
        it = samples_.emplace(m.a, pcm).first;
    }
    if (!it->second) return;
    placements_(c.lod, placeTmp_);
    const auto bones = c.model.pose(c.clip, t);
    for (const auto& w : placeTmp_) {
        if ((m.b & 0x80) && (std::rand() % 2)) continue;  // [port RNG]
        const auto bw = mulRows(bones[size_t(bone)], w);
        const float sc = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        audio_->playSample(it->second, true, {bw[12], bw[13], bw[14]}, 1.0f, sc > 1e-4f ? sc : 1.0f);
    }
}

void WorldAnimation::startFx(Class& c, int bone, uint32_t data, double t) {  // IT_ANIMATED 0x517680 -> 0x4ee870
    ++stats_.fxStarts;
    if (!placements_) return;
    auto em = emitters_.find(data);
    if (em == emitters_.end()) {  // 0x4ee270: a ParticlesAgent_Z of the Particles_Z keyed `data` (link word 0)
        std::shared_ptr<const fuel::particles::Emitter> e;
        for (const fuel::DpcFile* f : files_) {
            for (uint32_t id : f->ids(0xc56963de)) {
                const fuel::DpcFile::Entry* en = f->find(id);
                if (en == nullptr) continue;
                const std::vector<uint8_t> link = f->link(*en);
                uint32_t key = 0;
                if (link.size() >= 4) std::memcpy(&key, link.data(), 4);
                if (key != data) continue;
                try {
                    if (const auto b = f->body(*en)) e = std::make_shared<const fuel::particles::Emitter>(fuel::particles::parseParticles(*b, link));
                } catch (const std::exception& ex) {
                    std::fprintf(stderr, "fuelview: world animation FX %08x: %s\n", data, ex.what());
                }
                if (e) break;
            }
            if (e) break;
        }
        em = emitters_.emplace(data, e).first;
    }
    if (!em->second || em->second->elements.empty()) return;
    placements_(c.lod, placeTmp_);
    if (std::getenv("FUEL_WORLD_ANIM_TRACE"))
        std::printf("world animation %08x: FX %08x at t %.3f on bone %d, %zu placements, %zu slots\n", c.lod, data, t, bone, placeTmp_.size(), fx_.size());
    const auto bones = c.model.pose(c.clip, t);
    for (size_t pi = 0; pi < placeTmp_.size(); ++pi) {
        Fx* f = nullptr;
        for (Fx& x : fx_)  // 0x4ee3f0: the slot with the same object, instance, bone, kind and FX (age = 0)
            if (x.keyed && x.lod == c.lod && x.placement == pi && x.bone == bone && x.data == data) f = &x;
        if (f == nullptr) {
            Fx n;
            n.lod = c.lod, n.placement = pi, n.bone = bone, n.data = data, n.emitter = em->second;
            n.inst.emitter = n.emitter.get();
            n.inst.reset();
            fx_.push_back(std::move(n));
            f = &fx_.back();
        }
        f->age = 0;
        f->place = placeTmp_[pi];
        // 0x677960(bone): follow the owner's bone; 0x6774e0 restart: the emission starts over (age, rate accumulator,
        // curve keys), the live particles stay.
        f->inst.world = mulRows(bones[size_t(bone)], f->place);
        const float sx = std::sqrt(f->inst.world[0] * f->inst.world[0] + f->inst.world[1] * f->inst.world[1] + f->inst.world[2] * f->inst.world[2]);
        f->inst.scale = sx > 1e-4f ? sx : 1.0f;
        for (fuel::particles::ElementState& s : f->inst.state) s.accum = 0, s.age = 0, s.k58 = s.k4c = s.k88 = 1, s.stopped = false;
    }
}

void WorldAnimation::stepFx(float dt) {
    if (fx_.empty()) return;
    const fuel::LodView view;
    const auto rnd = [this]() {  // the engine's 0x6460f0 (port: own generator, as PlacedSystem)
        rng_ ^= rng_ << 13, rng_ ^= rng_ >> 17, rng_ ^= rng_ << 5;
        return float(rng_ >> 8) / 16777216.0f;
    };
    std::sort(fx_.begin(), fx_.end(), [](const Fx& a, const Fx& b) { return a.lod < b.lod; });  // one pose per class
    uint32_t poseLod = 0;
    std::vector<fuel::anim::Mat4> pose;
    for (Fx& f : fx_) {
        if (f.keyed) {  // GameParticleMgr vf17 0x4ef150: age; past 3 x the emitter's duration -> released (0x4eedc0)
            f.age += dt;
            if (f.age > 3.0f * f.emitter->elements[0].duration) {
                f.keyed = false;
                for (fuel::particles::ElementState& s : f.inst.state) s.stopped = true;  // data vt+0x5c: stop emitting
            }
        }
        if (f.keyed && f.bone >= 0) {  // follow the bone (SuperSpray +0x29, owner +0x68, skeleton +0x6c, bone +0x70)
            if (f.lod != poseLod) {
                poseLod = f.lod, pose.clear();
                for (const auto& c : classes_)
                    if (c->lod == f.lod && c->clip) pose = c->model.pose(c->clip, c->t);
            }
            if (size_t(f.bone) < pose.size()) f.inst.world = mulRows(pose[size_t(f.bone)], f.place);
        }
        const float dx = f.inst.world[12] - eye_[0], dy = f.inst.world[13] - eye_[1], dz = f.inst.world[14] - eye_[2];
        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
        const fuel::particles::Emitter& E = *f.emitter;  // 0x80c540 as PlacedSystem (node scale x the view scale)
        f.fade = (E.viewDist == 0 && E.nearFade == 0) ? 1.0f : fuel::lodFade(E.viewDist, E.nearFade, 0, f.inst.scale * view.viewScale, d, view);
        if (d < 1000.0f && dt > 0) fuel::particles::step(f.inst, dt, rnd);  // port: no simulation beyond 1 km
    }
    if (static const bool tr = std::getenv("FUEL_WORLD_ANIM_TRACE") != nullptr; tr) {
        static int n = 0;
        if (++n % 300 == 0)
            for (const Fx& f : fx_) {
                size_t live = 0;
                for (const auto& st : f.inst.state) live += st.live.size();
                std::printf("world animation FX %08x of %08x bone %d: %s, age %.1f, %zu particles, fade %.2f at (%.1f, %.1f, %.1f) d %.0f\n", f.data, f.lod,
                            f.bone, f.keyed ? "keyed" : "released", f.age, live, f.fade, f.inst.world[12], f.inst.world[13], f.inst.world[14],
                            std::sqrt((f.inst.world[12] - eye_[0]) * (f.inst.world[12] - eye_[0]) + (f.inst.world[14] - eye_[2]) * (f.inst.world[14] - eye_[2])));
            }
    }
    fx_.erase(std::remove_if(fx_.begin(), fx_.end(),
                             [](const Fx& f) {
                                 if (f.keyed) return false;
                                 for (const auto& s : f.inst.state)
                                     if (!s.live.empty()) return false;
                                 return true;
                             }),
              fx_.end());
}

void WorldAnimation::appendFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches) const {
    for (const Fx& f : fx_) {
        if (f.fade < 0.01f) continue;
        for (size_t e = 0; e < f.inst.state.size() && e < f.emitter->elements.size(); ++e) {
            const fuel::particles::Element& E = f.emitter->elements[e];
            fuel::particles::Batch b;
            b.material = E.material, b.first = uint32_t(quads.size());
            const int orient = (E.flags & 2) ? 2 : (E.flags & 4) ? 1 : 0;
            for (const fuel::particles::Particle& p : f.inst.state[e].live) {
                if (p.age <= 0) continue;
                fuel::particles::Quad q;
                q.pos = p.render, q.angle = p.angle, q.sx = p.drawX, q.sy = p.drawY, q.orient = orient;
                q.rgba = {p.colour[0], p.colour[1], p.colour[2], p.colour[3] * f.fade};
                quads.push_back(q);
            }
            b.count = uint32_t(quads.size()) - b.first;
            if (b.count) batches.push_back(b);
        }
    }
}

bool WorldAnimation::start(uint32_t lod) {
    for (auto& c : classes_)
        if (c->lod == lod && c->event && !c->started) {
            c->started = true;  // 0x518170: +0x74 = 1, play slot 334 (0x667a00 ecx 0x14e)
            play(*c, c->event);
            return true;
        }
    return false;
}

void WorldAnimation::reset(uint32_t lod) {
    for (auto& c : classes_)
        if (c->lod == lod) c->started = false, play(*c, c->idle);  // 0x517f80 -> 0x667190: slot 0, +0x74 = 0
}

void WorldAnimation::upload(fuel::MeshRenderer& r, const fuel::OrbitCamera* cam) {
    static const bool all = std::getenv("FUEL_WORLD_ANIM_ALL") != nullptr;  // P92: skin every class (to compare)
    for (auto& c : classes_) {
        if (cam != nullptr && !all) {
            bool seen = false;
            for (size_t i = 0; i < c->model.parts.size() && !seen; ++i) seen = r.bakedMeshDrawable(c->model.parts[i].meshId, *cam);
            if (!seen) continue;
        }
        if (!c->upload) continue;  // PK29: the pose of a stopped channel holds
        c->upload = false;
        const auto bones = c->model.pose(c->clip, std::max(0.0, c->t));
        for (size_t i = 0; i < c->model.parts.size(); ++i) {
            if (c->partFlags[i] & 2) {  // PK29: a hidden group (0x7a6ce0): drawn as nothing (collapsed vertices)
                c->verts[i].assign(c->model.parts[i].verts.size() * fuel::BakedMesh::kStride, 0);
            } else {
                c->model.skin(c->model.parts[i], bones, c->verts[i]);
            }
            r.updateBakedVertices(c->model.parts[i].meshId, c->verts[i]);
        }
    }
}

// ---- RtcView --------------------------------------------------------------------------------------

bool RtcView::play(const std::string& dataDir, const std::string& name, const fuel::anim::Mat4& father, int flags,
                   fuel::ResourceLibrary& library, fuel::MeshRenderer& r) {
    stop(r);
    const std::string path = (fs::path(dataDir) / "CommonComponent/RTC" / (name + ".DPC")).string();
    dpc_ = nullptr;
    for (const auto& f : library.files())
        if (f->path() == path) dpc_ = f.get();
    if (dpc_ == nullptr) {
        try {
            auto f = std::make_unique<fuel::DpcFile>(path);
            dpc_ = f.get();
            library.add(std::move(f));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: RTC %s: %s\n", name.c_str(), e.what());
            return false;
        }
    }
    std::string err;
    bool parsed = false;
    for (uint32_t id : dpc_->ids(0x2A11CEE8))  // Rtc_Z
        if (auto b = dpc_->body(*dpc_->find(id))) parsed = fuel::rtc::parse(*b, rtc_, &err);
    if (!parsed) {
        std::fprintf(stderr, "fuelview: RTC %s: %s\n", name.c_str(), err.c_str());
        return false;
    }
    std::vector<const fuel::DpcFile*> files = {dpc_};
    for (const auto& f : library.files())
        if (f.get() != dpc_) files.push_back(f.get());
    fxFiles_ = files;  // PK13: where 0x730370 resolves node FX resources
    objs_.clear();
    for (size_t i = 0; i < rtc_.objects.size(); ++i) {  // F: shown while playing (0x728da0), hidden at the end
        const fuel::rtc::Object& o = rtc_.objects[i];
        Obj ob;
        ob.e = int(o.e);
        const fuel::DpcFile::Entry* e = dpc_->find(o.res);
        if (e == nullptr || o.e >= rtc_.binds.size()) continue;
        if (e->classCrc == fuel::cls::Mesh) {
            ob.mesh = o.res;
        } else {
            ob.skin = std::make_unique<fuel::SkinnedModel>();
            if (!ob.skin->load(files, o.res, &err)) continue;
            ob.clip = o.anim ? ob.skin->clipById(o.anim) : nullptr;
            if (ob.clip && !ob.clip->matTracks.empty()) ob.mats.init(*ob.skin, files);  // PK29
            for (size_t p = 0; p < ob.skin->parts.size(); ++p) {
                const uint32_t key = 0xF1000000u + uint32_t(i << 8) + uint32_t(p);
                if (r.addBakedMesh(key, ob.skin->parts[p].mesh, &err)) ob.keys.push_back(key);
            }
        }
        objs_.push_back(std::move(ob));
    }
    name_ = name;
    father_ = father;
    flags_ = flags;
    cam_ = -1;
    attachNode_ = -1;
    slot_ = {};
    fx_.clear();
    played_ = true;
    hudHidden_ = false;
    hold_ = {};  // Start 0x53bcb0: +0x6e4 / +0x6e8 cleared
    hiddenBind_.assign(rtc_.binds.size(), 0);
    cables_.clear();  // Start 0x53bcb0: the 16 AMEO records cleared
    {  // P96: global bone ids before the RTC: the loaded skeletons' name lists (Skel_Z::Load 0x7b4830) and the bones of
       // their instances (0x66c0e0: the world's animated objects, vehicles and riders are built before the race start)
        registry_ = fuel::rtc::BoneRegistry();
        for (const fuel::DpcFile* f : files) {
            if (f == dpc_) continue;
            for (uint32_t id : f->ids(fuel::cls::Skel)) {
                const fuel::DpcFile::Entry* e = f->find(id);
                const auto b = e ? f->body(*e) : std::nullopt;
                if (!b) continue;
                try {
                    const fuel::Skel sk = fuel::parseSkelBody(*b);
                    for (const auto& l : sk.palettes)
                        for (uint32_t c : l) registry_.id(c);
                    for (const fuel::Bone& bn : sk.bones) registry_.id(bn.name);
                } catch (const std::exception&) {
                }
            }
        }
    }
    pending_.clear();
    samples_.clear();
    if (std::getenv("FUEL_RTC_TRACE")) {  // PK13: every event (node -1 = RTC level), the raw message words
        auto dump = [&](int n, const std::vector<fuel::rtc::EventKey>& ev) {
            for (const auto& k : ev)
                for (const auto& m : k.msgs)
                    std::printf("RTC %s event t %.4f node %d (E%d) type %#x a %#x b %#x c %#x (%g) d %#x\n", name.c_str(), k.time, n,
                                n >= 0 ? int(rtc_.nodes[size_t(n)].e) : -1, m.type, m.a, m.b, m.c, [&] { float f; std::memcpy(&f, &m.c, 4); return double(f); }(), m.d);
        };
        dump(-1, rtc_.events);
        for (size_t n = 0; n < rtc_.nodes.size(); ++n) dump(int(n), rtc_.nodes[n].events);
        for (size_t i = 0; i < rtc_.objects.size(); ++i)
            std::printf("RTC %s object F%zu e %u res %08x anim %08x\n", name.c_str(), i, rtc_.objects[i].e, rtc_.objects[i].res, rtc_.objects[i].anim);
    }
    player_ = std::make_unique<fuel::rtc::Player>(rtc_);
    player_->start();
    std::printf("RTC %s: %.2f s, %zu nodes, %zu cameras, %zu objects\n", name.c_str(), rtc_.duration, rtc_.nodes.size(), rtc_.cameras.size(),
                objs_.size());
    return true;
}

// P96: 0x103 'AMEO' (0x5389d0, asm 0x538aa5-0x538be8): a = 1..4 on the helicopter's node (A1).
void RtcView::ameo(const fuel::rtc::Fired& e) {
    if (e.node < 0 || size_t(e.node) >= rtc_.nodes.size() || cables_.size() >= 16) return;
    const int a = int(e.msg.a);
    const uint16_t bind = rtc_.nodes[size_t(e.node)].e;
    size_t oi = objs_.size();  // 0x539710: the sub-anim whose node is the event node
    for (size_t i = 0; i < objs_.size(); ++i)
        if (objs_[i].e == int(bind) && objs_[i].skin) oi = i;
    if (oi == objs_.size()) return;
    const fuel::Skel& sk = objs_[oi].skin->skeleton();
    std::vector<uint32_t> names;
    for (const fuel::Bone& b : sk.bones) names.push_back(b.name);
    const int joint = fuel::rtc::ameoJoint(registry_, sk.palettes, names, a);  // instance list [a + 0x49], 0 -> nothing
    auto bindNamed = [&](const std::string& n) -> int {  // 0x536ab0: E +4 id
        const uint32_t c = fuel::asoboCrc(n);
        for (size_t b = 0; b < rtc_.binds.size(); ++b)
            if (rtc_.binds[b].nodeId == c) return int(b);
        return -1;
    };
    int target = vehicleMesh_.empty() ? -1 : bindNamed(vehicleMesh_ + "_" + std::to_string(a));
    if (target < 0) target = bindNamed("MESH_ID_MUSCLE_OFF_2_" + std::to_string(a));  // 0x9b5f9c
    if (target < 0 || joint < 0) return;
    cables_.push_back({oi, joint, size_t(target)});  // 0x53bf80
    if (std::getenv("FUEL_RTC_TRACE"))
        std::printf("RTC %s: AMEO %d -> joint %d (%08x) to bind %d (%s_%d)\n", name_.c_str(), a, joint, names[size_t(joint)], target,
                    vehicleMesh_.c_str(), a);
}

// PK13: 0x539140 (cache lookup / instantiate) + the 0x20 / 0x22 / 0x23 bodies of vf44 0x5389d0 (asm 0x539012-0x539119).
void RtcView::nodeFx(const fuel::rtc::Fired& e) {
    const fuel::rtc::NodeFxOp op = fuel::rtc::nodeFxOp(e.msg.type);
    if (op == fuel::rtc::NodeFxOp::None || e.node < 0) return;
    NodeFx* f = nullptr;
    for (NodeFx& x : fx_)
        if (x.node == e.node && x.data == e.msg.d) f = &x;
    if (f == nullptr) {  // 0x730370: the resource keyed d -> a ParticlesAgent_Z under the RTC world
        std::shared_ptr<const fuel::particles::Emitter> em;
        for (const auto* file : fxFiles_) {
            for (uint32_t id : file->ids(0xc56963de)) {  // Particles_Z: link word 0 = the resource key [I: the handle key]
                const fuel::DpcFile::Entry* en = file->find(id);
                if (en == nullptr) continue;
                const std::vector<uint8_t> link = file->link(*en);
                uint32_t key = 0;
                if (link.size() >= 4) std::memcpy(&key, link.data(), 4);
                if (key != e.msg.d && id != e.msg.d) continue;
                try {
                    if (const auto b = file->body(*en)) em = std::make_shared<const fuel::particles::Emitter>(fuel::particles::parseParticles(*b, link));
                } catch (const std::exception& ex) {
                    std::fprintf(stderr, "fuelview: RTC %s FX %08x: %s\n", name_.c_str(), e.msg.d, ex.what());
                }
                if (em) break;
            }
            if (em) break;
        }
        if (!em) {
            if (std::getenv("FUEL_RTC_TRACE")) std::printf("RTC %s: node FX %08x not found\n", name_.c_str(), e.msg.d);
            return;  // 0x539140 returns 0: only the vf44 tail runs
        }
        NodeFx n;
        n.node = e.node, n.data = e.msg.d, n.emitter = em;
        n.inst.emitter = n.emitter.get();
        n.inst.reset();
        fx_.push_back(std::move(n));
        f = &fx_.back();
        if (std::getenv("FUEL_RTC_TRACE"))
            std::printf("RTC %s: node FX %08x on node %d (%zu elements)\n", name_.c_str(), e.msg.d, e.node, em->elements.size());
    }
    if (op == fuel::rtc::NodeFxOp::Hide) {  // flags 0x40000 (hidden), 0x677430 (stop)
        f->shown = false;
        return;
    }
    if (op == fuel::rtc::NodeFxOp::Start) f->inst.reset();  // 0x6774e0: restart
    f->shown = true;
}

void RtcView::appendFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches, const fuel::particles::V3& eye) const {
    static const bool off = std::getenv("FUEL_RTC_NO_FX") != nullptr;  // PK13: compare frames without the node FX
    if (off) return;
    const fuel::LodView view;
    for (const NodeFx& f : fx_) {
        if (!f.shown || !f.emitter) continue;
        const float dx = f.inst.world[12] - eye[0], dy = f.inst.world[13] - eye[1], dz = f.inst.world[14] - eye[2];
        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
        const fuel::particles::Emitter& E0 = *f.emitter;  // 0x80c540 as PlacedSystem (node scale x the view scale)
        const float fade = (E0.viewDist == 0 && E0.nearFade == 0) ? 1.0f : fuel::lodFade(E0.viewDist, E0.nearFade, 0, f.inst.scale * view.viewScale, d, view);
        if (fade < 0.01f) continue;
        for (size_t e = 0; e < f.inst.state.size() && e < E0.elements.size(); ++e) {
            const fuel::particles::Element& E = E0.elements[e];
            fuel::particles::Batch b;
            b.material = E.material, b.first = uint32_t(quads.size());
            const int orient = (E.flags & 2) ? 2 : (E.flags & 4) ? 1 : 0;
            for (const fuel::particles::Particle& p : f.inst.state[e].live) {
                if (p.age <= 0) continue;
                fuel::particles::Quad q;
                q.pos = p.render, q.angle = p.angle, q.sx = p.drawX, q.sy = p.drawY, q.orient = orient;
                q.rgba = {p.colour[0], p.colour[1], p.colour[2], p.colour[3] * fade};
                quads.push_back(q);
            }
            b.count = uint32_t(quads.size()) - b.first;
            if (b.count) batches.push_back(b);
            if (static const bool tr = std::getenv("FUEL_RTC_FX_TRACE") != nullptr; tr && b.count) {
                float amax = 0, smax = 0, smin = 1e9f;
                for (uint32_t k = b.first; k < b.first + b.count; ++k)
                    amax = std::max(amax, quads[k].rgba[3]), smax = std::max(smax, quads[k].sx), smin = std::min(smin, quads[k].sx);
                const auto& q = quads[b.first];
                std::printf("RTC FX draw: %u quads material %08x orient %d fade %.2f d %.1f alpha max %.3f size %.2f..%.2f rgb %.2f %.2f %.2f first (%.1f, %.1f, %.1f)\n",
                            b.count, b.material, orient, fade, d, amax, smin, smax, q.rgba[0], q.rgba[1], q.rgba[2], q.pos[0], q.pos[1], q.pos[2]);
            }
        }
    }
}

void RtcView::stopSounds() {
    if (audio_)
        for (const Voice& v : voices_) audio_->stopSample(v.handle);
    voices_.clear();
}

void RtcView::stop(fuel::MeshRenderer& r) {
    if (!player_) return;
    stopSounds();  // End 0x536230: SoundManager vt+0x6c(0)
    pending_.clear();
    hold_ = {};
    cables_.clear();
    fx_.clear();  // PK13: End 0x536230 -> 0x539310: every cached node FX detached and removed
    slot_ = {};
    attachNode_ = -1;
    player_.reset();
    objs_.clear();
    r.setRtcInstances({});
    name_.clear();
}

void RtcView::skip(fuel::MeshRenderer& r) {
    if (!player_ || !(flags_ & 0x20)) return;
    const float t = player_->time(), dur = rtc_.duration;
    if (t <= 0.25f || t >= dur - 1) return;
    if (flags_ & 0x80) player_->seek(dur - 1 + 0.01f, &pending_);  // SKIP_BEFORE_END: the last second plays (0x72aeb0)
    else stop(r);                                                  // message 0x75 [I: stop]
}

RtcView::Frame RtcView::update(float dt, fuel::MeshRenderer& r, fuel::rtc::Fader* fader, fuel::DriveAudio* audio) {
    Frame fr;
    if (!player_) return fr;
    if (played_) {  // PK13: Play pre-start 0x53b6b0 (flag 2: radial-blur flag 0x8000 off on every viewport)
        played_ = false;
        fr.clearRadialBlur = fuel::rtc::PlayFlags::clearsRadialBlurAtPlay(flags_);
    }
    bool detachFinal = false;
    if (audio_ != audio) stopSounds(), audio_ = audio;
    std::vector<fuel::rtc::Fired> fired = std::move(pending_);
    pending_.clear();
    player_->advance(dt, fired);
    std::vector<fuel::anim::Mat4> world;
    player_->evaluate(father_, world);
    // A node's world position (sounds follow their node: SoundManager vt+0x9c gets the node handle).
    auto nodePos = [&](int n) -> std::array<double, 3> {
        if (n < 0 || size_t(n) >= rtc_.nodes.size() || rtc_.nodes[size_t(n)].e >= world.size()) return {father_[12], father_[13], father_[14]};
        const fuel::anim::Mat4& m = world[rtc_.nodes[size_t(n)].e];
        return {m[12], m[13], m[14]};
    };
    // P96: the node's scale as 0x733080 reads it (length of the world matrix's first column): the sound distances x it.
    auto nodeScale = [&](int n) -> float {
        if (n < 0 || size_t(n) >= rtc_.nodes.size() || rtc_.nodes[size_t(n)].e >= world.size()) return 1.0f;
        const fuel::anim::Mat4& m = world[rtc_.nodes[size_t(n)].e];
        return float(std::sqrt(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]));
    };
    for (const fuel::rtc::Fired& e : fired) {
        switch (e.msg.type) {
            case 0x30:  // camera cut to the camera with id == d (the first one takes the viewport)
                if (const int c = player_->cameraById(e.msg.d); c >= 0) cam_ = c;
                break;
            case 0x10b: ++fr.respawns; break;
            case 0x10e:  // P76: RTC level (TR_DEATH): pause hold, 0x536d40 -> 0x53c740
                if (e.node < 0) hold_.start();
                break;
            case 0x10f:  // P76: node level (0x5389d0): hidden in daylight (day factor 0x5b98d0 >= 0.8), else nothing
                if (e.node >= 0 && size_t(e.node) < rtc_.nodes.size() &&
                    fuel::rtc::hidesInDaylight(fuel::snd::dayFactor(r.timeOfDay()))) {
                    const size_t root = rtc_.nodes[size_t(e.node)].e;
                    if (root < hiddenBind_.size()) hiddenBind_[root] = 1;
                    for (int pass = 0; pass < 64; ++pass) {  // 0x65eed0: the children too
                        bool more = false;
                        for (size_t b = 0; b < rtc_.binds.size(); ++b) {
                            const int pe = rtc_.binds[b].parentE;
                            if (!hiddenBind_[b] && pe >= 0 && hiddenBind_[size_t(pe)]) hiddenBind_[b] = 1, more = true;
                        }
                        if (!more) break;
                    }
                }
                break;
            case 0xc6:  // ---- PK28: 0x536d40 -> 0x7aabe0: every world Omni_Z with +0x60 0x8000000 (not 0x10000000) loses
            case 0xc7:  // (0xc6) / regains (0xc7) its gather flag 0x400000. No shipped RTC sends these (notes pk28.md).
                r.setRtcOmnis(e.msg.type == 0xc7);
                break;  // ---- PK28 end
            case 0x10c: hudHidden_ = false; break;  // SetHudsVisibleState on [I order]
            case 0x10d: hudHidden_ = true; break;   // off
            case 0x98:  // P67: fades (vf43 0x536d40 -> fader 0x438030 / 0x438190)
                if (fader) fuel::rtc::fadeMessage(*fader, e.msg.a, e.skipping);
                break;
            case 0xa3:  // P67: STARTRTCFX (command 0x15, 0x53e740); kind = a & 0xffff. Only SHAKECAM is ported.
                if ((e.msg.a & 0xffff) == 1) fr.shake = true;
                break;
            case 0x24: {  // P67: play Sound_Z a (b = flags, 0x80: 50 % chance) at the node, or 2D for RTC-level events;
                // not while skipping (0x729de0). Volume c is not passed on by the player (vt+0x9c gets id, flags, node).
                if (e.skipping || audio_ == nullptr) break;
                if ((e.msg.b & 0x80) && (std::rand() % 2)) break;  // [port RNG; FUEL: 0x6460f0 Park-Miller]
                auto it = samples_.find(e.msg.a);
                if (it == samples_.end()) {
                    std::shared_ptr<const fuel::snd::Pcm> pcm;
                    try {
                        pcm = fuel::snd::Pcm::from(fuel::decodeSound(*dpc_, e.msg.a));
                    } catch (const std::exception& ex) {
                        std::fprintf(stderr, "fuelview: RTC %s sound %08x: %s\n", name_.c_str(), e.msg.a, ex.what());
                    }
                    it = samples_.emplace(e.msg.a, pcm).first;
                }
                const int h = audio_->playSample(it->second, e.node >= 0, nodePos(e.node), 1.0f, nodeScale(e.node));
                if (h >= 0) voices_.push_back({e.msg.a, e.node, h});
                break;
            }
            case 0x103: ameo(e); break;  // P96: the helicopter's cable joints
            case 0xf1:  // P67 / PK13: the player vehicle hangs on the node (0x53bfc0 -> 0x514b70), blend time = c (float)
                attachNode_ = e.node;
                {
                    float blend;
                    std::memcpy(&blend, &e.msg.c, 4);
                    slot_.attach(blend);
                }
                break;
            case 0xf2:  // PK13: detach (0x514b70, invalid handle): elapsed = duration, final 0x5148a0(0) = the node's pose
                if (slot_.active) slot_.elapsed = slot_.duration, slot_.active = false, detachFinal = true;
                break;
            case 0x20:
            case 0x22:
            case 0x23: nodeFx(e); break;  // PK13: RTC node FX (0x539140); 0x21 has no handler (vf44 -> vf43 ignores it)
            case 0x25:  // P67: stop Sound_Z a (SoundManager vt+0xa4, fade -1 = at once)
                for (auto v = voices_.begin(); v != voices_.end();) {
                    if (v->id == e.msg.a) {
                        if (audio_) audio_->stopSample(v->handle);
                        v = voices_.erase(v);
                    } else {
                        ++v;
                    }
                }
                break;
            default: break;
        }
    }
    // P76: 0x53c770 after Advance: the 0x10e hold lets go after 0.5 s and seeks past the loop end (events next update).
    hold_.update(dt, *player_, &pending_);
    if (audio_)
        for (const Voice& v : voices_)
            if (v.node >= 0) audio_->moveSample(v.handle, nodePos(v.node), nodeScale(v.node));
    std::vector<fuel::MeshInstance> list;
    for (Obj& o : objs_) {
        if (size_t(o.e) < hiddenBind_.size() && hiddenBind_[size_t(o.e)]) continue;  // P76: 0x10f
        const simd_float4x4 w = simdFromRows(world[size_t(o.e)]);
        if (o.mesh) {
            list.push_back({o.mesh, w});
            continue;
        }
        if (o.clip) {  // the object's own animation (helicopter rotor), looping on its clock
            o.t += dt;
            if (o.clip->loops())
                while (o.t > o.clip->duration) o.t -= o.clip->duration;
        }
        std::vector<std::pair<size_t, fuel::anim::Vec3>> joints;  // P96: 0x53ba20 after Advance: joint origin = target
        if (!cables_.empty()) {
            const fuel::anim::Mat4 inv = fuel::anim::invertAffine(world[size_t(o.e)]);  // the event node's world (0x412760)
            for (const Cable& c : cables_) {
                if (&objs_[c.obj] != &o || c.target >= world.size()) continue;
                const fuel::anim::Mat4& t = world[c.target];  // node +0xd4 +0x30
                joints.push_back({size_t(c.joint),
                                  {t[12] * inv[0] + t[13] * inv[4] + t[14] * inv[8] + inv[12], t[12] * inv[1] + t[13] * inv[5] + t[14] * inv[9] + inv[13],
                                   t[12] * inv[2] + t[13] * inv[6] + t[14] * inv[10] + inv[14]}});
            }
        }
        const auto bones = o.skin->pose(o.clip, o.t, joints);
        if (o.clip && o.mats.size()) {  // PK29: 0x813e50 (the helicopter's blinking beacon 04034a5f)
            o.mats.apply(*o.clip, o.t);
            o.mats.push(r, [&o](size_t part) { return part < o.keys.size() ? o.keys[part] : 0u; });
        }
        std::vector<uint8_t> v;
        for (size_t p = 0; p < o.keys.size(); ++p) {
            o.skin->skin(o.skin->parts[p], bones, v);
            r.updateBakedVertices(o.keys[p], v);
            list.push_back({o.keys[p], w});
        }
    }
    r.setRtcInstances(list);
    if (cam_ >= 0) {
        const fuel::rtc::Player::CamPose p = player_->camera(size_t(cam_), world);
        fr.camera = true;
        fr.cam.explicitPose = true;
        fr.cam.eye = simd_make_float3(float(p.eye[0]), float(p.eye[1]), float(p.eye[2]));
        fr.cam.target = simd_make_float3(float(p.target[0]), float(p.target[1]), float(p.target[2]));
        fr.cam.roll = float(p.roll);
        fr.cam.zNear = float(p.nearZ);
        fr.cam.fovH43 = float(p.fov);
    }
    fr.hudHidden = hudHidden_;
    // PK13: 0x5148a0 after Advance (RtcScripted_G::Update 0x53c150): the slot's pose, from the node's world position and
    // rotation and the vehicle's pose now; a detach (0xf2) takes one final step(0) and clears the slot.
    if ((slot_.active || detachFinal) && attachNode_ >= 0 && size_t(attachNode_) < rtc_.nodes.size() && rtc_.nodes[size_t(attachNode_)].e < world.size()) {
        const fuel::anim::Mat4& n = world[rtc_.nodes[size_t(attachNode_)].e];
        fuel::anim::Vec3 p;
        fuel::anim::Quat q;
        slot_.step(detachFinal ? 0.0f : dt, {n[12], n[13], n[14]}, fuel::rtc::quatOfRows(n), {attachedNow_[12], attachedNow_[13], attachedNow_[14]},
                   fuel::rtc::quatOfRows(attachedNow_), p, q);
        fr.attached = true, fr.attach = fuel::rtc::poseMatrix(p, q);
        if (std::getenv("FUEL_RTC_TRACE") && (slot_.elapsed <= dt || detachFinal))
            std::printf("RTC %s: attach slot t %.3f blend %.3f%s node scale %.4f pos (%.2f, %.2f, %.2f)\n", name_.c_str(), player_->time(), slot_.duration,
                        slot_.done ? " (node pose)" : " (easing)", std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]), p[0], p[1], p[2]);
        if (detachFinal) attachNode_ = -1;
    }
    // PK13: the node FX follow their node (0x677960: SuperSpray +0x29) and run; not while the 0x10e hold pauses special
    // effects (0xa65878).
    if (!hold_.holding)
        for (NodeFx& f : fx_) {
            if (!f.shown || f.node < 0 || size_t(f.node) >= rtc_.nodes.size() || rtc_.nodes[size_t(f.node)].e >= world.size()) continue;
            const fuel::anim::Mat4& n = world[rtc_.nodes[size_t(f.node)].e];
            for (size_t i = 0; i < 16; ++i) f.inst.world[i] = float(n[i]);
            const float sx = std::sqrt(f.inst.world[0] * f.inst.world[0] + f.inst.world[1] * f.inst.world[1] + f.inst.world[2] * f.inst.world[2]);
            f.inst.scale = sx > 1e-4f ? sx : 1.0f;
            uint32_t& rng = fxRng_;
            fuel::particles::step(f.inst, dt, [&rng]() {  // the engine's 0x6460f0 (port: own generator, as PlacedSystem)
                rng ^= rng << 13, rng ^= rng >> 17, rng ^= rng << 5;
                return float(rng >> 8) / 16777216.0f;
            });
        }
    if (static const bool tr = std::getenv("FUEL_RTC_TRACE") != nullptr; tr)
        for (const NodeFx& f : fx_) {
            static int n = 0;
            if (++n % 60) continue;
            size_t live = 0;
            for (const auto& st : f.inst.state) live += st.live.size();
            std::printf("RTC %s: node FX %08x t %.2f at (%.1f, %.1f, %.1f) scale %.2f, %zu particles%s\n", name_.c_str(), f.data, player_->time(),
                        f.inst.world[12], f.inst.world[13], f.inst.world[14], f.inst.scale, live, f.shown ? "" : " (hidden)");
        }
    fr.pauseGame = hold_.holding;
    fr.loading = hold_.holding;  // PK13: 0x539a50 = the player's +0x6e8
    fr.ended = player_->finished();
    return fr;
}

// ---- CutsceneFx (P67) -----------------------------------------------------------------------------

void CutsceneFx::load(const std::string& dataDir) {
    pp_ = fuel::rtc::readPpRows((fs::path(dataDir) / "CommonComponent/Param/pp001.tsc").string(), 64);
}

void CutsceneFx::frame(float dt, fuel::OrbitCamera& view, bool rtcCamera, fuel::MeshRenderer& r, fuel::DriveAudio* audio) {
    fader.update(dt);
    const auto q = fader.rgba();
    r.screenFade = simd_make_float4(q[0], q[1], q[2], q[3]);
    if (audio) audio->setFadeGain(fader.soundGain());
    const auto o = shake.update(dt);  // 0x4156e0: the position only
    view.eye += simd_make_float3(o[0], o[1], o[2]);
    endGrade.update(dt);
    auto pp = [&](size_t n) { return n < pp_.size() ? pp_[n] : 0.0f; };
    // 0x4b6050: Tint.rgb = PP39..41, Tint.a = PP43 x ramp, viewport +0x1dc = PP42 x ramp (written while the ramp moves
    // and kept after; the game camera rewrites +0x1dc every frame, so it only shows under an RTC camera).
    if (endGrade.ramp > 0) r.tintOverride = simd_make_float4(pp(39), pp(40), pp(41), pp(43) * endGrade.ramp);
    else r.tintOverride = simd_make_float4(0, 0, 0, -1);
    if (rtcCamera) r.speedBlur = 0, r.speedBlurAbsolute = rtcBlur();
}
