// A skinned object outside the rider; see skinned_model.hpp.
#include "skinned_model.hpp"

#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

namespace fuel {
namespace {

constexpr uint32_t kKeep = 0xFFFFFFFF, kEnd = 0xFFFFFFFE;
constexpr size_t kPaletteSlots = 21;

uint8_t packUnit(double v) { return uint8_t(std::lround(std::clamp((v * 0.5 + 0.5) * 255.0, 0.0, 255.0))); }

uint32_t linkName(const DpcFile& dpc, const DpcFile::Entry& e) {
    const std::vector<uint8_t> link = dpc.link(e);
    uint32_t v = 0;
    if (link.size() >= 4) std::memcpy(&v, link.data(), 4);
    return v;
}

}  // namespace

const DpcFile* SkinnedModel::fileOf(uint32_t id) const {
    for (const DpcFile* f : files_)
        if (f && f->find(id)) return f;
    return nullptr;
}

std::optional<std::vector<uint8_t>> SkinnedModel::body(uint32_t id) const {
    const DpcFile* f = fileOf(id);
    return f ? f->body(*f->find(id)) : std::nullopt;
}

uint32_t SkinnedModel::materialNamed(uint32_t nameCrc, const std::vector<uint32_t>& preferred) const {
    for (uint32_t id : preferred)
        if (const DpcFile* f = fileOf(id))
            if (const DpcFile::Entry* e = f->find(id); e && e->classCrc == cls::Material && linkName(*f, *e) == nameCrc) return id;
    if (materialByName_.empty())
        for (const DpcFile* f : files_)
            for (uint32_t id : f->ids(cls::Material)) materialByName_.insert({linkName(*f, *f->find(id)), id});
    const auto it = materialByName_.find(nameCrc);
    return it == materialByName_.end() ? (preferred.empty() ? 0 : preferred[0]) : it->second;
}

bool SkinnedModel::load(const std::vector<const DpcFile*>& files, uint32_t id, std::string* error) {
    files_ = files;
    try {
        const DpcFile* f = fileOf(id);
        if (!f) throw std::runtime_error("resource missing");
        uint32_t skinId = id;
        lod_ = Lod{};
        if (f->find(id)->classCrc == cls::Lod) {
            lod_ = parseLodBody(body(id).value());
            if (lod_.objects.empty()) throw std::runtime_error("Lod_Z without objects");
            skinId = lod_.objects[0];
        }
        const DpcFile* sf = fileOf(skinId);
        if (!sf || sf->find(skinId)->classCrc != cls::Skin) throw std::runtime_error("not a skin");
        const Skin skin = parseSkinBody(body(skinId).value());
        skel_ = parseSkelBody(body(parseObjectLink(sf->link(*sf->find(skinId))).dataId).value());
        boneByName_.clear();
        invBind_.clear();
        for (size_t i = 0; i < skel_.bones.size(); ++i) {
            boneByName_[skel_.bones[i].name] = i;
            anim::Mat4 bind;
            for (size_t k = 0; k < 16; ++k) bind[k] = skel_.bones[i].world[k];
            invBind_.push_back(anim::invertAffine(bind));
        }
        parts.clear();
        for (size_t m = 0; m < skin.meshes.size() && m < skin.sets.size(); ++m) {
            const DpcFile* mf = fileOf(skin.meshes[m]);
            if (!mf) continue;
            const DpcFile::Entry* me = mf->find(skin.meshes[m]);
            const MeshLink link = parseMeshLink(mf->link(*me));
            const Mesh mesh = parseMeshBody(mf->body(*me).value(), link.flags);
            const std::vector<SkinRecord>& recs = skin.sets[m];
            if (recs.size() != mesh.batches.size()) continue;
            Part part;
            part.meshId = skin.meshes[m];
            BakedMesh& out = part.mesh;
            std::map<uint32_t, uint32_t> materialIndex;
            std::array<int, kPaletteSlots> palette;
            palette.fill(-1);
            bool ok = true;
            for (size_t bi = 0; bi < mesh.batches.size() && ok; ++bi) {
                const Batch& b = mesh.batches[bi];
                const SkinRecord& rec = recs[bi];
                for (size_t k = 0; k < kPaletteSlots; ++k) {  // delta palette
                    if (rec[1 + k] == kEnd) break;
                    if (rec[1 + k] == kKeep) continue;
                    const auto bn = boneByName_.find(rec[1 + k]);
                    palette[k] = bn == boneByName_.end() ? -1 : int(bn->second);
                }
                const VertexBuffer& vb = mesh.vbs.at(b.vb);
                if (vb.stride != 60 && vb.stride != 48) {
                    ok = false;
                    break;
                }
                BakedMaterial bm;
                bm.materialId = materialNamed(rec[0], mesh.materials);
                auto mi = materialIndex.find(bm.materialId);
                if (mi == materialIndex.end()) {
                    mi = materialIndex.emplace(bm.materialId, uint32_t(out.materials.size())).first;
                    out.materials.push_back(bm);
                }
                const std::vector<uint32_t> tris = batchTriangles(mesh, b);
                const std::set<uint32_t> used(tris.begin(), tris.end());
                std::map<uint32_t, uint32_t> remap;
                for (uint32_t vi : used) {
                    remap[vi] = uint32_t(part.verts.size());
                    const uint8_t* v = &vb.data.at(size_t(vi) * vb.stride);
                    Part::V sv{};
                    std::memcpy(sv.pos, v, 12);
                    float idx[4] = {0, 0, 0, 0}, wt[4] = {1, 0, 0, 0};
                    if (vb.stride == 60) {
                        std::memcpy(idx, v + 28, 16);
                        std::memcpy(wt, v + 44, 16);
                    } else {
                        std::memcpy(idx, v + 28, 4);
                        std::memcpy(wt, v + 44, 4);
                    }
                    sv.nrm[0] = float(v[18] / 127.5 - 1.0), sv.nrm[1] = float(v[17] / 127.5 - 1.0), sv.nrm[2] = float(v[16] / 127.5 - 1.0);
                    sv.tan[0] = float(v[14] / 127.5 - 1.0), sv.tan[1] = float(v[13] / 127.5 - 1.0), sv.tan[2] = float(v[12] / 127.5 - 1.0);
                    for (int k = 0; k < 4; ++k) {
                        sv.w[k] = wt[k] > 0 ? wt[k] : 0;
                        sv.bone[k] = 0;
                        if (wt[k] <= 0) continue;
                        const int slot = int(idx[k]) / 6;  // 6 shader registers per palette slot
                        const int bone = slot >= 0 && size_t(slot) < kPaletteSlots ? palette[size_t(slot)] : -1;
                        if (bone < 0) sv.w[k] = 0;
                        else sv.bone[k] = bone;
                    }
                    part.verts.push_back(sv);
                    uint8_t o[BakedMesh::kStride] = {};
                    std::memcpy(o, sv.pos, 12);
                    std::memcpy(o + 12, v + 12, 8);  // bind tangent and normal as stored
                    std::memcpy(o + 20, v + 20, 8);
                    out.vertices.insert(out.vertices.end(), o, o + sizeof o);
                }
                BakedBatch bb;
                bb.material = mi->second;
                bb.firstIndex = uint32_t(out.indices.size());
                for (uint32_t i : tris) out.indices.push_back(remap.at(i));
                bb.indexCount = uint32_t(tris.size());
                out.batches.push_back(bb);
            }
            if (ok && !out.indices.empty()) parts.push_back(std::move(part));
        }
        if (parts.empty()) throw std::runtime_error("no skinnable mesh");
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

const anim::Clip* SkinnedModel::clipById(uint32_t animId) const {
    if (auto it = clips_.find(animId); it != clips_.end()) return it->second.get();
    auto c = std::make_unique<anim::Clip>();
    const auto b = body(animId);
    if (!b || !c->parse(*b)) c.reset();
    return (clips_[animId] = std::move(c)).get();
}

const anim::Clip* SkinnedModel::clipForSlot(uint32_t slot) const {
    for (const auto& [s, id] : lod_.animations)
        if (s == slot) return clipById(id);
    return nullptr;
}

std::vector<anim::Mat4> SkinnedModel::pose(const anim::Clip* clip, double t, const std::vector<std::pair<size_t, anim::Vec3>>& joints) const {
    std::vector<anim::Mat4> world(skel_.bones.size());
    for (size_t i = 0; i < skel_.bones.size(); ++i) {
        const Bone& b = skel_.bones[i];
        anim::Pose p;
        p.q = {b.localQuat[0], b.localQuat[1], b.localQuat[2], b.localQuat[3]};
        p.s = {b.localScale[0], b.localScale[1], b.localScale[2]};
        p.t = {b.localT[0], b.localT[1], b.localT[2]};
        if (clip) clip->sample(clip->track(b.name), t, p);
        for (const auto& [j, at] : joints)  // P96: 0x676480 (joint translation = point x inverse(parent frame))
            if (j == i) {
                if (b.parent >= 0) {
                    const anim::Mat4 inv = anim::invertAffine(world[size_t(b.parent)]);
                    p.t = {at[0] * inv[0] + at[1] * inv[4] + at[2] * inv[8] + inv[12], at[0] * inv[1] + at[1] * inv[5] + at[2] * inv[9] + inv[13],
                           at[0] * inv[2] + at[1] * inv[6] + at[2] * inv[10] + inv[14]};
                } else {
                    p.t = at;
                }
            }
        const anim::Mat4 local = anim::localMatrix(p.s, p.q, p.t);
        world[i] = b.parent >= 0 ? anim::mul(local, world[size_t(b.parent)]) : local;
    }
    return world;
}

void SkinnedModel::skin(const Part& part, const std::vector<anim::Mat4>& world, std::vector<uint8_t>& vertices) const {
    std::vector<anim::Mat4> M(world.size());
    for (size_t i = 0; i < world.size(); ++i) M[i] = anim::mul(invBind_[i], world[i]);
    vertices.resize(part.verts.size() * BakedMesh::kStride);
    for (size_t vi = 0; vi < part.verts.size(); ++vi) {
        const Part::V& v = part.verts[vi];
        double p[3] = {0, 0, 0}, n[3] = {0, 0, 0}, tg[3] = {0, 0, 0};
        for (int k = 0; k < 4; ++k) {
            if (v.w[k] <= 0) continue;
            const anim::Mat4& m = M[size_t(v.bone[k])];
            const double w = v.w[k];
            for (size_t c = 0; c < 3; ++c) {
                p[c] += w * (v.pos[0] * m[c] + v.pos[1] * m[4 + c] + v.pos[2] * m[8 + c] + m[12 + c]);
                n[c] += w * (v.nrm[0] * m[c] + v.nrm[1] * m[4 + c] + v.nrm[2] * m[8 + c]);
                tg[c] += w * (v.tan[0] * m[c] + v.tan[1] * m[4 + c] + v.tan[2] * m[8 + c]);
            }
        }
        auto norm = [](double* a) {
            const double l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if (l > 0)
                for (int c = 0; c < 3; ++c) a[c] /= l;
        };
        norm(n);
        norm(tg);
        uint8_t* o = &vertices[vi * BakedMesh::kStride];
        const uint8_t* src = &part.mesh.vertices[vi * BakedMesh::kStride];
        const float pf[3] = {float(p[0]), float(p[1]), float(p[2])};
        std::memcpy(o, pf, 12);
        o[12] = packUnit(tg[2]), o[13] = packUnit(tg[1]), o[14] = packUnit(tg[0]), o[15] = src[15];
        o[16] = packUnit(n[2]), o[17] = packUnit(n[1]), o[18] = packUnit(n[0]), o[19] = 255;
        std::memcpy(o + 20, src + 20, 16);
    }
}

}  // namespace fuel
