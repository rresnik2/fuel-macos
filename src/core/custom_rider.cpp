// PHYSIQUE (port deviation): the player's custom rider (custom_rider.hpp).
#include "custom_rider.hpp"

#include "resources.hpp"

#include <mach-o/dyld.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>

namespace fs = std::filesystem;

namespace fuel {

namespace {

struct In {
    std::vector<uint8_t> b;
    size_t p = 0;
    void need(size_t n) {
        if (p + n > b.size()) throw std::runtime_error("rider.fra: truncated");
    }
    uint32_t u32() {
        need(4);
        uint32_t v;
        std::memcpy(&v, &b[p], 4);
        p += 4;
        return v;
    }
    float f32() {
        need(4);
        float v;
        std::memcpy(&v, &b[p], 4);
        p += 4;
        return v;
    }
    std::string str() {
        need(2);
        uint16_t n;
        std::memcpy(&n, &b[p], 2);
        p += 2;
        need(n);
        std::string s(reinterpret_cast<const char*>(&b[p]), n);
        p += n;
        return s;
    }
    void raw(void* dst, size_t n) {
        need(n);
        std::memcpy(dst, &b[p], n);
        p += n;
    }
};

uint8_t packUnit(double v) { return uint8_t(std::lround(std::clamp((v * 0.5 + 0.5) * 255.0, 0.0, 255.0))); }

}  // namespace

std::string CustomRider::findDir() {
    if (const char* e = std::getenv("FUEL_CUSTOM_RIDER"); e && *e) return std::string(e) == "off" ? std::string() : std::string(e);
    char path[4096];
    uint32_t size = sizeof path;
    if (_NSGetExecutablePath(path, &size) == 0) {
        std::error_code ec;
        const fs::path exe = fs::weakly_canonical(fs::path(path), ec);
        const fs::path dir = exe.parent_path().parent_path() / "Resources" / "rider";
        if (fs::exists(dir / "rider.fra", ec)) return dir.string();
    }
    return {};
}

const CustomRider* CustomRider::shared() {
    static std::once_flag once;
    static std::unique_ptr<CustomRider> inst;
    std::call_once(once, [] {
        const std::string dir = findDir();
        if (dir.empty()) return;
        std::string err;
        inst = load(dir, &err);
        if (!inst) std::fprintf(stderr, "fuelview: custom rider (%s): %s; FUEL's rider is used\n", dir.c_str(), err.c_str());
    });
    return inst.get();
}

std::unique_ptr<CustomRider> CustomRider::load(const std::string& dir, std::string* error) {
    try {
        std::ifstream f(fs::path(dir) / "rider.fra", std::ios::binary);
        if (!f) throw std::runtime_error("no rider.fra");
        In in;
        in.b.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        char magic[8];
        in.raw(magic, 8);
        if (std::memcmp(magic, "FUELRDR1", 8) != 0) throw std::runtime_error("not a rider.fra");
        const uint32_t version = in.u32();
        if (version != 1 && version != 2) throw std::runtime_error("unknown rider.fra version");
        auto r = std::make_unique<CustomRider>();
        r->dir_ = dir;
        r->version_ = int(version);
        const uint32_t nb = in.u32();
        if (nb > 256) throw std::runtime_error("bone table too large");
        for (uint32_t k = 0; k < nb; ++k) r->bones_.push_back(in.u32());
        if (version >= 2) {
            const uint32_t nh = in.u32();
            if (nh > RiderJiggle::kBones) throw std::runtime_error("too many helper bones");
            for (uint32_t k = 0; k < nh; ++k) {
                Helper h;
                h.name = in.u32(), h.base = in.u32();
                for (float& c : h.centre) c = in.f32();
                h.radius = in.f32();
                r->helpers_.push_back(h);
            }
        }
        const uint32_t nm = in.u32();
        if (nm > 4096) throw std::runtime_error("too many materials");
        for (uint32_t k = 0; k < nm; ++k) {
            CustomMaterial m;
            m.flags = in.u32();
            for (float& v : m.spec) v = in.f32();
            m.fuelName = in.u32();
            m.fuelMaterial = in.u32();
            std::string* tex[3] = {&m.diffuse, &m.normal, &m.specular};
            for (std::string* t : tex) {
                const std::string s = in.str();
                *t = s.empty() ? s : (fs::path(dir) / s).string();
            }
            m.name = in.str();
            r->materials_.push_back(std::move(m));
        }
        // rider4: skin tone variants of the Skin materials (<name>_d_t<k>.dds next to rider.fra; optional)
        r->toneIndex_.assign(r->materials_.size(), {});
        for (auto& ti : r->toneIndex_) ti.fill(-1);
        r->toneMaterials_.reserve(r->materials_.size() * (kSkinTones - 1));  // no reallocation: prepare() keeps pointers
        for (size_t k = 0; k < r->materials_.size(); ++k) {
            const CustomMaterial& m = r->materials_[k];
            if (!(m.flags & CustomMaterial::Skin) || m.diffuse.empty()) continue;
            for (int tone = 2; tone <= kSkinTones; ++tone) {
                const fs::path p = fs::path(dir) / (m.name + "_d_t" + std::to_string(tone) + ".dds");
                std::error_code ec;
                if (!fs::exists(p, ec)) continue;
                CustomMaterial v = m;
                v.diffuse = p.string();
                r->toneIndex_[k][size_t(tone - 2)] = int(r->toneMaterials_.size());
                r->toneMaterials_.push_back(std::move(v));
                ++r->toneVariants_;
            }
        }
        const uint32_t nmesh = in.u32();
        if (nmesh > 4096) throw std::runtime_error("too many meshes");
        for (uint32_t k = 0; k < nmesh; ++k) {
            CustomMesh m;
            m.name = in.str();
            m.lod = int(in.u32());
            m.garment = int(in.u32()) - 1;
            const uint32_t nv = in.u32();
            if (nv > 4000000) throw std::runtime_error("mesh too large");
            m.verts.resize(nv);
            for (CustomMesh::V& v : m.verts) {
                in.raw(v.pos, 12), in.raw(v.nrm, 12), in.raw(v.tan, 16), in.raw(v.uv, 8), in.raw(v.bone, 4), in.raw(v.w, 16);
                if (version >= 2) {
                    in.raw(&v.jig, 1), v.jw = in.f32();
                    if (v.jig > r->helpers_.size()) throw std::runtime_error("vertex helper bone out of range");
                }
                for (int q = 0; q < 4; ++q)
                    if (v.w[q] > 0 && v.bone[q] >= nb) throw std::runtime_error("vertex bone out of range");
            }
            const uint32_t nbat = in.u32();
            for (uint32_t b = 0; b < nbat; ++b) {
                CustomMesh::Batch bt;
                bt.material = in.u32();
                if (bt.material >= nm) throw std::runtime_error("batch material out of range");
                const uint32_t ni = in.u32();
                if (ni % 3 != 0) throw std::runtime_error("index count not a multiple of 3");
                bt.indices.resize(ni);
                in.raw(bt.indices.data(), size_t(ni) * 4);
                for (uint32_t i : bt.indices)
                    if (i >= nv) throw std::runtime_error("index out of range");
                if (in.u32() != 0) {
                    bt.mask.resize(ni / 3);
                    in.raw(bt.mask.data(), bt.mask.size() * 4);
                }
                m.batches.push_back(std::move(bt));
            }
            r->lods_ = std::max(r->lods_, m.lod + 1);
            r->meshes_.push_back(std::move(m));
        }
        if (!r->mesh("body", 0)) throw std::runtime_error("no body mesh");
        for (const CustomMaterial& m : r->materials_)
            for (const std::string* t : {&m.diffuse, &m.normal, &m.specular})
                if (!t->empty() && !fs::exists(*t)) throw std::runtime_error("missing texture " + *t);
        return r;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return nullptr;
    }
}

const CustomMaterial& CustomRider::toned(uint32_t m, int skin) const {
    if (m < toneIndex_.size() && skin >= 2 && skin <= kSkinTones)
        if (const int i = toneIndex_[m][size_t(skin - 2)]; i >= 0) return toneMaterials_[size_t(i)];
    return materials_[m];
}

const CustomMesh* CustomRider::mesh(const std::string& name, int lod) const {
    const CustomMesh* best = nullptr;
    for (const CustomMesh& m : meshes_)  // the level asked for, else the nearest finer one
        if (m.name == name && m.lod <= lod && (!best || m.lod > best->lod)) best = &m;
    return best;
}

Rider::SkinTemplate CustomRider::prepare(const Rider& rider, const RiderOutfit& o, const Rider::Parts& parts, int lod) const {
    std::vector<const CustomMesh*> pieces;
    auto add = [&](const std::string& n) {
        if (const CustomMesh* m = mesh(n, lod)) pieces.push_back(m);
    };
    if (!o.helmet) add("eyes"), add("hair");
    else add("helmet_head");  // FUEL's own head piece (the helmets are made around it); the body's head is masked (Helmet)
    if (o.outfit == 1) {
        add("gear_chest" + std::to_string(std::clamp(parts.chest, 1, 5)));
        add("gear_legs" + std::to_string(std::clamp(parts.legs, 1, 3)));
    } else if (o.outfit == 2) {
        add("tank"), add("cutoffs"), add("boots"), add("gloves");
    } else {
        add("shorts"), add("boots"), add("gloves");
    }
    uint32_t covered = o.helmet ? 1u << Helmet : 0u;  // FUEL's helmets cover the face (as FUEL's own head has none)
    for (const CustomMesh* m : pieces)
        if (m->garment >= 0) covered |= 1u << m->garment;
    const CustomMesh* body = mesh("body", lod);
    pieces.insert(pieces.begin(), body);

    // FUEL's helmet (the garage's head part) on the new head: its own skin template, merged below.
    Rider::SkinTemplate st;
    if (o.helmet) st = rider.prepareSkin(parts, 0, 1u);
    BakedMesh& out = st.mesh;

    std::vector<int> boneMap(bones_.size(), -1);
    for (size_t k = 0; k < bones_.size(); ++k) boneMap[k] = rider.boneByCrc(bones_[k]);
    for (size_t k = 0; k < helpers_.size() && k < size_t(RiderJiggle::kBones); ++k) {  // the glute helper bones on Hips
        JiggleBone& jb = st.jiggleBones[k];
        jb.base = rider.boneByCrc(helpers_[k].base);
        for (int c = 0; c < 3; ++c) jb.centre[size_t(c)] = helpers_[k].centre[c];
        jb.radius = helpers_[k].radius;
        st.hasJiggle = st.hasJiggle || jb.base >= 0;
    }
    std::map<uint32_t, uint32_t> matIndex;  // custom material -> BakedMesh material
    auto material = [&](uint32_t m) {
        if (auto it = matIndex.find(m); it != matIndex.end()) return it->second;
        const CustomMaterial& cm = toned(m, parts.skin);  // rider4: the garage's skin tone
        BakedMaterial bm;
        if (cm.flags & CustomMaterial::Fuel) {
            bm = rider.partMaterial(cm.fuelName, cm.fuelMaterial ? std::vector<uint32_t>{cm.fuelMaterial} : std::vector<uint32_t>{}, parts);
        } else {
            bm.custom = &cm;
        }
        const uint32_t idx = uint32_t(out.materials.size());
        out.materials.push_back(bm);
        return matIndex[m] = idx;
    };
    for (const CustomMesh* m : pieces) {
        const bool isBody = m == body;
        for (const CustomMesh::Batch& b : m->batches) {
            std::vector<uint32_t> tris;
            tris.reserve(b.indices.size());
            for (size_t t = 0; t + 2 < b.indices.size(); t += 3) {
                if (isBody && !b.mask.empty() && (b.mask[t / 3] & covered) != 0) continue;
                tris.insert(tris.end(), {b.indices[t], b.indices[t + 1], b.indices[t + 2]});
            }
            if (tris.empty()) continue;
            std::map<uint32_t, uint32_t> remap;
            for (uint32_t vi : tris) {
                if (remap.count(vi)) continue;
                remap[vi] = uint32_t(st.verts.size());
                const CustomMesh::V& v = m->verts[vi];
                Rider::SkinTemplate::V sv{};
                std::memcpy(sv.pos, v.pos, 12);
                std::memcpy(sv.nrm, v.nrm, 12);
                std::memcpy(sv.tan, v.tan, 12);
                for (int q = 0; q < 4; ++q) {
                    sv.w[q] = v.w[q] > 0 ? v.w[q] : 0;
                    sv.bone[q] = 0;
                    if (sv.w[q] <= 0) continue;
                    const int bi = boneMap[v.bone[q]];
                    if (bi < 0) throw std::runtime_error("custom rider: a bone the skeleton lacks");
                    sv.bone[q] = bi;
                }
                if (v.jig > 0 && v.jw > 0 && size_t(v.jig) <= helpers_.size()) sv.jig = v.jig, sv.jw = v.jw;
                st.verts.push_back(sv);
                uint8_t o8[BakedMesh::kStride] = {};
                std::memcpy(o8, v.pos, 12);
                o8[12] = packUnit(v.tan[2]), o8[13] = packUnit(v.tan[1]), o8[14] = packUnit(v.tan[0]);
                o8[15] = v.tan[3] < 0 ? 0 : 255;  // handedness (vsMain: w > 0.5 -> +1)
                o8[16] = packUnit(v.nrm[2]), o8[17] = packUnit(v.nrm[1]), o8[18] = packUnit(v.nrm[0]), o8[19] = 255;
                std::memcpy(o8 + 20, v.uv, 8);
                out.vertices.insert(out.vertices.end(), o8, o8 + sizeof o8);
            }
            BakedBatch bb;
            bb.material = material(b.material);
            bb.firstIndex = uint32_t(out.indices.size());
            for (uint32_t i : tris) out.indices.push_back(remap.at(i));
            bb.indexCount = uint32_t(tris.size());
            out.batches.push_back(bb);
        }
    }
    return st;
}

namespace {
bool rayHitsTri(const float o[3], const float d[3], const float* a, const float* b, const float* c, double tmin, double tmax) {
    double e1[3], e2[3], h[3], sv[3], q[3];
    for (int i = 0; i < 3; ++i) e1[i] = b[i] - a[i], e2[i] = c[i] - a[i];
    h[0] = d[1] * e2[2] - d[2] * e2[1], h[1] = d[2] * e2[0] - d[0] * e2[2], h[2] = d[0] * e2[1] - d[1] * e2[0];
    const double det = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2];
    if (std::fabs(det) < 1e-12) return false;
    const double inv = 1.0 / det;
    for (int i = 0; i < 3; ++i) sv[i] = o[i] - a[i];
    const double u = (sv[0] * h[0] + sv[1] * h[1] + sv[2] * h[2]) * inv;
    if (u < 0 || u > 1) return false;
    q[0] = sv[1] * e1[2] - sv[2] * e1[1], q[1] = sv[2] * e1[0] - sv[0] * e1[2], q[2] = sv[0] * e1[1] - sv[1] * e1[0];
    const double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv;
    if (v < 0 || u + v > 1) return false;
    const double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
    return t >= tmin && t <= tmax;
}
}  // namespace

int CustomRider::coverageCheck(int* configs, std::vector<std::string>* problems) const {
    int bad = 0, n = 0;
    auto problem = [&](const std::string& s) {
        ++bad;
        if (problems) problems->push_back(s);
    };
    const CustomMesh* body = mesh("body", 0);
    if (!body) {
        problem("no body mesh");
        return bad;
    }
    struct Cfg { int outfit, chest, legs; bool helmet; };
    std::vector<Cfg> cfgs;
    for (int h = 0; h < 2; ++h) {
        for (int c = 1; c <= 5; ++c)
            for (int l = 1; l <= 3; ++l) cfgs.push_back({1, c, l, h != 0});
        cfgs.push_back({2, 1, 1, h != 0});
        cfgs.push_back({3, 1, 1, h != 0});
    }
    for (const Cfg& cf : cfgs) {
        ++n;
        char tag[96];
        std::snprintf(tag, sizeof tag, "outfit %d chest %d legs %d helmet %d", cf.outfit, cf.chest, cf.legs, int(cf.helmet));
        std::vector<std::string> names;
        if (!cf.helmet) names.push_back("eyes");
        else names.push_back("helmet_head");
        if (cf.outfit == 1) names.push_back("gear_chest" + std::to_string(cf.chest)), names.push_back("gear_legs" + std::to_string(cf.legs));
        else if (cf.outfit == 2) names.insert(names.end(), {"tank", "cutoffs", "boots", "gloves"});
        else names.insert(names.end(), {"shorts", "boots", "gloves"});
        std::vector<const CustomMesh*> worn;
        uint32_t covered = 0;
        for (const std::string& nm : names) {
            const CustomMesh* m = mesh(nm, 0);
            if (!m || m->verts.empty() || m->batches.empty()) {
                problem(std::string(tag) + ": piece '" + nm + "' missing or empty");
                continue;
            }
            size_t tris = 0;
            for (const auto& b : m->batches) tris += b.indices.size() / 3;
            if (tris == 0) problem(std::string(tag) + ": piece '" + nm + "' has no triangles");
            worn.push_back(m);
            if (m->garment >= 0) covered |= 1u << m->garment;
        }
        // every culled body triangle (Helmet aside: FUEL's helmets are their own parts) has a worn piece over it
        size_t culled = 0, uncovered = 0;
        for (const auto& b : body->batches) {
            if (b.mask.empty()) continue;
            for (size_t t = 0; t < b.mask.size(); ++t) {
                const uint32_t bits = b.mask[t] & covered & ~(1u << Helmet);
                if (!bits || (b.mask[t] & covered & (1u << Helmet))) continue;
                ++culled;
                const auto& A = body->verts[b.indices[t * 3]];
                const auto& B = body->verts[b.indices[t * 3 + 1]];
                const auto& C = body->verts[b.indices[t * 3 + 2]];
                float o[3], nrm[3];
                for (int i = 0; i < 3; ++i) o[i] = (A.pos[i] + B.pos[i] + C.pos[i]) / 3, nrm[i] = A.nrm[i] + B.nrm[i] + C.nrm[i];
                const float len = std::sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
                for (float& v : nrm) v /= len > 0 ? len : 1;
                bool hit = false;
                for (const CustomMesh* m : worn) {
                    if (m->garment < 0 || !(bits & (1u << m->garment))) continue;
                    for (const auto& gb : m->batches) {
                        for (size_t k = 0; k + 2 < gb.indices.size() && !hit; k += 3)
                            hit = rayHitsTri(o, nrm, m->verts[gb.indices[k]].pos, m->verts[gb.indices[k + 1]].pos, m->verts[gb.indices[k + 2]].pos, -0.026, 0.08);
                        if (hit) break;
                    }
                    if (hit) break;
                }
                if (!hit) ++uncovered;
            }
        }
        if (uncovered > 0) problem(std::string(tag) + ": " + std::to_string(uncovered) + " of " + std::to_string(culled) + " culled body triangles have no garment over them");
    }
    if (configs) *configs = n;
    return bad;
}

const char* riderOutfitName(const RiderOutfit& o) {
    switch (o.outfit) {
        case 0: return "FUEL RIDER";
        case 1: return o.helmet ? "RIDING GEAR" : "RIDING GEAR, NO HELMET";
        case 2: return o.helmet ? "TANK TOP & CUTOFFS, HELMET" : "TANK TOP & CUTOFFS";
        default: return o.helmet ? "SHIRTLESS & SHORTS, HELMET" : "SHIRTLESS & SHORTS";
    }
}

}  // namespace fuel
