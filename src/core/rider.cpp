#include "rider.hpp"

#include "reader.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <sstream>
#include <tuple>

namespace fuel {

namespace {

constexpr uint32_t kPilotLod = 0x9A9C3E61;  // Lod_Z of IT_PILOT_SKIN
constexpr uint32_t kKeep = 0xFFFFFFFF, kEnd = 0xFFFFFFFE;
constexpr size_t kPaletteSlots = 21;

const Mat4 kIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// Row-vector product: applies `a` first, then `b`.
Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            r[size_t(i * 4 + j)] = s;
        }
    return r;
}

// Inverse of an affine matrix (general 3x3 plus translation).
Mat4 invertAffine(const Mat4& m) {
    double a[3][3], inv[3][3];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) a[r][c] = m[size_t(r * 4 + c)];
    const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                       a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (std::fabs(det) < 1e-12) throw std::runtime_error("singular bone matrix");
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            // inverse[r][c] = cofactor of a[c][r] over the determinant
            const int r1 = (c + 1) % 3, r2 = (c + 2) % 3, c1 = (r + 1) % 3, c2 = (r + 2) % 3;
            inv[r][c] = (a[r1][c1] * a[r2][c2] - a[r1][c2] * a[r2][c1]) / det;
        }
    Mat4 o = kIdentity;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) o[size_t(r * 4 + c)] = inv[r][c];
    const double t[3] = {m[12], m[13], m[14]};
    for (int c = 0; c < 3; ++c) o[size_t(12 + c)] = -(t[0] * inv[0][c] + t[1] * inv[1][c] + t[2] * inv[2][c]);
    return o;
}

// D3DXMatrixRotationQuaternion (row-vector convention), q = xyzw.
Mat4 rotation(std::array<double, 4> q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0)
        for (double& c : q) c /= n;
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0, 2 * (x * y - z * w), 1 - 2 * (x * x + z * z),
            2 * (y * z + x * w), 0, 2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0, 0, 0, 0, 1};
}

// Local bone matrix: Scale * Rotation * Translation.
Mat4 localMatrix(const std::array<double, 3>& scale, const std::array<double, 4>& q, const std::array<double, 3>& t) {
    Mat4 m = rotation(q);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m[size_t(r * 4 + c)] *= scale[size_t(r)];
    m[12] = t[0], m[13] = t[1], m[14] = t[2];
    return m;
}

std::array<double, 4> slerp(std::array<double, 4> a, std::array<double, 4> b, double f) {
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (d < 0) {
        for (double& c : b) c = -c;
        d = -d;
    }
    std::array<double, 4> o;
    if (d > 0.9995) {
        for (size_t i = 0; i < 4; ++i) o[i] = a[i] + (b[i] - a[i]) * f;
        return o;
    }
    const double th = std::acos(std::min(1.0, d)), s = std::sin(th);
    const double wa = std::sin((1 - f) * th) / s, wb = std::sin(f * th) / s;
    for (size_t i = 0; i < 4; ++i) o[i] = wa * a[i] + wb * b[i];
    return o;
}

// Value of a keyframe track at time `t`: linear, or spherical for rotations.
template <size_t N>
void sampleKeys(const std::vector<std::pair<float, std::array<float, N>>>& keys, double t, std::array<double, N>* out) {
    if (keys.empty()) return;
    auto at = [&](size_t i) {
        std::array<double, N> v;
        for (size_t k = 0; k < N; ++k) v[k] = keys[i].second[k];
        return v;
    };
    size_t i = 1;
    while (i < keys.size() && t > keys[i].first) ++i;
    if (t <= keys[0].first) {
        *out = at(0);
    } else if (i >= keys.size()) {
        *out = at(keys.size() - 1);
    } else {
        const double t0 = keys[i - 1].first, t1 = keys[i].first;
        const double f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0;
        const auto a = at(i - 1), b = at(i);
        if constexpr (N == 4) {
            *out = slerp(a, b, f);
        } else {
            for (size_t k = 0; k < N; ++k) (*out)[k] = a[k] + (b[k] - a[k]) * f;
        }
    }
}

uint32_t linkName(const DpcFile& dpc, const DpcFile::Entry& e) {
    const std::vector<uint8_t> link = dpc.link(e);
    uint32_t v = 0;
    if (link.size() >= 4) std::memcpy(&v, link.data(), 4);
    return v;
}

uint8_t packUnit(double v) { return uint8_t(std::lround(std::clamp((v * 0.5 + 0.5) * 255.0, 0.0, 255.0))); }

}  // namespace

std::array<float, 3> BakedMesh::position(uint32_t i) const {
    std::array<float, 3> p;
    std::memcpy(p.data(), &vertices[size_t(i) * kStride], 12);
    return p;
}

std::string vehicleArchetype(const std::string& tscText, const std::string& variant) {
    std::istringstream in(tscText);
    std::string line;
    const std::string want = "MESH_ID_" + variant;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::vector<std::string> t;
        for (std::string w; ls >> w;) t.push_back(w);
        if (t.size() >= 9 && t[0] == "AddVehicleInfo" && t[1] == want) return t[8];
    }
    return {};
}

Rider::Rider(const DpcFile& pilot) : dpc_(pilot) {
    const DpcFile::Entry* lodE = pilot.find(kPilotLod);
    if (lodE == nullptr) throw std::runtime_error("PILOT.DPC has no rider Lod_Z");
    const auto lodBody = pilot.body(*lodE);
    if (!lodBody) throw std::runtime_error("rider Lod_Z has no body");
    const Lod lod = parseLodBody(*lodBody);
    if (lod.objects.empty()) throw std::runtime_error("rider Lod_Z has no skin");
    for (const auto& [slot, id] : lod.animations) animSlots_[slot] = id;
    const DpcFile::Entry* skinE = pilot.find(lod.objects[0]);
    if (skinE == nullptr) throw std::runtime_error("rider Skin_Z missing");
    skin_ = parseSkinBody(pilot.body(*skinE).value());
    const DpcFile::Entry* skelE = pilot.find(parseObjectLink(pilot.link(*skinE)).dataId);
    if (skelE == nullptr) throw std::runtime_error("rider Skel_Z missing");
    skel_ = parseSkelBody(pilot.body(*skelE).value());
    for (size_t i = 0; i < skel_.bones.size(); ++i) boneByName_[skel_.bones[i].name] = i;
    for (uint32_t id : pilot.ids(cls::Material)) materialByName_.insert({linkName(pilot, *pilot.find(id)), id});
    for (uint32_t id : pilot.ids(cls::MaterialAnim)) {
        const DpcFile::Entry* e = pilot.find(id);
        const auto body = pilot.body(*e);
        if (!body || body->size() < 0x40) continue;
        uint32_t mat;
        std::memcpy(&mat, body->data() + 0x3C, 4);  // the livery's Material_Z
        liveryByName_.insert({linkName(pilot, *e), mat});
    }
}

const Rider::Animation& Rider::animation(int slot) const {
    const auto s = animSlots_.find(uint32_t(slot));
    if (s == animSlots_.end()) throw std::runtime_error("no rider animation in slot " + std::to_string(slot));
    if (auto it = anims_.find(s->second); it != anims_.end()) return it->second;
    const DpcFile::Entry* e = dpc_.find(s->second);
    if (e == nullptr) throw std::runtime_error("rider animation missing");
    const std::vector<uint8_t> body = dpc_.body(*e).value();
    // Animation_Z body: duration, three header values, then the arrays: A rotation keys (12 B),
    // B (40 B), C scale keys (24 B), D translation keys (24 B), E, F-L, M bone tracks (28 B), N, O, P.
    ByteReader r(body);
    Animation a;
    a.duration = r.f32();
    r.u32();
    r.u16();
    r.u16();
    const uint32_t nrot = r.count();
    const uint8_t* rotData = r.raw(size_t(nrot) * 12);
    r.skipArray(0x28);
    r.u16();
    const uint32_t nsca = r.count();
    const uint8_t* scaData = r.raw(size_t(nsca) * 0x18);
    r.u16();
    const uint32_t ntra = r.count();
    const uint8_t* traData = r.raw(size_t(ntra) * 0x18);
    for (uint32_t i = 0, n = r.count(); i < n; ++i) {
        r.u32();
        r.skipArray(20);
    }
    for (size_t sz : {size_t(12), size_t(12), size_t(0x18), size_t(0x18), size_t(12), size_t(12), size_t(12)}) {
        r.u16();
        r.skipArray(sz);
    }
    const uint32_t ntracks = r.count();
    const uint8_t* trackData = r.raw(size_t(ntracks) * 0x1C);
    r.skipArray(0x1C);
    r.skipArray(12);
    r.skipArray(12);
    r.expectEnd();

    auto vec = [](const uint8_t* p, size_t i) {
        float t;
        int16_t v[3];
        std::memcpy(&t, p + i * 0x18, 4);
        std::memcpy(v, p + i * 0x18 + 4, 6);
        return std::make_pair(t, std::array<float, 3>{v[0] / 4096.0f, v[1] / 4096.0f, v[2] / 4096.0f});
    };
    for (uint32_t i = 0; i < ntracks; ++i) {
        const uint8_t* p = trackData + size_t(i) * 0x1C;
        uint32_t crc;
        uint16_t u[12];
        std::memcpy(&crc, p, 4);
        std::memcpy(u, p + 4, 24);
        Track tr;
        if (size_t(u[2]) + u[3] > ntra || size_t(u[4]) + u[5] > nrot || size_t(u[6]) + u[7] > nsca)
            throw std::runtime_error("animation track outside its key arrays");
        for (size_t k = u[2]; k < size_t(u[2]) + u[3]; ++k) tr.translation.push_back(vec(traData, k));
        for (size_t k = u[6]; k < size_t(u[6]) + u[7]; ++k) tr.scale.push_back(vec(scaData, k));
        for (size_t k = u[4]; k < size_t(u[4]) + u[5]; ++k) {
            float t;
            int16_t q[4];
            std::memcpy(&t, rotData + k * 12, 4);
            std::memcpy(q, rotData + k * 12 + 4, 8);
            tr.rotation.push_back({t, {q[0] / 2000.0f, q[1] / 2000.0f, q[2] / 2000.0f, q[3] / 2000.0f}});
        }
        a.tracks[crc] = std::move(tr);
    }
    return anims_.emplace(s->second, std::move(a)).first->second;
}

std::vector<Mat4> Rider::poseWorlds(const Animation* a, double time) const {
    std::vector<Mat4> world(skel_.bones.size());
    for (size_t i = 0; i < skel_.bones.size(); ++i) {
        const Bone& b = skel_.bones[i];
        std::array<double, 4> q = {b.localQuat[0], b.localQuat[1], b.localQuat[2], b.localQuat[3]};
        std::array<double, 3> t = {b.localT[0], b.localT[1], b.localT[2]}, s = {b.localScale[0], b.localScale[1], b.localScale[2]};
        if (a != nullptr) {
            if (auto it = a->tracks.find(b.name); it != a->tracks.end()) {
                sampleKeys(it->second.rotation, time, &q);
                sampleKeys(it->second.translation, time, &t);
                sampleKeys(it->second.scale, time, &s);
            }
        }
        const Mat4 local = localMatrix(s, q, t);
        world[i] = b.parent >= 0 ? mul(local, world[size_t(b.parent)]) : local;
    }
    return world;
}

std::vector<Mat4> Rider::pose(int slot, double time) const { return poseWorlds(&animation(slot), time); }

int Rider::ridingSlot(const std::string& archetype) const {
    static const std::map<std::string, std::pair<int, int>> blocks = {
        {"BUGGY", {18, 26}},           {"BUGGY_ROAD", {28, 36}},   {"MBIKE", {38, 80}},       {"MBIKE_SPORT", {81, 123}},
        {"MBIKE_CHOPPER", {124, 166}}, {"MBIKE_TRAIL", {167, 209}}, {"QUAD", {210, 250}},     {"QUAD_TRAIL", {251, 291}},
        {"TRUCK", {292, 302}},         {"MUSCLE", {306, 316}},     {"SUV", {320, 330}}};
    const auto blk = blocks.find(archetype);
    if (blk == blocks.end()) return -1;
    auto bone = [&](const char* n) { return boneByName_.at(asoboCrc(n)); };
    const size_t hips = bone("Hips"), lh = bone("LeftHand"), rh = bone("RightHand"), head = bone("Head"), lf = bone("LeftFoot"),
                 rf = bone("RightFoot");
    // The neutral pose of the block: hips centred between the hands, hands level and apart,
    // head forward of the hips, feet together and off the ground; ties go to the longest loop.
    int best = -1;
    std::tuple<bool, float, int> bestKey;
    for (int slot = blk->second.first; slot <= blk->second.second; ++slot) {
        if (!animSlots_.count(uint32_t(slot))) continue;
        const Animation& an = animation(slot);
        const std::vector<Mat4> w = poseWorlds(&an, 0.0);
        auto p = [&](size_t i) { return std::array<double, 3>{w[i][12], w[i][13], w[i][14]}; };
        const auto h = p(hips), l = p(lh), r = p(rh), hd = p(head), fl = p(lf), fr = p(rf);
        const double cx = 0.5 * (l[0] + r[0]);
        const double sym = std::fabs(l[1] - r[1]) + std::fabs(l[2] - r[2]);
        const bool feet = std::fabs(fl[1] - fr[1]) < 0.05 && std::fabs(fl[2] - fr[2]) < 0.08 && std::min(fl[1], fr[1]) > 0.1;
        const bool ok = std::fabs(h[0] - cx) < 0.05 && sym < 0.05 && hd[2] > h[2] && l[0] - r[0] > 0.15 && feet;
        const std::tuple<bool, float, int> key{ok, an.duration, -slot};
        if (best < 0 || key > bestKey) best = slot, bestKey = key;
    }
    return best;
}

uint32_t Rider::materialNamed(uint32_t nameCrc, const std::vector<uint32_t>& preferred) const {
    for (uint32_t id : preferred) {
        const DpcFile::Entry* e = dpc_.find(id);
        if (e != nullptr && e->classCrc == cls::Material && linkName(dpc_, *e) == nameCrc) return id;
    }
    const auto it = materialByName_.find(nameCrc);
    return it == materialByName_.end() ? 0 : it->second;
}

uint32_t Rider::liveryMaterial(const std::string& name) const {
    const auto it = liveryByName_.find(asoboCrc(name));
    return it == liveryByName_.end() ? 0 : it->second;
}

BakedMesh Rider::bake(int slot, double time, int head, int chest, int legs, int livery) const {
    const std::vector<Mat4> world = pose(slot, time);
    // skin matrix per bone: inverse(bind world) * pose world
    std::vector<Mat4> skinM(world.size());
    for (size_t i = 0; i < world.size(); ++i) {
        Mat4 bind;
        for (size_t k = 0; k < 16; ++k) bind[k] = skel_.bones[i].world[k];
        skinM[i] = mul(invertAffine(bind), world[i]);
    }

    char buf[48];
    const std::string partNames[3] = {"Head" + std::to_string(head), "Chest" + std::to_string(chest), "Legs" + std::to_string(legs)};
    std::map<uint32_t, std::string> liveryOf;  // part material name CRC -> livery material name
    auto addLivery = [&](const std::string& part) {
        std::snprintf(buf, sizeof buf, "LIVERY_%02d_%s", livery, part.c_str());
        liveryOf[asoboCrc("LIVERY_XX_" + part)] = buf;
    };
    addLivery("HEAD" + std::to_string(head));
    addLivery("CHEST" + std::to_string(chest));
    addLivery("LEGS" + std::to_string(legs));
    std::snprintf(buf, sizeof buf, "LIVERY_%02d_BODY1", livery);
    liveryOf[asoboCrc("LIVERY_XX_BODY")] = buf;

    BakedMesh out;
    std::map<std::tuple<uint32_t, uint32_t, uint32_t>, uint32_t> materialIndex;
    for (const std::string& part : partNames) {
        const uint32_t nameCrc = asoboCrc(part);
        size_t m = 0;
        for (; m < skin_.meshes.size(); ++m) {
            const DpcFile::Entry* e = dpc_.find(skin_.meshes[m]);
            if (e != nullptr && linkName(dpc_, *e) == nameCrc) break;
        }
        if (m == skin_.meshes.size() || m >= skin_.sets.size()) throw std::runtime_error("rider part " + part + " not found");
        const DpcFile::Entry* me = dpc_.find(skin_.meshes[m]);
        const MeshLink link = parseMeshLink(dpc_.link(*me));
        const Mesh mesh = parseMeshBody(dpc_.body(*me).value(), link.flags);
        const std::vector<SkinRecord>& recs = skin_.sets[m];
        if (recs.size() != mesh.batches.size()) throw std::runtime_error("rider part " + part + ": records and batches differ");

        std::array<int, kPaletteSlots> palette;
        palette.fill(-1);
        for (size_t bi = 0; bi < mesh.batches.size(); ++bi) {
            const Batch& b = mesh.batches[bi];
            const SkinRecord& rec = recs[bi];
            for (size_t k = 0; k < kPaletteSlots; ++k) {  // delta palette
                if (rec[1 + k] == kEnd) break;
                if (rec[1 + k] == kKeep) continue;
                const auto bn = boneByName_.find(rec[1 + k]);
                palette[k] = bn == boneByName_.end() ? -1 : int(bn->second);
            }
            const VertexBuffer& vb = mesh.vbs.at(b.vb);
            if (vb.stride != 60 && vb.stride != 48) throw std::runtime_error("rider part with unexpected vertex stride");

            BakedMaterial bm;
            bm.materialId = materialNamed(rec[0], mesh.materials);
            if (auto lv = liveryOf.find(rec[0]); lv != liveryOf.end()) {
                if (const uint32_t lm = liveryMaterial(lv->second)) {
                    if (const DpcFile::Entry* le = dpc_.find(lm)) {
                        const Material mat = parseMaterialBody(dpc_.body(*le).value());
                        bm.diffuseTex = mat.textures[0];
                        bm.specularTex = mat.textures[4];
                    }
                }
            }
            const auto key = std::make_tuple(bm.materialId, bm.diffuseTex, bm.specularTex);
            auto mi = materialIndex.find(key);
            if (mi == materialIndex.end()) {
                mi = materialIndex.emplace(key, uint32_t(out.materials.size())).first;
                out.materials.push_back(bm);
            }

            const std::vector<uint32_t> tris = batchTriangles(mesh, b);
            const std::set<uint32_t> used(tris.begin(), tris.end());
            std::map<uint32_t, uint32_t> remap;
            for (uint32_t vi : used) {
                remap[vi] = out.vertexCount();
                const uint8_t* v = &vb.data.at(size_t(vi) * vb.stride);
                float pos[3], uv[2];
                std::memcpy(pos, v, 12);
                std::memcpy(uv, v + 20, 8);
                float idx[4] = {0, 0, 0, 0}, wt[4] = {1, 0, 0, 0};
                if (vb.stride == 60) {
                    std::memcpy(idx, v + 28, 16);
                    std::memcpy(wt, v + 44, 16);
                } else {
                    std::memcpy(idx, v + 28, 4);
                    std::memcpy(wt, v + 44, 4);
                }
                // D3DCOLOR bytes are B, G, R, A: x = R, y = G, z = B.
                const double nrm[3] = {v[18] / 127.5 - 1.0, v[17] / 127.5 - 1.0, v[16] / 127.5 - 1.0};
                const double tan[3] = {v[14] / 127.5 - 1.0, v[13] / 127.5 - 1.0, v[12] / 127.5 - 1.0};
                double p[3] = {0, 0, 0}, n[3] = {0, 0, 0}, t[3] = {0, 0, 0};
                for (int k = 0; k < 4; ++k) {
                    if (wt[k] <= 0) continue;
                    const int slotIdx = int(idx[k]) / 6;  // 6 shader registers per palette slot
                    const int bone = slotIdx >= 0 && size_t(slotIdx) < kPaletteSlots ? palette[size_t(slotIdx)] : -1;
                    if (bone < 0) throw std::runtime_error("rider vertex uses an empty palette slot");
                    const Mat4& M = skinM[size_t(bone)];
                    const double w = wt[k];
                    for (size_t c = 0; c < 3; ++c) {
                        p[c] += w * (pos[0] * M[c] + pos[1] * M[4 + c] + pos[2] * M[8 + c] + M[12 + c]);
                        n[c] += w * (nrm[0] * M[c] + nrm[1] * M[4 + c] + nrm[2] * M[8 + c]);
                        t[c] += w * (tan[0] * M[c] + tan[1] * M[4 + c] + tan[2] * M[8 + c]);
                    }
                }
                auto norm = [](double* a) {
                    const double l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
                    if (l > 0)
                        for (int c = 0; c < 3; ++c) a[c] /= l;
                };
                norm(n);
                norm(t);
                uint8_t o[BakedMesh::kStride] = {};
                const float pf[3] = {float(p[0]), float(p[1]), float(p[2])};
                std::memcpy(o, pf, 12);
                o[12] = packUnit(t[2]), o[13] = packUnit(t[1]), o[14] = packUnit(t[0]), o[15] = v[15];
                o[16] = packUnit(n[2]), o[17] = packUnit(n[1]), o[18] = packUnit(n[0]), o[19] = 255;
                std::memcpy(o + 20, uv, 8);
                out.vertices.insert(out.vertices.end(), o, o + sizeof o);
            }
            BakedBatch bb;
            bb.material = mi->second;
            bb.firstIndex = uint32_t(out.indices.size());
            for (uint32_t i : tris) out.indices.push_back(remap.at(i));
            bb.indexCount = uint32_t(tris.size());
            out.batches.push_back(bb);
        }
    }
    return out;
}

}  // namespace fuel
