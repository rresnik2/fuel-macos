#include "rider.hpp"

#include "reader.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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

std::optional<SeatPoint> riderSeatPoint(const DpcFile& vehicleDpc, const AssembledVehicle& av) {
    if (!av.skeleton) return std::nullopt;
    const DpcFile::Entry* e = vehicleDpc.find(*av.skeleton);
    if (e == nullptr) return std::nullopt;
    const auto body = vehicleDpc.body(*e);
    if (!body) return std::nullopt;
    const Skel sk = parseSkelBody(*body);
    static const uint32_t kHead = asoboCrc("Head");  // registry node 61
    for (const Bone& b : sk.bones) {
        if (b.name != kHead) continue;
        SeatPoint s;
        s.t = {b.world[12], b.world[13], b.world[14]};
        const float id[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (std::fabs(b.world[size_t(r * 4 + c)] - id[r * 3 + c]) > 1e-4f) s.rotationIdentity = false;
        return s;
    }
    return std::nullopt;
}

std::array<std::array<float, 3>, 4> riderIkPoints(const DpcFile& dpc, const AssembledVehicle& av) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::array<std::array<float, 3>, 4> out;
    for (auto& p : out) p = {nan, nan, nan};
    static const uint32_t crc[4] = {asoboCrc("IK_LEFTHAND"), asoboCrc("IK_RIGHTHAND"), asoboCrc("IK_LEFTFOOT"), asoboCrc("IK_RIGHTFOOT")};
    auto place = [](const Mat4& m, const std::array<float, 3>& c) {
        const double x = c[0], y = c[1], z = c[2];
        return std::array<float, 3>{float(x * m[0] + y * m[4] + z * m[8] + m[12]), float(x * m[1] + y * m[5] + z * m[9] + m[13]),
                                    float(x * m[2] + y * m[6] + z * m[10] + m[14])};
    };
    for (const PlacedPart& part : av.parts) {  // parts in assembly order; the first that has the point wins
        if (!part.drawn() || part.shape != PartShape::Mesh) continue;
        const DpcFile::Entry* e = dpc.find(part.meshId);
        if (e == nullptr) continue;
        std::vector<NamedSphere> spheres;
        std::vector<NamedBox> boxes;
        try {
            const std::vector<uint8_t> link = dpc.link(*e);
            spheres = parseMeshLinkSpheres(link);
            boxes = parseMeshLinkBoxes(link);
        } catch (const std::exception&) {
            continue;
        }
        for (int k = 0; k < 4; ++k) {
            if (!std::isnan(out[size_t(k)][0])) continue;
            for (const NamedBox& b : boxes)
                if (b.name == crc[k]) {
                    out[size_t(k)] = place(part.world, b.centre());
                    break;
                }
            if (!std::isnan(out[size_t(k)][0])) continue;
            for (const NamedSphere& sp : spheres)
                if (sp.name == crc[k]) {
                    out[size_t(k)] = place(part.world, sp.centre);
                    break;
                }
        }
    }
    return out;
}

std::array<std::array<double, 9>, 2> riderIkHandFrames(const DpcFile& dpc, const AssembledVehicle& av) {
    std::array<std::array<double, 9>, 2> out{};
    for (auto& f : out) f[0] = std::numeric_limits<double>::quiet_NaN();
    static const uint32_t crc[2] = {asoboCrc("IK_LEFTHAND"), asoboCrc("IK_RIGHTHAND")};
    bool done[2] = {false, false};
    for (const PlacedPart& part : av.parts) {  // the same first-part-wins search as riderIkPoints
        if (!part.drawn() || part.shape != PartShape::Mesh) continue;
        const DpcFile::Entry* e = dpc.find(part.meshId);
        if (e == nullptr) continue;
        std::vector<NamedSphere> spheres;
        std::vector<NamedBox> boxes;
        try {
            const std::vector<uint8_t> link = dpc.link(*e);
            spheres = parseMeshLinkSpheres(link);
            boxes = parseMeshLinkBoxes(link);
        } catch (const std::exception&) {
            continue;
        }
        for (int k = 0; k < 2; ++k) {
            if (done[k]) continue;
            for (const NamedBox& b : boxes)
                if (b.name == crc[k]) {
                    // box axes: row r = (rows[r * 4], rows[r * 4 + 1], rows[r * 4 + 2]) = image of axis r (row-vector; the
                    // other reading turns the driving hands 106 deg from their boxes, this one 26 deg: --animcheck), then the part
                    double P[9];
                    for (int r = 0; r < 3; ++r) {
                        const double l = std::sqrt(double(part.world[size_t(r * 4)]) * part.world[size_t(r * 4)] + double(part.world[size_t(r * 4 + 1)]) * part.world[size_t(r * 4 + 1)] +
                                                   double(part.world[size_t(r * 4 + 2)]) * part.world[size_t(r * 4 + 2)]);
                        for (int c = 0; c < 3; ++c) P[r * 3 + c] = l > 0 ? part.world[size_t(r * 4 + c)] / l : (r == c);
                    }
                    double B[9];
                    for (int r = 0; r < 3; ++r) {  // unit rows
                        double v[3] = {b.rows[size_t(r * 4)], b.rows[size_t(r * 4 + 1)], b.rows[size_t(r * 4 + 2)]};
                        const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                        for (int c = 0; c < 3; ++c) B[r * 3 + c] = l > 0 ? v[c] / l : (r == c);
                    }
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c) out[size_t(k)][size_t(r * 3 + c)] = B[r * 3] * P[c] + B[r * 3 + 1] * P[3 + c] + B[r * 3 + 2] * P[6 + c];
                    done[k] = true;
                    break;
                }
            if (done[k]) continue;
            for (const NamedSphere& sp : spheres)
                if (sp.name == crc[k]) {
                    done[k] = true;  // found as a point: no orientation (stays NaN)
                    break;
                }
        }
    }
    return out;
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
    lodSkins_.push_back(skin_);  // P104: the lower levels (the same parts and skeleton)
    for (size_t k = 1; k < lod.objects.size(); ++k)
        if (const DpcFile::Entry* e = pilot.find(lod.objects[k]); e != nullptr && e->classCrc == cls::Skin)
            lodSkins_.push_back(parseSkinBody(pilot.body(*e).value()));
        else
            break;
    lodD_ = lod.baseDistance;
    if (const std::vector<uint8_t> l = pilot.link(*lodE); l.size() >= 92) {
        float v;
        std::memcpy(&v, &l[88], 4);
        lodView_ = std::isfinite(v) ? v : 0;
    }
    const DpcFile::Entry* skelE = pilot.find(parseObjectLink(pilot.link(*skinE)).dataId);
    if (skelE == nullptr) throw std::runtime_error("rider Skel_Z missing");
    skel_ = parseSkelBody(pilot.body(*skelE).value());
    for (size_t i = 0; i < skel_.bones.size(); ++i) boneByName_[skel_.bones[i].name] = i;
    for (const Bone& b : skel_.bones) {
        Mat4 bind;
        for (size_t k = 0; k < 16; ++k) bind[k] = b.world[k];
        invBind_.push_back(invertAffine(bind));
    }
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
    Animation a;
    if (!a.clip.parse(body)) throw std::runtime_error("malformed rider animation");
    a.duration = a.clip.duration;
    a.loop = a.clip.loops();  // flags (+0x14 at run time): bit 0 = loop (0x7c02a0)
    for (size_t i = 0; i < a.clip.tracks.size(); ++i) a.tracks[a.clip.tracks[i].bone] = int(i);
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
                anim::Pose p{q, s, t};
                a->clip.sample(it->second, time, p);
                q = p.q, s = p.s, t = p.t;
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

float Rider::duration(int slot) const { return animation(slot).duration; }

void Rider::events(int slot, double from, double to, bool closed, std::vector<std::pair<uint32_t, uint32_t>>& out) const {
    const Animation& a = animation(slot);
    std::vector<std::pair<float, uint32_t>> ev;
    for (size_t t = 0; t < a.clip.tracks.size(); ++t) {
        ev.clear();
        a.clip.events(int(t), from, to, closed, ev);
        for (const auto& e : ev) out.push_back({a.clip.tracks[t].bone, e.second});
    }
}
bool Rider::loops(int slot) const { return animation(slot).loop; }

int Rider::bone(const std::string& name) const {
    const auto it = boneByName_.find(asoboCrc(name));
    return it == boneByName_.end() ? -1 : int(it->second);
}

std::vector<Rider::BoneTRS> Rider::bindLocal() const {
    std::vector<BoneTRS> out(skel_.bones.size());
    for (size_t i = 0; i < skel_.bones.size(); ++i) {
        const Bone& b = skel_.bones[i];
        out[i].q = {b.localQuat[0], b.localQuat[1], b.localQuat[2], b.localQuat[3]};
        out[i].t = {b.localT[0], b.localT[1], b.localT[2]};
        out[i].s = {b.localScale[0], b.localScale[1], b.localScale[2]};
    }
    return out;
}

void Rider::sampleLocal(int slot, double time, std::vector<BoneTRS>& out, std::vector<uint8_t>* has) const {
    const Animation& a = animation(slot);
    out = bindLocal();
    if (has) has->assign(skel_.bones.size(), 0);
    for (size_t i = 0; i < skel_.bones.size(); ++i) {
        const auto it = a.tracks.find(skel_.bones[i].name);
        if (it == a.tracks.end()) continue;
        anim::Pose p{out[i].q, out[i].s, out[i].t};
        a.clip.sample(it->second, time, p);
        out[i].q = p.q, out[i].s = p.s, out[i].t = p.t;
        if (has) (*has)[i] = 1;
    }
}

std::vector<Mat4> Rider::worlds(const std::vector<BoneTRS>& local) const {
    std::vector<Mat4> world(skel_.bones.size());
    for (size_t i = 0; i < skel_.bones.size(); ++i) {
        const Mat4 m = localMatrix(local[i].s, local[i].q, local[i].t);
        const int p = skel_.bones[i].parent;
        world[i] = p >= 0 ? mul(m, world[size_t(p)]) : m;
    }
    return world;
}

BakedMesh Rider::bake(int slot, double time, int head, int chest, int legs, int livery) const {
    const SkinTemplate t = prepareSkin(head, chest, legs, livery);
    BakedMesh out = t.mesh;
    skin(t, pose(slot, time), out.vertices);
    return out;
}

Mat4 Rider::skinMatrix(int b, const std::vector<Mat4>& world) const { return mul(invBind_[size_t(b)], world[size_t(b)]); }

void Rider::skin(const SkinTemplate& t, const std::vector<Mat4>& world, std::vector<uint8_t>& vertices, const RiderJiggle* jiggle) const {
    std::vector<Mat4> skinM(world.size());
    for (size_t i = 0; i < world.size(); ++i) skinM[i] = mul(invBind_[i], world[i]);
    const bool jig = jiggle != nullptr && jiggle->active() && t.hasJiggle;
    vertices.resize(t.verts.size() * BakedMesh::kStride);
    for (size_t vi = 0; vi < t.verts.size(); ++vi) {
        const SkinTemplate::V& v = t.verts[vi];
        double p[3] = {0, 0, 0}, n[3] = {0, 0, 0}, tg[3] = {0, 0, 0};
        for (int k = 0; k < 4; ++k) {
            if (v.w[k] <= 0) continue;
            const Mat4& M = skinM[size_t(v.bone[k])];
            const double w = v.w[k];
            for (size_t c = 0; c < 3; ++c) {
                p[c] += w * (v.pos[0] * M[c] + v.pos[1] * M[4 + c] + v.pos[2] * M[8 + c] + M[12 + c]);
                n[c] += w * (v.nrm[0] * M[c] + v.nrm[1] * M[4 + c] + v.nrm[2] * M[8 + c]);
                tg[c] += w * (v.tan[0] * M[c] + v.tan[1] * M[4 + c] + v.tan[2] * M[8 + c]);
            }
        }
        auto norm = [](double* a) {
            const double l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if (l > 0)
                for (int c = 0; c < 3; ++c) a[c] /= l;
        };
        if (jig && v.jig > 0 && v.jw > 0) {  // PHYSIQUE: the glute helper bone's offset, moved like its base bone (Hips)
            const JiggleBone& jb = t.jiggleBones[size_t(v.jig - 1)];
            if (jb.base >= 0 && size_t(jb.base) < skinM.size()) {
                const std::array<double, 3> d = jiggle->displace(v.jig - 1, v.pos);
                const Mat4& M = skinM[size_t(jb.base)];
                for (size_t c = 0; c < 3; ++c) p[c] += v.jw * (d[0] * M[c] + d[1] * M[4 + c] + d[2] * M[8 + c]);
            }
        }
        norm(n);
        norm(tg);
        uint8_t* o = &vertices[vi * BakedMesh::kStride];
        const uint8_t* src = &t.mesh.vertices[vi * BakedMesh::kStride];
        const float pf[3] = {float(p[0]), float(p[1]), float(p[2])};
        std::memcpy(o, pf, 12);
        o[12] = packUnit(tg[2]), o[13] = packUnit(tg[1]), o[14] = packUnit(tg[0]), o[15] = src[15];
        o[16] = packUnit(n[2]), o[17] = packUnit(n[1]), o[18] = packUnit(n[0]), o[19] = 255;
        std::memcpy(o + 20, src + 20, 16);
    }
}

std::set<uint32_t> riderLiveryNames(const DpcFile& pilot) {
    std::set<uint32_t> out;
    for (uint32_t id : pilot.ids(cls::MaterialAnim))
        if (const DpcFile::Entry* e = pilot.find(id)) out.insert(linkName(pilot, *e));
    return out;
}

int riderLiveryCount(const std::set<uint32_t>& names, const std::string& part, bool types) {
    char buf[64];
    int n = 0;
    for (int k = 1; k < 1000; ++k) {
        if (types) std::snprintf(buf, sizeof buf, "LIVERY_01_%s%d", part.c_str(), k);
        else std::snprintf(buf, sizeof buf, "LIVERY_%02d_%s", k, part.c_str());
        if (!names.count(asoboCrc(buf))) break;
        ++n;
    }
    return n;
}

Rider::SkinTemplate Rider::prepareSkin(int head, int chest, int legs, int livery) const {
    Parts p;
    p.head = head, p.chest = chest, p.legs = legs;
    p.headLivery = p.chestLivery = p.legsLivery = p.bodyLivery = livery;
    return prepareSkin(p);
}

namespace {
// The livery material names of a garage choice: part material name CRC -> livery material name (prepareSkin).
std::map<uint32_t, std::string> liveryNames(const Rider::Parts& parts) {
    char buf[48];
    std::map<uint32_t, std::string> liveryOf;
    auto addLivery = [&](const std::string& part, int livery) {
        std::snprintf(buf, sizeof buf, "LIVERY_%02d_%s", livery, part.c_str());
        liveryOf[asoboCrc("LIVERY_XX_" + part)] = buf;
    };
    addLivery("HEAD" + std::to_string(parts.head), parts.headLivery);
    addLivery("CHEST" + std::to_string(parts.chest), parts.chestLivery);
    addLivery("LEGS" + std::to_string(parts.legs), parts.legsLivery);
    // The skin: body type <skin> with its livery (0x42b420: pilot +0xe8 / +0xe0, counts 0x5dc850 / 0x5dc710).
    std::snprintf(buf, sizeof buf, "LIVERY_%02d_BODY%d", parts.bodyLivery, parts.skin);
    liveryOf[asoboCrc("LIVERY_XX_BODY")] = buf;
    return liveryOf;
}
}  // namespace

BakedMaterial Rider::partMaterial(uint32_t nameCrc, const std::vector<uint32_t>& preferred, const Parts& parts) const {  // PHYSIQUE
    const std::map<uint32_t, std::string> liveryOf = liveryNames(parts);
    BakedMaterial bm;
    bm.materialId = materialNamed(nameCrc, preferred);
    if (auto lv = liveryOf.find(nameCrc); lv != liveryOf.end())
        if (const uint32_t lm = liveryMaterial(lv->second))
            if (const DpcFile::Entry* le = dpc_.find(lm)) {
                const Material mat = parseMaterialBody(dpc_.body(*le).value());
                bm.diffuseTex = mat.textures[0];
                bm.specularTex = mat.textures[4];
            }
    return bm;
}

Rider::SkinTemplate Rider::prepareSkin(const Parts& parts, int level) const { return prepareSkin(parts, level, 7u); }

Rider::SkinTemplate Rider::prepareSkin(const Parts& parts, int level, unsigned partMask) const {
    const Skin& skin_ = lodSkins_.at(size_t(std::clamp(level, 0, int(lodSkins_.size()) - 1)));  // P104
    const std::string allParts[3] = {"Head" + std::to_string(parts.head), "Chest" + std::to_string(parts.chest), "Legs" + std::to_string(parts.legs)};
    std::vector<std::string> partNames;
    for (int k = 0; k < 3; ++k)
        if (partMask & (1u << k)) partNames.push_back(allParts[k]);  // PHYSIQUE: a subset
    const std::map<uint32_t, std::string> liveryOf = liveryNames(parts);  // part material name CRC -> livery material name

    SkinTemplate st;
    BakedMesh& out = st.mesh;
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
                remap[vi] = uint32_t(st.verts.size());
                const uint8_t* v = &vb.data.at(size_t(vi) * vb.stride);
                SkinTemplate::V sv{};
                std::memcpy(sv.pos, v, 12);
                float idx[4] = {0, 0, 0, 0}, wt[4] = {1, 0, 0, 0};
                if (vb.stride == 60) {
                    std::memcpy(idx, v + 28, 16);
                    std::memcpy(wt, v + 44, 16);
                } else {
                    std::memcpy(idx, v + 28, 4);
                    std::memcpy(wt, v + 44, 4);
                }
                // D3DCOLOR bytes are B, G, R, A: x = R, y = G, z = B.
                sv.nrm[0] = float(v[18] / 127.5 - 1.0), sv.nrm[1] = float(v[17] / 127.5 - 1.0), sv.nrm[2] = float(v[16] / 127.5 - 1.0);
                sv.tan[0] = float(v[14] / 127.5 - 1.0), sv.tan[1] = float(v[13] / 127.5 - 1.0), sv.tan[2] = float(v[12] / 127.5 - 1.0);
                for (int k = 0; k < 4; ++k) {
                    sv.w[k] = wt[k] > 0 ? wt[k] : 0;
                    sv.bone[k] = 0;
                    if (wt[k] <= 0) continue;
                    const int slotIdx = int(idx[k]) / 6;  // 6 shader registers per palette slot
                    const int bone = slotIdx >= 0 && size_t(slotIdx) < kPaletteSlots ? palette[size_t(slotIdx)] : -1;
                    if (bone < 0) throw std::runtime_error("rider vertex uses an empty palette slot");
                    sv.bone[k] = bone;
                }
                st.verts.push_back(sv);
                uint8_t o[BakedMesh::kStride] = {};
                std::memcpy(o, sv.pos, 12);
                o[15] = v[15];
                float uv[2];
                std::memcpy(uv, v + 20, 8);
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
    return st;
}

}  // namespace fuel
