// P55: STARTANIMATED trigger volumes (see anim_trigger.hpp); PK12: EvenementVolAgent_G options and STARTFX children.
#include "anim_trigger.hpp"

#include "assembly.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <strings.h>

namespace fuel {

namespace {
constexpr uint32_t kNodeClass = 0x85D02528, kCollisionVolClass = 0x8EF49632;

const DpcFile::Entry* findIn(const std::vector<const DpcFile*>& lookup, uint32_t id, const DpcFile** file) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) {
            *file = f;
            return e;
        }
    return nullptr;
}

// ---- PK12: the STARTFX test of 0x570ae0 (asm 0x570e50..0x570ecf): child C whose object is a Spline_Z (object type 2)
// and whose first child G (+0xa4) holds a CollisionVol_Z (type 0xe). 0x5c0710: TIME of C (Node_UserDefineKeyword, atof),
// SAND of G, G's OBB (0x7375d0: box 0 x G's world matrix).
constexpr uint32_t kSplineClass = 0x43A9B06F;
std::vector<std::string> nodeLines(const Node& n, const std::vector<const DpcFile*>& lookup) {
    if (n.userDefine == 0) return {};
    const DpcFile* f = nullptr;
    const DpcFile::Entry* e = findIn(lookup, n.userDefine, &f);
    if (e == nullptr) return {};
    const auto b = f->body(*e);
    return b ? parseUserDefineBody(*b) : std::vector<std::string>{};
}
bool keyword(const std::vector<std::string>& lines, const char* key, std::string* value) {
    for (const std::string& l : lines) {
        std::istringstream in(l);
        std::string k, v;
        in >> k >> v;
        if (strcasecmp(k.c_str(), key) == 0) {
            if (value) *value = v;
            return true;
        }
    }
    return false;
}
void addStartFx(AnimStartVolume& vol, uint32_t cId, const Node& c, const DpcFile& dpc, const std::vector<const DpcFile*>& lookup) {
    const DpcFile* of = nullptr;
    const DpcFile::Entry* oe = c.object ? findIn(lookup, c.object, &of) : nullptr;
    if (oe == nullptr || oe->classCrc != kSplineClass || c.child == 0) return;
    const DpcFile::Entry* ge = dpc.find(c.child);
    if (ge == nullptr || ge->classCrc != kNodeClass) return;
    const auto gb = dpc.body(*ge);
    if (!gb) return;
    const Node g = parseNodeBody(*gb);
    const DpcFile* bf = nullptr;
    const DpcFile::Entry* be = g.object ? findIn(lookup, g.object, &bf) : nullptr;
    if (be == nullptr || be->classCrc != kCollisionVolClass) return;
    const auto bb = bf->body(*be);
    uint32_t n = 0;
    if (!bb || bb->size() < 4 + 64 || (std::memcpy(&n, bb->data(), 4), n == 0)) return;
    std::array<float, 16> box;
    std::memcpy(box.data(), bb->data() + 4, 64);
    StartFxChild sc;
    sc.spline = cId;
    sc.fire.node = c.child;
    std::string t;
    sc.fire.hasTime = keyword(nodeLines(c, lookup), "TIME", &t);
    if (sc.fire.hasTime) sc.fire.time = float(std::atof(t.c_str()));
    sc.fire.sand = keyword(nodeLines(g, lookup), "SAND", nullptr);
    sc.fire.obb = sand::collisionBoxObb(box, g.world);
    vol.startFx.push_back(sc);
}
// ---- PK12 end
}  // namespace

bool AnimStartVolume::contains(double x, double y, double z) const {
    const double d[3] = {x - centre[0], y - centre[1], z - centre[2]};
    for (int i = 0; i < 3; ++i) {
        const double p = d[0] * axis[size_t(i)][0] + d[1] * axis[size_t(i)][1] + d[2] * axis[size_t(i)][2];
        if (std::fabs(p) > half[size_t(i)]) return false;
    }
    return true;
}

std::vector<AnimStartVolume> loadAnimStartVolumes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup) {
    std::vector<AnimStartVolume> out;
    for (uint32_t id : dpc.ids(kNodeClass)) {
        try {
            const auto body = dpc.body(*dpc.find(id));
            if (!body) continue;
            const Node node = parseNodeBody(*body);
            if (node.userDefine == 0) continue;
            const DpcFile* uf = nullptr;
            const DpcFile::Entry* ue = findIn(lookup, node.userDefine, &uf);
            if (ue == nullptr) continue;
            const auto ub = uf->body(*ue);
            if (!ub) continue;
            const std::vector<std::string> lines = parseUserDefineBody(*ub);
            // ---- PK12: 0x570ae0 / vf24 0x570580 read each ';' line as "KEYWORD value" (0x53f640) and _stricmp the keyword.
            EventVolumeOptions opt;
            for (const std::string& l : lines) {
                std::istringstream in(l);
                std::string k, arg;
                in >> k >> arg;
                auto is = [&](const char* w) { return strcasecmp(k.c_str(), w) == 0; };
                if (is("STARTANIMATED")) opt.startAnimated = true;
                else if (is("STARTFX")) opt.startFx = true;
                else if (is("ACTIVE_IF_PREVIOUS_DONE")) opt.activeIfPreviousDone = true;
                else if (is("ACTION_DANS_VOLUME")) opt.actionInVolume = true;
                else if (is("ALWAYS_ACTIVE")) opt.alwaysActive = true;
                else if (is("INACTIVE_IN_MISSION")) opt.inactiveInMission = true;
                else if (is("SWITCH_MODE")) opt.switchMode = true;
                else if (is("ONLYWHENMISSION")) opt.onlyWhenMission = arg;
            }
            if (!opt.startAnimated && !opt.startFx) continue;
            // ---- PK12 end
            // The box: CollisionVol_Z box 0 (u32 n, then 0x80-byte boxes: matrix, inverse) x the node matrix.
            const DpcFile* bf = nullptr;
            const DpcFile::Entry* be = findIn(lookup, node.object, &bf);
            if (be == nullptr || be->classCrc != kCollisionVolClass) continue;
            const auto bb = bf->body(*be);
            if (!bb || bb->size() < 4 + 64) continue;
            uint32_t n;
            std::memcpy(&n, bb->data(), 4);
            if (n == 0) continue;
            // ---- PK12: vf24's text id (+0x78 = ftol(+0xac + 0.5)) and facing flag (+0xb0 > 1e-6): CollisionVol_Z vf03
            // 0x7fedd0 copies the 0x30 bytes after the boxes, two u32 and twelve u32 to +0xac.
            if (const size_t at = 4 + size_t(n) * 0x80 + 8 + 48; bb->size() >= at + 8) {
                float ac, b0;
                std::memcpy(&ac, bb->data() + at, 4);
                std::memcpy(&b0, bb->data() + at + 4, 4);
                opt.textInfo = int(ac + 0.5f);
                opt.facing = b0 > 1e-6f;
            }
            // ---- PK12 end
            float b[16];
            std::memcpy(b, bb->data() + 4, 64);
            const std::array<float, 16>& w = node.world;
            float m[16];  // b x w (row vectors)
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) {
                    float s = 0;
                    for (int k = 0; k < 4; ++k) s += b[r * 4 + k] * w[size_t(k * 4 + c)];
                    m[r * 4 + c] = s;
                }
            AnimStartVolume v;
            v.node = id;
            for (int r = 0; r < 3; ++r) {
                const float len = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
                v.half[size_t(r)] = 0.5f * len;  // PK12: 0x7375d0's unit OBB has extents 0.5 (0x9960c0); P55 used the full length
                for (int c = 0; c < 3; ++c) v.axis[size_t(r)][size_t(c)] = len > 0 ? m[r * 4 + c] / len : float(r == c);
            }
            v.centre = {m[12], m[13], m[14]};
            if (auto c = parseObjectCondition(lines)) v.condition = std::make_shared<const ObjectCondition>(std::move(*c));
            v.options = opt;  // PK12
            for (uint32_t ch = node.child, k = 0; ch != 0 && k < 64; ++k) {  // children: +0xa4, then +0xac
                const DpcFile::Entry* ce = dpc.find(ch);
                if (ce == nullptr || ce->classCrc != kNodeClass) break;
                const auto cb = dpc.body(*ce);
                if (!cb) break;
                const Node cn = parseNodeBody(*cb);
                if (cn.object && opt.startAnimated) v.objects.push_back(cn.object);
                if (opt.startFx) addStartFx(v, ch, cn, dpc, lookup);  // PK12
                ch = cn.prev;
            }
            out.push_back(std::move(v));
        } catch (const std::exception&) {
        }
    }
    return out;
}

}  // namespace fuel
