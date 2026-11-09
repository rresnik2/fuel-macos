// P55: FUEL's dynamic objects (see dyn_object.hpp; notes ~/re-work/fuel/notes/p55-objects2.md).
#include "dyn_object.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <strings.h>

namespace fuel {

DynObjUserDefine parseDynObjUserDefine(const std::vector<std::string>& lines) {
    DynObjUserDefine ud;
    for (const std::string& line : lines) {
        std::istringstream ls(line);
        std::string word, value;
        if (!(ls >> word)) continue;
        ls >> value;
        // 0x51bd10 upper-cases the first word (_strupr) and compares it whole (the length includes the NUL), so a keyword
        // counts in any case. Values: atof.
        // PK30: PhysAgent_Z's copy 0x740e80 cannot match a keyword in retail: its splitter 0x740df0 stops after 5
        // non-blank characters and leaves the token unterminated (emulated: "SPEEDFACTOR 150" -> "SPEED" + stale bytes,
        // no value). It runs first in IT_DYNBASE_G's reset 0x51b930 (0x741280), then the mover's defaults (vt+0x38) and
        // 0x51c170 -> 0x51bd10, whose splitter 0x53f640 counts fields (emulated: "SPEEDFACTOR", "150"), so this full
        // parse is FUEL's result for every dynamic object; nothing to change.
        std::string key = word;
        for (char& c : key) c = char(std::toupper(static_cast<unsigned char>(c)));
        const bool haveValue = !value.empty();
        const float v = haveValue ? float(std::atof(value.c_str())) : 0.0f;
        if (key == "MASSEVOLUMIQUE") {
            // 0x51be20 / 0x740fd0: no value or 0 -> 500 (0x9dbfc4); below 10 -> 10 (0x996494); above 4000 -> 4000 (0x9dc484)
            ud.massVolumique = !haveValue || v == 0.0f ? 500.0f : std::clamp(v, 10.0f, 4000.0f);
        } else if (key == "SPEEDFACTOR") {
            ud.speedFactor = std::clamp(v * 0.01f, 0.25f, 2.0f);  // 0x51bdcf: x 0.01 (0x99614c), 0.25 .. 2
        } else if (key == "HIMMOBILE") {
            ud.immobile = true;
        } else if (key == "NOMAGNET") {
            ud.noMagnet = true;
        } else if (key == "HRDEPTH") {
            ud.hrDepth = true;
        } else if (key == "DISAPPEAREXPLO" || key == "DELAY") {
            ud.explodeDelay = haveValue ? v * 0.001f : 0.5f;  // 0x741220
        } else if (key == "EXPLODAMAGE" || key == "MINPOWER") {
            ud.explodeDamage = v;  // 0x7410a2 (no value: 0)
        } else if (key == "DAMAGE") {
            ud.damage = haveValue ? v : 1.0f;
        } else if (key == "EXPLOTOUCH") {
            ud.explodeOnTouch = true;
        } else if (key == "EXPLOFX" || key == "REPLACE") {
            // a name: its CRC goes to +0xdfc / +0xe00; the delay +0xe28 becomes 0.01 s when not set (> 0)
            if (!haveValue) continue;
            (key == "EXPLOFX" ? ud.explodeFx : ud.replaceWith) = value;
            if (ud.explodeDelay <= 0) ud.explodeDelay = 0.01f;
            if (ud.explodeDamage < 0) ud.explodeDamage = 0;
        } else if (key.rfind("DISTACTIVE_", 0) == 0) {
            // the animated-object keyword (0x7b1a80): not a body property
        } else {
            ud.names.push_back(word);
        }
    }
    return ud;
}

std::vector<int> parseDistActive(const std::vector<std::string>& lines) {
    std::vector<int> out;
    for (const std::string& line : lines) {
        if (line.size() < 11 || strncasecmp(line.c_str(), "DISTACTIVE_", 11) != 0) continue;
        std::string buf = line.substr(11);
        while (true) {
            int v = -1;
            if (std::sscanf(buf.c_str(), "%d", &v) == 1 && v >= 0) out.push_back(v);
            const size_t u = buf.find('_');
            if (u == std::string::npos) break;
            buf.erase(0, u + 1);
        }
    }
    return out;
}

float distActive2(const std::vector<std::string>& lines, float fallback) {
    const std::vector<int> d = parseDistActive(lines);
    return d.empty() ? fallback : float(int32_t(uint32_t(d[0]) * uint32_t(d[0])));  // (float)(v * v), 32-bit int product
}

DynObjParams loadDynObjParams(const std::string& pp001Path) {
    DynObjParams p;
    std::ifstream f(pp001Path, std::ios::binary);
    std::string line;
    bool big = false, small = false;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tag;
        int n = -1;
        float v = 0;
        if (!(ls >> tag >> n >> v) || tag != "PP") continue;
        if (n == 150) p.densityBig = v, big = true;
        if (n == 151) p.densitySmall = v, small = true;
    }
    p.loaded = big && small;
    return p;
}

double dynObjVolume(const StaticCollision& c, float scale) {
    const double s3 = double(scale) * scale * scale;
    double v = 0.0001;  // 0x712de6: starts at 0x9dc1e8
    for (const auto& sp : c.spheres) v += 4.1762004 * double(sp.r) * sp.r * sp.r * s3;  // 0x9dc9e4
    for (const auto& b : c.boxes) v += 8.0 * double(b.half[0]) * b.half[1] * b.half[2] * s3;
    return v;
}

double dynObjMass(double volume, const DynObjUserDefine& ud, const DynObjParams& pp) {
    // 0x531590: fld1; fcomp volume -> 1 < volume: mass = +0xe24 x volume, else PP151 x volume
    const double density = ud.massVolumique >= 0 ? ud.massVolumique : pp.densityBig;
    return volume > 1.0 ? density * volume : double(pp.densitySmall) * volume;
}

std::array<float, 3> dynObjDrag(const std::array<float, 3>& v, float cap, float dt) {
    const float s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (s <= 0.0001f) return v;  // 0x5321e0: below 0x9dc1e8 no drag
    const float dv = std::min(std::min(s, cap) * kDynDragRate * dt, s);
    const float k = (s - dv) / s;
    return {v[0] * k, v[1] * k, v[2] * k};
}

}  // namespace fuel
