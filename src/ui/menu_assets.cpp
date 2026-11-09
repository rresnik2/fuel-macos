#include "ui/menu_assets.hpp"

#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "core/resources.hpp"
#include "ui/tsc.hpp"

#include <cstring>
#include <filesystem>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

uint32_t u32(const uint8_t* p) {
    uint32_t v;
    std::memcpy(&v, p, 4);
    return v;
}
float f32(const uint8_t* p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}

}  // namespace

std::vector<uint32_t> utf8Codes(const std::string& s) {
    std::vector<uint32_t> out;
    for (size_t i = 0; i < s.size();) {
        const uint8_t c = uint8_t(s[i]);
        size_t n = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 1;
        if (i + n > s.size()) n = 1;
        uint32_t code = 0;
        for (size_t k = 0; k < n; ++k) code = (code << 8) | uint8_t(s[i + k]);
        out.push_back(code);
        i += n;
    }
    return out;
}

MenuAssets::MenuAssets(const std::string& dataDir, const std::vector<std::string>& first, bool menuFiles) {
    const std::string common = (fs::path(dataDir) / "CommonComponent").string();
    std::vector<std::string> rels = first;
    for (const char* rel : {"DATAS\\SHARED.DPC", "DATAS\\FONTES.DPC", "DATAS\\LMENU.DPC", "WORLD\\MENU.DPC"})
        if (menuFiles || std::string(rel) == "DATAS\\FONTES.DPC") rels.push_back(rel);
    // CUSTOM VEHICLES: the packs' archives carry their garage carousel picture (MaterialAnim_Z "VEH_<V>").
    if (menuFiles)
        for (const custom::Vehicle& cv : custom::vehicles()) rels.push_back(cv.dpc);
    for (const std::string& rel : rels) {
        const std::string path = !rel.empty() && rel[0] == '/' ? rel : findPathNoCase(common, rel);
        if (path.empty()) {
            problems.push_back("missing " + rel);
            continue;
        }
        try {
            files_.push_back(std::make_unique<DpcFile>(path));
        } catch (const std::exception& e) {
            problems.push_back(path + ": " + e.what());
        }
    }
    const uint32_t animClass = asoboCrc("MaterialAnim_Z"), fontClass = asoboCrc("Fonts_Z");
    std::unordered_map<uint32_t, std::pair<const DpcFile*, uint32_t>> fontsByName;
    for (const auto& f : files_) {
        for (uint32_t id : f->ids(animClass)) {
            const auto link = f->link(*f->find(id));
            if (link.size() >= 4) anims_.emplace(u32(link.data()), std::make_pair(f.get(), id));
        }
        for (uint32_t id : f->ids(fontClass)) {
            const auto link = f->link(*f->find(id));
            if (link.size() >= 4) fontsByName[u32(link.data())] = {f.get(), id};
        }
    }
    // initlib.tsc: LoadFont SMALL_FONT, BIG_FONT and (#if _PC) SMALL_FONT_PC.
    const char* names[2][2] = {{"SMALL_FONT_PC", "SMALL_FONT"}, {"BIG_FONT", "BIG_FONT"}};
    for (int k = 0; k < 2; ++k) {
        const std::pair<const DpcFile*, uint32_t>* hit = nullptr;
        for (const char* n : names[k])
            if (auto it = fontsByName.find(asoboCrc(n)); it != fontsByName.end() && !hit) hit = &it->second, fonts_[k].name = n;
        if (!hit) {
            problems.push_back(std::string("font not found: ") + names[k][0]);
            continue;
        }
        const auto body = hit->first->body(*hit->first->find(hit->second));
        if (!body || body->size() < 4) {
            problems.push_back(fonts_[k].name + ": no body");
            continue;
        }
        const uint8_t* p = body->data();
        const uint32_t n = u32(p);
        size_t o = 4;
        for (uint32_t g = 0; g < n && o + 28 <= body->size(); ++g, o += 28)
            fonts_[k].glyphs[u32(p + o)] = Glyph{u32(p + o + 4), f32(p + o + 8), f32(p + o + 12), f32(p + o + 16), f32(p + o + 20), f32(p + o + 24)};
        const uint32_t pages = o + 4 <= body->size() ? u32(p + o) : 0;
        o += 4;
        for (uint32_t q = 0; q < pages && o + 4 <= body->size(); ++q, o += 4) {
            BitmapRef b = materialBitmap(u32(p + o));
            if (!b) problems.push_back(fonts_[k].name + ": page " + std::to_string(q) + " has no bitmap");
            fonts_[k].pages.push_back(b);
        }
        if (const Glyph* e = fonts_[k].glyph(0xC389)) {  // "É" (FUEL.exe 0x9ab0f8)
            fonts_[k].refTop = e->top;
            fonts_[k].refHeight = std::max(1.0f, e->y1 - e->y0);
        } else {
            problems.push_back(fonts_[k].name + ": no reference glyph");
        }
    }
    // ---- PK26: the pad-button icons of SMALL_FONT (codes 1..19: A, B, X, Y, sticks, LB/RB, RT, BACK, START) join the small
    // font as codes 0x101..0x113 (pages appended). SMALL_FONT_PC has keyboard key caps at 1..18 (the glyphs 0x4e2580 picks
    // for keys); the {STR_*} pad labels (one byte = the 0xa675c8 index) draw these instead while a pad is present
    // (UiRenderer::padIcons) [I: FUEL's menu text resolves "SMALL_FONT" by name (0x608f20); the port's small font is _PC].
    if (auto it = fontsByName.find(asoboCrc("SMALL_FONT")); it != fontsByName.end() && fonts_[0].name == "SMALL_FONT_PC") {
        const DpcFile* df = it->second.first;
        const auto body = df->body(*df->find(it->second.second));
        if (body && body->size() >= 4) {
            const uint8_t* p = body->data();
            const uint32_t n = u32(p);
            size_t o = 4;
            std::vector<std::pair<uint32_t, Glyph>> icons;
            for (uint32_t g = 0; g < n && o + 28 <= body->size(); ++g, o += 28)
                if (const uint32_t code = u32(p + o); code >= 1 && code < 0x20)
                    icons.emplace_back(code, Glyph{u32(p + o + 4), f32(p + o + 8), f32(p + o + 12), f32(p + o + 16), f32(p + o + 20), f32(p + o + 24)});
            const uint32_t pages = o + 4 <= body->size() ? u32(p + o) : 0;
            o += 4;
            const uint32_t base = uint32_t(fonts_[0].pages.size());
            for (uint32_t q = 0; q < pages && o + 4 <= body->size(); ++q, o += 4) fonts_[0].pages.push_back(materialBitmap(u32(p + o)));
            for (auto& [code, g] : icons) g.page += base, fonts_[0].glyphs[0x100 + code] = g;
            padIcons_ = icons.size();
        }
    }
    // ---- end PK26
}

BitmapRef MenuAssets::materialBitmap(uint32_t materialId) const {
    for (const auto& f : files_) {
        const DpcFile::Entry* e = f->find(materialId);
        if (!e || e->classCrc != cls::Material) continue;
        try {
            const auto body = f->body(*e);
            if (!body) continue;
            const Material m = parseMaterialBody(*body);
            const uint32_t bm = m.textures[0];
            for (const auto& g : files_) {
                const DpcFile::Entry* be = g->find(bm);
                if (!be || be->classCrc != cls::Bitmap) continue;
                const BitmapInfo info = parseBitmapLink(g->link(*be));
                return BitmapRef{g.get(), bm, info.width, info.height};
            }
        } catch (const std::exception&) {
        }
    }
    return {};
}

BitmapRef MenuAssets::image(const std::string& name) const {
    auto it = anims_.find(asoboCrc(name));
    if (it == anims_.end()) return {};
    const DpcFile* f = it->second.first;
    const auto body = f->body(*f->find(it->second.second));
    if (!body || body->size() < 9) return {};
    // The material id sits 9 bytes before the end (+60 in the common 69-byte body).
    return materialBitmap(u32(body->data() + body->size() - 9));
}

}  // namespace fuel::ui
