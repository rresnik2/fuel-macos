#include "ui/menu_check.hpp"

#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"

#include "core/dpc.hpp"
#include "core/resources.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
#include <cctype>

namespace fuel::ui {

int runMenuCheck(const std::string& dataDir, const std::string& language) {
    const MenuScripts ms = loadMenuScripts(dataDir);
    const TextTable text = loadText(dataDir, language);
    const MenuAssets assets(dataDir);
    std::vector<std::string> problems = ms.problems;
    for (const std::string& p : assets.problems) problems.push_back(p);
    size_t items = 0, texts = 0, images = 0, glyphsMissing = 0;
    std::set<std::string> badImages;
    std::map<uint32_t, std::string> badText;
    std::set<uint32_t> missingGlyphs[2];
    auto checkText = [&](const std::string& page, const std::string& id, uint32_t tt, int font) {
        if (tt == 0) return;
        ++texts;
        const std::string* s = text.get(tt);
        if (!s) {
            badText.emplace(tt, page + "/" + id);
            return;
        }
        for (uint32_t c : utf8Codes(*s)) {
            if (c == '^' || c < 0x20) continue;
            if (!assets.font(font).glyph(c) && missingGlyphs[font].insert(c).second) ++glyphsMissing;
        }
    };
    for (const Page& p : ms.pages) {
        for (const Item& it : p.items) {
            ++items;
            if (it.kind == ItemKind::Image) {
                for (const std::string* n : {&it.imgNormal, &it.imgEnabled, &it.imgFocus}) {
                    if (n->empty()) continue;
                    ++images;
                    if (!assets.image(*n)) badImages.insert(*n);
                }
            }
            if (it.isText()) checkText(p.name, it.id, it.tt, it.font);
            for (const ListEntry& e : it.entries) checkText(p.name, it.id + "/" + e.key, e.tt, it.font);
        }
    }
    // Names the menus cannot find: real problems only if some DPC of the game has them
    // (the engine draws nothing for a name no file defines: 0x49ba80 falls back to a default).
    std::set<uint32_t> everywhere;
    if (!badImages.empty()) {
        namespace fs = std::filesystem;
        const uint32_t animClass = asoboCrc("MaterialAnim_Z");
        std::error_code ec;
        for (const auto& e : fs::recursive_directory_iterator(fs::path(dataDir) / "CommonComponent", ec)) {
            std::string ext = e.path().extension().string();
            for (char& c : ext) c = char(std::toupper(static_cast<unsigned char>(c)));
            if (ext != ".DPC") continue;
            try {
                DpcFile f(e.path().string());
                for (uint32_t id : f.ids(animClass)) {
                    const auto link = f.link(*f.find(id));
                    uint32_t v = 0;
                    if (link.size() >= 4) std::memcpy(&v, link.data(), 4), everywhere.insert(v);
                }
            } catch (const std::exception&) {
            }
        }
    }
    size_t absentImages = 0;
    std::string absentList;
    for (const std::string& n : badImages) {
        if (everywhere.count(asoboCrc(n))) problems.push_back("image not loaded: " + n + " (it is in a DPC the menus do not open)");
        else ++absentImages, absentList += " " + n;
    }
    if (absentImages) std::printf("menu check: note: %zu image names are in no DPC of the game (drawn as nothing, as the original does):%s\n", absentImages, absentList.c_str());
    if (!badText.empty()) {
        std::string list;
        for (const auto& [tt, where] : badText) list += " " + std::to_string(tt);
        std::printf("menu check: note: %zu text ids are not in %s (shown empty):%s\n", badText.size(), text.file.c_str(), list.c_str());
    }
    for (int f = 0; f < 2; ++f)
        if (!missingGlyphs[f].empty()) {
            std::string list;
            for (uint32_t c : missingGlyphs[f]) {
                char b[16];
                std::snprintf(b, sizeof b, " %X", c);
                list += b;
            }
            problems.push_back("font " + assets.font(f).name + " lacks glyphs:" + list);
        }
    for (const std::string& p : problems) std::printf("menu check: %s\n", p.c_str());
    std::printf("menu check: %zu pages, %zu items, %zu text refs (%zu strings in %s), %zu image refs (%zu names in no DPC), fonts %s %zu glyphs / %s %zu glyphs\n",
                ms.pages.size(), items, texts, text.strings.size(), text.file.c_str(), images, absentImages, assets.font(0).name.c_str(),
                assets.font(0).glyphs.size(), assets.font(1).name.c_str(), assets.font(1).glyphs.size());
    std::printf("menu check: %s (%zu problems)\n", problems.empty() ? "ok" : "PROBLEMS", problems.size());
    return problems.empty() ? 0 : 1;
}

}  // namespace fuel::ui
