#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include <sstream>
#include "ui/menu_model.hpp"
#include "ui/help_texts.hpp"  // PK26

#include "ui/tsc.hpp"

#include <algorithm>
#include <cstdio>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// The console's argument i (command name excluded).
struct Args {
    const std::vector<std::string>& v;
    size_t n() const { return v.size() - 1; }
    const std::string& s(size_t i) const {
        static const std::string empty;
        return i + 1 < v.size() ? v[i + 1] : empty;
    }
    float f(size_t i) const { return float(std::atof(s(i).c_str())); }
    int i(size_t k) const { return int(std::lround(f(k))); }  // ftol of the float form (0x8e01d0)
    bool b(size_t k) const { return i(k) != 0; }
};

// 0x49c200: alignment code (the item's "code" >> 1).
void applyCode(Item& it, int code, float offset) {
    it.font = (code & 1) ? 1 : 0;
    const int a = code >> 1;
    switch (a) {
        case 1: it.align = 2, it.xOffset = offset; break;
        case 2: it.align = 1, it.xOffset = offset; break;
        case 3: it.align = 0, it.slant = true; break;
        case 4: it.align = 2, it.xOffset = offset, it.slant = true; break;
        case 5: it.align = 1, it.xOffset = offset, it.slant = true; break;
        default: it.align = 0; break;
    }
}

void readGeometry(Item& it, const Args& a) {
    it.id = a.s(1);
    it.group = a.s(2);
    it.x = a.f(3), it.y = a.f(4), it.w = a.f(5), it.h = a.f(6);
}

// The tail common to the text items: [tt] enabled visible cc [code b19 upper offset anchorH anchorV].
// `k` = index of `enabled`; `full` = argument count above which the six-value tail is read,
// `fontOnly` = count above which only the code is read.
void readTextTail(Item& it, const Args& a, size_t k, size_t full, size_t fontOnly) {
    it.enabled = a.b(k);
    it.visible = a.b(k + 1);
    it.flagCC = a.b(k + 2);
    if (a.n() > full) {
        applyCode(it, a.i(k + 3), a.f(k + 6));
        it.b19 = a.b(k + 4);
        it.upper = a.b(k + 5);
        it.anchorH = a.i(k + 7);
        it.anchorV = a.i(k + 8);
    } else if (a.n() > fontOnly) {
        it.font = std::min(1, std::max(0, a.i(k + 3)));
    }
}

}  // namespace

Rgba parseArgb(const std::string& s, bool* ok) {
    int v[4] = {255, 255, 255, 255};
    int n = 0;
    size_t p = 0;
    while (n < 4 && p <= s.size()) {
        const size_t q = s.find(':', p);
        const std::string part = s.substr(p, q == std::string::npos ? std::string::npos : q - p);
        if (part.empty() || part.find_first_not_of("0123456789") != std::string::npos) break;
        v[n++] = std::atoi(part.c_str());
        if (q == std::string::npos) break;
        p = q + 1;
    }
    if (ok) *ok = n == 4;
    return Rgba{v[1] / 255.0f, v[2] / 255.0f, v[3] / 255.0f, v[0] / 255.0f};
}

const char* itemKindName(ItemKind k) {
    switch (k) {
        case ItemKind::Image: return "Image";
        case ItemKind::FillRect: return "FillRect";
        case ItemKind::LabelTT: return "LabelTT";
        case ItemKind::LabelDyn: return "LabelDyn";
        case ItemKind::ButtonTT: return "ButtonTT";
        case ItemKind::ButtonDyn: return "ButtonDyn";
        case ItemKind::ListBoxTT: return "ListBoxTT";
    }
    return "?";
}

Item* Page::find(const std::string& id) {
    for (Item& it : items)
        if (it.id == id) return &it;
    return nullptr;
}
const Item* Page::find(const std::string& id) const { return const_cast<Page*>(this)->find(id); }

Page* MenuScripts::page(const std::string& name) {
    auto it = index.find(upper(name));
    return it == index.end() ? nullptr : &pages[it->second];
}
const Page* MenuScripts::page(const std::string& name) const { return const_cast<MenuScripts*>(this)->page(name); }

MenuScripts loadMenuScripts(const std::string& dataDir) {
    MenuScripts ms;
    TscReader reader((fs::path(dataDir) / "LocalizedComponent").string());
    auto colour = [&](const std::string& s, const TscLine& l) {
        bool ok = false;
        Rgba c = parseArgb(s, &ok);
        if (!ok) ms.problems.push_back(l.file + ":" + std::to_string(l.line) + ": bad colour '" + s + "'");
        return c;
    };
    reader.run("GameTsc\\interface\\InterfacesInfosPacked.tsc", {"interface"}, [&](const TscLine& l) {
        const std::string cmd = upper(l.args[0]);
        const Args a{l.args};
        if (cmd == "CREATEPAGE") {
            ms.index[upper(a.s(0))] = ms.pages.size();
            ms.pages.push_back(Page{a.s(0), {}, {}});
            return;
        }
        Page* page = ms.page(a.s(0));
        if (!page && cmd.rfind("ADDMENUITEM", 0) == 0) {
            ms.problems.push_back(l.file + ":" + std::to_string(l.line) + ": item for unknown page " + a.s(0));
            return;
        }
        Item it;
        if (cmd == "ADDMENUITEMIMAGE") {  // 0x49ca30
            it.kind = ItemKind::Image;
            it.id = a.s(1), it.group = a.s(2);
            it.imgEnabled = a.s(3), it.imgNormal = a.s(4);
            it.x = a.f(5), it.y = a.f(6), it.w = a.f(7), it.h = a.f(8);
            it.a7 = a.i(9), it.a8 = a.i(10);
            it.cNormal = colour(a.s(11), l), it.cEnabled = colour(a.s(12), l), it.cFocus = colour(a.s(13), l);
            it.enabled = a.b(14), it.visible = a.b(15);
            it.imgFocus = it.imgEnabled;
            if (a.n() > 17) it.anchorH = a.i(16), it.anchorV = a.i(17), it.imgFocus = a.s(18);
        } else if (cmd == "ADDMENUITEMFILLRECT") {  // 0x49c800
            it.kind = ItemKind::FillRect;
            readGeometry(it, a);
            it.a7 = a.i(7), it.a8 = a.i(8);
            it.cNormal = colour(a.s(9), l), it.cEnabled = colour(a.s(10), l), it.cFocus = colour(a.s(11), l);
            it.enabled = a.b(12), it.visible = a.b(13);
            if (a.n() > 15) it.anchorH = a.i(14), it.anchorV = a.i(15);
        } else if (cmd == "ADDMENUITEMLABELTT" || cmd == "ADDMENUITEMLABELDYN") {  // 0x49cd60 / 0x49d170
            it.kind = cmd == "ADDMENUITEMLABELTT" ? ItemKind::LabelTT : ItemKind::LabelDyn;
            readGeometry(it, a);
            it.a7 = a.i(7), it.a8 = a.i(8), it.f9 = a.f(9);
            it.cNormal = colour(a.s(10), l), it.cEnabled = colour(a.s(11), l);
            it.tNormal = colour(a.s(12), l), it.tEnabled = colour(a.s(13), l);
            it.cFocus = it.cEnabled, it.tFocus = it.tEnabled;
            it.tt = uint32_t(std::max(0, a.i(14)));
            readTextTail(it, a, 15, 20, 19);
        } else if (cmd == "ADDMENUITEMBUTTONTT" || cmd == "ADDMENUITEMBUTTONDYN") {  // 0x49d540 / 0x49d970
            it.kind = cmd == "ADDMENUITEMBUTTONTT" ? ItemKind::ButtonTT : ItemKind::ButtonDyn;
            readGeometry(it, a);
            it.a7 = a.i(7), it.a8 = a.i(8), it.f9 = a.f(9);
            it.cNormal = colour(a.s(10), l), it.cEnabled = colour(a.s(11), l), it.cFocus = colour(a.s(12), l);
            it.tNormal = colour(a.s(13), l), it.tEnabled = colour(a.s(14), l), it.tFocus = colour(a.s(15), l);
            it.tt = uint32_t(std::max(0, a.i(16)));
            readTextTail(it, a, 17, 22, 21);
        } else if (cmd == "ADDMENUITEMLISTBOXTT") {  // 0x49dda0
            it.kind = ItemKind::ListBoxTT;
            readGeometry(it, a);
            it.a7 = a.i(7), it.a8 = a.i(8), it.f9 = a.f(9);
            it.cNormal = colour(a.s(10), l), it.cEnabled = colour(a.s(11), l), it.cFocus = colour(a.s(12), l);
            it.tNormal = colour(a.s(13), l), it.tEnabled = colour(a.s(14), l), it.tFocus = colour(a.s(15), l);
            readTextTail(it, a, 16, 21, 20);
        } else if (cmd == "ADDTTINFOS") {  // 0x49e1c0: page item KEY tt flag
            if (Item* lb = page->find(a.s(1)))
                lb->entries.push_back(ListEntry{a.s(2), uint32_t(std::max(0, a.i(3))), upper(a.s(4)) == "TRUE"});
            else
                ms.problems.push_back(l.file + ":" + std::to_string(l.line) + ": AddTTInfos for unknown item " + a.s(1));
            return;
        } else if (cmd == "SETSTARTTTINFOSINDEX") {  // 0x49e290
            if (Item* lb = page->find(a.s(1))) lb->entryIndex = a.i(2);
            return;
        } else if (cmd == "SETSTARTPAGEMENUITEM") {
            if (page) page->startItem = a.s(1);
            return;
        } else {
            return;
        }
        page->items.push_back(std::move(it));
    });
    for (std::string& p : reader.problems) ms.problems.push_back(p);
    if (ms.pages.empty()) ms.problems.push_back("no menu pages (is the data folder right?)");
    return ms;
}

TextTable loadText(const std::string& dataDir, const std::string& language) {
    TextTable t;
    const fs::path loc = fs::path(dataDir) / "LocalizedComponent";
    std::string want = upper(language);
    static const std::map<std::string, std::string> kNames = {
        {"EN", "ENGLISH"}, {"EN-UK", "ENGLISH_UK"}, {"FR", "FRENCH"}, {"ES", "SPANISH"}, {"DE", "GERMAN"}, {"IT", "ITALIAN"}};
    if (auto it = kNames.find(want); it != kNames.end()) want = it->second;
    int tsc = 1;
    TscReader reader(loc.string());
    reader.run("LangDef.tsc", {}, [&](const TscLine& l) {
        // AddLangDefine TT_NUM TSC_ID WAV_ID MPEG_ID CONSOLE_LANG (the language name is last)
        if (upper(l.args[0]) == "ADDLANGDEFINE" && l.args.size() > 5 && upper(l.args.back()) == want) tsc = std::atoi(l.args[2].c_str());
    });
    char name[32];
    std::snprintf(name, sizeof name, "trtext\\tt%02d.pc", tsc);
    if (findPathNoCase(loc.string(), name).empty()) std::snprintf(name, sizeof name, "trtext\\tt01.pc");
    t.file = name;
    TscReader text(loc.string());
    helpTexts().load(dataDir);  // PK26: input.PC's RemapTextAdd records come first (Story/init.tsc LoadINPUT)
    text.run(name, {}, [&](const TscLine& l) {
        if (l.args.size() >= 3 && upper(l.args[0]) == "TT") {
            std::string& v = t.strings[uint32_t(std::atoi(l.args[1].c_str()))] = l.args[2];
            helpTexts().transText(v);  // PK26: TransText 0x677dd0 -> 0x4e2820 ("STR_X" -> "{STR_X}")
        }
    });
    for (const custom::Vehicle& cv : custom::vehicles())  // CUSTOM VEHICLES: the packs' PortText strings (every language)
        for (const auto& [id, text] : cv.texts) t.strings.emplace(id, text);
    return t;
}

std::vector<VehicleInfo> loadVehicleInfos(const std::string& dataDir) {
    std::vector<VehicleInfo> out;
    std::map<std::string, size_t> byMesh;
    TscReader reader((fs::path(dataDir) / "LocalizedComponent").string());
    const auto line = [&](const TscLine& l) {
        const std::string cmd = upper(l.args[0]);
        // AddVehicleInfo MESH_ID_X CLASS SURFACE TIER VEH_X TT ? FAMILY
        if (cmd == "ADDVEHICLEINFO" && l.args.size() > 6) {
            VehicleInfo v;
            v.name = l.args[1].rfind("MESH_ID_", 0) == 0 ? l.args[1].substr(8) : l.args[1];
            v.family = upper(l.args[2]);
            v.surface = upper(l.args[3]);
            v.tier = std::atoi(l.args[4].c_str());
            v.nameText = uint32_t(std::atoi(l.args[6].c_str()));
            v.picture = l.args[5];
            byMesh[l.args[1]] = out.size();
            out.push_back(v);
        } else if (cmd == "ADDVEHICLECOLOR" && l.args.size() > 2) {
            int r = 0, g = 0, b = 0;
            if (auto it = byMesh.find(l.args[1]); it != byMesh.end() && std::sscanf(l.args[2].c_str(), "R%dG%dB%d", &r, &g, &b) == 3)
                out[it->second].colors.push_back({uint8_t(r), uint8_t(g), uint8_t(b)});
        } else if (cmd == "ADDVEHICLEINTERFACEPARAMS" && l.args.size() > 2) {
            if (auto it = byMesh.find(l.args[1]); it != byMesh.end())
                for (size_t k = 2; k < l.args.size(); ++k) out[it->second].bars.push_back(float(std::atof(l.args[k].c_str())));
        }
    };
    reader.run("GameTsc\\Story\\VehiclesDef.tsc", {}, line);
    {  // CUSTOM VEHICLES: the packs' lines after FUEL's (core/custom_vehicles.hpp)
        std::istringstream extra(custom::extraScript("VehiclesDef.tsc"));
        bool block = false;
        for (std::string text; std::getline(extra, text);) {
            TscLine l;
            l.args = splitTscLine(text, &block);
            l.file = "custom";
            if (!l.args.empty()) line(l);
        }
    }
    return out;
}

}  // namespace fuel::ui
