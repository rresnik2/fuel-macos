// FUEL's in-game menu pages as the scripts define them: CreatePage .. SetStartPageMenuItem
// blocks of AddMenuItem* commands in GameTsc/Interface/InterfacesInfosPacked.tsc (the file
// the game sources at level load, FUEL.exe 0x559931), and the localized text tables
// (trtext/ttNN.pc). Argument layouts are the ones the command handlers in FUEL.exe read;
// see ~/re-work/fuel/notes/p19-menus.md. Nothing here is copied from the game: it is all
// parsed from the player's files at run time.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel::ui {

struct Rgba {
    float r = 1, g = 1, b = 1, a = 1;
};
// "A:R:G:B" with 0..255 components (FUEL.exe 0x49bba0); `ok` false when malformed.
Rgba parseArgb(const std::string& s, bool* ok = nullptr);

enum class ItemKind { Image, FillRect, LabelTT, LabelDyn, ButtonTT, ButtonDyn, ListBoxTT };
const char* itemKindName(ItemKind k);

struct ListEntry {
    std::string key;
    uint32_t tt = 0;
    bool flag = false;
};

struct Item {
    ItemKind kind = ItemKind::Image;
    std::string id, group;
    float x = 0, y = 0, w = 0, h = 0;  // 1280 x 720 canvas
    int a7 = 0, a8 = 0;                // +0x34 / +0x38 (meaning unknown)
    float f9 = 0;                      // text items: +0xc8 (unknown, 358 on left-menu buttons)
    // Images: the picture per state; colours per state (images and boxes: tint;
    // text items: outline). State order as drawn: focus, else enabled, else normal.
    std::string imgNormal, imgEnabled, imgFocus;
    Rgba cNormal, cEnabled, cFocus;
    Rgba tNormal, tEnabled, tFocus;  // text items: text colour per state
    uint32_t tt = 0;                 // text id (TT items; Dyn items' default)
    bool enabled = false, visible = true, flagCC = false;
    int font = 0;                    // 0 small, 1 big
    int align = 0;                   // 0 left, 1 centre, 2 right (+0xc4)
    float xOffset = 0;               // +0xb8 (canvas units)
    bool slant = false;              // +0xe4 = -0.2
    bool upper = false;              // +0xcd: text drawn upper-case
    bool b19 = false;                // +0xce (unknown)
    int anchorH = 0, anchorV = 0;
    std::vector<ListEntry> entries;  // list boxes
    int entryIndex = 0;

    // Run-time state, set by the menu code.
    bool focus = false;
    std::string dynText;   // text of Dyn items (and overrides for others when non-empty)
    bool hasDynText = false;

    bool isText() const { return kind != ItemKind::Image && kind != ItemKind::FillRect; }
    bool isButton() const { return kind == ItemKind::ButtonTT || kind == ItemKind::ButtonDyn || kind == ItemKind::ListBoxTT; }
};

struct Page {
    std::string name;
    std::vector<Item> items;
    std::string startItem;
    Item* find(const std::string& id);
    const Item* find(const std::string& id) const;
};

struct MenuScripts {
    std::vector<Page> pages;
    std::map<std::string, size_t> index;  // upper-case name -> pages
    std::vector<std::string> problems;
    Page* page(const std::string& name);
    const Page* page(const std::string& name) const;
};

// Reads every page (LocalizedComponent/GameTsc/Interface/InterfacesInfosPacked.tsc).
MenuScripts loadMenuScripts(const std::string& dataDir);

struct TextTable {
    std::unordered_map<uint32_t, std::string> strings;
    std::string file;
    const std::string* get(uint32_t id) const {
        auto it = strings.find(id);
        return it == strings.end() ? nullptr : &it->second;
    }
};

// Language code (en, fr, de, es, it, en-uk) -> trtext table number, from LangDef.tsc
// (TSC_ID column); falls back to the English table.
TextTable loadText(const std::string& dataDir, const std::string& language);

// The vehicles from GameTsc/Story/VehiclesDef.tsc (AddVehicleInfo).
struct VehicleInfo {
    std::string name;    // e.g. "QUAD_ROAD_2" (the variant label --assemble takes)
    std::string family;  // e.g. "QUAD" (class)
    std::string surface; // ROAD, OFF, ALL
    int tier = 0;
    uint32_t nameText = 0;
    std::vector<float> bars;  // AddVehicleInterfaceParams: performance bars 0..1
};
std::vector<VehicleInfo> loadVehicleInfos(const std::string& dataDir);

}  // namespace fuel::ui
