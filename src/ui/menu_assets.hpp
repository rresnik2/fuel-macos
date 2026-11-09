// The menus' pictures and fonts, read from the player's DPC files:
// - images: a script name hashes (Asobo CRC) to the 4-byte link of a MaterialAnim_Z
//   (DATAS/SHARED.DPC "Interface" folder and others); its body + 60 names the Material_Z,
//   whose slot 0 is the Bitmap_Z (FUEL.exe 0x49ba80 / 0x67a6c0).
// - fonts: Fonts_Z in DATAS/FONTES.DPC (glyph table + page materials, 0x6979f0).
// See ~/re-work/fuel/notes/p19-menus.md.
#pragma once

#include "core/dpc.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel::ui {

struct BitmapRef {
    const DpcFile* file = nullptr;
    uint32_t bitmapId = 0;
    uint32_t width = 0, height = 0;
    explicit operator bool() const { return file != nullptr; }
};

struct Glyph {
    uint32_t page = 0;
    float top = 0;               // offset from the line top (texels)
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // texel rectangle
    float scale = 1;             // [port] pad-ux: font texels per page texel (the pad-prompt atlas is drawn finer)
};

struct Font {
    std::string name;
    std::unordered_map<uint32_t, Glyph> glyphs;  // key: UTF-8 bytes as a big-endian number
    std::vector<BitmapRef> pages;
    float refTop = 0, refHeight = 1;  // extent of "É" (FUEL.exe 0x608ca0 scales text by it)
    const Glyph* glyph(uint32_t code) const {
        auto it = glyphs.find(code);
        return it == glyphs.end() ? nullptr : &it->second;
    }
};

class MenuAssets {
public:
    // Opens DATAS/SHARED, FONTES, LMENU and WORLD/MENU; problems go to `problems`. `first`: CommonComponent-relative
    // DPCs searched before those (P78: the world level's loading library DATAS/LWYOMING, Levels.tsc AddMaterialLib USA1,
    // whose LOADING_BITMAP replaces LMENU's).
    // `menuFiles` false: only `first` and the fonts (the loading window opens before anything else is read).
    explicit MenuAssets(const std::string& dataDir, const std::vector<std::string>& first = {}, bool menuFiles = true);

    BitmapRef image(const std::string& name) const;  // empty when unknown
    // 0 = small (SMALL_FONT_PC on PC, else SMALL_FONT), 1 = big (BIG_FONT).
    const Font& font(int index) const { return fonts_[index == 1 ? 1 : 0]; }
    std::vector<std::string> problems;
    size_t padIcons() const { return padIcons_; }  // PK26: SMALL_FONT's pad icons in the small font at 0x101..0x113

private:
    BitmapRef materialBitmap(uint32_t materialId) const;
    std::vector<std::unique_ptr<DpcFile>> files_;
    std::unordered_map<uint32_t, std::pair<const DpcFile*, uint32_t>> anims_;  // name crc -> anim
    Font fonts_[2];
    size_t padIcons_ = 0;  // PK26
};

// Splits UTF-8 text into glyph codes (the bytes of each character as a big-endian number).
std::vector<uint32_t> utf8Codes(const std::string& s);

}  // namespace fuel::ui
