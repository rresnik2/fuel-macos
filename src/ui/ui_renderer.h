// Metal 2-D renderer for FUEL's menu pages: textured and plain quads on the game's
// 1280 x 720 canvas, letterboxed into the target the way FUEL.exe 0x49bec0 fits it
// (16:9, full width, centred vertically), and text in the game's own fonts.
#pragma once

#import <Metal/Metal.h>

#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"

#include <unordered_map>
#include <vector>

namespace fuel::ui {

struct TextStyle {
    int font = 0;
    float height = 22;     // item height (canvas units): the text scale is height / height("É")
    Rgba fill, outline;
    int align = 0;         // 0 left, 1 centre, 2 right inside [x, x + width]
    float slant = 0;       // shear (x shift per unit of height, negative leans right)
    bool upper = false;
};

struct UiVertex {
    float x, y, u, v;  // canvas units
    Rgba c;
};

class UiRenderer {
public:
    UiRenderer(id<MTLDevice> device, MTLPixelFormat target, const MenuAssets& assets);

    // Starts a frame for a target of w x h pixels.
    void begin(float w, float h);
    // Canvas rectangle -> target pixels.
    float sx(float x) const { return offX_ + x * scale_; }
    float sy(float y) const { return offY_ + y * scaleY_; }
    float canvasScale() const { return scale_; }

    void rect(float x, float y, float w, float h, const Rgba& c);
    void image(const BitmapRef& b, float x, float y, float w, float h, const Rgba& c, float u0 = 0, float v0 = 0, float u1 = 1, float v1 = 1);
    void text(const std::string& s, float x, float y, float w, const TextStyle& st);
    float textWidth(const std::string& s, const TextStyle& st) const;  // canvas units
    // A picture turned by `deg` (clockwise on screen) about the canvas point (px, py), which sits at
    // (pivotU, pivotV) of the w x h picture (FUEL.exe 0x60ae00 / 0x60b490).
    void sprite(const BitmapRef& b, float px, float py, float w, float h, float pivotU, float pivotV, float deg, const Rgba& c);
    // P78: the same with a texture made outside the DPCs (pictures FUEL.exe packs itself, e.g. the save icon).
    void sprite(id<MTLTexture> t, float px, float py, float w, float h, float pivotU, float pivotV, float deg, const Rgba& c);
    id<MTLDevice> device() const { return device_; }
    // P78: the whole target in canvas units (y of its top edge and its height; the canvas is 0..720 of it).
    float viewTop() const { return -offY_ / scaleY_; }
    float viewHeight() const { return targetH_ / scaleY_; }
    float viewLeft() const { return -offX_ / scale_; }
    float viewWidth() const { return targetW_ / scale_; }
    // Triangle list in canvas units (null = untextured).
    void triangles(const BitmapRef* b, const std::vector<UiVertex>& v);
    // Following draws stay inside the canvas rectangle and, when discR > 0, inside the disc (minimap).
    void setClip(float x, float y, float w, float h, float discX = 0, float discY = 0, float discR = 0);
    void clearClip();
    // P78: FUEL's rich-text engine (TextGameDraw 0x43c960 line breaks, 0x43ce60 layout, 0x43d3c0 draw) inside the box
    // [x0, x1] x [y0, y1]: lines break at a space (not one before '!' or '?'), at '|' and at '~' (forced), wrapping when
    // `wrap`; '^ddd' colours, '§NNN' scales the rest by NNN/100 (a line is as tall as its largest scale), 'µL/C/R' sets the
    // alignment of the lines after it; each line placed by `align` (0 left, 1 centre, 2 right; st.align is ignored).
    // `shrink`: scale x 0.95 until the lines fit the box height (0x43ce60). Returns the height used (canvas units).
    float richText(const std::string& s, float x0, float y0, float x1, float y1, int align, const TextStyle& st, bool wrap = true, bool shrink = false);
    // Keeps text inside [x, x+w] by shrinking the scale when needed.
    void textFit(const std::string& s, float x, float y, float w, TextStyle st);

    // Draws everything queued since begin() into `target` (clearing it first when asked).
    void encode(id<MTLCommandBuffer> cb, id<MTLTexture> target, bool clear);

    id<MTLTexture> texture(const BitmapRef& b);

    // Multiplies the alpha of everything drawn while set (P75: FUEL's page alpha argument of 0x60fe10, e.g. the
    // end-race pages fading in with MissionHUD +0xd3c).
    float alpha = 1;

    // PK18: the text rotation of 0x615240 -> 0x6090e0 (script F9, degrees; 358 on the left-menu labels): the glyphs of the
    // following text() calls turn about the point at fractions (fx, fy) of the text's own box (0x6090e0 builds the offset
    // from the alignment factors a7 / 2, a8 / 2 and the text extent); positive = clockwise on screen (as sprite()) [I].
    void setTextRotation(float deg, float fx, float fy) { rotDeg_ = deg, rotFx_ = fx, rotFy_ = fy; }
    void clearTextRotation() { rotDeg_ = 0; }
    // PK26: while a pad is present (PaddleChecker_G +0x24) the {STR_*} labels' glyph bytes 1..19 draw the pad icons
    // (the small font's 0x101..0x113, MenuAssets); else the key caps of SMALL_FONT_PC. Process-wide, set each frame.
    // [port] pad-ux: the pad icons are the connected pad's own (ui/pad_glyphs.*): the {STR_*} labels carry pad-prompt
    // characters (U+E000 + icon) and glyph bytes 1..19 map to the same icons while padIcons is on.
    static bool padIcons;
    static uint32_t glyphCode(uint32_t c) {
        if (c == 0xc28a) c = 10;  // ui/help_texts.cpp glyphByte: the help-text glyph 10 (not a line break)
        return padIcons && c >= 1 && c < 0x20 ? 0x100 + c : c;
    }
    // The glyph of character code `c` in `f`: a pad prompt from the atlas (sized and placed like the font's old pad
    // icons), else the font's own glyph (glyphCode). Null when neither has it.
    const Glyph* glyphAt(const Font& f, uint32_t c) const;
    static constexpr uint32_t kPadPage = 0xffffffffu;  // Glyph::page of an atlas glyph

private:
    // pad-ux: the atlas glyphs per font, for the atlas serial they were made for.
    mutable std::unordered_map<const Font*, std::vector<Glyph>> padGlyphs_;
    mutable int padSerial_ = -2;
    mutable id<MTLTexture> padTex_ = nil;
    mutable float padW_ = 1, padH_ = 1;
    // The texture of a glyph and its texel size (1 / width, 1 / height); nil when it has none.
    id<MTLTexture> glyphTexture(const Font& f, const Glyph& g, float& iw, float& ih);
    float rotDeg_ = 0, rotFx_ = 0, rotFy_ = 0;
    // the pivot of the current text() call in target pixels while a rotation is set
    float rotPx_ = 0, rotPy_ = 0;
    struct Vertex {
        float x, y, u, v;
        float r, g, b, a;
    };
    struct Clip {
        bool on = false;
        float x = 0, y = 0, w = 0, h = 0;  // target pixels
        float disc[4] = {0, 0, 0, 0};
    };
    struct Batch {
        id<MTLTexture> tex;
        size_t first, count;
        Clip clip;
    };
    void batchFor(id<MTLTexture> tex);
    static bool sameClip(const Clip& a, const Clip& b);
    Clip clip_;
    void quad(id<MTLTexture> tex, float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const Rgba& c, float shear = 0);
    std::string displayText(const std::string& s, bool upper) const;

    id<MTLDevice> device_;
    id<MTLRenderPipelineState> pipeline_;
    id<MTLSamplerState> sampler_;
    id<MTLTexture> white_;
    const MenuAssets& assets_;
    std::unordered_map<uint64_t, id<MTLTexture>> textures_;
    std::vector<Vertex> verts_;
    std::vector<Batch> batches_;
    float targetW_ = 1280, targetH_ = 720, offX_ = 0, offY_ = 0, scale_ = 1, scaleY_ = 1;
};

}  // namespace fuel::ui
