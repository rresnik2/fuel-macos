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
    // Keeps text inside [x, x+w] by shrinking the scale when needed.
    void textFit(const std::string& s, float x, float y, float w, TextStyle st);

    // Draws everything queued since begin() into `target` (clearing it first when asked).
    void encode(id<MTLCommandBuffer> cb, id<MTLTexture> target, bool clear);

    id<MTLTexture> texture(const BitmapRef& b);

private:
    struct Vertex {
        float x, y, u, v;
        float r, g, b, a;
    };
    struct Batch {
        id<MTLTexture> tex;
        size_t first, count;
    };
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
