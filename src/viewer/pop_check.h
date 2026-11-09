// P94: fuelview --pop-check: a headless pop detector. It drives the --perf-check race film (CR2-Yell-CP by default,
// chase camera, cut-scenes, streaming) without rendering, and every frame asks the renderer what it would draw
// (MeshRenderer::popProbe): each world object's level mesh and fade alpha, each plant's mesh and card. Between two
// frames an object in view, at least --pop-pixels tall on a 3420x2224 frame, "pops" when
//   - it appears at fade alpha >= 0.25 (from nothing), or vanishes from >= 0.25;
//   - its alpha jumps by >= 0.25 in one frame;
//   - it switches mesh while drawn: by FUEL's Lod_Z rule (Lod_Z::Draw 0x884170 picks one level, no blend: FUEL
//     pops these too, so they are counted apart), or by another path (a placed object becoming a prop, a plant
//     changing from mesh to card).
// FUEL fades everything else (ComputeFade 0x672950 over the last 20 m; the decor fades of ground_rotshape.vhl), so
// every pop but the Lod_Z level switches is the port's: the check fails on any. Camera cuts (the eye moving more
// than 5 m in a frame) start the comparison afresh.
//
//   fuelview --data DIR --pop-check [--race NAME] [--drive-time S] [--pop-pixels N] [--pop-csv FILE]
#pragma once

#include <memory>
#include <string>

#include "viewer/renderer.h"

namespace fuel {

class PopCheck {
public:
    struct Options {
        int width = 3420, height = 2224;  // the frame the pixel sizes are measured on (the player's fullscreen size)
        float minPixels = 3.0f;           // smaller objects do not count
        std::string csv;                  // every pop: t, kind, event, distance, pixels, alpha before / after
        std::string label;
    };
    explicit PopCheck(const Options& o);
    ~PopCheck();
    void frame(const MeshRenderer& renderer, const OrbitCamera& cam, double t);
    // PK15: a frame the teleport's logo covers (nothing of the world shows): no comparison, and the next frame starts
    // afresh as after a camera cut.
    void covered();
    // PK15: the pops counted so far that FUEL would not have (the teleport case reports those after its reveal).
    long portPops() const;
    // Prints the report; true when the port popped nothing FUEL would not.
    bool finish();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fuel
