// FUEL's GPS samples and in-world guidance (notes/p40-work/gps.md):
//  - samples (MapHUD_UpdateGPS 0x4a2250): the next 30 points, 10 m apart, along the player's race path; more than
//    10 m off the path, a straight line back to it comes first;
//  - the floating trail (Navigator3dHUD 0x4bc0a0): the samples smoothed toward the previous frame, lifted 7.5 m,
//    leaving the player in the camera direction and curving onto the route 35 m ahead;
//  - 16 ribbon arrows (FX_GPS_ARROW, emitter 0x4bbba0, per frame 0x4bca50) flowing along the trail at 40 m/s,
//    looping every 300 m, faded near the player and the trail's ends; each a 4-point rope (2 m links) banked toward
//    the player, drawn by the renderer in the 3-D scene (P86: SubCode 0x1e vh_billboard / ph_billboard, 0x5fba20
//    strip: half width 2.25 m, u 0 .. 0.25 of FX_GPS_ARROW from the head, i.e. only the red chevron).
// Port choices: no road detour (0x6d8e20 with cost 0x4a1de0) and no rejoin search (0x4a1f30) when off the path,
// heights from the terrain only (FUEL also casts rays for bridges).
#pragma once

#include <array>
#include <functional>
#include <vector>

namespace fuel::hud {

class GpsGuide {
public:
    using V3 = std::array<float, 3>;
    struct Ribbon {
        std::vector<V3> points;  // head first, then the tail 2 m apart
        float alpha = 0;         // particle +0x70 (Fade_One x it = the vertex alpha +0x64)
        V3 up{0, 1, 0};          // +0xbc, the bank the strip's sides are built across (0x5fba20)
    };
    // path: the race path (x, z) with its cumulative length; ground(x, z) = terrain height.
    // camera: eye and forward (world). Clears everything when the path is empty.
    void update(float dt, const V3& player, const V3& eye, const V3& camForward, const std::vector<std::array<float, 2>>& path,
                const std::vector<float>& along, const std::function<float(float, float)>& ground);
    void reset();
    const std::vector<V3>& samples() const { return samples_; }
    const std::vector<V3>& trail() const { return trail_; }
    const std::vector<Ribbon>& ribbons() const { return ribbons_; }

    // Pieces (for --hudcheck)
    static float fade(float x, float L);  // 0x4bca50: 0 below 10 m, ramp to 1 at 40 m, 1 to L - 50, ramp to 0 at L
    static float advance(float d, float dt, bool* wrapped = nullptr);  // d + 40 dt, wrapped into (.., 300]

private:
    std::vector<V3> samples_, trail_, prevA_;
    std::vector<float> trailAlong_;
    std::vector<float> d_;  // ribbon path distances
    std::vector<std::array<V3, 4>> tails_;  // each ribbon's 4 points (+0xb8)
    std::vector<Ribbon> ribbons_;
    V3 at(float s) const;   // point `s` metres along the trail (0x4bf170: linear in the xz distance)
};

}  // namespace fuel::hud
