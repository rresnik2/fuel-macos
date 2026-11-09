// TAA [port performance option, not a FUEL behaviour]: the arithmetic of the MetalFX temporal path (taa_render.mm,
// renderer.mm taaFrameUniforms), shared with --rendercheck. Notes: ~/re-work/fuel/notes/taa.md.
//
// The scene's clip position is fuelClip: clip = viewProj (w - eye, 0) + (0, 0, zn q, 0) (camera-relative, P60), with
// clip.w = the view distance. The temporal jitter moves the image by (px, py) render pixels (x right, y down):
// clip.x += 2 px / W clip.w, clip.y -= 2 py / H clip.w, i.e. row 0 += jx row 3 and row 1 += jy row 3 of viewProj.
// Motion vectors are in uv of the render area (x right, y down), from where a point is this frame (without the
// jitter) to where it was in the last frame: MetalFX's convention with motionVectorScale = the render size in pixels.
#pragma once

#include <simd/simd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace fuel::taa {

// Radical inverse of i in base b (Halton): base 2 gives 1/2, 1/4, 3/4, 1/8, ...
inline double halton(uint32_t i, uint32_t b) {
    double f = 1.0, r = 0.0;
    while (i > 0) {
        f /= double(b);
        r += f * double(i % b);
        i /= b;
    }
    return r;
}

// The jitter of temporal frame n in render pixels, each axis in [-0.5, 0.5): Halton (2, 3) from index 1, kPhases long.
// 32 phases cover the 8 x (output / render)^2 Apple suggests for a 2x upscale.
constexpr uint32_t kPhases = 32;
inline simd_float2 jitter(uint32_t n) {
    const uint32_t i = n % kPhases + 1;
    return simd_make_float2(float(halton(i, 2) - 0.5), float(halton(i, 3) - 0.5));
}

// viewProj with the jitter (render pixels px, py on a W x H render area).
inline simd_float4x4 jittered(simd_float4x4 m, simd_float2 px, float w, float h) {
    const float jx = 2.0f * px.x / w, jy = -2.0f * px.y / h;
    for (int c = 0; c < 4; ++c) {
        m.columns[c].x += jx * m.columns[c].w;
        m.columns[c].y += jy * m.columns[c].w;
    }
    return m;
}

// A 4 x 4 in double (column-major like simd).
struct Mat4d {
    double m[4][4] = {};  // m[column][row]
    static Mat4d identity() {
        Mat4d r;
        for (int i = 0; i < 4; ++i) r.m[i][i] = 1.0;
        return r;
    }
    Mat4d operator*(const Mat4d& b) const {
        Mat4d r;
        for (int c = 0; c < 4; ++c)
            for (int i = 0; i < 4; ++i) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k) s += m[k][i] * b.m[c][k];
                r.m[c][i] = s;
            }
        return r;
    }
    simd_float4x4 toFloat() const {
        simd_float4x4 r;
        for (int c = 0; c < 4; ++c) r.columns[c] = simd_make_float4(float(m[c][0]), float(m[c][1]), float(m[c][2]), float(m[c][3]));
        return r;
    }
    // Gauss-Jordan with partial pivoting; false when singular.
    bool inverse(Mat4d& out) const {
        double a[4][8];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) a[r][c] = m[c][r], a[r][c + 4] = r == c ? 1.0 : 0.0;
        for (int c = 0; c < 4; ++c) {
            int p = c;
            for (int r = c + 1; r < 4; ++r)
                if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
            if (std::fabs(a[p][c]) < 1e-300) return false;
            if (p != c)
                for (int k = 0; k < 8; ++k) std::swap(a[p][k], a[c][k]);
            const double d = a[c][c];
            for (int k = 0; k < 8; ++k) a[c][k] /= d;
            for (int r = 0; r < 4; ++r) {
                if (r == c) continue;
                const double f = a[r][c];
                if (f == 0.0) continue;
                for (int k = 0; k < 8; ++k) a[r][k] -= f * a[c][k];
            }
        }
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) out.m[c][r] = a[r][c + 4];
        return true;
    }
};

// fuelClip as one 4 x 4 on (rel, 1), rel = w - eye: the linear part of viewProj and the constant (0, 0, zn q, 0).
inline Mat4d clipOfRelative(const simd_float4x4& vp, float zn, float q) {
    Mat4d r;
    for (int c = 0; c < 3; ++c)
        for (int i = 0; i < 4; ++i) r.m[c][i] = double(vp.columns[c][i]);
    r.m[3][2] = double(zn) * double(q);
    return r;
}

// The camera reprojection of a frame: for a pixel's (ndc x, ndc y, depth, 1) of this frame's jittered projection,
// `reproj` gives the static point's clip position in the last frame (unjittered) and `unjit` its clip position in this
// frame without the jitter. motion = (reproj h).xy / w - (unjit h).xy / w, times (0.5, -0.5).
struct Reprojection {
    simd_float4x4 reproj, unjit;
    bool ok = false;
};
inline Reprojection reprojection(const simd_float4x4& jitteredVp, const simd_float4x4& vp, const simd_float4x4& prevVp, float zn, float q,
                                 simd_float3 eye, simd_float3 prevEye) {
    Reprojection out;
    Mat4d inv;
    if (!clipOfRelative(jitteredVp, zn, q).inverse(inv)) return out;
    Mat4d shift = Mat4d::identity();  // rel_prev = w - prevEye = rel + (eye - prevEye)
    shift.m[3][0] = double(eye.x) - double(prevEye.x);
    shift.m[3][1] = double(eye.y) - double(prevEye.y);
    shift.m[3][2] = double(eye.z) - double(prevEye.z);
    out.reproj = (clipOfRelative(prevVp, zn, q) * shift * inv).toFloat();
    out.unjit = (clipOfRelative(vp, zn, q) * inv).toFloat();
    out.ok = true;
    return out;
}

// Texture LOD bias of a temporal frame (Apple's guidance for temporal upscalers: sample the mip the output resolution
// would pick): FUEL's own bias (-1, PK28) plus log2(render scale); a native frame keeps FUEL's.
inline float lodBias(float fuelBias, float renderScale) {
    return renderScale < 1.0f ? fuelBias + std::log2(std::max(renderScale, 1e-3f)) : fuelBias;
}

}  // namespace fuel::taa
