// PK12: the STARTFX records and the sand-storm walls (core/sand_tank.hpp, notes ~/re-work/fuel/notes/pk12.md).
//
// SubCode 0x14 (sand.vhl vh_sandstorm / sand.phl ph_sandstorm): the shaders are written here from their decode, no game
// shader code. SandMesh vf04 0x5ea5f0 per instance (the slot centre, y 0; within 10000 m of the camera): world =
// translation only (0x5e6220 with the identity quaternion 0xa065b8 and scale 0xa06308 = 1), material FX_SANDSTORM01
// (diffuse slot 0, normal slot 3), constants slot 9 = C_USER0..3:
//   C_USER0 = (PP5384, PP5385, +0x8c, 0)   diffuse uv scale / offset
//   C_USER1 = (PP5394, PP5395, 0, +0x90)   normal uv scale / offset
//   C_USER2 = (2.0, 0.3, 0.02, 0)          second normal uv scale, its scroll speed (0x99601c, 0x9dbfa4, 0x9dc018)
//   C_USER3 = (+0x98 alpha, 0, 0, 0)
// state: preset 0x13 (blended queue), sort key +0xd1c = the wall's edge distance +0x94 [I], bits 0x9d (z test, no z
// write, colour + alpha write, blend, alpha test GREATER 0, cull CCW).
#import "renderer.h"
#import "gpu_profile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace fuel {

extern const size_t kFrameUniformsSize;  // renderer.mm
void frameCameraOf(const FrameUniforms& f, simd_float4x4& viewProj, simd_float3& eye, simd_float3& right, simd_float3& up);  // renderer.mm

// Appended after kTornadoShaderSource (uses tornadoViewZ / tornadoSceneZ / tornadoNormal and the scattering helpers).
const char* kSandShaderSource = R"METAL(
struct SandVtx {            // SandStormVertex_Z (0x4c)
    packed_float4 pos;      // POSITION (w 1)
    packed_float3 normal;   // TEXCOORD0 normalU
    float u;
    packed_float3 tangent;  // TEXCOORD1 tangentV
    float v;
    packed_float4 colour;   // TEXCOORD2 diffuse
    packed_float3 binormal; // TEXCOORD3
};

struct SandParams {
    float4 world;  // xyz: the instance position (0x5e6220 translation)
    float4 user0, user1, user2, user3;
    float4 flags;  // x: normal map is DXT5 (bCompression: wyz)
};

struct SandOut {
    float4 position [[position]];
    float3 eyevec;
    float3 T;
    float3 B;
    float3 N;
    float4 uvDN;   // uvDiffuseNormal
    float2 uvN2;
    float4 diffuse;
    float3 ext;
    float3 insc;
    float viewZ;
};

// scatteringbase(worldPos, C_HALF, C_SCATTERCONSTANTS3.xyz): the density x muldensity 0.5.
Scatter sandScattering(constant FrameUniforms& f, float3 p) {
    if (uint(f.fog.x) & 2u) return Scatter{float3(1.0), float3(0.0)};
    float3 look = f.eye.xyz - p;
    const float len = length(look);
    look /= max(len, 1e-6);
    const float cosT = dot(look, f.scat[SC3].xyz);
    const float density = fuelScatterIntegral(f, len, 0.25 + max(p.y, 0.0), f.eye.y) * 0.5;
    return fuelScatterFinish(f, min(look.y, 0.0), density, cosT);
}

vertex SandOut vsSandStorm(uint vid [[vertex_id]], const device SandVtx* vs [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                           constant SandParams& p [[buffer(2)]]) {
    const SandVtx i = vs[vid];
    const float2 uv = float2(i.u, i.v);
    float2 uvD = uv * p.user0.xy + p.user0.zw;
    uvD.y = saturate((1.0 - uv.y) * p.user0.y);
    const float2 uvN = uv * p.user1.xy + p.user1.zw;
    SandOut o;
    o.uvDN = float4(uvD, uvN);
    o.uvN2 = uv * p.user2.xy - p.user1.ww * p.user2.z;
    // normalize() of the float4 inputs (u / v in w), as vh_sandstorm does.
    o.T = -normalize(float4(float3(i.tangent), i.v)).xyz;
    o.N = normalize(float4(float3(i.normal), i.u)).xyz;
    o.B = normalize(float3(i.binormal));
    const float3 w = float4(i.pos).xyz + p.world.xyz;
    o.position = fuelClip(f, w);
    o.eyevec = normalize(f.eye.xyz - w);
    const Scatter sc = sandScattering(f, w);
    o.ext = sc.ext;
    o.insc = sc.insc;
    o.diffuse = float4(float4(i.colour).rgb, p.user3.x);
    o.viewZ = tornadoViewZ(f, w);
    return o;
}

fragment float4 fsSandStorm(SandOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tDiffuse [[texture(1)]],
                            texture2d<float> tNormal [[texture(2)]], constant FrameUniforms& f [[buffer(1)]],
                            constant SandParams& p [[buffer(2)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    const float4 tD = tDiffuse.sample(smp, in.uvDN.xy);
    if (p.flags.y > 0.5) {  // FUEL_SAND_DEBUG=1: coverage (r = texture alpha, g = facing alpha, b = 1)
        const float d0 = dot(in.N, in.eyevec);
        return float4(tD.a * 4.0, (d0 > 0.0 ? saturate(pow(d0, 1.5) * 5.0) : 0.0) * 4.0, 4.0, 1.0);
    }
    const bool dxt5 = p.flags.x > 0.5;
    const float3 t = tornadoNormal(tNormal, smp, in.uvDN.zw, dxt5) * 0.5 + tornadoNormal(tNormal, smp, in.uvN2, dxt5) * 0.5;
    const float3 n = normalize(t.x * in.T + t.y * in.B + t.z * in.N);
    // alpha = saturate(pow(dot(axe2, eyevec), 1.5) x 5) x diffuse.a (pow of a negative: NaN, saturated to 0 [I]).
    const float d = dot(in.N, in.eyevec);
    const float a = (d > 0.0 ? saturate(pow(d, 1.5) * 5.0) : 0.0) * in.diffuse.a;
    // lightingsandstorm: sun from (0, -1, 0) with k = 0.8, sky along gDSkyDir with xz negated, no specular / shadow / omni.
    const float sun = saturate(dot(n, float3(0.0, -1.0, 0.0)) * 0.8 + 0.2);
    float skydot;
    const float sky = fuelSkylighting(n, float3(-f.skyDir.x, f.skyDir.y, -f.skyDir.z), skydot);
    float3 c = in.diffuse.rgb * tD.rgb * (sky * f.skyColour.rgb + sun * f.sunColour.rgb);
    c = c * in.ext + in.insc;
    const float alpha = tD.a * a;
    if (!(alpha > 0.0)) discard_fragment();  // ALPHATEST GREATER, ref 0
    return float4(max(c, 0.0), saturate(alpha));
}
)METAL";

void MeshRenderer::makeSandPipeline() {
    MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
    d.vertexFunction = [shaders_ newFunctionWithName:@"vsSandStorm"];
    d.fragmentFunction = [shaders_ newFunctionWithName:@"fsSandStorm"];
    d.colorAttachments[0].pixelFormat = kRenderColorFormat;
    d.colorAttachments[0].blendingEnabled = YES;  // SRCALPHA / INVSRCALPHA, the destination alpha kept (as the tornado)
    d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
    d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
    NSError* e = nil;
    sandPipeline_ = [device_ newRenderPipelineStateWithDescriptor:d error:&e];
    if (sandPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
}

void MeshRenderer::startFxFire(const sand::Fire& f) {
    std::lock_guard<std::mutex> lk(sandMu_);
    sandQueue_.push_back({0, f, {}});
}

void MeshRenderer::startFxRelease(uint32_t node) {
    std::lock_guard<std::mutex> lk(sandMu_);
    sand::Fire f;
    f.node = node;
    sandQueue_.push_back({1, f, {}});
}

void MeshRenderer::setStartFxHidden(std::unordered_map<uint32_t, bool> hidden) {
    std::lock_guard<std::mutex> lk(sandMu_);
    sandHidden_ = std::move(hidden);
}

void MeshRenderer::startFxDropHidden(std::vector<uint32_t> nodes) {
    std::lock_guard<std::mutex> lk(sandMu_);
    sandQueue_.push_back({2, sand::Fire{}, std::move(nodes)});
}

void MeshRenderer::setSandGroundColour(std::function<std::array<float, 4>(float, float, float)> c) {
    std::lock_guard<std::mutex> lk(sandMu_);
    sandColour_ = std::move(c);
}

// WeatherManager vf17 (0x5bc3b0 after 0x5bba30) then the FX manager's tank / mesh update (0x5f62c0), per game frame:
// the port steps them in 1/60 s steps of the frame's effect time (FUEL's frame count, not only its time, drives the
// one-point-per-frame ground refresh). Frozen while paused (PK17: ManipulatorMgr skips both).
void MeshRenderer::updateSandFx(float dt) {
    if (!weatherFields_ || std::getenv("FUEL_NO_SANDSTORM")) return;
    std::lock_guard<std::mutex> lk(sandMu_);
    if (!startFx_) {
        std::vector<float> pp(5400, 0.0f);
        for (int n = 5380; n < 5400; ++n) pp[size_t(n)] = weatherFields_->P(n);
        sandParams_ = sand::Params::fromPP(pp);
        startFx_ = std::make_unique<sand::StartFx>(sandParams_);
    }
    auto hidden = [this](uint32_t node) {
        const auto it = sandHidden_.find(node);
        return it != sandHidden_.end() && it->second;
    };
    static const bool trace = std::getenv("FUEL_SAND_TRACE") != nullptr;
    for (const SandOp& op : sandQueue_) {
        const int kind = op.kind;
        const sand::Fire& f = op.fire;
        if (kind == 0) startFx_->fire(f);
        else if (kind == 1) startFx_->release(f.node);
        else startFx_->dropHidden([&op](uint32_t n) { return std::find(op.nodes.begin(), op.nodes.end(), n) != op.nodes.end(); });
        if (trace)
            std::printf("pk12 startfx %s node %08x sand %d time %s%.0f -> %zu records, %d walls\n", kind == 0 ? "fire" : kind == 1 ? "release" : "drop hidden",
                        f.node, int(f.sand), f.hasTime ? "" : "(default) ", f.hasTime ? f.time : 120.0f, startFx_->records().size(), startFx_->tank().count());
    }
    sandQueue_.clear();
    if (pauseSky_ || !lastFrame_) return;
    if (startFx_->records().empty() && startFx_->tank().slots() == 0) return;
    simd_float4x4 vp;
    simd_float3 eye, right, up;
    frameCameraOf(*lastFrame_, vp, eye, right, up);
    sand::Ground g;
    if (terrainData_.width > 0) {
        g.loaded = true;
        // 0x63eb10 + 0x63ea40: the 1 m lattice point nearest below-left of (x + 0.5, z + 0.5) (as GroundGenerator::at).
        g.height = [this](float x, float z) { return terrainData_.smoothHeightAt(std::floor(double(x) + 0.5), std::floor(double(z) + 0.5)); };
        // The point colour is costly (the full GenWorld point, ~0.3 ms); FUEL quantises the point too (0x63e4d0's key).
        // Cached per 1 m lattice point: state 1 samples every ring point at the slot centre (the points are still 0
        // there), so a new wall costs one evaluation instead of n.
        if (sandColour_)
            g.colour = [this](float x, float y, float z) {
                const int64_t ix = int64_t(std::floor(double(x) + 0.5)), iz = int64_t(std::floor(double(z) + 0.5));
                const uint64_t key = (uint64_t(uint32_t(int32_t(ix))) << 32) | uint32_t(int32_t(iz));
                if (const auto it = sandColourCache_.find(key); it != sandColourCache_.end()) return it->second;
                if (sandColourCache_.size() > 4096) sandColourCache_.clear();
                return sandColourCache_[key] = sandColour_(float(ix), y, float(iz));
            };
    }
    sandAcc_ = std::min(sandAcc_ + double(dt), 2.0);  // films render every N frames: up to 120 steps
    int steps = 0;
    while (sandAcc_ >= 1.0 / 60.0 - 1e-9) {
        sandAcc_ -= 1.0 / 60.0;
        startFx_->step(1.0f / 60.0f, hidden);
        startFx_->tank().update(1.0f / 60.0f, {eye.x, eye.y, eye.z}, g);
        ++steps;
    }
    sand::Tank& t = startFx_->tank();
    if (t.fogSet) sandFog_ = t.fog, sandFogSet_ = true, t.fogSet = false;  // weather +0x120 / +0x11c (read by 0x5bba30)
    if (trace && steps > 0 && frame_ % 30 == 0)
        for (size_t k = 0; k < t.slots(); ++k)
            if (const sand::Mesh* m = t.mesh(int(k)); m && !m->instances.empty() && !m->vertices().empty()) {
                const auto& v = m->vertices();
                const int R = m->rings();
                std::printf("pk12 wall %zu: at (%.0f, %.0f) scale (%.0f %.0f %.0f) edge %.0f alpha %.2f; col 0 bottom (%.0f %.0f %.0f) top (%.0f %.0f %.0f) "
                            "col 15 bottom (%.0f %.0f %.0f) n (%.2f %.2f %.2f) colour (%.2f %.2f %.2f)\n", k, m->instances[0][0], m->instances[0][2],
                            m->matrix[12], m->matrix[13], m->matrix[14], m->edge, m->alpha, v[0].pos[0], v[0].pos[1], v[0].pos[2], v[size_t(R - 1)].pos[0],
                            v[size_t(R - 1)].pos[1], v[size_t(R - 1)].pos[2], v[size_t(15 * R)].pos[0], v[size_t(15 * R)].pos[1], v[size_t(15 * R)].pos[2],
                            v[size_t(15 * R + 3)].normal[0], v[size_t(15 * R + 3)].normal[1], v[size_t(15 * R + 3)].normal[2], v[size_t(15 * R + 4)].colour[0],
                            v[size_t(15 * R + 4)].colour[1], v[size_t(15 * R + 4)].colour[2]);
            }
    if (trace && steps > 0 && frame_ % 30 == 0)
        std::printf("pk12 sand: %zu records, %d walls, camera edge %.1f m%s, alpha %.2f, fog floor %.2f\n", startFx_->records().size(), t.count(),
                    t.minEdge, t.inside ? " (inside)" : "", t.slots() && t.mesh(0) ? t.mesh(0)->alpha : 0.0f, t.fog);
}

void MeshRenderer::encodeSandWalls(id<MTLCommandBuffer> cb) {
    if (!startFx_ || !lastFrame_ || sandPipeline_ == nil || sandTex_[0] == nil) return;
    std::lock_guard<std::mutex> lk(sandMu_);
    const sand::Tank& t = startFx_->tank();
    if (t.count() == 0) return;
    simd_float4x4 vp;
    simd_float3 eye, right, up;
    frameCameraOf(*lastFrame_, vp, eye, right, up);
    struct Item {
        const sand::Mesh* m;
        simd_float3 pos;
    };
    std::vector<Item> items;
    for (size_t s = 0; s < t.slots(); ++s)
        if (const sand::Mesh* m = t.mesh(int(s)))
            for (const sand::V3& p : m->instances) {
                const simd_float3 w = simd_make_float3(p[0], p[1], p[2]);
                if (simd_distance(w, eye) <= 10000.0f) items.push_back({m, w});  // 0xa2a4e0
            }
    if (items.empty()) return;
    // The blended queue sorts by +0xd1c (the wall's edge distance, 10000 inside) [I]: far first.
    std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.m->edge > b.m->edge; });
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.sand");
    [enc setRenderPipelineState:sandPipeline_];
    [enc setVertexBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:sandTex_[0] atIndex:1];
    [enc setFragmentTexture:(sandTex_[1] ? sandTex_[1] : sandTex_[0]) atIndex:2];
    [enc setFrontFacingWinding:MTLWindingClockwise];  // D3D CULL_CCW (bit 0x80), as the scene pass
    static const char* cullEnv = std::getenv("FUEL_SAND_CULL");  // diagnostics: none | front | back (default)
    [enc setCullMode:cullEnv && !strcmp(cullEnv, "none") ? MTLCullModeNone : cullEnv && !strcmp(cullEnv, "front") ? MTLCullModeFront : MTLCullModeBack];
    const sand::Params& sp = sandParams_;
    static const bool sandDebug = std::getenv("FUEL_SAND_DEBUG") != nullptr;
    for (const Item& it : items) {
        const sand::Mesh& m = *it.m;
        const auto& vs = m.vertices();
        if (vs.empty()) continue;
        if (sandIdxRings_ != m.rings() || sandIdxCols_ != m.columns() || sandIdx_ == nil) {
            const std::vector<uint16_t> ix = sand::Mesh::indices(m.rings(), m.columns());
            sandIdx_ = ix.empty() ? nil : [device_ newBufferWithBytes:ix.data() length:ix.size() * sizeof(uint16_t) options:MTLResourceStorageModeShared];
            sandIdxCount_ = ix.size(), sandIdxRings_ = m.rings(), sandIdxCols_ = m.columns();
        }
        if (sandIdx_ == nil) continue;
        struct Params {
            simd_float4 world, user0, user1, user2, user3, flags;
        } p{simd_make_float4(it.pos, 1), simd_make_float4(sp.uvDiffuse[0], sp.uvDiffuse[1], m.scroll0, 0),
            simd_make_float4(sp.uvNormal[0], sp.uvNormal[1], 0, m.scroll1), simd_make_float4(2.0f, 0.3f, 0.02f, 0),
            simd_make_float4(m.alpha, 0, 0, 0), simd_make_float4(sandNormalBC3_ ? 1.0f : 0.0f, sandDebug ? 1.0f : 0.0f, 0, 0)};
        id<MTLBuffer> vb = [device_ newBufferWithBytes:vs.data() length:vs.size() * sizeof(sand::Vertex) options:MTLResourceStorageModeShared];
        [enc setVertexBuffer:vb offset:0 atIndex:0];
        [enc setVertexBytes:&p length:sizeof p atIndex:2];
        [enc setFragmentBytes:&p length:sizeof p atIndex:2];
        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:sandIdxCount_ indexType:MTLIndexTypeUInt16 indexBuffer:sandIdx_ indexBufferOffset:0];
    }
    [enc setFrontFacingWinding:MTLWindingCounterClockwise];  // back to the shared pass's defaults
    [enc setCullMode:MTLCullModeNone];
}

const sand::StartFx* MeshRenderer::startFx() const { return startFx_.get(); }

}  // namespace fuel
