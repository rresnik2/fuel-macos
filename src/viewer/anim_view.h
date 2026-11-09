// P47: FUEL's animated world objects and cut-scenes in the viewer (notes/p47-input-anim.md §2-3).
//  - WorldAnimation: every USA1 Lod_Z whose object is a Skin_Z with an f1 slot-0 Animation_Z loops slot 0 on one
//    clock per class (BaseSharingPlayAnimAgent_G 0x434a20: one PlayAnim_Z per class, so every flag of a class shows
//    the same pose; IT_ANIMATED idles on slot 0 the same way). The skinned vertices replace the class's Mesh_Z
//    in the renderer, so every placement of it is drawn animated.
//  - RtcView: plays an RTC DPC (CommonComponent/RTC/<name>.DPC): RtcPlayer_Z time and events, the binds' world
//    matrices under the RTC's father, the RTC-only objects (F: meshes, skinned Lods with their own loop) drawn
//    through MeshRenderer::setRtcInstances, and the RTC camera after the first 0x30 cut.
#pragma once

#include "audio/sound_mixer.hpp"
#include "core/anim_clip.hpp"
#include "core/world_anim.hpp"
#include "core/particles.hpp"
#include "core/rtc.hpp"
#include "core/rtc_fade.hpp"
#include "core/skinned_model.hpp"
#include "renderer.h"

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fuel {
class DriveAudio;
}

// PK29: an animated skin instance's material clones (Skel_Z vf18 0x66c0e0 clones the skeleton's Material_Z list; the
// skin's batches draw with them by material name) and PlayAnim_Z's material tracks on them (0x813e50).
class MaterialClones {
public:
    void init(const fuel::SkinnedModel& m, const std::vector<const fuel::DpcFile*>& files);
    // 0x813e50 for every track of `clip` at t: the sub-tracks present overwrite their fields, then vt+0x44 =
    // Material_Z::UpdateUVMatrix 0x695ee0. The clones keep their values when no track writes them.
    void apply(const fuel::anim::Clip& clip, double t);
    // Hands the changed clones to the renderer (part p's baked mesh = key(p), 0 = none); returns how many.
    size_t push(fuel::MeshRenderer& r, const std::function<uint32_t(size_t part)>& key);
    size_t size() const { return slots_.size(); }

private:
    struct Slot {
        size_t part = 0, slot = 0;
        uint32_t name = 0;
        fuel::Material cur;
        bool dirty = false;
    };
    std::vector<Slot> slots_;
};

class WorldAnimation {
public:
    // files: where the world resolves IDs (USA1 and the sub-worlds); registers the baked meshes. Returns the classes.
    size_t init(const std::vector<const fuel::DpcFile*>& files, const fuel::DpcFile& usa1, fuel::MeshRenderer& r);
    // PK29: one PlayAnim_Z channel per class (0x7bc150 / 0x7bc3d0: loop markers) and its 0x7bc5c0 passes: events
    // 0x24 (sound) and 0x20 (IT_ANIMATED: pooled FX on the bone), material tracks (0x813e50) and visibility tracks
    // (0x7a6ce0 / 0x7a6de0).
    void advance(float dt);
    // P55: IT_ANIMATED events for the class of Lod `lod`: start = 0x518170 (once until reset: play slot 334 one-shot, no
    // blend-in with anim flag 2, then hold the last pose), reset = 0x517f80 (slot 0 loop again, re-armed). FUEL does this
    // per object; the port's skins are per class (one clock, as BaseSharingPlayAnimAgent_G / IT_ANIMATED idle).
    bool start(uint32_t lod);
    void reset(uint32_t lod);
    // Skins the classes whose clock moved and hands the vertices to the renderer. P92: with the camera of the next
    // frame, only the classes some object can show (MeshRenderer::bakedMeshDrawable); the rest are skinned when they
    // come into range (a pose depends only on the clip and its clock).
    void upload(fuel::MeshRenderer& r, const fuel::OrbitCamera* cam = nullptr);

    // PK29: where a class's objects stand (WorldStreamer::placementsOf), for its event sounds and FX.
    using PlacementFn = std::function<void(uint32_t lod, std::vector<std::array<float, 16>>& out)>;
    void setPlacements(PlacementFn f) { placements_ = std::move(f); }
    void setAudio(fuel::DriveAudio* a) { audio_ = a; }
    // PK29: the event FX (GameParticleMgr pool, 0x4ee870 from IT_ANIMATED message 0x20), stepped in advance() near
    // `eye` and appended as engine particle quads.
    void setEye(const fuel::particles::V3& eye) { eye_ = eye; }
    bool hasFx() const { return !fx_.empty(); }
    void appendFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches) const;
    // For the checks.
    struct Stats {
        size_t sounds = 0, fxStarts = 0, hides = 0, shows = 0, materialWrites = 0;
    };
    const Stats& stats() const { return stats_; }

private:
    struct Class {
        uint32_t lod = 0;
        fuel::SkinnedModel model;
        const fuel::anim::Clip* clip = nullptr;
        const fuel::anim::Clip* idle = nullptr;   // slot 0
        const fuel::anim::Clip* event = nullptr;  // slot 334 (STARTANIMATED / tornado), may be null
        bool started = false;                     // IT_ANIMATED +0x74
        double t = 0;
        std::vector<std::vector<uint8_t>> verts;
        fuel::anim::AnimChannel ch;   // PK29: the PlayAnim_Z channel
        std::vector<uint32_t> partName;   // PK29: the parts' mesh names (the instance groups of 0x66ce90)
        std::vector<uint32_t> partFlags;  // PK29: group +0xc: 2 hidden (4 locked: never set on world objects)
        std::vector<int> trackBone;       // PK29: the clip's tracks -> instance bones (-1: none; only tracks with events)
        MaterialClones mats;              // PK29: 0x813e50's targets
        bool upload = true;
    };
    std::vector<std::unique_ptr<Class>> classes_;
    // PK29
    void play(Class& c, const fuel::anim::Clip* clip);
    void passes(Class& c, const std::vector<fuel::anim::AnimPass>& ps);
    void setGroup(Class& c, uint32_t name, bool show);
    void sound(Class& c, int bone, const fuel::anim::Clip::Msg& m, double t);
    void startFx(Class& c, int bone, uint32_t data, double t);
    void stepFx(float dt);
    PlacementFn placements_;
    fuel::DriveAudio* audio_ = nullptr;
    fuel::MeshRenderer* renderer_ = nullptr;  // PK29: the material clones' values (setBakedMaterial)
    std::vector<const fuel::DpcFile*> files_;
    std::map<uint32_t, std::shared_ptr<const fuel::snd::Pcm>> samples_;
    std::map<uint32_t, std::shared_ptr<const fuel::particles::Emitter>> emitters_;  // by data key (link word 0)
    struct Fx {  // a GameParticleMgr slot (0x18 B: object, instance, bone, particle, kind, age) and its emitter
        uint32_t lod = 0;
        size_t placement = 0;
        int bone = -1;
        uint32_t data = 0;
        bool keyed = true;  // false once 0x4ef150 released it (age > 3 x duration): the particles live out
        float age = 0;
        std::array<float, 16> place{};
        std::shared_ptr<const fuel::particles::Emitter> emitter;
        fuel::particles::Instance inst;
        float fade = 0;
    };
    std::vector<Fx> fx_;
    fuel::particles::V3 eye_{};
    uint32_t rng_ = 0x51ed270bu;
    Stats stats_;
    std::vector<std::array<float, 16>> placeTmp_;
};

class RtcView {
public:
    // FUEL's AddRTC flags (Story/Levels.tsc, parser 0x54d3b0): SKIPABLE 0x20, SKIP_BEFORE_END 0x80, ON_GROUND 0x100.
    bool play(const std::string& dataDir, const std::string& name, const fuel::anim::Mat4& father, int flags,
              fuel::ResourceLibrary& library, fuel::MeshRenderer& r);
    void stop(fuel::MeshRenderer& r);
    bool active() const { return player_ != nullptr; }
    int flags() const { return flags_; }  // PK17: the Play record flags (2 = freeze player, 0x534b40)
    const std::string& name() const { return name_; }
    float time() const { return player_ ? player_->time() : 0; }
    void setFather(const fuel::anim::Mat4& f) { father_ = f; }
    // P96: the player vehicle's MESH_ID name ("MESH_ID_QUAD_ROAD_2"): 0x103 'AMEO' hooks the helicopter's cables to
    // the RTC binds "<MESH_ID>_1..4" (profile 0x559ec0 +0x154 -> game+0x3258 entry +0x30).
    void setVehicleMesh(const std::string& meshId) { vehicleMesh_ = meshId; }
    // The skip of 0x53c150: SKIPABLE and 0.25 < t < dur - 1: SKIP_BEFORE_END seeks to dur - 1 + 0.01, else stops.
    void skip(fuel::MeshRenderer& r);
    void releaseLoop() {
        if (player_) player_->releaseLoop();
    }
    // PK13: the player vehicle's pose now (row-vector, rigid): 0x5148a0 latches it when an 0xf1 attach starts easing.
    void setAttachedPose(const fuel::anim::Mat4& m) { attachedNow_ = m; }
    // PK13: the node FX (0x539140: TELEPORT_HELICO's rotor dust on A306) as engine particle quads, faded as placed
    // emitters (0x80c540); appended to `quads` / `batches`.
    void appendFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches, const fuel::particles::V3& eye) const;
    bool hasFx() const { return !fx_.empty(); }

    struct Frame {
        bool camera = false;          // the RTC holds the camera (after its first 0x30 cut)
        fuel::OrbitCamera cam;
        int respawns = 0;             // 0x10b events this frame (the vehicle code respawns, 0x52f420)
        bool hudHidden = false;       // 0x10c / 0x10d
        bool shake = false;           // STARTRTCFX SHAKECAM (0xa3 kind 1): start the game camera's scripted shake
        // 0xf1 / 0xf2 (node events, 0x5389d0 -> 0x53bfc0 -> 0x514b70): the player vehicle is attached to the node
        // (TELEPORT_HELICO: under the helicopter from 0 s to the end); `attach` = the node's world (row-vector).
        // PK13: the slot's pose (0x5148a0: eased from the vehicle over the blend time, then the node's position and
        // rotation, no scale).
        bool attached = false;
        fuel::anim::Mat4 attach = fuel::anim::kIdentity;
        bool ended = false;
        // P76: 0x10e pause hold (0xa65878 "pause all animation"): FUEL skips the mission update (race clock), the
        // player's input (PlayerMove 0x42dc40), traffic and special effects while it is set.
        bool pauseGame = false;
        // PK13: 0x539a50 (a running RTC's player holds 0x10e): LoadingDraw vf18 0x502710 draws "LOADING..." (0x503ce0)
        // and the vehicles' sound update 0x5d8ed0 is skipped (their voices keep their last settings). Same as pauseGame.
        bool loading = false;
        // PK13: the frame of Play with record flag 2 (0x53b6b0): every viewport's radial-blur flag 0x8000 cleared.
        bool clearRadialBlur = false;
    };
    // P67: the fader takes the 0x98 fades (0x536d40), `audio` (may be null) the 0x24 / 0x25 sounds (0x729de0).
    Frame update(float dt, fuel::MeshRenderer& r, fuel::rtc::Fader* fader = nullptr, fuel::DriveAudio* audio = nullptr);

private:
    struct Obj {
        int e = 0;
        uint32_t mesh = 0;                           // a Mesh_Z
        std::unique_ptr<fuel::SkinnedModel> skin;    // or a skinned Lod
        const fuel::anim::Clip* clip = nullptr;
        std::vector<uint32_t> keys;                  // baked mesh keys of the skin's parts
        double t = 0;
        MaterialClones mats;                         // PK29: the clip's material tracks (TELEPORT_HELICO's beacon)
    };
    std::string name_;
    fuel::DpcFile* dpc_ = nullptr;  // owned by the library
    fuel::rtc::Rtc rtc_;
    std::unique_ptr<fuel::rtc::Player> player_;
    std::vector<Obj> objs_;
    fuel::anim::Mat4 father_ = fuel::anim::kIdentity;
    int flags_ = 0, cam_ = -1, attachNode_ = -1;
    fuel::rtc::AttachSlot slot_;  // PK13: 0x514b70 / 0x5148a0
    fuel::anim::Mat4 attachedNow_ = fuel::anim::kIdentity;
    bool played_ = false;         // PK13: Play's pre-start (0x53b6b0) not reported yet
    struct NodeFx {               // PK13: 0x539140 cache entry (0x18 B): event node, data id, the emitter instance
        int node = -1;
        uint32_t data = 0;
        std::shared_ptr<const fuel::particles::Emitter> emitter;
        fuel::particles::Instance inst;
        bool shown = false;
    };
    std::vector<NodeFx> fx_;
    uint32_t fxRng_ = 0x13579bdfu;
    void nodeFx(const fuel::rtc::Fired& e);
    std::vector<const fuel::DpcFile*> fxFiles_;
    bool hudHidden_ = false;
    fuel::rtc::PauseHold hold_;     // P76: 0x10e
    std::vector<char> hiddenBind_;  // P76: 0x10f (binds hidden with their children)
    // P67: events of a skip (0x72aeb0 seek, skipping) handled on the next update; live sounds (0x24) by Sound_Z id,
    // with the node they follow (-1: 2D); decoded samples by id.
    std::vector<fuel::rtc::Fired> pending_;
    struct Voice {
        uint32_t id = 0;
        int node = -1;
        int handle = -1;
    };
    std::vector<Voice> voices_;
    std::map<uint32_t, std::shared_ptr<const fuel::snd::Pcm>> samples_;
    fuel::DriveAudio* audio_ = nullptr;
    void stopSounds();
    // P96: 0x103 'AMEO' records (player +0x624, 16 x {event node, joint, target node}, 0x53bf80): the object (helicopter
    // skin on the event node), its joint bone, the target bind.
    struct Cable {
        size_t obj = 0;
        int joint = -1;
        size_t target = 0;
    };
    std::vector<Cable> cables_;
    std::string vehicleMesh_;
    fuel::rtc::BoneRegistry registry_;  // ids registered before the RTC (the loaded skeletons' name lists)
    void ameo(const fuel::rtc::Fired& e);
};

// P67: what FUEL shows around its cut-scenes (notes/p67-cutscenes.md): the screen fader (BaseInGameDatas_G at
// game+0x3310, fed by Mission SetState 3 and the RTC 0x98 events), the scripted camera shake (STARTRTCFX SHAKECAM,
// 0x4156e0) and the race-end grade (MissionHUD 0x4b6050, pp001 PP39..43).
class CutsceneFx {
public:
    void load(const std::string& dataDir);  // pp001.tsc rows
    fuel::rtc::Fader fader;
    fuel::rtc::CameraShake shake;
    fuel::rtc::EndGrade endGrade;
    // Mission SetState 3 STREAMING (0x632c10, offline): 0x4382e0 holds the screen black; TELEPORT_HELICO's
    // FROMBLACK (0x98 kind 1 at 0 s) lifts it.
    void missionStreaming() { fader.holdBlack(); }
    void missionEnded() { endGrade.dir = 1; }  // SetState 6/7: MissionHUD +0xd38 = 1
    void missionLeft() { endGrade.dir = -1; }  // SetState 0: +0xd38 = -1
    // Once per frame after the camera is chosen: advances the fader, shake and grade; writes the renderer's fade,
    // tint and (under an RTC camera) radial blur, and the sound fade.
    void frame(float dt, fuel::OrbitCamera& view, bool rtcCamera, fuel::MeshRenderer& r, fuel::DriveAudio* audio);
    float rtcBlur() const { return pp_.size() > 42 ? pp_[42] * endGrade.ramp : 0.0f; }  // viewport +0x1dc under an RTC camera
    float pp(int n) const { return n >= 0 && size_t(n) < pp_.size() ? pp_[size_t(n)] : 0.0f; }  // PK17: pp001.tsc PP n (n < 64)

private:
    std::vector<float> pp_;
};

// Row-vector (D3DX) matrix -> simd (column vectors).
simd_float4x4 simdFromRows(const fuel::anim::Mat4& m);
// P67: moves a vehicle's part matrices rigidly so that its body frame `chassis` lands on `node` (both row-vector).
void attachParts(std::vector<simd_float4x4>& parts, const fuel::anim::Mat4& chassis, const fuel::anim::Mat4& node);
