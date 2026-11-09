// FUEL's vehicle headlights (notes p77-shadows-lights.md 2): one Omni_Z per vehicle variant, instanced by the
// `V:HEADLIGHT:n "<group>" "<V>_HEADLIGHT"` bone command (Skel_Z::BuildVehicleLinks 0x69e020) into the skeleton's
// light list (+0x150), placed at the bone of the `V:LINK "<bone>" ... LIGHT:n` command; switched by 0x5cbde0 (on while
// the day factor 0x5b98d0 < 1) and drawn by the deferred omni pass (Omni_Z::vf43 0x798050, omni.phl) into the light
// buffer that every outdoor shader adds to its sun / sky light (GetDeferredLights).
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace fuel::omni {

using V3 = std::array<float, 3>;

// The Omni_Z fields vf43 reads (body offset b = object offset - 0x6c).
struct Params {
    V3 colour{};          // +0xfc..+0x104: colour (+0xec) x intensity (+0xf8) (gDOmniColor)
    float inner = 0;      // +0x124: r (gDOmniValue.x = (r s)^2)
    float outer = 0;      // +0x128: R (gDOmniValue.y = 1 / ((R s)^2 - (r s)^2))
    float cutZ = 0;       // +0x11c (gDOmniValue.z)
    float cutW = 0;       // +0x120 (gDOmniValue.w)
};
// From an Omni_Z body (0xc0+ bytes); false when it is too short.
bool parseParams(const std::vector<uint8_t>& body, Params& out);

// ---- PK28: the Omni_Z object fields the light gathers read (link: +0x60 flags = link byte 92, +0xc bound sphere = link
// bytes 8..23) and the spot angles of the record builder (+0x114 inner, +0x118 outer, radians).
struct ObjectInfo {
    uint32_t flags = 0;   // +0x60: 0x400000 = gathered by 0x7fd670 (RTC 0xc6 / 0xc7 switch it), 0x20000000 = spot
                          // (only spots reach the deferred pass 0x7fdd80), 0x8000000 / 0x10000000 = RTC switchable / not
    float bound = 0;      // +0xc .. +0x18: bound sphere radius (centre (0,0,0) in every shipped Omni_Z)
    float inner = 0, outer = 0;  // +0x114 / +0x118
};
// From the Omni_Z link (98 bytes) and body; false when either is too short.
bool parseObjectInfo(const std::vector<uint8_t>& link, const std::vector<uint8_t>& body, ObjectInfo& out);

inline constexpr uint32_t kOmniGathered = 0x400000, kOmniSpot = 0x20000000, kOmniRtcSwitch = 0x8000000, kOmniRtcKeep = 0x10000000;
inline constexpr int kMaxLightRecords = 64;  // the viewport lists +0x164 / +0x3178 (0x6a2ed0, 0x7fd670)

// 0x7984c0: one 0x68-byte light record (world-space fields; the view-space copies of [0..10] when view +0x11c are
// the same vectors in the camera's axes). Inputs: node world translation T (+0xd4 +0x30), node quaternion q as a
// rotation of (0, 0, -1) already applied (axis = q x (0,0,-1) [0xa0635c]), node scale s (+0x44).
struct Record {
    V3 pos{};            // +0x38 world translation (w 1 at +0x44 from the matrix)
    V3 axis{};           // +0x48: q x (0, 0, -1), w 1 (+0x54)
    V3 dir{};            // +0x20: spot: the axis; omni: the constant 0xa66208 (bss, 0 at load)
    float range2 = 0;    // +0x58: (+0x124 x s)^2
    float invSin = 0;    // +0x2c: spot 1 / sin(+0x118); omni 0
    float sin2 = 0;      // +0x30: sin^2(+0x118)
    float cos2 = 0;      // +0x34: cos^2(+0x118)
    float radius = 0;    // +0x0c: the bound sphere radius x s (0x821830's out w)
    bool spot = false;
};
Record makeRecord(const ObjectInfo& o, const Params& p, const V3& translation, const V3& axis, float scale);

// 0x821830 (sphere vs view): centre c and radius r in view space (+z forward); planes = the four side planes of the
// view as inward unit normals (camera +0x1c0..+0x1ec, through the eye); near / far camera +0x6bc / +0x6c0.
// Visible when z >= near - r, z <= far + r and dot(n_i, c) >= -r for every side plane.
bool sphereVisible(const V3& c, float r, const std::array<V3, 4>& planes, float zNear, float zFar);

// 0x6a2ed0 / 0x7fd670: append to a 64-record list; when it is full the last slot (63) is overwritten.
template <class T>
void appendRecord(std::vector<T>& list, const T& r) {
    if (int(list.size()) < kMaxLightRecords)
        list.push_back(r);
    else
        list[size_t(kMaxLightRecords - 1)] = r;
}

// RtcScriptAgent_G 0x536d40 messages 0xc6 (off) / 0xc7 (on) over every Omni_Z of the world (0x7aabe0): the gather flag
// 0x400000 of each Omni_Z with 0x8000000 and without 0x10000000. Returns the new flags.
uint32_t rtcSwitch(uint32_t flags, bool on);
// ---- PK28 end

// gDOmniValue for node scale s (node +0x44).
std::array<float, 4> value(const Params& p, float s = 1.0f);

// 0x5b98d0: 0 before 05:30, ramp to 1 at 06:30, 1 until 17:30, ramp to 0 at 18:30 (t in seconds of the day).
float dayFactor(double t);
// 0x5cbde0: the lights are switched off when the day factor is 1 and on when it is below 1.
inline bool headlightsOn(double timeOfDay) { return dayFactor(timeOfDay) < 1.0f; }

// omni.phl for one light: P the lit point, N its normal, p the light, dir = gDOmniDir (the record's direction; the
// port passes minus the light bone's z axis, see the notes), viewUp the view's y axis (v.y is a view-space value).
V3 term(const V3& P, const V3& N, const V3& p, const V3& dir, const std::array<float, 4>& value, const V3& colour, const V3& viewUp);

}  // namespace fuel::omni
