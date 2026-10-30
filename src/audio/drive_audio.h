// Driving sounds for fuelview --drive: FUEL's own engine, tyre, wind and
// impact samples, decoded from the vehicle archive and mixed with AVAudioEngine.
#pragma once

#include "core/dpc.hpp"
#include "core/sound.hpp"
#include "physics/vehicle_physics.hpp"

#include <array>
#include <map>
#include <utility>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

// Every sample one vehicle needs, decoded once when the vehicle is chosen.
// A missing sample is empty (`samples.empty()`).
struct VehicleSounds {
    std::string variant;
    EngineModel engine;
    std::array<Sound, EngineModel::kLayers> idleOrOff;  // [0] idle, [1..5] off-load layers
    std::array<Sound, EngineModel::kLayers> on;         // [1..5] on-load layers ([0] unused)
    Sound rollSlow, rollFast, skid, wind;
    std::vector<Sound> impacts;                         // one-shots, in bank order
    int surfaceClass = -1;                              // the rolling/skid class used (see drive_audio.mm)
};

// Decodes the sound set for `variant`. Throws std::runtime_error when the
// vehicle has no bank or no engine samples.
VehicleSounds loadVehicleSounds(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params);

class DriveAudio {
public:
    DriveAudio();
    ~DriveAudio();
    DriveAudio(const DriveAudio&) = delete;
    DriveAudio& operator=(const DriveAudio&) = delete;

    // Replaces the sound set (decodes and builds the audio graph; not for the frame loop).
    // Prints a message and stays silent if the vehicle's sounds cannot be built.
    void setVehicle(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params);
    // Call once per frame.
    void update(float dt, const VehicleState& state, float throttle);
    void toggleMute();
    bool muted() const;
    void setMasterVolume(float v);  // 0..1

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Headless: builds the sound sets of the listed variants and prints their
// layers, weights and pitches. Returns the number of failures.
int audioCheck(const std::vector<std::pair<const DpcFile*, std::string>>& variants, const std::map<std::string, std::vector<float>>& params);

}  // namespace fuel
