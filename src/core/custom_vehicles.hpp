// CUSTOM VEHICLES (port deviation, user-requested): extra drivable vehicles that are not part of FUEL.
//
// Each vehicle is a "pack" folder (never in the repo): ~/re-work/fuel/custom-assets/vehicles/<pack>/ while building,
// FUEL.app/Contents/Resources/vehicles/<pack>/ in the app (CMake copies the packs that are complete). A pack holds
//   vehicle.tsc   FUEL's own script syntax: the vehicle's VehiclesDef.tsc lines (AddVehicleInfo, AddVehicleParams,
//                 AddVehicleGPS, AddVehicleUnlock, AddVehicleInterfaceParams, AddVehicleColor), its VehiclesUnlock.tsc
//                 line (UnlockNeed VEHICLE ...), its VehiclesParam.tsc block (VehiclePP 0 "MESH_ID_<V>" .. EndOfVehiclePP)
//                 and the port's own lines:
//                   PortVehicle <V> <file.DPC>        the variant name and its archive (in the pack folder)
//                   PortVehicleSound <DONOR> <DPC>    a FUEL variant whose engine bank it uses (DPC relative to DATAS)
//                   PortVehicleAI 0|1                 whether opponents may drive it (default 0)
//                   PortVehicleSynth EV               electric: the donor bank's engine samples are replaced by a
//                                                     synthesized motor whine (audio/ev_whine.hpp; vehicles4)
//                   PortText <id> "<text>"            strings for its texts (name, description); ids >= 990000
//                   PortCredit <text>                 a line of the credits roll as written (licence attribution)
//   <file>.DPC    the vehicle in FUEL's own archive format (built by tools/fuelvehpack from a Blender export)
//   CREDITS.txt   the asset licence text (for people; the game reads PortCredit)
// The scripts' lines are appended after FUEL's own (the AddVehicleInfo order = FUEL's vehicle table index stays the same
// for FUEL's vehicles). A pack whose DPC is missing is skipped: the vehicle does not exist then.
#pragma once

#include <string>
#include <vector>

namespace fuel::custom {

struct Vehicle {
    std::string name;   // variant label ("VOLVO_V60_T8"); mesh id = "MESH_ID_" + name
    std::string dir;    // the pack folder
    std::string dpc;    // absolute path of its archive
    std::string soundDonor, soundDpc;  // FUEL variant + DPC (relative to CommonComponent/DATAS) for the engine bank
    std::string family, surface;  // AddVehicleInfo's class ("MUSCLE") and surface ("ROAD")
    bool ai = false;    // PortVehicleAI
    std::string synth;  // PortVehicleSynth ("EV": audio/ev_whine.hpp)
    std::vector<std::string> credits;   // PortCredit lines
    std::vector<std::pair<unsigned, std::string>> texts;  // PortText
    std::string defs, unlocks, params;  // VehiclesDef / VehiclesUnlock / VehiclesParam lines
};

// Where the packs are: $FUEL_CUSTOM_VEHICLES (a folder of packs; "off" = none), else FUEL.app/Contents/Resources/vehicles
// next to the running executable (also build/FUEL.app/... for the tools in build/). Empty = none.
std::string root();
// The packs found (scanned once; complete packs only, sorted by folder name).
const std::vector<Vehicle>& vehicles();
// By variant label or "MESH_ID_<label>" (case-insensitive); nullptr when it is not a custom vehicle.
const Vehicle* find(const std::string& nameOrMesh);
inline bool isCustom(const std::string& nameOrMesh) { return find(nameOrMesh) != nullptr; }

// The text to append to one of FUEL's scripts, by its file name (case-insensitive): "VehiclesDef.tsc",
// "VehiclesUnlock.tsc", "VehiclesParam.tsc"; empty for any other file.
std::string extraScript(const std::string& fileName);
// Reads `path` and appends extraScript(its file name) (the readers of those three scripts use this).
std::string readScript(const std::string& path);

// The game's data folder (the one that holds CommonComponent/): set once by the program (fuelview, tools). Code that
// finds FUEL.exe or Param/ from a vehicle archive's path uses anchor(): for a custom archive it is a path of the same
// name under <data>/CommonComponent/DATAS/, else the path itself.
void setDataDir(const std::string& dir);
const std::string& dataDir();
std::string anchor(const std::string& dpcPath);

// The credits roll's port section (empty when there are no packs): a heading and each pack's lines.
std::vector<std::string> creditLines();

}  // namespace fuel::custom
