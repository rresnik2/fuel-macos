// STUB (see events_stub.hpp): delete when P17's loadEvents is merged.
#include "ui/events_stub.hpp"

#include "ui/tsc.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace fs = std::filesystem;

namespace fuel::ui {

std::vector<Event> loadEventsStub(const std::string& dataDir) {
    const std::string loc = (fs::path(dataDir) / "LocalizedComponent").string();
    std::map<std::string, Race> races;
    for (const char* file : {"GameTsc\\Story\\miss_official.tsc", "GameTsc\\Story\\missChallenge.tsc"}) {
        std::ifstream f(findPathNoCase(loc, file));
        std::stringstream text;
        text << f.rdbuf();
        for (Race& r : parseMissions(text.str())) races[r.name] = r;
    }
    std::vector<Event> out;
    std::map<std::string, std::string> family, surface;
    std::map<std::string, int> tierOfHub;
    TscReader reader(loc);
    reader.run("GameTsc\\Story\\hubinfos.tsc", {}, [&](const TscLine& l) {
        std::string cmd = l.args[0];
        for (char& c : cmd) c = char(std::toupper(static_cast<unsigned char>(c)));
        if ((cmd == "ADDHUBCAREERRACES" || cmd == "ADDHUBCHALLENGERACES") && l.args.size() > 3) {
            Event e;
            e.kind = cmd == "ADDHUBCAREERRACES" ? Event::Career : Event::Challenge;
            e.hub = l.args[1];
            for (char& c : e.hub) c = char(std::toupper(static_cast<unsigned char>(c)));
            e.id = l.args[2];
            e.nameText = uint32_t(std::atoi(l.args[3].c_str()));
            auto it = races.find(e.id);
            // As main.mm's loadRaces keeps them: a start and a route.
            e.playable = it != races.end() && (it->second.start[0] != 0 || it->second.start[2] != 0) &&
                         (!it->second.checkpoints.empty() || it->second.finish[0] != 0 || it->second.finish[2] != 0);
            if (it != races.end()) e.race = it->second;
            if (!e.playable) e.note = "no start or route decoded";
            out.push_back(e);
        } else if (cmd == "ADDMISSIONNEEDEDVEHICLE" && l.args.size() > 4) {
            // event, specific vehicle (MESH_ID_x or NONE), surface, class (QUAD, MUSCLE_BUGGY, ...)
            surface[l.args[1]] = l.args[3] == "NONE" ? "" : l.args[3];
            if (l.args[2] != "NONE") family[l.args[1]] = l.args[2].rfind("MESH_ID_", 0) == 0 ? l.args[2].substr(8) : l.args[2];
            else family[l.args[1]] = l.args[4] == "NONE" ? "" : l.args[4];
        } else if (cmd == "SETHUBTARGETINFOS" && l.args.size() > 2) {
            std::string h = l.args[1];
            for (char& c : h) c = char(std::toupper(static_cast<unsigned char>(c)));
            tierOfHub[h] = std::atoi(l.args[2].c_str());
        }
    });
    for (Event& e : out) {
        e.vehicleFamily = family[e.id];
        e.surface = surface[e.id];
        e.tier = tierOfHub.count(e.hub) ? tierOfHub[e.hub] : 0;
    }
    return out;
}

}  // namespace fuel::ui
