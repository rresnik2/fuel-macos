// PK20: FUEL's 3-D garage preview (notes/pk20.md): the pause menu's InGame3DVehicle (InGamePauseHUD +0xf0) and
// InGame3DPerso (+0x214) with their InGameObjects_Z scenes, drawn into the garage pages. The page side (which preview
// runs, the delays, the restarts) is the menu's (ui/garage.mm garage3DUpdate); this runs the calls it queues and draws.
// Only made when a garage page opens, so nothing of it runs in a race.
#pragma once

#import <Metal/Metal.h>

#include "ui/menu.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

class GaragePreview {
public:
    // `dpcOf(label)`: the data-relative DPC that holds a vehicle variant (ui::vehicleDpc).
    GaragePreview(id<MTLDevice> device, const std::string& dataDir, std::function<std::string(const std::string&)> dpcOf);
    ~GaragePreview();
    GaragePreview(const GaragePreview&) = delete;
    GaragePreview& operator=(const GaragePreview&) = delete;

    // FUEL's calls into the preview objects, in the order the garage page made them this frame.
    void run(const std::vector<ui::Menu::Garage3DCmd>& cmds);
    // The scenes that are on, each into its rectangle of `target`, over what the target holds: the host draws them after
    // the world (the menus' backdrop) and before the menu pages (the viewport list sorts the scenes, priority +0x14 =
    // 0x17, before the HUDs, 0x666990). The renderer is built on a worker thread; until it is ready nothing is drawn.
    void draw(id<MTLCommandBuffer> cb, id<MTLTexture> target);
    void waitReady();  // --menu-shot: block until the renderer is built
    // What is up (shots / logs): 1 vehicle scene, 2 rider scene (bits).
    int scenesOn() const;
    // ---- [upgrades U3, DEVIATION] the stages the preview vehicle shows (upg::Category order e,s,t,w,np,nt; 0 stock .. 3),
    // from the menu's Garage3DCmd::Upgrades: the shown vehicle's equipped stages (stock when it is not owned or upgrades
    // are off), with the UPGRADES page's browsed stage in its focused category. The menu sends them after every
    // LoadVehicle and whenever they change. U3's stub stores them; U5 (visible upgrade parts) draws them.
    void setUpgradeStages(const std::array<int, 6>& stages);
    const std::array<int, 6>& upgradeStages() const;
    int upgradeStagesSet() const;  // how many times they were set (checks / shots)

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

}  // namespace fuel
