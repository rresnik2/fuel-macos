// [DEVIATION, the user's request 2026-11-06] Vehicle upgrades in the career (package U2): buying and equipping stages per
// vehicle, the gates and the session's stages. FUEL has no upgrades; the economy follows FUEL's vehicle purchase (0x4ce590).
// Plan: ~/re-work/fuel/notes/upgrades-plan.md (3, 6, 7); this package: notes/upg2.md. The save section is in career.cpp
// (PORT_UPGRADES), the stage tables and prices in core/upgrades.hpp (upgrade_rules.cpp).
// [DEVIATION dev mode] (notes devmode.md; career.hpp Career::setDevMode): while dev mode is on, every stage of a classed vehicle
// reads as owned (so upgradeLock / buyUpgrade answer AlreadyOwned and nothing is spent) and equipping writes the dev store
// (CareerState::dev, PORT_DEVMODE), never state_.upgrades.
#include "career.hpp"

#include <algorithm>
#include <iterator>

namespace fuel {

namespace {
constexpr uint32_t kFuelCap = 99999999;  // FUEL's fuel cap (game +0x408, 0x630dd0 / 0x630ca0 / 0x44e020) [O]
bool goodArgs(int category, int stage) { return category >= 0 && category < upg::kCategories && stage >= 0 && stage <= upg::kMaxStage; }
}  // namespace

const char* Career::upgradeResultName(UpgradeResult r) {
    switch (r) {
        case UpgradeResult::Bought: return "bought";
        case UpgradeResult::AlreadyOwned: return "already owned";
        case UpgradeResult::VehicleNotOwned: return "vehicle not owned";
        case UpgradeResult::NeedsStars: return "needs more stars";
        case UpgradeResult::TooExpensive: return "too expensive";
        case UpgradeResult::NotUpgradable: return "not upgradable";
    }
    return "?";
}

bool Career::upgradable(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 && rules_.vehicles[size_t(i)].cls >= 0;
}

int Career::vehicleLevel(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 ? rules_.vehicles[size_t(i)].level : -1;
}

CareerState::VehicleUpgrades* Career::upgradesAt(const std::string& v) {
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return nullptr;
    if (state_.upgrades.size() < state_.vehicles.size()) state_.upgrades.resize(state_.vehicles.size());
    return size_t(i) < state_.upgrades.size() ? &state_.upgrades[size_t(i)] : nullptr;
}

upg::Stages Career::upgradesOwned(const std::string& v) const {
    upg::Stages st;
    const int i = rules_.vehicleIndex(v);
    if (devOn(i)) {  // [dev mode] every stage owned (no star gate, no fuel); the real stages stay in state_.upgrades
        for (int c = 0; c < upg::kCategories; ++c) st[c] = upg::kMaxStage;
        return st;
    }
    if (i >= 0 && size_t(i) < state_.upgrades.size())
        for (int c = 0; c < upg::kCategories; ++c) st[c] = state_.upgrades[size_t(i)].owned[c];
    return st;
}

upg::Stages Career::upgradesEquipped(const std::string& v) const {
    upg::Stages st;
    const int i = rules_.vehicleIndex(v);
    if (const CareerState::DevVehicle* d = devAt(i); d && devOn(i) && (d->flags & CareerState::kDevStages)) {
        for (int c = 0; c < upg::kCategories; ++c) st[c] = std::min<int>(d->equipped[c], upg::kMaxStage);  // [dev mode] the dev store's
        return st;
    }
    if (i >= 0 && size_t(i) < state_.upgrades.size())
        for (int c = 0; c < upg::kCategories; ++c) st[c] = state_.upgrades[size_t(i)].equipped[c];
    return st;
}

uint32_t Career::upgradesSpent(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 && size_t(i) < state_.upgrades.size() ? state_.upgrades[size_t(i)].spent : 0;
}

int Career::upgradePrice(const std::string& v, int category, int stage) const {
    if (!upgradable(v) || !goodArgs(category, stage) || stage < 1) return -1;
    const int have = upgradesOwned(v)[category];
    int sum = 0;
    for (int k = have + 1; k <= stage; ++k) sum += upg::price(vehicleLevel(v), category, k);  // plan 3.4: the missing ones too
    return sum;
}

Career::UpgradeResult Career::upgradeLock(const std::string& v, int category, int stage) const {
    if (!upgradable(v) || !goodArgs(category, stage) || stage < 1) return UpgradeResult::NotUpgradable;
    if (!vehicleOwned(v)) return UpgradeResult::VehicleNotOwned;              // stage 1's gate (0x47c050's rule)
    if (upgradesOwned(v)[category] >= stage) return UpgradeResult::AlreadyOwned;  // equip it instead
    if (stars() < upg::starsNeeded(stage)) return UpgradeResult::NeedsStars;  // 20 / 45 stars (0x578b50)
    if (uint32_t(upgradePrice(v, category, stage)) > state_.fuel) return UpgradeResult::TooExpensive;
    return UpgradeResult::Bought;
}

Career::UpgradeResult Career::buyUpgrade(const std::string& v, int category, int stage) {
    const UpgradeResult r = upgradeLock(v, category, stage);
    if (r != UpgradeResult::Bought) return r;
    const uint32_t price = uint32_t(upgradePrice(v, category, stage));
    CareerState::VehicleUpgrades* u = upgradesAt(v);
    if (!u || !spendFuel(price)) return UpgradeResult::TooExpensive;  // FUEL's spend path (0x4ce590)
    u->owned[category] = uint8_t(stage);     // the stages below come with it
    u->equipped[category] = uint8_t(stage);  // and it is fitted at once (plan 4.4: "Yes buys, equips, saves")
    u->spent = uint32_t(std::min<uint64_t>(uint64_t(u->spent) + price, kFuelCap));
    return UpgradeResult::Bought;
}

bool Career::equipUpgrade(const std::string& v, int category, int stage) {
    if (!upgradable(v) || !goodArgs(category, stage) || !vehicleOwned(v)) return false;
    if (const int i = rules_.vehicleIndex(v); devOn(i)) {
        // [dev mode] any stage, free, into the dev store (it starts from the real equipped stages); the real ones stay
        CareerState::DevVehicle* d = devAt(i);
        if (!d) return false;
        if (!(d->flags & CareerState::kDevStages)) {
            std::fill(std::begin(d->equipped), std::end(d->equipped), uint8_t(0));
            if (size_t(i) < state_.upgrades.size()) std::copy(std::begin(state_.upgrades[size_t(i)].equipped), std::end(state_.upgrades[size_t(i)].equipped), d->equipped);
        }
        d->flags |= CareerState::kDevStages;
        d->equipped[category] = uint8_t(stage);
        return true;
    }
    CareerState::VehicleUpgrades* u = upgradesAt(v);
    if (!u || stage > u->owned[category]) return false;
    u->equipped[category] = uint8_t(stage);  // free, any owned stage (stock included): no refund, no resale
    return true;
}

upg::Stages Career::sessionUpgrades(const std::string& vehicle, bool upgradesOn, bool handedOver) const {
    if (!upgradesOn || handedOver || !upgradable(vehicle) || !vehicleOwned(vehicle)) return {};
    return upgradesEquipped(vehicle);
}

}  // namespace fuel
