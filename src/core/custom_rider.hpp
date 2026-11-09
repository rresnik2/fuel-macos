// PHYSIQUE (port deviation, user request 2026-11-06; ~/re-work/fuel/notes/physique.md): the player's rider can wear a new
// body built in Blender instead of FUEL's pilot meshes, with three outfits. The assets are NOT part of the port: they are
// read at run time from a folder (rider.fra + DDS textures), $FUEL_CUSTOM_RIDER or FUEL.app/Contents/Resources/rider
// (CMake copies ~/re-work/fuel/custom-assets/rider there when it exists). Without them everything stays FUEL's rider.
//
// The body is rigged to FUEL's own rider skeleton (PILOT.DPC Skel_Z d1ab9557, bind pose, rider model space), so FUEL's
// animations, IK, seat point, ragdoll and garage poses drive it unchanged; it is skinned on the CPU into a BakedMesh like
// the pilot parts (rider.hpp). AI riders keep FUEL's riders.
//
// rider.fra (little-endian): "FUELRDR1", u32 version 1 or 2; u32 n, n x u32 bone name CRC; [v2: u32 n helper bones
// {u32 name CRC, u32 base bone name CRC, f32 rest centre[3], f32 radius} (the glute jiggle bones, rider_jiggle.hpp: not
// FUEL bones, never in the bone table)]; u32 n materials {u32 flags, f32 spec
// rgb + power, u32 FUEL Material_Z name CRC, u32 FUEL Material_Z id, str diffuse, str normal, str specular, str name}; u32 n
// meshes {str name, u32 lod, u32 garment bit + 1 (0 none), u32 n vertices x {f32 pos[3], nrm[3], tan[4] (w = handedness),
// uv[2] (D3D: v down), u8 bone[4] (into the bone table), f32 w[4], [v2: u8 helper (1 + index, 0 none), f32 its weight]},
// u32 n batches {u32 material, u32 n indices, indices,
// u32 has mask, [n / 3 x u32 garment bits that cover the triangle]}}. str = u16 length + bytes. Positions in FUEL rider
// space (x = the rider's left, y up, z forward), the skeleton's bind pose.
#pragma once

#include "rider.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

// What the player wears: 0 = FUEL's own rider (the pilot parts as before), 1 = the new body in FUEL's riding gear (the
// garage's head / chest / legs types and liveries, refitted), 2 = tank top + denim cutoffs, 3 = shirtless + shorts.
// Outfits 2 and 3 keep FUEL's riding boots and gloves. `helmet` = the garage's head part (FUEL's helmets) on the new head.
struct RiderOutfit {
    int outfit = 1;
    bool helmet = true;
    bool operator==(const RiderOutfit& o) const { return outfit == o.outfit && helmet == o.helmet; }
    bool operator!=(const RiderOutfit& o) const { return !(*this == o); }
};
inline constexpr int kRiderOutfits = 4;  // 0..3

struct CustomMaterial {
    // [upgrades U5] Solid: no textures, the diffuse is `rgba` (a 1 x 1 texture); Chrome: + FUEL's cube reflection (bCubemap,
    // Fresnel `fresnel`); Paint: the vehicle's paint colour (code 19, the diffuse alpha 0) (core/upgrade_visuals).
    enum Flags : uint32_t { Skin = 1, AlphaTest = 2, TwoSided = 4, Hair = 8, Eye = 16, Fuel = 32, Solid = 64, Chrome = 128, Paint = 256,
                            Carbon = 512 /* [upgrades U5] the paint all carbon weave */ };
    std::string name;
    std::string diffuse, normal, specular;  // absolute paths of DDS files ("" = none)
    std::array<float, 4> spec{0.25f, 0.25f, 0.25f, 24.0f};
    uint32_t flags = 0;
    std::array<float, 4> rgba{1, 1, 1, 1};     // [upgrades U5] Solid: the colour (gamma space, as FUEL's textures)
    std::array<float, 2> fresnel{0.4f, 1.0f};  // [upgrades U5] Chrome
    uint32_t fuelName = 0, fuelMaterial = 0;  // Fuel: a FUEL Material_Z (name CRC, id) with the garage livery applied
};

struct CustomMesh {
    std::string name;
    int lod = 0;
    int garment = -1;  // the body-mask bit this piece sets when worn, -1 none
    struct V {
        float pos[3], nrm[3], tan[4], uv[2];
        uint8_t bone[4];
        float w[4];
        uint8_t jig = 0;  // v2: 1 + the helper (jiggle) bone, 0 none
        float jw = 0;
    };
    std::vector<V> verts;
    struct Batch {
        uint32_t material = 0;
        std::vector<uint32_t> indices;
        std::vector<uint32_t> mask;  // per triangle (body only): the garment bits covering it
    };
    std::vector<Batch> batches;
};

class CustomRider {
public:
    // Garment bits of the body mask.
    enum Garment { ChestGear1 = 0, LegsGear1 = 5, Tank = 8, Cutoffs = 9, Shorts = 10, Boots = 11, Gloves = 12, Helmet = 13 };
    struct Helper {  // v2: a jiggle helper bone
        uint32_t name = 0, base = 0;
        float centre[3] = {0, 0, 0};
        float radius = 0.09f;
    };

    // The asset folder: $FUEL_CUSTOM_RIDER, else <executable>/../Resources/rider, else "" (none).
    static std::string findDir();
    // Loads <dir>/rider.fra; nullptr (and a reason) when missing or malformed. Textures are only named here.
    static std::unique_ptr<CustomRider> load(const std::string& dir, std::string* error);
    // The process-wide instance (loaded on first use from findDir(); nullptr without assets). FUEL_CUSTOM_RIDER=off disables.
    static const CustomRider* shared();

    // The skin template of `o` on FUEL's skeleton for LOD `lod` (0..2): the custom pieces (bone indices mapped to `rider`'s
    // skeleton by name), the body without the triangles the worn pieces cover, plus FUEL's head part when `o.helmet`.
    // `parts` = the garage's FUEL parts (gear types, liveries). Throws on a bone the skeleton lacks.
    Rider::SkinTemplate prepare(const Rider& rider, const RiderOutfit& o, const Rider::Parts& parts, int lod = 0) const;
    int lodCount() const { return lods_; }
    // The coverage check (--animcheck): for every outfit (1: each CHEST 1-5 x LEGS 1-3; 2; 3) with and without a helmet,
    // the pieces it needs are in the file, and every body triangle it culls has one of the worn pieces over it (a ray
    // along the triangle's normal, bind pose). Returns the number of problems; *configs = configurations checked.
    int coverageCheck(int* configs, std::vector<std::string>* problems) const;
    // rider4: FUEL's skin tones (the garage SKIN choice, Rider::Parts::skin = body type BODY<k>, 1 .. kSkinTones). FUEL
    // swaps the skin batch's texture (LIVERY_XX_BODY -> LIVERY_<l>_BODY<k>, 0x5dcfe0); the custom rider's Skin materials
    // have their diffuse per tone next to them: <name>_d_t<k>.dds (tone 1 = the material's own). The material to draw
    // material `m` with at tone `skin` (itself when the tone has no file, or `m` is not a Skin material).
    static constexpr int kSkinTones = 3;
    const CustomMaterial& toned(uint32_t m, int skin) const;
    int toneVariants() const { return toneVariants_; }
    const std::vector<CustomMaterial>& materials() const { return materials_; }
    const std::vector<CustomMesh>& meshes() const { return meshes_; }
    const std::string& dir() const { return dir_; }
    const std::vector<Helper>& helpers() const { return helpers_; }

private:
    const CustomMesh* mesh(const std::string& name, int lod) const;
    std::string dir_;
    std::vector<uint32_t> bones_;
    std::vector<Helper> helpers_;
    int version_ = 1;
    std::vector<CustomMaterial> materials_;
    std::vector<CustomMaterial> toneMaterials_;               // rider4: the Skin materials at tones 2 .. kSkinTones
    std::vector<std::array<int, kSkinTones - 1>> toneIndex_;  // per material: index into toneMaterials_, -1 none
    int toneVariants_ = 0;
    std::vector<CustomMesh> meshes_;
    int lods_ = 1;
};

// Outfit names for the garage carousel (port texts).
const char* riderOutfitName(const RiderOutfit& o);

}  // namespace fuel
