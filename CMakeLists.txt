cmake_minimum_required(VERSION 3.24)
project(fuel_port LANGUAGES C CXX OBJCXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 17)
set(CMAKE_OSX_ARCHITECTURES arm64)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# Game-independent engine code: file formats, resource decoding, assembly.
add_library(fuelcore STATIC
  src/core/lz.cpp
  src/core/dpc.cpp
  src/core/resources.cpp
  src/core/material_anim.cpp
  src/core/blob_shadow.cpp
  src/core/vehicle_lights.cpp
  src/core/vehicle.cpp
  src/core/world.cpp
  src/core/storms.cpp
  src/core/tornado_fx.cpp
  src/core/wind_tank.cpp
  src/core/automesh.cpp
  src/core/terrain.cpp
  src/core/terrain_height.cpp
  src/core/terrain_mesh.cpp
  src/core/road_types.cpp
  src/core/ground.cpp
  src/core/vegetation.cpp
  src/core/genworld_noise.cpp
  src/core/ground_point.cpp
  src/core/roads.cpp
  src/core/road_table.cpp
  src/core/road_segments.cpp
  src/core/road_graph.cpp src/core/traffic.cpp src/core/hub_zones.cpp
  src/core/water.cpp
  src/core/surface.cpp
  src/core/surface_patch.cpp
  src/core/vehicle_params.cpp
  src/core/vehicle_damage.cpp
  src/core/vehicle_links.cpp
  src/core/vehicle_fx.cpp  # PK23
  src/core/races.cpp
  src/core/world_missions.cpp
  src/core/spline_graph.cpp
  src/core/static_collision.cpp
  src/core/dyn_object.cpp
  src/core/anim_trigger.cpp
  src/core/events.cpp
  src/core/career.cpp
  src/core/career_upgrades.cpp  # [upgrades U2] vehicle upgrades: buy / equip / gates (user request)
  src/core/upgrade_rules.cpp  # [upgrades U2] upgrades.hpp's categories, prices and gates (the career's side)
  src/core/custom_vehicles.cpp  # CUSTOM VEHICLES (port deviation)
  src/core/save_import.cpp  # PK25
  src/core/career_check.cpp
  src/core/freeroam.cpp
  src/core/race_helicopter.cpp
  src/core/f14_flyby.cpp
  src/core/race_rules.cpp
  src/core/race_editor.cpp  # PK21
  src/core/race_markers.cpp
  src/core/object_script.cpp
  src/core/console.cpp
  src/core/hubs.cpp
  src/core/assembly.cpp
  src/core/sound.cpp
  src/core/rider.cpp
  src/core/custom_rider.cpp  # PHYSIQUE (port deviation)
  src/core/rider_jiggle.cpp  # PHYSIQUE: glute secondary motion
  src/core/camera_params.cpp
  src/core/rider_tables.cpp
  src/core/rider_anim.cpp
  src/core/settings.cpp
  src/core/input_map.cpp
  src/core/remap.cpp  # [port] remap
  src/core/input_check.cpp
  src/core/vibration.cpp
  src/core/anim_clip.cpp
  src/core/world_anim.cpp
  src/core/rtc.cpp
  src/core/rtc_fade.cpp
  src/core/skinned_model.cpp
  src/core/anim_check.cpp
  src/core/atmosphere.cpp
  src/core/sfx.cpp
  src/core/weather.cpp
  src/core/clouds.cpp
  src/core/weather_fx.cpp
  src/core/sand_tank.cpp  # PK12
  src/core/particles.cpp
  src/core/rotshape.cpp
)
target_include_directories(fuelcore PUBLIC src)

# FUEL's menus: script reader, page model, text tables, fonts and images (no Metal).
add_library(fuelui STATIC
  src/ui/tsc.cpp
  src/ui/menu_model.cpp
  src/ui/menu_assets.cpp
  src/ui/menu_check.cpp
  src/ui/port_options.cpp
  src/ui/hud_math.cpp
  src/ui/gps_guide.cpp
  src/ui/help_texts.cpp  # PK26
  src/ui/pad_prompts.cpp  # pad-ux
)
target_link_libraries(fuelui PUBLIC fuelcore)

# Parses every resource in a data directory and reports what decodes.
add_executable(fuelcheck src/tools/fuelcheck.cpp)
target_link_libraries(fuelcheck PRIVATE fuelcore)

# Assembles vehicle variants and prints their placed parts (or JSON).
add_executable(fuelscene src/tools/fuelscene.cpp)
target_link_libraries(fuelscene PRIVATE fuelcore)

# Bakes the rider for a vehicle and checks it against a reference export.
add_executable(fuelrider src/tools/fuelrider.cpp)
target_link_libraries(fuelrider PRIVATE fuelcore)

# The event catalogue and a physics-free run of every event's rules.
add_executable(fuelevents src/tools/fuelevents.cpp)
target_link_libraries(fuelevents PRIVATE fuelcore)

# Mesh viewer: the first native Metal program (P5 spike).
add_executable(fuelview
  src/viewer/main.mm
  src/viewer/renderer.mm
  src/viewer/drive.mm
  src/viewer/anim_view.mm
  src/viewer/heli_view.mm
  src/viewer/f14_view.mm
  src/viewer/storm_view.mm
  src/viewer/damage_view.mm
  src/viewer/tornado_render.mm
  src/viewer/lens_render.mm  # PK24
  src/viewer/vehicle_fx_render.mm  # PK23
  src/viewer/sand_render.mm  # PK12
  src/viewer/taa_render.mm  # TAA: MetalFX temporal upscaling (port option)
  src/viewer/garage_preview.mm  # PK20
  src/viewer/marker_view.mm
  src/viewer/recorder.mm
  src/viewer/hubcheck.mm
  src/viewer/freeroam_view.mm src/viewer/traffic_view.mm
  src/viewer/capture.mm
  src/viewer/eventcheck.mm
  src/viewer/game_settings.mm
  src/viewer/pad_haptics.mm
  src/viewer/pad_triggers.mm
  src/viewer/rendercheck.mm
  src/viewer/gpu_profile.mm
  src/viewer/perf_check.mm
  src/viewer/pop_check.mm
  src/audio/ambience.cpp
  src/audio/drive_audio.mm
  src/audio/music.cpp
  src/audio/menu_audio.mm
  src/audio/music_check.cpp
  src/audio/sound_mixer.cpp
  src/audio/vehicle_sound.cpp
  src/audio/ev_whine.cpp  # vehicles4: custom EVs
  src/audio/nitro_sound.cpp  # [nitro]
  src/ui/ui_renderer.mm
  src/ui/menu.mm
  src/ui/garage.mm
  src/ui/garage_upgrades.mm  # [upgrades U3] the garage's UPGRADES page (user request)
  src/ui/dev_mode.mm  # [dev mode] the overlay's options hook, DEV tag and --menu-check (user request)
  src/ui/profile.mm
  src/ui/hud.mm
  src/ui/nitro_gauge.mm  # [nitro]
  src/ui/map_page.mm
  src/ui/nav_hud.mm
  src/ui/tutorial.mm
  src/ui/game_menu.mm
  src/ui/loading_screen.mm
  src/ui/bonus_codes.mm
  src/ui/career_audit.mm
  src/ui/options_pages.mm
  src/ui/menu_chrome.mm  # PK18
  src/ui/editor.mm  # PK21
  src/ui/alerts.mm  # PK25
  src/ui/boot_screens.mm  # PK26
  src/ui/pad_glyphs.mm  # pad-ux
  src/ui/menu_input.mm  # [port] remap
  src/ui/remap_page.mm  # [port] remap
)
target_compile_options(fuelview PRIVATE -fobjc-arc)
target_link_libraries(fuelview PRIVATE fuelphysics fuelui "-framework GameController" "-framework CoreHaptics" "-framework IOKit"
  "-framework Cocoa" "-framework AVFoundation" "-framework Metal" "-framework MetalKit" "-framework MetalFX"
  "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics"
  "-framework UniformTypeIdentifiers")

# PK25: imports an original Windows FUEL career.sav (read only) into a FUELPSAV file at an explicit path.
add_executable(fuelsaveimport src/tools/fuelsaveimport.cpp)
target_link_libraries(fuelsaveimport PRIVATE fuelcore)

# Decodes the world heightfield and checks it against a reference export.
add_executable(fuelterrain src/tools/fuelterrain.cpp)
target_link_libraries(fuelterrain PRIVATE fuelcore)

# Jolt Physics (MIT licence), downloaded at configure time.
include(FetchContent)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(CPP_RTTI_ENABLED ON CACHE BOOL "" FORCE)
# FUEL's world is 140 km across: double precision keeps physics stable far from the origin.
set(DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
FetchContent_Declare(JoltPhysics
  GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
  GIT_TAG v5.6.0
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR Build)
FetchContent_MakeAvailable(JoltPhysics)

# FUEL's own vehicle model (decoded from FUEL.exe, notes p23/p24), the world around it (Jolt only for the
# static objects, props and ray/sphere queries) and the opponents' driver AI.
add_library(fuelphysics STATIC src/physics/vehicle_physics.cpp src/physics/race_ai.cpp src/physics/atv_path.cpp src/physics/atv_check.cpp
  src/physics/fuel_contacts.cpp src/physics/fuel_collide.cpp src/physics/fuel_vehicle.cpp src/physics/fuel_vehicle_control.cpp
  src/physics/fuel_camera.cpp src/physics/camcheck.cpp src/physics/breadcrumbs.cpp
  src/physics/nitro.cpp src/physics/ai_nitro.cpp  # [nitro] the port's nitrous (user request)
  src/core/upgrades.cpp  # [upgrades U1] vehicle upgrades (user request)
  src/core/upgrade_visuals.cpp  # [upgrades U5] the visible upgrades (user request)
  src/core/susp_anim.cpp)  # [suspanim] suspension / steering parts every frame (0x6a0200)
target_link_libraries(fuelphysics PUBLIC fuelcore Jolt)

# The decode's test vectors (emulated on FUEL.exe) run through the port's functions.
add_executable(fuelphysvec src/tools/fuelphysvec.cpp)
target_link_libraries(fuelphysvec PRIVATE fuelphysics)

# Drives a vehicle headless and prints the result.
add_executable(fueldrive src/tools/fueldrive.cpp)
target_link_libraries(fueldrive PRIVATE fuelphysics)
# CUSTOM VEHICLES (port deviation): builds a custom vehicle pack archive; inspects FUEL vehicles.
add_executable(fuelvehpack src/tools/fuelvehpack.cpp)
target_link_libraries(fuelvehpack PRIVATE fuelcore)

# ---- FUEL.app: the double-clickable launcher (P20) -------------------------------------------
# build/FUEL.app = Contents/MacOS/FUEL (launcher) + Contents/MacOS/fuelview (the game) + an
# icon drawn by fuelicon at build time, ad-hoc signed. No game data goes into the bundle.
add_executable(fuellauncher src/launcher/main.mm)
target_compile_options(fuellauncher PRIVATE -fobjc-arc)
target_link_libraries(fuellauncher PRIVATE fuelcore "-framework Cocoa")

add_executable(fuelicon src/launcher/icon.mm)
target_compile_options(fuelicon PRIVATE -fobjc-arc)
target_link_libraries(fuelicon PRIVATE "-framework Foundation" "-framework CoreGraphics"
  "-framework ImageIO" "-framework UniformTypeIdentifiers")

# LSMinimumSystemVersion: the deployment target, which defaults to the macOS the build runs on.
if(CMAKE_OSX_DEPLOYMENT_TARGET)
  set(FUEL_MIN_MACOS "${CMAKE_OSX_DEPLOYMENT_TARGET}")
else()
  execute_process(COMMAND sw_vers -productVersion OUTPUT_VARIABLE FUEL_HOST_MACOS OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "^[0-9]+(\\.[0-9]+)?" FUEL_MIN_MACOS "${FUEL_HOST_MACOS}")
endif()
configure_file(src/launcher/Info.plist.in ${CMAKE_BINARY_DIR}/launcher/Info.plist @ONLY)

set(FUEL_APP ${CMAKE_BINARY_DIR}/FUEL.app)
# PHYSIQUE (port deviation): the player's custom rider assets live outside the repo; copied into the bundle when present.
set(FUEL_CUSTOM_RIDER_DIR "$ENV{HOME}/re-work/fuel/custom-assets/rider" CACHE PATH "custom rider assets (rider.fra + DDS)")
# CUSTOM VEHICLES (port deviation): the custom vehicle packs live outside the repo; copied into the bundle when present.
set(FUEL_CUSTOM_VEHICLES_DIR "$ENV{HOME}/re-work/fuel/custom-assets/vehicles" CACHE PATH "custom vehicle packs (vehicle.tsc + DPC)")
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/launcher/FUEL.icns
  COMMAND fuelicon ${CMAKE_BINARY_DIR}/launcher/FUEL.iconset
  COMMAND iconutil -c icns -o ${CMAKE_BINARY_DIR}/launcher/FUEL.icns ${CMAKE_BINARY_DIR}/launcher/FUEL.iconset
  DEPENDS fuelicon
  COMMENT "Drawing the app icon")
add_custom_target(FUEL_app ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${FUEL_APP}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${FUEL_APP}/Contents/MacOS ${FUEL_APP}/Contents/Resources
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:fuellauncher> ${FUEL_APP}/Contents/MacOS/FUEL
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:fuelview> ${FUEL_APP}/Contents/MacOS/fuelview
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/launcher/Info.plist ${FUEL_APP}/Contents/Info.plist
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/launcher/FUEL.icns ${FUEL_APP}/Contents/Resources/FUEL.icns
  COMMAND ${CMAKE_COMMAND} -DSRC=${FUEL_CUSTOM_RIDER_DIR} -DDST=${FUEL_APP}/Contents/Resources/rider -P ${CMAKE_SOURCE_DIR}/cmake/copy_custom_rider.cmake
  COMMAND ${CMAKE_COMMAND} -DSRC=${FUEL_CUSTOM_VEHICLES_DIR} -DDST=${FUEL_APP}/Contents/Resources/vehicles -P ${CMAKE_SOURCE_DIR}/cmake/copy_custom_vehicles.cmake
  COMMAND codesign --force --deep -s - ${FUEL_APP}
  DEPENDS fuellauncher fuelview ${CMAKE_BINARY_DIR}/launcher/FUEL.icns
  COMMENT "Assembling and ad-hoc signing FUEL.app")

# P73: Surface_Z Bezier patches (survey of the data, emulated vectors).
add_executable(fuelsurface src/tools/fuelsurface.cpp)
target_link_libraries(fuelsurface PRIVATE fuelcore)
