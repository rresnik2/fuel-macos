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
  src/core/vehicle.cpp
  src/core/world.cpp
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
  src/core/water.cpp
  src/core/surface.cpp
  src/core/vehicle_params.cpp
  src/core/races.cpp
  src/core/events.cpp
  src/core/race_rules.cpp
  src/core/hubs.cpp
  src/core/assembly.cpp
  src/core/sound.cpp
  src/core/rider.cpp
  src/core/camera_params.cpp
  src/core/rider_tables.cpp
  src/core/rider_anim.cpp
  src/core/settings.cpp
  src/core/atmosphere.cpp
)
target_include_directories(fuelcore PUBLIC src)

# FUEL's menus: script reader, page model, text tables, fonts and images (no Metal).
add_library(fuelui STATIC
  src/ui/tsc.cpp
  src/ui/menu_model.cpp
  src/ui/menu_assets.cpp
  src/ui/menu_check.cpp
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
  src/viewer/recorder.mm
  src/viewer/hubcheck.mm
  src/viewer/capture.mm
  src/viewer/eventcheck.mm
  src/viewer/game_settings.mm
  src/viewer/rendercheck.mm
  src/audio/drive_audio.mm
  src/ui/ui_renderer.mm
  src/ui/menu.mm
  src/ui/game_menu.mm
)
target_compile_options(fuelview PRIVATE -fobjc-arc)
target_link_libraries(fuelview PRIVATE fuelphysics fuelui "-framework GameController"
  "-framework Cocoa" "-framework AVFoundation" "-framework Metal" "-framework MetalKit"
  "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics"
  "-framework UniformTypeIdentifiers")

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
add_library(fuelphysics STATIC src/physics/vehicle_physics.cpp src/physics/race_ai.cpp
  src/physics/fuel_contacts.cpp src/physics/fuel_vehicle.cpp src/physics/fuel_vehicle_control.cpp
  src/physics/fuel_camera.cpp src/physics/camcheck.cpp)
target_link_libraries(fuelphysics PUBLIC fuelcore Jolt)

# The decode's test vectors (emulated on FUEL.exe) run through the port's functions.
add_executable(fuelphysvec src/tools/fuelphysvec.cpp)
target_link_libraries(fuelphysvec PRIVATE fuelphysics)

# Drives a vehicle headless and prints the result.
add_executable(fueldrive src/tools/fueldrive.cpp)
target_link_libraries(fueldrive PRIVATE fuelphysics)

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
  COMMAND codesign --force --deep -s - ${FUEL_APP}
  DEPENDS fuellauncher fuelview ${CMAKE_BINARY_DIR}/launcher/FUEL.icns
  COMMENT "Assembling and ad-hoc signing FUEL.app")
