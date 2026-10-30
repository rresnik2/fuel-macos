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
  src/core/ground.cpp
  src/core/vegetation.cpp
  src/core/roads.cpp
  src/core/vehicle_params.cpp
  src/core/races.cpp
  src/core/assembly.cpp
  src/core/rider.cpp
)
target_include_directories(fuelcore PUBLIC src)

# Parses every resource in a data directory and reports what decodes.
add_executable(fuelcheck src/tools/fuelcheck.cpp)
target_link_libraries(fuelcheck PRIVATE fuelcore)

# Assembles vehicle variants and prints their placed parts (or JSON).
add_executable(fuelscene src/tools/fuelscene.cpp)
target_link_libraries(fuelscene PRIVATE fuelcore)

# Bakes the rider for a vehicle and checks it against a reference export.
add_executable(fuelrider src/tools/fuelrider.cpp)
target_link_libraries(fuelrider PRIVATE fuelcore)

# Mesh viewer: the first native Metal program (P5 spike).
add_executable(fuelview
  src/viewer/main.mm
  src/viewer/renderer.mm
  src/viewer/drive.mm
)
target_compile_options(fuelview PRIVATE -fobjc-arc)
target_link_libraries(fuelview PRIVATE fuelphysics "-framework GameController"
  "-framework Cocoa" "-framework Metal" "-framework MetalKit"
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

# Vehicle physics on Jolt, tuned from the game's data.
add_library(fuelphysics STATIC src/physics/vehicle_physics.cpp)
target_link_libraries(fuelphysics PUBLIC fuelcore Jolt)

# Drives a vehicle headless and prints the result.
add_executable(fueldrive src/tools/fueldrive.cpp)
target_link_libraries(fueldrive PRIVATE fuelphysics)
