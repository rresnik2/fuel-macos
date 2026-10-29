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
  src/core/assembly.cpp
)
target_include_directories(fuelcore PUBLIC src)

# Parses every resource in a data directory and reports what decodes.
add_executable(fuelcheck src/tools/fuelcheck.cpp)
target_link_libraries(fuelcheck PRIVATE fuelcore)

# Assembles vehicle variants and prints their placed parts (or JSON).
add_executable(fuelscene src/tools/fuelscene.cpp)
target_link_libraries(fuelscene PRIVATE fuelcore)

# Mesh viewer: the first native Metal program (P5 spike).
add_executable(fuelview
  src/viewer/main.mm
  src/viewer/renderer.mm
)
target_compile_options(fuelview PRIVATE -fobjc-arc)
target_link_libraries(fuelview PRIVATE fuelcore
  "-framework Cocoa" "-framework Metal" "-framework MetalKit"
  "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics"
  "-framework UniformTypeIdentifiers")
