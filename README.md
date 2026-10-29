# FUEL for macOS (work in progress)

A native Apple silicon port of **FUEL** (Asobo Studio / Codemasters, 2009), the
open-world off-road racer. The goal is a Mac app that runs the original game
data with a new engine layer: Metal graphics, Core Audio, GameController input,
and no Windows compatibility layer.

**This repository contains no game files.** You need your own copy of FUEL for
Windows; the programs here read its data at run time.

## Status

Early. What works today:

- **`fuelview`**, a Metal model viewer. It opens any of the game's `.DPC`
  archives and shows each vehicle part with its textures, materials, paint and
  liveries. This is the first native program in the port.
- **`fuelcheck`**, a format checker. It decodes every texture, mesh and
  material in the game data and reports what parses (all of them, for the
  archive version the game loads).

Next steps, roughly in order: node hierarchy and assembled vehicles, skinning,
the shader set translated to Metal, terrain and world streaming, then game
logic, audio, video and input. See `docs/formats.md` for the file formats
decoded so far.

## Requirements

- A Mac with Apple silicon
- Xcode or the Xcode Command Line Tools (`xcode-select --install`)
- CMake 3.24 or newer (`brew install cmake`)
- FUEL for Windows (disc or a digital copy)

## Getting the game data

The port reads the game's data as it is stored on the disc: the folder that
contains `CommonComponent/`. Unpack it from the disc's InstallShield cabinet
with [unshield](https://github.com/twogood/unshield) (replace `/Volumes/FUEL`
with wherever the disc or its image is mounted):

```sh
brew install unshield
mkdir -p ~/Library/Application\ Support/FUEL/game
unshield -d ~/Library/Application\ Support/FUEL/game x /Volumes/FUEL/data1.cab
```

Reading an existing Windows installation is not supported yet.

The programs look for the data in `--data DIR`, then `$FUEL_DATA`, then
`~/Library/Application Support/FUEL/game`. A built-in importer will replace
this step later.

## Build and run

```sh
cmake -S . -B build
cmake --build build
./build/fuelview                                   # quad bike parts
./build/fuelview CommonComponent/DATAS/BUGGY.DPC   # any other archive
./build/fuelcheck ~/Library/Application\ Support/FUEL/game
```

Viewer controls: drag to orbit, scroll or pinch to zoom, left/right arrows for
the previous/next mesh (largest first), `C` back-face culling, `W` wireframe,
`R` reset view, `Q` or Esc to quit.

Paint: `--vehicle NAME` (for example `QUAD_OFF_5`) paints the bodywork with
that vehicle's palette and liveries, as set up in the game's `VehiclesDef.tsc`
and its livery library. `--livery N` (1-9, 0 = plain paint), `--color N`
(palette index 0-9) or `--paint R,G,B` choose the look; in the window, `L` and
`P` cycle liveries and colours.

`fuelview --shot out.png` renders one image without opening a window (options:
`--mesh ID`, `--size WxH`, `--yaw DEG`, `--pitch DEG`).

## Legal

FUEL and its content belong to their rights holders. This project is not
affiliated with or endorsed by Codemasters, Electronic Arts or Asobo Studio. It
ships no game code or data and does not bypass any copy protection; you need
your own copy of the game to use it.
