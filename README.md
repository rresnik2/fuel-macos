# FUEL for macOS (work in progress)

A native Apple silicon port of **FUEL** (Asobo Studio / Codemasters, 2009), the
open-world off-road racer. The goal is a Mac app that runs the original game
data with a new engine layer: Metal graphics, Core Audio, GameController input,
and no Windows compatibility layer.

**This repository contains no game files.** You need your own copy of FUEL for
Windows; the programs here read its data at run time.

## Status

Early. What works today:

- **`fuelview`**, a Metal viewer. It assembles any of the game's vehicles from
  its `.DPC` archive (`--assemble QUAD_ROAD_2`) or shows single parts, with the
  game's textures, materials, paint, liveries and dirt, lit by a modern renderer
  (physically based shading, sun shadows, sky light and reflections). This is
  the first native program in the port.
- **Terrain**: `--terrain` draws FUEL's 140 km world heightfield (decoded from
  the game's data) with a modern GPU terrain renderer, sky and haze;
  `--at X,Z` stands the vehicle on it. `--world` draws a world archive's
  placed objects. `fuelterrain` decodes and checks the heightfield.
- **Driving**: `--drive` puts an assembled vehicle on the terrain under Jolt
  Physics, tuned from the game's own vehicle data (mass, engine, torque curve,
  gearbox, drive axles, collision shapes), plus arcade additions (air control,
  landing stabilisation, speed-sensitive steering, self-righting). Keys:
  W/S or Up/Down throttle and brake/reverse, A/D or Left/Right steer, Space
  handbrake, R back on the wheels, Tab / Shift-Tab to switch to another of the
  72 drivable vehicles (bikes, quads, buggies, cars, SUVs, trucks); gamepads
  work (stick, triggers, X handbrake, Y reset). The world's 14,000 placed
  objects stream in around you and are solid. `fueldrive` runs a scripted drive without a window.
- **Driving sounds**: in the window, `--drive` plays FUEL's own samples,
  decoded at start-up from the vehicle archive (no extracted files): the engine
  as rpm-crossfaded idle and on/off-load layers, tyre rolling and skid loops,
  wind, and impact one-shots on hard braking or landings. M mutes.
  `fuelview --drive --audio-check` prints the sound layers of a few vehicles
  without opening audio. The tyre loops follow the surface under each wheel
  (crossfaded per class); backfire pops on fast rpm drops, suspension thumps on
  hard compression and a scrape loop when the body slides against the ground or
  an object are played too.
- **Surfaces**: the ground type under each wheel (the 40-type map, overridden by
  tarmac and dirt roads) scales that tyre's grip (tarmac 1.15, dirt 1.0, grass
  0.9, sand 0.75, snow 0.6, mud/wet 0.65, rock 0.95) and adds rolling resistance
  on soft ground; see docs/formats.md. `fueldrive DATA DPC VARIANT --surfaces`
  runs a throttle-then-brake test on tarmac, grass and sand and prints top
  speed, 0-80 km/h and braking distance.
- **Races**: `--race NAME` (for example `CR2-Yell-CP`) loads one of FUEL's 250
  races from the game's mission scripts: start line, ordered checkpoints shown
  as glowing beacons, laps, finish and a timer. N / Shift-N picks the next or
  previous race, Return restarts.
- **AI opponents**: races put five opponents (`--opponents N`, 0 to 11) of the
  player's class on the grid beside and behind the player, each a random
  livery and colour. They follow the checkpoint line looking ahead, brake for
  the corners to come, see static objects and steep terrain with a fan of rays
  and steer round them, back out when stuck and are put back on the line after
  ~4 s stuck, on the roof, or 12 s without progress (further along for each
  reset that did not help). Each has a skill (0.85-1.0 of the top speed) and
  is rubber-banded on the player (up to 12% slower when 200+ m ahead, 8% faster
  when behind). The HUD shows the race position; the finishing order is printed
  at the end. `fuelview --assemble BUGGY_OFF_1 --race CR2-Yell-CP --ai-check
  CommonComponent/DATAS/BUGGY.DPC` races headless (the player on the same AI)
  for up to `--drive-time` seconds (default 120) and prints every racer's
  checkpoints, speed and resets; `FUEL_AI_TRACE=N` follows opponent N.
- **Knock-aside props**: world objects with a collision radius of 0.4 to 3 m
  (barriers, banners, signs, cones, barrels) within 120 m of the player are
  dynamic Jolt bodies (convex hulls, mass from volume at 150 kg/m^3, capped at a
  fifth of the vehicle's mass). They sleep until hit, then tumble and are drawn
  where the physics puts them; knocked props stay where they landed until the
  player is 300 m away. Larger objects stay static. `fuelview --assemble
  QUAD_ROAD_2 --race CR2-Yell-CP --prop-check` (or `--at 45850,-6250`) drives at
  props headless and reports how many moved and the speed before and after;
  add `--shot` for a picture of the aftermath.
- **The rider**: every vehicle in `--assemble` and `--drive` carries FUEL's
  rider (default livery), skinned on the CPU into the seated pose of the
  vehicle's archetype from the game's own animations and drawn with the
  vehicle. On quads, buggies and trucks the rider leans into turns and pitches
  with throttle and brake (bikes lean as a whole). `--no-rider` hides it. `fuelrider DATA
  VARIANT [REF.obj]` bakes it and compares vertices with a reference export.
- **`fuelscene`** lists the vehicle variants in an archive and prints how each
  one is assembled.
- **`fuelcheck`**, a format checker. It decodes every texture, mesh and
  material in the game data and reports what parses (all of them, for the
  archive version the game loads).

Next steps, roughly in order: racing lines that follow roads instead of
straight lines between checkpoints (raids and some circuits still lean on AI
resets), bails and full rider animation, then game logic (career, unlocks,
menus), video and input. See `docs/formats.md` for the file formats
decoded so far.

## Requirements

- A Mac with Apple silicon
- An internet connection for the first build (CMake downloads Jolt Physics)
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
./build/fuelview --assemble QUAD_ROAD_2             # a whole quad bike
./build/fuelview --assemble TRUCK_OFF_1 CommonComponent/DATAS/TRUCK.DPC
./build/fuelscene ~/Library/Application\ Support/FUEL/game/CommonComponent/DATAS/QUAD.DPC
./build/fuelview --assemble QUAD_ROAD_2 --drive          # drive it
./build/fuelview --assemble BUGGY_ROAD_2 --race CR2-Yell-CP CommonComponent/DATAS/BUGGY.DPC   # race
./build/fuelview --assemble QUAD_ROAD_2 --at -1000,-30000   # on the terrain
./build/fuelview                                   # quad bike parts
./build/fuelview CommonComponent/DATAS/BUGGY.DPC   # any other archive
./build/fuelcheck ~/Library/Application\ Support/FUEL/game
```

Viewer controls: drag to orbit, scroll or pinch to zoom, left/right arrows for
the previous/next variant (with `--assemble`) or mesh, `C` back-face culling,
`W` wireframe, `D` dirt level, `G` ground, `R` reset view, `Q` or Esc to quit.

The look need not match the original exactly: lighting and graphics use modern
techniques driven by the game's own material data.

Paint: `--vehicle NAME` (for example `QUAD_OFF_5`) paints the bodywork with
that vehicle's palette and liveries, as set up in the game's `VehiclesDef.tsc`
and its livery library. `--livery N` (1-9, 0 = plain paint), `--color N`
(palette index 0-9) or `--paint R,G,B` choose the look; in the window, `L` and
`P` cycle liveries and colours.

`fuelview --shot out.png` renders one image without opening a window (options:
`--mesh ID`, `--size WxH`, `--yaw DEG`, `--pitch DEG`).

## Launcher

`cmake --build build` also assembles **`build/FUEL.app`**: a double-clickable
launcher (AppKit, no extra packages) with the game binary, `fuelview`, next to it
in `Contents/MacOS/`, an icon drawn from code at build time, and an ad-hoc
signature so it runs on the machine that built it. The bundle holds no game data.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
open build/FUEL.app
```

- **First run:** it asks for the game folder, the one holding `CommonComponent/`,
  `LocalizedComponent/` and `XFinalComponent/FUEL.exe` (the port reads engine
  tables from your copy of `FUEL.exe`; it never runs it). A folder that lacks
  `CommonComponent/WORLD/USA1.DPC`, `LocalizedComponent/GameTsc/` or
  `XFinalComponent/FUEL.exe` is refused with the missing item named. The folder is remembered.
- **Settings:** resolution (presets and the display's native size), windowed or
  fullscreen, post effects, grass, volume and language (en, fr, de, es, it). Every
  change is saved at once.
- **Play** starts `fuelview --game`. If it stops with an error, the launcher shows
  its output in a "Last run log" panel.
- **Where settings live:** `~/Library/Application Support/FUEL Port/settings.txt`,
  plain `key = value` lines (`data`, `width`, `height`, `fullscreen`,
  `post_effects`, `grass`, `volume`, `language`); see `src/core/settings.hpp`. Edit it
  by hand or delete it to start over. `fuelview --settings` applies the same file
  (window size, fullscreen, post effects, grass, volume) to a window run.

## Legal

FUEL and its content belong to their rights holders. This project is not
affiliated with or endorsed by Codemasters, Electronic Arts or Asobo Studio. It
ships no game code or data and does not bypass any copy protection; you need
your own copy of the game to use it.
