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
  decoded at start-up from the vehicle archive (no extracted files), with FUEL's
  own sound model (decoded from FUEL.exe, notes p41): the engine's rpm layers,
  load and pitch tables, backfire pops (only above half revs with the throttle at
  most half and the revs falling fast), tyre rolling / skid loops per ground
  class, wind, impacts, scrapes, landings, suspension thumps, the rev limiter,
  the game's mix (camera-view categories, engine ducking in the air and after
  hits) and its DirectSound output (distance attenuation 50 / 350 m, pan, no
  doppler). Opponents' engines play too. M mutes.
  `fuelview --assemble QUAD_ROAD_2 --drive --audio-check` checks the sound maths
  against decoded and emulated vectors and renders drives offline (no device).
- **Controller vibration**: a DualSense, DualShock 4 or Xbox controller rumbles
  as FUEL's pad does (VibrationManager_C with the game's Vibrations.tsc, notes
  p89-pad-vibration.md): crashes and hits (three strengths by impact), scraping
  along walls, hard suspension hits (landings, big bumps) and backfire pops;
  FUEL's left (low-frequency) motor drives the left grip, the right one the
  right grip. Pausing stops it. `vibration = 0` in settings.txt turns it off.
  `fuelview --assemble QUAD_ROAD_2 --drive --haptics-check` checks the rules
  against emulated vectors and prints the motors over a drive and impacts.
- **DualSense adaptive triggers** (an addition: FUEL has none; it only reads
  the game, notes p91-adaptive-triggers.md): both triggers resist
  progressively (throttle gentle to firmer, brake soft at the top and stiff at
  the bottom like a pedal; trucks heavier, bikes lighter), and they tell you
  the grip left from FUEL's own tyre model: the throttle gives way as the
  driven wheels spin (deep wheelspin buzzes), the brake's bottom softens toward
  lock-up and pulses slowly when the wheels slide, and loose or wet ground makes
  both lighter. Off in menus, pause, cut-scenes and loading, and on quit.
  settings.txt `adaptive_triggers = off | light | full` (default full);
  `adaptive_triggers_extras = gear,limiter,air,impact,damage` (or `all`) adds
  the secondary effects (off by default). USB updates up to 60 times a second;
  Bluetooth 10 times in 5 grip bands. `fuelview --assemble QUAD_ROAD_2
  --trigger-check` prints the modes over a scripted drive along CR2-Yell-CP
  and fails if a trigger is ever on in a pause, menu or cut-scene.
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
  livery and colour. They are FUEL's AI racers (decoded from FUEL.exe, notes
  p29-ai.md): each runs a "ghost" along a line that follows the roads where
  the checkpoints lie near them, smoothed through a corridor that other racers,
  objects and steep ground narrow; the ghost's speed races the event's target
  time (`FUEL_AI_LEVEL` 0-2 picks which, default 1), with class margins, a
  catch-up near the player and corner speeds by radius. Opponents within 150 m
  of the player are simulated and pulled to their ghost by FUEL's guided
  forces (throttle and brake only); farther ones ride their ghost. Stuck ghosts
  back up, then reset onto the line. The HUD shows the race position; the
  finishing order is printed at the end. `fuelview --assemble BUGGY_OFF_1
  --race CR2-Yell-CP --ai-check CommonComponent/DATAS/BUGGY.DPC` races headless
  (the player on the same AI) for up to `--drive-time` seconds (default 120)
  and prints every racer's class, checkpoints, speed and resets (the AI's and
  the vehicle's own); `FUEL_AI_TRACE=N` follows opponent N. `--ai-player stop`
  keeps the player on the grid, `--ai-player launch` drives it flat out from
  GO; the check also fails on an opponent flying over the ground in the first
  5 s and on leads past what FUEL's launch rules allow (P97).
  `fuelview --data DIR --aicheck-vectors` checks the line, aim and corner
  speeds against FUEL.exe run in an emulator (the guided forces are in
  `build/fuelphysvec`).
- **Knock-aside props**: FUEL's dynamic objects (IT_DYNOBJ_G, placed and
  AutoMesh road signs, barriers and wrecks) within 120 m of the player are
  dynamic Jolt bodies made of the object's own collision spheres and boxes,
  with FUEL's mass (volume x density: MASSEVOLUMIQUE or pp001 PP150 above
  1 m^3, PP151 below) and FUEL's drag. They sleep until hit, then tumble and
  are drawn where the physics puts them; knocked props stay where they landed
  until the player is 300 m away. Breakable decor (fences, guard rails,
  sheds) breaks under any vehicle faster than its P575, or a prop faster than
  5 m/s, and leaves its IT_DYNOBJ_G pieces. `fuelview --assemble QUAD_ROAD_2
  --at -51400,-30100 --prop-check` drives at props headless and reports how
  many moved and the speed before and after (`FUEL_P55_AIM_BREAKABLE=1`
  aims at breakables instead); add `--shot` for a picture of the aftermath.
  A race retry (RESTART) runs FUEL's KillMission first: props go back on
  their placements, break pieces vanish, the course sub-world comes back
  fresh. `fuelview --assemble QUAD_ROAD_2 --retry-check` knocks props near
  CR3-Yell-CT's start (`--race`, `--at` change it), retries the race and
  checks they stand where they were placed.
- **The rider**: every vehicle in `--assemble` and `--drive` carries FUEL's
  rider (default livery), skinned on the CPU into the seated pose of the
  vehicle's archetype from the game's own animations and drawn with the
  vehicle. On quads, buggies and trucks the rider leans into turns and pitches
  with throttle and brake (bikes lean as a whole). `--no-rider` hides it. `fuelrider DATA
  VARIANT [REF.obj]` bakes it and compares vertices with a reference export.
- **Performance check**: `fuelview --perf-check` races CR2-Yell-CP (BUGGY_OFF_3
  with the event's opponents; `--race`, `--time HH.MM`, `--drive-time S`,
  `--size WxH` change it) and renders every frame offscreen at 3420x2224. It
  prints each render pass's GPU time (mean, 95th percentile, vertex / fragment
  stages), the CPU time by phase, slow frames with their cause, and fails when
  the 95th-percentile GPU frame is over `--perf-budget MS` (13 ms: 60 fps with
  20% headroom), the CPU one over 16.7 ms or a frame stalls over 25 ms.
  `--perf-breakdown` prices the scene pass's parts (and the shadow slices'
  terrain / object casters and the cube face); `--perf-csv FILE` logs
  every frame. A MacBook Air's GPU slows when hot: compare runs at similar
  "clock probe" times, or compare two settings inside one run with
  `--perf-alternate scale=0.8 | upscaler=0.8 | temporal=0.8 | shadow | grain[=AMP] | soften[=K] | opt=MASK` (even frames
  A, odd B; `opt=MASK` turns the renderer's P95 optimisations in MASK off in
  the A frames; `FUEL_P95_OFF=MASK` turns them off for a whole run, e.g. to
  compare shots).
  It runs with the game's defaults (dynamic resolution on, 2x2 shadows; `--dynres
  off`, `--shadow-filter 3x3` change them) and reports the share of frames at
  each render scale. `--perf-pace 60` starts frames at the window's 60 Hz.
  `--render-scale S` renders every frame (also captures and films) at a fixed
  scale, upscaled, for quality comparisons.
- **Pop check**: `fuelview --pop-check` drives the same race without
  rendering and compares every frame's draw decisions with the last frame's:
  an object in view (3 px or more at 3420x2224) that appears or vanishes at
  more than a quarter of its alpha, jumps in alpha, or changes mesh by any rule
  but FUEL's own (Lod_Z levels, AutoMesh cells, decor chunks) fails the run.
  FUEL fades objects over the last 20 m of their view distance, and trees and
  bushes cross-fade from mesh to card between 117.5 and 162.5 m.
  `--pop-csv FILE` lists every pop; `FUEL_POP_LEGACY=1` shows the older
  behaviour for comparison.
- **`fuelscene`** lists the vehicle variants in an archive and prints how each
  one is assembled.
- **`fuelcheck`**, a format checker. It decodes every texture, mesh and
  material in the game data and reports what parses (all of them, for the
  archive version the game loads).
- **`fuelsaveimport`** brings a career over from the Windows game:
  `fuelsaveimport --data GAMEDIR IN.sav OUT.sav` reads the original
  `Documents\My Games\...\<profile>\<name>.sav` (it never writes to it),
  checks it and unpacks it the way FUEL does, and writes a FUEL Port
  `career.sav` to OUT. OUT must be given; an existing OUT is kept unless
  `--force`. Point `FUEL_CAREER_SAVE` at OUT, or copy it to
  `~/Library/Application Support/FUEL Port/career.sav` yourself.

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
- **Wind debris (a deliberate deviation from FUEL):** `wind_debris = smooth`
  (the default) makes the windy-weather leaves and twigs drift at 3-6 m/s and
  swirl gently, fading at the edges of their box round the camera; FUEL itself
  moves them about 40 m every frame (notes p88-tank-motion.md,
  p90-debris-smooth.md). Within 100-200 m of a tornado they whirl as in FUEL.
  `wind_debris = fuel` restores FUEL's motion everywhere (`FUEL_WIND_DEBRIS=fuel`
  for one run).
- **Dynamic resolution:** `dynamic_resolution = on` (the default) renders the 3-D
  scene below the screen's resolution, down to `dynamic_resolution_min` (0.8 per
  axis by default), only when a frame's GPU time would miss 60 fps with headroom,
  and goes back to full resolution when there is room; the result is upscaled
  with a sharpened bilinear filter (`dynamic_resolution_upscaler = metalfx` uses
  MetalFX spatial scaling instead, sharper but 3.3 ms a frame at 3420x2224 on an
  M2, more than it saves there). The HUD and menus are always at full resolution.
  `dynamic_resolution = off` keeps full resolution.
- **MetalFX temporal upscaling (port option, off by default):**
  `dynamic_resolution_upscaler = metalfx-temporal` (GRAPHICS page: Upscaler >
  MetalFX temporal; `--upscaler metalfx-temporal` for one run) rebuilds the
  full-resolution frame from several lower-resolution ones: the 3-D scene is
  drawn with a sub-pixel camera jitter and writes motion vectors, and MetalFX's
  temporal scaler upscales the HDR scene before the tone map. While it is
  selected every frame goes through it, including full-resolution ones (then it
  is temporal anti-aliasing), whenever dynamic resolution or `--render-scale` is
  on. Particles, smoke, rain and the GPS arrows have no motion vectors and are
  marked "reactive" so they do not smear. Teleports, respawns, cut-scene cuts
  and the pause menu restart its history. It needs MetalFX temporal (macOS 14.4+,
  Apple GPUs); otherwise the sharpened bilinear is used. FUEL_TAA_NO_JITTER=1
  turns the jitter off for A/B shots. With any other upscaler none of this runs.
- **Film grain and soften (deviations from FUEL, on request):** FUEL's own
  post chain has no grain, no soften and no grade pass (its look is the HDR
  chain the port runs as is, plus the VIGNETTAGE picture its HUD layer draws
  over the whole screen in every in-game frame, cut-scenes included; notes
  p102-post-look.md). `film_grain = off | light | strong` (default light) adds
  a fine monochrome grain after the tone map, new every frame, strongest in the
  dark mid-tones and none in black or white; `soften = off | light | strong`
  (default off) passes the scene through a 3x3 tent before the tone map, for
  the softer look of video captures. `FUEL_FILM_GRAIN` / `FUEL_SOFTEN` override
  them for one run (captures and films without `--in-game` have both off).
- **Shadow edges:** `shadow_filter = 2x2` (the default) filters the sun shadow's
  edges over 2x2 shadow-map texels, a little harder and cheaper; `3x3` is FUEL's
  softer 3x3 filter.

## Legal

FUEL and its content belong to their rights holders. This project is not
affiliated with or endorsed by Codemasters, Electronic Arts or Asobo Studio. It
ships no game code or data and does not bypass any copy protection; you need
your own copy of the game to use it.
