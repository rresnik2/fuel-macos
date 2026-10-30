# P12 — look & feel port (post-processing, camera, dust, atmosphere)

(Meant for ~/re-work/fuel/notes/p12-look-port.md; the agent's worktree sandbox could not write
there, so the notes live here.)

Reference frames: ~/re-work/fuel/notes/ref-look/

## Before (commit a5d618b)
- Bright lime grass, pale blue sky, sun high (elevation ~48 deg), flat lighting, ACES tonemap in
  every object shader, no post, FOV fixed 0.70 rad (40 deg), camera orbits the vehicle centre at
  --distance radii, so the vehicle sits in the middle of the frame.

## Plan
1. Renderer: scene -> RGBA16F MSAA + depth, resolve both; dust pass on the resolved target; bloom
   chain (1/2 .. 1/32), quarter-res light shafts, composite (radial blur, bloom, glare, ACES,
   grade, vignette). New MeshRenderer::render(cb, target, cam) used by the window and --shot/--film.
2. Chase camera helper shared by the window and --film (speed FOV, lag, low aim point).
3. Dust: CPU particles from VehicleState wheels/surface/skid, instanced soft sprites.
4. Low warm sun, warm haze, desaturated ground.

## Progress log
- b7593b9 HDR + post. MeshRenderer::render(cb, target, cam) does shadows, scene (RGBA16F, 4x MSAA,
  memoryless MSAA attachments, colour + depth resolve with depth filter Min), dust, bloom chain
  (bright pass at 1/2 with soft threshold 1.4/knee 0.7, 4x4-box downsamples to 1/32, 3x3 tent
  upsamples added back), light shafts at 1/4 (28 steps towards the sun through the sky mask kept
  in bloom alpha), composite (10-tap radial blur, bloom 0.10, shafts 0.55, sun glare scaled by sky
  share around the sun from the 1/32 level, Narkowicz ACES, display-space grade, vignette, dither).
  Object shaders now output linear HDR; haze stays in-scene. MTKView is single-sample, no depth.
  Sun: elevation ~18 deg, colour (1.0, 0.74, 0.46) x 4.2; haze 1/3600 per m, max 85 %, coloured by
  the sky near the horizon in that direction (warm glow towards the sun).
- b5b669a ChaseCamera (drive.h): 2.4 x vehicle radius behind (min 3.4 m), 0.36 x that up, aims
  0.2 x above the focus; FOV 56 -> 68 deg with speed (20..160 km/h); lags the heading (2.5..5 /s),
  surges back on acceleration; focus height smoothed. Window: mouse drag/scroll offset it.
  --film uses the same camera stepped at 60 Hz; --yaw (if given) orbits it (0 = behind).
- Dust (DustSystem, drive.h): per wheel on the ground, rate by grip category (dirt 34/s, sand 40,
  snow 18, rock 10, mud 7, grass 3, tarmac 0.6) x (speed 4..80 km/h + 1.5 x slip once moving);
  particles grow to 2-4 m, drag 1.8/s, drift up; sprites lit as a scattering cloud with
  Henyey-Greenstein g=0.55 forward scattering, soft against resolved depth, fade near the camera.
  Opponents emit too (within 250 m of the player).
- Grade iterations: first pass was too sepia/washed; final WB (1.03, 1, 0.94), contrast 1.22,
  sat 1.0, split tone cool shadows / golden highlights, ground palette desaturated (greens most)
  and darkened x(0.92, 0.86, 0.76); bluer zenith.

## GPU cost (M-series, FUEL_GPU_BENCH=60, whole frame incl. shadows; CR2-Yell-CP start)
| size | post on | FUEL_NO_POST |
| 1600x1000 | 4.6-5.6 ms | 4.2-4.7 ms |
| 2560x1600 | 7.4-8.4 ms | 5.9-6.6 ms |
Window --record (retina 2560x1600, vsync 60): median gpu_ms 12.4 vs 12.2 before the change
(this counter runs at low GPU clocks), p90 15.8 vs 15.3; dt median 16.7 ms both.

## Tools
- FUEL_GPU_BENCH=N fuelview ... --shot: renders each shot N more times and prints GPU time.
- FUEL_NO_POST=1 or the O key in the window: plain tone mapping only.
