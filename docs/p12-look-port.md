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
