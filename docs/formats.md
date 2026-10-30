# FUEL data formats

What the port knows about the PC version's files so far. All integers are
little-endian unless noted. "Sector" means 0x800 bytes.

## IDs

Class IDs and resource IDs are a CRC of a string: the class name (for example
`Mesh_Z`) or the resource's source asset path (for example
`DB:>TEXTURES>BILLBOARDS>1x2>BILLBOARDCR71_S.TIF`). The CRC uses the MSB-first
CRC-32 table for polynomial 0x04C11DB7, but is driven like a reflected CRC
(`crc = (crc >> 8) ^ T[(lower(b) ^ crc) & 0xFF]`), starts at 0, has no final
xor and folds A-Z to lower case. See `asoboCrc` in `src/core/resources.cpp`.

## `.DPC` archives

| Part | Content |
|---|---|
| Header (sector 0) | 0x000 version string (the game accepts only `v1.381.67.09 - Asobo Studio - Internal Cross Technology` and hangs on any other); 0x104 block count; 0x120 up to 64 block descriptors of 0x18 bytes (object count, padded size, data size, buffer offset, first ID, 0); 0x720 manifest size in sectors; 0x724 manifest position in sectors |
| Blocks | From sector 1, one after another; each holds objects packed back to back |
| Manifest | Streaming table for the pool (optional) |
| Pool | One object per sector-aligned slot, from the end of the manifest to EOF; holds the bodies of streamed textures, meshes and sounds, some stored more than once |

Object header (0x18 bytes): data size (link + stored body), link size,
decompressed body size, compressed size (0 = stored raw), class ID, resource ID.
The link header follows, then the body. A block object with both sizes 0 has
its body in the pool.

### Compression

A compressed body starts with `u32 decSize, u32 compSize`, then a bit stream.
Each group starts with a big-endian u32 of flags; its low 2 bits (`shift`) set
the split between length and distance, and bits 31..2 describe 30 items, most
significant first. A set bit is a big-endian u16 `v`: copy
`(v >> (14 - shift)) + 3` bytes from `(v & (0x3FFF >> shift)) + 1` bytes back
(the ranges may overlap). A clear bit is one literal byte. Stop once the output
reaches `decSize`.

## Bitmap_Z (textures)

The 32-byte link header describes the texture: +4 u16 type (0 = 2D, 2 = cube),
+6 width, +10 height, +14 body size, +0x1B mip count - 1, +0x1E format code,
+0x1F alpha kind (0 opaque, 1 alpha test, 2 alpha blend). The body is the raw
mip chain, largest level first, linear (not swizzled); a cube map stores six
faces. Format codes: 0x07 A1R5G5B5, 0x08 R5G6B5, 0x0A A4R4G4B4, 0x0C A8R8G8B8,
0x0D R8G8B8, 0x0E DXT1, 0x0F DXT3, 0x10/0x11 DXT5, 0x14 R32F,
0x17 A16B16G16R16F, 0x18 A32B32G32R32F, 0x1A L8. The shipped data uses DXT1,
DXT5 and three A1R5G5B5 textures.

## Material_Z

172-byte body: +0x00 diffuse RGB, +0x0C opacity, +0x10 emissive RGB,
+0x20..0x37 and +0x40..0x4B UV transform, +0x38 u16 texture-slot mask, +0x3C
flags, +0x50 specular RGB and power, +0x60 four parameters, +0x70 UV offset,
+0x78 UV scale, +0x80/+0x84 Fresnel reflectance at normal and grazing angles
(nonzero turns on reflections, mask bit 8), +0x88 nine texture IDs. Slots: 0 diffuse, 1 envmap,
2 occlusion, 3 normal, 4 specular, 5 normal (local), 6 dirt, 7 added normal
(local), 8 unused. Flags: bits 0-4 select a special material type (17 is used
for paintable vehicle bodywork, drawn with a white diffuse texture), 0x80 skips
drawing, 0x100/0x200 clamp U/V (0x400/0x800 border), 0x3000 unlit. The texture
mask selects the shader variant (`+bSpecular`, `+bNormal`, `+bDirt`, ...).

## Vehicle paint and liveries

Material codes (flags bits 0-4) 17-20 are car parts: 17 painted with a
two-state sheet, 18 plain, 19 painted, 20 two-state sheet. A two-state sheet
holds the intact part in its left half and the damaged part in its right half,
so intact parts sample the diffuse and normal maps at (u/2, v). Painted parts
mix the paint colour under the diffuse texture by its alpha
(`rgb = mix(paint, tex.rgb, tex.a)`, alpha 0 = paint) and are drawn opaque.

Palettes: `LocalizedComponent/GameTsc/Story/VehiclesDef.tsc` has ten
`AddVehicleColor MESH_ID_<vehicle> R<r>G<g>B<b>` lines per vehicle; the paint
colour is RGB/255. Liveries: each vehicle DPC has MaterialObj_Z libraries
(body `u32 groups; {u32 CRC(vehicle name); u32 n; u32 MaterialAnim_Z[n]}`).
The MaterialAnim_Z whose link is CRC(`LIVERY_<nn>_LOD<k>`) points (body +0x3C)
to a Material_Z whose slot-0 texture is livery nn for LOD k. That texture
replaces slot 0 of the vehicle's materials whose link is
CRC(`LIVERY_XX_LOD<k>`), which otherwise use a plain white texture.

## Mesh_Z

The link header is a 98-byte object part (+4 MeshData_Z ID, +8 bounding sphere
xyz/r, +92 flags) plus a dependency list. The body, in order:

1. four point arrays (`u32 n` then n×12, n×16, n×8, n×12 bytes)
2. groups: `u32 n`, each `u32 k; u16[k]; u32; u32`, plus n×u32 when link flag 2 is set
3. `u32 n`, each `u32 k; k×8 bytes`
4. `u32 n; u32 materialId[n]`
5. arrays of 0x20, 8 and 0x20-byte records
6. vertex buffers: `u32 n`, each `u32 count, u32 stride, u32 flags, count×stride bytes`
7. index buffers: `u32 n`, each `u32 count, u32 flags, u16[count]`
8. array of 0x3C-byte records
9. draw batches: `u32 n`, each a 0x30-byte header and `u32 k; k×0x1C bytes`
10. arrays of 0x20, 4 and 2-byte records
11. morph targets: `u32 n`, each `u32 len; name[len]; u32; u16; u32 k; u16[k]; u32 j; j×8 bytes`
12. array of 6-byte records

A batch header maps onto one Direct3D 9 `DrawIndexedPrimitive`: +0 VB index,
+4 IB index (out of range = non-indexed), +0x10 u16 primitive type (4 =
triangle list), +0x12 u16 vertex type, +0x14 u16 min vertex, +0x18 vertex
count, +0x1C start index, +0x20 primitive count, +0x24 base vertex, +0x28 start
vertex (non-indexed), +0x2C u16 stride, +0x2E u16 material index (0xFFFF =
none).

Vertex types: 0 static (stride 36), 2 one-bone skin (48), 4 four-bone skin (60),
7 water, 8 morph (stride 36 plus a stride-12 delta stream). All drawable layouts
share float3 position at 0, a D3DCOLOR normal at 16 (read as BGRA, then
`*2 - 1`) and float2 uv0 at 20; static vertices add a D3DCOLOR tangent at 12 and
uv1 at 28.

## Coordinate system

Right-handed, y up, metres. With right-handed view and projection matrices
(clip z in 0..1), text on textures reads correctly and front faces wind
clockwise on screen, which is Direct3D 9's default culling.

## World and terrain

`WORLD/USA1.DPC` is the world: metres, y up, about 140 km square centred on
the origin. Its GenWorld_Z body starts `u32, u32, u32 road graph, u32 n,
u32 Binary_Z map[n]`; map 0 is the heightfield: 4668x4668 12-bit samples
about 30 m apart (`x = (col - 2334) * 140000 / 4668`, same for rows and z),
10000 m for the full value range (2.44 m per step). The Binary_Z body is
`u32 size`, a header (`u32 width, u32 height, f32, i32, f32 maxValue`), then
`u32 nA, u32 K`, block data and one u32 per 4x4 block: bits 0-11 a base
value, bits 12-31 a block index. Blocks below nA store sixteen 4-bit offsets
(8 bytes at 8 + 8*index from the start of `nA`), the others sixteen byte
offsets (16 bytes at 16*index + 8 - 8*nA). The engine smooths heights with a
cubic B-spline and adds procedural detail at run time.

The 82 `USA1_*.DPC` sub-worlds (courses, hubs, dressing sets) hold only
Node_Z placements whose world matrices are final; their meshes live in
USA1.DPC. WorldRef_Z objects in USA1 give each sub-world's bounds.
A Node_Z whose UserDefine_Z (body offset 0x18) has the line `FITTOGROUND`
(also spelt `FITOGROUND`) is moved at load so its height equals the
terrain's at its x, z; `GROUNDNORMAL` also tilts it to the terrain normal.
About 2,700 USA1 nodes and 1,166 sub-world placements carry the flag.

## GenWorld_Z procedural layers

After the map list the GenWorld_Z body continues: `u32 n, ground texture[n]`,
`u32 n, Material_Z[n]`, `u32 seed`, `u32 groups` of `{u32 len, name, u32,
u32 n, Node_Z[n]}` (TREES, BUSHES, ROCKS), `u32 n` flat surfaces of 0x83
bytes, then further tables.

Seeded permutation: the engine shuffles 0..255 from the seed (USA1: 41).
Start from a fixed 256-entry shuffle P; for d = 0..255 with seed bytes
s0..s3: `e = P[(d+s0)&255]; e = P[(e+d+s1)&255]; e = P[(e+d+s2)&255];
k = (e+d+s3)&255`, take the first still-unused entry from k upwards
(wrapping) as RT[d] and mark it used. Positions hash to RT entries with
`RT[round(x*12.317513 + z*10.193578 + offset) & 255]`.

Fractal noise (used for forest clearings and flat-surface edges): a
midpoint-displacement value noise over `levels` halvings of a `size`
metre cell. Top-cell corners are `RT[...]/255` from a seed-offset hash;
each edge midpoint is `m + (a*(hi-lo)/255*m + |A-B|/510 + a*lo/255) *
(RT[h]-127.5)` and each centre the mean of the corners plus a similar
term, with `a` halving per level; the finest cell is bilinear.
Clearings: 3 levels of 500 m, amplitudes 0.025 / 0.15, gain 0.5.

Trees and bushes (no positions are stored):
- One candidate per 20 m cell at (20i, 20j), moved by `(b & 15) * 1.2` m
  in x and `(b >> 4) * 1.2` m in z, b = RT hash with offset 33.13 (trees)
  or 37.82 (bushes).
- Density d = S(2F-1)/2 + 1/2 - 0.95 * (S(2G^2-1)/2 + 1/2), clamped, with
  F the forest map (slot 7, cubic B-spline), G the clearing noise and
  S(a) = a <= 0 ? (a+1)^8 - 1 : 1 - (1-a)^8; faded to zero on slopes past
  45 degrees. A tree is kept if RT[hash]/255 < d; bushes skip this test.
- Rejected within 10 m of a road closer than the road type's clearance
  (6-32 m), above 3000 m, on slopes over 30 degrees, on rocky ground and in
  fields (not ported), and within 5 m of placed objects.
- Species: each TREES / BUSHES Node_Z leads to a Lod_Z whose last u32 is a
  UserDefine_Z script `"GenWorld Arbre"` of items `{ "<date key>" min max }`
  in percent. Keys: `12/11/2007 19:25` size range, `19:29` ground
  alignment, `19:30` tilt (degrees), `20:01` weight, `19:32` humidity
  (map 2), `19:34` density d, `17/04/2008 19:32` stormed (map 14),
  `19:33` incinerated (map 13), `19:34` sanded (map 16), `19:35` flooded
  (map 15), `19:36` high-ground factor (0 below about 2500 m). Ranges are
  widened by 0.001 each way. Among the species whose ranges all hold, one
  is picked by weight with the same RT[hash]/255 value (scaled by the
  weight total when that exceeds 1, else possibly none).
- Instance: four chained draws r0..r3 (r0 hashes x + 425.23886, the next
  ones x + 931.9283 * previous). Scale = 6 * lerp(max, min, r0) times the
  mesh, so 3..6 for every USA1 species; yaw = r3 * 6.28; up axis blended
  towards the terrain normal by the alignment, then tilted by
  (2r-1) * tilt on two axes; set 0.25 m into the ground.
- LOD: chunks below half the tree view setting draw meshes, below it
  meshes and impostor cards, beyond it cards only (setting values unknown).

Grass is not a mesh scatter: on a 1 m grid the engine draws camera-facing
tufts from the Grass material's 4x4 atlas where the per-point ground data
has enough grass (the port approximates with low meshes by surface type).

Flat surfaces (0x83 bytes): u32 id, axis X (+4), plane normal (+0x14),
axis Z (+0x24), centre (+0x34) as float4s; +0x4c blend margin (metres,
inside the shape), +0x50 strength, +0x44/+0x48 scale and offset for an
optional height map (index u16 at +0x78, 0xffff = none), +0x7d priority
(default 0x80), +0x7f adds 0.1 of a noise, +0x82 shape (0 rectangle,
1 ellipse). Distance inside: rectangle `min((1-|u|)|X|, (1-|v|)|Z|)` with
u, v the coordinates along the half-axes (corners on the -Z side rounded
off by 15 m), ellipse the distance to the boundary along the radius;
minus an edge jitter of 5 m times two small fractals. At a point the
covering surfaces are taken by ascending priority; a priority change
commits the height so far. Each adds `w((1-w)h + wP)` and `w` to two
sums, w = (1/2 + c/2) * strength, c the 1.25-power S-curve of
clamp(distance / margin), P the surface's plane height; the height is
sum1 / sum2 once the weights exceed 0.01.

## Races

`LocalizedComponent/GameTsc/Story/miss_official.tsc` (career) and
`missChallenge.tsc` (hub challenges) hold `EMD_Begin` .. `EMD_End` blocks:
`EMD_SetName`, `EMD_SetNbLoop` (laps), `EMD_SetTimer`, `EMD_SetRaceMode`
(0 A to B, 1 checkpoints, 2 knock-out, 3 time trial, 4 circuit, 5 baja,
6 raid), `EMD_SetStartPos/Rot`, `EMD_SetEndPos/Rot` and one
`EMD_SetCheckpoint x y z rx ry rz 1.0 flag` per checkpoint, in order.
Rotations are Euler angles in radians; ry is roughly the heading. The
scripts give no checkpoint width.

## Sound_Z (sounds)

Each sound has a 14-byte link record: `u32` name CRC, `u32` sample rate (always
44100), `u32` size in bytes, `u16` flags. Flag bit 1 (value 2) means the sound
loops (the whole buffer; there are no loop points), bit 2 (value 4) means
stereo. The body is raw signed 16-bit little-endian PCM, LZ-compressed like
other pool bodies when the object says so, so `DpcFile::body` already returns
the samples. Looping roles all have bit 1 set and one-shots do not.

A vehicle's sound bank is in the Lod_Z named `<VARIANT>_SKEL`. After its points,
object list and two header words come two optional pair lists; the second holds
`(symbol, Sound_Z ID)` pairs (78 for bikes, 86 for four-wheelers). Symbols are
the game's sound slots: 2 engine idle, 3..7 engine layers when off the throttle,
9..13 the same layers on the throttle, 0x0e..0x1e skids, 0x20..0x39 rolling
(a slow and a fast loop per ground class), 0x40 wind, 0x42..0x4b impacts.

Engine mixing uses VehiclesParam values: x = (rpm - P325) / (P324 - P325),
clamped to 0.001..0.99. The idle sample fades out over x < 0.15; layers 1..5
are Hann windows overlapping by half, spread by P644, with pitch running
piecewise-linear through the neighbouring layers' recorded rpm (P636..P642), so
the x axis is a virtual rpm rather than the real one. Weights at each x are
normalised to P649. On-throttle layers play at weight times load, off-throttle
at weight times (1 - load).

Other driving sounds (agent Q's decode of the game's sound updaters): a ground
class table gives each of 18 classes a slow and a fast rolling loop and a skid
loop (class: slow/fast/skid symbols; 0: 20/21/0e, 1: 2c/2d/10, 2: 2e/2f/11,
3: 22/23/0f, 4: 30/31/12, 5: 32/33/13, 6: 34/35/14, 7: 36/37/15, 8: 38/39/16,
9, 10: 29/-/1a, 11, 12: 27/28/19, 13..17: 26/-/18). Rolling fades slow to fast
over 4..40 m/s. 0x47 is the scrape loop (body sliding against something),
0x54..0x59 suspension thumps plus 0x5a a big one (P651 = compression speed
threshold, P652 = scale), 0x5e..0x61 backfire pops (P645 interval, P646 rpm drop
rate threshold, P653 throttle ceiling, P654 normalised rpm floor).

## Surfaces under the wheels

Port behaviour (`core/surface.cpp`); the engine's own 18 sound classes and its
per-class drive multipliers (P246..P263) are not named in the data, so the
mapping below is chosen by the classes' gravel rate and sound character.
The surface type is the ground map's (40 types) overridden by roads: within
half a road's drawn width of a tarmac line (types 10, 11, 12, 15, 18, 19) it is
ASPHALT (11), of a dirt road or trail DIRT_FLAT (13).

| category | tyre friction x | rolling resistance | surface types | sound class |
|---|---|---|---|---|
| tarmac | 1.15 | 0 | ASPHALT, ASPHALT2 | 0 |
| dirt | 1.00 | 0.01 | EARTH(2), DIRT_*(3 flat, 4 rough), PEBBLE(5), CRACKED, STEEP_EARTH, ROUGHEARTH(4) | 2, 3, 4, 5 |
| grass | 0.90 | 0.03 | GRASS, GRASS2, STEPPENL, HILLSNL, PLAINNL | 7 |
| sand | 0.75 | 0.08 | SAND, SAND2, DESERTNL, ROUGHSAND* | 8 |
| snow | 0.60 | 0.05 | SNOW, SNOW2 | 9 |
| mud / wet | 0.65 | 0.07 | PLOUGH(4), HUMIDEARTH*, WETSAND* | 11 (PLOUGH 4) |
| rock | 0.95 | 0.01 | STONE*, ROCK*, ROCKNL, BIG_STONE*(6), STEEP_STONE(6), ROUGHROCK*(6) | 1 (big stone and rough rock 6) |

Each wheel scales both friction impulses of Jolt's tyre model by its own
smoothed multiplier (looked up at the wheel's contact point every 1/60 s).
Unknown types count as dirt. A bank without a class falls back to class 2.

## Skinning and the rider

Every vehicle carries the same pilot, `IT_PILOT_SKIN` in
`CommonComponent/DATAS/PILOT.DPC`. Its Lod_Z (`9a9c3e61`) lists one Skin_Z per
LOD and, in its first pair list, `(slot, Animation_Z)` pairs. The Skin_Z names
17 part meshes (Head1-9, Chest1-5, Legs1-3; the mesh link name is the CRC of
the part name) and a 57-bone Skel_Z. The default rider draws Head1, Chest1 and
Legs1.

Skin_Z body, after the mesh list and per-bone tables: one set per mesh, one
0x60-byte record per draw batch of that mesh. Record word 0 is the batch's
material name; words 1-21 are a palette of bone-name CRCs for 21 slots, where
`ffffffff` keeps the slot from the previous batch of the same mesh and
`fffffffe` ends the list. Skinned vertices are 60 bytes (position, tangent and
normal as D3DCOLOR at 12 and 16, uv at 20, four float blend indices at 28, four
float weights at 44) or, for rigid helmets, 48 bytes (one index at 28, one
weight at 44). A blend index is six times the palette slot (the original
shaders spend six constant registers per bone). Vertices are in the model space
of the bind pose, so a bone's skin matrix is `inverse(bind world) * pose world`
(row-vector convention) and a vertex is the weighted sum over its bones.

Skel_Z bones store local scale, rotation and translation in a 0xA0-byte block
(quaternion at +0, scale at +0x10, translation at +0x20) plus the bind world
matrix. A bone's local matrix is `Scale * Rotation * Translation` and its world
matrix is `local * parent world`.

Animation_Z: duration, then arrays of rotation keys (`f32 time`, four int16
divided by 2000, xyzw), scale and translation keys (`f32 time`, three int16
divided by 4096, plus tangents) and per-bone tracks (28 bytes: bone name CRC,
then 16-bit start/count pairs into the translation, rotation and scale arrays).
Bones without a track keep their bind transform. Animation slots come in
blocks, one per vehicle archetype (the last word of `AddVehicleInfo` in
VehiclesDef.tsc: for example QUAD, MBIKE_CHOPPER, BUGGY_ROAD, TRUCK); this port
picks the neutral seated pose of the block (hips centred between the hands,
both feet off the ground). The rider sits in the vehicle's own space, so it
needs no offset.

Livery: batch materials named `LIVERY_XX_<PART>` (and `LIVERY_XX_BODY` for the
suit) take their diffuse and specular bitmaps (slots 0 and 4) from the
Material_Z that the MaterialAnim_Z `LIVERY_<nn>_<PART>` points to (body +0x3C).
The default rider uses livery 01 throughout.

## Movable props (port behaviour, not a file format)

The world DPCs carry no mass or "breakable" data that the port decodes, so the
knock-aside rule is a heuristic: a placed mesh whose collision radius (bounding
sphere around its origin) is 0.4 to 3 m is a dynamic prop; 3 m and larger is a
static solid. Mass comes from the convex hull volume (hulls overstate hollow
props, so it is capped relative to the vehicle). The physics keeps tyre casts
on a separate layer that sees only static ground, so tyres never ride up props.
