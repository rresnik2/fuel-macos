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
