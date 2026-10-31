# Native Rural Course Integration

## Deliverables

- `nizhen_rural.zip`: actual portable native art mod, about 1.72 MB compressed.
- `mod/art/shapes/nizhen_rural/`: eight original Collada meshes, ten original procedural PNG textures, version 1.5 `main.materials.json`, CC0 asset license.
- `mod/art/prefabs/nizhen_rural/course.prefab.json`: native newline-delimited prefab containing eleven TSStatic instances and their SimGroup; no vehicle.
- `map_objects.json`: runtime constructor definitions, world bounds, route nodes, spawn candidates and geometry statistics.
- `route_samples.json`: 2,001 world-space center/left/right samples for integration and ground-contact inspection.
- `asset_provenance.json`: named Italy member hashes and native syntax evidence.
- `build_manifest.json`: package allowlist, triangle counts, local mesh bounds, ZIP digest.

This is a custom native course overlay on existing Italy, not a new terrain/map replacement. Italy remains separately required. No stock Italy files, game art, vehicles, license keys, or absolute machine paths are packaged. No files under `levels/` are packaged, so installation cannot replace Italy level data. Original meshes and textures are CC0; no external textures were needed.

## Runtime API

Run from the workspace root with BeamNGpy available:

```python
import json
from pathlib import Path
from rural_scene.geometry import scenario_objects

manifest = json.loads(Path("rural_scene/generated/map_objects.json").read_text())
for obj in scenario_objects():
    scenario.add_object(obj)
# Vehicle and sensors are deliberately left to the runtime owner.
candidate = manifest["course_spawn"]
# scenario.add_vehicle(vehicle, pos=candidate["pos"],
#                      rot_quat=candidate["rot_quat"])
```

`scenario_objects(manifest_path=None)` returns eleven `beamngpy.ScenarioObject` instances; it performs no simulator connection, installation, scene mutation, or vehicle creation. All `manifest["objects"]` dictionaries can also be passed directly to `ScenarioObject(**definition)`. Constructor compatibility was tested using the workspace's installed BeamNGpy.

Every definition uses `otype="TSStatic"`, `shapeName="/art/shapes/nizhen_rural/<name>.dae"`, `collisionType="Visible Mesh"`, unit scale, world `pos`, and XYZW `rot_quat`. Road/fence meshes use local coordinates with a shared world origin. Small farm props use independent local origins and world placements. Do not add the shared origin again to object positions.

Use EITHER the native prefab `/art/prefabs/nizhen_rural/course.prefab.json` OR `scenario_objects()`, never both: duplicate surfaces cause overlapping collisions. The mod only supplies assets; mounting it alone does not spawn course objects or alter Italy. The prefab follows BeamNG's newline-delimited JSON structure, not a JSON array.

## Placement

The selected road is `4819fd4a-4388-41aa-a9bb-8f68953d6391`, nodes 5 through 13 inclusive, in the same verified road-details member as the original preview. The original preview's damaged-asphalt decal is only approximately 17.1 m long and is not stretched into the new course.

| Property | Value |
| --- | --- |
| Shared world origin | `[749.964172, 745.677979, 178.189575]` |
| Route station length | `166.34603961472916 m` |
| Driving width | `3.6 m` within source road's `7.5 m` width |
| Road mesh world minimum XYZ | `[749.3194657493261, 729.0688575331201, 178.149575]` |
| Road mesh world maximum XYZ | `[902.7525252581795, 773.4370576378195, 180.64414223097276]` |
| Entire authored scene minimum XYZ | `[749.3194657493261, 727.5609470653037, 178.149575]` |
| Entire authored scene maximum XYZ | `[902.7525252581795, 773.4370576378195, 182.1733032911418]` |
| Maximum sampled centerline grade | `0.04999535142068317` |
| Concrete / cobble / concrete stations | `0-45 / 45-95 / 95-166.346 m` |
| Course candidate spawn XYZ | `[757.3994076490783, 742.6754902828627, 179.03860678149348]` |
| Course candidate quaternion XYZW | `[0, 0, 0.5577166374975648, 0.8300314164285649]` |
| Existing verified Italy spawn | `spawn_crossroads`, `[729.62677, 763.914001, 177.753006]` |

Station distances are cumulative source-node XY chord distances, not an assertion of exact cubic arc length. The course spawn is station 8 m, lifted 0.5 m above the authored lane, with its heading derived for BeamNG vehicle forward `-Y`. It is an unverified candidate, not a claim of safe terrain placement. Stations 0-18 have no authored roadside obstacles. The existing crossroads spawn remains available in the manifest with its original verified rotation, but is not aligned with the new course.

## Surface And Limits

Height is shape-preserving cubic interpolation through actual road-node Z values, without segment height overshoot. All road surfaces share the same function and exact boundary coordinates. Road sampling is at most 0.25 m; the 20 mm overlay lift smoothly eases to zero over the first/last 5 m. The cobble section retains a continuous physical mortar bedding so its 6 mm inter-stone joints are not holes. Three thousand individually beveled stones are merged into one 30,000-triangle mesh, rather than thousands of entities. Relief tapers over 2 m at cobble/concrete boundaries and never exceeds 28 mm above bedding. Thin road skirts extend 40 mm below their top; they are not elevated ramps.

The tank, barrels, trough and fencing are static collision targets, not deformable JBeam props. Fences are merged into one mesh and leave access gaps at stations 36-43, 88-97 and 125-132 m. Materials use muted mineral, weathered wood, zinc and olive-painted metal textures, not bright debug primitives. Friction uses existing ground types; there is no custom friction model claim.

IMPORTANT: DecalRoad control nodes are not measurements of terrain or a perfect reconstruction of BeamNG's road spline. Lateral bank, road crown, terrain contact between nodes, existing vegetation/buildings, spawn settling, native Collada import, actual collision response and performance are NOT simulator-verified. The native Italy pavement remains underneath. The 20 mm overlay may intersect original terrain or expose local floating edges where the underlying surface differs. No large designed ramps or internal road seam gaps exist in authored geometry, but absence of visible terrain gaps cannot be guaranteed offline. Do not claim the course is driving-certified or expand it to 300 m without further evidence.

The runtime owner should inspect native loading and terrain contact first, then perform a low-speed traversal (initial recommendation 15 km/h), checking both lane edges, stone boundaries, spawn settling and scenery clearance. Keep autopilot/traffic disabled until that inspection. This implementation intentionally did not run BeamNG or install the ZIP.

## Native Syntax Evidence

Installed BeamNG.tech 0.39.4.0 `lua/ge/extensions/editor/resourceChecker/resourceUtil.lua:1710` explicitly recognizes collision strings `Visible Mesh` and `Visible Mesh Final`. The visible road and stone triangles are therefore also the requested collision surfaces; no separately named low-poly collision mesh substitutes for them. Native loading still needs verification.

Installed `lua/ge/extensions/editor/materialEditor.lua:2537` labels the BaseColor Map editor field `diffuseMap`. However, the shipped Italy `art/road/main.materials.json` material `italy_asphalt_overlay_light` serializes version `1.5` as `Stages[0].baseColorMap`, with `baseColorFactor`, `roughnessFactor` and other PBR keys. Generated JSON uses that shipped serialization, NOT a guessed `diffuseMap` JSON key. Texture files use `.color.png`. Collada material names, material `mapTo` values and packaged texture references match and are tested.

## Build And Installation Contract

```powershell
python rural_scene/build_assets.py
.\beamng_scene\.beamng-venv\Scripts\python.exe -m unittest rural_scene.test_geometry -v
```

The standard-library builder accepts `--italy-zip PATH` for a different installation. It reads only small named archive members in a normal build and refuses a road-member hash mismatch against existing provenance. It does not read keys or extract game art. Generated files are rebuilt, not manually maintained. ZIP packaging uses an explicit authored-file allowlist, fixed timestamps and no workspace recursion.

Installation is reserved for the runtime owner: copy only `nizhen_rural.zip` into the selected BeamNG USER folder's `mods/` directory, never the game installation or stock Italy archive. Resolve the actual user folder explicitly. If that destination already exists, compare its SHA-256 with `build_manifest.json`; treat identical content as a no-op and refuse/ask before replacing different content. Do not delete caches or overwrite other mods. Removing this single mod after unloading dependent scenarios reverses installation. Portable servers must mount the same ZIP and separately provide a licensed Italy base.

Nine offline tests passed, including actual BeamNGpy construction, mesh normals/winding and indices, source height interpolation and grade, shared road seams and width, cobble count/relief, translation invariance, material/texture references, archive allowlist/digest, and candidate spawn heading/obstacle clearance. This is offline validation, not simulator driving validation.
