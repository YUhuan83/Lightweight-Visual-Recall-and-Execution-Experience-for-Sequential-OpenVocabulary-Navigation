# WildOS East Coast Dual Region Overlay

This overlay keeps two experiment areas from the official `east_coast_usa`
level in one pre-cropped package:

- `eca_town_industrial`: town/building experiment
- `eca_sawmill_outdoor`: low-building outdoor experiment near the sawmill

The official terrain, terrain materials, roads, decals and `GroundCover`
definitions remain enabled. Scene and forest instances outside both experiment
regions are removed before level creation. The official source ZIP is never
modified.

## Regions

### Town Industrial

- Spawn: `[734.294, -11.946, 52.154]`
- Rotation Euler: `[0, 0, -80.042]` degrees
- Polygon: `[461.856, -3.856]`, `[734.294, -11.946]`,
  `[835.645, -285.575]`, `[777.200, -461.684]`

### Sawmill Outdoor

- Spawn: `[150.596, -576.816, 41.307]`
- Rotation Euler: `[0, 0, 0]` degrees
- 160 x 160 m polygon: `[70.596, -496.816]`,
  `[230.596, -496.816]`, `[230.596, -656.816]`,
  `[70.596, -656.816]`
- Buildings inside the box: 0

## Runtime API

Use the BeamNG GE Lua console, BeamNGpy GE Lua execution, or another extension:

```lua
wildos_regionCuller.selectTownIndustrial()
wildos_regionCuller.selectSawmillOutdoor()
```

Generic API:

```lua
wildos_regionCuller.selectRegion("eca_sawmill_outdoor")
wildos_regionCuller.getSelectedRegion()
wildos_regionCuller.getRegions()
wildos_regionCuller.teleportToSelected()
```

`selectRegion()` persists the selection in:

```text
/settings/wildos_east_coast_region.json
```

The selected region is restored and applied the next time `east_coast_usa`
loads. Switching regions teleports the current player vehicle immediately.

The two regions share one loaded map overlay, so switching does not load the
complete official East Coast scene again.

## Build and install

```powershell
python east_coast_region/build.py
Copy-Item east_coast_region/generated/wildos_east_coast_region.zip `
  "$env:LOCALAPPDATA/BeamNG/BeamNG.tech/current/mods/" -Force
```

Fully exit BeamNG before replacing the ZIP, then restart and load
`east_coast_usa`.

The full official `TerrainBlock` remains because BeamNG cannot overlay only a
polygon subsection of a `.ter` file. Terrain geometry/material memory is the
remaining whole-map cost; buildings, roads, forest instances, decals and
signals outside both regions are removed before level creation.
