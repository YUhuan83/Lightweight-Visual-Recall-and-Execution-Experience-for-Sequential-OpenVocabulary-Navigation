# Native Italy + D-Series Scene

Implemented in this directory only. Uses installed BeamNG.tech 0.39.4.0 and
BeamNGpy **1.36**, installed in `.beamng-venv` with global Python 3.13.
No original environment, game archive, editor save, or license file was edited.
No license file was read. Runtime uses localhost only, without cloud services.
The one-time dependency installation used PyPI; running this scene is offline.

## What Exists

- Existing native Italy terrain and road network, not a newly built terrain.
- Native four-wheel Gavril D-Series `pickup`, factory automatic 4WD configuration
  `vehicles/pickup/d15_4wd_A.pc`. Its config metadata explicitly says `4WD`.
  This is an existing native car, not a newly authored JBeam vehicle.
- Official `spawn_crossroads` position `[729.62677, 763.914001, 177.753006]`.
  Rotation is derived from its shipped rotation matrix and round-trip checked.
- Independent 0.7 x 0.7 x 0.5 m cube obstacle and radius 0.3 m, height 1.5 m
  cylinder visual target, using supported procedural meshes. Both are solid
  native mesh objects, not dynamic loose stones. They are more than 15 m from
  the spawn, on the short road strip leading south from the crossroads.
- Placement interpolates actual nearby road nodes, including their height.
  No large prefab or replacement surface covers the car. Visual clearance,
  exact ground contact and collision behavior still need in-game inspection.
- The preview road is existing asphalt, **not newly built cement/cobblestone**.
  No decal is presented as physical pebbles. Italy supplies the larger rural
  environment; a custom physical cobblestone route is not part of this delivery.

`generated/provenance.json` contains original metadata, per-member SHA-256,
archive paths, chosen pose, road nodes and calculated object positions.
Only small named archive members are read; game archives are never extracted.

## Run

Run from the shared workspace in PowerShell. All scene commands require no
internet. `run.ps1` forwards arguments to the dedicated Python environment.

Generate native files without contacting or changing the game:

```powershell
powershell -ExecutionPolicy Bypass -File .\beamng_scene\run.ps1 --offline
```

Connect to an already running simulator with TCOM enabled, explicitly consenting
to replace its current runtime world:

```powershell
powershell -ExecutionPolicy Bypass -File .\beamng_scene\run.ps1 --replace-current
```

If no simulator is running, explicitly launch and load the scene:

```powershell
powershell -ExecutionPolicy Bypass -File .\beamng_scene\run.ps1 --launch --replace-current
```

`--launch` refuses if any BeamNG process is already running. It never closes or
restarts a process. If your existing game lacks TCOM, save any editor changes and
exit it yourself before using the launch command. Alternatively start its normal
executable with `-tcom 64256 -tcom-listen-ip 127.0.0.1` and use connect mode.
The script never loads a world without `--replace-current`, even from a menu.
That flag replaces the runtime world, so save unsaved editor work first.

Default sensors: front RGB camera 640x360 at requested 10 Hz, AdvancedIMU at
100 Hz physics sampling. Latest snapshots are polled at approximately 5 Hz.
This is a preview, not a synchronized dataset recorder.

```powershell
# Lowest-load world/vehicle check, no sensors:
powershell -ExecutionPolicy Bypass -File .\beamng_scene\run.ps1 --replace-current --no-sensors --seconds 5
# Front + left + right cameras, IMU and roof LiDAR:
powershell -ExecutionPolicy Bypass -File .\beamng_scene\run.ps1 --replace-current --full-sensors
```

`--side-cameras` and `--lidar` independently extend the default sensors.
`--no-objects` gives car-only preview. `--seconds 10` ends capture after ten
seconds; without it, Ctrl+C ends capture. Sensors are removed on normal exit;
the scenario and game remain open. The car starts braked, with zero throttle.
No AI or autonomous driving is enabled. Release brakes yourself for manual driving.
Sensor mounting offsets are vehicle-local preview defaults, not calibrated mounts.

Latest data are written to `captures/<unique-scenario-name>/`: `front.png`,
optional `left.png`, `right.png`, `lidar.npy`, and `imu.json`.
Each runtime load creates a new `nizhen_italy_dseries_preview_<id>` namespace
using `Scenario.make`, leaving previous scenarios and saves untouched.

## Native Files

`generated/levels/italy/scenarios/nizhen_italy_dseries_preview/` already contains
the native scenario JSON and vehicle prefab JSON generated through the pinned
`Scenario.make` serializer using an offline payload writer. These are actual
native files, not just a plan. The writer mirrors the installed Lua handler's
one-element JSON array and `.prefab.json` format.

The offline prefab contains the **car only**. Procedural objects are placed by
BeamNGpy when the scenario connects, and sensors are created after start.
`generated/runtime_objects.json` records those runtime objects. Use the script
for the full scene; copying the prefab alone does not create objects or sensors.
No mod was copied into your user folder manually.

## Verification

- Dependency installation and `pip check` succeeded on Python 3.13.
- Offline generation succeeded against the actual installed content archives.
- Local launch reached `Scenario.make`, `scenario.load`, `scenario.start` and
  the brake command, then wrote the native scenario path to
  `captures/nizhen_italy_dseries_preview_2bc8d69e85/scenario_path.txt`.
- That terminal invocation was interrupted by the execution tool. It did not
  produce a clean completion result. A later default-sensor attempt refused
  safely because localhost port 64256 no longer had a listener. No second game
  was launched and no running game was closed.
- Consequently camera/IMU/LiDAR output, visual appearance, and driving/collision
  behavior are **not claimed as runtime-verified**.

Run local tests:

```powershell
.\beamng_scene\.beamng-venv\Scripts\python.exe -m unittest discover -s beamng_scene -p test_scene.py -v
.\beamng_scene\.beamng-venv\Scripts\python.exe -m compileall -q beamng_scene\scene.py beamng_scene\test_scene.py
```

If rebuilding only this environment is ever needed:

```powershell
py -3.13 -m venv .\beamng_scene\.beamng-venv
.\beamng_scene\.beamng-venv\Scripts\python.exe -m pip install --index-url https://pypi.org/simple -r .\beamng_scene\requirements.txt
```

## Sources

- Installed `content/levels/italy.zip`: `levels/italy/main/MissionGroup/spawnpoints/items.level.json`.
- Same archive: `levels/italy/main/MissionGroup/decalroads/asphalt/road_details/items.level.json`,
  road persistent ID `d638a2ec-6b50-44e9-a5a9-6445e17a4d8a`.
- Installed `content/vehicles/pickup.zip`: `vehicles/pickup/info_d15_4wd_A.json`
  and `vehicles/pickup/d15_4wd_A.pc` (legacy native syntax, intentionally not rewritten).
- [BeamNGpy v1.36 scenario implementation](https://github.com/BeamNG/BeamNGpy/blob/v1.36/src/beamngpy/scenario/scenario.py),
  `make`, `add_procedural_mesh`, `connect`.
- [BeamNGpy v1.36 procedural API](https://github.com/BeamNG/BeamNGpy/blob/v1.36/src/beamngpy/scenario/procedural.py).
- Installed `.beamng-venv/Lib/site-packages/beamngpy/sensors/`: `camera/camera.py`,
  `advanced_IMU.py`, `lidar.py`, verified constructor signatures and poll methods.
- Installed game `lua/ge/extensions/tech/techCore.lua`: `handleCreateCylinder`
  line 2662, `handleCreateCube` line 2710, `handleCreateScenario` line 3221.
- Installed game `lua/ge/extensions/util/trackBuilder/proceduralPrimitives.lua`:
  centered cylinder geometry line 66, cube line 472, built-in fallback material
  `track_editor_A_border`. Object Z offset uses half-height accordingly.
