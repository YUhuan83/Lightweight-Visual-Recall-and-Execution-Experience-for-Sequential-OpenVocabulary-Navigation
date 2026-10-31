# Rural Scene Runtime

Direct BeamNG.tech Italy test scene, not full WildOS, ROS integration, an autonomous driver, or a synchronized dataset recorder. The native D-Series remains parked. No simulator is started by installation or tests.

## Asset Contract

The runtime owns no geometry or generated files. `generated/map_objects.json` must be an object with integer `schema_version: 1`, `level: "italy"`, `asset_root: "/art/shapes/nizhen_rural/"`, `course_spawn`, and a nonempty `objects` list. Each object is passed directly to `ScenarioObject(**definition)` using native keys: `oid`, `name` (both unique identifiers), `otype: "TSStatic"`, `pos` (world XYZ metres), `rot_quat` (unit XYZW quaternion), `scale` (three positive numbers), `shapeName` (canonical `.dae` path under `/art/shapes/nizhen_rural/`), `collisionType` (`Visible Mesh` or `Visible Mesh Final`), and optional `groundType` (`ASPHALT` or `WOOD`). Unknown constructor fields are rejected. There is no alternate snake-case schema or generic `fields` adapter. Do not load the packaged prefab as well: the runtime already adds the objects, and doing both duplicates collision surfaces.

The runtime uses manifest `course_spawn.pos` and `course_spawn.rot_quat`, with status `candidate_not_simulator_verified`. The current candidate is at station 8 m, `[757.3994076490783, 742.6754902828627, 179.03860678149348]`. It is structurally validated, NOT runtime driving verified: terrain contact, spawn settling, native collision loading and scenery clearance remain untested. The crossroads spawn is not a fallback. Native `pickup` with `vehicles/pickup/d15_4wd_A.pc` remains embedded from verified vehicle provenance, without any dependency on `beamng_scene/`. `course_config.json` is geometry build input, not a runtime override; rebuild generated assets to change the course. The geometry agent owns the config, manifests, and `generated/nizhen_rural.zip`.

Before any connection, runtime validates the schema version, native fields, object count, referenced meshes, and the zip SHA-256, byte count, CRC and exact file allowlist in `generated/build_manifest.json`. Archive entries are confined to this mod's shape/prefab roots. The installer performs the same archive integrity checks before copying. Runtime reports the scene manifest SHA-256, schema version and mod SHA-256 to the console. The scene SHA identifies the exact loaded manifest; it is not a signed authenticity check or an expected hash supplied by the generator. Offline integration tests also check material/texture references and prefab consistency. Collision behavior still requires simulator verification.

## Install

Use a licensed, compatible BeamNG.tech installation and its Linux Vulkan graphics dependencies/driver. BeamNGpy is pinned to 1.36; simulator compatibility and physical scene appearance must be checked on the target machine. Python 3.10+ is required. No ROS is needed for this sample.

Linux, from this directory (Python 3.10 and its venv support must already be available):

```sh
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
export BEAMNG_HOME=/absolute/path/to/BeamNG.tech
export BEAMNG_USER=/absolute/path/to/dedicated/beamng-user
.venv/bin/python install_mod.py --user "$BEAMNG_USER"
sh run.sh --connect --replace-current --duration 10 --output ./captures
```

If system Python packages are intentionally needed, use `python3.10 -m venv --system-site-packages .venv` instead. Install only inside the venv, never `sudo pip`. This does not configure ROS.

Windows PowerShell:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:BEAMNG_HOME = 'E:\BeamNG.tech'
$env:BEAMNG_USER = 'E:\BeamNG-user'
.\.venv\Scripts\python.exe install_mod.py --user $env:BEAMNG_USER
.\run.ps1 --connect --replace-current --duration 10 --output .\captures
```

Paths above are examples, not runtime defaults. Wrappers use this directory's `.venv`, or the executable in `RURAL_PYTHON`. `--home`/`BEAMNG_HOME` and `--user`/`BEAMNG_USER` are mandatory. The installer copies only this bundle's fixed-name zip into `<user>/mods`, never other mods, and does not extract it. Identical content is a no-op; differing content is refused unless `--backup` is supplied. It retains a uniquely named backup. Use the actual versioned user directory if your BeamNG setup versions user folders; connect mode checks the reported effective user directory before making changes. Install before starting BeamNG so it discovers the mod.

## Run Safety

Default `--connect` probes only `127.0.0.1:64256` (override `--port`). Start your intended instance with TCOM enabled on that localhost port and the same explicit user folder. It will not auto-start an unknown app. `--launch` refuses if Windows `tasklist` or Linux `ps` reports a BeamNG process or the port is occupied. Close existing instances yourself; process inspection failure also aborts. `--replace-current` is mandatory before launch or scenario changes. There is a small process-start race with external launchers; do not start BeamNG concurrently.

For Linux launch, add `--launch --headless`; Vulkan rendering stays enabled for cameras/Lidar. Existing GPU-selection environment settings are inherited unchanged. Select GPU 1 using your driver's Vulkan device-selection mechanism before launch and verify the selected device in BeamNG logs. `CUDA_VISIBLE_DEVICES=1` does **not** guarantee Vulkan GPU selection. Connect mode cannot change rendering/GPU settings and rejects `--headless`.

Default duration is finite, 10 seconds of preview polling after sensor setup; startup and blocking API calls are additional time (socket timeout 30 seconds). Use `--duration N` and `--cadence N` (positive finite seconds). Default sensors: front RGB camera and AdvancedIMU. `--side-cameras`, `--lidar`, or `--full-sensors` add left/right cameras and Lidar; `--no-sensors` disables all. Sensors have independent requested update rates; polling is not synchronized or guaranteed to deliver fresh frames.

Without `--output`, readings are polled and discarded. With it, each run creates a unique subdirectory with `index.jsonl`, immutable per-poll samples and `latest/` RGB images. Index rows contain host wall/monotonic request/receive times, relative sample paths, and metadata actually returned by sensors. Camera/Lidar timestamps are not fabricated if absent. IMU readings retain their actual physics timestamps. Only RGB, Lidar point clouds, IMU measurements, and acquisition metadata are exported; annotations, instance labels, object maps, vehicle state, and ground-truth poses are not algorithm output. Lidar semantic colour data is deliberately omitted. This is a sampled preview, not a synchronized recorder or online ROS publisher.

On completion, Ctrl+C, or failure, the runtime attempts brakes, removes attached sensors, and disconnects with `quit_on_close=False`. It leaves the simulator and loaded scenario open; it does not restore the previous scene. Each scenario has a unique name to avoid overwriting editor scenarios.

## Validation And Bundle

```sh
python -m unittest discover -s tests -v
python run.py --help
```

Tests use fakes, real generated assets and real BeamNGpy 1.36 constructors/signature binding, never launch BeamNG, and do not validate physics, graphics, or hardware. Run them in the requirements-installed venv; BeamNGpy integration checks are mandatory, not silently skipped. A portable bundle must include these runtime files, `tests/`, `course_config.json`, `generated/map_objects.json`, `generated/build_manifest.json`, and `generated/nizhen_rural.zip`. Do not bundle venvs, credentials, simulator binaries, captures, or source-workspace paths. Asset production and final bundling are separate from this runtime.
