# Tomorrow: Pointgoal Baseline

This is NOT WildOS, VFM, semantic navigation, object search, sensor-only localization,
or a generally capable off-road driver. It is an observed-flat-ground LiDAR/A*
baseline using oracle SELF pose and, only by explicit opt-in, BeamNG's AI tracker.
No target-object truth, road network, ground-truth terrain/map, or annotations are read.
The original visual/material diagnosis is unchanged. No simulator runtime was verified.

## Contents

The source-only ZIP contains one standalone `beamng_pointgoal.py`, its offline tests,
this README, `requirements.json`, and a SHA256 manifest. There are no game files,
maps, weights, environments, dependency wheels or recorded simulator data.
Canonical workspace sources are `tools/beamng_pointgoal.py` and its neighboring test.
`package.py` rebuilds the ZIP from those sources; it needs only Python stdlib.

## First Session

Use the existing server venv with BeamNGpy **1.36**, NumPy and optionally Pillow.
No Torch, Transformers, SciPy or ROS is required. Do not install a new environment
blindly while offline. The documented compatible simulator is BeamNG.tech **0.39**,
not a simulator version called 1.36. Keep the existing scene running, unpaused,
and the pickup `american_smoke_pickup` parked. Close other driving/control clients.
Keep the simulator UI accessible for human braking/pause, especially during motion.

From the extracted kit directory:

```bash
python -B test_beamng_pointgoal.py
python -B beamng_pointgoal.py --help
python -B beamng_pointgoal.py --goal-forward 2 --goal-left 0 --output-root ./runs
```

The default host/port are `127.0.0.1:64256`; override `--host`/`--port` if needed.
Tests use synthetic data and mocked transports; they do not connect to BeamNG.

**DRY RUN IS NOT READ-ONLY:** it connects to the EXISTING named pickup, disables AI,
requests realistic automatic shift mode, applies service and parking brakes, adds a
temporary LiDAR, then scans/plans. It never creates, replaces, loads or teleports a
scene/vehicle. It leaves the pickup braked and disconnects without quitting the game.
The gear selector is not guessed or forced. No motion command is sent in dry run.

## Inspect Before Motion

Each run has `report.json` and `events.ndjson` with actual states and Electrics.
Each scan directory retains `cloud_world_raw.npy` BEFORE quality rejection,
`acquisition.json`, and on valid acquisition `cloud_local.npy`. There are at most
three quality-rejected stationary attempts per planning step. Request timeout aborts.

Successful map construction saves `map_grid.npy`, `safe_centers.npy`, `map.json`,
and optional `path_grid.png`; successful planning adds `plan.json`. A failed map
still leaves raw cloud and state evidence; a failed path does not create a fake path.
PNG: forward is upward, left is rightward on this array visualization. Gray unknown,
red blocked, pale green observed ground, gold initial-footprint assumption, green
inflated-safe centers, blue A* cells, white start. NPY grid axes are forward/left.

1. Confirm the selected vehicle, world pose, level orientation, near-zero speed and actual brakes.
2. Inspect raw cloud shape/ranges and rejection reasons. Failed frames must stay rejected.
3. Independently compare world XYZ against fixed geometry, and check the forward/left transform with known features. Range consistency alone is NOT calibration.
4. Confirm the near-road low-quantile height really represents road ground, not a ditch, body return or vegetation. Check both sides and full vehicle length.
5. Confirm the entire short straight corridor is visible, flat, clear and static. The trial 5.5m by 2.5m rectangle and 1.4m inflation circle are assumptions, not an exact pickup footprint.
6. Require `dry_run_plan_ready_no_motion`; inspect the actual path and ground coverage. Unknown support or turns must fail, not be made free by loosening thresholds.

World frame math uses normalized horizontal `state.dir` as forward, world-up cross
forward as left. Row-vector world points become `(points - state.pos) @ basis`.
The full `up` and `dir` are tilt-gated at 8 degrees. Mount direction `(0,-1,0)` is
native vehicle-local forward; `is_dir_world_space=False` does not make returned XYZ
sensor-local. Ad-hoc request/collection is bracketed with pose and sensor-position
queries; no exact acquisition time, ring, intensity, per-point timestamp or deskew
is invented. The simulator must advance during stationary capture.

## Optional Supervised Motion

Only after the checks above, supply a real landmark-check note, not the placeholder:

```bash
python -B beamng_pointgoal.py --goal-forward 2 --goal-left 0 --execute-motion --tracker beamng-ai --world-frame-verified "describe the fixed geometry comparison actually performed" --output-root ./runs
```

The goal is relative to the settled CURRENT ego pose, forward/left in metres, at
most 10m. It remains fixed in world coordinates through this run. Start with 2m,
not 10m. Motion is limited to straight, initially aligned corridors (3 degrees).
A* finds a supported route, but only a footprint-checked straight visibility
shortcut is executable. Non-straight paths and sharp initial headings are rejected.
This intentionally cannot navigate around a typical blocking obstacle yet.

The tracker gets `ai.set_line([{pos: ..., speed: ...}, ...], cling=False)` with
constant current vehicle-origin world Z on observed flat ground, target 0.5m/s,
and endpoint speed zero. No road waypoint API or simulator terrain clinging is used.
The AI owns steering, propulsion and transmission, not our own calibrated controller.
If it remains in neutral or fails to move, the chunk times out: NO gear=1 assumption
and NO escalating throttle. Check `gear`, `gear_index`, `gear_a`, `reverse`, `rpm`,
`running`, wheel speed and brake/throttle inputs in the events log.

Each chunk is at most 2m, followed by braking, settling, a fresh scan and replanning.
State polling targets ~20Hz, but transport and footprint checks reduce actual rate.
The run aborts on >0.5s feedback gaps, stalled sim time, speed >1m/s, reverse >0.1m/s,
cross-track >0.2m, heading divergence, unsupported footprint, chunk timeout or
overshoot. Braking begins around 0.25m before the chunk endpoint. These are initial
conservative thresholds, NOT measured braking/controller performance guarantees.
Arrival requires <=0.35m horizontal goal error after stationary reacquisition.

## Stop And Failure Policy

Ctrl+C/SIGTERM and normal exceptions attempt AI disable plus full braking FIRST,
then sensor removal and disconnect. Repeat signals are ignored during cleanup.
Process kill, disconnected sockets, a blocked simulator, AI response latency or
excessive momentum can defeat this best-effort stop. There is NO simulator-side
watchdog or guaranteed fail-safe. A two-second socket timeout is still too long
to promise emergency braking. Human pause/brake access is mandatory for motion.
No moving-obstacle detection is performed during a chunk; only the parked scan is
used. Use a static, empty, supervised test area, not traffic.

## Expected Blockers

Sparse/occluded ground, footprint-adjacent blind zones, curbs, slopes, ditches,
vegetation and unknown patches can legitimately prevent even a 2m plan. Free
space is established only by >=2 near-flat ground returns per 0.5m cell; no convex
hull, blanket ray clearing or fabricated support is used. Only the initial occupied
rectangle is assumed supported; the assumption is not dragged along the route.
Cells with above-ground/low returns are blocked and unknown is inflated as blocked.

64 LiDAR lines and 90-degree vertical coverage aim to improve near ground sampling,
but GPU performance, mounting, world coordinates, automatic shift behavior, AI line
tracking, braking distance and actual ground coverage all require tomorrow's run.
If these fail, retain artifacts and stop. Do not call this complete WildOS or claim
successful motion based on offline tests. VFM inference remains a separate blocker.
