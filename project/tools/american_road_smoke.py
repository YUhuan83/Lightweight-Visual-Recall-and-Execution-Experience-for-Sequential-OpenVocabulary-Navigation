#!/usr/bin/env python3
"""Connect-only, stationary American Road camera smoke test.

Offline mode reads the ZIP and exercises the installed BeamNGpy Scenario.make
serialization with a fake transport and socket creation forbidden. Live mode
replaces the current session ONLY with --replace-current; unsaved work is not
preserved. Never launches, quits, installs maps, or touches license files.

Camera mount is an UNCALIBRATED trial: vehicle-local (0, -1.5, 2), forward
(0, -1, 0), up (0, 0, 1). Roof/body occlusion is possible. Inspect colour.png
before using this mount for measurements. No driving commands are issued;
physics settling is possible even with both brakes applied.
"""

import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import uuid
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation
from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.connection import Connection
from beamngpy.misc.quat import compute_rotation_matrix
from beamngpy.sensors import Camera, AdvancedIMU, Lidar, Electrics


LEVEL = "american_road"
CONFIG = "vehicles/pickup/d15_4wd_A.pc"
SPAWNS = ("spawns_sawmill", "spawns_road1")
TOLERANCE = 2e-6  # Serialized map matrices have float32 rounding.
CAMERA = dict(
    pos=(0, -1.5, 2), dir=(0, -1, 0), up=(0, 0, 1),
    resolution=(640, 360), field_of_view_y=60, requested_update_time=0.2,
    is_render_colours=True, is_render_annotations=False,
    is_render_instance=False, is_render_depth=False,
    is_using_shared_memory=False, is_streaming=False, is_static=False,
    is_snapping_desired=False, is_force_inside_triangle=False,
    is_dir_world_space=False,
)


def progress(stage):
    print(f"[{datetime.now(timezone.utc).isoformat()}] {stage}", flush=True)


def lidar_quality(cloud, position_before, position_after):
    """Stationary diagnostic under the explicit world-XYZ hypothesis, not calibration."""
    points = np.asarray(cloud, dtype=np.float64)
    before, after = np.asarray(position_before, dtype=float), np.asarray(position_after, dtype=float)
    if before.shape != (3,) or after.shape != (3,) or not np.isfinite([before, after]).all():
        raise RuntimeError('Invalid sensor world-position diagnostic')
    finite = np.isfinite(points).all(axis=1)
    valid = points[finite]
    ranges = np.minimum(np.linalg.norm(valid-before, axis=1), np.linalg.norm(valid-after, axis=1))
    displacement = float(np.linalg.norm(after-before))
    outside = int((ranges > 62).sum())
    return dict(hypothesis='XYZ is world-space; pose queries bracket poll, not acquisition-synchronized',
                position_before=before.tolist(), position_after=after.tolist(),
                position_change_m=displacement, max_range_requested_m=60,
                range_tolerance_m=2, out_of_range_points=outside,
                nonfinite_points=int((~finite).sum()),
                max_observed_range_m=float(ranges.max()) if len(ranges) else None,
                quality_pass=bool(len(points) and finite.all() and outside == 0 and displacement <= .1))


def imu_records(value):
    """Keep measured vectors only, with validity checked separately from container size."""
    if isinstance(value, dict):
        allowed = {'time', 'accRaw', 'accSmooth', 'angVel', 'angVelSmooth'}
        if allowed.intersection(value):
            return [{k: v for k, v in value.items() if k in allowed}]
        return [row for item in value.values() for row in imu_records(item)]
    if isinstance(value, list):
        return [row for item in value for row in imu_records(item)]
    return []


def valid_imu_record(row):
    try:
        for alternatives in (('accRaw','accSmooth'), ('angVel','angVelSmooth')):
            vector = next(row[key] for key in alternatives if key in row)
            if isinstance(vector, dict):
                vector = [vector[key] for key in ('x','y','z')]
            array = np.asarray(vector, dtype=float)
            if array.shape != (3,) or not np.isfinite(array).all():
                return False
        return bool('time' in row and np.isfinite(float(row['time'])))
    except (KeyError, StopIteration, TypeError, ValueError):
        return False


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_records(text):
    """Accept JSON objects/arrays and Torque's whitespace-separated JSON."""
    decoder = json.JSONDecoder()
    offset = 0
    while offset < len(text):
        if text[offset].isspace():
            offset += 1
            continue
        start = offset
        value, offset = decoder.raw_decode(text, offset)
        records = value if isinstance(value, list) else [value]
        for record in records:
            yield text.count("\n", 0, start) + 1, record


def transform(record):
    position = np.asarray(record["position"], dtype=float)
    scale = np.asarray(record.get("scale", [1, 1, 1]), dtype=float)
    matrix = np.asarray(record.get("rotationMatrix", np.eye(3)), dtype=float).reshape(3, 3)
    if position.shape != (3,) or scale.shape != (3,):
        raise ValueError("Invalid position or scale")
    if not all(np.isfinite(x).all() for x in (position, scale, matrix)):
        raise ValueError("Non-finite spawn transform")
    np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-5, rtol=0)
    np.testing.assert_allclose(np.linalg.det(matrix), 1, atol=1e-5, rtol=0)
    # Both SciPy and BeamNGpy use xyzw and the same row-major matrix formula.
    # Do not transpose, invert, or apply an extra 180-degree vehicle rotation.
    quat = Rotation.from_matrix(matrix).as_quat()
    roundtrip = compute_rotation_matrix(tuple(quat))
    np.testing.assert_allclose(roundtrip, matrix, atol=TOLERANCE, rtol=0)
    return dict(
        position=position.tolist(), scale=scale.tolist(),
        rotationMatrix=matrix.reshape(-1).tolist(), rot_quat_xyzw=quat.tolist(),
        rotation_source="explicit" if "rotationMatrix" in record else "omitted: identity default",
        scale_source="explicit" if "scale" in record else "omitted: unit default",
        roundtrip_max_abs_error=float(np.max(np.abs(roundtrip - matrix))),
    )


def inspect_archive(path):
    progress(f"Hashing and reading map archive: {path}")
    archive_hash = sha256(path)
    info_name = f"levels/{LEVEL}/info.json"
    found = {}
    scanned = 0
    with zipfile.ZipFile(path) as archive:
        info_bytes = archive.read(info_name)
        info = json.loads(info_bytes.decode("utf-8-sig"))
        for member in archive.infolist():
            if not (member.filename.startswith(f"levels/{LEVEL}/")
                    and member.filename.endswith("items.level.json")):
                continue
            scanned += 1
            data = archive.read(member)
            text = data.decode("utf-8-sig")
            if not any(name in text for name in SPAWNS):
                continue
            for line, record in json_records(text):
                if not isinstance(record, dict) or record.get("name") not in SPAWNS:
                    continue
                name = record["name"]
                if name in found:
                    raise ValueError(f"Ambiguous duplicate spawn: {name}")
                if record.get("class") != "SpawnSphere":
                    raise ValueError(f"Not a SpawnSphere: {name}")
                found[name] = dict(
                    member=member.filename, line=line, raw_object=record,
                    member_sha256=hashlib.sha256(data).hexdigest(),
                    **transform(record),
                )
    if set(found) != set(SPAWNS):
        raise ValueError(f"Missing target spawns: {set(SPAWNS) - set(found)}")
    return dict(path=str(path), sha256=archive_hash, info_member=info_name,
                info_sha256=hashlib.sha256(info_bytes).hexdigest(),
                default_spawn=info.get("defaultSpawnPointName"),
                items_files_scanned=scanned, spawns=found)


def build_scenario(name, spawn):
    scenario = Scenario(LEVEL, name, description="Stationary colour camera smoke test")
    vehicle = Vehicle("american_smoke_pickup", model="pickup", part_config=CONFIG)
    # Preserve the archived transform, not a safe-spawn relocation chosen by the game.
    scenario.add_vehicle(vehicle, pos=tuple(spawn["position"]),
                         rot_quat=tuple(spawn["rot_quat_xyzw"]), safe_spawn=False)
    return scenario, vehicle


def offline_test(archive, output):
    results = {}
    # This is the real Scenario.make and Jinja prefab, only the wire is mocked.
    with patch("socket.socket", side_effect=AssertionError("Offline mode forbids sockets")):
        for name, spawn in archive["spawns"].items():
            scenario, vehicle = build_scenario(f"offline_{name}", spawn)
            captured = {}

            class Transport:
                def _message(self, kind, **payload):
                    if kind != "CreateScenario":
                        raise AssertionError(kind)
                    captured.update(payload)
                    return f"levels/{LEVEL}/scenarios/{scenario.name}.json"

            scenario.make(Transport())
            records = [record for _, record in json_records(captured["prefab"])]
            if len(records) != 2 or captured["level"] != LEVEL:
                raise AssertionError("Expected one vehicle and one scenario group")
            car = next(record for record in records if record["class"] == "BeamNGVehicle")
            if (car["name"] != vehicle.vid or car["jBeam"] != "pickup"
                    or car["partConfig"] != CONFIG or car["spawnAutoplace"] != "false"):
                raise AssertionError("Incorrect serialized vehicle configuration")
            np.testing.assert_allclose(car["position"], spawn["position"], atol=0, rtol=0)
            np.testing.assert_allclose(car["rotationMatrix"], spawn["rotationMatrix"],
                                       atol=TOLERANCE, rtol=0)
            json.dumps(captured["info"], allow_nan=False)
            (output / f"{name}.prefab.json").write_text(captured["prefab"], encoding="utf-8")
            (output / f"{name}.scenario.json").write_text(
                json.dumps(captured["info"], indent=2), encoding="utf-8")
            results[name] = "PASS: full Scenario.make payload and prefab transform"
        tilted = Rotation.from_euler("xyz", [23, -17, 71], degrees=True).as_matrix()
        transform(dict(position=[0, 0, 0], rotationMatrix=tilted.tolist()))
        try:
            transform(dict(position=[0, 0, 0], rotationMatrix=np.zeros((3, 3))))
        except AssertionError:
            results["invalid_rotation_rejected"] = True
        else:
            raise AssertionError("Invalid rotation accepted")
    results["network"] = "forbidden; no simulator connection"
    return results


def live_capture(args, scenario, vehicle, output, report):
    bng = BeamNGpy(args.host, args.port, quit_on_close=False, socket_timeout=args.timeout)
    camera = None
    imu = None
    lidar = None
    cleanup_errors = []
    stationary_origin = None
    electrics = Electrics()
    vehicle.sensors.attach('diagnostic_electrics', electrics)

    def observe_vehicle(stage, enforce=False):
        vehicle.control(throttle=0, brake=1, parkingbrake=1, steering=0)
        vehicle.sensors.poll()
        state = dict(vehicle.state)
        pos = np.asarray(state['pos'], dtype=float)
        velocity = np.asarray(state['vel'], dtype=float)
        if pos.shape != (3,) or velocity.shape != (3,) or not np.isfinite([pos, velocity]).all():
            raise RuntimeError('Invalid vehicle state')
        distance = float(np.linalg.norm(pos-stationary_origin)) if stationary_origin is not None else None
        row = dict(stage=stage, simulation_time=state.get('time'), pos=pos.tolist(),
                   speed_m_s=float(np.linalg.norm(velocity)), displacement_m=distance)
        direction = np.asarray(state.get('dir', [0, 0, 0]), dtype=float)
        row['signed_forward_speed_m_s'] = float(np.dot(velocity, direction))
        keys = ('gear', 'gear_index', 'gear_a', 'reverse', 'throttle',
                'throttle_input', 'engine_throttle', 'brake', 'brake_input',
                'parkingbrake', 'parkingbrake_input', 'wheelspeed', 'rpm')
        row['electrics'] = {key: electrics.get(key) for key in keys}
        if stage == 'settling' and len(report.get('vehicle_observations', [])) % 10 == 0:
            progress(f'Braking diagnostic: {row}')
        report.setdefault('vehicle_observations', []).append(row)
        report['last_vehicle_state'] = state
        if enforce and (row['speed_m_s'] > .15 or distance > .3):
            raise RuntimeError(f'Stationary capture aborted: speed={row["speed_m_s"]:.3f}m/s, displacement={distance:.3f}m; inspect spawn/contact/control')
        return state, row
    try:
        progress(f"Connecting only to {args.host}:{args.port}; never launch")
        bng.open(launch=False)
        if not bng.tech_enabled():
            raise RuntimeError("BeamNG.tech sensor authorization is not enabled")
        report["tech_enabled"] = True
        progress("Creating unique scenario; replacing current session as explicitly authorized")
        scenario.make(bng)
        report["scenario_path"] = scenario.path
        progress("Loading American Road (shader compilation may take several minutes)")
        bng.scenario.load(scenario, connect_player_vehicle=False, connect_existing_vehicles=False)
        # Vehicle.connect does not inherit BeamNGpy.socket_timeout in this API.
        endpoint = bng.vehicles.start_connection(vehicle, vehicle.extensions)
        if endpoint["vid"] != vehicle.vid:
            raise RuntimeError("Simulator returned a different vehicle connection")
        vehicle.connection = Connection(args.host, int(endpoint["result"]), timeout=args.timeout)
        vehicle.connect(bng)
        vehicle.set_shift_mode('realistic_automatic')
        vehicle.control(throttle=0, brake=1, parkingbrake=1)
        progress("Starting scenario with brakes applied; no driving")
        bng.scenario.start()
        vehicle.ai.set_mode('disabled')
        vehicle.set_shift_mode('realistic_automatic')
        report['shift_mode_requested'] = 'realistic_automatic'
        progress('AI disabled, realistic_automatic requested (no arcade brake/reverse); waiting for stable braking')
        stable_since = None
        stable_pos = None
        settle_deadline = time.monotonic() + 30
        while time.monotonic() < settle_deadline:
            state, row = observe_vehicle('settling')
            now_sim = float(state['time'])
            pos = np.asarray(state['pos'],dtype=float)
            if row['speed_m_s'] > .1:
                stable_since = stable_pos = None
            elif stable_since is None or np.linalg.norm(pos-stable_pos) > .1 or now_sim < stable_since:
                stable_since, stable_pos = now_sim, pos.copy()
            elif now_sim-stable_since >= 3:
                stationary_origin = pos.copy()
                report['initial_vehicle_state'] = state
                break
            time.sleep(.2)
        else:
            raise RuntimeError('Vehicle did not settle within 30 wall seconds; sensors not created. See vehicle_observations.')
        progress("Creating front colour camera; trial mount is NOT calibrated")
        camera = Camera(f"front_{scenario.name}", bng, vehicle=vehicle, **report['camera'])
        report['camera_world_position_at_creation'] = list(camera.get_position())
        progress("Waiting for actual 640x360 colour reading")
        camera_started = time.monotonic()
        next_snapshot = 0.0
        report['exposure_samples'] = []
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            time.sleep(0.2)
            observe_vehicle('camera_warmup', enforce=True)
            reading = camera.poll()
            colour = reading.get("colour")
            if colour is None:
                continue
            if colour.size != (640, 360):
                raise RuntimeError(f"Unexpected colour image size: {colour.size}")
            elapsed = time.monotonic() - camera_started
            if elapsed < next_snapshot:
                continue
            pixels = np.asarray(colour.convert('RGB'))
            filename = f'exposure_{len(report["exposure_samples"]):02d}.png'
            colour.save(output / filename, format='PNG')
            sample = dict(file=filename, elapsed_wall_s=round(elapsed,3),
                          mean_rgb=float(pixels.mean()),
                          near_white_fraction=float(np.all(pixels >= 250, axis=2).mean()),
                          lower_half_near_white_fraction=float(np.all(pixels[pixels.shape[0]//2:] >= 250, axis=2).mean()))
            report['exposure_samples'].append(sample)
            progress(f'Exposure sample: {sample}')
            next_snapshot = elapsed + 2.0
            if elapsed < args.warmup_seconds:
                continue
            colour.save(output / "colour.png", format="PNG")
            pixels = np.asarray(colour.convert("RGB"))
            report["capture"] = dict(
                file="colour.png", sha256=sha256(output / "colour.png"),
                resolution=list(colour.size), mode=colour.mode,
                rgb_min=int(pixels.min()), rgb_max=int(pixels.max()),
                rgb_std=float(pixels.std()), timestamp_utc=datetime.now(timezone.utc).isoformat(),
            )
            vehicle.sensors.poll()
            report["final_vehicle_state"] = dict(vehicle.state)
            displacement = np.asarray(vehicle.state["pos"]) - np.asarray(
                report["initial_vehicle_state"]["pos"])
            report["displacement_metres"] = float(np.linalg.norm(displacement))
            if pixels.max() == pixels.min():
                raise RuntimeError("Uniform colour image saved for diagnosis; rendering not verified")
            progress("Saved actual colour.png; inspect for roof occlusion and scene correctness")
            if args.imu:
                progress('Creating IMU: gravity included; mounting and axes not calibrated')
                imu = AdvancedIMU('imu_' + scenario.name, bng, vehicle,
                                  pos=(0, 0, 1), physics_update_time=0.01,
                                  gfx_update_time=0.1, is_using_gravity=True,
                                  is_allow_wheel_nodes=False, is_visualised=False)
            if args.lidar:
                progress('Creating 16-line LiDAR; moderate density, no shared memory')
                lidar = Lidar('lidar_' + scenario.name, bng, vehicle,
                              pos=(0, 0, 2.2), requested_update_time=0.2,
                              vertical_resolution=16, frequency=10, max_distance=60,
                              is_using_shared_memory=False, is_visualised=False,
                              is_annotated=False)
            if imu is not None or lidar is not None:
                report['sensor_samples'] = []
                for index in range(args.sensor_samples):
                    time.sleep(0.5)
                    observe_vehicle('sensor_capture', enforce=True)
                    sample = {'index': index, 'host_poll_unix_ns': time.time_ns()}
                    if imu is not None:
                        raw = imu.poll()
                        reading = imu_records(raw)
                        filename = f'imu_{index:02d}.json'
                        (output / filename).write_text(json.dumps(reading, indent=2, allow_nan=False), encoding='utf-8')
                        sample['imu'] = {'file': filename, 'gravity_requested': True,
                                         'nonempty': bool(reading), 'record_count': len(reading),
                                         'valid_records': sum(valid_imu_record(row) for row in reading),
                                         'raw_top_level_type': type(raw).__name__,
                                         'raw_top_level_keys': list(map(str, raw.keys())) if isinstance(raw, dict) else []}
                    if lidar is not None:
                        before = lidar.get_position()
                        raw = lidar.poll()
                        after = lidar.get_position()
                        cloud = np.asarray(raw['pointCloud'])
                        if cloud.ndim != 2 or cloud.shape[1] != 3:
                            raise RuntimeError(f'Unexpected LiDAR XYZ shape {cloud.shape}')
                        finite = np.isfinite(cloud).all(axis=1)
                        filename = f'lidar_{index:02d}.npy'
                        np.save(output / filename, cloud, allow_pickle=False)
                        valid = cloud[finite]
                        sample['lidar'] = dict(file=filename, shape=list(cloud.shape),
                                               finite_points=int(finite.sum()), total_points=len(cloud),
                                               returned_keys=list(raw.keys()),
                                               xyz_min=valid.min(axis=0).tolist() if len(valid) else None,
                                               xyz_max=valid.max(axis=0).tolist() if len(valid) else None,
                                               frame='upstream coordinates, not yet landmark-validated',
                                               per_point_time='not supplied by this capture')
                        sample['lidar']['quality'] = lidar_quality(cloud, before, after)
                    sample['host_receive_unix_ns'] = time.time_ns()
                    report['sensor_samples'].append(sample)
                    observe_vehicle('sensor_capture_after_poll', enforce=True)
                    progress(f'Sensor sample: {sample}')
                if lidar is not None:
                    report['lidar_rejected_frames'] = [s['index'] for s in report['sensor_samples'] if not s['lidar']['quality']['quality_pass']]
                    if report['lidar_rejected_frames']:
                        raise RuntimeError('LiDAR quality check failed; raw frames retained, see quality diagnostics. Do not feed to navigation.')
                if imu is not None and not all(s['imu']['valid_records'] > 0 for s in report['sensor_samples']):
                    raise RuntimeError('Some IMU responses lack finite acceleration/angular velocity/source time')
                report['sensor_note'] = 'Sequential snapshots, not synchronized acquisition or ROS/LIO validation. Host times are not simulation timestamps.'
            return
        raise TimeoutError("No colour frame received before camera deadline")
    finally:
        progress("Cleanup: remove camera, keep pickup braked, disconnect; leave game running")
        actions = []
        if lidar is not None:
            actions.append(('lidar.remove', lidar.remove))
        if imu is not None:
            actions.append(('imu.remove', imu.remove))
        if camera is not None:
            actions.append(("camera.remove", camera.remove))
        if vehicle.is_connected():
            actions.append(("brakes", lambda: vehicle.control(throttle=0, brake=1, parkingbrake=1)))
        actions.extend((("vehicle.disconnect", vehicle.disconnect), ("bng.disconnect", bng.disconnect)))
        for label, action in actions:
            try:
                action()
            except Exception as exc:
                cleanup_errors.append(f"{label}: {type(exc).__name__}: {exc}")
                progress(f"Cleanup warning: {cleanup_errors[-1]}")
        report["cleanup_errors"] = cleanup_errors
        report["cleanup_policy"] = "Sensor removed; brakes retained; disconnect only; scenario left loaded"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map-zip", type=Path, default=Path("~/beamng-maps/american_road_0.39.zip"))
    parser.add_argument("--spawn", choices=SPAWNS, default="spawns_sawmill")
    parser.add_argument("--offline-only", action="store_true", help="Archive evidence and full prefab tests; no sockets")
    parser.add_argument("--replace-current", action="store_true", help="Authorize losing current unsaved simulator session")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=64256)
    parser.add_argument("--timeout", type=float, default=600, help="Socket and camera wait timeout, seconds (minimum 300)")
    parser.add_argument("--output-root", type=Path, default=Path("~/wildos-runs"))
    parser.add_argument('--warmup-seconds', type=float, default=10,
                        help='Observe raw image brightness before keeping final frame; wall seconds')
    parser.add_argument('--imu', action='store_true')
    parser.add_argument('--lidar', action='store_true')
    parser.add_argument('--sensor-samples', type=int, default=5)
    parser.add_argument('--camera-pos', type=float, nargs=3, default=list(CAMERA['pos']),
                        metavar=('X','Y','Z'), help='Trial vehicle-local mounting offset; not calibrated')
    args = parser.parse_args()
    if not args.offline_only and not args.replace_current:
        parser.error("Live capture requires --replace-current; current unsaved changes would be lost")
    if not np.isfinite(args.timeout) or args.timeout < 300:
        parser.error("--timeout must be finite and at least 300 seconds")
    if not np.isfinite(args.warmup_seconds) or not 0 <= args.warmup_seconds < args.timeout:
        parser.error('--warmup-seconds must be finite, nonnegative and below timeout')
    if not 1 <= args.sensor_samples <= 100:
        parser.error('--sensor-samples must be in 1..100')
    if not all(np.isfinite(v) and abs(v) <= 10 for v in args.camera_pos):
        parser.error('--camera-pos must be finite and within 10 metres of vehicle origin')
    run_id = "american-smoke-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:10]
    output = args.output_root.expanduser().resolve() / run_id
    output.mkdir(parents=True, exist_ok=False)
    report = dict(
        run_id=run_id, mode="offline" if args.offline_only else "live", status="started",
        source_sha256=sha256(Path(__file__).resolve()),
        versions={name: version(name) for name in ("beamngpy", "numpy", "scipy")},
        camera=dict(CAMERA, pos=tuple(args.camera_pos)), camera_calibration="UNVERIFIED trial; possible roof/body occlusion",
        vehicle=dict(model="pickup", part_config=CONFIG, scale=[1, 1, 1], safe_spawn=False,
                     config_evidence="Existing native configuration verified by operator; not validated by map ZIP"),
        endpoint=dict(host=args.host, port=args.port), timeout_seconds=args.timeout,
        warmup_wall_seconds=args.warmup_seconds,
        replace_current=args.replace_current, selected_spawn=args.spawn,
    )
    progress(f"Output: {output}")
    try:
        report["archive"] = inspect_archive(args.map_zip.expanduser().resolve())
        progress("Testing full prefab serialization offline for both verified spawns")
        report["offline_tests"] = offline_test(report["archive"], output)
        for name, evidence in report["archive"]["spawns"].items():
            progress(f"{name}: pos={evidence['position']} scale={evidence['scale']} "
                     f"quat_xyzw={evidence['rot_quat_xyzw']}")
        if not args.offline_only:
            scenario, vehicle = build_scenario(run_id.replace("-", "_"), report["archive"]["spawns"][args.spawn])
            report["scenario_name"] = scenario.name
            live_capture(args, scenario, vehicle, output, report)
        report["status"] = "passed" if not report.get("cleanup_errors") else "cleanup_failed"
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        progress(report["error"])
    finally:
        (output / "metadata.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
        progress(f"{report['status']}: {output / 'metadata.json'}")
        progress(f"Source SHA256: {report['source_sha256']}")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
