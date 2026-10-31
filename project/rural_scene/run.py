"""Finite, parked Italy preview using BeamNGpy 1.36 on localhost only."""
import argparse
import hashlib
import importlib.metadata
import json
import logging
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
import uuid

from install_mod import MOD_NAME, ROOT, digest, validate_mod, installed_mod

# Copied from verified native D15 provenance; spawn comes from the authored manifest.
# No source workspace or simulator installation paths are needed at runtime.
FACTS = {
    'level': 'italy', 'model': 'pickup',
    'part_config': 'vehicles/pickup/d15_4wd_A.pc',
}
ASSET_ROOT = '/art/shapes/nizhen_rural/'


def vector(value, size, name, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} requires {size} numbers')
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or
           not math.isfinite(x) or (positive and x <= 0) for x in value):
        raise ValueError(f'{name} contains invalid numbers')
    if size == 4 and abs(sum(x*x for x in value) - 1) > 0.001:
        raise ValueError(f'{name} must be a unit XYZW quaternion')
    return tuple(value)


def load_inputs(root=ROOT):
    raw = (root / 'generated/map_objects.json').read_bytes()
    manifest = json.loads(raw.decode('utf-8-sig'))
    if not isinstance(manifest, dict) or type(manifest.get('schema_version')) is not int or manifest['schema_version'] != 1:
        raise ValueError('map_objects requires schema_version 1')
    if manifest.get('level') != FACTS['level'] or manifest.get('asset_root') != ASSET_ROOT:
        raise ValueError('map_objects must use Italy and ' + ASSET_ROOT)
    spawn = manifest.get('course_spawn')
    if not isinstance(spawn, dict) or spawn.get('status') != 'candidate_not_simulator_verified':
        raise ValueError('course_spawn must identify an unverified candidate')
    vector(spawn.get('pos'), 3, 'course_spawn.pos')
    vector(spawn.get('rot_quat'), 4, 'course_spawn.rot_quat')
    build_manifest = validate_mod(root)
    objects = manifest.get('objects')
    if not isinstance(objects, list) or not objects:
        raise ValueError('map_objects.objects must be a nonempty list')
    if type(build_manifest.get('objects')) is not int or build_manifest['objects'] != len(objects):
        raise ValueError('Object count does not match build_manifest')
    names, ids = set(), set()
    required = {'oid', 'name', 'otype', 'pos', 'rot_quat', 'scale', 'shapeName', 'collisionType'}
    for obj in objects:
        if not isinstance(obj, dict) or not required.issubset(obj) or set(obj) - required - {'groundType'}:
            raise ValueError('Object requires native constructor fields; only groundType is optional')
        for key, seen in (('name', names), ('oid', ids)):
            value = obj[key]
            if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value) or value in seen or value == 'rural_ego':
                raise ValueError(f'Object {key} values must be unique identifiers')
            seen.add(value)
        name = obj['name']
        if obj['otype'] != 'TSStatic':
            raise ValueError('Only TSStatic objects are supported')
        vector(obj['pos'], 3, name + '.pos')
        vector(obj['rot_quat'], 4, name + '.rot_quat')
        vector(obj['scale'], 3, name + '.scale', positive=True)
        shape = obj['shapeName']
        if (not isinstance(shape, str) or not shape.startswith(ASSET_ROOT) or
                '\\' in shape or ':' in shape or '..' in shape.split('/') or
                PurePosixPath(shape).as_posix() != shape or not shape.endswith('.dae')):
            raise ValueError('shapeName must be a canonical .dae path under ' + ASSET_ROOT)
        if shape[1:] not in build_manifest['packaged_files']:
            raise ValueError('Object shape is missing from the mod: ' + shape)
        if obj['collisionType'] not in ('Visible Mesh', 'Visible Mesh Final'):
            raise ValueError('Unsupported native collisionType')
        if 'groundType' in obj and obj['groundType'] not in ('ASPHALT', 'WOOD'):
            raise ValueError('Unsupported native groundType')
    facts = dict(FACTS, spawn={'position': spawn['pos']}, rot_quat=spawn['rot_quat'],
                 schema_version=manifest['schema_version'], scene_sha256=hashlib.sha256(raw).hexdigest(),
                 mod_sha256=build_manifest['zip_sha256'], spawn_status=spawn['status'])
    return facts, objects


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', default=os.environ.get('BEAMNG_HOME'))
    parser.add_argument('--user', default=os.environ.get('BEAMNG_USER'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--port', type=int, default=64256)
    parser.add_argument('--timeout', type=float, default=180,
                        help='Socket timeout including handshake and map loading')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--launch', action='store_true')
    modes.add_argument('--connect', action='store_true')
    parser.add_argument('--replace-current', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', '--seconds', type=float, default=10)
    parser.add_argument('--cadence', type=float, default=0.2)
    for flag in ('full-sensors', 'side-cameras', 'lidar', 'no-sensors'):
        parser.add_argument('--' + flag, action='store_true')
    args = parser.parse_args(argv)
    for key in ('home', 'user'):
        if not getattr(args, key):
            parser.error(f'--{key} or BEAMNG_{key.upper()} is required')
        path = Path(getattr(args, key)).expanduser().resolve()
        if not path.is_dir():
            parser.error(f'--{key} must be an existing directory: {path}')
        setattr(args, key, path)
    if not args.replace_current:
        parser.error('--replace-current is required; no runtime changes made')
    if not 1 <= args.port <= 65535:
        parser.error('--port must be in 1..65535')
    if any(not math.isfinite(v) or v <= 0 for v in (args.duration, args.cadence, args.timeout)):
        parser.error('--duration and --cadence must be positive and finite')
    if args.headless and (not args.launch or platform.system() != 'Linux'):
        parser.error('--headless requires --launch on Linux')
    if args.no_sensors and (args.full_sensors or args.side_cameras or args.lidar):
        parser.error('--no-sensors conflicts with sensor selection flags')
    return args


def guard_connection(args):
    if args.launch:
        system = platform.system()
        if system not in ('Windows', 'Linux'):
            raise ValueError('Launch process checks support Windows and Linux only')
        command = ['tasklist', '/FO', 'CSV', '/NH'] if system == 'Windows' else ['ps', '-A', '-o', 'comm=']
        result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=10)
        if 'beamng' in result.stdout.lower():
            raise ValueError('BeamNG is already running; close it yourself or use --connect')
    try:
        with socket.create_connection(('127.0.0.1', args.port), timeout=2):
            listening = True
    except ConnectionRefusedError:
        listening = False
    except (TimeoutError, socket.timeout) as exc:
        if not args.launch:
            raise ValueError(f'Local TCOM probe timed out on port {args.port}; no world changed') from exc
        # A timed-out connect is inconclusive. Check exclusive local ownership
        # rather than treating it as either an occupied or definitely free port.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            if platform.system() == 'Windows':
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            try:
                probe.bind(('127.0.0.1', args.port))
            except OSError as bind_error:
                raise ValueError(f'Port {args.port} unavailable; refusing launch') from bind_error
        print(f'Port probe timed out, but local bind succeeded on {args.port}; proceeding with explicit launch', flush=True)
        listening = False
    if args.launch and listening:
        raise ValueError('Local port is already occupied; refusing launch')
    if not args.launch and not listening:
        raise ValueError('No localhost TCOM listener; start the intended instance or explicitly --launch')


def build(facts, objects, Scenario, Vehicle, ScenarioObject):
    scene = Scenario(facts['level'], 'nizhen_rural_' + uuid.uuid4().hex,
                     description='Parked native D-Series rural preview')
    car = Vehicle('rural_ego', model=facts['model'], part_config=facts['part_config'])
    scene.add_vehicle(car, pos=tuple(facts['spawn']['position']), rot_quat=tuple(facts['rot_quat']))
    for obj in objects:
        definition = dict(obj)
        for key in ('pos', 'rot_quat', 'scale'):
            definition[key] = tuple(definition[key])
        scene.add_object(ScenarioObject(**definition))
    return scene, car


def sensor_specs(args, Camera, AdvancedIMU, Lidar):
    if args.no_sensors:
        return []
    cameras = [('front', (0, -1.5, 2), (0, -1, 0))]
    if args.side_cameras or args.full_sensors:
        cameras += [('left', (-1.2, 0, 2), (-1, 0, 0)), ('right', (1.2, 0, 2), (1, 0, 0))]
    specs = [(Camera, name, dict(pos=pos, dir=direction, resolution=(640, 360),
              requested_update_time=args.cadence, is_render_colours=True,
              is_render_annotations=False, is_render_instance=False, is_render_depth=False,
              is_using_shared_memory=False, is_visualised=False)) for name, pos, direction in cameras]
    specs.append((AdvancedIMU, 'imu', dict(pos=(0, 0, 1), gfx_update_time=args.cadence,
                  physics_update_time=0.01, is_allow_wheel_nodes=False, is_visualised=False)))
    if args.lidar or args.full_sensors:
        specs.append((Lidar, 'lidar', dict(pos=(0, 0, 2.2), requested_update_time=args.cadence,
                      vertical_resolution=16, frequency=10, max_distance=60,
                      is_annotated=False, is_using_shared_memory=False, is_visualised=False)))
    return specs


def imu_measurements(data):
    # AdvancedIMU also returns world pose; never leak it into algorithm samples.
    allowed = {'time', 'accRaw', 'accSmooth', 'angVel', 'angVelSmooth'}
    if isinstance(data, list):
        return [imu_measurements(item) for item in data]
    if isinstance(data, dict):
        if allowed.intersection(data):
            return {k: v for k, v in data.items() if k in allowed}
        return {k: imu_measurements(v) for k, v in data.items() if isinstance(v, (dict, list))}
    return None


def save_sample(output, index, name, reading, timing):
    folder = output / 'samples' / f'{index:08d}_{name}'
    folder.mkdir(parents=True)
    paths = []
    if name == 'imu':
        metadata = imu_measurements(reading)
    else:
        # Keep only genuine timestamp metadata, never annotations or world pose.
        metadata = {k: v for k, v in reading.items() if k.lower() in ('time', 'timestamp', 'frame', 'frameid')}
        if name == 'lidar':
            import numpy as np
            path = folder / 'points.npy'
            np.save(path, reading['pointCloud'], allow_pickle=False)
        else:
            path = folder / 'colour.png'
            reading['colour'].save(path)
            shutil.copyfile(path, output / 'latest' / f'{name}.png')
        paths.append(str(path.relative_to(output).as_posix()))
    meta = folder / 'measurement.json'
    meta.write_text(json.dumps(metadata, allow_nan=False), encoding='utf-8')
    paths.append(meta.relative_to(output).as_posix())
    row = dict(sample=index, sensor=name, files=paths, sensor_metadata=metadata, **timing)
    with (output / 'index.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, allow_nan=False) + '\n')


def execute(args):
    if importlib.metadata.version('beamngpy') != '1.36':
        raise ValueError('Use beamngpy==1.36 in a dedicated venv')
    facts, objects = load_inputs()
    print(f"Scene schema {facts['schema_version']}, SHA-256 {facts['scene_sha256']}; mod SHA-256 {facts['mod_sha256']}")
    print('Using authored course_spawn candidate; terrain contact, settling and driving are NOT runtime verified.')
    print(f'Verified installed mod: {installed_mod(args.user)}', flush=True)
    guard_connection(args)
    from beamngpy import BeamNGpy, Scenario, Vehicle, ScenarioObject
    from beamngpy.sensors import Camera, AdvancedIMU, Lidar
    binary = 'Bin64/BeamNG.tech.x64.exe' if platform.system() == 'Windows' else 'BinLinux/BeamNG.tech.x64'
    bng = BeamNGpy('127.0.0.1', args.port, home=str(args.home), user=str(args.user),
                   binary=binary,
                   quit_on_close=False, headless=args.headless,
                   gfx='vk' if args.headless else None, socket_timeout=args.timeout)
    scene, car = build(facts, objects, Scenario, Vehicle, ScenarioObject)
    sensors = []
    loaded = False
    stage = 'connect/launch and handshake'
    try:
        print(f'[1/5] {stage}; timeout={args.timeout}s', flush=True)
        # BeamNGpy inherits cwd when launching. Resolve art/startup resources from home.
        previous_cwd = Path.cwd()
        try:
            if args.launch:
                os.chdir(args.home)
            bng.open(launch=args.launch, listen_ip='127.0.0.1')
        finally:
            os.chdir(previous_cwd)
        effective = Path(bng.user_with_version).resolve()
        if effective != (args.user / 'current').resolve():
            raise ValueError(f'Connected user directory is {effective}, not requested {args.user}; no scenario loaded')
        stage = 'create native scenario'
        print(f'[2/5] {stage}', flush=True)
        scene.make(bng)
        stage = 'load Italy and rural assets'
        print(f'[3/5] {stage}; first load may take several minutes', flush=True)
        bng.scenario.load(scene)
        loaded = True
        stage = 'start scenario and connect vehicle'
        print(f'[4/5] {stage}', flush=True)
        bng.scenario.start()
        car.control(throttle=0, brake=1, parkingbrake=1)
        stage = 'attach sensors and capture'
        print(f'[5/5] {stage}', flush=True)
        for cls, name, options in sensor_specs(args, Camera, AdvancedIMU, Lidar):
            sensors.append((name, cls('rural_' + name, bng, car, **options)))
        output = args.output.expanduser().resolve() / scene.name if args.output else None
        if output:
            (output / 'latest').mkdir(parents=True, exist_ok=False)
        print(f'Parked preview for {args.duration}s; output: {output or "disabled"}')
        start = time.monotonic()
        index = 0
        while time.monotonic() - start < args.duration:
            for name, sensor in sensors:
                if time.monotonic() - start >= args.duration:
                    break
                timing = dict(host_request_unix_ns=time.time_ns(), host_request_monotonic_ns=time.monotonic_ns())
                reading = sensor.poll()
                timing.update(host_receive_unix_ns=time.time_ns(), host_receive_monotonic_ns=time.monotonic_ns())
                if output:
                    save_sample(output, index, name, reading, timing)
                index += 1
            time.sleep(min(args.cadence, max(0, args.duration - (time.monotonic() - start))))
    except Exception:
        print(f'Failure stage: {stage}', file=sys.stderr, flush=True)
        logging.getLogger('rural').exception('Runtime failed at %s', stage)
        raise
    finally:
        if loaded:
            try:
                car.control(throttle=0, brake=1, parkingbrake=1)
            except Exception as exc:
                print(f'Brake cleanup failed: {exc}', file=sys.stderr)
        for name, sensor in reversed(sensors):
            try:
                sensor.remove()
            except Exception as exc:
                print(f'Sensor cleanup failed ({name}): {exc}', file=sys.stderr)
        bng.disconnect()


def main():
    args = parse_args()
    logs = args.user / 'rural-diagnostics'
    logs.mkdir(parents=True, exist_ok=True)
    logfile = logs / ('run-' + uuid.uuid4().hex[:10] + '.log')
    logging.basicConfig(filename=logfile, level=logging.DEBUG,
                        format='%(asctime)s %(name)s %(levelname)s %(message)s')
    print(f'Diagnostic log: {logfile}', flush=True)
    try:
        execute(args)
    except KeyboardInterrupt:
        print('Preview interrupted; simulator remains open.')
    except Exception as exc:
        logging.getLogger('rural').exception('Runtime failed, including prelaunch validation')
        print(f'Runtime stopped: {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
