"""Explicit opt-in runtime diagnostics. Never connects or launches by default."""
import argparse
import importlib.metadata
import json
import math
import platform
import socket
import subprocess
import time
import uuid
from pathlib import Path
import numpy as np
from build import write_json
from fixture import ROOT, ART, MOD, scenario, coordinates, fit_intrinsics
from install import user_paths, verified_mod
from inspect_local import inspect_home


def progress(message):
    print(time.strftime('[%H:%M:%S] ') + message, flush=True)


class DiagnosticFailure(ValueError):
    """Invalid measurement/fit, not a transport or sensor creation failure."""


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home', type=Path, required=True)
    p.add_argument('--user', type=Path, required=True, help='Userpath parent or its current directory')
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--connect', action='store_true')
    mode.add_argument('--launch', action='store_true')
    p.add_argument('--replace-current', action='store_true')
    p.add_argument('--port', type=int, default=64256)
    p.add_argument('--timeout', type=float, default=60)
    p.add_argument('--mounted', action='store_true', help='Also test a vehicle-mounted camera')
    p.add_argument('--lidar', action='store_true', help='Also demand static LiDAR geometry validation')
    p.add_argument('--mounted-lidar', action='store_true', help='Attach LiDAR to vehicle; requires --lidar --preview --preview-only')
    p.add_argument('--imu', action='store_true', help='Vehicle AdvancedIMU; requires --preview --preview-only')
    p.add_argument('--opencv', action='store_true', help='Detect actual image chessboard; required for calibrated=true')
    p.add_argument('--preview', action='store_true', help='Keep selected sensors alive with OpenCV windows')
    p.add_argument('--preview-seconds', type=float, default=0, help='Preview duration; 0 waits for q/Esc/window close')
    p.add_argument('--gfx', choices=('dx11','vk'), help='Explicit renderer; launch only')
    p.add_argument('--preview-only', action='store_true', help='Periodic polling preview without ad-hoc calibration calls')
    p.add_argument('--manual-drive', action='store_true', help='Release brakes after settling for supervised keyboard driving; preview-only, not calibration')
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error('--replace-current is required before any connection or world modification')
    if not 1 <= a.port <= 65535 or not math.isfinite(a.timeout) or a.timeout <= 0:
        p.error('Invalid port/timeout')
    if not math.isfinite(a.preview_seconds) or a.preview_seconds < 0:
        p.error('Invalid preview duration')
    if a.preview_seconds and not a.preview:
        p.error('--preview-seconds requires --preview')
    if a.preview_only and not a.preview:
        p.error('--preview-only requires --preview')
    if a.imu and not (a.preview and a.preview_only):
        p.error('--imu requires --preview --preview-only')
    if a.mounted_lidar and not (a.lidar and a.preview and a.preview_only):
        p.error('--mounted-lidar requires --lidar --preview --preview-only')
    if a.manual_drive and not (a.preview and a.preview_only and a.mounted):
        p.error('--manual-drive requires --preview --preview-only --mounted')
    if a.gfx and not a.launch:
        p.error('--gfx requires --launch; cannot change running renderer')
    return a


def adhoc(sensor, timeout, silent=False, stop=None):
    if not silent:
        progress('Requesting sensor sample: ' + getattr(sensor, 'name', type(sensor).__name__))
    deadline = time.monotonic() + timeout

    def call(method, *args):
        if stop is not None and stop.is_set():
            raise InterruptedError('Preview stopped')
        if time.monotonic() >= deadline:
            raise TimeoutError('Ad-hoc sensor request timed out')
        return method(*args)

    before = dict(position=call(sensor.get_position), direction=call(sensor.get_direction))
    start = time.monotonic()
    request = call(sensor.send_ad_hoc_poll_request)
    while not call(sensor.is_ad_hoc_poll_request_ready, request):
        if time.monotonic()-start > timeout:
            raise TimeoutError('Ad-hoc sensor request timed out')
        time.sleep(.05)
    reading = call(sensor.collect_ad_hoc_poll_request, request)
    meta = dict(source='BeamNGpy 1.36 collect_ad_hoc_poll_request', request_id=request,
                host_monotonic_start=start, host_monotonic_end=time.monotonic(),
                simulation_timestamp=None, timestamp_note='API does not supply a synchronized timestamp',
                 pose_before=before, pose_after=dict(position=call(sensor.get_position), direction=call(sensor.get_direction)),
                reading_keys=list(reading))
    return reading, meta


def periodic_poll(sensor, timeout, silent=False, stop=None):
    from beamngpy.sensors import AdvancedIMU
    if stop is not None and stop.is_set():
        raise InterruptedError('Preview stopped')
    start = time.monotonic()
    if isinstance(sensor, AdvancedIMU):
        reading = sensor.poll()
        return reading, dict(source='BeamNGpy 1.36 AdvancedIMU.poll; bulk; cached samples possible',
            host_monotonic_start=start, host_monotonic_end=time.monotonic(),
            timestamp_note='Per-measurement time is supplied by sensor; host times are not simulation time')
    before = dict(position=sensor.get_position(), direction=sensor.get_direction())
    reading = sensor.poll()
    return reading, dict(source='periodic poll; cached frames possible', request_id=None,
        host_monotonic_start=start, host_monotonic_end=time.monotonic(),
        simulation_timestamp=None, pose_before=before,
        pose_after=dict(position=sensor.get_position(), direction=sensor.get_direction()),
        reading_keys=list(reading))


def camera_check(sensor, manifest, output, name, opencv, up, timeout):
    progress('Camera check: ' + name)
    reading, meta = adhoc(sensor, timeout)
    image = reading['colour']
    if image is None or image.size != (960,640):
        raise DiagnosticFailure('Missing/incorrect RGB image')
    image.save(output/(name+'.png'))
    progress('Image saved; checking engine projection: ' + name)
    points = manifest['checker']['points_world']
    pixels = np.asarray([sensor.world_point_to_pixel(tuple(p)) for p in points])
    pose = meta['pose_before']
    try:
        xy = coordinates(points, pose['position'], pose['direction'], up)
        fit, predicted = fit_intrinsics(xy, pixels)
    except (ValueError, np.linalg.LinAlgError) as exc:
        write_json(output/(name+'.json'), dict(passed=False, metadata=meta, error=str(exc)))
        raise DiagnosticFailure(str(exc)) from exc
    stable = np.linalg.norm(np.asarray(pose['position'])-meta['pose_after']['position']) < .005
    stable = bool(stable and np.linalg.norm(np.asarray(pose['direction'])-meta['pose_after']['direction']) < .002)
    rgb = np.asarray(image.convert('RGB'), dtype=float)
    visual = dict(passed=False, reason='OpenCV check not requested')
    if opencv:
        try:
            import cv2
            progress('Detecting checkerboard in actual image: ' + name)
            gray = cv2.cvtColor(np.asarray(image.convert('RGB')), cv2.COLOR_RGB2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, (9,6), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            visual = dict(passed=False, detected=bool(found), reason='Chessboard not detected')
            if found:
                grid = corners.reshape(6,9,2)
                variants = [g.reshape(-1,2) for g in (grid,grid[::-1],grid[:,::-1],grid[::-1,::-1])]
                detected = min(variants, key=lambda q: np.mean(np.linalg.norm(q-pixels, axis=1)))
                errors = np.linalg.norm(detected-predicted, axis=1)
                visual = dict(passed=bool(max(errors)<3), detected=True,
                              max_reprojection_px=float(max(errors)), corners_px=detected.tolist(),
                              reason='Actual image corners compared to held-out API fit; not full lens calibration')
        except ImportError:
            visual = dict(passed=False, reason='opencv-python is not installed')
    report = dict(metadata=meta, up_basis_used=list(up), up_note='Static world up or observed vehicle up; camera API has no get_up',
                  api_pixels=pixels.tolist(), known_points_world=points, api_fit=fit, visual=visual,
                  pose_stable=stable, rgb_std=float(rgb.std()),
                  near_white_fraction=float(np.mean(np.all(rgb>245, axis=2))),
                  saturated_fraction=float(np.mean(np.any(rgb>=254, axis=2))))
    report['passed'] = bool(fit['passed'] and visual['passed'] and stable)
    if name == 'static':
        report['requested_pose_matches'] = bool(
            np.linalg.norm(np.asarray(pose['position'])-manifest['camera']['pos'])<.005
            and np.linalg.norm(np.asarray(pose['direction'])-manifest['camera']['dir'])<.002)
        report['passed'] = report['passed'] and report['requested_pose_matches']
    write_json(output/(name+'.json'), report)
    return report


def lidar_check(sensor, manifest, output, timeout):
    reading, meta = adhoc(sensor, timeout)
    cloud = np.asarray(reading['pointCloud'])
    np.save(output/'lidar.npy', cloud, allow_pickle=False)
    if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud) or not np.isfinite(cloud).all():
        raise DiagnosticFailure('Empty/nonfinite/malformed LiDAR cloud; raw saved')
    pos = np.asarray(meta['pose_before']['position'])
    # Do not silently assume a frame. Test raw-world and translated-local hypotheses.
    hypotheses = {}
    for name, world in (('raw_world',cloud), ('translated_local_world_axes',cloud+pos)):
        ranges = np.linalg.norm(world-pos, axis=1)
        valid = (ranges>.05)&(ranges<=60.1)
        ground = (np.abs(world[:,2]-.1)<.035)&(np.abs(world[:,0])<11.8)&(world[:,1]>-17.8)&(world[:,1]<11.8)
        board = (np.abs(world[:,1]+10)<.035)&(np.abs(world[:,0])<2.45)&(world[:,2]>.65)&(world[:,2]<4.05)
        hypotheses[name] = dict(valid_range_fraction=float(valid.mean()), ground_hits=int(ground.sum()),
                                board_hits=int(board.sum()), range_min=float(ranges.min()), range_max=float(ranges.max()),
                                passed=bool(valid.mean()>.99 and ground.sum()>30 and board.sum()>10))
    winners = [k for k,v in hypotheses.items() if v['passed']]
    stable = bool(np.linalg.norm(pos-meta['pose_after']['position'])<.005
                  and np.linalg.norm(np.asarray(meta['pose_before']['direction'])-meta['pose_after']['direction'])<.002)
    report = dict(metadata=meta, points=len(cloud), finite=True, hypotheses=hypotheses,
                  frame_observed=winners[0] if len(winners)==1 else 'unresolved',
                  pose_stable=stable, passed=len(winners)==1 and stable,
                  scope='Static range/fixture alignment only; no camera-lidar time synchronization')
    write_json(output/'lidar.json', report)
    return report


def execute(a):
    if a.preview:
        from preview import preflight, run_preview
        progress('Checking OpenCV GUI before connecting')
        cv2 = preflight()
    progress('Checking installation and fixture mod')
    if importlib.metadata.version('beamngpy') != '1.36':
        raise ValueError('Requires beamngpy==1.36')
    inspect_home(a.home)
    parent, effective = user_paths(a.user)
    if (effective/'mods'/MOD).read_bytes() != verified_mod().read_bytes():
        raise ValueError('Own installed ZIP differs')
    if a.launch:
        cmd = ['tasklist','/FO','CSV','/NH'] if platform.system()=='Windows' else ['ps','-A','-o','comm=']
        if 'beamng' in subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=10).stdout.lower():
            raise ValueError('BeamNG process exists; refusing second launch')
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',a.port))
    from beamngpy import BeamNGpy
    from beamngpy.sensors import AdvancedIMU, Camera, Lidar
    binary = 'Bin64/BeamNG.tech.x64.exe' if platform.system()=='Windows' else 'BinLinux/BeamNG.tech.x64'
    bng = BeamNGpy('127.0.0.1', a.port, home=str(a.home.resolve()), user=str(parent),
                   binary=binary, quit_on_close=False, socket_timeout=a.timeout, gfx=a.gfx)
    output = ROOT/'captures'/uuid.uuid4().hex
    output.mkdir(parents=True)
    progress('Output: ' + str(output))
    report = dict(calibrated=False, runtime_observed=False, scope='fixture diagnostics only; not navigation or production calibration',
                  demanded=dict(static_camera=True, mounted=a.mounted, lidar=a.lidar, visual=True), checks={})
    sensors = []
    selected = {}
    car = None
    preview_state = {}
    try:
        progress('Connecting to BeamNG')
        bng.open(launch=a.launch, listen_ip='127.0.0.1')
        actual = bng.control.queue_lua_command('return FS:getUserPath()', response=True)
        report['actual_userpath'] = actual
        if not isinstance(actual, str) or Path(actual).resolve() != effective:
            raise ValueError('Actual FS:getUserPath differs from expected current directory; world untouched')
        mounted = bng.control.queue_lua_command('return FS:fileExists('+json.dumps(ART+'fixture.dae')+')', response=True)
        # techCore.handleQueueLuaCommandGE serializes every returned value with tostring.
        if mounted != 'true':
            raise ValueError('Fixture not mounted; restart intended profile manually; world untouched')
        manifest = json.loads((ROOT/'generated/fixture.json').read_text(encoding='utf-8'))
        scene, car = scenario(manifest, 'nzs_cal_'+uuid.uuid4().hex)
        progress('Creating fixture scenario')
        scene.make(bng)
        progress('Loading smallgrid; connecting vehicle with bounded timeout')
        bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
        from beamngpy.connection import Connection
        endpoint = bng.vehicles.start_connection(car, car.extensions)
        if endpoint['vid'] != car.vid:
            raise RuntimeError('Unexpected vehicle connection ID')
        car.connection = Connection('127.0.0.1', int(endpoint['result']), timeout=a.timeout)
        car.connect(bng)
        progress('Starting and checking parked vehicle')
        bng.scenario.start()
        report['runtime_observed'] = True
        car.ai.set_mode('disabled')
        car.set_shift_mode('realistic_automatic')
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0)
        bng.control.resume()
        time.sleep(5)
        states = []
        for _ in range(5):
            car.sensors.poll()
            states.append(dict(car.state))
            time.sleep(.25)
        positions = np.asarray([s['pos'] for s in states])
        speeds = np.asarray([np.linalg.norm(s['vel']) for s in states])
        settled = bool(np.isfinite(positions).all() and max(speeds)<.05
                       and np.max(np.linalg.norm(positions-positions[0], axis=1))<.02
                       and np.linalg.norm(positions[-1,:2]-[0,4])<.5 and .1<positions[-1,2]<2
                       and states[-1]['up'][2]>.98)
        report['checks']['settle'] = dict(passed=settled, states=states, speed_max=float(max(speeds)))
        if not settled:
            raise ValueError('D-Series did not settle upright on fixture')
        configs = [('static',None,manifest['camera']['pos'],[0,-1,0],[0,0,1])]
        if a.mounted:
            configs.append(('mounted',car,[0,-1.5,2.35],[0,-1,0],states[-1]['up']))
        for name, vehicle, pos, direction, up in configs:
            progress('Creating camera: ' + name)
            cam = Camera('nzs_'+name, bng, vehicle, pos=tuple(pos), dir=tuple(direction), up=(0,0,1),
                         is_static=vehicle is None, is_dir_world_space=vehicle is None,
                         resolution=(960,640), field_of_view_y=60, near_far_planes=(.05,60),
                           requested_update_time=.2 if a.preview_only else -1, is_using_shared_memory=False, is_visualised=False,
                         is_render_colours=True, is_render_depth=False, is_render_annotations=False)
            sensors.append(cam)
            selected[name] = cam
        if a.lidar:
            progress('Creating LiDAR')
            lidar = Lidar('nzs_lidar',bng,car if a.mounted_lidar else None,pos=(0,0,2.35),dir=(0,-1,0),up=(0,0,1),
                           is_static=not a.mounted_lidar,is_dir_world_space=not a.mounted_lidar,requested_update_time=.2 if a.preview_only else -1,
                          vertical_resolution=64,vertical_angle=60,max_distance=60,density=1,
                           is_using_shared_memory=False,is_visualised=False,is_annotated=False)
            sensors.append(lidar)
            selected['lidar'] = lidar
        if a.imu:
            progress('Creating vehicle AdvancedIMU')
            imu = AdvancedIMU('nzs_imu', bng, car, pos=(0,0,1), dir=(0,-1,0), up=(0,0,1),
                              is_using_gravity=True, physics_update_time=.01, gfx_update_time=.2,
                              is_send_immediately=False, is_dir_world_space=False, is_visualised=False)
            sensors.append(imu)
            selected['imu'] = imu
        progress('All selected sensors initialized; starting diagnostics')
        for name, sensor in selected.items():
            if a.preview_only:
                report['checks'][name] = dict(passed=False, reason='Preview-only: calibration not executed')
                continue
            try:
                if name == 'lidar':
                    result = lidar_check(sensor,manifest,output,a.timeout)
                else:
                    up = next(c[4] for c in configs if c[0] == name)
                    result = camera_check(sensor,manifest,output,name,a.opencv,up,a.timeout)
            except DiagnosticFailure as exc:
                if not a.preview:
                    raise
                result = dict(passed=False, error=str(exc))
                progress('Diagnostic NOT PASSED: ' + name + ': ' + str(exc))
            report['checks'][name] = result
        report['calibrated'] = all(c['passed'] for c in report['checks'].values())
        if a.preview:
            if a.manual_drive:
                report['calibrated'] = False
                report['manual_driving'] = True
                car.ai.set_mode('disabled')
                car.control(throttle=0, brake=0, parkingbrake=0, steering=0)
                progress('Manual preview: brakes released; focus GAME window, select forward gear using your bindings. No autonomous control.')
            write_json(output/'runtime_report.json', report)
            report['preview'] = preview_state
            progress('Preview: q/Esc/close to exit; S saves latest raw samples. calibrated=' + str(report['calibrated']))
            run_preview(cv2, selected, periodic_poll if a.preview_only else adhoc,
                        a.timeout, a.preview_seconds, output, report, preview_state, progress)
    except Exception as exc:
        report['calibrated'] = False
        report['error'] = type(exc).__name__+': '+str(exc)
        progress('Failed: ' + report['error'])
        raise
    finally:
        safe_cleanup = not preview_state.get('worker_alive', False)
        if not safe_cleanup:
            report['cleanup_errors'] = ['Worker still blocked: skipped brake/remove/disconnect to avoid socket race; sensors may remain']
            report['calibrated'] = False
            progress(report['cleanup_errors'][0])
        if safe_cleanup and car is not None and car.connection is not None:
            try:
                car.ai.set_mode('disabled')
                car.control(throttle=0, brake=1, parkingbrake=1, steering=0)
            except Exception as exc:
                report.setdefault('cleanup_errors', []).append(str(exc))
                report['calibrated'] = False
        for sensor in reversed(sensors) if safe_cleanup else []:
            try:
                sensor.remove()
            except Exception as exc:
                report.setdefault('cleanup_errors', []).append(str(exc))
                report['calibrated'] = False
        write_json(output/'runtime_report.json',report)
        if safe_cleanup:
            bng.disconnect()
        progress('Report: ' + str(output/'runtime_report.json'))
    return 0 if report['calibrated'] else 2


if __name__ == '__main__':
    raise SystemExit(execute(parse_args()))
