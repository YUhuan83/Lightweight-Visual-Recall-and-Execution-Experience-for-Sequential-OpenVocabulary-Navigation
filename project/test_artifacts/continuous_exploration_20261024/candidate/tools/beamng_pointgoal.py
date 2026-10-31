#!/usr/bin/env python3
"""Observed-flat-ground pointgoal baseline. NOT WildOS or object search.

Live default: disable AI, brake EXISTING vehicle, scan and plan, never drive.
Motion requires --execute-motion --tracker beamng-ai --world-frame-verified NOTE.
No scene creation/loading, teleport, terrain query, annotation or gear guessing.
"""
import argparse
import heapq
import importlib.metadata
import json
import math
from pathlib import Path
import signal
import time
import uuid

import numpy as np


RES = 0.5
HALF = 16.0
SIZE = 65
RADIUS = 1.4
# Trial pickup envelope, NOT a measured bounding box or rear-axle origin.
HALF_LENGTH = 2.75
HALF_WIDTH = 1.25


class Unsafe(RuntimeError):
    pass


def unit(value):
    v = np.asarray(value, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-6:
        raise Unsafe('Invalid direction vector')
    return v / np.linalg.norm(v)


def frame(state):
    """Horizontal forward/left/up basis; retain true up separately for tilt gates."""
    f = unit(state['dir'])
    up = unit(state['up'])
    if up[2] < math.cos(math.radians(8)) or abs(f[2]) > math.sin(math.radians(8)):
        raise Unsafe('Only near-level vehicle poses supported (8 degree tilt limit)')
    f = unit([f[0], f[1], 0])
    left = np.cross([0., 0., 1.], f)
    return np.column_stack((f, left, [0., 0., 1.]))


def validate_state(state):
    for key in ('pos', 'vel', 'dir', 'up'):
        v = np.asarray(state[key], dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise Unsafe('Invalid state ' + key)
    if not math.isfinite(float(state['time'])):
        raise Unsafe('Invalid simulation time')
    frame(state)
    return state


def cloud_check(cloud, before, after, sensor_before, sensor_after):
    validate_state(before)
    validate_state(after)
    p = np.asarray(cloud)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 100 or not np.isfinite(p).all():
        raise Unsafe('Cloud must contain >=100 finite XYZ points')
    sb, sa = np.asarray(sensor_before), np.asarray(sensor_after)
    if sb.shape != (3,) or sa.shape != (3,) or not np.isfinite([sb, sa]).all():
        raise Unsafe('Invalid sensor position')
    dt = float(after['time']) - float(before['time'])
    shift = np.linalg.norm(np.asarray(after['pos']) - before['pos'])
    angles = [math.degrees(math.acos(float(np.clip(np.dot(unit(before[k]), unit(after[k])), -1, 1))))
              for k in ('dir', 'up')]
    if dt <= 0 or shift > .05 or np.linalg.norm(sa-sb) > .05 or max(angles) > .5:
        raise Unsafe('Acquisition bracket moved/rotated, or simulation clock did not advance')
    if max(np.linalg.norm(before['vel']), np.linalg.norm(after['vel'])) > .08:
        raise Unsafe('Acquisition not stationary')
    ranges = np.minimum(np.linalg.norm(p-sb, axis=1), np.linalg.norm(p-sa, axis=1))
    if ranges.max() > 61:
        raise Unsafe('World-XYZ hypothesis failed 60m range consistency check')
    return dict(sim_bracket_s=[before['time'], after['time']], displacement_m=float(shift),
                angle_change_deg=max(angles), max_range_m=float(ranges.max()),
                world_xyz='range-consistent hypothesis; landmark verification remains operator responsibility',
                per_point_time=None, acquisition_time='bounded request, not exact timestamp')


def cell(xy):
    return tuple(np.floor((np.asarray(xy) + HALF) / RES + .5).astype(int))


def center(ij):
    return np.asarray(ij, dtype=float) * RES - HALF


def inside(ij):
    return all(0 <= a < SIZE for a in ij)


def build_grid(points):
    """Only directly sampled flat ground is free; no convex hull/ray clearing."""
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
        raise Unsafe('Invalid local points')
    ego = (np.abs(p[:, 0]) <= HALF_LENGTH) & (np.abs(p[:, 1]) <= HALF_WIDTH)
    radius = np.linalg.norm(p[:, :2], axis=1)
    nearby = p[(~ego) & (radius < 6) & (radius > 1.5)]
    if len(nearby) < 60:
        raise Unsafe('Insufficient observed nearby ground candidates')
    ground = float(np.quantile(nearby[:, 2], .10))
    if not -3 <= ground <= .5:
        raise Unsafe('Implausible observed ground offset relative to vehicle origin')
    candidates = nearby[np.abs(nearby[:, 2]-ground) <= .12]
    if len(candidates) < 40 or np.ptp(candidates[:, 0]) < 2 or np.ptp(candidates[:, 1]) < 2:
        raise Unsafe('Nearby flat-ground support too narrow')
    # Reject measured slope; low-quantile estimate is not a terrain oracle.
    fit = np.linalg.lstsq(np.column_stack((candidates[:, :2], np.ones(len(candidates)))),
                         candidates[:, 2], rcond=None)[0]
    if np.linalg.norm(fit[:2]) > math.tan(math.radians(3)):
        raise Unsafe('Observed ground is not flat enough (3 degree limit)')
    grid = np.zeros((SIZE, SIZE), dtype=np.uint8)  # 0 unknown, 1 observed free, 2 blocked, 3 ego assumption
    bins = {}
    for point in p[~ego]:
        ij = cell(point[:2])
        if inside(ij):
            bins.setdefault(ij, []).append(float(point[2]))
    for ij, heights in bins.items():
        h = np.asarray(heights)
        near = h[(h >= ground-.3) & (h <= ground+3)]
        if np.any((h > ground+.18) & (h < ground+3)) or np.any(h < ground-.18):
            grid[ij] = 2
        elif len(near) >= 2 and np.all(np.abs(near-ground) <= .12) and np.ptp(near) <= .12:
            grid[ij] = 1
    # Only cells entirely inside the initial trial footprint are assumed supported.
    # Never reapply this assumption at planned or moving positions.
    for i in range(SIZE):
        for j in range(SIZE):
            xy = center((i, j))
            if abs(xy[0])+RES/2 <= HALF_LENGTH and abs(xy[1])+RES/2 <= HALF_WIDTH:
                grid[i, j] = 3
    free = (grid == 1) | (grid == 3)
    safe = np.zeros_like(free)
    # Include grid-cell half diagonal: cell-center distance alone underinflates.
    offsets = [(i, j) for i in range(-4, 5) for j in range(-4, 5)
               if math.hypot(i*RES, j*RES) <= RADIUS + RES/math.sqrt(2)]
    for i, j in np.argwhere(free):
        # The initial occupied trial rectangle is assumed supported by definition,
        # so it is not eroded. Observed cells are still eroded by the inflation radius.
        if grid[i, j] == 3:
            safe[i, j] = True
        else:
            safe[i, j] = all(inside((i+di, j+dj)) and free[i+di, j+dj] for di, dj in offsets)
    return grid, safe, dict(ground_offset_m=ground, ground_fit=fit.tolist(),
                            observed_free_cells=int((grid == 1).sum()),
                            blocked_cells=int((grid == 2).sum()),
                            unknown_cells=int((grid == 0).sum()), resolution_m=RES,
                            inflation_radius_m=RADIUS, trial_half_length_m=HALF_LENGTH,
                            trial_half_width_m=HALF_WIDTH,
                            assumption='initial occupied trial rectangle only; not calibrated vehicle footprint')


def astar(safe, start, goal):
    if not inside(start) or not inside(goal) or not safe[start] or not safe[goal]:
        raise Unsafe('Start or goal unsupported after conservative inflation')
    todo = [(0., start)]
    costs, parent = {start: 0.}, {}
    while todo:
        _, here = heapq.heappop(todo)
        if here == goal:
            result = [here]
            while here in parent:
                here = parent[here]
                result.append(here)
            return list(reversed(result))
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                nxt = here[0]+di, here[1]+dj
                if (di == dj == 0) or not inside(nxt) or not safe[nxt]:
                    continue
                if di and dj and (not safe[here[0]+di, here[1]] or not safe[here[0], here[1]+dj]):
                    continue
                cost = costs[here] + math.hypot(di, dj)
                if cost < costs.get(nxt, float('inf')):
                    costs[nxt], parent[nxt] = cost, here
                    heuristic = math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                    heapq.heappush(todo, (cost+heuristic, nxt))
    raise Unsafe('No observed-ground A* path; unknown remains blocked')


def supported_segment(a, b, safe, grid):
    delta = np.asarray(b)-a
    length = np.linalg.norm(delta)
    if length < 1e-8:
        return False
    f = delta/length
    left = np.array([-f[1], f[0]])
    for t in np.linspace(0, 1, max(2, int(length/.1)+2)):
        pos = np.asarray(a)+t*delta
        ij = cell(pos)
        if not inside(ij) or not safe[ij]:
            return False
        # Conservative cell intersection with oriented rectangle, padded for
        # between-sample motion and cell extent. Checks body, not just center.
        reach = math.hypot(HALF_LENGTH, HALF_WIDTH)+RES
        lo, hi = cell(pos-reach), cell(pos+reach)
        for i in range(lo[0], hi[0]+1):
            for j in range(lo[1], hi[1]+1):
                rel = center((i, j))-pos
                if (abs(np.dot(rel, f)) <= HALF_LENGTH+RES/math.sqrt(2)+.1 and
                        abs(np.dot(rel, left)) <= HALF_WIDTH+RES/math.sqrt(2)+.1):
                    if not inside((i, j)) or grid[i, j] not in (1, 3):
                        return False
    return True


def plan(points, target):
    grid, safe, details = build_grid(points)
    raw = astar(safe, cell([0, 0]), cell(target))
    route = [np.zeros(2)] + [center(ij) for ij in raw[1:-1]] + [np.asarray(target, dtype=float)]
    simplified = [route[0]]
    k = 0
    while k < len(route)-1:
        next_k = next((j for j in range(len(route)-1, k, -1)
                       if supported_segment(route[k], route[j], safe, grid)), None)
        if next_k is None:
            raise Unsafe('A* route has no footprint-supported continuation')
        simplified.append(route[next_k])
        k = next_k
    path = np.asarray(simplified)
    headings = np.unwrap(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])))
    if abs(headings[0]) > math.radians(12) or np.any(np.abs(np.diff(headings)) > math.radians(12)):
        raise Unsafe('Sharp turn/initial heading unsupported; no forced steering fallback')
    # This deliberately narrow baseline executes only a straight visibility
    # shortcut of the A* solution. Polyline corners need a calibrated tracker.
    if len(path) != 2:
        raise Unsafe('Non-straight route needs curvature-calibrated tracking; plan rejected')
    details['astar_cells'] = [list(map(int, ij)) for ij in raw]
    details['path_local_xy'] = path.tolist()
    return grid, safe, details


def motion_guard(state, origin, basis, end, safe, grid):
    validate_state(state)
    pos = (np.asarray(state['pos'])-origin) @ basis
    speed = float(np.linalg.norm(state['vel']))
    signed = float(np.dot(state['vel'], unit(state['dir'])))
    direction = end / np.linalg.norm(end)
    along = float(np.dot(pos[:2], direction))
    lateral = abs(float(np.cross(np.append(direction, 0), np.append(pos[:2], 0))[2]))
    yaw = math.atan2(float(np.dot(unit(state['dir']), basis[:, 1])),
                     float(np.dot(unit(state['dir']), basis[:, 0])))
    path_yaw = math.atan2(end[1], end[0])
    if speed > 1.0 or signed < -.1 or along < -.2 or along > np.linalg.norm(end)+.3 or lateral > .2:
        raise Unsafe('Motion speed/reverse/endpoint/corridor limit exceeded')
    if abs(math.atan2(math.sin(yaw-path_yaw), math.cos(yaw-path_yaw))) > math.radians(15):
        raise Unsafe('Heading deviated from verified straight corridor')
    if not supported_segment(pos[:2], pos[:2]+.1*np.array([math.cos(yaw), math.sin(yaw)]), safe, grid):
        raise Unsafe('Current trial footprint left supported ground')
    return along, speed


def jsonable(value):
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, np.generic):
        return jsonable(value.item())
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def write_json(path, data):
    path.write_text(json.dumps(jsonable(data), indent=2, allow_nan=False), encoding='utf-8')


def save_grid(folder, grid, safe, details=None):
    np.save(folder/'map_grid.npy', grid, allow_pickle=False)
    np.save(folder/'safe_centers.npy', safe, allow_pickle=False)
    try:
        from PIL import Image
    except ImportError:
        return
    palette = np.array([[45, 45, 50], [160, 185, 160], [210, 65, 55], [200, 165, 65]], dtype=np.uint8)
    rgb = palette[grid].copy()
    rgb[safe] = [70, 140, 90]
    if details:
        for ij in details['astar_cells']:
            rgb[tuple(ij)] = [65, 135, 250]
    rgb[cell([0, 0])] = [255, 255, 255]
    Image.fromarray(np.flip(rgb, axis=0)).resize((650, 650), resample=Image.Resampling.NEAREST).save(folder/'path_grid.png')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=64256)
    p.add_argument('--vehicle', default='american_smoke_pickup')
    p.add_argument('--goal-forward', type=float, required=True)
    p.add_argument('--goal-left', type=float, default=0.)
    p.add_argument('--execute-motion', action='store_true')
    p.add_argument('--tracker', choices=['beamng-ai'])
    p.add_argument('--world-frame-verified', default='', metavar='LANDMARK_CHECK_NOTE')
    p.add_argument('--output-root', type=Path, default=Path('pointgoal-runs'))
    p.add_argument('--socket-timeout', type=float, default=2.)
    return p


def validate_args(args):
    distance = math.hypot(args.goal_forward, args.goal_left)
    if not math.isfinite(distance) or not .5 <= distance <= 10 or args.goal_forward <= 0:
        raise Unsafe('Goal must be finite, forward, and 0.5..10m from current ego pose')
    if not math.isfinite(args.socket_timeout) or not .1 <= args.socket_timeout <= 3:
        raise Unsafe('Socket timeout must be 0.1..3 seconds; not an emergency-stop guarantee')
    if args.execute_motion and (args.tracker != 'beamng-ai' or not args.world_frame_verified.strip()):
        raise Unsafe('Motion requires --tracker beamng-ai and --world-frame-verified with actual landmark-check note')


def run(args):
    validate_args(args)
    if importlib.metadata.version('beamngpy') != '1.36':
        raise Unsafe('This kit requires beamngpy==1.36')
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection
    from beamngpy.sensors import Electrics, Lidar

    folder = args.output_root.expanduser().resolve()/('pointgoal-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    print('Output:', folder, flush=True)
    print('LIVE DRY RUN ALSO DISABLES AI AND BRAKES THE EXISTING VEHICLE.', flush=True)
    report = dict(status='started', algorithm='observed-flat-ground A* + optional BeamNG AI straight tracker',
                  is_wildos=False, object_search=False, oracle_self_pose=True, gt_map=False,
                  world_frame_operator_evidence=args.world_frame_verified or None,
                  world_frame_landmark_verified_by_script=False,
                  args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                  versions={k: importlib.metadata.version(k) for k in ('beamngpy', 'numpy')})
    write_json(folder/'report.json', report)
    bng = BeamNGpy(args.host, args.port, quit_on_close=False, socket_timeout=args.socket_timeout)
    vehicle = lidar = None
    electrics = Electrics()
    events = (folder/'events.ndjson').open('w', encoding='utf-8', buffering=1)
    old_handlers = {}

    def log(kind, **data):
        events.write(json.dumps(jsonable(dict(event=kind, monotonic_s=time.monotonic(), **data)), allow_nan=False)+'\n')

    def state():
        vehicle.sensors.poll()
        s = dict(vehicle.state)
        e = {k: electrics.get(k) for k in ('gear', 'gear_index', 'gear_a', 'reverse', 'rpm', 'running',
             'wheelspeed', 'throttle_input', 'brake_input', 'parkingbrake_input', 'steering_input')}
        log('state', state=s, electrics=e)
        validate_state(s)
        return s

    def brake():
        # Attempt physical braking even if disabling AI fails; errors retained.
        errors = []
        for action in (lambda: vehicle.ai.set_mode('disabled'),
                       lambda: vehicle.control(throttle=0, brake=1, parkingbrake=1, steering=0)):
            try:
                action()
            except Exception as exc:
                errors.append(str(exc))
        log('brake', errors=errors)
        if errors:
            raise Unsafe('Best-effort braking error: '+'; '.join(errors))

    def settle(guard=None):
        brake()
        deadline = time.monotonic()+15
        first = None
        while time.monotonic() < deadline:
            s = state()
            if guard is not None:
                guard(s)
            if np.linalg.norm(s['vel']) > .08:
                first = None
            elif first is None:
                first = s
            elif float(s['time']) < float(first['time']):
                raise Unsafe('Simulation clock reset')
            elif (np.linalg.norm(np.asarray(s['pos'])-first['pos']) > .05 or
                  np.dot(unit(s['dir']), unit(first['dir'])) < math.cos(math.radians(.5))):
                first = s
            elif float(s['time'])-float(first['time']) >= 1.5:
                return s
            time.sleep(.05)
        raise Unsafe('Vehicle did not settle or simulation is paused')

    def scan(step):
        for attempt in range(3):
            capture = folder/('step_%02d_scan_%d' % (step, attempt))
            capture.mkdir()
            settle()
            before = state()
            sb = lidar.get_position()
            request_wall = time.monotonic()
            rid = lidar.send_ad_hoc_poll_request()
            deadline = time.monotonic()+8
            while not lidar.is_ad_hoc_poll_request_ready(rid):
                if time.monotonic() > deadline:
                    raise Unsafe('Fresh LiDAR request timeout; abort rather than queue more requests')
                time.sleep(.05)
            raw = lidar.collect_ad_hoc_poll_request(rid)
            # Preserve even malformed/invalid first clouds before quality checks.
            cloud = np.asarray(raw['pointCloud'])
            np.save(capture/'cloud_world_raw.npy', cloud, allow_pickle=False)
            sa = lidar.get_position()
            after = state()
            meta = dict(before=before, after=after, sensor_before=sb, sensor_after=sa,
                        request_id=rid, request_monotonic_s=request_wall,
                        receive_monotonic_s=time.monotonic(), returned_keys=list(raw))
            try:
                meta['quality'] = cloud_check(cloud, before, after, sb, sa)
            except Unsafe as exc:
                meta['rejected'] = str(exc)
                write_json(capture/'acquisition.json', meta)
                log('scan_rejected', attempt=attempt, reason=str(exc))
                continue
            write_json(capture/'acquisition.json', meta)
            basis = frame(after)
            points = (cloud-np.asarray(after['pos'])) @ basis
            np.save(capture/'cloud_local.npy', points, allow_pickle=False)
            return capture, after, basis, points
        raise Unsafe('All three stationary clouds rejected; raw clouds retained')

    def interrupted(signum, unused):
        raise KeyboardInterrupt('Signal '+str(signum))

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupted)
        bng.open(launch=False)
        if not bng.tech_enabled():
            raise Unsafe('BeamNG.tech sensor authorization unavailable')
        current = bng.vehicles.get_current(include_config=False)
        if args.vehicle not in current:
            raise Unsafe('Existing vehicle not found; available: '+', '.join(current))
        vehicle = current[args.vehicle]
        if vehicle.model != 'pickup':
            raise Unsafe('Trial footprint only intended for pickup; refusing other model')
        vehicle.sensors.attach('pointgoal_electrics', electrics)
        endpoint = bng.vehicles.start_connection(vehicle, vehicle.extensions)
        if endpoint['vid'] != vehicle.vid:
            raise Unsafe('Vehicle connection identity mismatch')
        vehicle.connection = Connection(args.host, int(endpoint['result']), timeout=args.socket_timeout)
        vehicle.connect(bng, tries=1)
        vehicle.ai.set_mode('disabled')
        vehicle.set_shift_mode('realistic_automatic')
        initial = settle()
        basis0 = frame(initial)
        goal_world = np.asarray(initial['pos']) + basis0 @ np.array([args.goal_forward, args.goal_left, 0.])
        report.update(initial_state=initial, goal_world=goal_world)
        lidar = Lidar('pointgoal_'+uuid.uuid4().hex[:10], bng, vehicle,
                      pos=(0, 0, 2.8), dir=(0, -1, 0), up=(0, 0, 1),
                      requested_update_time=-1, vertical_resolution=96, vertical_angle=130,
                      frequency=10, max_distance=60, is_360_mode=True, is_rotate_mode=False,
                      is_using_shared_memory=False, is_streaming=False, is_visualised=False,
                      is_annotated=False, is_static=False, is_snapping_desired=False,
                      is_force_inside_triangle=False, is_dir_world_space=False)
        for step in range(8):
            capture, pose, basis, points = scan(step)
            target = ((goal_world-np.asarray(pose['pos'])) @ basis)[:2]
            if np.linalg.norm(target) <= .35:
                report['status'] = 'arrived_stopped' if args.execute_motion else 'already_at_goal_stopped'
                break
            grid, safe, map_details = build_grid(points)
            save_grid(capture, grid, safe)
            write_json(capture/'map.json', map_details)
            grid, safe, details = plan(points, target)
            write_json(capture/'plan.json', details)
            save_grid(capture, grid, safe, details)
            log('plan', step=step, **details)
            if not args.execute_motion:
                report['status'] = 'dry_run_plan_ready_no_motion'
                break
            origin = np.asarray(pose['pos'])
            end = target * min(1., 2./np.linalg.norm(target))
            # Keep the first move straight ahead unless current heading is already
            # aligned. AI's turn-in swept path has not been calibrated.
            if abs(math.atan2(end[1], end[0])) > math.radians(3):
                raise Unsafe('Motion requires straight-ahead goal within 3 degrees; dry planning may allow 12')
            length = float(np.linalg.norm(end))
            line = []
            for t in (0., .5, 1.):
                wp = origin + basis @ np.array([end[0]*t, end[1]*t, 0.])
                line.append(dict(pos=wp.tolist(), speed=0. if t == 1 else .5))
            write_json(capture/'ai_line.json', dict(line=line, cling=False,
                       z_policy='constant initial vehicle-origin world Z on observed flat ground; not GT terrain'))
            log('execute_line', line=line, cling=False)
            # AI owns propulsion and transmission. Never guess automatic gear=1.
            vehicle.control(throttle=0, brake=0, parkingbrake=0, steering=0)
            vehicle.ai.set_line(line, cling=False)
            start_wall = last_wall = time.monotonic()
            last_sim = float(pose['time'])
            while True:
                s = state()
                now = time.monotonic()
                if now-last_wall > .5 or float(s['time']) <= last_sim:
                    raise Unsafe('Motion feedback stale/too slow (>0.5s) or simulation time stalled')
                last_wall, last_sim = now, float(s['time'])
                along, speed = motion_guard(s, origin, basis, end, safe, grid)
                if along >= length-.25:
                    break
                if now-start_wall > 12:
                    raise Unsafe('Chunk timeout/stuck/neutral; no throttle or gear escalation')
                time.sleep(.05)
            brake()
            stopped = settle(lambda s: motion_guard(s, origin, basis, end, safe, grid))
            motion_guard(stopped, origin, basis, end, safe, grid)
            if np.linalg.norm((np.asarray(stopped['pos'])-origin)[:2]) < .2:
                raise Unsafe('No meaningful chunk progress')
        else:
            raise Unsafe('Eight-chunk budget exhausted')
        report['final_state'] = state()
        report['final_goal_distance_m'] = float(np.linalg.norm((goal_world-np.asarray(vehicle.state['pos']))[:2]))
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='failed_stopped_not_guaranteed', error=type(exc).__name__+': '+str(exc))
        log('failure', error=report['error'])
    finally:
        # Ignore repeat signals during bounded best-effort stop/cleanup.
        for sig in old_handlers:
            signal.signal(sig, signal.SIG_IGN)
        errors = []
        actions = []
        if vehicle is not None and vehicle.is_connected():
            actions.append(('brake', brake))
        if lidar is not None:
            actions.append(('lidar.remove', lidar.remove))
        if vehicle is not None:
            actions.append(('vehicle.disconnect', vehicle.disconnect))
        actions.append(('bng.disconnect', bng.disconnect))
        for name, action in actions:
            try:
                action()
            except Exception as exc:
                errors.append(name+': '+str(exc))
        report['cleanup_errors'] = errors
        report['stop_guarantee'] = 'None on process kill/connection loss. Human simulator stop required.'
        if errors:
            report['status'] = 'cleanup_failed_stop_not_guaranteed'
        write_json(folder/'report.json', report)
        events.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    print(report['status']+': '+str(report.get('error', folder)), flush=True)
    return 0 if report['status'] in ('dry_run_plan_ready_no_motion', 'arrived_stopped', 'already_at_goal_stopped') else 1


def main():
    args = parser().parse_args()
    try:
        return run(args)
    except (Exception, KeyboardInterrupt) as exc:
        print(type(exc).__name__+': '+str(exc), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
