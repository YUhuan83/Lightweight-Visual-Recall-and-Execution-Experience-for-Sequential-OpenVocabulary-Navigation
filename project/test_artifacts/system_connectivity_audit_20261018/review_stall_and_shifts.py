"""Read closed physical/control evidence; no simulator or production mutation."""
from pathlib import Path
import bisect
import collections
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SESSION = ROOT/'test_artifacts/smac_full_field_20261018/closed_evidence/session'
assert (SESSION/'session_result.json').is_file()
goal = json.loads((SESSION/'goal_001/result.json').read_text())
lo = goal['started']; hi = lo+goal['elapsed_s']
native = []; plans = []; odom = []; paths = []; control = {}; sources = {}; topics = collections.Counter()
def identity(v): return tuple(v.get(k) for k in ('sender_id', 'generation', 'sequence'))
def stats(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v))
    return dict(n=len(values), minimum=values[0], median=statistics.median(values), maximum=values[-1]) if values else None
def yaw(q): return math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
def xy(p): return (p['x'], p['y'])
for line in (SESSION/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row = json.loads(line); topic = row['topic']; t = row['received_monotonic_ns']/1e9
    topics[topic] += 1
    if topic == '/wildos/native_control_command':
        value = json.loads(row['message']['data']); sources[identity(value)] = value
    elif topic == '/wildos/control_status':
        value = json.loads(row['message']['data']); control[identity(value.get('source_control') or {})] = value
    elif topic == '/wildos/native_control_status' and lo <= t <= hi:
        native.append((t, json.loads(row['message']['data'])))
    elif topic == '/wildos/plan_status' and lo <= t <= hi: plans.append((t, json.loads(row['message']['data'])))
    elif topic == '/spot1/odom' and lo <= t <= hi: odom.append((t, row['message']))
    elif topic in ('/wildos/planned_path', '/received_global_plan') and lo <= t <= hi:
        paths.append((t, topic, row['message']))

def matched_control(value):
    receipt = value.get('source_receipt') or {}; key = identity(receipt)
    frame, status = sources.get(key), control.get(key)
    if (not frame or not status or receipt.get('applied') is not True or
        any(receipt.get(k) != frame.get(k) for k in ('issued_wall_ns', 'expires_wall_ns', 'control')) or
        receipt['control'] != value['sent']): return None
    return status

stall_reports = []
for when, event in plans:
    if event.get('reason') != 'execution_stalled': continue
    duration = event['no_progress_sec']
    window = [(t, v, matched_control(v)) for t, v in native if when-duration <= t <= when]
    positions = [v['position'][:2] for _, v, _ in window]
    odom_window = [(t, v) for t, v in odom if when-duration <= t <= when]
    contacts = {}
    for t, value, _ in window:
        contact = value.get('wheel_contact') or {}
        stamp = contact.get('wall_time')
        if stamp and 0 <= value['wall_time']-stamp <= 1.5:
            contacts[stamp] = contact
    wheels = [w for c in contacts.values() for w in c.get('wheels', [])]
    preceding_paths = [(t, topic, v) for t, topic, v in paths if t <= when]
    last_by_topic = {}
    for t, topic, value in preceding_paths: last_by_topic[topic] = (t, value)
    path_reports = {}
    for topic, (t, path) in last_by_topic.items():
        points = [xy(p['pose']['position']) for p in path['poses']]
        path_reports[topic] = dict(goal_elapsed=t-lo, frame=path['header']['frame_id'],
            poses=len(points), first=points[0] if points else None,
            last=points[-1] if points else None,
            length=sum(math.dist(a, b) for a, b in zip(points, points[1:])))
    world_path = last_by_topic.get('/wildos/planned_path')
    lateral_errors = []
    if world_path:
        pp = [xy(p['pose']['position']) for p in world_path[1]['poses']]
        for _, o in odom_window:
            point = xy(o['pose']['pose']['position']); gaps = []
            for a, b in zip(pp, pp[1:]):
                dx, dy = b[0]-a[0], b[1]-a[1]; squared = dx*dx+dy*dy
                fraction = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/squared)) if squared else 0.
                gaps.append(math.dist(point, (a[0]+fraction*dx, a[1]+fraction*dy)))
            lateral_errors.append(min(gaps))
    stall_reports.append(dict(event_goal_elapsed=when-lo, event=event, samples=len(window),
        matched_source_controls=sum(c is not None for _, _, c in window),
        position_displacement=math.dist(positions[0], positions[-1]),
        travelled=sum(math.dist(a, b) for a, b in zip(positions, positions[1:])),
        source_reasons=dict(collections.Counter(c['reason'] if c else 'unmatched' for _, _, c in window)),
        target_speed=stats([c['target_speed'] for _, _, c in window if c]),
        target_angular=stats([c['target_angular'] for _, _, c in window if c]),
        sent_throttle=stats([v['sent']['throttle'] for _, v, _ in window]),
        sent_brake=stats([v['sent']['brake'] for _, v, _ in window]),
        sent_steering=stats([v['sent']['steering'] for _, v, _ in window]),
        speed=stats([v['speed'] for _, v, _ in window]),
        actual={k: stats([v.get('actual', {}).get(k) for _, v, _ in window])
                for k in ('throttle', 'brake', 'steering_input', 'wheelspeed', 'rpm', 'clutchRatio1')},
        engine_running_values=sorted({str(v.get('actual', {}).get('engineRunning')) for _, v, _ in window}),
        actual_gear_counts=dict(collections.Counter(str(v.get('actual', {}).get('gear')) for _, v, _ in window)),
        native_actuator_healthy=all(v.get('actuator_ok') is True for _, v, _ in window),
        native_telemetry_healthy=all(v.get('telemetry_ok') is True for _, v, _ in window),
        watchdog_samples=sum(v['watchdog_stop'] for _, v, _ in window),
        body_pitch_radians=stats([math.atan2(v['direction'][2], math.hypot(*v['direction'][:2])) for _, v, _ in window]),
        rear_axle_cross_track_m=stats(lateral_errors),
        demanded_curvature=stats([c['target_angular']/c['target_speed'] for _, _, c in window if c and abs(c['target_speed']) > .01]),
        odom_samples=len(odom_window),
        odom_start=(xy(odom_window[0][1]['pose']['pose']['position']) if odom_window else None),
        odom_end=(xy(odom_window[-1][1]['pose']['pose']['position']) if odom_window else None),
        last_paths=path_reports,
        contacts=dict(snapshots=len(contacts), wheels=len(wheels),
                      downforce=stats([w.get('downForceRaw') for w in wheels]),
                      propulsion_torque=stats([w.get('propulsionTorque') for w in wheels]),
                      propulsion_torque_abs=stats([abs(w['propulsionTorque']) for w in wheels if 'propulsionTorque' in w]),
                      braking_torque=stats([w.get('brakingTorque') for w in wheels]),
                      wheel_speed=stats([w.get('wheelSpeed') for w in wheels]),
                      samples=list(contacts.values())),
        limits='Window follows public stall status receipt. SDK electrics can precede sent command; wheel snapshots deduplicated by source wall time. No collision object identity recorded.'))
    approach = next((p for t, p in reversed(plans) if t <= when and
                     p['status'] == 'visible_target_approach'), None)
    release = next(((t, p) for t, p in plans if t > when and
                    p['status'] == 'visible_target_route_timeout'), None)
    stall_reports[-1]['visible_handoff'] = dict(
        approach_age_at_stall_sec=(event['stamp_ns']-approach['stamp_ns'])*1e-9 if approach else None,
        next_release_after_stall_sec=release[0]-when if release else None,
        release=release[1] if release else None,
        recovery_started_status_count=sum(p['status'] == 'recovery_started' for _, p in plans))

# Matching applied command ownership rather than nearest status avoids calling
# a shift request executed merely because its publisher logged it.
shift_episodes = []; episode = None
for (t, v), (end, _) in zip(native, native[1:]):
    c = matched_control(v)
    shifting = c is not None and c['reason'] == 'gear_change' and v['speed'] < .05
    if shifting:
        if episode is None:
            episode = dict(start=t-lo, covered_sec=0., samples=0, pending_gears=[], actual_gears=[], target_signs=[])
        episode['covered_sec'] += min(.2, max(0., end-t)); episode['samples'] += 1
        episode['pending_gears'].append(c.get('pending_gear'))
        episode['actual_gears'].append(v.get('actual', {}).get('gear'))
        episode['target_signs'].append(math.copysign(1., c['target_speed']))
    elif episode is not None:
        shift_episodes.append(episode); episode = None
if episode is not None: shift_episodes.append(episode)
for episode in shift_episodes:
    for name in ('pending_gears', 'actual_gears', 'target_signs'):
        episode[name] = dict(collections.Counter(str(x) for x in episode[name]))
requested_changes = []; actual_changes = []; last_requested = last_actual = None
for t, value in native:
    c = matched_control(value)
    requested = value['sent'].get('gear') if c else None
    actual = value.get('actual', {}).get('gear')
    if requested in (2, -1) and requested != last_requested:
        if last_requested is not None: requested_changes.append(dict(goal_elapsed=t-lo, gear=requested))
        last_requested = requested
    if actual in ('D', 'R') and actual != last_actual:
        if last_actual is not None: actual_changes.append(dict(goal_elapsed=t-lo, gear=actual))
        last_actual = actual
canonical = json.loads((SESSION/'full_query_review.json').read_text())
assert abs(sum(e['covered_sec'] for e in shift_episodes)-canonical['stopped_by_source_reason']['gear_change']) < 1e-6
report = dict(scope=__doc__, session=str(SESSION), stalls=stall_reports,
    shift_episode_count=len(shift_episodes), shift_stopped_sec=sum(e['covered_sec'] for e in shift_episodes),
    shift_count_note='Episodes count contiguous stopped samples with gear_change reason, not completed shifts.',
    requested_direction_changes=len(requested_changes), actual_selector_changes=len(actual_changes),
    requested_changes=requested_changes, actual_changes=actual_changes,
    shift_episodes=shift_episodes, observed_topics=dict(topics),
    production_modified=False, new_navigation_test=False,
    contact_object_identified=False, physical_model_calibrated=False)
(OUT/'stall_and_shifts.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
compact = {k: v for k, v in report.items() if k not in ('observed_topics', 'shift_episodes', 'requested_changes', 'actual_changes')}
for stall in compact['stalls']: stall['contacts'] = {k: v for k, v in stall['contacts'].items() if k != 'samples'}
print(json.dumps(compact, indent=2))
