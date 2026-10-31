"""Closed local evidence only. Never connects to ROS, DDS or a server."""
from pathlib import Path
import bisect,collections,hashlib,json,math,statistics,sys,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.tracking_preview import preview_tracking
old=ROOT/'test_artifacts/frontier_motion_handoff_20261016'
base=old/'server_field';session=base/'field_results/session_20261016_205443';goal=session/'goal_001'
assert (session/'session_result.json').is_file() and (goal/'result.json').is_file()
assert (base/'field_scope_baseline_search_20261016_205441/result.json').is_file()
manifest=json.loads((ROOT/'deploy/20261016_frontier_motion_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
mismatches=[n for n,h in manifest['members'].items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
assert not mismatches,mismatches
identity=json.loads((old/'field_archive_identity.json').read_text(encoding='utf8'))
archive=old/Path(identity['path']).name
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
events=[json.loads(l) for l in (goal/'events.jsonl').read_text(encoding='utf8').splitlines()]
plans=[e for e in events if e.get('kind')=='plan_status'];planned=[e for e in plans if e['status']=='planned']
failures=[e for e in plans if e['status']=='follow_failed']
topics=['/wildos/planned_path','/received_global_plan','/lookahead_point','/spot1/odom',
        '/wildos/control_status','/wildos/native_control_status','/wildos/cmd_vel']
rows={t:[] for t in topics}
with zipfile.ZipFile(archive) as z:
    name=next(n for n in z.namelist() if n.endswith('messages.jsonl'))
    for line in z.open(name):
        d=json.loads(line)
        if d['topic'] not in rows:continue
        if d['topic'].endswith('status'):d['payload']=json.loads(d['message']['data'])
        rows[d['topic']].append(d)
def stats(a):
    a=sorted(x for x in a if isinstance(x,(float,int)) and math.isfinite(x))
    return dict(count=len(a),median=statistics.median(a),p95=a[math.ceil((len(a)-1)*.95)],maximum=a[-1]) if a else None
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def pose(p):return [p['position']['x'],p['position']['y'],yaw(p['orientation'])]
times={t:[r['received_ros_ns'] for r in rs] for t,rs in rows.items()}
def before(topic,t):
    i=bisect.bisect_right(times[topic],t)-1
    return rows[topic][i] if i>=0 else None
controls=sorted((r['payload'] for r in rows['/wildos/control_status']),key=lambda x:x['stamp_ns'])
goal_native=[json.loads(l) for l in (goal/'native_samples.jsonl').read_text(encoding='utf8').splitlines()]
active_start,active_end=goal_native[0]['wall_time'],goal_native[-1]['wall_time']
controls=[c for c in controls if active_start<=c['stamp_ns']/1e9<=active_end]
ct=[x['stamp_ns']/1e9 for x in controls]
native=sorted((r['payload'] for r in rows['/wildos/native_control_status']),key=lambda x:x['wall_time'])
native=[n for n in native if active_start<=n['wall_time']<=active_end]
nt=[n['wall_time'] for n in native]
conflicts=[]
for n in native:
    i=bisect.bisect_right(ct,n['wall_time'])-1
    if i<0:continue
    c=controls[i];age=n['wall_time']-ct[i]
    if (0<=age<=.1 and c['brake']>.2 and c['throttle']==0.
            and n['sent']['throttle']>.001 and n['sent']['brake']==0.):
        conflicts.append(dict(wall=n['wall_time'],control_stamp_ns=c['stamp_ns'],
            output_brake=c['brake'],sent=n['sent'],speed=n['speed'],command_age=n['command_age']))
cases=[]
for fail in failures:
    at=fail['stamp_ns'];p=next(p for p in reversed(planned) if p['stamp_ns']<at)
    path=before('/wildos/planned_path',p['stamp_ns']+20_000_000)['message']
    points=[pose(x['pose'])[:2] for x in path['poses']];headings=[pose(x['pose'])[2] for x in path['poses']]
    start=p['actual_start_pose'];pr=preview_tracking(points,headings,start[:2],start[2],8.,position_tolerance=p['goal_position_tolerance_m'])
    window=[c for c in controls if p['stamp_ns']<=c['stamp_ns']<=at]
    positive=[c for c in window if c['throttle']>.001 and c['brake']==0.]
    last_throttle=positive[-1]['stamp_ns'] if positive else p['stamp_ns']
    braking=next((c for c in window if c['stamp_ns']>last_throttle and c['brake']>.01),None)
    sent_brake=next((n for n in native if braking and n['wall_time']>=braking['stamp_ns']/1e9 and n['sent']['brake']>.01),None)
    disk=[]
    for r in rows['/received_global_plan']:
        if not p['stamp_ns']<=r['received_ros_ns']<=at:continue
        poses=r['message']['poses']
        # Only a terminal path which includes the actual accepted endpoint.
        # Cropped path endpoints are not arrival evidence.
        if not poses:continue
        o=before('/spot1/odom',r['received_ros_ns'])
        if o is None:continue
        observed=pose(o['message']['pose']['pose'])
        gap=math.dist(observed[:2],p['goal'])
        yaw_error=abs(math.remainder(observed[2]-headings[-1],2*math.pi))
        n=native[max(0,bisect.bisect_right(nt,r['received_ros_ns']/1e9)-1)]
        if gap<=p['goal_position_tolerance_m'] and yaw_error<=.35:
            disk.append(dict(received_ns=r['received_ros_ns'],odom_gap_m=gap,native_speed=n['speed'],
                             yaw_error_rad=yaw_error,native_sample_age_s=n['telemetry_age_sec']))
    cases.append(dict(failure=fail,planned=p,preview=dict(reached=pr.reached,reason=pr.reason,closest_m=pr.closest_m),
        first_brake=braking,first_native_brake=sent_brake,
        brake_output_to_native_s=(sent_brake['wall_time']-braking['stamp_ns']/1e9) if braking and sent_brake else None,
        recorded_odom_inside_disk=disk))
# Exact control topic signatures for the three previously exported closed windows.
delivery=json.loads((ROOT/'test_artifacts/cusp_execution_repair_20261016/delivery_review.json').read_text())
matched=[]
def signature(x):return tuple(x[k] for k in ('throttle','brake','steering','gear'))
for w in delivery['windows']:
    for n in w['native']:
        if n['sent']['throttle']<=.001 or n['command_age']>.05:continue
        source=[c for c in w['controls'] if signature(c)==signature(n['sent']) and c['received_ns']<=n['received_ns']]
        if source:
            c=source[-1]
            matched.append(dict(failure_wall=w['failure_wall'],native_wall=n['wall_time'],
                matched_control_received_ns=c['received_ns'],sent=n['sent'],
                minimum_source_to_sent_s=n['wall_time']-c['received_ns']/1e9,
                native_receipt_based_command_age_s=n['command_age']))
field=json.loads((base/'field/baseline_summary.json').read_text(encoding='utf8'))
summary=dict(scope='Offline closed-recording/source audit; no new vehicle/DDS experiment or production edit',
    source=dict(candidate=manifest['sha256'],verified_members=len(manifest['members']),archive=identity['sha256']),
    active_window_wall=[active_start,active_end],
    controls=len(controls),native_samples=len(native),
    native_command_age_s=stats([n['command_age'] for n in native]),
    native_command_age_counts={str(limit):sum(n['command_age']>limit for n in native) for limit in (.1,.2,.4,.5)},
    native_control_rpc_ms=stats([n['control_rpc_ms'] for n in native]),
    command_tick_period_s=stats([n['period'] for n in native]),
    native_telemetry_age_s=stats([n['telemetry_age_sec'] for n in native]),
    control_state_receipt_age_s=stats([c['state_receipt_age_steady_sec'] for c in controls]),
    control_state_source_age_s=stats([c['state_source_age_ros_sec'] for c in controls]),
    upstream_braking_while_native_sends_throttle=len(conflicts),
    exact_old_source_matches=matched,
    failures=[dict(elapsed=x['failure']['elapsed'],preview=x['preview'],
        motion_sections=x['planned']['motion_sections_remaining'],checker=x['planned']['goal_checker_id'],
        brake_output_to_native_s=x['brake_output_to_native_s'],
        inside_disk_samples=len(x['recorded_odom_inside_disk']),
        inside_disk_native_speed_s=stats([r['native_speed'] for r in x['recorded_odom_inside_disk']])) for x in cases],
    field=dict(state=field['goal']['state'],masks=field['masks'],confirmations=field['confirmations'],
        memory_priors_disabled=field['memory_priors_disabled'],stops=field['stationary_intervals_ge5s']),
    limitations=['Exact source matches identify payload ages in the bag, not DDS callback entry timestamps',
        'Repeated signatures can only bound age; receipt age is not producer source age',
        'Odom disk entry uses recorded delayed poses and is not a simultaneous ground-truth stop claim',
        'DDS reliability/executor/CPU attribution requires a controlled ROS experiment; none run here'])
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
(OUT/'failure_cases.json').write_text(json.dumps(cases,indent=2)+'\n',encoding='utf8')
(OUT/'brake_conflicts.json').write_text(json.dumps(conflicts,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,indent=2))
