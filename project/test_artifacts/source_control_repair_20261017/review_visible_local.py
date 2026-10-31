"""Closed field evidence: correlate temporary approach, controller and source ACKs."""
import bisect
import collections
import json
import math
import statistics
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
folder = root/'closed_local'
session = folder/'field_results/session_20261017_151623'
window = json.loads((folder/'visible_window.json').read_text(encoding='utf8'))
origin = window['origin_wall']
rows = [json.loads(l) for l in (session/'execution_observer/messages.jsonl').open(encoding='utf8')]
by_topic = collections.defaultdict(list)
for row in rows:
    by_topic[row['topic']].append(row)

def stats(xs):
    if not xs:
        return None
    xs = sorted(xs)
    return dict(count=len(xs), median=statistics.median(xs), p95=xs[int((len(xs)-1)*.95)], max=xs[-1])

def identity(x):
    return tuple(x[k] for k in ('sender_id','generation','sequence'))

issued = {identity(x): x for row in by_topic['/wildos/native_control_command']
          for x in [json.loads(row['message']['data'])]}
sampled = [json.loads(l) for l in (session/'goal_001/native_samples.jsonl').open(encoding='utf8')]
lo,hi = sampled[0]['wall_time'],sampled[-1]['wall_time']
native = [x for row in by_topic['/wildos/native_control_status']
          for x in [json.loads(row['message']['data'])] if lo <= x['wall_time'] <= hi]
matched = 0
bad = []
for x in native:
    rec = x.get('source_receipt')
    if rec and rec.get('applied') is True:
        frame = issued.get(identity(rec))
        valid = (frame is not None and
            all(rec[k] == frame[k] for k in ('issued_wall_ns','expires_wall_ns','control')) and
            rec['control'] == x['sent'] and
            rec['issued_wall_ns'] <= rec['applied_wall_ns'] < rec['expires_wall_ns'])
        if valid:
            matched += 1
        else:
            bad.append(x['wall_time'])
    elif x.get('control_acknowledged') is True and x['sent']['throttle'] > 0. and x['sent']['brake'] < .9:
        bad.append(x['wall_time'])
source = dict(scope='All passive Native messages during goal sample time range; delivery loss is still possible',
    sample_range=[lo,hi], native_messages=len(native), issued_observed=len(issued),
    matched_timely_source_sdk_acks=matched, invalid_or_positive_without_valid_ack=bad,
    source_age_sec=stats([x['command_age'] for x in native]),
    source_age_at_callback_sec=stats([x['command_age']-x['command_receipt_age'] for x in native]),
    control_rpc_ms=stats([x['control_rpc_ms'] for x in native if x.get('control_rpc_ms') is not None]),
    watchdog_messages=sum(bool(x.get('watchdog_stop')) for x in native),
    actuator_failed_messages=sum(x.get('actuator_ok') is not True for x in native),
    telemetry_unhealthy_messages=sum(x.get('telemetry_ok') is not True for x in native))
(folder/'source_control_full_observer_review.json').write_text(json.dumps(source,indent=2),encoding='utf8')

def xy(p):
    return (p['position']['x'],p['position']['y'])

odom = [(r['received_ros_ns']/1e9,xy(r['message']['pose']['pose'])) for r in by_topic['/spot1/odom']]
paths = []
for r in by_topic['/wildos/planned_path']:
    t = r['received_ros_ns']/1e9
    if origin+375 <= t < origin+426:
        paths.append(dict(wall=t,xy=[xy(z['pose']) for z in r['message']['poses']]))

def lateral(point,path):
    p = np.array(point); points = np.array(path)
    a,b = points[:-1],points[1:]
    delta = b-a; denominator = np.sum(delta*delta,axis=1)
    u = np.sum((p-a)*delta,axis=1)/np.maximum(denominator,1e-20)
    return float(np.min(np.linalg.norm(p-a-np.clip(u,0,1)[:,None]*delta,axis=1)))

events = [json.loads(l) for l in (session/'goal_001/events.jsonl').open(encoding='utf8')]
events = [e for e in events if e.get('kind')=='plan_status']
summaries = []
for i,p in enumerate(paths):
    t = p['wall']; end = paths[i+1]['wall'] if i+1 < len(paths) else origin+424.34
    event = min((e for e in events if e['status']=='planned'), key=lambda e:abs(e['stamp_ns']/1e9-t))
    terminal = next((e for e in events if e['stamp_ns']/1e9>t and e['status'] in
                     ('maneuver_done','segment_done','route_done','exploration_segment_finished','follow_failed')),
                    None)
    if terminal:
        end = min(end, terminal['stamp_ns']/1e9)
    actual = [(ts,pnt) for ts,pnt in odom if t <= ts <= end]
    control = [r['message'] for r in by_topic['/wildos/cmd_vel'] if t <= r['received_ros_ns']/1e9 <= end]
    received = [r for r in by_topic['/received_global_plan'] if t <= r['received_ros_ns']/1e9 <= end]
    lookahead = [r for r in by_topic['/lookahead_point'] if t <= r['received_ros_ns']/1e9 <= end]
    summary = dict(relative_sec=t-origin, duration_sec=end-t, source=event['source'],
        start=p['xy'][0], goal=p['xy'][-1], poses=len(p['xy']),
        length_m=sum(math.dist(a,b) for a,b in zip(p['xy'],p['xy'][1:])),
        controller_transformed_plan_messages=len(received), lookahead_messages=len(lookahead),
        command_count=len(control), linear_min=min((c['linear']['x'] for c in control),default=None),
        linear_max=max((c['linear']['x'] for c in control),default=None),
        actual_nearest_path_distance_m=stats([lateral(pnt,p['xy']) for _,pnt in actual]),
        actual_min_goal_distance_m=min((math.dist(pnt,p['xy'][-1]) for _,pnt in actual),default=None),
        terminal=terminal['status'] if terminal else None)
    summaries.append(summary)
mask = window['goal_bag']['/spot1/object_mask']
finish = next(e for e in events if e['status']=='visible_target_observation_step_done')
at = finish['stamp_ns']/1e9
previous = max((m for m in mask if m['received_wall'] <= at),key=lambda m:m['stamp'])
report = dict(origin_wall=origin, scope=window['scope'], sections=summaries,
    observation_step_finished_relative_sec=at-origin,
    last_mask_capture_age_at_finish_sec=at-previous['stamp'],
    mask_capture_query_relative_sec=[m['stamp']-origin for m in mask],
    next_approach_relative_sec=next(e['stamp_ns']/1e9-origin for e in events
        if e['status']=='visible_target_approach' and e['stamp_ns']/1e9>at))
(folder/'visible_execution_review.json').write_text(json.dumps(report,indent=2),encoding='utf8')

fig,axes = plt.subplots(1,2,figsize=(12,6),sharex=True,sharey=True)
for ax in axes:
    ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('World x (m)')
    ax.set_ylabel('World y (m)')
for ax,low,high,title in zip(axes,(375,397.13),(397.13,424.34),
        ('Temporary visual approach: 6:16 to 6:37','After visual release: 6:37 to 7:04')):
    for p in window['sensor_bag']['/wildos/full_planned_path']:
        if low <= p['relative_sec'] < high:
            points=np.array(p['poses']);ax.plot(points[:,0],points[:,1],color='.65',lw=2,alpha=.7,
                                              label='Full candidate path')
    for i,p in enumerate(paths):
        if low <= p['wall']-origin < high:
            points=np.array(p['xy']);ax.plot(points[:,0],points[:,1],lw=2,
                label=f'Executed section {p["wall"]-origin:.1f}s')
    points=np.array([pnt for t,pnt in odom if low <= t-origin <= high])
    ax.plot(points[:,0],points[:,1],'k--',lw=1.5,label='Actual odometry')
    ax.scatter([711.53463],[-48.02006],marker='X',s=100,c='red',zorder=5,label='Temporary observation goal')
    hints=[h['data']['point'] for h in window['goal_bag']['/spot1/visible_target_approach']
           if 392 <= h['relative_sec'] <= 396]
    points=np.array(hints);ax.scatter(points[:,0],points[:,1],c='green',s=18,label='Later bearing-only hints')
    ax.set_title(title);ax.legend(fontsize=7,loc='upper right')
fig.tight_layout();fig.savefig(folder/'visible_execution_paths.png',dpi=160);plt.close(fig)
print(json.dumps(dict(full_source=source,visual=report),indent=2))
