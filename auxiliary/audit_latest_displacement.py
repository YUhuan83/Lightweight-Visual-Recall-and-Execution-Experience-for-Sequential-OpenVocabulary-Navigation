"""Read closed field evidence only; no ROS or runtime mutation."""
import argparse
import bisect
import collections
import hashlib
import json
import math
import statistics
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('root', type=Path)
parser.add_argument('--output', default='displacement_audit.json')
parser.add_argument('--evidence-subdir', default='field')
args = parser.parse_args()
root = args.root
receipt = json.loads((root/'field_result.json').read_text(encoding='utf8'))
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
assert not receipt['postcheck']['display148_lock']
goal = next((root/args.evidence_subdir).glob('sessions/*/goal_001'))
native = [json.loads(line) for line in (goal/'native_samples.jsonl').read_text(encoding='utf8').splitlines()]
events = [json.loads(line) for line in (goal/'events.jsonl').read_text(encoding='utf8').splitlines()]
plans = sorted((e for e in events if e['kind'] == 'plan_status'), key=lambda e:e['stamp_ns'])
times = [e['stamp_ns']*1e-9 for e in plans]
# Event elapsed uses receipt time, status stamp uses generation time. Report the
# median offset and spread explicitly rather than asserting exact query crop.
offsets = [e['stamp_ns']*1e-9-e['elapsed'] for e in plans]
session_origin = statistics.median(offsets)
query = next(e for e in events if e['kind'] == 'query_sent')
query_start = session_origin + query['elapsed']
origin = native[0]['position'][:2]

def last_plan(stamp):
    i = bisect.bisect_right(times,stamp)-1
    return plans[i] if i >= 0 else {}

def measure(samples):
    totals = collections.Counter()
    states = collections.Counter()
    sources = {}
    owners = {}
    for a,b in zip(samples,samples[1:]):
        dt = b['wall_time']-a['wall_time']
        if not 0 < dt <= .5:
            totals['excluded_gap_s'] += max(dt,0)
            continue
        distance = math.dist(a['position'][:2],b['position'][:2])
        totals['coverage_s'] += dt
        totals['path_m'] += distance
        signed = sum(v*d for v,d in zip(a['velocity'],a['direction']))
        totals['reverse_m' if signed < -.05 else 'forward_m' if signed > .05 else 'near_stationary_m'] += distance
        p = last_plan(a['wall_time'])
        source = p.get('source','before_status')
        entry = sources.setdefault(source,collections.Counter())
        entry['time_s'] += dt
        entry['path_m'] += distance
        if a['speed'] < .1:
            totals['near_stop_s'] += dt
            states[p.get('status','before_status')] += dt
            entry['near_stop_s'] += dt
        identity = p.get('exploration_id')
        if identity is not None:
            item = owners.setdefault(identity,dict(time_s=0.,path_m=0.,goal=p.get('exploration_goal')))
            item['time_s'] += dt
            item['path_m'] += distance
    totals['all_sample_pairs_path_m'] = sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(samples,samples[1:]))
    totals['net_m'] = math.dist(samples[0]['position'][:2],samples[-1]['position'][:2]) if len(samples)>1 else 0
    totals['near_stop_pct'] = 100*totals['near_stop_s']/totals['coverage_s'] if totals['coverage_s'] else 0
    return dict(totals=dict(totals),stopped_by_latest_status_s=dict(states),by_latest_source={k:dict(v) for k,v in sources.items()},owners=sorted([dict(identity=k,**v) for k,v in owners.items()],key=lambda p:p['time_s'],reverse=True))

windows=[]
for lo,hi in ((0,30),(30,60),(60,90),(90,120),(120,150),(150,180)):
    samples = [n for n in native if lo <= n['wall_time']-query_start < hi]
    windows.append(dict(query_from_s=lo,query_to_s=hi,**measure(samples)))

stops=[]
start=None
for i,n in enumerate(native):
    if n['speed'] < .1:
        if start is None: start=i
    elif start is not None:
        if n['wall_time']-native[start]['wall_time'] >= 2:
            stops.append((start,i))
        start=None
if start is not None and native[-1]['wall_time']-native[start]['wall_time'] >= 2:
    stops.append((start,len(native)-1))

stop_report=[]
for first,last in sorted(stops,key=lambda pair:native[pair[1]]['wall_time']-native[pair[0]]['wall_time'],reverse=True):
    lo=native[first]['wall_time'];hi=native[last]['wall_time']
    preceding=last_plan(lo)
    sequence=[dict(query_s=p['stamp_ns']*1e-9-query_start,status=p['status'],source=p.get('source'),candidate=p.get('candidate_index'),serial=p.get('request_serial'),background=p.get('background'),goal=p.get('requested_goal_pose',p.get('goal')),reason=p.get('reason')) for p in plans if lo-1 <= p['stamp_ns']*1e-9 <= hi+.2]
    stopped_samples=native[first:last]
    telemetry=dict(samples=len(stopped_samples),max_age_s=max(n['telemetry_age_sec'] for n in stopped_samples),all_ok=all(n.get('telemetry_ok') and n.get('actuator_ok') for n in stopped_samples),watchdog_samples=sum(bool(n.get('watchdog_stop')) for n in stopped_samples),source_rejection_samples=sum(n.get('source_rejection') is not None for n in stopped_samples),braking_samples=sum(n.get('sent',{}).get('brake',0)>.5 for n in stopped_samples))
    stop_report.append(dict(query_start_s=lo-query_start,duration_s=hi-lo,preceding_status=preceding.get('status'),position=native[first]['position'][:2],telemetry=telemetry,events=sequence))

selected=[]
for p in plans:
    if p['status'] in ('visible_target_approach','visible_target_local_fallback','visible_target_no_vehicle_path','visible_target_approach_timeout','replanning_after_maneuver','visible_target_observation_step_done'):
        selected.append(dict(query_s=p['stamp_ns']*1e-9-query_start,status=p['status'],goal=p.get('goal'),reason=p.get('reason'),retained_current_section=p.get('retained_current_section')))
furthest=max(native,key=lambda n:math.dist(origin,n['position'][:2]))
out=dict(trial=receipt['trial'],source_hashes={f:hashlib.sha256((goal/f).read_bytes()).hexdigest() for f in ('events.jsonl','native_samples.jsonl')},time_alignment=dict(method='median of status source wall stamp minus event receipt elapsed; query time approximate',offset_spread_s=max(offsets)-min(offsets)),whole=measure(native),furthest_from_start_m=math.dist(origin,furthest['position'][:2]),furthest_query_s=furthest['wall_time']-query_start,windows=windows,long_stops=stop_report,approach_events=selected)
reference=receipt['fluency']
assert math.isclose(out['whole']['totals']['net_m'],reference['net_displacement_m'],abs_tol=1e-8)
assert math.isclose(out['whole']['totals']['near_stop_s'],reference['near_stop_s'],abs_tol=1e-8)
assert math.isclose(out['whole']['totals']['all_sample_pairs_path_m'],reference['sampled_distance_m'],abs_tol=1e-8)
(root/args.output).write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
compact=dict(trial=out['trial'],whole=out['whole']['totals'],furthest_from_start_m=out['furthest_from_start_m'],furthest_query_s=out['furthest_query_s'],windows=[dict(query_from_s=w['query_from_s'],query_to_s=w['query_to_s'],**w['totals']) for w in windows],long_stops=stop_report[:4],approach_events=selected)
print(json.dumps(compact,ensure_ascii=False,indent=2))
