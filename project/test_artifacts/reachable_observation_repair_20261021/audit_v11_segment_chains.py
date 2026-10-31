"""Read source-stamped closed v11 status/path/native logs; no ROS or mutations."""
from pathlib import Path
import bisect
from collections import Counter
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parent
TRIAL = ROOT / 'server_v11/full/trial_20261021_221131'
SESSION = TRIAL / 'sessions/session_20261021_221131'
GOAL = SESSION / 'goal_001'
SUMMARY = json.loads((TRIAL/'fluency_audit/closed_trial_summary.json').read_text(encoding='utf8'))
Q0 = SUMMARY['query_start_bag_wall']
events = [json.loads(line) for line in (GOAL/'events.jsonl').read_text(encoding='utf8').splitlines()]
native = [json.loads(line) for line in (GOAL/'native_samples.jsonl').read_text(encoding='utf8').splitlines()]
native.sort(key=lambda n:n['wall_time'])
native_t = [n['wall_time'] for n in native]
states = sorted((e for e in events if e.get('kind')=='plan_status'), key=lambda e:e['stamp_ns'])
states = [e for e in states if 0 <= e['stamp_ns']/1e9-Q0 <= 180]
plans = [e for e in states if e.get('status')=='planned']
paths = [e for e in events if e.get('kind')=='planned_path']


def relative(e):
    return e['stamp_ns']/1e9-Q0


def owner(e):
    return (e.get('exploration_id') or e.get('retired_exploration_id') or '').rsplit(':',1)[-1]


def measured(a,b):
    """Native position deltas only; don't compare absolute native origins with odom."""
    ai = min(range(len(native)), key=lambda i:abs(native_t[i]-(Q0+a)))
    bi = min(range(len(native)), key=lambda i:abs(native_t[i]-(Q0+b)))
    first,last = native[ai],native[bi]
    travel = sum(math.dist(x['position'][:2],y['position'][:2]) for x,y in zip(native[ai:bi+1],native[ai+1:bi+1]))
    return dict(first_source_s=first['wall_time']-Q0,last_source_s=last['wall_time']-Q0,
                first_time_error_s=first['wall_time']-(Q0+a),last_time_error_s=last['wall_time']-(Q0+b),
                delta_xy=[last['position'][i]-first['position'][i] for i in range(2)],
                net_m=math.dist(first['position'][:2],last['position'][:2]),sampled_travel_m=travel)


rows=[]
for i,p in enumerate(plans):
    path=next(v for v in paths if math.dist(v['points'][-1],p['goal'])<.01 and v['poses']==p['poses'])
    paths.remove(path)
    dx=path['points'][1][0]-path['points'][0][0]
    dy=path['points'][1][1]-path['points'][0][1]
    cos=math.cos(math.atan2(dy,dx)-path['headings'][0])
    t=relative(p)
    tn=relative(plans[i+1]) if i+1<len(plans) else 180
    between=[e for e in states if (relative(plans[i-1]) if i else -1)<relative(e)<t]
    seen_owner=any(owner(e)==owner(p) for e in plans[:i])
    prior_done=next((e for e in reversed(between) if e.get('status')=='maneuver_done' and owner(e)==owner(p)),None)
    replan=next((e for e in reversed(between) if e.get('status')=='replanning_after_maneuver' and owner(e)==owner(p)),None)
    cause='first_dispatch_for_owner' if not seen_owner else 'collision_replan' if replan else 'queue_continuation' if prior_done else 'unclassified'
    selected=[e for e in states if t<=relative(e)<tn and e.get('status') not in ('following','planning','planned')]
    rows.append(dict(index=i+1,source_s=t,owner=owner(p),dispatch_kind=cause,
                     goal=p['goal'],graph_goal=p['graph_goal'],actual_start_pose=p['actual_start_pose'],
                     actual_start_stamp_ns=p['actual_start_stamp_ns'],motion_sections_remaining=p.get('motion_sections_remaining'),
                     goal_checker=p.get('goal_checker_id'),poses=p['poses'],path_length_m=path['length'],
                     path_first_cos=cos,path_direction='D' if cos>0 else 'R',path_start=path['points'][0],path_end=path['points'][-1],
                     nominal_graph_distance_before_m=math.dist(p['actual_start_pose'][:2],p['graph_goal']),
                     nominal_graph_distance_after_section_m=math.dist(p['goal'],p['graph_goal']),
                     next_dispatch_s=tn,native_interval_to_next_dispatch=measured(t,tn),
                     intervening_states=[dict(source_s=relative(e),status=e['status'],owner=owner(e),**{k:e[k] for k in ('reason','retirement_reason','motion_sections_remaining','remaining_distance_m','goal','position_error','heading_error') if k in e}) for e in selected]))

direction_changes=[]
for prev,row in zip(rows,rows[1:]):
    if prev['path_direction']==row['path_direction']:
        continue
    switches=[e for e in SUMMARY['sent_gear_runs'][1:] if row['source_s']<=e['source_s']<row['next_dispatch_s']]
    direction_changes.append(dict(source_s=row['source_s'],index=row['index'],owner=row['owner'],
                                  dispatch_kind=row['dispatch_kind'],direction=row['path_direction'],native_sent_gear_changes=switches))

cases=[]
for name,ids,end in [('initial_short_forward_reverse_retirement',['2'],14.079913854598999),
                     ('same_owner_collision_replan_then_retired_long_tail',['4'],69.63308024406433),
                     ('short_cusps_before_productive_tail_is_retired',['10'],116.42919945716858)]:
    selected=[row for row in rows if row['owner'] in ids]
    start=selected[0]['source_s']
    case_states=[e for e in states if start<=relative(e)<=end+.01]
    later=next((row for row in rows if row['source_s']>end),None)
    cases.append(dict(name=name,owners=ids,start_s=start,end_s=end,planned_indices=[r['index'] for r in selected],
                      measured=measured(start,end),
                      states=[dict(source_s=relative(e),status=e['status'],owner=owner(e),**{k:e[k] for k in ('reason','retirement_reason','goal','graph_goal','motion_sections_remaining','remaining_distance_m','position_error','heading_error') if k in e}) for e in case_states if e['status']!='following'],
                      next_dispatch=None if later is None else dict(source_s=later['source_s'],owner=later['owner'],goal=later['goal'],graph_goal=later['graph_goal'])))

report=dict(query_start_bag_wall=Q0,window_s=180,
            input_sha256={str(p.relative_to(TRIAL)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (GOAL/'events.jsonl',GOAL/'native_samples.jsonl',TRIAL/'fluency_audit/closed_trial_summary.json')},
            counts=dict(planned=len(rows),dispatch_kind=dict(Counter(r['dispatch_kind'] for r in rows)),
                        unique_owners=len(set(r['owner'] for r in rows)),maneuver_done=sum(e['status']=='maneuver_done' for e in states),
                        section_direction_changes=len(direction_changes),native_sent_gear_changes=SUMMARY['sent_gear_switches'],
                        direction_changes_with_native_change=dict(Counter(r['dispatch_kind'] for r in direction_changes if r['native_sent_gear_changes'])),
                        short_sections_under_1m=sum(r['path_length_m']<1 for r in rows)),
            sections=rows,direction_changes=direction_changes,cases=cases,
            limitations=['source stamps relative to bag query receipt; events.elapsed is observer receipt and can lag, unused for ordering',
                         'Native telemetry at ~10Hz in jsonl; bag ~20Hz near-stop summary retained as authoritative speed integration',
                         'Native position deltas are measured within same native origin; absolute coordinates not mixed with ROS axle/odom reference',
                         'path first edge/yaw gives planned D/R; gear sent is downstream command, not proof every planned section moved',
                         '23 planned are section dispatches, not independent ComputePath calls',
                         'frontier_resolved states alone do not prove retirement incorrect; must inspect upstream route_status/retired_exploration input for full cause'])
target=TRIAL/'fluency_audit/segment_chains.json'
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(report['counts'],ensure_ascii=False))
for case in cases:
    print(case['name'],round(case['start_s'],3),round(case['end_s'],3),case['measured'])
