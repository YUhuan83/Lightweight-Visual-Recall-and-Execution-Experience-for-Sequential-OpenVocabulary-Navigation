"""Read-only audit of closed JSONL/log evidence; no ROS or live SQLite access."""
from pathlib import Path
from decimal import Decimal
import bisect,collections,hashlib,json,math,re,statistics,uuid,zipfile

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
SOURCE=ROOT/'test_artifacts/frontier_resolution_repair_20261016/server_field'
SESSION=SOURCE/'field_results/session_20261016_200923'
GOAL=SESSION/'goal_001'
assert (GOAL/'result.json').exists() and (SESSION/'session_result.json').exists()
assert (SOURCE/'field_scope_baseline_search_20261016_200921/result.json').exists()
manifest=json.loads((ROOT/'deploy/20261016_frontier_resolution_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
for name in ('nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
             'nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
             'nav2_adapter/wildos_nav2_adapter/exploration_cost.py',
             'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner_node.cpp'):
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==manifest['members'][name],name

def rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf8').splitlines() if line]
events=rows(GOAL/'events.jsonl');plans=[x for x in events if x.get('kind')=='plan_status']
native=rows(GOAL/'native_samples.jsonl');times=[x['wall_time'] for x in native]
review=json.loads((SOURCE/'field/closed_review.json').read_text(encoding='utf8'))
stops=review['stationary_intervals_ge5s'];retired=review['retirements']
stack=(SESSION/'stack.log').read_text(encoding='utf8',errors='replace').splitlines()
def stamp(line):
    match=re.search(r'\[(\d{10}\.\d+)\]',line)
    return int(Decimal(match[1])*10**9) if match else None
cancel=[dict(stamp_ns=stamp(line),log=line) for line in stack if 'Cancelling follow_path action' in line]
stopped=[dict(stamp_ns=stamp(line),log=line) for line in stack if 'Goal was canceled. Stopping the robot.' in line]
following=[x for x in plans if x.get('status')=='following']
terminal_names={'maneuver_done','segment_done','follow_failed','recovery_done','recovery_failed','exploration_retired'}
correlations=[]
for c in cancel:
    ts=c['stamp_ns']
    feedback=next((p for p in plans if ts<=p['stamp_ns']<=ts+300000000 and p['status']=='exploration_retired'),None)
    retire=next((r for r in retired if feedback and r['exploration_id']==feedback.get('exploration_id') and 0<=ts-r['stamp_ns']<1e9),None)
    controller=next((x for x in stopped if 0<=x['stamp_ns']-ts<1e9),None)
    accepted=next((x for x in following if x['stamp_ns']>ts),None)
    recovered=next((x for x in plans if x['stamp_ns']>ts and x['status']=='execution_recovered'),None)
    before=next((x for x in reversed(following) if x['stamp_ns']<=ts),None)
    sample=native[max(0,bisect.bisect_right(times,ts/1e9)-1)]
    correlations.append(dict(**c,source_retirement=retire,bridge_retirement=feedback,
        controller_cancel_stop=controller,previous_follow=before,
        speed_at_cancel=sample['speed'],sent_at_cancel=sample['sent'],
        quarantine_release_delay_s=(recovered['stamp_ns']-ts)/1e9 if recovered else None,
        next_follow_delay_s=(accepted['stamp_ns']-ts)/1e9 if accepted else None))

def percentile(values,prob):
    vals=sorted(v for v in values if v is not None and math.isfinite(v))
    return vals[min(len(vals)-1,round((len(vals)-1)*prob))] if vals else None
def stats(values):
    return dict(median=percentile(values,.5),p95=percentile(values,.95),maximum=max(values,default=None))
stop_details=[]
for stop in stops:
    start=stop['start_wall'];end=start+stop['duration_sec']
    subset=[x for x in native if start<=x['wall_time']<=end]
    preceding=next((x for x in reversed(correlations) if 0<=start-x['stamp_ns']/1e9<=3),None)
    changes=[p for p in plans if start-3<=p['stamp_ns']/1e9<=end+1]
    stop_details.append(dict(**stop,nearby_cancel=preceding,
        source_retirement_reason=preceding['source_retirement']['reason'] if preceding and preceding['source_retirement'] else None,
        prior_status=next((p for p in reversed(plans) if p['stamp_ns']/1e9<=start),None),
        status_changes=changes,samples=len(subset),
        full_brake_samples=sum(x.get('sent',{}).get('brake',0)>=.9 for x in subset),
        throttle_samples=sum(x.get('sent',{}).get('throttle',0)>.1 for x in subset),
        max_actual_wheelspeed=max((x.get('actual',{}).get('wheelspeed',0) for x in subset),default=None),
        telemetry_or_actuator_failures=sum(not x.get('telemetry_ok',False) or not x.get('actuator_ok',False) for x in subset)))

# Do not inflate the large execution observer JSONL into memory.
observer_counts=collections.Counter();control_examples=[];control_stats=collections.defaultdict(list)
active_control_stats=collections.defaultdict(list);stop_controls=[];active_control_reasons=collections.Counter()
control_states=[];commands=[]
active_start=min(p['stamp_ns'] for p in plans)
active_end=max(p['stamp_ns'] for p in plans)
with zipfile.ZipFile(ROOT/'test_artifacts/frontier_resolution_repair_20261016/scope_baseline_field_evidence.zip') as archive:
    name='field_results/session_20261016_200923/execution_observer/messages.jsonl'
    if name in archive.namelist():
        with archive.open(name) as stream:
            for line in stream:
                if not any(token in line for token in (b'"topic":"/wildos/control_status"',b'"topic":"/wildos/native_control_status"',b'"topic":"/wildos/cmd_vel"')):continue
                record=json.loads(line);observer_counts[record['topic']]+=1
                if record['topic']=='/wildos/cmd_vel':
                    commands.append(dict(stamp_ns=record['received_ros_ns'],linear_x=record['message']['linear']['x'],angular_z=record['message']['angular']['z']))
                    continue
                payload=json.loads(record['message']['data'])
                if len(control_examples)<2:control_examples.append(record)
                for key,value in payload.items():
                    if isinstance(value,(float,int)) and not isinstance(value,bool) and math.isfinite(value):
                        control_stats[record['topic']+':'+key].append(value)
                ts=record['received_ros_ns']
                if record['topic']=='/wildos/control_status' and active_start<=ts<=active_end:
                    control_states.append((ts/1e9,payload.get('reason')))
                    active_control_reasons[payload.get('reason')]+=1
                    for key in ('state_receipt_age_steady_sec','command_receipt_age_steady_sec','native_receipt_age_steady_sec','native_sample_age_steady_sec'):
                        value=payload.get(key)
                        if isinstance(value,(float,int)) and math.isfinite(value):active_control_stats[key].append(value)
                    for index,stop in enumerate(stops):
                        if stop['start_wall']<=ts/1e9<=stop['start_wall']+stop['duration_sec']:
                            stop_controls.append(dict(index=index,stamp_ns=ts,reason=payload.get('reason'),
                                state_age=payload.get('state_receipt_age_steady_sec'),
                                native_age=payload.get('native_sample_age_steady_sec'),
                                brake=payload.get('brake'),throttle=payload.get('throttle'),
                                native_healthy=payload.get('native_health_ok')))

control_stop_seconds=collections.Counter()
for (at,reason),(until,_) in zip(control_states,control_states[1:]):
    for stop in stops:
        overlap=max(0.,min(until,stop['start_wall']+stop['duration_sec'])-max(at,stop['start_wall']))
        control_stop_seconds[reason]+=overlap
for row in correlations:
    ts=row['stamp_ns'];end=ts+row['next_follow_delay_s']*1e9 if row['next_follow_delay_s'] is not None else ts
    row['commands_after_cancel_before_next_follow']=[c for c in commands if ts<=c['stamp_ns']<=end]

vision=rows(SESSION/'vision_participation.jsonl')
visual_scores=[x for x in vision if x.get('kind')=='scores']
target_review=json.loads((SOURCE/'field/target_progress_review.json').read_text(encoding='utf8'))
selected=[]
for target in target_review['targets']:
    preceding=next((x for x in reversed(visual_scores) if x['wall']<=target['first_received_wall']),None)
    uid=''.join(f'{x:03}' for x in uuid.UUID(target['identity'].split(':')[0]).bytes)
    candidate=next((x for x in preceding['candidates'] if x['id']==uid),None) if preceding else None
    selected.append(dict(identity=target['identity'],first_wall=target['first_received_wall'],goal=target['goal'],
        preceding_visual_age_s=target['first_received_wall']-preceding['wall'] if preceding else None,
        matched_candidate=candidate))

summary=dict(candidate=manifest['sha256'],scope='Closed public logs plus exact current production methods; no new simulator trial or production edits',
    follow_cancel_requests=len(cancel),controller_cancel_stop_logs=len(stopped),
    cancellations_correlated_to_source_retirement=sum(x['source_retirement'] is not None for x in correlations),
    cancel_reasons=dict(collections.Counter(x['source_retirement']['reason'] if x['source_retirement'] else 'unmatched' for x in correlations)),
    cancel_speed_stats=stats([x['speed_at_cancel'] for x in correlations]),
    cancellation_to_quarantine_release_s=stats([x['quarantine_release_delay_s'] for x in correlations if x['quarantine_release_delay_s'] is not None]),
    cancellation_to_next_follow_s=stats([x['next_follow_delay_s'] for x in correlations if x['next_follow_delay_s'] is not None]),
    stop_count=len(stops),stop_total_s=sum(x['duration_sec'] for x in stops),
    stop_trigger_reasons=dict(collections.Counter(x['source_retirement_reason'] or 'no_cancel_within_3s' for x in stop_details)),
    stops_telemetry_actuator_failures=sum(x['telemetry_or_actuator_failures'] for x in stop_details),
    native=dict(samples=len(native),telemetry_failures=sum(not x.get('telemetry_ok',False) for x in native),
        actuator_failures=sum(not x.get('actuator_ok',False) for x in native),
        watchdog_samples=sum(x.get('watchdog_stop',False) for x in native),
        rpc_ms=stats([x['control_rpc_ms'] for x in native]),telemetry_age_s=stats([x['telemetry_age_sec'] for x in native])),
    plan_status_counts=dict(collections.Counter(x['status'] for x in plans)),
    visual_queries=dict(collections.Counter(x.get('query') for x in visual_scores)),
    default_target_counts=dict(collections.Counter(str((x['matched_candidate'] or {}).get('default')) for x in selected)),
    observer_counts=dict(observer_counts),observer_numeric_stats={k:stats(v) for k,v in control_stats.items()},
    active_control_stats={k:stats(v) for k,v in active_control_stats.items()},active_control_reasons=dict(active_control_reasons),
    stop_control_reasons=dict(collections.Counter(x['reason'] for x in stop_controls)),
    stop_control_seconds=dict(control_stop_seconds),
    stop_control_state_age=stats([x['state_age'] for x in stop_controls if x['state_age'] is not None]),
    stop_control_native_age=stats([x['native_age'] for x in stop_controls if x['native_age'] is not None]),
    replan_after_maneuver_reasons=dict(collections.Counter(x.get('reason') for x in plans if x['status']=='replanning_after_maneuver')),
    read_only_candidate_rejections=dict(collections.Counter(x.get('reason') for x in plans if x['status']=='exploration_cost_rejected')),
    compute_requests=sum('ComputePath request sent:' in line for line in stack),
    known_limits=['No proof of every private costmap/request; no causal success-rate inference from two different trajectories',
        'Zero target masks does not prove the visible target was misrecognized',
        'Temporal linkage requires matching identity and source reason; unmatched events stay unmatched'])
OUT.mkdir(exist_ok=True)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
(OUT/'timeline.json').write_text(json.dumps(dict(cancellations=correlations,stops=stop_details,selected_targets=selected,control_examples=control_examples,stop_controls=stop_controls),indent=2)+'\n',encoding='utf8')
brief={k:v for k,v in summary.items() if k!='observer_numeric_stats'}
print(json.dumps(brief,indent=2))
