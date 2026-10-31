"""Read only a CLOSED v37 bag; compare actual source commands and whole paths."""
import collections,json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.input_quality import stamp_ns
from wildos_nav2_adapter.motion_path import motion_sections
version=sys.argv[1] if len(sys.argv)>1 else 'v37_full_path'
assert version in ('v37_full_path','v38_rolling_observation','v39_observation_session','v40_observation_heading','v41_general_observation','v42_retirement_boundary','v43_abort_recovery','v44_instance_handoff','v45_approach_contract','v46_approach_feedback','v47_depth_geometry','v48_range_candidate','v49_localization_range','v50_confirmed_continuation','v51_confirmed_action_handoff','v52_approach_local_map','v53_confirmed_graph_destination','v54_confirmed_graph_handoff','v55_frontier_endpoint','v56_confirmed_local_continuity','v57_upstream_target_approach')
root=Path('/home/ly/execution_continuity_review_20261022')/version
r=json.loads((root/'field_result.json').read_text())
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
trial=Path(r['trial']);g=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
events=[json.loads(line) for line in (g/'events.jsonl').read_text().splitlines() if line]
states=[e for e in events if e.get('kind')=='plan_status']
query_rows=rows('/spot1/object_search/query');start=next(t for t,m in query_rows if m.data=='green residential house')
plans={stamp_ns(p):p for t,p in rows('/wildos/planned_path')}
progress=rows('/wildos/controller_path_progress');commands=rows('/wildos/cmd_vel_stamped')
groups=collections.defaultdict(list);source_ages=[]
for t,p in progress:
    if not p.poses:continue
    source_ages.append((t-stamp_ns(p.poses[0]))*1e-9)
    direction=None
    for a,b in zip(p.poses,p.poses[1:]):
        dx=b.pose.position.x-a.pose.position.x;dy=b.pose.position.y-a.pose.position.y
        if math.hypot(dx,dy)<1e-9:continue
        q=a.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        direction=1 if dx*math.cos(yaw)+dy*math.sin(yaw)>0 else -1;break
    if direction:groups[stamp_ns(p)].append((t,direction))
identities=[]
for key,samples in groups.items():
    plan=plans.get(key);changes=[]
    for a,b in zip(samples,samples[1:]):
        if a[1]!=b[1]:changes.append((b[0]-start)*1e-9)
    points=[(p.pose.position.x,p.pose.position.y) for p in plan.poses] if plan else []
    identities.append(dict(path_stamp_ns=key,matched_dispatched_path=plan is not None,
        poses=len(points),geometric_directions=len(motion_sections(points)),
        controller_direction_transitions=changes,progress_messages=len(samples),
        first_elapsed=(samples[0][0]-start)*1e-9,last_elapsed=(samples[-1][0]-start)*1e-9))
violations=[];zero_yaw=[];gaps=[]
for t,c in commands:
    v,w=c.twist.linear.x,c.twist.angular.z
    if abs(w)>abs(v)/8.+1e-8:violations.append(dict(elapsed=(t-start)*1e-9,v=v,w=w))
    if abs(v)<1e-9 and abs(w)>1e-9:zero_yaw.append((t-start)*1e-9)
for (a,_),(b,_) in zip(commands,commands[1:]):
    if b-a>10**9:gaps.append(dict(start=(a-start)*1e-9,end=(b-start)*1e-9,seconds=(b-a)*1e-9))
failures=[{k:e[k] for k in ('elapsed','status','reason','detail','source','goal') if k in e}
          for e in states if any(k in e for k in ('reason','detail'))]
report=dict(trial=str(trial),execution_modes=dict(collections.Counter(e.get('execution_mode') for e in states)),
    statuses=dict(collections.Counter(e.get('status') for e in states)),
    planned_queue_sizes=dict(collections.Counter(e.get('motion_sections_remaining') for e in states if e.get('status')=='planned')),
    path_progress_messages=len(progress),command_messages=len(commands),
    path_identities=identities,internal_direction_transitions=sum(len(p['controller_direction_transitions']) for p in identities),
    curvature_bound_violations=violations,in_place_rotation_commands=zero_yaw,command_gaps_over_1s=gaps,
    progress_receipt_age_sec=dict(min=min(source_ages) if source_ages else None,max=max(source_ages) if source_ages else None),
    transitions_and_failures=failures,
    field_acceptance=False,scope='closed interfaces and actual commands; target success independently assessed')
(root/'continuous_path_audit.json').write_text(json.dumps(report,indent=2))
small={k:v for k,v in report.items() if k not in ('transitions_and_failures','path_identities')}
small['actual_paths']=len(identities)
small['multi_direction_actions']=sum(p['geometric_directions']>1 for p in identities)
print(json.dumps(small,indent=2))
