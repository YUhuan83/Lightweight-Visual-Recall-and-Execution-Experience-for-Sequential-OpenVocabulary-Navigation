"""Closed v34 observations, actual same-frame body gap and controller failure."""
import bisect,collections,json,math,sqlite3,tarfile,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import vehicle_body_distance
base=Path('/home/ly/execution_continuity_review_20261022')
version=sys.argv[1] if len(sys.argv)>1 else 'v34_vehicle_observation'
assert version in ('v34_vehicle_observation','v35_confirmed_vehicle','v36_weak_observation','v37_full_path','v38_rolling_observation','v39_observation_session','v40_observation_heading','v41_general_observation','v42_retirement_boundary','v43_abort_recovery','v44_instance_handoff','v45_approach_contract','v46_approach_feedback','v47_depth_geometry','v48_range_candidate','v49_localization_range','v50_confirmed_continuation','v51_confirmed_action_handoff','v52_approach_local_map','v53_confirmed_graph_destination','v54_confirmed_graph_handoff','v55_frontier_endpoint','v56_confirmed_local_continuity','v57_upstream_target_approach')
root=base/version
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
trial=Path(r['trial']);goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
confirmations=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
odom=rows('/spot1/odom');times=[t for t,_ in odom]
gaps=[]
for t,d in confirmations:
    i=bisect.bisect_right(times,t)-1
    if i<0 or not d.get('search_confirmed'):continue
    m=odom[i][1];p=m.pose.pose;q=p.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    target=d['object_position'];gap=vehicle_body_distance((p.position.x,p.position.y,p.position.z),yaw,target,'ackermann')
    gaps.append(dict(stamp_ns=t,body_gap_m=gap,odom_age_s=(t-times[i])*1e-9,
        object=target,standoff=d.get('standoff'),ready=d.get('approach_goal_ready'),clamped=d.get('standoff_clamped'),failure=d.get('approach_failure_reason'),observation_age_s=(t-d['observation_stamp_ns'])*1e-9))
loc=[json.loads(m.data) for _,m in rows('/spot1/object_localization')]
report=dict(trial=str(trial),masks=len(rows('/spot1/object_mask')),valid_localizations=sum(x.get('localization_valid') is True for x in loc),sources=dict(collections.Counter(x.get('source') for x in loc if x.get('localization_valid') is True)),confirmed=len(gaps),approach_ready=sum(bool(x['ready']) for x in gaps),min_body_gap_m=min((x['body_gap_m'] for x in gaps),default=None),confirmations=gaps,
    image_topics=[n for n in topics if any(s in n for s in ('image','rgb','camera'))])
db.close()
stack=(goal.parent/'stack.log').read_text()
report['controller_abort_lines']=[l for l in stack.splitlines() if any(s in l for s in ('bounded command arc','Failed to make progress','Controller patience','FollowPath aborted','Control loop missed'))]
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
report['geometry_refreshes']=sum(bool(e.get('geometry_refreshed')) for e in events)
report['vehicle_preflight_radius']=[]
def radii(value,path=''):
    if isinstance(value,dict):
        for k,v in value.items():
            if k=='observation_minimum_turning_radius':report['vehicle_preflight_radius'].append(dict(path=path,value=v))
            radii(v,path+'/'+k)
    elif isinstance(value,list):
        for i,v in enumerate(value):radii(v,path+'/'+str(i))
radii(json.loads((goal.parent/'server_preflight.json').read_text()))
(root/'closed_arrival_audit.json').write_text(json.dumps(report,indent=2))
archive=root/(trial.name+'_core.tar.gz')
with tarfile.open(archive,'w:gz') as tar:
    for p in trial.rglob('*'):
        if p.is_file() and 'interfaces' not in p.parts and p.suffix in ('.json','.jsonl','.log','.jpg','.png'):
            tar.add(p,arcname=p.relative_to(trial))
print(json.dumps({k:v for k,v in report.items() if k!='confirmations'},indent=2))
print('LAST_CONFIRMATIONS',json.dumps(gaps[-4:],indent=2))
print('ARCHIVE',archive,archive.stat().st_size)
