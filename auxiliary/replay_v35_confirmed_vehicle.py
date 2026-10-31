"""Execute the actual v35 producer on the closed v34 confirmed snapshots."""
import json,math,sqlite3,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.goal_standoff import GoalStandoff
base=Path('/home/ly/execution_continuity_review_20261022');evidence=base/'v34_vehicle_observation';root=base/'v35_confirmed_vehicle'
r=json.loads((evidence/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
baseline={p['stamp_ns']:p for p in json.loads((evidence/'confirmed_step_replay_1.json').read_text())['rows']}
goal=next(Path(r['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(typ)) for i,n,typ in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
conf=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
odom=rows('/spot1/odom');terrain=rows('/spot1/local_map');maps=rows('/global_costmap/costmap');rawmaps=rows('/global_costmap/costmap_raw')
def last(seq,t):return next((m for stamp,m in reversed(seq) if stamp<=t),None)
def heading(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
out=root/'confirmed_step_probe';out.mkdir(exist_ok=True)
rclpy.init();report=[];native=[]
for t,d in conf:
    if t not in baseline:continue
    n=GoalStandoff()
    try:
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=t,to_msg=lambda:Time(sec=t//10**9,nanosec=t%10**9)))
        original=n.get_parameter;n.get_parameter=lambda key:S(value=3.) if key=='standoff' else original(key)
        n.observation_minimum_turning_radius=12.;n.active_query=d['query'];n.query_started_ns=0
        n.odom=last(odom,t);n.occupancy=last(terrain,t);n.vehicle_grid=last(maps,t)
        n.occupancy_odom=next(m for _,m in odom if m.header.stamp==n.occupancy.header.stamp)
        n.publisher=Mock();n.confirmation_pub=Mock()
        target=PoseStamped();target.header.frame_id='spot1/odom'
        target.pose.position.x,target.pose.position.y,target.pose.position.z=d['object_position']
        n.pending_confirmation=(target,deepcopy(d));n.retry_confirmation(force=True)
        message=json.loads(n.confirmation_pub.publish.call_args.args[0].data)
        assert message['observation_stamp_ns']==d['observation_stamp_ns']
        current=message['standoff'];yaw=message.get('standoff_navigation_heading')
        if current is not None and yaw is None:
            yaw=math.atan2(d['object_position'][1]-current[1],d['object_position'][0]-current[0])
        p=n.odom.pose.pose;start=(p.position.x,p.position.y,heading(p.orientation))
        report.append(dict(stamp_ns=t,before=baseline[t]['old_filter_point'],after=current,heading=yaw,
                           source_stamp_ns=message['observation_stamp_ns'],ready=message['approach_goal_ready']))
        if baseline[t]['old_filter_point'] is None and current is not None and len(native)<3 and not any(math.dist(start[:2],x['start'][:2])<.5 for x in native):
            raw=last(rawmaps,t);i=raw.metadata;file=out/f'case_{len(native)}.txt'
            with file.open('w') as f:
                f.write(' '.join(map(str,[i.size_x,i.size_y,i.resolution,i.origin.position.x,i.origin.position.y,*start,1]))+'\n')
                f.write(' '.join(map(str,[*current,yaw]))+'\n')
                for idx,cost in enumerate(raw.data):
                    if cost:f.write(f'{idx} {cost}\n')
            native.append(dict(file=str(file),start=start,goal=[*current,yaw]))
    finally:n.destroy_node()
rclpy.shutdown();db.close()
summary=dict(scope='Actual new producer, independent controlled closed snapshots. Baseline rejection/previous-goal history not reconstructed. Public raw map is not private locked field map. Not vehicle acceptance.',snapshots=len(report),before_ready=sum(x['before'] is not None for x in report),after_ready=sum(x['ready'] for x in report),native=native,rows=report)
(root/(sys.argv[1] if len(sys.argv)>1 else 'confirmed_replay_1.json')).write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
