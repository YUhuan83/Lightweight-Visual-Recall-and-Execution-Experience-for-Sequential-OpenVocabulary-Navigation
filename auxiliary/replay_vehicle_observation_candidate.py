"""Actual v34 producer callback on closed v33 request/map/evidence snapshots."""
import json,math,sqlite3,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.goal_standoff import GoalStandoff
base=Path('/home/ly/execution_continuity_review_20261022')
old=base/'v33_integrated_progress'
version=sys.argv[1] if len(sys.argv)>1 else 'v34_vehicle_observation'
assert version in ('v34_vehicle_observation','v35_confirmed_vehicle')
root=base/version
receipt=json.loads((old/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(name):
    idx,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(idx,))]
odom=rows('/spot1/odom');maps=rows('/global_costmap/costmap');terrain=rows('/spot1/local_map');hints=rows('/spot1/visible_target_approach')
def last(seq,t):return next((m for stamp,m in reversed(seq) if stamp<=t),None)
out=root/'recorded_observation_probe';out.mkdir(exist_ok=True)
report=[]
rclpy.init()
for case in json.loads((old/'observation_pose_probe/manifest.json').read_text()):
    t=case['stamp_ns'];source=json.loads(last(hints,t).data)
    n=GoalStandoff()
    try:
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=t))
        n.odom=deepcopy(last(odom,t));p=n.odom.pose.pose
        p.position.x,p.position.y=case['start'][:2];p.orientation.x=p.orientation.y=0.
        p.orientation.z=math.sin(case['start'][2]/2);p.orientation.w=math.cos(case['start'][2]/2)
        n.occupancy=last(terrain,t);n.occupancy_odom=next(m for _,m in odom if m.header.stamp==n.occupancy.header.stamp)
        n.vehicle_grid=last(maps,t);n.vehicle_connectivity=None
        n.observation_minimum_turning_radius=12.
        n.active_query=source['query'];n.query_started_ns=0
        # Existing producer already established its mask continuity for this
        # source message. Reuse that evidence, retaining its original lifetime.
        n.visible_target=dict(stamp=source['observation_stamp_ns'],bearing=source['bearing'])
        n.visible_target_count=3;n.visible_approach_pub=Mock()
        n.publish_visible_step()
        calls=n.visible_approach_pub.publish.call_args_list
        proposal=json.loads(calls[-1].args[0].data) if calls else None
        name=Path(case['file']).name
        lines=(old/'observation_pose_probe'/name).read_text().splitlines()
        header=lines[0].split();old_count=int(header[-1])
        goals=[case['goals'][0]]
        if proposal:goals.append([*proposal['point'],proposal['navigation_heading']])
        header[-1]=str(len(goals));target=out/name
        target.write_text(' '.join(header)+'\n'+''.join(' '.join(map(str,g))+'\n' for g in goals)+'\n'.join(lines[old_count+1:])+'\n')
        report.append(dict(file=str(target),request_serial=case['request_serial'],source_age_s=(t-source['observation_stamp_ns'])*1e-9,original_source=source,new_proposal=proposal,goals=goals))
    finally:n.destroy_node()
rclpy.shutdown();db.close()
(out/'manifest.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
