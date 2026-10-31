"""Closed v34: controlled confirmed-target snapshots, no vehicle execution."""
import json,math,sqlite3,sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from beamng_wildos_adapter.visible_approach import vehicle_observation_step
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v34_vehicle_observation'
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(typ)) for i,n,typ in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
conf=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
odom=rows('/spot1/odom');terrain=rows('/spot1/local_map');maps=rows('/global_costmap/costmap');rawmaps=rows('/global_costmap/costmap_raw')
def last(seq,t):return next((m for stamp,m in reversed(seq) if stamp<=t),None)
out=root/'confirmed_step_probe';out.mkdir(exist_ok=True)
rclpy.init();report=[];native=[]
for t,data in conf:
    if not data.get('search_confirmed'):continue
    n=GoalStandoff()
    try:
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=t,to_msg=lambda:Time(sec=t//10**9,nanosec=t%10**9)))
        original=n.get_parameter
        n.get_parameter=lambda key:S(value=3.) if key=='standoff' else original(key)
        n.observation_minimum_turning_radius=12.;n.active_query=data['query'];n.query_started_ns=0
        n.odom=last(odom,t);n.occupancy=last(terrain,t);n.vehicle_grid=last(maps,t)
        n.occupancy_odom=next(m for _,m in odom if m.header.stamp==n.occupancy.header.stamp)
        n.publisher=Mock();target=PoseStamped();target.header.frame_id='spot1/odom'
        target.pose.position.x,target.pose.position.y,target.pose.position.z=data['object_position']
        old_failure=n.publish_standoff(target);old=n.last_goal
        p=n.odom.pose.pose;mp=n.occupancy_odom.pose.pose
        def heading(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        start=(p.position.x,p.position.y,heading(p.orientation))
        bearing=math.atan2(target.pose.position.y-p.position.y,target.pose.position.x-p.position.x)
        info=n.occupancy.info;grid=np.asarray(n.occupancy.data).reshape(info.height,info.width)
        accept=n.observation_pose_filter(target=data['object_position'][:2])
        clearance=n.approach_distance()[0]
        def advancing(point,yaw):
            gap=math.dist(point,data['object_position'][:2])
            return clearance-.05<=gap<math.dist(start[:2],data['object_position'][:2])-.6 and accept(point,yaw)
        fresh=(n.fresh(n.odom,1.) and n.fresh(n.occupancy,1.) and n.fresh(n.vehicle_grid,3.)
               and math.dist(start[:2],(mp.position.x,mp.position.y))<=1.)
        selected=vehicle_observation_step(grid,float(info.resolution),(info.origin.position.x,info.origin.position.y),
            (mp.position.x,mp.position.y),heading(mp.orientation),bearing,start[:2],start[2],12.,
            accept=advancing,sweep_clear=n.observation_pose_filter(bearing=bearing,include_rejected=False)) if fresh else None
        report.append(dict(stamp_ns=t,start=start,object=data['object_position'],original_message_ready=data.get('approach_goal_ready'),old_filter_failure=old_failure,old_filter_point=old,new_proposal=selected,fresh_map_pair=fresh))
        if old is None and selected and len(native)<3 and not any(math.dist(start[:2],x['start'][:2])<.5 for x in native):
            raw=last(rawmaps,t);i=raw.metadata;point,yaw=selected
            file=out/f'case_{len(native)}.txt'
            with file.open('w') as f:
                f.write(' '.join(map(str,[i.size_x,i.size_y,i.resolution,i.origin.position.x,i.origin.position.y,*start,1]))+'\n')
                f.write(' '.join(map(str,[*point,yaw]))+'\n')
                for idx,cost in enumerate(raw.data):
                    if cost:f.write(f'{idx} {cost}\n')
            native.append(dict(file=str(file),start=start,goal=[*point,yaw]))
            if len(native)==1:
                def odom_meta(m):
                    p=m.pose.pose;q=p.orientation
                    return dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                        position=[p.position.x,p.position.y,p.position.z],orientation=[q.x,q.y,q.z,q.w])
                def grid_meta(m):
                    i=m.info;p=i.origin.position;q=i.origin.orientation
                    return dict(frame=m.header.frame_id,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                        width=i.width,height=i.height,resolution=i.resolution,
                        origin=[p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                meta=dict(now_ns=t,confirmation=data,odom=odom_meta(n.odom),map_odom=odom_meta(n.occupancy_odom),
                    local_map=grid_meta(n.occupancy),vehicle_map=grid_meta(n.vehicle_grid))
                np.savez_compressed(out/'confirmed_vehicle_snapshot.npz',meta=json.dumps(meta),
                    local_map=np.asarray(n.occupancy.data,dtype=np.int16),vehicle_map=np.asarray(n.vehicle_grid.data,dtype=np.int16))
    finally:n.destroy_node()
rclpy.shutdown();db.close()
summary=dict(scope='Independent controlled producer snapshots; rejection/last_goal history not reconstructed. Real source times and paired poses/maps preserved. Public raw map is not private field lock snapshot. No runtime patch.',snapshots=len(report),old_filter_failures=sum(x['old_filter_point'] is None for x in report),new_arc_candidates=sum(x['new_proposal'] is not None for x in report),rescued_map_candidates=sum(x['old_filter_point'] is None and x['new_proposal'] is not None for x in report),native=native,rows=report)
(root/(sys.argv[1] if len(sys.argv)>1 else 'confirmed_step_replay_1.json')).write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
