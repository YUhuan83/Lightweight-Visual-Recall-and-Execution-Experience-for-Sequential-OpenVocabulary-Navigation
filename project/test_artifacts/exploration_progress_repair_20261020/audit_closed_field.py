"""Audit completed field decisions before designing a task-progress guard.

Read-only closed SQLite; public topic timing association is not proof of a
private planning-map version. No simulation, publishing or production changes.
"""
from pathlib import Path
import bisect
import collections
import hashlib
import json
import math
import sqlite3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parent
DB=Path('/home/ly/smoothing_boundary_repair_20261018/field_native_20261018_164222/sessions/session_20261018_164222/goal_001/interfaces/interfaces_0.db3')
connection=sqlite3.connect('file:'+str(DB)+'?mode=ro',uri=True)
topics={name:ident for ident,name in connection.execute('select id,name from topics')}
series={}
for topic in ('/wildos/plan_status','/spot1/graphnav_planner/route_status','/wildos/execution_feedback'):
    values=[]
    for timestamp,data in connection.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(topics[topic],)):
        try:values.append((timestamp,json.loads(deserialize_message(data,String).data)))
        except (TypeError,ValueError):continue
    series[topic]=values
odometry=[]
for timestamp,data in connection.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(topics['/spot1/odom'],)):
    msg=deserialize_message(data,Odometry);p=msg.pose.pose.position;q=msg.pose.pose.orientation
    odometry.append((timestamp,(p.x,p.y),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
connection.close()
route_rows=series['/spot1/graphnav_planner/route_status'];route_times=[t for t,_ in route_rows]
odom_times=[t for t,_,_ in odometry]
plans=series['/wildos/plan_status'];start=min(plans[0][0],odometry[0][0])
samples=[]
for timestamp,data in plans:
    if data.get('status') not in ('route_done','segment_done','maneuver_done','exploration_finishing_segment',
                                 'graph_reset_exhausted','route_recovery_exhausted'):continue
    index=bisect.bisect_right(route_times,timestamp)-1
    route=route_rows[index] if index>=0 else None
    oi=bisect.bisect_right(odom_times,timestamp)-1
    pose=odometry[oi] if oi>=0 else None
    identity=data.get('exploration_id')
    associated=(route is not None and route[1].get('exploration_id')==identity and identity is not None
                and timestamp-route[0]<5_000_000_000)
    samples.append(dict(time_s=(timestamp-start)*1e-9,status=data.get('status'),source=data.get('source'),
        query_generation=data.get('query_generation'),exploration_id=identity,
        goal=data.get('goal'),exploration_goal=data.get('exploration_goal'),
        requires_visual_confirmation=data.get('requires_visual_confirmation'),
        pose=list(pose[1]) if pose else None,yaw=pose[2] if pose else None,
        nearest_prior_route=route[1] if associated else None))
groups=[]
for row in samples:
    if row['status']!='route_done':continue
    key=(row['query_generation'],row['exploration_id'],tuple(row['goal'] or []),row['requires_visual_confirmation'])
    if groups and groups[-1]['key']==key and row['time_s']-groups[-1]['last_s']<3.:
        groups[-1]['last_s']=row['time_s'];groups[-1]['count']+=1
    else:groups.append(dict(key=key,first_s=row['time_s'],last_s=row['time_s'],count=1,
                           sample=row))
report=dict(scope=__doc__,db=str(DB),db_sha256=hashlib.sha256(DB.read_bytes()).hexdigest(),
    status_counts=dict(collections.Counter(v.get('status') for _,v in plans)),
    route_complete_counts=dict(collections.Counter(str(v.get('complete')) for _,v in route_rows)),
    execution_counts=dict(collections.Counter((v.get('reason') or 'none') for _,v in series['/wildos/execution_feedback'])),
    samples=samples,route_done_runs=groups,
    complete_navigation_passed=False)
(ROOT/'closed_field_progress.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('samples','scope')},indent=2))
