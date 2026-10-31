"""Real producer callbacks on CLOSED v55 sensors; no simulated actions or vehicle."""
import bisect,json,math,sqlite3,sys,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from beamng_wildos_adapter.goal_standoff import GoalStandoff
base=Path('/home/ly/execution_continuity_review_20261022');root=base/sys.argv[1]
receipt=json.loads((base/'v55_frontier_endpoint/field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,c=topics[name]
    return [(t,deserialize_message(data,c)) for t,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
odoms=rows('/spot1/odom');maps=rows('/global_costmap/costmap');local=rows('/local_costmap/costmap');occupancy=rows('/spot1/local_map');loc=rows('/spot1/object_localization');queries=rows('/spot1/object_search/query');db.close()
def last(seq,now):return next(m for t,m in reversed(seq) if t<=now)
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
odom_by_stamp={ns(m):m for _,m in odoms}
now=queries[0][0];rclpy.init();n=GoalStandoff();p=n.get_parameter
values=dict(standoff=3.,carrier_model='ackermann',vehicle_front_extent=3.41,graph_destination_approach=root.name=='v55_frontier_endpoint')
n.get_parameter=lambda name:S(value=values[name]) if name in values else p(name)
n.observation_minimum_turning_radius=12.;n.confirmation_window_sec=60.
n.get_clock=lambda:S(now=lambda:S(nanoseconds=now,to_msg=lambda:Time(sec=now//10**9,nanosec=now%10**9)))
n.active_query='green residential house';n.query_started_ns=queries[0][0]
n.publisher=n.confirmation_pub=n.visible_approach_pub=Mock()
# Keep separate mocks; a bare Pose must not be counted as a confirmation.
n.publisher=Mock();n.confirmation_pub=Mock();n.visible_approach_pub=Mock()
result=[];seen=0
with patch('time.monotonic',lambda:now*1e-9):
 for now,message in loc:
    n.odom=last(odoms,now);n.occupancy=last(occupancy,now)
    n.occupancy_odom=odom_by_stamp.get(ns(n.occupancy))
    n.on_vehicle_grid(last(maps,now));n.on_vehicle_grid(last(local,now),local=True)
    n.on_localization(message)
    calls=n.confirmation_pub.publish.call_args_list
    for call in calls[seen:]:
        d=json.loads(call.args[0].data)
        if d.get('approach_goal_ready'):
            pos=n.odom.pose.pose.position
            result.append(dict(receipt_ns=now,observation_stamp_ns=d['observation_stamp_ns'],
                current=[pos.x,pos.y],object=d['object_position'],point=d['standoff'],
                clamped=d['standoff_clamped'],intermediate=d.get('standoff_intermediate'),instance=d['instance_id'],
                target_distance_m=math.dist(d['object_position'][:2],(pos.x,pos.y)),
                waypoint_distance_m=math.dist(d['standoff'],(pos.x,pos.y))))
    seen=len(calls)
report=dict(scope=__doc__,version=root.name,recorded_vehicle_unchanged=True,
    confirmed_waypoints=result,bare_pose_publications=n.publisher.publish.call_count,
    confirmation_publications=n.confirmation_pub.publish.call_count)
out=root/'local_goal_replay.json';out.write_text(json.dumps(report,indent=2))
print(json.dumps(dict(version=root.name,waypoints=len(result),bare_poses=report['bare_pose_publications'],clamped=sum(r['clamped'] for r in result),max_waypoint_distance=max((r['waypoint_distance_m'] for r in result),default=None)),indent=2))
n.destroy_node();rclpy.shutdown()
