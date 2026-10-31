"""Recorded Smac paths through real bridge checks; action transport is isolated."""
import copy,json,math,sqlite3,time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath,OccupancyGrid,Odometry
from rclpy.serialization import deserialize_message
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge,xy,yaw
from wildos_nav2_adapter.input_quality import stamp_ns
from wildos_nav2_adapter.motion_path import motion_sections

root=Path('/home/ly/execution_continuity_review_20261022/fixed_heading_replay_20261024')
manifest=json.loads((root/'manifest.json').read_text());case=manifest[3];trial=Path(case['trial'])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def read(topic,kind):
    tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    return [deserialize_message(b,kind) for b, in db.execute('select data from messages where topic_id=? order by timestamp',(tid,))]
inputs=case['request']['replay_inputs']
grid=next(m for m in read('/global_costmap/costmap',OccupancyGrid) if stamp_ns(m)==inputs['global_costmap_stamp_ns'])
local=next(m for m in read('/local_costmap/costmap',OccupancyGrid) if stamp_ns(m)==inputs['local_costmap_stamp_ns'])
odom=next(m for m in read('/spot1/odom',Odometry) if stamp_ns(m)==inputs['odom_stamp_ns'])
route=next(m for m in read('/spot1/graphnav_planner/path',RosPath) if stamp_ns(m)==inputs['route_stamp_ns'])
db.close()
def path(variant):
    out=RosPath();out.header.frame_id='spot1/odom';out.header.stamp=odom.header.stamp
    for line in Path(case['fixture']+f'.free_0.variant_{variant}.path').read_text().splitlines():
        x,y,a=map(float,line.split());p=PoseStamped();p.header=out.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);out.poses.append(p)
    return out
output=[];rclpy.init()
try:
    for repeat in range(2):
        node=GraphPathBridge()
        try:
            node.carrier_model='ackermann';node.minimum_turning_radius=8.;node.planner_id='CarLike';node.follow_enabled=True;node.require_route_status=False
            node.odom=copy.deepcopy(odom);node.grid=copy.deepcopy(grid);node.local_grid=copy.deepcopy(local)
            node.route=node.latest_route=copy.deepcopy(route);node.route_complete=True;node.route_endpoint=xy(route.poses[-1].pose)
            node.visible_approach_active=True;node.expected=tuple(case['goals'][0][:2]);node.candidates=[(*case['goals'][0],case['distance'])];node.candidate_index=0
            now=time.monotonic();node.odom_received_at=node.grid_received_at=node.local_grid_received_at=now
            node.execution_transport_issue=Mock(return_value=None);node.plan_client=Mock();node.follow_client=Mock();node.follow_client.send_goal_async.return_value=Future()
            node.path_pub=Mock();node.full_path_pub=Mock();node.status_pub=Mock()
            baseline=path(0);curved=path(1)
            checks=[]
            for p in (baseline,curved):
                points=[xy(s.pose) for s in p.poses];heads=[yaw(s.pose) for s in p.poses]
                checks.append(dict(sections=len(motion_sections(points)),whole=node.validate_path(points,heads,[xy(odom.pose.pose),node.expected],grid,node.expected,start=xy(odom.pose.pose)),initial_tracking=node.initial_tracking_issue(points,heads),local=node.validate_local_path(points,heads,local,xy(odom.pose.pose),yaw(odom.pose.pose))))
            f=Future();f.set_result(S(status=4,result=S(path=baseline)));node.active_plan_serial=42;node.pending=True
            node.on_plan_result(f,node.epoch,0,42)
            retained=node.normal_best is not None;index=node.candidate_index
            f=Future();f.set_result(S(status=4,result=S(path=curved)));node.active_plan_serial=43;node.pending=True
            node.on_plan_result(f,node.epoch,index,43)
            sent=node.follow_client.send_goal_async.call_count
            output.append(dict(repeat=repeat,checks=checks,original_retained=retained,probe_index=index,follow_dispatches=sent,
                dispatched_sections=len(node.motion_queue),status=node.last_status))
            assert retained and index==1 and sent==1 and len(node.motion_queue)==1,output[-1]
        finally:node.destroy_node()
finally:rclpy.shutdown()
(root/'bridge_contract_replay.json').write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
