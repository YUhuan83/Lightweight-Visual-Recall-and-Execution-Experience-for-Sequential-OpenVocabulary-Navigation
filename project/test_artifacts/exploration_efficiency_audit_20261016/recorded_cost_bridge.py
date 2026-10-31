"""Production comparison on actual Smac recorded-map outputs; no actuation."""
from pathlib import Path
import csv,json,math,time
from copy import deepcopy
from unittest.mock import Mock
import rclpy
from nav_msgs.msg import Path as Route,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.input_quality import stamp_ns
from test_graph_path_bridge_routing import configure_ackermann

r=Path('/home/ly/frontier_cost_repair_20261016');old=Path('/home/ly/exploration_efficiency_audit_20261016')
source=json.loads((old/'frontier_alternatives/candidates.json').read_text())
case=json.loads((old/'cases.json').read_text())['cases'][0]
rows=source['candidates']
for row in rows:
 a=next(v for v in row['assessments'] if v['profile']=='default')
 row['benefit']=row['semantic_utility']*(.35+.65*a['confidence']*a['physical'])
rows=sorted(rows,key=lambda v:v['benefit']-.004*v['distance_m'],reverse=True)[:4]
rclpy.init();n=configure_ackermann(GraphPathBridge());report={}
try:
 n.follow_enabled=True
 fields=(old/'input0.txt').read_text().split()
 width,height=map(int,fields[:2]);res,ox,oy=map(float,fields[2:5])
 grid=OccupancyGrid();grid.header.frame_id='spot1/odom';grid.header.stamp=n.get_clock().now().to_msg()
 grid.info.width=width;grid.info.height=height;grid.info.resolution=res
 grid.info.origin.position.x=ox;grid.info.origin.position.y=oy;grid.info.origin.orientation.w=1.
 grid.data=list(map(int,fields[11:]));assert len(grid.data)==width*height
 from nav_msgs.msg import Odometry
 odom=Odometry();odom.header=deepcopy(grid.header)
 odom.pose.pose.position.x,odom.pose.pose.position.y=case['start'][:2]
 odom.pose.pose.orientation.z=math.sin(case['start'][2]/2);odom.pose.pose.orientation.w=math.cos(case['start'][2]/2)
 n.odom=odom;n.grid=grid;n.local_grid=deepcopy(grid)
 for name in ('odom_received_at','grid_received_at','local_grid_received_at','grid_arrived_at','native_status_at'):setattr(n,name,time.monotonic())
 route=Route();route.header=deepcopy(grid.header)
 for x,y,h in (case['start'],case['goal']):
  p=PoseStamped();p.header=deepcopy(route.header);p.pose.position.x=x;p.pose.position.y=y
  p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);route.poses.append(p)
 n.route=n.latest_route=route;n.route_endpoint=tuple(case['goal'][:2])
 n.route_metadata[stamp_ns(route)]=dict(exploration_id='recorded:1',complete=True,goal=case['goal'][:2],
  observation_heading=case['goal'][2],vehicle_options=[dict(uuid=row['uuid'],goal=row['goal'],benefit=row['benefit']) for row in rows])
 n.plan_client=Mock();n.plan_client.send_goal_async.return_value=__import__('concurrent.futures').futures.Future()
 n.execute_checked_path=Mock();n.frontier_choice_pub=Mock();n.status_pub=Mock()
 n.start_segment()
 assert n.frontier_comparison is not None,'Recorded offers did not enter comparison'
 for row in rows:
  assert n.frontier_comparison is not None
  path=Route();path.header=deepcopy(route.header)
  with (old/f'frontier_alternatives/case{row["index"]}_recorded_smooth.csv').open() as stream:
   for v in csv.DictReader(stream):
    p=PoseStamped();p.header=deepcopy(path.header);p.pose.position.x=float(v['x']);p.pose.position.y=float(v['y']);h=float(v['yaw'])
    p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);path.poses.append(p)
  n.grid_arrived_at=time.monotonic();n.native_status_at=time.monotonic()
  n.record_frontier_cost(path)
 choice=n.frontier_choice_pub.publish.call_args.args[0]
 chosen=next(row for row in rows if row['uuid']==choice.chosen_uuid)
 report=dict(scope='Actual recorded-map Smac CSVs, production bridge validators/cost comparator, synthetic current stamps; no DDS planner/action or vehicle execution',
  offered_indices=[v['index'] for v in rows],chosen_index=chosen['index'],vehicle_cost_m=choice.vehicle_cost,
  events=[json.loads(c.args[0].data) for c in n.status_pub.publish.call_args_list],
  execution_calls=n.execute_checked_path.call_count)
 assert n.execute_checked_path.call_count==0
finally:
 n.destroy_node();rclpy.shutdown()
(r/'recorded_comparison.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2))
