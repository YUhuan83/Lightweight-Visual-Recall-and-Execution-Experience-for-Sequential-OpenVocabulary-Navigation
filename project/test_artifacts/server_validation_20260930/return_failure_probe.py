"""Read-only planner probe at the position where the live return failed."""
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.graph_route import footprint_blockers, cell_cost, validate_vehicle_path
from wildos_nav2_adapter.memory_candidates import heading_options

rclpy.init()
n = Node('return_failure_probe')
s = {}
n.create_subscription(Odometry, '/spot1/odom', lambda m: s.update(odom=m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: s.update(grid=m), 1)
deadline = time.monotonic()+10
while len(s)<2 and time.monotonic()<deadline:
    rclpy.spin_once(n, timeout_sec=.1)
o, g = s['odom'], s['grid']
start, heading = xy(o.pose.pose), yaw(o.pose.pose)
station = (734.2914672748029, -11.945563448285796)
out = Path('/home/ly/full_town_memory_20260930')
label = sys.argv[1] if len(sys.argv)>1 else 'dubins'
report = {'label':label, 'start':[*start, heading], 'start_cost':cell_cost(g,*start),
          'start_footprint_blockers':footprint_blockers(g,start,heading),
          'station_cost':cell_cost(g,*station), 'plans':[]}
client = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
assert client.wait_for_server(timeout_sec=5)
targets = [('station', station, h) for h in heading_options(start,station,2.9675,heading)]
for d in (3., -3.):
    targets.append(('forward' if d>0 else 'backward',
                   (start[0]+d*math.cos(heading),start[1]+d*math.sin(heading)),heading))
for name, target, h in targets:
    for planner in ('CarLike','GridBased') if name=='station' else ('CarLike',):
        req = ComputePathToPose.Goal()
        req.planner_id = planner
        req.goal.header.frame_id = g.header.frame_id
        req.goal.header.stamp = n.get_clock().now().to_msg()
        req.goal.pose.position.x, req.goal.pose.position.y = target
        req.goal.pose.orientation.z,req.goal.pose.orientation.w = math.sin(h/2),math.cos(h/2)
        f = client.send_goal_async(req)
        rclpy.spin_until_future_complete(n,f,timeout_sec=10)
        handle = f.result()
        f = handle.get_result_async()
        rclpy.spin_until_future_complete(n,f,timeout_sec=10)
        result = f.result()
        p = result.result.path
        points,heads = [xy(q.pose) for q in p.poses],[yaw(q.pose) for q in p.poses]
        record = {'name':name,'planner':planner,'target':[*target,h], 'target_cost':cell_cost(g,*target),
                  'status':result.status,'poses':len(points), 'length':sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                  'validation':validate_vehicle_path(points,heads,[start,station],g,target,start=start,
                                target_heading=h if planner=='CarLike' else None,
                                heading_quantization_bins=72 if planner=='CarLike' else 0),
                  'points':points,'headings':heads}
        report['plans'].append(record)
        print(json.dumps({k:v for k,v in record.items() if k not in ('points','headings')}),flush=True)
np.savez_compressed(out/('return_costmap_'+label+'.npz'),data=np.array(g.data).reshape(g.info.height,g.info.width),
                    origin=[g.info.origin.position.x,g.info.origin.position.y],resolution=g.info.resolution)
out.joinpath('return_probe_'+label+'.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='plans'}),flush=True)
n.destroy_node()
rclpy.shutdown()
