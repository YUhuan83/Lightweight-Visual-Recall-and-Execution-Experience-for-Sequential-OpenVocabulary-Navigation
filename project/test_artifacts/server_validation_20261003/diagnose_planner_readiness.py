"""Bounded plan-only probe; never enables vehicle execution."""
from pathlib import Path
import json
import time
import sys
import rclpy
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped

ROOT = Path('/home/ly/town_memory_validation_20261003')
rclpy.init()
node = rclpy.create_node('planner_readiness_probe')

def wait(future, seconds):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    return future.result() if future.done() else None

rows = json.loads((ROOT/'restart_snapshot.json').read_text())
print(json.dumps({'launch': [{k: row[k] for k in ('name', 'args')} for row in rows
                 if row['name'] in ('planner', 'controller', 'lifecycle')]}), flush=True)
for target, names in (
    ('/global_costmap/global_costmap', ['plugins', 'obstacle_layer.enabled',
      'inflation_layer.enabled', 'robot_base_frame', 'global_frame', 'update_frequency']),
    ('/local_costmap/local_costmap', ['plugins', 'obstacle_layer.enabled',
      'inflation_layer.enabled', 'robot_base_frame', 'global_frame']),
    ('/planner_server', ['planner_plugins', 'CarLike.max_planning_time',
      'CarLike.minimum_turning_radius', 'use_sim_time']),
):
    client = node.create_client(GetParameters, target+'/get_parameters')
    if not client.wait_for_service(timeout_sec=2):
        print(json.dumps({'params_unavailable': target}), flush=True)
        continue
    req = GetParameters.Request(names=names)
    result = wait(client.call_async(req), 3)
    print(json.dumps({'node': target, 'parameters': dict(zip(names, [str(v) for v in result.values]))
                      if result else 'timeout'}), flush=True)

if '--clear' in sys.argv:
    clear = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    assert clear.wait_for_service(timeout_sec=3), 'No clear service'
    assert wait(clear.call_async(ClearEntireCostmap.Request()), 3) is not None, 'Clear timed out'
    print(json.dumps({'costmap_cleared': True}), flush=True)
    time.sleep(1)

client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
assert client.wait_for_server(timeout_sec=3), 'No planner action server'
for planner in ('GridBased', 'CarLike'):
    request = ComputePathToPose.Goal()
    request.planner_id = planner
    request.use_start = True
    for name, xy in [('start', (736.37, -12.65)), ('goal', (741.37, -12.65))]:
        pose = PoseStamped()
        pose.header.frame_id = 'spot1/odom'
        pose.pose.position.x, pose.pose.position.y = xy
        pose.pose.orientation.w = 1.0
        setattr(request, name, pose)
    started = time.monotonic()
    handle = wait(client.send_goal_async(request), 3)
    if handle is None or not handle.accepted:
        print(json.dumps({'planner': planner, 'accepted': False}), flush=True)
        continue
    result_future = handle.get_result_async()
    result = wait(result_future, 6)
    print(json.dumps({'planner': planner, 'elapsed': time.monotonic()-started,
       'result_status': result.status if result else None,
       'poses': len(result.result.path.poses) if result else None}), flush=True)
    if result is None:
        wait(handle.cancel_goal_async(), 2)
        late = wait(result_future, 2)
        print(json.dumps({'planner': planner, 'after_cancel_status': late.status if late else None}), flush=True)

print(json.dumps({'planner_log_tail': (ROOT/'planner.log').read_text()[-8500:]}), flush=True)
node.destroy_node()
rclpy.shutdown()
