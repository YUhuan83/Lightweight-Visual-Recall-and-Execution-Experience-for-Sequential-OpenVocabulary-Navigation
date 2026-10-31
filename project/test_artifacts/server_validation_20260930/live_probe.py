import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from beamng_msgs.msg import VehicleControl

rclpy.init()
node = Node('navigation_validation_probe')
state = {}
events = []
def receive(key, msg):
    state[key] = msg
    if key == 'status':
        try:
            value = json.loads(msg.data)
        except ValueError:
            value = msg.data
        if not events or events[-1]['data'] != value:
            events.append({'t': time.time(), 'data': value})
for key, cls, topic, qos in [
    ('odom', Odometry, '/spot1/odom', qos_profile_sensor_data),
    ('grid', OccupancyGrid, '/global_costmap/costmap', 10),
    ('graph_route', Route, '/spot1/graphnav_planner/path', 10),
    ('planned_route', Route, '/wildos/planned_path', 10),
    ('status', String, '/wildos/plan_status', 10),
    ('control', VehicleControl, '/control', 10),
    ('cmd_vel', Twist, '/wildos/cmd_vel', 10),
    ('memory', PoseStamped, '/spot1/remembered_object_goal', 10),
    ('camera', Image, '/spot1/realsense/front/color/image_raw', 1),
]:
    node.create_subscription(cls, topic, lambda msg, k=key: receive(k, msg), qos)
params = {}
for remote, names in [('/planner_server', ['CarLike.minimum_turning_radius', 'CarLike.motion_model_for_search']),
                      ('/controller_server', ['FollowPath.desired_linear_vel', 'FollowPath.use_rotate_to_heading']),
                      ('/wildos_cmd_vel_to_control', ['enable', 'max_speed', 'max_steering']),
                      ('/wildos_graph_path_bridge', ['follow_path'])]:
    client = node.create_client(GetParameters, remote + '/get_parameters')
    if client.wait_for_service(timeout_sec=2):
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if future.done():
            params[remote] = dict(zip(names, [rclpy.parameter.parameter_value_to_python(v) for v in future.result().values]))
end = time.monotonic() + 12
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.2)
report = {'parameters': params, 'topics_received': list(state), 'status_events': events,
          'clear_goal_service_available': node.create_client(Trigger, '/spot1/graphnav_planner/clear_goal').wait_for_service(timeout_sec=2)}
if 'odom' in state:
    o = state['odom']; p = o.pose.pose; q = p.orientation
    report['pose'] = [p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    report['speed'] = math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y)
    report['odom_age_s'] = time.time() - o.header.stamp.sec - o.header.stamp.nanosec*1e-9
for k in ('graph_route', 'planned_route'):
    if k in state:
        p = state[k]
        report[k] = {'poses': len(p.poses), 'frame': p.header.frame_id, 'stamp': p.header.stamp.sec + p.header.stamp.nanosec*1e-9,
                     'endpoint': [p.poses[-1].pose.position.x,p.poses[-1].pose.position.y] if p.poses else None}
if 'control' in state:
    c = state['control']; report['control'] = {'throttle':c.throttle,'brake':c.brake,'steering':c.steering,'gear':c.gear}
if 'grid' in state:
    g = state['grid']; report['costmap_age_s'] = time.time() - g.header.stamp.sec - g.header.stamp.nanosec*1e-9
out = Path('/home/ly/navigation_validation_20260930_logs')
if 'camera' in state:
    m = state['camera']; a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    cv2.imwrite(str(out/'town_current.png'), cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
out.joinpath('live_probe.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2), flush=True)
node.destroy_node()
rclpy.shutdown()
