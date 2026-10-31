"""Collect actual DDS inputs and controller state without enabling motion."""
from pathlib import Path
import json
import time
import os
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image
from object_search_msgs.msg import ObjectMaskWithTf
from graphnav_msgs.msg import NavigationGraph
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters

ROOT = Path('/home/ly/town_memory_validation_20261003')
rclpy.init()
n = rclpy.create_node('town_memory_20261003_health')
got = {}
counts = {}
def receive(topic, message):
    counts[topic] = counts.get(topic, 0)+1
    got[topic] = message

for kind, topic in (
    (Odometry, '/spot1/odom'),
    (OccupancyGrid, '/spot1/local_map'),
    (OccupancyGrid, '/global_costmap/costmap'),
    (Image, '/spot1/realsense/front/color/image_raw'),
    (Image, '/spot1/model_visualization'),
    (Image, '/wildos/viz/frontier_traversability'),
    (ObjectMaskWithTf, '/spot1/object_mask'),
    (String, '/spot1/object_localization'),
    (String, '/spot1/object_confirmation'),
    (NavigationGraph, '/spot1/nav_graph'),
    (NavigationGraph, '/spot1/scored_nav_graph'),
    (String, '/wildos/native_control_status'),
    (String, '/wildos/plan_status'),
):
    n.create_subscription(kind, topic, lambda m, t=topic: receive(t, m), qos_profile_sensor_data)
end = time.monotonic()+15
while time.monotonic()<end:
    rclpy.spin_once(n, timeout_sec=.05)
now = n.get_clock().now().nanoseconds
result = {'topics': {}, 'lifecycle': {}, 'parameters': {}}
for topic, m in got.items():
    info = {'received': counts[topic]}
    if hasattr(m, 'header'):
        stamp = m.header.stamp.sec*10**9 + m.header.stamp.nanosec
        info.update(frame=m.header.frame_id, stamp_ns=stamp, age_sec=(now-stamp)*1e-9)
    if isinstance(m, Odometry):
        p=m.pose.pose.position
        info['position']=[p.x,p.y,p.z]
    elif isinstance(m, OccupancyGrid):
        info.update(width=m.info.width,height=m.info.height,occupied=sum(v>=99 for v in m.data))
    elif isinstance(m, Image):
        info.update(width=m.width,height=m.height)
    elif isinstance(m, NavigationGraph):
        info.update(nodes=len(m.nodes),edges=len(m.edges))
    elif isinstance(m, ObjectMaskWithTf):
        info.update(query=m.query,positive_pixels=sum(m.object_mask.data))
    elif isinstance(m, String):
        try: info['value']=json.loads(m.data)
        except ValueError: info['value']=m.data
    result['topics'][topic]=info
for remote in ('/planner_server','/controller_server'):
    c=n.create_client(GetState,remote+'/get_state')
    if c.wait_for_service(timeout_sec=2):
        f=c.call_async(GetState.Request())
        rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if f.done() and f.result():result['lifecycle'][remote]=f.result().current_state.label
for remote,names in (
    ('/wildos_graph_path_bridge',['follow_path']),
    ('/wildos_cmd_vel_to_control',['enable','max_speed','overspeed_limit','minimum_turning_radius']),
    ('/controller_server',['odom_topic','FollowPath.desired_linear_vel']),
):
    c=n.create_client(GetParameters,remote+'/get_parameters')
    if c.wait_for_service(timeout_sec=2):
        f=c.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if f.done() and f.result():
            result['parameters'][remote]={k:(v.bool_value if v.type==1 else v.double_value if v.type==3 else v.string_value)
                                           for k,v in zip(names,f.result().values)}
(ROOT/'health_probe.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2),flush=True)
n.destroy_node()
rclpy.shutdown()
