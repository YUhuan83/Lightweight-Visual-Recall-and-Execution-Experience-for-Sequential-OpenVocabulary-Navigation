import json
from pathlib import Path
import time
from beamngpy import BeamNGpy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo
from graphnav_msgs.msg import NavigationGraph

b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
b.control.resume()
rclpy.init()
n = Node('full_test_stamp_probe')
data = {}
for key, typ, topic in [('odom', Odometry, '/spot1/odom'), ('graph', NavigationGraph, '/spot1/nav_graph'),
                         ('camera', Image, '/spot1/realsense/front/color/image_raw'),
                         ('info', CameraInfo, '/spot1/realsense/front/color/camera_info'),
                         ('map', OccupancyGrid, '/spot1/local_map')]:
    def receive(m, key=key):
        values = data.setdefault(key, [])
        values.append({'stamp': m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                       'arrival': time.time(), 'frame': m.header.frame_id})
    n.create_subscription(typ, topic, receive, 10)
try:
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=.1)
finally:
    b.control.pause()
    b.disconnect()
    n.destroy_node()
    rclpy.shutdown()
out = {k: {'count': len(v), 'first': v[0], 'last': v[-1]} for k, v in data.items()}
if data.get('camera') and data.get('graph'):
    out['closest_image_graph_delta'] = min(abs(a['stamp'] - g['stamp']) for a in data['camera'] for g in data['graph'])
Path('/home/ly/full_town_memory_20260930/stamps.json').write_text(json.dumps({'samples': data, 'summary': out}, indent=2))
print(json.dumps(out, indent=2))
