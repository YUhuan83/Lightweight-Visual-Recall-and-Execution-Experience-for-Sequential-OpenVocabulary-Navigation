"""Read sensor and odometry updates with the kinematic actuator disabled."""
import json
import os
from pathlib import Path
import sys
import time

env = json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text())
os.environ.update(env)
if os.environ.get('WILDOS_SENSOR_CHECK_ENV_READY') != '1':
    os.environ['WILDOS_SENSOR_CHECK_ENV_READY'] = '1'
    os.execve(sys.executable,[sys.executable,__file__],dict(os.environ))
for path in env['PYTHONPATH'].split(':'):
    sys.path.insert(0,path)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from beamngpy import BeamNGpy

rclpy.init()
node = Node('fixed_physics_sensor_check')
results = dict(rgb_frames=0,lidar_frames=0,odom_frames=0,native_frames=0)
def rgb(msg):
    results['rgb_frames'] += 1
    results['rgb_size'] = [msg.width,msg.height,len(msg.data)]
    results['rgb_sample_nonzero'] = any(msg.data[::max(1,len(msg.data)//1000)])
def lidar(msg):
    results['lidar_frames'] += 1
    results['lidar_points'] = msg.width*msg.height
def odom(msg):
    results['odom_frames'] += 1
    results['odom_position'] = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
def native(msg):
    results['native_frames'] += 1
    data=json.loads(msg.data)
    results['native_backend'] = data.get('backend')
    results['native_speed'] = data.get('speed')
node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',rgb,1)
node.create_subscription(PointCloud2,'/spot1/ouster/front/points_filtered',lidar,1)
node.create_subscription(Odometry,'/spot1/odom',odom,1)
node.create_subscription(String,'/wildos/native_control_status',native,1)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
try:
    b.control.resume()
    end=time.monotonic()+5
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    results['passed'] = (results['rgb_frames']>1 and results.get('rgb_sample_nonzero',False)
        and results['lidar_frames']>1 and results.get('lidar_points',0)>0
        and results['odom_frames']>1 and results.get('native_backend')=='kinematic')
    Path('/home/ly/town_memory_validation_20261003/physics_isolation_20261006/sensor_check.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results))
finally:
    b.control.pause()
    b.disconnect()
    node.destroy_node()
    rclpy.shutdown()
