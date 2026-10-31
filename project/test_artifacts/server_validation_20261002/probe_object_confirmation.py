"""Inspect raw object estimates and confirmation spread without changing nodes."""
import json
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,*sys.argv[1:],'--runtime-child'],env=runtime_env()).returncode)
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
rclpy.init();n=Node('object_confirmation_probe');values=[];counts={}
def cb(key,msg):
    counts[key]=counts.get(key,0)+1
    if key=='raw':values.append([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
for key,kind,topic in [('raw',PoseStamped,'/spot1/raw_object_waypoint'),('confirmed',PoseStamped,'/spot1/imgnav_waypoint'),('odom',Odometry,'/spot1/odom')]:
    n.create_subscription(kind,topic,lambda msg,key=key:cb(key,msg),qos_profile_sensor_data)
end=time.monotonic()+20
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
radii=[]
for i in range(4,len(values)):
    pts=np.array(values[i-4:i+1])[:,:2];radii.append(float(np.max(np.linalg.norm(pts-np.median(pts,axis=0),axis=1))))
report={'counts':counts,'raw_positions':values,'five_sample_radius_m':radii,
    'confirmation_subscribers':[{'name':p.node_name,'reliability':str(p.qos_profile.reliability)} for p in n.get_subscriptions_info_by_topic('/spot1/raw_object_waypoint')]}
(ROOT/sys.argv[1]/'object_confirmation_probe.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
n.destroy_node();rclpy.shutdown()
