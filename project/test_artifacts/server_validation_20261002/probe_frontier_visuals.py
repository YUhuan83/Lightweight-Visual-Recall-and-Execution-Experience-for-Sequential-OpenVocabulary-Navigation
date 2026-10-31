"""Read current visualization streams and capture their RViz rendering."""
import json
import subprocess
import sys
import time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, *sys.argv[1:], '--runtime-child'], env=runtime_env()).returncode)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
rclpy.init(); node = Node('frontier_visual_probe'); counts = {}; details = {}; ages = []
def receive(key, msg):
    counts[key] = counts.get(key, 0) + 1
    if key == 'model': details[key] = [msg.width, msg.height]
    elif key == 'native': ages.append(json.loads(msg.data)['command_age'])
    else: details[key] = len(msg.markers)
for key, kind, topic in [('model',Image,'/spot1/model_visualization'),
    ('frontiers',MarkerArray,'/spot1/within_range_geofrontiers'), ('rings',MarkerArray,'/spot1/score_rings'),
    ('graph',MarkerArray,'/spot1/nav_graph_viz'), ('scores',MarkerArray,'/wildos/viz/frontier_scores'),
    ('native',String,'/wildos/native_control_status')]:
    node.create_subscription(kind, topic, lambda m,key=key:receive(key,m), qos_profile_sensor_data)
until=time.monotonic()+15
while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
report={'counts':counts,'latest':details,'native_max_command_age':max(ages, default=None)}
out=ROOT/sys.argv[1]
(out/'visual_probe.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab','-video_size','1280x1024','-i',':99','-frames:v','1','-threads','1',str(out/'rviz_probe.png')], check=True,timeout=15)
node.destroy_node();rclpy.shutdown()
