import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as Route
from sensor_msgs.msg import Image
from beamngpy import BeamNGpy

rclpy.init()
n = Node('spawn_memory_validation')
pub = n.create_publisher(String, '/spot1/object_search/query', 10)
state = {}; events = []; last_status = None
def receive(key,msg):
    global last_status
    state[key] = msg
    if key == 'status':
        value = json.loads(msg.data)
        if value == last_status:
            return
        last_status = value
        events.append({'t':time.monotonic(),'status':value})
        print(json.dumps(events[-1]),flush=True)
    if key == 'memory':
        p = msg.pose.position
        events.append({'t':time.monotonic(),'memory':[p.x,p.y]})
        print(json.dumps(events[-1]),flush=True)
for key,typ,topic,qos in [('odom',Odometry,'/spot1/odom',qos_profile_sensor_data),('memory',PoseStamped,'/spot1/remembered_object_goal',10),('status',String,'/wildos/plan_status',10),('planned',Route,'/wildos/planned_path',10),('camera',Image,'/spot1/realsense/front/color/image_raw',1)]:
    n.create_subscription(typ,topic,lambda m,k=key:receive(k,m),qos)
def wait(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.1)
wait(2)
pub.publish(String(data='trash can'))
wait(5)
scenario=json.loads(Path('/home/ly/wildos-sim/east-coast-test-20260927/town.json').read_text())
v=scenario['vehicles'][0]
bng=BeamNGpy('127.0.0.1',25252)
bng.open(launch=False)
try:
    current=bng.scenario.get_current()
    if str(current.level) != scenario['level']:
        raise RuntimeError('Unexpected map: '+str(current.level))
    print('Same-map vehicle reset:',bng.vehicles.teleport('ego',tuple(v['pos']),tuple(v['rot']),reset=True),flush=True)
finally:
    bng.disconnect()
wait(8)
pub.publish(String(data='tree'))
events.append({'t':time.monotonic(),'query':'tree'})
wait(30)
report={'events':events,'wildos_pid_retained':Path('/proc/1575639').exists()}
if 'odom' in state:
    p=state['odom'].pose.pose; q=p.orientation
    report['pose']=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
if 'planned' in state:
    report['planned_poses']=len(state['planned'].poses)
out=Path('/home/ly/navigation_validation_20260930_logs')
if 'camera' in state:
    m=state['camera']; a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    cv2.imwrite(str(out/'town_spawn.png'),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
out.joinpath('spawn_memory_test.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
