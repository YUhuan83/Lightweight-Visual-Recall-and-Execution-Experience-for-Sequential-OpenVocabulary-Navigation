import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from beamng_msgs.msg import VehicleControl
from wildos_nav2_adapter.graph_route import validate_vehicle_path, distance
from wildos_nav2_adapter.graph_path_bridge import xy,yaw

rclpy.init();n=Node('bounded_follow_path_validation');s={};events=[];samples=[]
logs=Path('/home/ly/navigation_validation_20260930_logs')
last_status=None
def receive(key,msg):
    global last_status
    s[key]=msg;s[key+'_received']=time.monotonic()
    if key=='status':
        v=json.loads(msg.data)
        if v!=last_status:
            events.append({'t':time.monotonic(),'data':v});last_status=v
            print(json.dumps(events[-1]),flush=True)
for key,cls,topic,qos in [('odom',Odometry,'/spot1/odom',qos_profile_sensor_data),('grid',OccupancyGrid,'/global_costmap/costmap',10),('path',Route,'/wildos/planned_path',10),('status',String,'/wildos/plan_status',10),('cmd',Twist,'/wildos/cmd_vel',10),('control',VehicleControl,'/control',10)]:
    n.create_subscription(cls,topic,lambda m,k=key:receive(k,m),qos)
pub=n.create_publisher(String,'/spot1/object_search/query',10)
def wait(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
params=n.create_client(SetParameters,'/wildos_graph_path_bridge/set_parameters')
def follow(enabled):
    if not params.wait_for_service(timeout_sec=3):raise RuntimeError('bridge param service missing')
    p=Parameter(name='follow_path',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=enabled))
    f=params.call_async(SetParameters.Request(parameters=[p]));rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done() or not all(v.successful for v in f.result().results):raise RuntimeError('follow mode update failed')
def launch(argv,name):
    with logs.joinpath(name+'.log').open('ab') as f:
        p=subprocess.Popen(argv,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    logs.joinpath(name+'.pid').write_text(str(p.pid))
    return p
wait(2);pub.publish(String(data='trash can'));wait(4);pub.publish(String(data='tree'))
deadline=time.monotonic()+45
while time.monotonic()<deadline and (last_status is None or last_status.get('status')!='planned'):
    rclpy.spin_once(n,timeout_sec=.1)
if last_status is None or last_status.get('status')!='planned':raise RuntimeError('No collision-checked memory path; no motion')
p=s['path'];points=[xy(v.pose) for v in p.poses];headings=[yaw(v.pose) for v in p.poses]
reason=validate_vehicle_path(points,headings,points,s['grid'],points[-1],start=xy(s['odom'].pose.pose))
if reason:raise RuntimeError('Path changed before execution: '+reason)
hold_pid=int(logs.joinpath('control_hold.pid').read_text())
argv=Path('/proc',str(hold_pid),'cmdline').read_bytes().decode().split('\0')[:-1]
if not any(a.endswith('/cmd_vel_to_control') for a in argv) or 'enable:=false' not in argv:raise RuntimeError('Unexpected hold process')
start=xy(s['odom'].pose.pose);actuator=None;stop_reason='timeout';started=time.monotonic()
try:
    os.kill(hold_pid,signal.SIGINT);time.sleep(.3)
    actuator=launch([a.replace('enable:=false','enable:=true') for a in argv],'control_drive_test')
    wait(.5);follow(True)
    deadline=time.monotonic()+14
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.05)
        now=time.monotonic()
        if now-s.get('odom_received',0)>1:stop_reason='stale_odom';break
        o=s['odom'];position=xy(o.pose.pose);speed=math.hypot(o.twist.twist.linear.x,o.twist.twist.linear.y)
        c=s.get('control');cmd=s.get('cmd')
        samples.append({'t':now-started,'xy':position,'speed':speed,'yaw':yaw(o.pose.pose),
                        'cmd_v':cmd.linear.x if cmd else None,'cmd_w':cmd.angular.z if cmd else None,
                        'throttle':c.throttle if c else None,'brake':c.brake if c else None,'steering':c.steering if c else None})
        if distance(position,start)>=3:stop_reason='3m_limit';break
        if speed>2:stop_reason='overspeed';break
        if last_status.get('status') in ('follow_failed','start_footprint_blocked','stale_odom','stale_costmap'):
            stop_reason=last_status['status'];break
finally:
    try:follow(False)
    except Exception as exc:print('follow disable:',str(exc),flush=True)
    if actuator is not None:
        actuator.send_signal(signal.SIGINT)
        try:actuator.wait(timeout=5)
        except subprocess.TimeoutExpired:actuator.terminate()
    launch(argv,'control_hold')
    wait(3)
report={'stop_reason':stop_reason,'initial_xy':start,'final_xy':xy(s['odom'].pose.pose),'displacement':distance(start,xy(s['odom'].pose.pose)),
        'max_speed':max((v['speed'] for v in samples),default=0),'final_speed':math.hypot(s['odom'].twist.twist.linear.x,s['odom'].twist.twist.linear.y),
        'events':events,'samples':samples,'wildos_pid_retained':Path('/proc/1575639').exists()}
logs.joinpath('drive_test.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('events','samples')},indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
