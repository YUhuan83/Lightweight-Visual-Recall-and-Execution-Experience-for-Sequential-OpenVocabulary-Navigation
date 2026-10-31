"""Bounded live coordinate checks; keeps map loaded and leaves actuator disabled."""
import os
import json
from pathlib import Path
import sys
env=json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text())
os.environ.update(env)
if os.environ.get('COORD_PROBE_ENV')!='1':
    os.environ['COORD_PROBE_ENV']='1'
    os.execve(sys.executable,[sys.executable,*sys.argv],dict(os.environ))
import math
import time
import hashlib
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import ChangeState, GetState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from beamngpy import BeamNGpy

out=Path('/home/ly/coordinate_probe_20261007');out.mkdir(exist_ok=True)
rclpy.init();node=Node('coordinate_probe_20261007')
samples=[];latest={};counts={'rgb':0,'lidar':0}
def odom(msg):
    q=msg.pose.pose.orientation;p=msg.pose.pose.position
    data={'t':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
          'p':[p.x,p.y,p.z],'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
          'v':[msg.twist.twist.linear.x,msg.twist.twist.linear.y], 'w':msg.twist.twist.angular.z}
    samples.append(data);latest['odom']=data
node.create_subscription(Odometry,'/spot1/odom',odom,qos_profile_sensor_data)
node.create_subscription(String,'/wildos/native_control_status',lambda m:latest.update(native=json.loads(m.data)),10)
node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',lambda m:counts.update(rgb=counts['rgb']+1),1)
node.create_subscription(PointCloud2,'/spot1/ouster/front/points_filtered',lambda m:counts.update(lidar=counts['lidar']+1),1)
pub=node.create_publisher(Twist,'/wildos/cmd_vel',1)
def spin(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.025)
def wait_future(future,timeout=8):
    until=time.monotonic()+timeout
    while not future.done():
        if time.monotonic()>until:raise TimeoutError('ROS service/action response timeout')
        rclpy.spin_once(node,timeout_sec=.05)
    return future.result()
def param(name,key,value):
    c=node.create_client(SetParameters,name+'/set_parameters')
    if not c.wait_for_service(timeout_sec=5):raise RuntimeError('missing parameter service '+name)
    result=wait_future(c.call_async(SetParameters.Request(parameters=[Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value))])))
    node.destroy_client(c)
    assert all(r.successful for r in result.results),str(result)
def transition(id):
    c=node.create_client(ChangeState,'/controller_server/change_state')
    assert c.wait_for_service(timeout_sec=5)
    request=ChangeState.Request();request.transition.id=id
    result=wait_future(c.call_async(request));node.destroy_client(c)
    assert result.success,str(result)
def controller_state():
    c=node.create_client(GetState,'/controller_server/get_state');assert c.wait_for_service(timeout_sec=5)
    result=wait_future(c.call_async(GetState.Request()));node.destroy_client(c)
    return result.current_state.id
def zero():
    for _ in range(5):pub.publish(Twist());spin(.05)
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def command(v,w,duration):
    zero();start=len(samples);before=dict(latest['odom']);deadline=time.monotonic()+duration
    while time.monotonic()<deadline:
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pub.publish(msg);spin(.05)
    zero();after=dict(latest['odom']);rows=samples[start:]
    delta=np.array(after['p'])-before['p'];heading=before['yaw']
    turn=sum(wrap(b['yaw']-a['yaw']) for a,b in zip([before]+rows,rows))
    return dict(command=[v,w],duration_sec=duration,source_duration_sec=after['stamp']-before['stamp'],
                yaw_change_rad=turn,net_xy_m=float(np.linalg.norm(delta[:2])),
                forward_displacement_m=float(delta[0]*math.cos(heading)+delta[1]*math.sin(heading)),
                lateral_displacement_m=float(-delta[0]*math.sin(heading)+delta[1]*math.cos(heading)),
                peak_xy_drift_m=max([float(np.linalg.norm(np.array(r['p'][:2])-before['p'][:2])) for r in rows],default=0),
                before=before,after=after,samples=len(rows))

b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
def lua(code):return b.control.queue_lua_command(code,response=True)
report={'map_reload_count':0,'mode':sys.argv[1] if len(sys.argv)>1 else 'preflight'}
goal_handle=None;changed_controller=False
try:
    spin(3)
    report['live_native']=latest.get('native');report['counts']=dict(counts)
    try:report['engine_before']=json.loads(lua("return jsonEncode(extensions.wildosKinematic.status())"))
    except Exception as exc:
        report['engine_before_error']=repr(exc)
        if report['mode']!='reload':raise
    report['node_names']=node.get_node_names_and_namespaces()
    report['cmd_publishers']=[dict(node_name=x.node_name,node_namespace=x.node_namespace) for x in node.get_publishers_info_by_topic('/wildos/cmd_vel')]
    if report['mode']=='preflight':
        print(json.dumps(report,indent=2));sys.exit(0)
    param('/wildos_graph_path_bridge','follow_path',False)
    param('/wildos_kinematic_vehicle_control','enable',False)
    if controller_state()==3:transition(4);changed_controller=True
    b.control.pause()
    if report['mode']=='reload':
        source=Path('/home/ly/wildosKinematic_20261007.lua').read_text()
        assert 'world-z-passive-v2' in source
        report['lua_sha256']=hashlib.sha256(source.encode()).hexdigest()
        lua('local source='+json.dumps(source)+"; assert(loadstring(source)); FS:directoryCreate('lua/ge/extensions'); writeFile('lua/ge/extensions/wildosKinematic.lua',source); extensions.unload('wildosKinematic'); extensions.load('wildosKinematic'); return true")
        lua("local v=scenetree.findObject('ego'); assert(v); extensions.wildosKinematic.start(v:getID(),-2.172,0.330,'ideal_point',0); return true")
    report['engine_loaded']=json.loads(lua("return jsonEncode(extensions.wildosKinematic.status())"))
    assert report['engine_loaded']['rotationConvention']=='world-z-passive-v2','Old Lua remains loaded'
    b.vehicles.teleport('ego',(734.294,-11.946,52.154),(0.,0.,0.7658087977066911,0.6430683364581344),reset=False)
    b.control.resume();spin(5)
    assert latest.get('odom') and counts['rgb']>0 and counts['lidar']>0,'Missing sensor/odom stream'
    param('/wildos_kinematic_vehicle_control','enable',True);spin(1)
    report['probes']={}
    report['probes']['straight']=command(1.2,0,3)
    print('STRAIGHT',json.dumps(report['probes']['straight']),flush=True)
    report['probes']['left']=command(0,.3,3)
    print('LEFT',json.dumps(report['probes']['left']),flush=True)
    report['probes']['right']=command(0,-.3,3)
    print('RIGHT',json.dumps(report['probes']['right']),flush=True)
    report['probes']['reverse_restore']=command(-1.2,0,3)
    straight=report['probes']['straight'];left=report['probes']['left'];right=report['probes']['right']
    report['coordinate_passed']=(straight['forward_displacement_m']>2 and abs(straight['lateral_displacement_m'])<.15
        and left['yaw_change_rad']>.5 and right['yaw_change_rad']<-.5
        and left['peak_xy_drift_m']<.15 and right['peak_xy_drift_m']<.15)
    assert report['coordinate_passed'],'Coordinate motion check failed; skip FollowPath'
    state=controller_state()
    if state==1:transition(1)
    if controller_state()==2:transition(3)
    assert controller_state()==3,'Controller is not active'
    changed_controller=False;zero();spin(1)
    action=ActionClient(node,FollowPath,'/follow_path');assert action.wait_for_server(timeout_sec=5)
    origin=dict(latest['odom']);path=NavPath();path.header.frame_id='spot1/odom';path.header.stamp=node.get_clock().now().to_msg()
    # A smooth, fixed 12-m route containing a mild lateral change tests turning.
    yaw=origin['yaw'];local=[]
    for i in range(61):
        x=i*.2;y=1.5*(1-math.cos(math.pi*x/12))/2
        theta=math.atan2(1.5*math.pi/24*math.sin(math.pi*x/12),1.)
        pose=PoseStamped();pose.header=path.header
        pose.pose.position.x=origin['p'][0]+x*math.cos(yaw)-y*math.sin(yaw)
        pose.pose.position.y=origin['p'][1]+x*math.sin(yaw)+y*math.cos(yaw)
        pose.pose.orientation.z=math.sin((yaw+theta)/2);pose.pose.orientation.w=math.cos((yaw+theta)/2)
        path.poses.append(pose);local.append((pose.pose.position.x,pose.pose.position.y))
    request=FollowPath.Goal();request.path=path;request.controller_id='FollowPath';request.goal_checker_id='maneuver_goal_checker'
    feedback=[]
    goal_handle=wait_future(action.send_goal_async(request,feedback_callback=lambda f:feedback.append(dict(t=time.monotonic(),distance=f.feedback.distance_to_goal,speed=f.feedback.speed))))
    assert goal_handle.accepted,'FollowPath rejected fixed route'
    future=goal_handle.get_result_async();start=len(samples);until=time.monotonic()+35
    while not future.done() and time.monotonic()<until:spin(.1)
    if not future.done():wait_future(goal_handle.cancel_goal_async());spin(1)
    result=future.result() if future.done() else None
    rows=samples[start:];pts=np.asarray(local)
    def path_error(p):
        a=pts[:-1];d=pts[1:]-a;t=np.clip(np.sum((np.array(p)-a)*d,axis=1)/np.sum(d*d,axis=1),0,1)
        return float(np.min(np.linalg.norm(np.array(p)-(a+t[:,None]*d),axis=1)))
    report['fixed_path']=dict(status=result.status if result else None,result=str(result.result) if result else 'timeout',
        endpoint_error_m=float(np.linalg.norm(np.array(latest['odom']['p'][:2])-pts[-1])),
        max_cross_track_m=max([path_error(r['p'][:2]) for r in rows],default=0),samples=len(rows),feedback=feedback,
        path=local)
    report['fixed_path_passed']=bool(result and result.status==4 and report['fixed_path']['endpoint_error_m']<.8)
    report['counts']=dict(counts);report['live_native_after']=latest.get('native')
except BaseException as exc:
    if not isinstance(exc,SystemExit):report['error']=repr(exc);print('PROBE_ERROR',repr(exc),flush=True)
    if isinstance(exc,SystemExit):raise
finally:
    if report['mode']!='preflight':
        if goal_handle and not goal_handle.get_result_async().done():
            try:wait_future(goal_handle.cancel_goal_async(),3)
            except Exception:pass
        try:zero();param('/wildos_kinematic_vehicle_control','enable',False)
        except Exception as exc:report['stop_error']=repr(exc)
        try:lua('extensions.wildosKinematic.command(0,0); return true');b.control.pause()
        except Exception as exc:report['pause_error']=repr(exc)
        if changed_controller:
            try:
                if controller_state()==2:transition(3)
            except Exception as exc:report['restore_controller_error']=repr(exc)
    (out/(report['mode']+'_report.json')).write_text(json.dumps(report,indent=2))
    (out/(report['mode']+'_odom.json')).write_text(json.dumps(samples))
    b.disconnect();node.destroy_node();rclpy.shutdown()
if report['mode']!='preflight':print(json.dumps({k:v for k,v in report.items() if k not in ('node_names','live_native','live_native_after')},indent=2))
