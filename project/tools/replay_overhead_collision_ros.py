"""Replay one recorded collision through real Nav2, in an isolated ROS domain.

No simulator socket, actuator or native control topic is used. Odometry stays
fixed; a successful first velocity command is cancelled immediately.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
from nav_msgs.msg import Odometry, Path as Route
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from wildos_nav2_adapter.pointcloud import split_costmap_points
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET

ROOT=Path('/home/ly/town_memory_validation_20261003/collision_source_fix')
assert os.environ['ROS_DOMAIN_ID']=='49', 'Isolated domain required'
fixture=np.load(ROOT/'cloud_0.npz')
audit=json.loads((ROOT/'audit_before.json').read_text())[0]
recorded_plan=audit['controller_plan']['points']
rotation=fixture['rotation'];origin=fixture['position']
heading=math.atan2(rotation[1,0],rotation[0,0])
opts=dict(min_height=.1,filter_footprint=True,front_extent=3.4,rear_extent=1.05,
    half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
controller=None
rclpy.init();node=Node('isolated_overhead_collision_replay')
tfpub=node.create_publisher(TFMessage,'/tf',20)
opub=node.create_publisher(Odometry,'/spot1/odom',10)
mpub=node.create_publisher(PointCloud2,'/wildos/points',10)
cpub=node.create_publisher(PointCloud2,'/wildos/clearing_points',10)
commands=[]
node.create_subscription(Twist,'/wildos/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),10)

def make_cloud(points,stamp):
    cloud=PointCloud2();cloud.header.frame_id='spot1/odom';cloud.header.stamp=stamp
    cloud.height=1;cloud.width=len(points);cloud.point_step=12;cloud.row_step=len(points)*12
    cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    cloud.data=np.asarray(points,dtype='<f4').tobytes();cloud.is_dense=True
    return cloud

def publish():
    now=node.get_clock().now().to_msg()
    transforms=[]
    for child,position,y in [('spot1/nav_base_link',[origin[0],origin[1],0.],heading),
        ('spot1/ouster/front/os_lidar',origin+rotation@(-np.asarray(SBR_BASE_OFFSET)),heading)]:
        t=TransformStamped();t.header.frame_id='spot1/odom';t.header.stamp=now;t.child_frame_id=child
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,position)
        t.transform.rotation.z=math.sin(y/2);t.transform.rotation.w=math.cos(y/2);transforms.append(t)
    tfpub.publish(TFMessage(transforms=transforms))
    odom=Odometry();odom.header.frame_id='spot1/odom';odom.header.stamp=now;odom.child_frame_id='spot1/nav_base_link'
    odom.pose.pose.position.x,odom.pose.pose.position.y=map(float,origin[:2])
    odom.pose.pose.orientation.z=math.sin(heading/2);odom.pose.pose.orientation.w=math.cos(heading/2)
    opub.publish(odom)
    mpub.publish(make_cloud(mark_world,now));cpub.publish(make_cloud(clear_world,now))

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)

def wait(future,seconds=8):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    if not future.done():raise TimeoutError('ROS future')
    return future.result()

def service(typ,name,request):
    client=node.create_client(typ,name)
    if not client.wait_for_service(timeout_sec=8):raise TimeoutError(name)
    return wait(client.call_async(request))

def path():
    p=Route();p.header.frame_id='spot1/odom';p.header.stamp=node.get_clock().now().to_msg()
    for x,y,h in recorded_plan:
        pose=PoseStamped();pose.header=p.header
        pose.pose.position.x=float(origin[0]+math.cos(heading)*x-math.sin(heading)*y)
        pose.pose.position.y=float(origin[1]+math.sin(heading)*x+math.cos(heading)*y)
        pose.pose.orientation.z=math.sin((h+heading)/2);pose.pose.orientation.w=math.cos((h+heading)/2)
        p.poses.append(pose)
    return p

results=[]
try:
    log=(ROOT/'isolated_controller.log').open('w')
    controller=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args',
        '--params-file','/home/ly/beamng_ros2_0394_ws/install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/nav2_official_control.yaml',
        '-r','cmd_vel:=/wildos/cmd_vel'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    marks,clearing=split_costmap_points(fixture['points'],max_height=3.,**opts)
    mark_world=marks@rotation.T+origin;clear_world=clearing@rotation.T+origin
    timer=node.create_timer(.1,publish)
    for transition in (1,3):
        req=ChangeState.Request();req.transition.id=transition
        response=service(ChangeState,'/controller_server/change_state',req)
        if not response.success:raise RuntimeError('Lifecycle transition failed')
    spin(2.)
    action=ActionClient(node,FollowPath,'/follow_path')
    if not action.wait_for_server(timeout_sec=8):raise TimeoutError('FollowPath')
    for height in (3.,1.5):
        marks,clearing=split_costmap_points(fixture['points'],max_height=height,**opts)
        mark_world=marks@rotation.T+origin;clear_world=clearing@rotation.T+origin
        spin(.5)
        service(ClearEntireCostmap,'/local_costmap/clear_entirely_local_costmap',ClearEntireCostmap.Request())
        spin(1.5)
        grid=service(GetCostmap,'/local_costmap/get_costmap',GetCostmap.Request()).map
        info=grid.metadata
        mx=math.floor((727.1-info.origin.position.x)/info.resolution)
        my=math.floor((-35.9-info.origin.position.y)/info.resolution)
        commands.clear()
        goal=FollowPath.Goal();goal.path=path();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        handle=wait(action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('FollowPath refused')
        result=handle.get_result_async()
        deadline=time.monotonic()+2.
        while not result.done() and not any(abs(c[0])>.01 for c in commands) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        row=dict(max_height=height,markings=len(marks),reference_cell_raw_cost=int(grid.data[my*info.size_x+mx]),
                 first_commands=list(commands),action_status=result.result().status if result.done() else None)
        if not result.done():wait(handle.cancel_goal_async());wait(result)
        results.append(row);print(json.dumps(row),flush=True)
        spin(.3)
    passed=results[0]['action_status']==6 and any(abs(c[0])>.01 for c in results[1]['first_commands'])
    report=dict(scope='Actual Nav2 local costmap and RPP; fixed recorded odometry in isolated domain49; no BeamNG motion',
        passed=passed,cases=results)
    (ROOT/'isolated_replay.json').write_text(json.dumps(report,indent=2))
    if not passed:raise RuntimeError('Recorded collision repair not reproduced')
finally:
    if controller and controller.poll() is None:
        controller.send_signal(signal.SIGINT)
        try:controller.wait(timeout=8)
        except subprocess.TimeoutExpired:controller.terminate();controller.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()
