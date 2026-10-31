"""Stationary live verification after a height-policy change; no navigation."""
import json
import math
from pathlib import Path
import time
import numpy as np
from beamngpy import BeamNGpy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParameters
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from wildos_nav2_adapter.pointcloud import decode_xyz

ROOT=Path('/home/ly/town_memory_validation_20261003/collision_source_fix')
rclpy.init();node=Node('verify_collision_filter_live')
clouds=[];positions=[];sim=None
node.create_subscription(PointCloud2,'/wildos/points',lambda m:clouds.append((time.monotonic(),m)),qos_profile_sensor_data)
node.create_subscription(Odometry,'/spot1/odom',lambda m:positions.append((m.pose.pose.position.x,m.pose.pose.position.y)),qos_profile_sensor_data)

def call(typ,name,req):
    c=node.create_client(typ,name)
    if not c.wait_for_service(timeout_sec=5):raise TimeoutError(name)
    f=c.call_async(req);deadline=time.monotonic()+5
    while not f.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    if not f.done():raise TimeoutError(name)
    return f.result()

def parameters(name,names):
    response=call(GetParameters,name+'/get_parameters',GetParameters.Request(names=names))
    return {key:(v.bool_value if v.type==1 else v.double_value if v.type==3 else v.string_value)
            for key,v in zip(names,response.values)}

def snapshot():
    out={}
    for key in ('local','global'):
        m=call(GetCostmap,'/'+key+'_costmap/get_costmap',GetCostmap.Request()).map
        meta=m.metadata;data=np.asarray(m.data);count=np.count_nonzero(data==254)
        x=math.floor((727.1-meta.origin.position.x)/meta.resolution)
        y=math.floor((-35.9-meta.origin.position.y)/meta.resolution)
        out[key]=dict(lethal_cells=int(count),reference_cell_raw_cost=int(data[y*meta.size_x+x])
            if 0<=x<meta.size_x and 0<=y<meta.size_y else None)
    return out

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)

result=dict(scope='stationary live point-cloud/costmap verification; navigation not enabled',map_reload_count=0)
try:
    result['drive_parameters']=parameters('/wildos_cmd_vel_to_control',['enable'])
    result['graph_parameters']=parameters('/wildos_graph_path_bridge',['follow_path','local_costmap_timeout_sec'])
    assert result['drive_parameters']['enable'] is False and result['graph_parameters']['follow_path'] is False
    result['controller_parameters']=parameters('/controller_server',['FollowPath.use_collision_detection'])
    result['filter_before']=parameters('/wildos_official_cloud_filter',['max_height'])
    result['maps_before']=snapshot()
    req=SetParameters.Request(parameters=[Parameter('max_height',value=1.5).to_parameter_msg()])
    response=call(SetParameters,'/wildos_official_cloud_filter/set_parameters',req)
    assert all(v.successful for v in response.results)
    result['filter_after']=parameters('/wildos_official_cloud_filter',['max_height'])
    sim=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
    sim.control.resume()
    spin(5.)
    # One-time migration: obsolete tall returns would otherwise remain marked
    # after the height policy changes. Never clear maps as a recovery loop.
    cleared_at=time.monotonic()
    for key in ('local','global'):
        service='/'+key+'_costmap/clear_entirely_'+key+'_costmap'
        call(ClearEntireCostmap,service,ClearEntireCostmap.Request())
    spin(6.)
    result['maps_after']=snapshot()
    points=[decode_xyz(m) for t,m in clouds if t>cleared_at and m.width]
    result['new_cloud_frames']=len(points)
    result['max_marking_height']=max(float(p[:,2].max()) for p in points) if points else None
    result['odom_frames']=len(positions)
    result['max_position_change']=max(math.dist(positions[0],p) for p in positions) if positions else None
    result['passed']=bool(points and positions and result['max_marking_height']<=1.50001
        and result['max_position_change']<.1
        and result['controller_parameters']['FollowPath.use_collision_detection'] is True)
    print(json.dumps(result,indent=2),flush=True)
    if not result['passed']:raise RuntimeError('Stationary map/filter verification failed')
finally:
    if sim is not None:sim.control.pause();sim.disconnect()
    result['paused_at_end']=sim is not None
    (ROOT/'live_verification.json').write_text(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
