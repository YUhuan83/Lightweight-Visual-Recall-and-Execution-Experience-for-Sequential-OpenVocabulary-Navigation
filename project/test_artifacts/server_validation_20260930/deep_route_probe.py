import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from wildos_nav2_adapter.graph_route import route_candidates, distance_to_route, distance, cell_cost, footprint_blockers, validate_vehicle_path
from wildos_nav2_adapter.graph_path_bridge import yaw, xy

rclpy.init();n=Node('deep_vehicle_route_probe');s={}
for key,typ,topic,qos in [('odom',Odometry,'/spot1/odom',qos_profile_sensor_data),('grid',OccupancyGrid,'/global_costmap/costmap',10),('route',Route,'/spot1/graphnav_planner/path',10)]:
    n.create_subscription(typ,topic,lambda m,k=key:s.__setitem__(k,m),qos)
end=time.monotonic()+10
while time.monotonic()<end and len(s)<3:
    rclpy.spin_once(n,timeout_sec=.2)
if 'odom' not in s or 'grid' not in s: raise RuntimeError('Missing topics: '+str(s.keys()))
o=s['odom'];g=s['grid'];route=[xy(p.pose) for p in s['route'].poses] if 'route' in s else [xy(o.pose.pose)]
target=(740.5373794974876,-14.956937346942368)
if not route or distance(route[-1],target)>1e-6:route.append(target)
report={'pose':[*xy(o.pose.pose),yaw(o.pose.pose)],'route':route,'route_source':'live_graphnav' if 'route' in s else 'direct_probe_polyline','services':[v for v,_ in n.get_service_names_and_types() if 'planner_server' in v], 'parameters':{},'plans':[]}
for remote,names in [('/planner_server',['planner_plugins','CarLike.minimum_turning_radius','CarLike.minimum_turning_radius','CarLike.smooth_path','CarLike.smoother.max_iterations','CarLike.motion_model_for_search'])]:
    cl=n.create_client(GetParameters,remote+'/get_parameters')
    if cl.wait_for_service(timeout_sec=4):
        fut=cl.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,fut,timeout_sec=10)
        if fut.done():report['parameters'][remote]=dict(zip(names,[rclpy.parameter.parameter_value_to_python(v) for v in fut.result().values]))
client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
if not client.wait_for_server(timeout_sec=5):raise RuntimeError('planner unavailable')
for x,y,h,arc in route_candidates(xy(o.pose.pose),route,g):
    # The terminal remembered viewpoint must retain its observed heading.
    if distance((x,y),target)<1e-6:h=-.48737
    req=ComputePathToPose.Goal();req.use_start=False;req.planner_id='CarLike';req.goal.header.frame_id=g.header.frame_id;req.goal.header.stamp=n.get_clock().now().to_msg()
    req.goal.pose.position.x=x;req.goal.pose.position.y=y;req.goal.pose.orientation.z=math.sin(h/2);req.goal.pose.orientation.w=math.cos(h/2)
    f=client.send_goal_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    handle=f.result();f=handle.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    wrapped=f.result();p=wrapped.result.path;pts=[xy(v.pose) for v in p.poses];heads=[yaw(v.pose) for v in p.poses]
    record={'target':[x,y,h],'status':wrapped.status,'poses':len(pts),'points':pts,'headings':heads,'validation':validate_vehicle_path(pts,heads,[xy(o.pose.pose)]+route,g,(x,y),start=xy(o.pose.pose),target_heading=h if distance((x,y),target)<1e-6 else None)}
    if pts:
        record['maximum_corridor_distance']=max(distance_to_route(a,[xy(o.pose.pose)]+route) for a in pts)
        record['center_blocked']=[{'xy':a,'cost':cell_cost(g,*a)} for a in pts if cell_cost(g,*a)>=99]
        record['footprint_blocked']=[{'xy':a,'heading':b,'samples':footprint_blockers(g,a,b,limit=2)} for a,b in zip(pts,heads) if footprint_blockers(g,a,b,limit=1)]
        traveled=[0]
        for a,b in zip(pts,pts[1:]):traveled.append(traveled[-1]+distance(a,b))
        curvatures=[]
        for i in range(len(pts)-1):
            j=i+1
            while j<len(pts)-1 and traveled[j]-traveled[i]<2:j+=1
            length=traveled[j]-traveled[i]
            if length>=.5:curvatures.append(abs(math.atan2(math.sin(heads[j]-heads[i]),math.cos(heads[j]-heads[i])))/length)
        record['maximum_curvature']=max(curvatures,default=0)
        record['minimum_radius']=1/record['maximum_curvature'] if record['maximum_curvature'] else None
    report['plans'].append(record)
    print(json.dumps({k:v for k,v in record.items() if k not in ('points','headings','footprint_blocked')}),flush=True)
out=Path('/home/ly/navigation_validation_20260930_logs')
out.joinpath('deep_route_probe.json').write_text(json.dumps(report,indent=2))
import numpy as np
np.savez_compressed(out/'costmap_snapshot.npz',data=np.array(g.data).reshape(g.info.height,g.info.width),origin=[g.info.origin.position.x,g.info.origin.position.y],resolution=g.info.resolution)
print(json.dumps({'pose':report['pose'],'route':route,'parameters':report['parameters'],'services':report['services']},indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
