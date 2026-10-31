from pathlib import Path
import json,time,math,traceback
import rclpy
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
out=Path(__file__).parent;rclpy.init();n=rclpy.create_node('plan_clear_probe_20261009');state={};report={}
n.create_subscription(Odometry,'/spot1/odom',lambda m:state.update(odom=m),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:state.update(grid=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
def wait(f,sec,label):
 start=time.monotonic()
 while not f.done() and time.monotonic()-start<sec:rclpy.spin_once(n,timeout_sec=.02)
 assert f.done(),label+' timed out'
 return f.result(),time.monotonic()-start
try:
 report['clear_services']=[]
 for service in ('/local_costmap/clear_entirely_local_costmap','/global_costmap/clear_entirely_global_costmap'):
  c=n.create_client(ClearEntireCostmap,service);assert c.wait_for_service(timeout_sec=5),service+' unavailable'
  _,duration=wait(c.call_async(ClearEntireCostmap.Request()),5,service);report['clear_services'].append({'name':service,'duration_s':duration})
 spin(3);assert 'grid' in state and 'odom' in state,'missing map/odom'
 odom=state['odom'];q=odom.pose.pose.orientation;h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 goal=ComputePathToPose.Goal();goal.start=PoseStamped();goal.start.header.frame_id='spot1/odom';goal.start.header.stamp=n.get_clock().now().to_msg();goal.start.pose=odom.pose.pose
 goal.goal=PoseStamped();goal.goal.header=goal.start.header;goal.goal.pose.orientation=q;goal.goal.pose.position.x=odom.pose.pose.position.x+3*math.cos(h);goal.goal.pose.position.y=odom.pose.pose.position.y+3*math.sin(h);goal.planner_id='GridBased';goal.use_start=True
 a=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert a.wait_for_server(timeout_sec=5)
 handle,_=wait(a.send_goal_async(goal),5,'goal response');assert handle.accepted,'plan rejected'
 res,duration=wait(handle.get_result_async(),10,'plan result');report.update(status=res.status,duration_s=duration,poses=len(res.result.path.poses))
 assert res.status==4 and res.result.path.poses,'no successful path'
 grid=state['grid'];assert grid.header.frame_id=='spot1/odom';hits=[];outside=[]
 for p in res.result.path.poses:
  x=int(math.floor((p.pose.position.x-grid.info.origin.position.x)/grid.info.resolution));y=int(math.floor((p.pose.position.y-grid.info.origin.position.y)/grid.info.resolution))
  if 0<=x<grid.info.width and 0<=y<grid.info.height:
   if grid.data[y*grid.info.width+x]>=99:hits.append([x,y])
  else:outside.append([x,y])
 report.update(obstacle_cells=sum(v>=99 for v in grid.data),occupied_path_cells=hits,outside_path=outside,resolution=grid.info.resolution,map_stamp=[grid.header.stamp.sec,grid.header.stamp.nanosec])
 assert report['obstacle_cells']>0,'no actual obstacles observed'
 assert not hits and not outside,'path enters obstacle or outside current map'
 report['passed']=True
except BaseException as e:report.update(passed=False,error=repr(e),traceback=traceback.format_exc())
finally:
 (out/'plan_clear_probe.json').write_text(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
print(json.dumps(report))
