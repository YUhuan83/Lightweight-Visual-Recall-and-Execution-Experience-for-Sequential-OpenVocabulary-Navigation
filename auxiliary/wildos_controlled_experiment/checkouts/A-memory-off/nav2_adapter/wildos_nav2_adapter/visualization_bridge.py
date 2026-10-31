"""Read-only RViz overlays: in-process memory, navigation state and odom trail."""
import json
import math
import cv2
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from .model_visualization import rgb_target_image
from object_search_msgs.msg import ObjectMaskWithTf


class VisualizationBridge(Node):
    def __init__(self):
        super().__init__('wildos_visualization_bridge')
        cv2.setNumThreads(1)
        self.pose=None; self.query=''; self.status={}; self.selection={}; self.views=[]
        self.history=[]
        self.overlays={}
        self.pub=self.create_publisher(MarkerArray,'/wildos/debug_markers',10)
        self.frontier_scores_pub=self.create_publisher(MarkerArray,'/wildos/viz/frontier_scores',10)
        self.frontier_map_pub=self.create_publisher(GridMap,'/wildos/viz/unexplored_space_map',10)
        self.target_mask=None
        image_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.model_pub=self.create_publisher(Image,'/wildos/viz/rgb_target',image_qos)
        self.create_subscription(ObjectMaskWithTf,'/spot1/object_mask',self.on_mask,1)
        self.create_subscription(Image,'/spot1/realsense/front/color/image_raw',
                                 self.on_camera,image_qos)
        self.trail=self.create_publisher(Path,'/wildos/odom_trail',10)
        self.create_subscription(Odometry,'/spot1/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,'/spot1/object_search/query',self.on_query,10)
        self.create_subscription(String,'/wildos/plan_status',lambda m:self.on_json('status',m),10)
        self.create_subscription(String,'/wildos/memory_selection',lambda m:self.on_json('selection',m),10)
        self.create_subscription(String,'/spot1/remembered_object_candidates',self.candidates,10)
        for key, kind, topic in (
                ('local_costmap',OccupancyGrid,'/local_costmap/costmap'),
                ('global_costmap',OccupancyGrid,'/global_costmap/costmap'),
                ('planned_path',Path,'/wildos/planned_path'),
                ('graph_path',Path,'/spot1/graphnav_planner/path'),
                ('received_plan',Path,'/received_global_plan'),
                ('footprint',PolygonStamped,'/local_costmap/published_footprint')):
            pub=self.create_publisher(kind,'/wildos/viz/'+key,10)
            self.overlays[key]=(pub,None)
            self.create_subscription(kind,topic,lambda m,key=key:self.overlay(key,m),qos_profile_sensor_data)
        self.frontier_scores=None; self.frontier_map=None
        self.declare_parameter('show_frontiers',False)
        if bool(self.get_parameter('show_frontiers').value):
            self.create_subscription(MarkerArray, '/spot1/graphnav_planner/frontier_scores',
                                     lambda m: setattr(self, 'frontier_scores', m), 10)
            self.create_subscription(GridMap, '/spot1/graphnav_planner/unexplored_space_map',
                                     lambda m: setattr(self, 'frontier_map', m), 10)
        self.create_timer(.5,self.draw)

    def overlay(self,key,m):
        pub,_=self.overlays[key]
        self.overlays[key]=(pub,m)

    def publish_overlays(self):
        for pub,source in self.overlays.values():
            if source is None:continue
            m=deepcopy(source)
            if isinstance(m,Path) and m.header.frame_id=='spot1/nav_base_link':
                q=self.pose.pose.pose.orientation
                heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                c,s=math.cos(heading),math.sin(heading);p=self.pose.pose.pose.position
                m.header.frame_id=self.pose.header.frame_id
                for ps in m.poses:
                    x,y=ps.pose.position.x,ps.pose.position.y
                    ps.pose.position.x=p.x+c*x-s*y;ps.pose.position.y=p.y+s*x+c*y
                    q=ps.pose.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+heading
                    q.x=q.y=0.;q.z=math.sin(yaw/2);q.w=math.cos(yaw/2)
                    ps.header=deepcopy(m.header)
            # Display copies only: original Nav2 data and coordinates stay intact.
            if m.header.frame_id==self.pose.header.frame_id:
                z=self.pose.pose.pose.position.z
                if isinstance(m,OccupancyGrid):m.info.origin.position.z=z-.1
                elif isinstance(m,Path):
                    for ps in m.poses:ps.pose.position.z=z+.05
                elif isinstance(m,PolygonStamped):
                    for p in m.polygon.points:p.z=z+.1
            pub.publish(m)

    def on_query(self,m):
        if m.data!=self.query:
            self.query=m.data; self.views=[]; self.selection={}; self.target_mask=None

    def on_mask(self,m):
        if m.query==self.query:
            self.target_mask=m

    def on_camera(self,m):
        # Inference often takes more than 0.5 s. A stationary scene can keep
        # its mask longer; moving scenes must discard an old projection.
        max_age=.5
        if self.pose is not None:
            stamp=self.pose.header.stamp.sec*10**9+self.pose.header.stamp.nanosec
            v=self.pose.twist.twist.linear
            if (0 <= self.get_clock().now().nanoseconds-stamp <= 500000000
                    and math.hypot(v.x,v.y)<.1
                    and abs(self.pose.twist.twist.angular.z)<.05):
                max_age=2.
        self.model_pub.publish(rgb_target_image(m,self.target_mask,max_mask_age_sec=max_age))

    def on_json(self,name,m):
        try: value=json.loads(m.data)
        except (ValueError,TypeError): return
        if not isinstance(value,dict): return
        if name=='selection' and value.get('query')!=self.query: return
        setattr(self,name,value)

    def candidates(self,m):
        try: data=json.loads(m.data)
        except (ValueError,TypeError): return
        if isinstance(data,dict) and data.get('query')==self.query and isinstance(data.get('views'),list):
            self.views=data['views']

    def odom(self,m):
        self.pose=m
        p=m.pose.pose.position
        if not self.history or math.hypot(p.x-self.history[-1].pose.position.x,p.y-self.history[-1].pose.position.y)>.05:
            ps=PoseStamped(); ps.header=m.header; ps.pose=m.pose.pose
            self.history.append(ps); self.history=self.history[-1500:]

    def marker(self,idx,kind,frame='spot1/odom',color=(1.,1.,1.)):
        m=Marker(); m.header.frame_id=frame; m.header.stamp=self.get_clock().now().to_msg()
        m.ns='wildos'; m.id=idx; m.type=kind; m.action=Marker.ADD
        m.pose.orientation.w=1.; m.color.r,m.color.g,m.color.b=color; m.color.a=1.
        return m

    def draw(self):
        if self.pose is None: return
        delete=Marker(); delete.action=Marker.DELETEALL
        markers=[delete]
        state=self.marker(0,Marker.TEXT_VIEW_FACING,'spot1/base_link')
        state.pose.position.z=3.; state.scale.z=.5
        state.text='query: '+self.query+'\n'+self.status.get('status','waiting')+'\n'+self.status.get('source','')
        markers.append(state)
        z=self.pose.pose.pose.position.z+.25
        for i,view in enumerate(self.views[:64]):
            if not isinstance(view,dict):continue
            try:
                x,y,_=map(float,view['position']); heading=float(view['heading'])
                if not all(map(math.isfinite,(x,y,heading))):continue
            except (KeyError,TypeError,ValueError):continue
            arrow=self.marker(10+i,Marker.ARROW,view.get('frame','spot1/odom'),(.3,.65,1.))
            arrow.points=[Point(x=x,y=y,z=z),Point(x=x+2*math.cos(heading),y=y+2*math.sin(heading),z=z)]
            arrow.scale.x=.12; arrow.scale.y=.3; arrow.scale.z=.3
            markers.append(arrow)
        goal=self.selection.get('goal')
        if self.selection.get('status')=='memory_candidate_selected' and isinstance(goal,list) and len(goal)>=2:
            try:
                x,y=map(float,goal[:2])
                if not all(map(math.isfinite,(x,y))):raise ValueError('nonfinite goal')
            except (TypeError,ValueError):pass
            else:
                sphere=self.marker(100,Marker.SPHERE,color=(1.,.7,.1))
                sphere.pose.position.x=x;sphere.pose.position.y=y;sphere.pose.position.z=z
                sphere.scale.x=sphere.scale.y=sphere.scale.z=.8
                markers.append(sphere)
        self.pub.publish(MarkerArray(markers=markers))
        self.publish_overlays()
        if self.frontier_scores is not None:
            self.frontier_scores_pub.publish(deepcopy(self.frontier_scores))
        if self.frontier_map is not None:
            frontier = deepcopy(self.frontier_map)
            if not frontier.header.frame_id:
                frontier.header.frame_id = self.pose.header.frame_id
            self.frontier_map_pub.publish(frontier)
        route=Path(); route.header=self.pose.header; route.poses=self.history
        self.trail.publish(route)


def main(args=None):
    rclpy.init(args=args); node=VisualizationBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
