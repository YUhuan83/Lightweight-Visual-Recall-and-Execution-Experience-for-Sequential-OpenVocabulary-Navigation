"""Read-only RViz overlays: in-process memory, navigation state and odom trail."""
import json
import math
import cv2
from copy import deepcopy
from collections import deque
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
from graphnav_msgs.msg import NavigationGraph
from .scene_display import graph_primitives, ObjectLabelIndex


class VisualizationBridge(Node):
    def __init__(self):
        super().__init__('wildos_visualization_bridge')
        cv2.setNumThreads(1)
        self.pose=None; self.query=''; self.status={}; self.selection={}; self.views=[]
        self.history=[]
        self.pose_samples=deque(maxlen=128)
        self.graph=None;self.labels=ObjectLabelIndex();self.last_camera_stamp_ns=None
        self.scene_pub=self.create_publisher(MarkerArray,'/wildos/viz/topology',10)
        self.object_labels_pub=self.create_publisher(MarkerArray,'/wildos/viz/object_labels',10)
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
        self.create_subscription(NavigationGraph,'/spot1/nav_graph',self.on_graph,qos_profile_sensor_data)
        self.create_subscription(String,'/spot1/object_localization',lambda m:self.on_object(m),10)
        self.create_subscription(String,'/spot1/object_confirmation',lambda m:self.on_object(m,confirmed=True),10)
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

    def on_graph(self,m):
        self.graph=m

    def on_object(self,m,confirmed=False):
        try:data=json.loads(m.data)
        except (ValueError,TypeError):return
        self.labels.observe(data,self.get_clock().now().nanoseconds,confirmed=confirmed)

    def pose_at(self,stamp):
        wanted=stamp.sec*10**9+stamp.nanosec
        candidates=list(self.pose_samples)
        if self.pose is not None:candidates.append(self.pose)
        candidates=[p for p in candidates if p.header.frame_id==self.pose.header.frame_id]
        if not candidates:return None
        nearest=min(candidates,key=lambda p:abs(p.header.stamp.sec*10**9+p.header.stamp.nanosec-wanted))
        gap=abs(nearest.header.stamp.sec*10**9+nearest.header.stamp.nanosec-wanted)
        return nearest if gap<=150_000_000 else None

    def publish_overlays(self):
        for pub,source in self.overlays.values():
            if source is None:continue
            m=deepcopy(source)
            if isinstance(m,Path) and m.header.frame_id=='spot1/nav_base_link':
                observed_pose=self.pose_at(m.header.stamp)
                if observed_pose is None:continue
                q=observed_pose.pose.pose.orientation
                heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                c,s=math.cos(heading),math.sin(heading);p=observed_pose.pose.pose.position
                m.header.frame_id=observed_pose.header.frame_id
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
            self.labels.set_query(m.data,self.get_clock().now().nanoseconds)

    def on_mask(self,m):
        if m.query==self.query:
            self.target_mask=m

    def on_camera(self,m):
        # Inference often takes more than 0.5 s. A stationary scene can keep
        # its mask longer; moving scenes must discard an old projection.
        max_age=.5
        self.last_camera_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
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
        if self.pose is not None and self.pose.header.frame_id!=m.header.frame_id:
            self.history=[];self.pose_samples.clear();self.graph=None
        self.pose=m
        self.pose_samples.append(m)
        self.labels.set_frame(m.header.frame_id)
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
        now=self.get_clock().now().nanoseconds
        if self.last_camera_stamp_ns is None:state.text+='\nCamera: waiting'
        elif now-self.last_camera_stamp_ns>1_000_000_000:state.text+='\nCamera: STALE'
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
        self.draw_scene(now)
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

    def draw_scene(self,now):
        frame=self.pose.header.frame_id;z=self.pose.pose.pose.position.z+.18
        clear=Marker();clear.action=Marker.DELETEALL
        markers=[deepcopy(clear)]
        if self.graph is not None and self.graph.header.frame_id==frame:
            data=graph_primitives(self.graph)
            for index,key,kind,color,size in (
                    (0,'edges',Marker.LINE_LIST,(.45,.5,.6),.07),
                    (1,'nodes',Marker.SPHERE_LIST,(.2,.9,.45),.4),
                    (2,'frontiers',Marker.SPHERE_LIST,(1.,.65,.1),.8),
                    (3,'frontier_points',Marker.CUBE_LIST,(.85,.3,1.),.25)):
                marker=self.marker(index,kind,frame,color);marker.ns='topology/'+key
                marker.scale.x=marker.scale.y=marker.scale.z=size
                marker.points=[Point(x=x,y=y,z=z) for x,y in data[key]]
                markers.append(marker)
        self.scene_pub.publish(MarkerArray(markers=markers))
        labels=[deepcopy(clear)]
        for item in self.labels.visible(now):
            color=(.3,1.,.6) if item.confirmed else (1.,.7,.2)
            marker=self.marker(item.serial*2,Marker.TEXT_VIEW_FACING,item.frame,color)
            marker.ns='object_labels';marker.pose.position.x=item.position[0]
            marker.pose.position.y=item.position[1]+.7;marker.pose.position.z=z+.1
            marker.scale.z=.7;marker.text=item.label+(' [confirmed]' if item.confirmed else ' [localized]')
            labels.append(marker)
            dot=self.marker(item.serial*2+1,Marker.SPHERE,item.frame,color);dot.ns='object_positions'
            dot.pose.position.x=item.position[0];dot.pose.position.y=item.position[1];dot.pose.position.z=z
            dot.scale.x=dot.scale.y=dot.scale.z=.5;labels.append(dot)
        self.object_labels_pub.publish(MarkerArray(markers=labels))


def main(args=None):
    rclpy.init(args=args); node=VisualizationBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
