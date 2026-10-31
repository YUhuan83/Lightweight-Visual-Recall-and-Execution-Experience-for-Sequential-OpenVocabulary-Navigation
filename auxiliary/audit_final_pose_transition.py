"""Controlled interface audit, not field evidence or a model detection test."""
import json,math,sys,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
root=Path(sys.argv[1])
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),
             str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from nav_msgs.msg import Odometry,OccupancyGrid
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.arrival_criteria import vehicle_body_distance
n=GraphPathBridge()
try:
    stamp=100_300_000_000
    n.get_clock=lambda:S(now=lambda:S(nanoseconds=stamp,to_msg=lambda:S(sec=100,nanosec=300000000)))
    n.query='green residential house';n.query_started_ns=100_000_000_000
    n.query_reset_pending=False;n.carrier_model='ackermann';n.minimum_turning_radius=12.;n.follow_enabled=True
    n.planner_id='CarLike';n.query_generation=1
    odom=Odometry();odom.header.frame_id='spot1/odom';odom.pose.pose.orientation.w=1.
    odom.header.stamp=Time(sec=100,nanosec=300000000)
    n.odom=odom;n.odom_received_at=time.monotonic();n.input_freshness_issue=lambda:None
    grid=OccupancyGrid();grid.header.frame_id='spot1/odom';grid.header.stamp=odom.header.stamp
    grid.info.width=grid.info.height=200;grid.info.resolution=.5
    grid.info.origin.position.x=grid.info.origin.position.y=-50.;grid.info.origin.orientation.w=1.
    grid.data=[0]*40000
    n.grid=n.local_grid=grid;n.grid_arrived_at=n.grid_received_at=time.monotonic()
    target=[4.7/math.sqrt(2),4.7/math.sqrt(2),2.]
    clearance=4.351995092
    point=[(4.7-clearance)/math.sqrt(2)]*2
    n.expected=n.confirmed_route_goal=(0.,0.);n.confirmed_route_object=tuple(target[:2])
    n.confirmed_route_radius=.02;n.confirmed_route_started_at=time.monotonic()-11
    n.confirmed_route_heading=0.
    n.waypoint_pub=Mock();n.status_pub=Mock();n.retry_graph_reset=Mock()
    data=dict(query=n.query,frame='spot1/odom',observation_stamp_ns=stamp,object_position=target,
        observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],radius=.02,inliers=4,observations=4,
        source='lidar',localization_valid=True,position_confirmed=True,
        quality=dict(confidence=.8,association_validated=True),instance_id='same-target',
        search_confirmed=True,approach_goal_ready=True,standoff=point,standoff_clamped=True,
        standoff_navigation_heading=None)
    n.on_object_confirmation(String(data=json.dumps(data)))
    report=dict(scope='Controlled final transition contract only, hypothetical target/pose not observed field geometry',
        current_body_gap_m=vehicle_body_distance((0.,0.),0.,target,'ackermann'),
        requested_final_point=point,requested_final_heading=math.pi/4,
        current_heading=0.,point_distance_m=math.hypot(*point),
        status=n.last_status,object_found=n.object_found,
        navigation_intents_published=n.waypoint_pub.publish.call_count,expected=n.expected,
        following=n.following,pending=n.pending)
    print(json.dumps(report,indent=2))
finally:n.destroy_node()
