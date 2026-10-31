"""Exercise the deployed arrival callbacks, without driving or changing code."""
import hashlib, inspect, json, sys, time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter import arrival_criteria

root=Path('/home/ly/execution_continuity_review_20261022/target_contract_20261025')
rows=[]
rclpy.init()
now=100_000_000_000
def setup(node):
    node.get_clock=lambda:S(now=lambda:Time(nanoseconds=now))
    o=Odometry();o.header.frame_id='spot1/odom';o.header.stamp=Time(nanoseconds=now).to_msg();o.pose.pose.orientation.w=1.
    if isinstance(node,GoalStandoff):
        node.publisher=Mock();node.confirmation_pub=Mock();node.on_odom(o);node.on_query(String(data='green residential house'))
    else:
        node.status_pub=Mock();node.waypoint_pub=Mock();node.retry_graph_reset=Mock()
        node.on_query(String(data='green residential house'));node.query_reset_pending=False
        node.odom=o;node.odom_received_at=time.monotonic();node.route_confirmed_target=Mock(return_value=False)
    return o
try:
    for count,distance in ((3,4.),(4,4.),(4,5.01)):
        now=100_000_000_000;n=GoalStandoff();setup(n)
        n.publish_standoff=Mock(return_value='no_reachable_standoff')
        for i in range(count):
            now+=100_000_000
            payload=dict(query=n.active_query,frame='spot1/odom',observation_stamp_ns=now,
                position=[distance,0.,2.],observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],
                source='lidar',localization_valid=True,quality=dict(confidence=.65,association_validated=True))
            n.on_localization(String(data=json.dumps(payload)))
        emitted=[json.loads(c.args[0].data) for c in n.confirmation_pub.publish.call_args_list]
        rows.append(dict(stage='GoalStandoff',observations=count,target_distance=distance,confirmation_count=len(emitted),last=emitted[-1] if emitted else None))
        assert (len(emitted)>0)==(count>=4)
        if emitted:assert emitted[-1]['position_confirmed'] and not emitted[-1]['search_confirmed']
        n.destroy_node()
    for distance,standoff in ((4.,[0.,3.]),(4.8,[0.,0.]),(4.,[0.,0.])):
        now=100_000_000_000;n=GraphPathBridge();setup(n)
        now+=100_000_000
        data=dict(query=n.query,frame='spot1/odom',observation_stamp_ns=now,search_confirmed=True,
            position_confirmed=True,localization_valid=True,quality=dict(confidence=.65,association_validated=True),
            object_position=[distance,0.,2.],observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],
            radius=.1,inliers=4,observations=4,standoff=standoff,standoff_clamped=False,instance_id='query-local-test')
        n.on_object_confirmation(String(data=json.dumps(data)))
        state=n.last_status['status']
        rows.append(dict(stage='GraphPathBridge',target_distance=distance,standoff=standoff,status=state,found=n.object_found))
        assert n.object_found==(distance==4. and standoff==[0.,0.])
        n.destroy_node()
finally:rclpy.shutdown()
sources={cls.__name__:dict(path=inspect.getfile(cls),sha256=hashlib.sha256(Path(inspect.getfile(cls)).read_bytes()).hexdigest()) for cls in (GoalStandoff,GraphPathBridge)}
sources['arrival_criteria']=dict(path=arrival_criteria.__file__,sha256=hashlib.sha256(Path(arrival_criteria.__file__).read_bytes()).hexdigest())
encoded=json.dumps(dict(sources=sources,cases=rows,scope='Deployed real ROS classes, isolated callbacks with controlled messages; no simulator, no field-success claim.'),sort_keys=True).encode()
(root/'arrival_contract.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),cases=rows)))
