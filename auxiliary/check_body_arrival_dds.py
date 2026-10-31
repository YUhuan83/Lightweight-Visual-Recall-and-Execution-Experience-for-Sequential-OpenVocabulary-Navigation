"""Synthetic DDS arrival-contract test. No simulator, model or actuation."""
import hashlib,inspect,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String,MultiArrayDimension
from object_search_msgs.msg import ObjectMaskWithTf
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
from wildos_nav2_adapter import arrival_criteria
from town_trial_harness import Harness

ROOT=Path('/home/ly/execution_continuity_review_20261022/v26_body_arrival')
OUT=ROOT/'dds_contract';OUT.mkdir(exist_ok=True)
reports=[]
for repeat in (1,2):
    out=OUT/f'repeat_{repeat}';out.mkdir(exist_ok=False)
    rclpy.init(args=['--ros-args','-p','carrier_model:=ackermann','-p','minimum_turning_radius:=8.0'])
    bridge=ContinuousGraphPathBridge();goal=GoalStandoff()
    harness=Harness(out,os.getpid(),carrier_model='ackermann')
    source=Node('controlled_arrival_source')
    odom_pub=source.create_publisher(Odometry,'/spot1/odom',10)
    query_pub=source.create_publisher(String,'/spot1/object_search/query',10)
    loc_pub=source.create_publisher(String,'/spot1/object_localization',10)
    mask_pub=source.create_publisher(ObjectMaskWithTf,'/spot1/object_mask',10)
    executor=SingleThreadedExecutor()
    for node in (bridge,goal,harness,source):executor.add_node(node)
    def spin(seconds,predicate=None):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def query(text):
        harness.query=text;harness.query_started_ns=source.get_clock().now().nanoseconds
        harness.arrival_statuses.clear();harness.confirmation_history.clear();harness.mask_history.clear()
        harness.masks.pop(text,None)
        query_pub.publish(String(data=text))
        assert spin(3.,lambda:bridge.query==text and goal.active_query==text)
    def measurement(text,x=5.4,stamp=None):
        stamp=stamp or source.get_clock().now().to_msg()
        odom=Odometry();odom.header.frame_id='spot1/odom';odom.header.stamp=stamp
        odom.child_frame_id='spot1/base_link';odom.pose.pose.orientation.w=1.
        odom_pub.publish(odom)
        mask=ObjectMaskWithTf();mask.header=odom.header;mask.query=text;mask.odom=odom
        mask.object_mask.layout.dim=[MultiArrayDimension(label=k,size=v) for k,v in zip(('B','C','H','W'),(1,1,2,2))]
        mask.object_mask.data=[1]*4
        info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='camera';info.width=info.height=2
        info.k=[1.,0.,.5,0.,1.,.5,0.,0.,1.]
        tr=TransformStamped();tr.header=odom.header;tr.child_frame_id='camera'
        tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
        mask.cam_infos=[info];mask.cam_transforms.transforms=[tr]
        mask_pub.publish(mask)
        value=dict(query=text,frame='spot1/odom',observation_stamp_ns=stamp.sec*10**9+stamp.nanosec,
            position=[x,0.,2.],observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],
            source='lidar',localization_valid=True,quality=dict(confidence=.65,association_validated=True))
        loc_pub.publish(String(data=json.dumps(value)))
        spin(.12)
        return value
    try:
        assert spin(5.,lambda:odom_pub.get_subscription_count()>=3 and query_pub.get_subscription_count()>=2 and loc_pub.get_subscription_count()>=1 and mask_pub.get_subscription_count()>=2)
        query('house')
        for _ in range(3):measurement('house',5.6)
        assert not bridge.object_found and not harness.confirmation_history
        measurement('house',5.6)
        assert spin(2.,lambda:bool(harness.confirmation_history))
        assert not bridge.object_found and harness.arrival_evidence('house',3) is None
        for _ in range(6):
            measurement('house',5.4)
            if bridge.object_found:break
        assert spin(2.,lambda:harness.arrival_evidence('house',3) is not None)
        accepted=harness.arrival_evidence('house',3)
        assert bridge.object_found and accepted['status']['arrival_basis']=='vehicle_body_clearance'
        assert math.isclose(accepted['status']['object_distance_m'],2.,abs_tol=1e-9)
        old=accepted['confirmation'];query('gas station')
        loc_pub.publish(String(data=json.dumps(dict(query='house',frame='spot1/odom',
            observation_stamp_ns=old['observation_stamp_ns'],position=old['object_position'],
            observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],source='lidar',
            localization_valid=True,quality=dict(confidence=.65,association_validated=True)))))
        spin(.2);assert not bridge.object_found
        for _ in range(3):measurement('gas station')
        assert not bridge.object_found and harness.arrival_evidence('gas station',3) is None
        measurement('gas station')
        assert spin(2.,lambda:harness.arrival_evidence('gas station',3) is not None)
        report=dict(repeat=repeat,passed=True,transport='DDS real ROS messages',
            carrier=bridge.carrier_model,object_arrival_distance=bridge.get_parameter('object_arrival_distance').value,
            traffic_light_arrival_distance=bridge.get_parameter('traffic_light_arrival_distance').value,
            first_arrival=accepted['status'],second_arrival=harness.arrival_evidence('gas station',3)['status'],
            negative_checks=['three_observations','body_gap_2_2m','old_query_after_switch'],
            source='controlled synthetic masks/localizations and stationary odometry; no physical or model validation',
            modules={name:dict(path=inspect.getfile(cls),sha256=hashlib.sha256(Path(inspect.getfile(cls)).read_bytes()).hexdigest()) for name,cls in [('bridge',ContinuousGraphPathBridge),('goal',GoalStandoff)]},
            arrival_helper=arrival_criteria.__file__)
        reports.append(report)
        (OUT/'results.json').write_text(json.dumps(reports,indent=2))
        print(json.dumps(dict(repeat=repeat,passed=True,body_distance_m=accepted['status']['object_distance_m'],scope=report['source'])),flush=True)
    finally:
        executor.shutdown()
        for node in (source,harness,goal,bridge):node.destroy_node()
        for f in (harness.log,harness.native_log,harness.sensor_timing_log):f.close()
        rclpy.shutdown()
