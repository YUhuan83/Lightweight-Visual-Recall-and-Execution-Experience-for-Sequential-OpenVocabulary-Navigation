"""Capture one exact raw/scored graph pair and odometry, read-only during field test."""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry

p=argparse.ArgumentParser();p.add_argument('output',type=Path);args=p.parse_args();args.output.mkdir(exist_ok=False)
rclpy.init();node=Node('field_graph_snapshot_capture');raw=collections.deque(maxlen=32);state={}
def key(g):return (g.header.frame_id,g.header.stamp.sec,g.header.stamp.nanosec)
def scored(message):
    if 'pair' in state:return
    for original in reversed(raw):
        if key(original)==key(message) and len(original.nodes)==len(message.nodes) and original.trav_classes==message.trav_classes:
            if all(a.uuid==b.uuid and a.pose==b.pose for a,b in zip(original.nodes,message.nodes)):
                state['pair']=(original,message);state['captured_ros_ns']=node.get_clock().now().nanoseconds;return
node.create_subscription(NavigationGraph,'/spot1/nav_graph',raw.append,qos_profile_sensor_data)
node.create_subscription(NavigationGraph,'/spot1/scored_nav_graph',scored,qos_profile_sensor_data)
node.create_subscription(Odometry,'/spot1/odom',lambda m:state.__setitem__('odom',m),qos_profile_sensor_data)
report=dict(passed=False,scope=__doc__)
try:
    until=time.monotonic()+15.
    while time.monotonic()<until and not ('pair' in state and 'odom' in state):rclpy.spin_once(node,timeout_sec=.05)
    assert 'pair' in state and 'odom' in state
    original,message=state['pair'];combined=NavigationGraph()
    combined.header=message.header;combined.nodes=message.nodes;combined.edges=original.edges
    combined.trav_classes=message.trav_classes;combined.current_node_idx=message.current_node_idx
    files={}
    for name,obj in (('raw',original),('scored',message),('combined',combined),('odom',state['odom'])):
        data=serialize_message(obj);(args.output/(name+'.cdr')).write_bytes(data)
        (args.output/(name+'.json')).write_text(json.dumps(message_to_ordereddict(obj),indent=2,allow_nan=True)+'\n')
        files[name]=hashlib.sha256(data).hexdigest()
    report.update(passed=True,captured_ros_ns=state['captured_ros_ns'],files=files,nodes=len(combined.nodes),edges=len(combined.edges))
finally:
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
raise SystemExit(not report['passed'])
