"""Read-only exact source/scored snapshots; independent of current action ownership."""
from pathlib import Path
import collections
import hashlib
import json
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from std_msgs.msg import String

root = Path(sys.argv[1]); root.mkdir(exist_ok=False)
rclpy.init(); node = Node('readonly_exact_graph_pair_observer')
cache = collections.deque(maxlen=128); state = {}; counts = collections.Counter()
last = {}; stream = (root/'snapshots.jsonl').open('x')

def key(g):
    return (g.header.frame_id, g.header.stamp.sec, g.header.stamp.nanosec)

def save(name, message):
    data = serialize_message(message)
    (root/name).write_bytes(data)
    return dict(file=name, sha256=hashlib.sha256(data).hexdigest(), size=len(data))

def raw(g):
    cache.append(g); counts['raw'] += 1
    # Also retain unscored geometry so score absence is observable.
    now = time.monotonic()
    if now-last.get('raw', 0.) < 5.: return
    last['raw'] = now
    row = dict(kind='raw', wall_ns=time.time_ns(), ros_ns=node.get_clock().now().nanoseconds,
               key=key(g), nodes=len(g.nodes), edges=len(g.edges), current_node_idx=g.current_node_idx,
               raw=save('raw_%06d.cdr'%counts['raw'], g))
    stream.write(json.dumps(row)+'\n'); stream.flush()

def scored(g):
    counts['scored'] += 1
    now = time.monotonic()
    if now-last.get('scored', 0.) < 1.: return
    last['scored'] = now
    original = next((s for s in reversed(cache) if key(s)==key(g)), None)
    contract = dict(source_found=original is not None, scored_edges_empty=not g.edges)
    if original is not None:
        contract.update(classes=original.trav_classes==g.trav_classes,
                        current_node=original.current_node_idx==g.current_node_idx,
                        node_count=len(original.nodes)==len(g.nodes),
                        nodes=all(a.uuid==b.uuid and a.pose==b.pose for a,b in zip(original.nodes,g.nodes)))
    seq=counts['scored']
    row=dict(kind='pair', wall_ns=time.time_ns(), ros_ns=node.get_clock().now().nanoseconds,
             key=key(g), contract=contract, plan=state.get('plan'), counts=dict(counts),
             scored=save('pair_%06d_scored.cdr'%seq, g))
    if original is not None: row['raw']=save('pair_%06d_raw.cdr'%seq, original)
    if 'odom' in state: row['odom']=save('pair_%06d_odom.cdr'%seq, state['odom'])
    stream.write(json.dumps(row,allow_nan=False)+'\n'); stream.flush()

def plan(m):
    state['plan']=json.loads(m.data)

def safe(callback, message):
    try: callback(message)
    except Exception as error:
        stream.write(json.dumps(dict(kind='error', wall_ns=time.time_ns(), error=repr(error)))+'\n');stream.flush()

for cls,topic,callback in ((NavigationGraph,'/spot1/nav_graph',raw),
                           (NavigationGraph,'/spot1/scored_nav_graph',scored),
                           (Odometry,'/spot1/odom',lambda m:state.__setitem__('odom',m)),
                           (String,'/wildos/plan_status',plan)):
    node.create_subscription(cls,topic,lambda m,f=callback:safe(f,m),qos_profile_sensor_data)
try: rclpy.spin(node)
finally:
    stream.close(); node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
