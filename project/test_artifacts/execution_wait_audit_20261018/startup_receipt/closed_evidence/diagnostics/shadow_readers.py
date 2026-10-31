"""Read-only comparison on actual source topic, never publishes controls."""
import json
import signal
import statistics
import sys
import threading
import time
from collections import Counter
from pathlib import Path
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String

output=Path(sys.argv[1]);assert not output.exists()
rclpy.init();node=rclpy.create_node('read_only_source_delivery_comparison')
rows=[];lock=threading.Lock();halt=threading.Event()
signal.signal(signal.SIGTERM,lambda *_:halt.set())
def save(kind,m):
    wall=time.time_ns();mono=time.monotonic_ns();v=json.loads(m.data)
    with lock:rows.append(dict(kind=kind,wall_ns=wall,mono_ns=mono,data=v))
for label,depth,reliability in [('reliable1',1,ReliabilityPolicy.RELIABLE),('reliable10',10,ReliabilityPolicy.RELIABLE),('best1',1,ReliabilityPolicy.BEST_EFFORT)]:
    node.create_subscription(String,'/wildos/native_control_command',lambda m,k=label:save(k,m),QoSProfile(depth=depth,reliability=reliability),callback_group=MutuallyExclusiveCallbackGroup())
node.create_subscription(String,'/wildos/native_control_status',lambda m:save('native',m),10,callback_group=MutuallyExclusiveCallbackGroup())
executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
try:
    deadline=time.monotonic()+150
    while not halt.is_set() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
finally:
    executor.shutdown()
    output.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    node.destroy_node();rclpy.shutdown()
counts=Counter(r['kind'] for r in rows);print(json.dumps(dict(counts=counts,scope=__doc__)))
