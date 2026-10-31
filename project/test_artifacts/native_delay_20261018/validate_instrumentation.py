"""Actual Humble timer/subscription diagnostic interface regression; no vehicle."""
import json
import os
from pathlib import Path
import importlib.util
import sys
import time
import rclpy
from std_msgs.msg import String
root=Path(__file__).parent
output=root/'instrumentation_validation.jsonl'
assert not output.exists()
sys.argv=['native_vehicle_control']
os.environ['WILDOS_NATIVE_TIMING_FILE']=str(output)
spec=importlib.util.spec_from_file_location('native_timing_diagnostic',root/'instrumentation/sitecustomize.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
from rclpy.executors import MultiThreadedExecutor
assert MultiThreadedExecutor.__name__=='TimingExecutor'
rclpy.init();node=rclpy.create_node('native_timing_interface_regression')
received=[];ticks=[]
pub=node.create_publisher(String,'/wildos/native_control_command',10)
node.create_subscription(String,'/wildos/native_control_command',lambda m:received.append(m.data),1)
def tick():
    ticks.append(time.monotonic())
    pub.publish(String(data=json.dumps(dict(issued_wall_ns=time.time_ns(),sequence=len(ticks),sender_id='diagnostic',generation=0))))
node.create_timer(.02,tick)
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
try:
    until=time.monotonic()+.7
    while time.monotonic()<until:executor.spin_once(timeout_sec=.01)
finally:
    executor.shutdown();node.destroy_node();rclpy.shutdown()
assert len(received)>=5 and len(ticks)>=5
rows=[json.loads(l) for l in output.read_text().splitlines()]
messages=[r for r in rows if 'issued_wall_ns' in r]
assert messages and any(r.get('kind')=='timer' for r in rows)
assert all(r['take_mono_ns']>=r['handle_entered_mono_ns']>=r['take_begin_mono_ns']>=r['ready_mono_ns'] for r in messages)
print(json.dumps(dict(passed=True,timers=len(ticks),messages=len(received),metadata_and_ready_boundaries=True)))
