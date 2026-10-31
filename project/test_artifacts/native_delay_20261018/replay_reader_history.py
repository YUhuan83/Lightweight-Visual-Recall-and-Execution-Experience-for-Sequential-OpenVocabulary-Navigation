"""Deterministic unread-history control: two actual DDS readers, no SDK or vehicle."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys,time
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String

parser=argparse.ArgumentParser();parser.add_argument('--publisher',action='store_true');args=parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='184'
rclpy.init();node=rclpy.create_node('history_publisher' if args.publisher else 'history_reader')
topic='/diagnostic_native_unread_history'
if args.publisher:
    pub=node.create_publisher(String,topic,10)
    for seq in range(120):
        pub.publish(String(data=json.dumps(dict(sequence=seq,issued_ns=time.time_ns()))));time.sleep(.05)
    node.destroy_node();rclpy.shutdown();raise SystemExit()
subs={kind:node.create_subscription(String,topic,lambda m:None,QoSProfile(depth=1,reliability=policy))
      for kind,policy in [('reliable',ReliabilityPolicy.RELIABLE),('best_effort',ReliabilityPolicy.BEST_EFFORT)]}
pub=subprocess.Popen([sys.executable,__file__,'--publisher'])
rows=[]
try:
    time.sleep(1.5)
    for phase in range(3):
        # A bounded consumer pause reproduces a delayed subscription take;
        # transport remains running, neither callback renews any source age.
        time.sleep(.4)
        until=time.monotonic()+.6
        first=set()
        while time.monotonic()<until:
            for kind,sub in subs.items():
                with sub.handle:packet=sub.handle.take_message(String,False)
                if packet:
                    value=json.loads(packet[0].data);wall=time.time_ns()
                    rows.append(dict(phase=phase,kind=kind,first=kind not in first,age_ms=(wall-value['issued_ns'])/1e6,sequence=value['sequence'],metadata=packet[1]));first.add(kind)
            time.sleep(.01)
    pub.wait(timeout=4)
finally:
    if pub.poll() is None:pub.terminate();pub.wait(timeout=2)
    node.destroy_node();rclpy.shutdown()
root=Path(__file__).parent
(root/'history_replay.json').write_text(json.dumps(dict(scope=__doc__,rows=rows),indent=2)+'\n')
print(json.dumps([r for r in rows if r['first']],indent=2))
