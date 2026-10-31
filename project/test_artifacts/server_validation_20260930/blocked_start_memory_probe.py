"""Verify retained memory on the real blocked costmap, with simulation paused."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

out=Path('/home/ly/full_town_memory_20260930')
pid=int(out.joinpath('graph_bridge.pid').read_text())
args=Path('/proc',str(pid),'cmdline').read_bytes().decode().split('\0')[:-1]
env=dict(e.split('=',1) for e in Path('/proc',str(pid),'environ').read_bytes().decode().split('\0') if '=' in e)
os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+8
while Path('/proc',str(pid)).exists() and time.monotonic()<deadline:time.sleep(.1)
if Path('/proc',str(pid)).exists():raise RuntimeError('Old graph bridge did not stop')
with out.joinpath('graph_bridge.log').open('ab') as f:
    p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
out.joinpath('graph_bridge.pid').write_text(str(p.pid))
rclpy.init()
n=Node('blocked_start_memory_probe')
report={'scope':'paused, actual blocked vehicle pose and actual remembered station', 'statuses':[], 'selections':[]}
def status(m):
    value=json.loads(m.data)
    if not report['statuses'] or value!=report['statuses'][-1]:report['statuses'].append(value)
n.create_subscription(String,'/wildos/plan_status',status,10)
n.create_subscription(String,'/wildos/memory_selection',lambda m:report['selections'].append(json.loads(m.data)),10)
pub=n.create_publisher(String,'/spot1/object_search/query',10)
start=time.monotonic()
last=0.
while time.monotonic()-start<18:
    rclpy.spin_once(n,timeout_sec=.05)
    if time.monotonic()-start>2 and time.monotonic()-last>5:
        pub.publish(String(data='gas station'))
        last=time.monotonic()
report['elapsed']=time.monotonic()-start
report['passed']=bool(report['statuses']) and report['statuses'][-1].get('status')=='start_footprint_blocked' and report['statuses'][-1].get('source')=='memory_via_graphnav' and not any(s.get('status')=='memory_unreachable' for s in report['selections'])
out.joinpath('blocked_start_memory_probe.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
if not report['passed']:raise RuntimeError('Blocked start discarded the memory target')
