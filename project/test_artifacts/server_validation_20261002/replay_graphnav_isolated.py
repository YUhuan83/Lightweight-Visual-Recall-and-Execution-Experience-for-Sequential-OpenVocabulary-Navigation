"""Replay saved graph/odom into a planner in domain 53; no vehicle/control nodes."""
from pathlib import Path
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env(domain=53)).returncode)
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Odometry,Path as Route
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph

folder=ROOT/'house_geometry_rviz_attempt2'
dest=ROOT/'failure_audit/house_geometry_rviz_attempt2'
audit=json.loads((dest/'audit.json').read_text());base=audit['trial_base_wall_time']
db=sqlite3.connect('file:'+str(next((folder/'rosbag').glob('*.db3')))+'?mode=ro',uri=True)
topics={name:(ident,get_message(kind)) for ident,name,kind in db.execute('select id,name,type from topics')}
def before(topic,t):
    ident,kind=topics[topic];row=db.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(ident,int(t*1e9))).fetchone()
    return deserialize_message(row[0],kind)
args=Path('/proc/2197084/cmdline').read_bytes().decode().strip('\0').split('\0')
# Exact deployed GraphNav arguments; domain isolation changes DDS scope only.
logfile=(dest/'graphnav_replay.log').open('w')
process=subprocess.Popen(args,stdout=logfile,stderr=subprocess.STDOUT)
try:
    rclpy.init();node=rclpy.create_node('failure_graphnav_replay')
    graphpub=node.create_publisher(NavigationGraph,'/spot1/scored_nav_graph',10)
    odompub=node.create_publisher(Odometry,'/spot1/odom',10)
    goalpub=node.create_publisher(PoseStamped,'/spot1/graphnav_destination',10)
    received=[]
    node.create_subscription(Route,'/spot1/graphnav_planner/path',lambda m:received.append(m),10)
    end=time.monotonic()+5
    while (graphpub.get_subscription_count()==0 or goalpub.get_subscription_count()==0) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if graphpub.get_subscription_count()==0 or goalpub.get_subscription_count()==0:raise RuntimeError('Isolated planner did not subscribe')
    result={'domain':53,'vehicle_control_nodes_started':False,'snapshots':[]}
    for at in (100.77,123.,155.):
        received.clear();graph=before('/spot1/scored_nav_graph',base+at);odom=before('/spot1/odom',base+at)
        goal=PoseStamped();goal.header.frame_id='spot1/odom';goal.header.stamp=node.get_clock().now().to_msg()
        goal.pose.position.x=736.3732348304133;goal.pose.position.y=-12.646839773953522;goal.pose.position.z=52.09686991681215;goal.pose.orientation.w=1.
        # Warm graph and odom, then publish the explicit goal exactly once,
        # as the production bridge does. Discovery is already verified.
        odompub.publish(odom);graphpub.publish(graph)
        warm=time.monotonic()+.5
        while time.monotonic()<warm:rclpy.spin_once(node,timeout_sec=.05)
        received.clear();goalpub.publish(goal)
        end=time.monotonic()+2.5;last=0.
        while time.monotonic()<end:
            if time.monotonic()-last>.25:
                odompub.publish(odom);graphpub.publish(graph);last=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.05)
        p=received[-1] if received else None
        endpoints=[tuple([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z]) for m in received for ps in m.poses[-1:]]
        result['snapshots'].append({'elapsed':at,'goal_subscribers':goalpub.get_subscription_count(),
             'publishers': [{'node':i.node_name,'namespace':i.node_namespace} for i in node.get_publishers_info_by_topic('/spot1/graphnav_planner/path')],
             'distinct_endpoints': sorted(set(endpoints)),
             'ordered_endpoints':endpoints,
             'navigation_distance':((odom.pose.pose.position.x-goal.pose.position.x)**2+(odom.pose.pose.position.y-goal.pose.position.y)**2)**.5,
             'goal': [goal.pose.position.x,goal.pose.position.y,goal.pose.position.z],
             'output_count':len(received),'nodes':len(graph.nodes),'current_node_idx':graph.current_node_idx,
             'path':[] if p is None else [[ps.pose.position.x,ps.pose.position.y,ps.pose.position.z] for ps in p.poses],
             'frame':None if p is None else p.header.frame_id})
    (dest/'graphnav_replay.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=3)
    except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)
    logfile.close()
    if rclpy.ok():rclpy.shutdown()
