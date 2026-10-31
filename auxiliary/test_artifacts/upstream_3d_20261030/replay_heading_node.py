"""Real PlannerNode comparison using recorded candidate attributes and isolated topology."""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph, Edge
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

parser=argparse.ArgumentParser();parser.add_argument('--library-root',required=True);parser.add_argument('--output',required=True);parser.add_argument('--full-recorded',action='store_true')
args=parser.parse_args();out=Path(args.output);out.mkdir(exist_ok=False,parents=True)
fixture=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s/full/trial_20261030_095616/sessions/session_20261030_095616/graph_pairs')
raw=deserialize_message((fixture/'pair_000181_raw.cdr').read_bytes(),NavigationGraph)
scores=deserialize_message((fixture/'pair_000181_scored.cdr').read_bytes(),NavigationGraph)
odom=deserialize_message((fixture/'pair_000181_odom.cdr').read_bytes(),Odometry)
stamp=raw.header.stamp.sec*10**9+raw.header.stamp.nanosec+100_000_000
rclpy.init();node=Node('heading_freshness_replay')
clock=node.create_publisher(Clock,'/clock',1)
graphs=node.create_publisher(NavigationGraph,'/planner_node/nav_graph',1)
poses=node.create_publisher(Odometry,'/planner_node/odom',1)
statuses=[]
node.create_subscription(String,'/planner_node/route_status',lambda m:statuses.append(json.loads(m.data)),10)
results=[]
for case in ('stale_blocked','fresh_blocked','fresh_allowed'):
    graph=deepcopy(raw)
    origin=deepcopy(scores.nodes[raw.current_node_idx]);origin.pose=deepcopy(odom.pose.pose)
    origin.robot_traversability=[];origin.heading_candidates=[]
    for prop in origin.trav_properties:prop.is_frontier=False
    road=deepcopy(scores.nodes[45]);forest=deepcopy(scores.nodes[48])
    if case!='stale_blocked':
        for a in road.robot_traversability:
            a.observed_at.sec,a.observed_at.nanosec=divmod(stamp,10**9)
            if case=='fresh_allowed':a.feasible=True;a.physical_traversability=1.
        if case=='fresh_allowed':
            for h in road.heading_candidates:h.traversable=True;h.predicted_utility=h.frontier_score
    graph.nodes=[origin,road,forest];graph.current_node_idx=0;graph.edges=[]
    for destination in (1,2):
        e=Edge();e.from_idx=0;e.to_idx=destination
        e.traversability=deepcopy(raw.edges[0].traversability)
        e.traversability[0].traversability_cost=math.hypot(graph.nodes[destination].pose.position.x-origin.pose.position.x,graph.nodes[destination].pose.position.y-origin.pose.position.y)
        graph.edges.append(e)
    if args.full_recorded:
        graph=deepcopy(raw);graph.nodes=deepcopy(scores.nodes)
        graph.nodes[45]=road;graph.nodes[48]=forest
    log=(out/(case+'.log')).open('w');env=os.environ.copy()
    env['LD_LIBRARY_PATH']=str(Path(args.library_root)/'install/graphnav_planner/lib')+':'+env.get('LD_LIBRARY_PATH','')
    child=subprocess.Popen([str(Path(args.library_root)/'install/graphnav_planner/lib/graphnav_planner/planner_node'),
        '--ros-args','-p','use_sim_time:=true','-p','autonomous_exploration_enabled:=true','-p','path_smoothness_period:=0.0'],
        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    statuses.clear()
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not statuses:
            c=Clock();c.clock.sec,c.clock.nanosec=divmod(stamp,10**9);clock.publish(c)
            odom.header.stamp=c.clock;poses.publish(odom)
            rclpy.spin_once(node,timeout_sec=.05)
            if graphs.get_subscription_count():graphs.publish(graph)
        assert statuses,'No route status from actual PlannerNode'
        position=statuses[0]['goal'];road_xy=[road.pose.position.x,road.pose.position.y]
        results.append(dict(case=case,goal=position,selected_road=math.dist(position[:2],road_xy)<.01))
    finally:
        os.killpg(child.pid,signal.SIGINT);child.wait(timeout=8);log.close()
        for _ in range(4):rclpy.spin_once(node,timeout_sec=.02)
node.destroy_node();rclpy.shutdown()
report=dict(scope=('Actual full recorded topology, scores and node attributes; no runtime history/backoff replay.' if args.full_recorded else 'Real PlannerNode, measured candidate attributes, controlled 3-node star topology; not a claim of full recorded-map reachability.'),results=results)
(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
