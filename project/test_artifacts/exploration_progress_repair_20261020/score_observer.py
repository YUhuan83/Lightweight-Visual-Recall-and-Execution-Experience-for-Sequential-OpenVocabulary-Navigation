"""Low-rate, read-only graph/decision observer; no features, paths or control mutations."""
from pathlib import Path
import collections
import json
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from std_msgs.msg import String

rclpy.init();node=Node('field_score_decision_observer');out=Path(sys.argv[1]).open('x')
state={};last={};counts=collections.Counter()
def receive(name,msg):
    counts[name]+=1
    if name=='plan':
        try:state['plan']=json.loads(msg.data)
        except ValueError:return
    elif name=='odom':state['odom']=[msg.pose.pose.position.x,msg.pose.pose.position.y]
    else:
        now=time.monotonic()
        if now-last.get(name,0.)<1.:return
        began=time.monotonic();last[name]=now
        index=msg.trav_classes.index('default') if 'default' in msg.trav_classes else None
        front=[n for n in msg.nodes if index is not None and index<len(n.trav_properties) and n.trav_properties[index].is_frontier]
        plan=state.get('plan',{});goal=plan.get('exploration_goal') or plan.get('goal')
        selected=min(front,key=lambda n:math.dist(goal,[n.pose.position.x,n.pose.position.y])) if front and goal else None
        def summary(n):
            return dict(uuid=bytes(n.uuid.id).hex(),position=[n.pose.position.x,n.pose.position.y],
                properties={p.key:list(p.value) for p in n.properties if p.key in ('frontier_scores','memory_prior_similarity','physical_violation_observed')},
                headings=[dict(heading=h.heading,score=h.frontier_score,utility=h.predicted_utility,
                    traversable=h.traversable,physical=h.physical_traversability,risk=h.dynamic_risk) for h in n.heading_candidates])
        row=dict(kind=name,wall_ns=time.time_ns(),source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                 nodes=len(msg.nodes),frontiers=len(front),eligible_headings=sum(h.traversable for n in front for h in n.heading_candidates),
                 selected=summary(selected) if selected else None,plan=plan,odom=state.get('odom'),counts=dict(counts),
                 observer_ms=(time.monotonic()-began)*1000)
        out.write(json.dumps(row,allow_nan=False)+'\n');out.flush()
def observe(name,message):
    try:receive(name,message)
    except Exception as error:
        out.write(json.dumps(dict(kind='observer_error',topic=name,error=repr(error),wall_ns=time.time_ns()))+'\n');out.flush()
for cls,topic,name in ((NavigationGraph,'/spot1/nav_graph','raw'),(NavigationGraph,'/spot1/scored_nav_graph','scored'),
                       (String,'/wildos/plan_status','plan'),(Odometry,'/spot1/odom','odom')):
    node.create_subscription(cls,topic,lambda m,k=name:observe(k,m),qos_profile_sensor_data)
try:rclpy.spin(node)
finally:
    out.close();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
