"""Real bridge publisher -> real graph planner: refuse one exit, retain object goal."""
import os,json,time,subprocess
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='167'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.msg import Path as PathMsg,Odometry
from std_msgs.msg import String
from graphnav_msgs.msg import NavigationGraph,Node as GraphNode,Edge,NodeTraversabilityProperties,EdgeTraversability
import test_graph_path_bridge_routing as fixture
root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback')
exe=root/'install/graphnav_planner/lib/graphnav_planner/planner_node'
log=(root/'feedback_node.log').open('w')
child=subprocess.Popen([str(exe),'--ros-args','-r','__node:=feedback_planner','-p','goal_radius:=0.5','-p','autonomous_exploration_enabled:=false','-p','use_compact_score_overlay:=false'],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();node=Node('feedback_verifier');bridge=None
try:
 paths=[];statuses={}
 node.create_subscription(PathMsg,'/feedback_planner/path',paths.append,10)
 node.create_subscription(String,'/feedback_planner/route_status',lambda m:statuses.update({json.loads(m.data)['stamp_ns']:json.loads(m.data)}),10)
 graph_pub=node.create_publisher(NavigationGraph,'/feedback_planner/nav_graph',10)
 goal_pub=node.create_publisher(PoseStamped,'/feedback_planner/goal_pose',10)
 odom_pub=node.create_publisher(Odometry,'/feedback_planner/odom',10)
 refusal_pub=node.create_publisher(PoseStamped,'/feedback_planner/rejected_route_endpoint',10)
 graph=NavigationGraph();graph.header.frame_id='spot1/odom';graph.header.stamp=node.get_clock().now().to_msg();graph.trav_classes=['default'];graph.current_node_idx=0
 for x in (0.,2.):
  n=GraphNode();n.pose.position.x=x;n.pose.orientation.w=1.;n.trav_properties=[NodeTraversabilityProperties()];graph.nodes.append(n)
 graph.nodes[1].trav_properties[0].is_frontier=True
 graph.nodes[1].trav_properties[0].frontier_points=[Point(x=3.),Point(x=4.)]
 edge=Edge();edge.from_idx=0;edge.to_idx=1;edge.traversability=[EdgeTraversability()];edge.traversability[0].traversability_cost=2.;graph.edges=[edge]
 target=PoseStamped();target.header=graph.header;target.pose.position.x=20.;target.pose.orientation.w=1.
 odom=Odometry();odom.header.frame_id='spot1/odom';odom.pose.pose.orientation.w=1.
 def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
 def wait_exit(x,publish_goal):
  until=time.monotonic()+10.
  while time.monotonic()<until:
   assert child.poll() is None,'planner exited'
   odom.header.stamp=node.get_clock().now().to_msg();odom_pub.publish(odom)
   graph_pub.publish(graph)
   if publish_goal:goal_pub.publish(target)
   rclpy.spin_once(node,timeout_sec=.1)
   p=next((p for p in reversed(paths) if p.poses and abs(p.poses[-1].pose.position.x-x)<1e-6 and stamp(p) in statuses),None)
   if p:return p
  raise AssertionError(f'No route to exit {x}')
 first=wait_exit(4.,True)
 f=fixture.GraphBridgeRoutingTests();f.setUp();bridge=f.node
 bridge.follow_enabled=True;bridge.upstream_target_approach=True;bridge.expected=(20.,0.)
 bridge.route=first;bridge.latest_route=None;bridge.route_metadata[stamp(first)]=statuses[stamp(first)]
 bridge.route_rejection_pub=refusal_pub
 assert bridge.reject_exploration_route(),'bridge did not report failed partial route'
 alternative=wait_exit(3.,False)
 status=statuses[stamp(alternative)]
 assert status['goal']==[20.,0.] and status['mode']=='frontier' and not status['complete'],status
 assert bridge.expected==(20.,0.)
 (root/'feedback_integration.json').write_text(json.dumps(dict(passed=True,first_exit=4.,replacement_exit=3.,retained_goal=status['goal'],complete=False,scope='real ROS bridge failure publisher to real graph planner; no car execution'),indent=2))
 print('PASS: failed exit 4 -> exit 3; object goal remains 20')
finally:
 if bridge is not None:bridge.destroy_node()
 node.destroy_node();rclpy.shutdown();child.terminate()
 try:child.wait(timeout=10)
 except subprocess.TimeoutExpired:child.kill();child.wait(timeout=10)
 log.close()
