"""Real DDS/GraphNav lifecycle tests. Synthetic graph/odom; no vehicle acceptance."""
import copy,json,subprocess,time,unittest,os
import rclpy
from rclpy.node import Node
from graphnav_msgs.msg import NavigationGraph,Node as GraphNode,Edge,NodeTraversabilityProperties,EdgeTraversability,KeyValue,HeadingCandidate
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.msg import Path,Odometry,OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger

class OwnerContract(unittest.TestCase):
 @classmethod
 def setUpClass(cls): rclpy.init()
 @classmethod
 def tearDownClass(cls): rclpy.shutdown()
 def setUp(self):
  self.log=open('/home/ly/frontier_motion_handoff_20261016/node_'+self._testMethodName+'.log','w')
  self.proc=subprocess.Popen(['ros2','run','graphnav_planner','planner_node','--ros-args','-p','autonomous_exploration_enabled:=true','-p','goal_radius:=6.0','-r','__ns:=/spot1','-r','__node:=graphnav_planner','-r','~/odom:=/spot1/odom'],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
  self.n=Node('owner_contract_observer');self.status=[];self.paths={};self.retirements=[]
  self.n.create_subscription(String,'/spot1/graphnav_planner/route_status',lambda x:self.status.append(json.loads(x.data)),10)
  self.n.create_subscription(String,'/spot1/graphnav_planner/retired_exploration',lambda x:self.retirements.append(json.loads(x.data)),10)
  self.n.create_subscription(Path,'/spot1/graphnav_planner/path',lambda x:self.paths.__setitem__(x.header.stamp.sec*10**9+x.header.stamp.nanosec,x),10)
  self.graphpub=self.n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
  self.odompub=self.n.create_publisher(Odometry,'/spot1/odom',10)
  self.ackpub=self.n.create_publisher(PoseStamped,'/spot1/graphnav_planner/observed_route_endpoint',10)
  self.rejectpub=self.n.create_publisher(PoseStamped,'/spot1/graphnav_planner/rejected_route_endpoint',10)
  self.goalpub=self.n.create_publisher(PoseStamped,'/spot1/graphnav_planner/goal_pose',10)
  self.clear=self.n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
  self.assertTrue(self.clear.wait_for_service(timeout_sec=15))
  self.pump(.7)
 def tearDown(self):
  import signal
  os.killpg(self.proc.pid,signal.SIGINT)
  try:self.proc.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(self.proc.pid,signal.SIGTERM);self.proc.wait(timeout=5)
  self.log.close()
  if getattr(self,'bridge',None) is not None:self.bridge.destroy_node()
  self.n.destroy_node()
 def pump(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   rclpy.spin_once(self.n,timeout_sec=.01)
   if getattr(self,'bridge',None) is not None:rclpy.spin_once(self.bridge,timeout_sec=.01)
 def graph(self,sa=.9,sb=.2,origin=0.,missing=False):
  g=NavigationGraph();g.header.frame_id='spot1/odom';g.header.stamp=self.n.get_clock().now().to_msg();g.trav_classes=['default']
  for ident,x,score in [(0,origin,0.),(1,20.,sa),(2,40.,sb)]:
   if missing and ident==1:continue
   n=GraphNode();n.uuid.id[0]=ident;n.pose.position.x=x;n.pose.orientation.w=1.
   prop=NodeTraversabilityProperties();prop.is_frontier=ident!=0;prop.frontier_points=[Point(x=x+1.)];n.trav_properties=[prop]
   n.properties=[KeyValue(key='frontier_scores',value=[float(score)])]
   if ident:n.heading_candidates=[HeadingCandidate(heading=0.,predicted_utility=float(score),traversable=True)]
   g.nodes.append(n)
  for j in range(1,len(g.nodes)):
   g.edges.append(Edge(from_idx=0,to_idx=j,traversability=[EdgeTraversability(traversability_cost=abs(g.nodes[j].pose.position.x-origin))]))
  return g
 def odom(self,x,stamp=None):
  o=Odometry();o.header.frame_id='spot1/odom';o.header.stamp=stamp or self.n.get_clock().now().to_msg();o.pose.pose.position.x=x;o.pose.pose.orientation.w=1.;return o
 def update(self,g,seconds=.35,x=0.,stamp=None):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   self.odompub.publish(self.odom(x,stamp));g.header.stamp=self.n.get_clock().now().to_msg();self.graphpub.publish(g);self.pump(.08)
  self.assertTrue(self.status);return self.status[-1]
 def endpoint(self,status):
  self.pump(.1);p=self.paths[status['stamp_ns']];m=PoseStamped();m.header=p.header;m.pose=p.poses[-1].pose;return m
 def test_score_change_missing_frontier_and_unacknowledged_nearby_pose_keep_identity(self):
  a=self.update(self.graph());self.assertEqual(a['goal'],[20.,0.]);identity=a['exploration_id']
  b=self.update(self.graph(.1,1.));self.assertEqual(b['exploration_id'],identity)
  b=self.update(self.graph(.1,1.,missing=True));self.assertEqual(b['exploration_id'],identity)
  b=self.update(self.graph(.1,1.,origin=20.),seconds=1.8,x=20.);self.assertEqual(b['exploration_id'],identity)
 def test_ack_stale_odometry_and_fresh_settled_completion(self):
  a=self.update(self.graph());m=self.endpoint(a);self.ackpub.publish(m);self.pump(.15)
  stamp=self.n.get_clock().now().to_msg();self.pump(.65)
  b=self.update(self.graph(.1,1.,origin=20.),seconds=1.8,x=20.,stamp=stamp)
  self.assertEqual(b['exploration_id'],a['exploration_id']);self.assertTrue(b['endpoint_observed'])
  b=self.update(self.graph(.1,1.,origin=20.),seconds=1.9,x=20.)
  self.assertNotEqual(b['exploration_id'],a['exploration_id']);self.assertEqual(b['goal'],[40.,0.])
 def test_query_clear_and_old_refusal_do_not_erase_new_owner(self):
  a=self.update(self.graph());old=self.endpoint(a)
  f=self.clear.call_async(Trigger.Request());rclpy.spin_until_future_complete(self.n,f,timeout_sec=3);self.assertTrue(f.result().success)
  b=self.update(self.graph(.1,1.));self.assertEqual(b['goal'],[40.,0.])
  self.rejectpub.publish(old);self.ackpub.publish(old);self.pump(.15)
  c=self.update(self.graph(.1,1.));self.assertEqual(c['exploration_id'],b['exploration_id']);self.assertFalse(c['endpoint_observed'])
 def test_correlated_refusal_releases_owner_without_completed_observation(self):
  a=self.update(self.graph());self.rejectpub.publish(self.endpoint(a));self.pump(.2)
  b=self.update(self.graph(.1,1.));self.assertEqual(b['goal'],[40.,0.]);self.assertNotEqual(b['exploration_id'],a['exploration_id'])
  self.assertFalse(b['endpoint_observed'])
  self.assertTrue(any(x['exploration_id']==a['exploration_id'] and x['reason']=='vehicle_route_rejected' for x in self.retirements))

 def test_real_bridge_observation_round_trip_retires_same_graph_owner(self):
  from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
  self.bridge=GraphPathBridge();self.bridge.follow_enabled=True
  # No actuator/action server in this DDS test. Only the stopped observation seam.
  self.bridge.execution_transport_issue=lambda:None
  gp=self.n.create_publisher(OccupancyGrid,'/global_costmap/costmap',10)
  lp=self.n.create_publisher(OccupancyGrid,'/local_costmap/costmap',10)
  self.pump(.5)
  grid=OccupancyGrid();grid.header.frame_id='spot1/odom';grid.info.resolution=1.;grid.info.width=100;grid.info.height=100;grid.info.origin.position.x=-30.;grid.info.origin.position.y=-50.;grid.info.origin.orientation.w=1.;grid.data=[0]*10000
  end=time.monotonic()+3.8
  while time.monotonic()<end:
   grid.header.stamp=self.n.get_clock().now().to_msg();gp.publish(grid);lp.publish(grid)
   self.odompub.publish(self.odom(20. if self.status else 0.));self.graphpub.publish(self.graph(origin=0.));self.pump(.08)
  identities=[x['exploration_id'] for x in self.status]
  self.assertGreaterEqual(len(set(identities)),2)
  self.assertTrue(any(x['reason']=='observation_completed' for x in self.retirements))
  first=identities[0]
  self.assertTrue(any(x['exploration_id']==first and x['endpoint_observed'] for x in self.status))
  self.assertTrue(any(x['exploration_id']!=first and x['goal']==[40.,0.] for x in self.status))

if __name__=='__main__':unittest.main(verbosity=2)
