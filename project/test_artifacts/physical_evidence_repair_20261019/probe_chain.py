"""Actual Mapper CDR -> DDS -> actual Planner; no sensors/actions/vehicle."""
from pathlib import Path
import hashlib,json,os,re,signal,sys,time,traceback
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from graphnav_msgs.msg import NavigationGraph
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix
from wildos_console import OwnedProcesses,proc_identity

ROOT=Path(__file__).resolve().parent
label=sys.argv[1];assert re.fullmatch(r'baseline|candidate[0-9]+',label)
assert os.environ['ROS_DOMAIN_ID']=='198'
OUT=ROOT/(label+'_dds_v2_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
owned=OwnedProcesses(OUT,ROOT);halt=False
def interrupt(*_):
 global halt
 halt=True
for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(s,interrupt)
rclpy.init();n=Node('physical_evidence_chain_probe');states=[]
n.create_subscription(String,'/spot1/graphnav_planner/route_status',lambda m:states.append(json.loads(m.data)),10)
gp=n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
op=n.create_publisher(Odometry,'/spot1/odom',10)
goal_pub=n.create_publisher(PoseStamped,'/spot1/graphnav_destination',10)
clock_pub=n.create_publisher(Clock,'/clock',10)
ready=n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
report=dict(scope=__doc__,label=label,passed=False,errors=[],stages=[],vehicle_started=False,complete_navigation_passed=False)
def pump(seconds,graph=None):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  if halt:raise InterruptedError('Interrupted')
  owned.healthy()
  if graph is not None:
   c=Clock();c.clock=graph.header.stamp;clock_pub.publish(c)
   odom=Odometry();odom.header=graph.header;odom.pose.pose=graph.nodes[graph.current_node_idx].pose
   odom.pose.pose.orientation.w=1.;op.publish(odom);gp.publish(graph)
  rclpy.spin_once(n,timeout_sec=.03)
try:
 prefix=Path(get_package_prefix('graphnav_planner'));expected=ROOT/label/'install/graphnav_planner'
 assert prefix==expected,(prefix,expected)
 exe=prefix/'lib/graphnav_planner/planner_node'
 child=owned.spawn('planner',[str(exe),'--ros-args','-p','use_sim_time:=true','-p','goal_radius:=0.5',
   '-p','autonomous_exploration_enabled:=false','-r','__ns:=/spot1','-r','__node:=graphnav_planner',
   '-r','~/odom:=/spot1/odom','-r','~/goal_pose:=/spot1/graphnav_destination'])
 assert ready.wait_for_service(timeout_sec=10.)
 pump(.4)
 libraries={line.split()[-1] for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines() if 'libgraphnav_planner_component.so' in line}
 assert libraries
 wanted=json.loads((ROOT/label/'result.json').read_text())['libraries']['install/graphnav_planner/lib/libgraphnav_planner_component.so']
 report['planner_identity']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in libraries}
 assert all(Path(p).is_relative_to(expected) and digest==wanted for p,digest in report['planner_identity'].items())
 exported=ROOT/label/'fixtures_v2'
 mapper_paths={line.split()[-1] for line in (exported/'runtime_maps.txt').read_text().splitlines() if 'libgraphnav_mapper_component.so' in line}
 report['mapper_identity']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in mapper_paths}
 wanted_mapper=json.loads((ROOT/label/'result.json').read_text())['libraries']['install/graphnav_mapper/lib/libgraphnav_mapper_component.so']
 assert mapper_paths and all(Path(p).is_relative_to(ROOT/label/'install/graphnav_mapper') and digest==wanted_mapper for p,digest in report['mapper_identity'].items())
 names=['partial_steep','unknown_after_steep','flat_after_steep','unmeasured_flat','separated_flats','complete_steep','stale_steep']
 expected_states=[True]*7 if label=='baseline' else [False,False,True,True,True,False,True]
 for index,(name,complete) in enumerate(zip(names,expected_states)):
  path=exported/(name+'.cdr');blob=path.read_bytes();graph=deserialize_message(blob,NavigationGraph)
  assert len(graph.nodes)==2 and graph.header.frame_id=='map'
  destination=next(i for i,node in enumerate(graph.nodes) if abs(node.pose.position.x-6.)<1e-6)
  forward=[e for e in graph.edges if e.from_idx==graph.current_node_idx and e.to_idx==destination]
  assert len(forward)==1 and any(p.key=='traversed' and list(p.value)==[1.] for p in forward[0].traversability[0].properties)
  pump(.25,graph)
  if index==0:
   goal=PoseStamped();goal.header=graph.header;goal.pose.position.x=6.;goal.pose.position.y=2.;goal.pose.orientation.w=1.
   goal_pub.publish(goal);pump(.15,graph)
  start=len(states);deadline=time.monotonic()+5.
  while time.monotonic()<deadline:
   pump(.1,graph)
   matches=[s for s in states[start:] if s['stamp_ns']==graph.header.stamp.sec*10**9+graph.header.stamp.nanosec and s['complete']==complete]
   if matches:break
  else:raise AssertionError((name,complete,states[start:]))
  report['stages'].append(dict(label=name,graph_sha256=hashlib.sha256(blob).hexdigest(),state=matches[-1],
       edges=[dict(confidence=e.robot_traversability[0].confidence,observed_at=[e.robot_traversability[0].observed_at.sec,e.robot_traversability[0].observed_at.nanosec],
                   traversability=e.robot_traversability[0].physical_traversability,
                   properties={p.key:list(p.value) for p in e.properties}) for e in graph.edges]))
 assert n.count_publishers('/wildos/native_control_command')==0
 report['passed']=True
except BaseException:report['errors'].append(traceback.format_exc())
finally:
 report['errors'].extend(owned.close());report['passed']=report['passed'] and not report['errors']
 report['route_states']=states;n.destroy_node();rclpy.shutdown()
 (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k!='route_states'},indent=2))
raise SystemExit(not report['passed'])
