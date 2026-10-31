"""Real store/candidate -> DDS -> C++ exploration decisions; no model/actions/vehicle."""
from pathlib import Path
import ast, hashlib, importlib, json, os, signal, time, traceback
from types import SimpleNamespace as S
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from graphnav_msgs.msg import NavigationGraph,Node as GraphNode,Edge,NodeTraversabilityProperties,EdgeTraversability,KeyValue,HeadingCandidate,RobotNodeTraversability
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wildos_console import OwnedProcesses,proc_identity
from visual_navigation.wildos.graph_observation_store import GraphObservationStore
from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore,fuse_execution_prior

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/('decision_dds_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='196'
owned=OwnedProcesses(OUT,ROOT)
rclpy.init();n=Node('recall_decision_probe');states=[]
n.create_subscription(String,'/spot1/graphnav_planner/route_status',lambda m:states.append(json.loads(m.data)),10)
gp=n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
op=n.create_publisher(Odometry,'/spot1/odom',10)
clear=n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
store=GraphObservationStore(':memory:','test');store.register_map('map')
experience=ExecutionExperienceStore(':memory:','test',map_id='map')
report=dict(scope=__doc__,passed=False,errors=[],stages=[],vehicle_started=False,complete_navigation_passed=False)
halt=False
def interrupt(*_):
 global halt
 halt=True
for sig in (signal.SIGHUP,signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupt)
source=ROOT/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
owner=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef))
method=next(x for x in owner.body if isinstance(x,ast.FunctionDef) and x.name=='_append_heading_candidates')
ns=dict(np=np,HeadingCandidate=HeadingCandidate)
exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
nav=S(visual_memory_map=store,object_search_mode=True,text_queries=['target'],memory_map_id='map',
 uuid_to_str=lambda u:str(u.id[0]),global_frame='spot1/odom',heading_low_confidence_weight=0.,
 heading_turn_cost_weight=0.,heading_dynamic_risk_weight=0.,object_memory_heading_weight=0.,search_coverage_weight=.2)

def graph(*,query='target',use_experience=False,blocked_b=False):
 nav.text_queries=[query]
 g=NavigationGraph();g.header.frame_id='spot1/odom';g.trav_classes=['default'];g.current_node_idx=0
 for ident,x,score,bin_index in ((0,0.,0.,0),(1,20.,.8,0),(2,-20.,.75,8)):
  item=GraphNode();item.uuid.id[0]=ident;item.pose.position.x=x;item.pose.orientation.w=1.
  prop=NodeTraversabilityProperties();prop.is_frontier=ident!=0
  if ident:prop.frontier_points=[Point(x=x+(1. if x>0 else -1.))]
  item.trav_properties=[prop]
  assessment=RobotNodeTraversability(robot_profile_id='default',feasible=not(blocked_b and ident==2),
       physical_traversability=1.,minimum_clearance=2.,dynamic_risk=0.,confidence=1.)
  assessment.observed_at=n.get_clock().now().to_msg();item.robot_traversability=[assessment]
  scores=np.zeros(16,dtype=np.float32);scores[bin_index]=score
  if use_experience:
   prior,_=experience.retrieve_execution_prior([1.,0.],16,current_heading=0.,heading_frame='spot1/odom')
   assert prior is not None
   scores=fuse_execution_prior(scores,prior,.5)
  item.properties=[KeyValue(key='frontier_scores',value=[float(s) for s in scores])]
  if ident:ns['_append_heading_candidates'](nav,item,scores,0.)
  g.nodes.append(item)
 for j in (1,2):g.edges.append(Edge(from_idx=0,to_idx=j,traversability=[EdgeTraversability(traversability_cost=20.)]))
 return g

def pump(seconds,g=None):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  if halt:raise InterruptedError('Interrupted')
  owned.healthy()
  if g is not None:
   g.header.stamp=n.get_clock().now().to_msg()
   for node in g.nodes:
    for a in node.robot_traversability:a.observed_at=g.header.stamp
   gp.publish(g)
   o=Odometry();o.header=g.header;o.child_frame_id='spot1/base_link';o.pose.pose.orientation.w=1.;op.publish(o)
  rclpy.spin_once(n,timeout_sec=.02)

def decision(label,expected,**kwargs):
 g=graph(**kwargs);pump(.25,g)
 future=clear.call_async(Trigger.Request());deadline=time.monotonic()+3.
 while not future.done() and time.monotonic()<deadline:pump(.03,g)
 assert future.done() and future.result().success,label
 start=len(states);deadline=time.monotonic()+5.
 while time.monotonic()<deadline:
  pump(.1,g)
  found=[v for v in states[start:] if v.get('goal')==[expected,0.] and v.get('exploration_id')]
  if found:break
 else:raise AssertionError((label,states[start:]))
 row=dict(label=label,state=found[-1],heading_utilities=[[c.predicted_utility for c in node.heading_candidates] for node in g.nodes])
 report['stages'].append(row)

try:
 identity=json.loads((ROOT/'candidate_delta.json').read_text())['members']
 report['source_identity']={}
 for name in ('graph_observation_store','execution_experience_store','execution_scene_samples','search_evidence'):
  p=Path(importlib.import_module('visual_navigation.wildos.'+name).__file__).resolve()
  relative='wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/'+name+'.py'
  assert p==ROOT/'candidate'/relative
  digest=hashlib.sha256(p.read_bytes()).hexdigest();assert digest==identity[relative]
  report['source_identity'][name]=digest
 exe=Path(get_package_prefix('graphnav_planner'))/'lib/graphnav_planner/planner_node'
 assert str(exe).startswith('/home/ly/goal_release_fence_20261019/install/')
 child=owned.spawn('planner',[str(exe),'--ros-args','-p','autonomous_exploration_enabled:=true','-p','goal_radius:=6.0',
   '-r','__ns:=/spot1','-r','__node:=graphnav_planner','-r','~/odom:=/spot1/odom','-r','~/goal_pose:=/spot1/graphnav_destination'])
 assert clear.wait_for_service(timeout_sec=10.)
 pump(.4)
 libraries={line.split()[-1] for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines() if 'libgraphnav_planner_component.so' in line}
 assert libraries,'Expected actual planner component library is absent'
 report['runtime_identity']=dict(process=proc_identity(child.pid),libraries={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in libraries})
 assert all(v=='86fecc865b6fd6a13a7f0b23f8336b8c4af5c370c95bd877232bc956e6aeba02' for v in report['runtime_identity']['libraries'].values())
 assert n.count_publishers('/wildos/native_control_command')==0
 decision('baseline_selects_A',20.)
 store.record_search_coverage('map','target','1',[0],1.)
 store.record_search_coverage('map','target','2',[0],1.)
 store.record_search_coverage('map','target','2',[8],.1)
 decision('per_heading_coverage_changes_selection_to_B',-20.)
 decision('new_query_does_not_inherit_negative_evidence',20.,query='other')
 experience.record_feedback([1.,0.],None,np.pi,1.,1,heading_count=16,scene_heading=0.,scene_frame='spot1/odom')
 decision('measured_execution_prior_changes_preference',-20.,query='other',use_experience=True)
 decision('memory_cannot_override_physical_rejection',20.,query='other',use_experience=True,blocked_b=True)
 report['passed']=True
except BaseException as exc:report['errors'].append(traceback.format_exc())
finally:
 report['errors'].extend(owned.close());report['passed']=report['passed'] and not report['errors']
 report['route_states']=states
 store.close();experience.close();n.destroy_node();rclpy.shutdown()
 (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k!='route_states'},indent=2))
raise SystemExit(not report['passed'])
