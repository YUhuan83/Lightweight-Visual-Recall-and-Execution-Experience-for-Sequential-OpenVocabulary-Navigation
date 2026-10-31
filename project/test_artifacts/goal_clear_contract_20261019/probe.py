"""Actual C++/DDS counterexample for clearing destination versus search history."""
from pathlib import Path
import copy
import hashlib
import json
import os
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point,PoseStamped
from graphnav_msgs.msg import NavigationGraph,Node as GraphNode,Edge,NodeTraversabilityProperties,EdgeTraversability,KeyValue,HeadingCandidate
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wildos_console import OwnedProcesses,proc_identity

ROOT=Path(__file__).resolve().parent
OUT=ROOT/('probe_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='194'
owned=OwnedProcesses(OUT,ROOT);halt=False
def interrupt(*_):
    global halt
    halt=True
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n=Node('clear_contract_audit');states=[];paths={};retirements=[]
report=dict(scope=__doc__,gap_reproduced=False,errors=[],beamng_started=False,
    production_modified=False,repair_verified=False,complete_navigation_passed=False,stages=[])
def receive_state(msg):states.append(json.loads(msg.data))
def receive_path(msg):paths[msg.header.stamp.sec*10**9+msg.header.stamp.nanosec]=msg
n.create_subscription(String,'/spot1/graphnav_planner/route_status',receive_state,10)
n.create_subscription(NavPath,'/spot1/graphnav_planner/path',receive_path,10)
n.create_subscription(String,'/spot1/graphnav_planner/retired_exploration',lambda m:retirements.append(json.loads(m.data)),10)
gp=n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
op=n.create_publisher(Odometry,'/spot1/odom',10)
rp=n.create_publisher(PoseStamped,'/spot1/graphnav_planner/rejected_route_endpoint',10)
clear=n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if halt:raise InterruptedError('Interrupted')
        owned.healthy();rclpy.spin_once(n,timeout_sec=.02)
def graph():
    g=NavigationGraph();g.header.frame_id='spot1/odom';g.trav_classes=['default']
    for ident,x,score in ((0,0.,0.),(1,20.,.9),(2,40.,.2)):
        item=GraphNode();item.uuid.id[0]=ident;item.pose.position.x=x;item.pose.orientation.w=1.
        prop=NodeTraversabilityProperties();prop.is_frontier=ident!=0
        if ident:prop.frontier_points=[Point(x=x+1.)]
        item.trav_properties=[prop];item.properties=[KeyValue(key='frontier_scores',value=[score])]
        if ident:item.heading_candidates=[HeadingCandidate(heading=0.,predicted_utility=score,traversable=True)]
        g.nodes.append(item)
    for j in (1,2):g.edges.append(Edge(from_idx=0,to_idx=j,traversability=[EdgeTraversability(traversability_cost=float(j*20))]))
    return g
def update(label,seconds=.6):
    count=len(states);g=graph();until=time.monotonic()+seconds
    while time.monotonic()<until:
        stamp=n.get_clock().now().to_msg()
        odom=Odometry();odom.header.frame_id='spot1/odom';odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.
        op.publish(odom);g.header.stamp=stamp;gp.publish(g);pump(.08)
    assert len(states)>count,('No new state',label)
    state=states[-1];report['stages'].append(dict(label=label,state=copy.deepcopy(state)))
    return state
try:
    assert n.count_publishers('/wildos/native_control_command')==0
    exe=Path(get_package_prefix('graphnav_planner'))/'lib/graphnav_planner/planner_node'
    child=owned.spawn('planner',[str(exe),'--ros-args','-p','autonomous_exploration_enabled:=true',
        '-p','goal_radius:=6.0','-r','__ns:=/spot1','-r','__node:=graphnav_planner','-r','~/odom:=/spot1/odom'])
    assert clear.wait_for_service(timeout_sec=10.),'No actual clear service'
    pump(.6)
    libraries={}
    for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines():
        name=line.split()[-1]
        if name.startswith('/') and '.so' in name and 'graphnav_planner' in name:
            p=Path(name);libraries[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    assert 'db52f00e06d7c598ecba3e8c42d1b34019693cd879f03537a5a36e9f6e1c6b8f' in libraries.values(),libraries
    report['runtime_identity']=dict(executable=str(exe),executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),
        process=proc_identity(child.pid),libraries=libraries,ros_domain=194)
    a=update('initial_A');assert a['goal']==[20.,0.],a
    stamp=a['stamp_ns'];pump(.1);assert stamp in paths
    path=paths[stamp];refusal=PoseStamped();refusal.header=copy.deepcopy(path.header);refusal.pose=copy.deepcopy(path.poses[-1].pose)
    refused_at=time.monotonic();rp.publish(refusal);pump(.2)
    b=update('refuse_A_choose_B');assert b['goal']==[40.,0.],b
    assert b['exploration_id']!=a['exploration_id']
    before=time.monotonic();future=clear.call_async(Trigger.Request())
    while not future.done() and time.monotonic()-before<3.:pump(.02)
    assert future.done() and future.result().success,'Clear did not complete'
    report['clear_response']=dict(success=future.result().success,message=future.result().message)
    c=update('same_graph_after_clear');report['elapsed_since_refusal_sec']=time.monotonic()-refused_at
    assert c['goal']==[20.,0.],c
    assert c['exploration_id'] not in (a['exploration_id'],b['exploration_id'])
    assert report['elapsed_since_refusal_sec']<10.,'Not a bounded pre-expiry reproduction'
    assert any(m['exploration_id']==a['exploration_id'] and m['reason']=='vehicle_route_rejected' for m in retirements)
    report['gap_reproduced']=True
except BaseException as exc:report['errors'].append(repr(exc))
finally:
    report['errors'].extend(owned.close())
    report['gap_reproduced']=report['gap_reproduced'] and not report['errors']
    report['route_states']=states;report['retirements']=retirements
    n.destroy_node();rclpy.shutdown()
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('route_states','retirements')},indent=2))
raise SystemExit(not report['gap_reproduced'])
