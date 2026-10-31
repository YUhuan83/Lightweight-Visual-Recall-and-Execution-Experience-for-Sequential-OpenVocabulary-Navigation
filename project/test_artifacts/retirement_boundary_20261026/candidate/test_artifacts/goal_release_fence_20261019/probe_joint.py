"""Real Bridge/C++/DDS goal handoffs over a synthetic graph; no vehicle/actions."""
from pathlib import Path
import copy,hashlib,importlib,json,os,signal,time
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
from server_test_preflight import goal_reset_service_errors
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/('dds_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='195'
owned=OwnedProcesses(OUT,ROOT);halt=False;bridge=None
def interrupt(*_):
    global halt
    halt=True
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n=Node('joint_goal_release_observer');states=[];paths={};retirements=[]
report=dict(scope=__doc__,passed=False,errors=[],beamng_started=False,production_deployment=False,
    complete_navigation_passed=False,stages=[])
n.create_subscription(String,'/spot1/graphnav_planner/route_status',lambda m:states.append(json.loads(m.data)),10)
n.create_subscription(NavPath,'/spot1/graphnav_planner/path',lambda m:paths.__setitem__(m.header.stamp.sec*10**9+m.header.stamp.nanosec,m),10)
n.create_subscription(String,'/spot1/graphnav_planner/retired_exploration',lambda m:retirements.append(json.loads(m.data)),10)
gp=n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
op=n.create_publisher(Odometry,'/spot1/odom',10)
rp=n.create_publisher(PoseStamped,'/spot1/graphnav_planner/rejected_route_endpoint',10)
ap=n.create_publisher(PoseStamped,'/spot1/graphnav_planner/observed_route_endpoint',10)
clear=n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
release=n.create_client(Trigger,'/spot1/graphnav_planner/release_goal')
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if halt:raise InterruptedError('Interrupted')
        owned.healthy();rclpy.spin_once(n,timeout_sec=.01)
        if bridge is not None:rclpy.spin_once(bridge,timeout_sec=.01)
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
def wait_barrier(operation):
    deadline=time.monotonic()+4.
    while bridge.query_reset_pending and time.monotonic()<deadline:pump(.02)
    assert not bridge.query_reset_pending,(operation,bridge.last_status)
    assert not bridge.graph_reset_waiting,(operation,'undrained requests')
    report.setdefault('completed_barriers',[]).append(dict(operation=operation,serial=bridge.graph_reset_serial))
def semantic_release():
    bridge.request_graph_reset();bridge.retry_graph_reset()
    assert not bridge.graph_reset_search
    wait_barrier('release_goal')
try:
    manifest=json.loads((ROOT/'candidate_delta.json').read_text())
    report['python_identity']={}
    for name in ('navigation_transactions','graph_path_bridge','execution_recovery','visible_approach'):
        path=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__).resolve()
        relative='nav2_adapter/wildos_nav2_adapter/'+name+'.py'
        assert path==ROOT/'candidate'/relative
        sha=hashlib.sha256(path.read_bytes()).hexdigest();assert sha==manifest['members'][relative]
        report['python_identity'][name]=dict(path=str(path),sha256=sha)
    exe=Path(get_package_prefix('graphnav_planner'))/'lib/graphnav_planner/planner_node'
    assert exe.is_relative_to(ROOT/'install'),str(exe)
    child=owned.spawn('planner',[str(exe),'--ros-args','-p','autonomous_exploration_enabled:=true',
        '-p','goal_radius:=6.0','-r','__ns:=/spot1','-r','__node:=graphnav_planner',
        '-r','~/odom:=/spot1/odom','-r','~/goal_pose:=/spot1/graphnav_destination'])
    assert clear.wait_for_service(timeout_sec=10.) and release.wait_for_service(timeout_sec=3.)
    pump(.5);services=n.get_service_names_and_types();assert not goal_reset_service_errors(services),services
    report['reset_services']=services
    libraries={}
    for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines():
        name=line.split()[-1]
        if name.startswith('/') and '.so' in name and 'graphnav_planner' in name:
            path=Path(name);assert path.is_relative_to(ROOT/'install'),name
            libraries[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    assert libraries,'No runtime planner library identity'
    report['runtime_identity']=dict(executable=str(exe),executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),
        process=proc_identity(child.pid),libraries=libraries,ros_domain=195)
    observed_goals=[]
    n.create_subscription(PoseStamped,'/spot1/graphnav_destination',lambda m:observed_goals.append(copy.deepcopy(m)),10)
    bridge=GraphPathBridge();assert not bridge.follow_enabled
    pump(.6)
    assert n.count_publishers('/wildos/native_control_command')==0
    bridge.on_query(String(data='audit first target'));wait_barrier('initial_search_reset')
    a=update('initial_A');assert a['goal']==[20.,0.],a
    pump(.1);path=paths[a['stamp_ns']]
    refusal=PoseStamped();refusal.header=copy.deepcopy(path.header);refusal.pose=copy.deepcopy(path.poses[-1].pose)
    refused_at=time.monotonic();rp.publish(refusal);pump(.2)
    b=update('refuse_A_choose_B');assert b['goal']==[40.,0.],b
    semantic_release()
    c=update('same_query_release_retains_backoff');assert c['goal']==[40.,0.],c
    assert c['exploration_id']!=b['exploration_id']
    rp.publish(refusal);ap.publish(refusal);pump(.2)
    d=update('old_refusal_ack_cannot_reject_new_owner')
    assert d['exploration_id']==c['exploration_id'] and not d['endpoint_observed'],d
    goal=PoseStamped();goal.header.frame_id='spot1/odom';goal.pose.orientation.w=1.;goal.pose.position.x=40.
    bridge.relay_destination(goal);pump(.2)
    explicit=update('bridge_published_explicit_goal')
    assert explicit['goal']==[40.,0.] and 'exploration_id' not in explicit,explicit
    semantic_release()
    e=update('release_after_explicit_goal_retains_backoff');assert e['goal']==[40.,0.],e
    bridge.on_query(String(data='audit second target'));wait_barrier('new_query_search_reset')
    f=update('new_query_can_select_A_again');assert f['goal']==[20.,0.],f
    assert f['exploration_id'] not in (a['exploration_id'],c['exploration_id'],e['exploration_id'])
    report['elapsed_since_refusal_sec']=time.monotonic()-refused_at
    assert report['elapsed_since_refusal_sec']<15.,'Backoff TTL could have expired'
    assert n.count_publishers('/wildos/native_control_command')==0
    assert observed_goals,'No actual goal sample'
    old_goal=observed_goals[0]
    bridge.waypoint_pub.publish(old_goal);pump(.2)
    late=update('old_goal_after_full_reset_is_ignored')
    assert late['goal']==[20.,0.] and late['exploration_id']==f['exploration_id'],late
    bridge.relay_destination(goal);pump(.2)
    fresh_b=update('fresh_goal_after_full_reset_is_accepted')
    assert fresh_b['goal']==[40.,0.] and 'exploration_id' not in fresh_b,fresh_b
    older_b=copy.deepcopy(observed_goals[-1])
    newer_a=copy.deepcopy(goal);newer_a.pose.position.x=20.
    bridge.relay_destination(newer_a);pump(.2)
    accepted_a=update('newer_explicit_goal_is_accepted')
    assert accepted_a['goal']==[20.,0.] and 'exploration_id' not in accepted_a,accepted_a
    bridge.waypoint_pub.publish(older_b);pump(.2)
    reordered=update('older_goal_in_same_phase_is_ignored')
    assert reordered['goal']==[20.,0.],reordered
    future_goal=copy.deepcopy(older_b);future_goal.header.stamp=n.get_clock().now().to_msg()
    future_goal.header.stamp.sec+=3600
    bridge.waypoint_pub.publish(future_goal);pump(.2)
    future_state=update('future_goal_is_ignored')
    assert future_state['goal']==[20.,0.],future_state
    bridge.relay_destination(goal);pump(.2)
    after_invalid=update('fresh_goal_after_invalid_future_is_accepted')
    assert after_invalid['goal']==[40.,0.] and 'exploration_id' not in after_invalid,after_invalid
    accepted_b=copy.deepcopy(observed_goals[-1])
    semantic_release()
    released=update('release_reopens_exploration')
    assert released['goal']==[20.,0.] and released['exploration_id'],released
    bridge.waypoint_pub.publish(accepted_b);pump(.2)
    late_release=update('old_goal_after_release_is_ignored')
    assert late_release['exploration_id']==released['exploration_id'],late_release
    report['full_elapsed_since_refusal_sec']=time.monotonic()-refused_at
    report['passed']=True
except BaseException as exc:report['errors'].append(repr(exc))
finally:
    if bridge is not None:bridge.destroy_node()
    report['errors'].extend(owned.close());report['passed']=report['passed'] and not report['errors']
    report['route_states']=states;report['retirements']=retirements
    n.destroy_node();rclpy.shutdown()
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('route_states','retirements','reset_services')},indent=2))
raise SystemExit(not report['passed'])
