"""Actual Planner -> DDS -> Bridge with an accepted ROS FollowPath action.

Synthetic maps, odometry and healthy transport; action server does not drive a
vehicle. Tests retirement ownership and query cancellation, not physical motion.
"""
from pathlib import Path
import copy
import hashlib
import json
import os
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point
from graphnav_msgs.msg import NavigationGraph, Node as GraphNode, Edge, NodeTraversabilityProperties, EdgeTraversability, KeyValue, HeadingCandidate
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wildos_console import OwnedProcesses
import test_graph_path_bridge_routing as fixtures

ROOT=Path(__file__).resolve().parent
OUT=ROOT/('dds_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='202'
owned=OwnedProcesses(OUT,ROOT);bridge=None;halt=False;gate=Future();cancels=[];action=None;handle=None
report=dict(scope=__doc__,passed=False,errors=[],stages=[],beamng_started=False,
            real_action_server=True,synthetic_inputs=True,complete_navigation_passed=False)
def interrupt(*_):
    global halt
    halt=True
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n=Node('frontier_absence_dds_observer');states=[];paths={};retirements=[]
n.create_subscription(String,'/spot1/graphnav_planner/route_status',lambda m:states.append(json.loads(m.data)),10)
n.create_subscription(NavPath,'/spot1/graphnav_planner/path',lambda m:paths.__setitem__(m.header.stamp.sec*10**9+m.header.stamp.nanosec,m),10)
n.create_subscription(String,'/spot1/graphnav_planner/retired_exploration',lambda m:retirements.append(json.loads(m.data)),10)
gp=n.create_publisher(NavigationGraph,'/spot1/graphnav_planner/nav_graph',10)
op=n.create_publisher(Odometry,'/spot1/odom',10)
retired_pub=n.create_publisher(String,'/spot1/graphnav_planner/retired_exploration',10)
async def execute(goal):
    await gate
    if goal.is_cancel_requested:goal.canceled()
    else:goal.succeed()
    return FollowPath.Result()
def cancel(goal):
    cancels.append(time.monotonic());return CancelResponse.ACCEPT
action=ActionServer(n,FollowPath,'/follow_path',execute,goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=cancel)
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if halt:raise InterruptedError('Probe interrupted')
        owned.healthy();rclpy.spin_once(n,timeout_sec=.005)
        if bridge is not None:
            bridge.native_status_at=bridge.odom_received_at=bridge.grid_received_at=bridge.local_grid_received_at=time.monotonic()
            rclpy.spin_once(bridge,timeout_sec=.005)
def wait(future,seconds=5.):
    until=time.monotonic()+seconds
    while not future.done() and time.monotonic()<until:pump(.01)
    assert future.done(),'Response deadline';return future.result()
def graph(missing=False):
    g=NavigationGraph();g.header.frame_id='spot1/odom';g.trav_classes=['default']
    for ident,x,score in ((0,0.,0.),(1,20.,.9),(2,40.,.2)):
        if missing and ident==1:continue
        item=GraphNode();item.uuid.id[0]=ident;item.pose.position.x=x;item.pose.orientation.w=1.
        prop=NodeTraversabilityProperties();prop.is_frontier=ident!=0
        if ident:prop.frontier_points=[Point(x=x+1.)]
        item.trav_properties=[prop];item.properties=[KeyValue(key='frontier_scores',value=[score])]
        if ident:item.heading_candidates=[HeadingCandidate(heading=0.,predicted_utility=score,traversable=True)]
        g.nodes.append(item)
    for j in range(1,len(g.nodes)):
        g.edges.append(Edge(from_idx=0,to_idx=j,traversability=[EdgeTraversability(traversability_cost=g.nodes[j].pose.position.x)]))
    return g
def feed(seconds,missing=False,fixed_stamp=None):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        stamp=n.get_clock().now().to_msg();odom=Odometry();odom.header.frame_id='spot1/odom'
        odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;op.publish(odom)
        if bridge is not None:bridge.odom=odom
        g=graph(missing);g.header.stamp=fixed_stamp or stamp;gp.publish(g);pump(.04)
    assert states,'No C++ route';return states[-1]
def record(label):
    report['stages'].append(dict(label=label,state=copy.deepcopy(states[-1]),
        retirements=copy.deepcopy(retirements),cancel_requests=len(cancels)))
try:
    exe=ROOT/'install/graphnav_planner/lib/graphnav_planner/planner_node'
    child=owned.spawn('planner',[str(exe),'--ros-args','-r','__node:=graphnav_planner','-r','__ns:=/spot1',
        '-p','autonomous_exploration_enabled:=true','-p','goal_radius:=6.0','-p','path_smoothness_period:=0.0',
        '-p','missing_frontier_timeout_seconds:=0.6','-p','stale_observation_seconds:=0.2',
        '-r','~/odom:=/spot1/odom'])
    ready=n.create_client(Trigger,'/spot1/graphnav_planner/clear_goal')
    assert ready.wait_for_service(timeout_sec=10.)
    libraries={}
    for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines():
        name=line.split()[-1]
        if name.startswith('/') and 'libgraphnav_planner_component.so' in name:
            p=Path(name);assert p.is_relative_to(ROOT/'install');libraries[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    assert libraries;report['runtime_libraries']=libraries
    params=n.create_client(SetParameters,'/spot1/graphnav_planner/set_parameters')
    changed=wait(params.call_async(SetParameters.Request(parameters=[Parameter('missing_frontier_timeout_seconds',value=.1).to_parameter_msg()])))
    assert not changed.results[0].successful;report['readonly_rejection']=changed.results[0].reason
    initial=feed(.3);old_id=initial['exploration_id'];assert initial['goal']==[20.,0.]
    record('initial_frontier_A')
    assert feed(.25,True)['exploration_id']==old_id
    assert feed(.2)['exploration_id']==old_id;record('temporary_missing_returns_same_owner')
    assert feed(.3,True)['exploration_id']==old_id
    pump(.4) # greater than test-only .2s maximum acquisition gap
    assert feed(.3,True)['exploration_id']==old_id;record('acquisition_hole_breaks_absence_sequence')
    feed(.15)
    frozen_stamp=n.get_clock().now().to_msg()
    assert feed(.8,True,frozen_stamp)['exploration_id']==old_id;record('duplicate_snapshot_does_not_expire_owner')
    feed(.15)
    fixture=fixtures.GraphBridgeRoutingTests();fixture.setUp();bridge=fixture.node
    for timer in list(bridge.timers):bridge.destroy_timer(timer)
    # Planner -> Bridge metadata and retirement remain actual DDS. The observer
    # captures the path and installs it once to isolate action ownership from
    # a second planning server, absent from this protocol test.
    for subscription in list(bridge.subscriptions):
        if subscription.topic_name=='/spot1/graphnav_planner/path':bridge.destroy_subscription(subscription)
    feed(.2);initial=states[-1];path=copy.deepcopy(paths[initial['stamp_ns']])
    bridge.route=bridge.latest_route=path;bridge.route_complete=True;bridge.route_endpoint=(20.,0.)
    bridge.route_metadata[initial['stamp_ns']]=copy.deepcopy(initial);bridge.follow_enabled=True
    bridge.motion_queue=[fixtures.drivable_route((0.,0.),(12.,0.))]
    # The default disabled bridge intentionally has no action client. This
    # protocol fixture enables it explicitly without enabling its timer loop.
    bridge.follow_client=ActionClient(bridge,FollowPath,'/follow_path')
    assert bridge.follow_client.wait_for_server(timeout_sec=5.)
    handle=wait(bridge.follow_client.send_goal_async(FollowPath.Goal(path=bridge.motion_queue[0],controller_id='FollowPath',goal_checker_id='goal_checker')))
    assert handle.accepted
    terminal=handle.get_result_async();bridge.follow_handle=handle;bridge.following=True
    bridge.follow_active=dict(serial=7,handle=handle);bridge.active_follow_serial=7
    epoch=bridge.epoch
    state=feed(.95,True)
    assert state['exploration_id']!=old_id and state['goal']==[40.,0.],state
    missing=[r for r in retirements if r['exploration_id']==old_id and r['reason']=='frontier_missing']
    assert len(missing)==1,retirements
    assert bridge.retired_motion and bridge.retired_motion['identity']==old_id,bridge.last_status
    assert bridge.following and bridge.follow_handle is handle and bridge.epoch==epoch
    assert not cancels and not terminal.done();record('persistent_absence_retires_intention_keeps_accepted_action')
    bridge.on_query(String(data='new search'))
    pump(.3);assert cancels,'Query change failed to cancel accepted action'
    assert bridge.retired_motion is None and bridge.follow_hold
    count=len(cancels);retired_pub.publish(String(data=json.dumps(missing[0])));pump(.1)
    assert len(cancels)==count;record('query_change_revokes_motion_late_retirement_is_fenced')
    gate.set_result(None);assert wait(terminal).status==5
    report.update(passed=True,accepted_actions=1,cancel_requests=len(cancels),old_action_terminal_status=5)
except BaseException as error:
    report['errors'].append(repr(error))
finally:
    halt=False
    if not gate.done():gate.set_result(None)
    if bridge is not None:
        try:pump(.15)
        except Exception as error:report['errors'].append('Drain: '+repr(error))
        bridge.destroy_node()
    action.destroy();n.destroy_node();rclpy.shutdown()
    report['errors'].extend(owned.close());report['passed']=report['passed'] and not report['errors']
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),**report),indent=2))
raise SystemExit(not report['passed'])
