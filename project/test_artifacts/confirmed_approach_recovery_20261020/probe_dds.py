"""Real C++ GraphNav, Bridge, GoalStandoff and ROS action handoff.

Maps, detections, stationary odometry/transport and planner/action outcomes are
synthetic. This verifies protocol connectivity, not Smac reachability or driving.
"""
from pathlib import Path
import copy
import hashlib
import importlib
import json
import os
import signal
import time
import traceback
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, PoseStamped
from graphnav_msgs.msg import NavigationGraph, Node as GraphNode, Edge, NodeTraversabilityProperties, EdgeTraversability, KeyValue, HeadingCandidate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wildos_console import OwnedProcesses
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from test_graph_path_bridge_routing import configure_ackermann, drivable_route

ROOT = Path(__file__).resolve().parent
OUT = ROOT/('dds_'+time.strftime('%Y%m%d_%H%M%S')); OUT.mkdir()
assert os.environ['ROS_DOMAIN_ID'] == '203'
owned = OwnedProcesses(OUT, ROOT); bridge = producer = None
halt = False; feed = False; all_failed = False
plans = []; follows = []; cancels = []; states = []; statuses = []; acknowledgments = []; destinations = []
report = dict(scope=__doc__, passed=False, errors=[], stages=[], complete_navigation_passed=False,
              synthetic_inputs=True, beamng_started=False, real_action_servers=True)
def interrupt(*_):
    global halt
    halt = True
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, interrupt)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node('confirmed_approach_recovery_probe')
gp = n.create_publisher(NavigationGraph, '/spot1/graphnav_planner/nav_graph', 10)
qp = n.create_publisher(String, '/spot1/object_search/query', 10)
n.create_subscription(String, '/spot1/graphnav_planner/route_status', lambda m: states.append(json.loads(m.data)), 10)
n.create_subscription(String, '/wildos/plan_status', lambda m: statuses.append(json.loads(m.data)), 10)
n.create_subscription(String, '/spot1/object_search/instance_release_ack', lambda m: acknowledgments.append(json.loads(m.data)), 10)
n.create_subscription(PoseStamped, '/spot1/graphnav_destination', lambda m: destinations.append(copy.deepcopy(m)), 10)

def graph():
    g = NavigationGraph(); g.header.frame_id = 'spot1/odom'; g.header.stamp = n.get_clock().now().to_msg()
    g.trav_classes = ['default']
    for ident, x, score in ((0, 0., 0.), (1, 14., 0.), (2, 16., 0.), (3, 40., .9)):
        v = GraphNode(); v.uuid.id[0] = ident; v.pose.position.x = x; v.pose.orientation.w = 1.
        prop = NodeTraversabilityProperties(); prop.is_frontier = ident == 3
        if prop.is_frontier: prop.frontier_points = [Point(x=x+1.)]
        v.trav_properties = [prop]; v.properties = [KeyValue(key='frontier_scores', value=[score])]
        if prop.is_frontier: v.heading_candidates = [HeadingCandidate(heading=0., predicted_utility=score, traversable=True)]
        g.nodes.append(v)
    for j in (1, 2, 3):
        g.edges.append(Edge(from_idx=0, to_idx=j, traversability=[EdgeTraversability(traversability_cost=g.nodes[j].pose.position.x)]))
    return g

async def plan(goal):
    target = goal.request.goal.pose.position
    plans.append(dict(x=target.x, y=target.y, expected=list(bridge.expected) if bridge.expected else None))
    result = ComputePathToPose.Result(); result.path.header.frame_id = 'spot1/odom'
    if not all_failed and bridge.expected == (14., 0.) and abs(target.y) < .01:
        result.path = drivable_route((0., 0.), (target.x, 0.))
        goal.succeed()
    else:
        goal.abort()
    return result

async def follow(goal):
    gate = Future(); follows.append((goal, gate))
    await gate
    if goal.is_cancel_requested: goal.canceled()
    else: goal.abort()
    return FollowPath.Result()

def cancel(goal):
    cancels.append(time.monotonic()); return CancelResponse.ACCEPT
plan_server = ActionServer(n, ComputePathToPose, '/compute_path_to_pose', plan)
follow_server = ActionServer(n, FollowPath, '/follow_path', follow, cancel_callback=cancel)

def refresh():
    stamp = n.get_clock().now().to_msg()
    odom = Odometry(); odom.header.frame_id = 'spot1/odom'; odom.header.stamp = stamp; odom.pose.pose.orientation.w = 1.
    odom_pub.publish(odom)
    if bridge:
        bridge.on_odom(copy.deepcopy(odom))
        grid.header.stamp = stamp; bridge.on_map(copy.deepcopy(grid)); bridge.on_local_map(copy.deepcopy(grid))
        bridge.on_native_status(String(data=json.dumps(dict(wall_time=time.time(), speed=0., telemetry_age_sec=0.,
            actuator_ok=True, telemetry_ok=True, control_acknowledged=True, sent={'brake': 1.}, actual={'brake': 1.}))))
        bridge.on_control_status(String(data=json.dumps(dict(stamp_ns=bridge.get_clock().now().nanoseconds,
            actuating=False, target_speed=0., target_angular=0.))))
    if feed: gp.publish(graph())
    for goal, gate in follows:
        if not gate.done() and goal.is_active: goal.publish_feedback(FollowPath.Feedback())

odom_pub = n.create_publisher(Odometry, '/spot1/odom', 10)
grid = OccupancyGrid(); grid.header.frame_id = 'spot1/odom'; grid.info.resolution = 1.
grid.info.width = grid.info.height = 160; grid.info.origin.position.x = grid.info.origin.position.y = -80.
grid.info.origin.orientation.w = 1.; grid.data = [0]*25600

def pump(seconds):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        if halt: raise InterruptedError('Interrupted')
        owned.healthy(); refresh()
        rclpy.spin_once(n, timeout_sec=.002)
        if bridge: rclpy.spin_once(bridge, timeout_sec=.002)
        if producer: rclpy.spin_once(producer, timeout_sec=.002)

def wait(test, label, seconds=8.):
    deadline = time.monotonic()+seconds
    while not test() and time.monotonic() < deadline: pump(.02)
    assert test(), (label, bridge.last_status if bridge else None, states[-1:] if states else [])
    report['stages'].append(dict(label=label, expected=bridge.expected if bridge else None,
                                planner_requests=len(plans), follow_actions=len(follows), cancel_requests=len(cancels)))

try:
    frozen = json.loads((ROOT/'frozen_source_manifest.json').read_text())['members']
    report['source_identity'] = {}
    for name in ('wildos_nav2_adapter.confirmed_approach', 'wildos_nav2_adapter.target_observation',
                 'wildos_nav2_adapter.graph_path_bridge', 'wildos_nav2_adapter.visible_approach', 'beamng_wildos_adapter.goal_standoff'):
        path = Path(importlib.import_module(name).__file__).resolve()
        relative = str(path.relative_to(ROOT/'candidate'))
        digest = hashlib.sha256(path.read_bytes()).hexdigest(); assert digest == frozen[relative]
        report['source_identity'][name] = dict(path=str(path), sha256=digest)
    exe = Path('/home/ly/frontier_absence_repair_20261020/install/graphnav_planner/lib/graphnav_planner/planner_node')
    child = owned.spawn('planner', [str(exe), '--ros-args', '-r', '__node:=graphnav_planner', '-r', '__ns:=/spot1',
        '-p', 'autonomous_exploration_enabled:=true', '-p', 'goal_radius:=1.0', '-p', 'path_smoothness_period:=0.0',
        '-r', '~/odom:=/spot1/odom', '-r', '~/goal_pose:=/spot1/graphnav_destination'])
    clear = n.create_client(Trigger, '/spot1/graphnav_planner/clear_goal')
    assert clear.wait_for_service(timeout_sec=10.)
    report['libraries'] = {}
    for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines():
        path = line.split()[-1]
        if path.startswith('/') and 'graphnav_planner' in path and '.so' in path:
            report['libraries'][path] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    assert '782102b16baa528ce5de76915d87acd9aa76151da385b857171c9e252129b470' in report['libraries'].values()
    bridge = configure_ackermann(GraphPathBridge()); producer = GoalStandoff()
    assert bridge.on_parameters([Parameter('follow_path', value=True)]).successful
    pump(.6); qp.publish(String(data='green residential house'))
    wait(lambda: bridge.query == producer.active_query == 'green residential house' and not bridge.query_reset_pending,
         'query_reset_and_producer_ready')
    # Do not let the ordinary query grace obscure the explicit-goal protocol.
    data = dict(query=bridge.query, frame='spot1/odom', observation_stamp_ns=n.get_clock().now().nanoseconds,
        localization_valid=True, search_confirmed=True, quality=dict(confidence=.9, association_validated=True),
        object_position=[20., 0., 2.], observer_position=[0., 0., 0.], standoff=[16., 0.],
        radius=.5, inliers=5, observations=5, instance_id='test-target-a')
    producer.confirmed_instance_anchor = copy.deepcopy(data)
    producer.confirmation_pub.publish(String(data=json.dumps(data)))
    wait(lambda: bridge.expected == (16., 0.), 'confirmed_object_goal_accepted')
    producer.confirmation_pub.publish(String(data=json.dumps(dict(data, standoff=[14., 0.]))))
    wait(lambda: bridge.deferred_confirmed_approach is not None, 'same_instance_farther_view_deferred')
    feed = True
    wait(lambda: bridge.expected == (14., 0.) and bridge.following, 'failed_old_view_replaced_and_action_accepted', seconds=12.)
    assert any(s['status'] == 'confirmed_approach_failed' for s in statuses)
    epoch = bridge.epoch
    producer.confirmation_pub.publish(String(data=json.dumps(dict(data, observation_stamp_ns=n.get_clock().now().nanoseconds))))
    pump(.25)
    assert bridge.expected == (14., 0.) and bridge.following and bridge.epoch == epoch and not cancels
    report['stages'].append(dict(label='fresh_old_goal_does_not_cancel_healthy_alternative'))
    for i in range(3):
        wait(lambda: len(follows) > i and bridge.following, f'controller_attempt_{i+1}_accepted', seconds=12.)
        follows[i][1].set_result(True)
        wait(lambda: not bridge.following, f'controller_attempt_{i+1}_terminal')
    all_failed = True
    wait(lambda: bool(acknowledgments), 'bounded_alternatives_released_instance_with_real_ack', seconds=18.)
    wait(lambda: bridge.expected is None and not bridge.query_reset_pending and any(
        s.get('exploration_id') and s.get('goal') == [40., 0.] for s in states), 'real_graph_planner_resumed_frontier')
    assert acknowledgments[-1].get('navigation_failed') is True
    assert producer.confirmed_instance_anchor is None and producer.navigation_region_backoff
    assert not bridge.object_found
    assert n.count_publishers('/wildos/native_control_command') == 0
    report['passed'] = True
except BaseException as exc:
    report['errors'].append(repr(exc)); report['traceback'] = traceback.format_exc()
finally:
    for _, gate in follows:
        if not gate.done(): gate.set_result(True)
    if bridge: bridge.destroy_node()
    if producer: producer.destroy_node()
    plan_server.destroy(); follow_server.destroy()
    report['errors'].extend(owned.close()); report['passed'] = report['passed'] and not report['errors']
    report.update(plans=plans, statuses=statuses, route_states=states, acknowledgments=acknowledgments)
    n.destroy_node(); rclpy.shutdown()
    (OUT/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(out=str(OUT), passed=report['passed'], errors=report['errors'], stages=report['stages']), indent=2))
raise SystemExit(not report['passed'])
