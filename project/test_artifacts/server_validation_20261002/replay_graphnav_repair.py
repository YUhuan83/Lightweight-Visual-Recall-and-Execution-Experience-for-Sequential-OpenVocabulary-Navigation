"""Saved field graph regression in isolated DDS domain 53, no control nodes."""
from pathlib import Path
import json
import math
import signal
import sqlite3
import subprocess
import sys
import time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, '--runtime-child'],
                                   env=runtime_env(domain=53)).returncode)
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Odometry, Path as Route
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from std_srvs.srv import Trigger

out = ROOT/'navigation_repair'
out.mkdir(exist_ok=True)
audit = json.loads((ROOT/'failure_audit/house_geometry_rviz_attempt2/audit.json').read_text())
base = audit['trial_base_wall_time']
db = sqlite3.connect('file:'+str(next((ROOT/'house_geometry_rviz_attempt2/rosbag').glob('*.db3')))+'?mode=ro', uri=True)
topics = {name: (ident, get_message(kind)) for ident, name, kind in db.execute('select id,name,type from topics')}
def before(topic, at):
    ident, kind = topics[topic]
    row = db.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',
                     (ident, int((base+at)*1e9))).fetchone()
    return deserialize_message(row[0], kind)
pidfile = ROOT/'graphnav_planner.pid'
pid = int(pidfile.read_text()) if pidfile.exists() else 2197084
args = Path('/proc', str(pid), 'cmdline').read_bytes().decode().strip('\0').split('\0')
log = (out/'graphnav_replay.log').open('w')
process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
node = None
try:
    rclpy.init()
    node = rclpy.create_node('navigation_repair_graphnav_replay')
    graphs = node.create_publisher(NavigationGraph, '/spot1/scored_nav_graph', 10)
    odoms = node.create_publisher(Odometry, '/spot1/odom', 10)
    goals = node.create_publisher(PoseStamped, '/spot1/graphnav_destination', 10)
    clear = node.create_client(Trigger, '/spot1/graphnav_planner/clear_goal')
    received = []
    node.create_subscription(Route, '/spot1/graphnav_planner/path', received.append, 10)
    end = time.monotonic()+5
    while time.monotonic() < end and (not graphs.get_subscription_count() or not goals.get_subscription_count()):
        rclpy.spin_once(node, timeout_sec=.1)
    assert graphs.get_subscription_count() == goals.get_subscription_count() == 1
    assert clear.wait_for_service(timeout_sec=3)
    def spin(duration, graph=None, odom=None):
        end = time.monotonic()+duration
        last = 0.
        while time.monotonic() < end:
            if graph is not None and time.monotonic()-last > .25:
                odoms.publish(odom)
                graphs.publish(graph)
                last = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.05)
    goal = PoseStamped()
    goal.header.frame_id = 'spot1/odom'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 736.3732348304133, -12.646839773953522, 52.09686991681215
    goal.pose.orientation.w = 1.
    target = [goal.pose.position.x, goal.pose.position.y]
    result = dict(domain=53, vehicle_control_nodes_started=False, goal_publish_count=1, snapshots=[])
    for index, at in enumerate((100.77, 123., 155.)):
        graph, odom = before('/spot1/scored_nav_graph', at), before('/spot1/odom', at)
        if index == 0:
            spin(.7, graph, odom)
            received.clear()
            goals.publish(goal)
        else:
            received.clear()
        spin(2.5, graph, odom)
        endpoints = [[m.poses[-1].pose.position.x, m.poses[-1].pose.position.y] for m in received if m.poses]
        assert endpoints and all(math.dist(p, target) < .01 for p in endpoints), (at, endpoints)
        result['snapshots'].append(dict(elapsed=at, outputs=len(endpoints), endpoint=endpoints[-1], goal_retained=True))
    # Verify the explicit clear service still restores autonomous exploration.
    future = clear.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    assert future.done() and future.result().success
    received.clear()
    spin(1.5, graph, odom)
    endpoints = [[m.poses[-1].pose.position.x, m.poses[-1].pose.position.y] for m in received if m.poses]
    assert endpoints and all(math.dist(p, target) > 7. for p in endpoints), endpoints
    result.update(clear_goal_restores_exploration=True, passed=True,
                  exploration_endpoint=endpoints[-1])
    (out/'graphnav_replay.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3)
    log.close()
    db.close()
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
