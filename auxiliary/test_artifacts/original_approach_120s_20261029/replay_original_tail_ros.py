"""Exercise the real PlannerNode with a recorded graph, no simulator/control."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Path as PathMsg, Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    fixture = Path(args.fixture)
    records = [json.loads(line) for line in (fixture/'snapshots.jsonl').read_text().splitlines()]
    pairs = [r for r in records if r.get('kind') == 'pair' and all(r.get('contract', {}).values())]
    pair = min(pairs, key=lambda r: abs(r['ros_ns']-1793260837000000000))
    raw = deserialize_message((fixture/pair['raw']['file']).read_bytes(), NavigationGraph)
    scores = deserialize_message((fixture/pair['scored']['file']).read_bytes(), NavigationGraph)
    odom = deserialize_message((fixture/pair['odom']['file']).read_bytes(), Odometry)
    current_ns = pair['ros_ns']
    rclpy.init()
    node = Node('original_tail_replay')
    clocks = node.create_publisher(Clock, '/clock', 1)
    topology = node.create_publisher(NavigationGraph, '/spot1/nav_graph', 1)
    graph_pub = node.create_publisher(NavigationGraph, '/planner_node/nav_graph', 1)
    odom_pub = node.create_publisher(Odometry, '/planner_node/odom', 1)
    goals = node.create_publisher(PoseStamped, '/planner_node/goal_pose', 1)
    paths, statuses = [], []
    node.create_subscription(PathMsg, '/planner_node/path', paths.append, 10)
    node.create_subscription(String, '/planner_node/route_status',
                             lambda m: statuses.append(json.loads(m.data)), 10)
    log = (output/'node.log').open('w')
    child = subprocess.Popen([args.executable, '--ros-args',
        '-p', 'use_sim_time:=true', '-p', 'use_compact_score_overlay:=true',
        '-p', 'goal_radius:=6.0', '-p', 'path_smoothness_period:=0.0'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = dict(passed=False, errors=[], fixture=pair, no_simulator=True)

    def pump(seconds, predicate=lambda: False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert child.poll() is None, 'PlannerNode exited'
            c = Clock()
            c.clock.sec, c.clock.nanosec = divmod(current_ns, 10**9)
            clocks.publish(c)
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return True
        return predicate()

    def request(position, label):
        nonlocal current_ns
        current_ns += 100_000_000
        paths.clear()
        statuses.clear()
        p = PoseStamped()
        p.header.frame_id = raw.header.frame_id
        p.header.stamp.sec, p.header.stamp.nanosec = divmod(current_ns, 10**9)
        p.pose.position.x, p.pose.position.y, p.pose.position.z = position
        p.pose.orientation.w = 1.
        pump(.1)
        topology.publish(raw)
        pump(.1)
        odom_pub.publish(odom)
        goals.publish(p)
        for _ in range(5):
            graph_pub.publish(scores)
            pump(.15)
            if paths and statuses:
                break
        assert paths and statuses, 'No paired path/status from actual node'
        path = paths[-1]
        status = statuses[-1]
        end = path.poses[-1].pose.position if path.poses else None
        terminal = [end.x, end.y, end.z] if end else None
        (output/(label+'.cdr')).write_bytes(serialize_message(path))
        return dict(request=position, endpoint=terminal,
                    endpoint_matches=terminal is not None and
                        max(abs(a-b) for a,b in zip(terminal, position)) < 1.e-6,
                    complete=status['complete'], poses=len(path.poses))

    try:
        assert pump(8., lambda: goals.get_subscription_count() > 0
                    and graph_pub.get_subscription_count() > 0), 'Discovery timeout'
        pump(.5)
        far = request([750.654, -130.083, 56.994], 'unknown_path')
        report['unknown_destination'] = far
        current = odom.pose.pose.position
        known = request([current.x, current.y, current.z], 'known_path')
        report['known_destination'] = known
        report['passed'] = (far['endpoint_matches'] and not far['complete']
                            and known['endpoint_matches'] and known['complete'])
    except Exception as exc:
        report['errors'].append(repr(exc))
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        report['node_exit'] = child.returncode
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        (output/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k:v for k,v in report.items() if k != 'fixture'}, indent=2))
    return report['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--fixture', required=True)
    parser.add_argument('--output', required=True)
    raise SystemExit(0 if run(parser.parse_args()) else 1)
