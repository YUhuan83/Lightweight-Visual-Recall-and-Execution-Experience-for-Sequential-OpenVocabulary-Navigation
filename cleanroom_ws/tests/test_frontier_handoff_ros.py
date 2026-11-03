"""Public-topic test of real GraphNav -> bridge -> Nav2 failure handoff."""
import json
from copy import deepcopy
import os
import subprocess
import tempfile
import threading
import time
import unittest

try:
    import rclpy
except ImportError:
    rclpy = None


def frontier_graph(stamp, first_exit=(7., 0.)):
    from geometry_msgs.msg import Point
    from graphnav_msgs.msg import NavigationGraph, Node, Edge, EdgeTraversability, NodeTraversabilityProperties, KeyValue
    g = NavigationGraph()
    g.header.frame_id, g.header.stamp = "spot1/odom", stamp
    g.trav_classes = ["default"]
    for i, (x, y) in enumerate(((0., 0.), (5., 0.), (0., 5.))):
        n = Node()
        n.uuid.id = [i + 1] + [0] * 15
        n.pose.position.x, n.pose.position.y, n.pose.position.z = x, y, 52.
        n.pose.orientation.w = 1.
        trav = NodeTraversabilityProperties(free_radius=5., explored_radius=5.)
        if i:
            trav.is_frontier = True
            ex, ey = first_exit if i == 1 else (0., 7.)
            trav.frontier_points = [Point(x=ex, y=ey, z=52.)]
            n.properties = [KeyValue(key="frontier_scores", value=[.9 if i == 1 else .5] * 36)]
        n.trav_properties = [trav]
        g.nodes.append(n)
        if i:
            g.edges.append(Edge(from_idx=0, to_idx=i, traversability=[EdgeTraversability(traversability_cost=5.)]))
    return g


@unittest.skipIf(rclpy is None, "Requires ROS host and built GraphNav")
class FrontierHandoffTests(unittest.TestCase):
    def test_removed_exit_is_replaced_even_when_its_node_stays_frontier(self):
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path
        from graphnav_msgs.msg import NavigationGraph
        from rclpy.node import Node

        rclpy.init()
        io = Node("frontier_membership_io")
        endpoints = []
        io.create_subscription(Path, "/spot1/graphnav_planner/path",
            lambda m: endpoints.append((m.poses[-1].pose.position.x, m.poses[-1].pose.position.y)) if m.poses else None, 10)
        raw = io.create_publisher(NavigationGraph, "/spot1/nav_graph", 10)
        score = io.create_publisher(NavigationGraph, "/spot1/scored_nav_graph", 10)
        odom = io.create_publisher(Odometry, "/spot1/odom", 10)
        executable = os.path.join(get_package_prefix("graphnav_planner"), "lib/graphnav_planner/planner_node")
        log = tempfile.TemporaryFile()
        native = subprocess.Popen([executable, "--ros-args", "-r", "__node:=graphnav_planner", "-r", "__ns:=/spot1"], stdout=log, stderr=log)
        exit_xy = (7., 0.)

        def wait(predicate):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                g = frontier_graph(io.get_clock().now().to_msg(), exit_xy)
                o = Odometry(); o.header = g.header
                o.pose.pose.position.z, o.pose.pose.orientation.w = 52., 1.
                odom.publish(o); raw.publish(g)
                scores = deepcopy(g); scores.edges = []; score.publish(scores)
                rclpy.spin_once(io, timeout_sec=.03)
                if predicate(): return
            self.fail("Planner kept an expired frontier exit: " + repr(endpoints[-5:]))

        try:
            wait(lambda: (7., 0.) in endpoints)
            endpoints.clear()
            exit_xy = (5., 2.)  # Same node/score, old outlet no longer exists.
            wait(lambda: (5., 2.) in endpoints)
        finally:
            native.terminate()
            try: native.wait(timeout=5)
            except subprocess.TimeoutExpired: native.kill(); native.wait()
            log.close(); io.destroy_node(); rclpy.shutdown()

    def test_unreachable_frontier_yields_to_alternative_without_clearing_target(self):
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import Point, PoseStamped
        from graphnav_msgs.msg import NavigationGraph, Node as GraphNode, Edge, EdgeTraversability, NodeTraversabilityProperties, KeyValue
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import ComputePathToPose, FollowPath
        from rclpy.action import ActionServer
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

        rclpy.init()
        io, bridge = Node("frontier_failure_io"), GraphPathBridge()
        group, stop = ReentrantCallbackGroup(), threading.Event()
        statuses, failures, requests = [], [], []
        pubs = {name: io.create_publisher(NavigationGraph, name, 10) for name in ("/spot1/nav_graph", "/spot1/scored_nav_graph")}
        odom_pub = io.create_publisher(Odometry, "/spot1/odom", 10)
        query = io.create_publisher(String, "/spot1/object_search/query", 10)
        target = io.create_publisher(PoseStamped, "/spot1/raw_object_waypoint", 10)
        io.create_subscription(String, "/wildos/plan_status", lambda m: statuses.append(json.loads(m.data)), 10)
        io.create_subscription(PoseStamped, "/spot1/graphnav_planner/unreachable", failures.append, 10)

        def publish():
            g = frontier_graph(io.get_clock().now().to_msg())
            o = Odometry()
            o.header = g.header
            o.pose.pose.position.z, o.pose.pose.orientation.w = 52., 1.
            odom_pub.publish(o)
            pubs["/spot1/nav_graph"].publish(g)
            scores = deepcopy(g)
            scores.edges = []  # WildOS returns nodes; topology supplies edges.
            pubs["/spot1/scored_nav_graph"].publish(scores)

        io.create_timer(.2, publish, callback_group=group)

        def compute(handle):
            p = handle.request.goal.pose.position
            requests.append((p.x, p.y))
            result = ComputePathToPose.Result()
            if p.x > 2.:
                handle.abort()
            else:
                result.path.header = handle.request.goal.header
                result.path.poses = [handle.request.start, handle.request.goal]
                handle.succeed()
            return result

        def follow(handle):
            stop.wait(15.)
            handle.abort()
            return FollowPath.Result()

        planner = ActionServer(io, ComputePathToPose, "/compute_path_to_pose", compute, callback_group=group)
        controller = ActionServer(io, FollowPath, "/follow_path", follow, callback_group=group)
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(io); executor.add_node(bridge)
        thread = threading.Thread(target=executor.spin); thread.start()
        log = tempfile.TemporaryFile()
        executable = os.path.join(get_package_prefix("graphnav_planner"), "lib/graphnav_planner/planner_node")
        native = subprocess.Popen([executable, "--ros-args", "-r", "__node:=graphnav_planner", "-r", "__ns:=/spot1"], stdout=log, stderr=log)

        def wait(predicate, timeout=12.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate(): return
                time.sleep(.02)
            self.fail("No frontier handoff: " + repr((requests, statuses[-5:])))

        try:
            wait(lambda: query.get_subscription_count() > 0 and pubs["/spot1/scored_nav_graph"].get_subscription_count() > 0)
            query.publish(String(data="church"))
            wait(lambda: any(s["state"] == "following" and s["goal"][1] > 5. for s in statuses))
            self.assertTrue(failures)
            self.assertTrue(any(x > 2. for x, y in requests), "Preferred frontier was never attempted")
            self.assertTrue(all(abs(m.pose.position.x - 7.) < .01 for m in failures))
            # A confirmed target remains owned by its query, even when Nav2
            # refuses it. It must never be converted to an exploration skip.
            before = len(failures)
            msg = PoseStamped()
            msg.header.frame_id = "spot1/odom"
            msg.header.stamp = io.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.z, msg.pose.orientation.w = 15., 52., 1.
            target.publish(msg)
            time.sleep(2.)
            self.assertEqual(len(failures), before)
        finally:
            native.terminate()
            try: native.wait(timeout=5)
            except subprocess.TimeoutExpired: native.kill(); native.wait()
            stop.set(); executor.shutdown(timeout_sec=3); thread.join(timeout=3)
            planner.destroy(); controller.destroy(); bridge.destroy_node(); io.destroy_node()
            rclpy.shutdown()
            log.seek(0)
            if native.returncode not in (0, -15): print(log.read().decode(errors="replace"))
            log.close()
