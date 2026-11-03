"""Black-box ROS Action/Topic smoke test; no bridge members are patched."""

import json
import threading
import time
import unittest

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 required; execute this gate on the ROS host")
class ActionFlowTests(unittest.TestCase):
    def test_confirmation_is_fresh_on_receipt_and_retained_only_for_current_task(self):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

        rclpy.init()
        io, bridge = Node("confirmation_lifetime_io"), GraphPathBridge()
        executor = SingleThreadedExecutor()
        executor.add_node(io)
        executor.add_node(bridge)
        statuses = []
        query = io.create_publisher(String, "/spot1/object_search/query", 10)
        targets = io.create_publisher(PoseStamped, "/spot1/raw_object_waypoint", 10)
        confirms = io.create_publisher(String, "/spot1/object_confirmation", 10)
        recalls = io.create_publisher(String, "/spot1/remembered_object_candidates", 10)
        odometry = io.create_publisher(Odometry, "/spot1/odom", 10)
        io.create_subscription(
            String,
            "/wildos/plan_status",
            lambda m: statuses.append(json.loads(m.data)),
            10,
        )
        vehicle_x = 0.0

        def step():
            odom = Odometry()
            odom.header.frame_id = "spot1/odom"
            odom.header.stamp = io.get_clock().now().to_msg()
            odom.pose.pose.position.x = vehicle_x
            odom.pose.pose.orientation.w = 1.0
            odometry.publish(odom)
            executor.spin_once(timeout_sec=0.01)

        def wait(predicate):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                step()
                if predicate():
                    return
            self.fail("Confirmation flow timed out: " + repr(statuses))

        def drain():
            for _ in range(35):
                step()

        def target():
            message = PoseStamped()
            message.header.frame_id = "spot1/odom"
            message.header.stamp = io.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.z = 20.0, 58.0
            targets.publish(message)

        def confirmation(name, observed):
            confirms.publish(
                String(
                    data=json.dumps(
                        dict(
                            query=name,
                            frame="spot1/odom",
                            search_confirmed=True,
                            observation_stamp_ns=observed,
                            object_position=[20.0, 0.0, 58.0],
                        )
                    )
                )
            )

        def found():
            return sum(s["state"] == "object_found" for s in statuses)

        def started():
            return sum(s["state"] == "query_started" for s in statuses)

        try:
            wait(
                lambda: (
                    query.get_subscription_count() == 1
                    and confirms.get_subscription_count() == 1
                    and recalls.get_subscription_count() == 1
                )
            )
            query.publish(String(data="traffic light"))
            wait(lambda: started() == 1)
            recalls.publish(String(data=json.dumps(dict(
                query="traffic light", views=[dict(
                    id="passive:traffic light:33", position=[0, 0, 52],
                    heading=0.0, search_confirmed=False,
                )],
            ))))
            wait(lambda: any(s["state"] == "recall_selected" for s in statuses))
            drain()
            self.assertEqual(found(), 0)
            vehicle_x = 17.1
            drain()
            target()
            confirmation(
                "traffic light", io.get_clock().now().nanoseconds - 6_000_000_000
            )
            drain()
            self.assertEqual(found(), 0)
            vehicle_x = 0.0
            drain()
            confirmation("traffic light", io.get_clock().now().nanoseconds)
            drain()
            deadline = time.monotonic() + 5.2
            while time.monotonic() < deadline:
                step()
            vehicle_x = 17.1  # Enter XY<3 after camera lost the object.
            wait(lambda: found() == 1)
            before_restart = io.get_clock().now().nanoseconds
            query.publish(String(data="traffic light"))  # Same words, new task.
            wait(lambda: started() == 2)
            target()
            confirmation("traffic light", before_restart)  # Fresh, but prior task.
            drain()
            self.assertEqual(found(), 1)
            confirmation("bus stop", io.get_clock().now().nanoseconds)
            drain()
            self.assertEqual(found(), 1)
            confirmation("traffic light", io.get_clock().now().nanoseconds)
            wait(lambda: found() == 2)
        finally:
            executor.shutdown()
            bridge.destroy_node()
            io.destroy_node()
            rclpy.shutdown()

    def test_recorder_cannot_replace_a_query_consumer_at_startup(self):
        from rclpy.node import Node
        from std_msgs.msg import String
        from scripts.trial_audit import query_consumers_ready

        rclpy.init()
        nodes = [
            Node(name)
            for name in (
                "query_readiness_smoke",
                "wildos_graph_path_bridge",
                "obj_mask_triangulator",
                "rosbag2_recorder",
            )
        ]
        io = nodes[0]
        publisher = io.create_publisher(String, "/spot1/object_search/query", 10)
        for node in nodes[1:]:
            node.create_subscription(
                String, "/spot1/object_search/query", lambda _: None, 10
            )

        def ready():
            return query_consumers_ready(
                info.node_name
                for info in io.get_subscriptions_info_by_topic(
                    "/spot1/object_search/query"
                )
            )

        try:
            deadline = time.monotonic() + 5
            while (
                publisher.get_subscription_count() < 3 and time.monotonic() < deadline
            ):
                rclpy.spin_once(io, timeout_sec=0.05)
            self.assertGreaterEqual(publisher.get_subscription_count(), 3)
            self.assertFalse(ready())
            vision = Node("wildos")
            nodes.append(vision)
            vision.create_subscription(
                String, "/spot1/object_search/query", lambda _: None, 10
            )
            deadline = time.monotonic() + 5
            while not ready() and time.monotonic() < deadline:
                rclpy.spin_once(io, timeout_sec=0.05)
            self.assertTrue(ready())
        finally:
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()

    def test_odometry_twist_is_in_declared_body_frame(self):
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import Odometry
        from beamng_msgs.msg import StateSensor
        from beamng_wildos_adapter.node import Adapter

        rclpy.init()
        adapter = Adapter()
        io = Node("sensor_contract_smoke")
        observed = []
        io.create_subscription(Odometry, "/spot1/odom", observed.append, 10)
        publisher = io.create_publisher(StateSensor, "/vehicles/ego/sensors/state", 10)
        executor = SingleThreadedExecutor()
        executor.add_node(adapter)
        executor.add_node(io)
        try:
            deadline = time.monotonic() + 5
            while not observed and time.monotonic() < deadline:
                message = StateSensor()
                message.header.stamp.sec = 10
                message.position.x, message.position.y, message.position.z = (
                    100.0,
                    200.0,
                    52.0,
                )
                message.dir.y, message.up.z, message.velocity.y = 1.0, 1.0, 1.2
                publisher.publish(message)
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(observed)
            self.assertEqual(observed[-1].child_frame_id, "spot1/base_link")
            self.assertAlmostEqual(observed[-1].twist.twist.linear.x, 1.2)
            self.assertAlmostEqual(observed[-1].twist.twist.linear.y, 0.0)
            self.assertAlmostEqual(observed[-1].pose.pose.position.x, 99.67)
        finally:
            executor.shutdown()
            adapter.map_worker.shutdown(wait=True)
            adapter.destroy_node()
            io.destroy_node()
            rclpy.shutdown()

    def test_failed_background_plan_keeps_action_then_refinement_preempts(self):
        from rclpy.node import Node
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from nav2_msgs.action import ComputePathToPose, FollowPath
        from nav_msgs.msg import Path, Odometry
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

        rclpy.init()
        io = Node("cleanroom_smoke_io")
        bridge = GraphPathBridge()
        group = ReentrantCallbackGroup()
        status, stop = [], threading.Event()
        unreachable = []
        io.create_subscription(PoseStamped, "/spot1/graphnav_planner/unreachable", unreachable.append, 10)
        control = dict(fail=False, follows=0, cancels=0, vehicle_x=0.0, abort_number=0)
        recalls = io.create_publisher(String, "/spot1/remembered_object_candidates", 10)
        query = io.create_publisher(String, "/spot1/object_search/query", 10)
        routes = io.create_publisher(Path, "/spot1/graphnav_planner/path", 10)
        odometry = io.create_publisher(Odometry, "/spot1/odom", 10)
        targets = io.create_publisher(PoseStamped, "/spot1/raw_object_waypoint", 10)
        confirms = io.create_publisher(String, "/spot1/object_confirmation", 10)
        io.create_subscription(
            String,
            "/wildos/plan_status",
            lambda m: status.append(json.loads(m.data)),
            10,
        )

        def send_odom():
            msg = Odometry()
            msg.header.frame_id = "spot1/odom"
            msg.header.stamp = io.get_clock().now().to_msg()
            msg.pose.pose.position.z = 52.0
            msg.pose.pose.position.x = control["vehicle_x"]
            msg.pose.pose.orientation.w = 1.0
            odometry.publish(msg)

        io.create_timer(0.05, send_odom, callback_group=group)

        def compute(handle):
            result = ComputePathToPose.Result()
            if control["fail"]:
                handle.abort()
                return result
            result.path.header = handle.request.goal.header
            result.path.poses = [handle.request.start, handle.request.goal]
            handle.succeed()
            return result

        def follow(handle):
            control["follows"] += 1
            number = control["follows"]
            while not stop.wait(0.02):
                if number == control["abort_number"]:
                    handle.abort()
                    return FollowPath.Result()
                if handle.is_cancel_requested:
                    control["cancels"] += 1
                    handle.canceled()
                    return FollowPath.Result()
                if number != control["follows"]:
                    handle.succeed()
                    return FollowPath.Result()
            handle.abort()
            return FollowPath.Result()

        planner = ActionServer(
            io,
            ComputePathToPose,
            "/compute_path_to_pose",
            compute,
            callback_group=group,
        )
        controller = ActionServer(
            io,
            FollowPath,
            "/follow_path",
            follow,
            callback_group=group,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(io)
        executor.add_node(bridge)
        thread = threading.Thread(target=executor.spin)
        thread.start()

        def wait(predicate, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                time.sleep(0.02)
            self.fail("ROS flow timed out: " + repr(status[-4:]))

        def route(offset, middle=None):
            msg = Path()
            msg.header.frame_id = "spot1/odom"
            msg.header.stamp = io.get_clock().now().to_msg()
            for x, y in [(0, 0), (9, offset if middle is None else middle), (18, offset)]:
                point = PoseStamped()
                point.header = msg.header
                point.pose.position.x, point.pose.position.y = float(x), float(y)
                point.pose.orientation.w = 1.0
                msg.poses.append(point)
            routes.publish(msg)

        try:
            wait(
                lambda: (
                    query.get_subscription_count() >= 1
                    and routes.get_subscription_count() >= 1
                )
            )
            query.publish(String(data="green house"))
            wait(lambda: any(s["state"] == "query_started" for s in status))
            route(0)
            wait(lambda: control["follows"] == 1)
            planned_before = sum(s["state"] == "planning" for s in status)
            route(.2)
            time.sleep(1.3)
            self.assertEqual(sum(s["state"] == "planning" for s in status), planned_before,
                             "Small topology refresh replaced a healthy distant path")
            control["fail"] = True
            route(6)
            wait(
                lambda: any(
                    s["state"] == "no_vehicle_path" and s["retained_action"]
                    for s in status
                )
            )
            self.assertEqual(control["cancels"], 0)
            self.assertFalse(unreachable, "Background failure retired a healthy route")
            planned_before = sum(s["state"] == "planning" for s in status)
            route(6, middle=7)
            time.sleep(1.3)
            self.assertEqual(sum(s["state"] == "planning" for s in status), planned_before,
                             "Failed intent was retried for an unchanged endpoint")
            control["fail"] = False
            route(9)
            wait(lambda: control["follows"] >= 2)
            self.assertEqual(control["cancels"], 0)
            # Once the current action aborts and every replacement fails, the
            # graph planner must receive the actual topology route endpoint.
            control["fail"] = True
            control["abort_number"] = control["follows"]
            wait(lambda: len(unreachable) >= 1)
            self.assertAlmostEqual(unreachable[-1].pose.position.x, 18.)
            self.assertAlmostEqual(unreachable[-1].pose.position.y, 9.)
            failures = len(unreachable)
            control["fail"] = False
            route(4)
            wait(lambda: control["follows"] > control["abort_number"])
            point = PoseStamped()
            point.header.frame_id = "spot1/odom"
            point.header.stamp = io.get_clock().now().to_msg()
            point.pose.position.x, point.pose.position.z = 10.0, 58.0
            targets.publish(point)
            wait(
                lambda: any(
                    s["state"] == "following" and abs(s["goal"][0] - 8) < .01
                    for s in status
                )
            )
            pinned = control["follows"]
            semantic_status_start = len(status)
            control["fail"] = True
            control["abort_number"] = pinned
            wait(lambda: any(s["state"] == "no_vehicle_path" and not s["retained_action"] for s in status[semantic_status_start:]))
            self.assertEqual(len(unreachable), failures, "Semantic target was rejected as a frontier")
            control["fail"] = False
            wait(lambda: control["follows"] > pinned)
            pinned = control["follows"]
            control["vehicle_x"] = 4.0
            for offset in (4, 5, 6):
                route(offset)
                point.header.stamp = io.get_clock().now().to_msg()
                point.pose.position.x, point.pose.position.z = 10.1, 60.0
                targets.publish(point)
                time.sleep(1.1)
            self.assertEqual(
                control["follows"], pinned, "Healthy terminal path was replaced"
            )
            control["abort_number"] = pinned
            wait(lambda: control["follows"] > pinned)
            recovered = control["follows"]
            point.header.stamp = io.get_clock().now().to_msg()
            point.pose.position.x = 14.0
            targets.publish(point)
            wait(lambda: control["follows"] > recovered)
            self.assertEqual(control["cancels"], 0)
            point.header.stamp = io.get_clock().now().to_msg()
            point.pose.position.x, point.pose.position.z = 2.9, 58.0
            point.pose.orientation.w = 1.0
            targets.publish(point)
            confirms.publish(
                String(
                    data=json.dumps(
                        dict(
                            query="green house",
                            frame="spot1/odom",
                            observation_stamp_ns=io.get_clock().now().nanoseconds,
                            object_position=[2.9, 0, 58],
                            search_confirmed=True,
                        )
                    )
                )
            )
            wait(lambda: any(s["state"] == "object_found" for s in status))
            wait(lambda: control["cancels"] == 1)
            recall_status_start = len(status)
            query.publish(String(data="gas station"))
            wait(lambda: any(s["state"] == "query_started" for s in status[recall_status_start:]))
            control["fail"] = True
            recalls.publish(String(data=json.dumps(dict(
                query="gas station", views=[dict(id="station-view", position=[18., 0., 52.])],
            ))))
            wait(lambda: any(s["state"] == "recall_selected" for s in status[recall_status_start:]))
            recall_attempt_start = len(status)
            route(7)
            wait(lambda: any(s["state"] == "planning" for s in status[recall_attempt_start:]) and any(s["state"] == "no_vehicle_path" and not s["retained_action"] for s in status[recall_attempt_start:]))
            self.assertEqual(len(unreachable), failures, "Recalled target was rejected as a frontier")
        finally:
            stop.set()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            planner.destroy()
            controller.destroy()
            bridge.destroy_node()
            io.destroy_node()
            rclpy.shutdown()
