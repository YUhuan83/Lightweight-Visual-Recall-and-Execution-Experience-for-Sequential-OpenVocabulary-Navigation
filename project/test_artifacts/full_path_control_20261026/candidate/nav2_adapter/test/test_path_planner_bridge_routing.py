import unittest
import math
from unittest.mock import Mock
from unittest.mock import patch
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from wildos_nav2_adapter.path_planner_bridge import PathPlannerBridge


def pose(x, y):
    message = PoseStamped()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    return message


class PathPlannerBridgeRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PathPlannerBridge()
        self.node.path_pub = Mock()
        self.node.on_goal = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def test_remembered_waypoint_overrides_graph_goals_until_target_changes(self):
        self.node.on_search_query(String(data="gas station"))
        self.node.on_graph_goal(pose(725, -10))
        self.node.on_direct_goal(pose(715, -16))
        self.node.on_graph_goal(pose(700, -12))
        self.node.on_search_query(String(data=" GAS  STATION "))
        self.node.on_graph_goal(pose(699, -12))
        self.assertTrue(self.node.direct_mode)
        self.assertEqual(self.node.on_goal.call_count, 2)
        self.assertEqual(self.node.on_goal.call_args.args[0].pose.position.x, 715)

        self.node.on_search_query(String(data="trash can"))
        self.node.on_graph_goal(pose(699, -12))
        self.assertFalse(self.node.direct_mode)
        self.assertEqual(self.node.on_goal.call_count, 3)

    def test_old_plan_acceptance_is_canceled_after_mode_change(self):
        old_epoch = self.node.plan_epoch
        handle = Mock(accepted=True)
        future = Mock()
        future.result.return_value = handle
        self.node.on_direct_goal(pose(715, -16))

        self.node.on_accepted(future, (725, -10), old_epoch)

        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.pending)

    def test_remembered_target_uses_vehicle_constrained_planner(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True

        self.node.on_search_query(String(data="gas station"))
        self.node.on_direct_goal(pose(715, -16))
        direct_request = self.node.client.send_goal_async.call_args.args[0]
        self.assertEqual(direct_request.planner_id, "CarLike")

        self.node.on_search_query(String(data="trash can"))
        self.node.on_graph_goal(pose(700, -12))
        graph_request = self.node.client.send_goal_async.call_args.args[0]
        self.assertEqual(graph_request.planner_id, "GridBased")

    def test_only_dedicated_memory_topic_overrides_graph_navigation(self):
        self.assertEqual(
            self.node.get_parameter("direct_goal_topic").value,
            "/spot1/remembered_object_goal")
        self.assertEqual(self.node.planner_id, "GridBased")
        self.assertEqual(self.node.direct_planner_id, "CarLike")

    def test_remembered_goal_replans_when_viewing_heading_changes(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True

        first = pose(715, -16)
        first.pose.orientation.w = 1.0
        self.node.on_direct_goal(first)
        original_epoch = self.node.plan_epoch
        self.node.pending = False
        turned = pose(715, -16)
        turned.pose.orientation.z = math.sin(0.6 / 2)
        turned.pose.orientation.w = math.cos(0.6 / 2)
        self.node.on_direct_goal(turned)

        self.assertGreater(self.node.plan_epoch, original_epoch)
        self.assertEqual(self.node.client.send_goal_async.call_count, 2)

    def test_remembered_target_does_not_follow_holonomic_fallback_path(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True

        self.node.on_search_query(String(data="gas station"))
        self.node.on_direct_goal(pose(715, -16))
        car_like_request = self.node.client.send_goal_async.call_args.args[0]
        self.assertEqual(car_like_request.planner_id, "CarLike")

        empty_result = Mock()
        empty_result.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=Path()))
        self.node.on_result(
            empty_result, (715.0, -16.0), self.node.plan_epoch, "CarLike")

        self.assertEqual(self.node.client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status["status"], "empty_path")
        self.assertFalse(self.node.pending)

    def test_failed_graph_goal_is_rate_limited_but_new_goal_is_immediate(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True
        empty_result = Mock()
        empty_result.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=Path()), status=6)

        with patch("wildos_nav2_adapter.path_planner_bridge.time.monotonic",
                   return_value=100.0):
            self.node.on_graph_goal(pose(700, -12))
            self.node.on_result(
                empty_result, (700.0, -12.0), self.node.plan_epoch, "CarLike")
            for _ in range(10):
                self.node.on_graph_goal(pose(700, -12))
            self.assertEqual(self.node.client.send_goal_async.call_count, 1)
            self.assertEqual(self.node.last_status["action_status"], 6)
            self.node.on_graph_goal(pose(710, -12))
            self.assertEqual(self.node.client.send_goal_async.call_count, 2)

    def test_failed_direct_goal_retries_after_cooldown_without_resetting_route(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True
        empty_result = Mock()
        empty_result.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=Path()), status=6)

        with patch("wildos_nav2_adapter.path_planner_bridge.time.monotonic",
                   return_value=100.0):
            self.node.on_direct_goal(pose(715, -16))
            epoch = self.node.plan_epoch
            self.node.on_result(
                empty_result, (715.0, -16.0), epoch, "CarLike")
            self.node.on_direct_goal(pose(715, -16))
            self.assertEqual(self.node.plan_epoch, epoch)
            self.assertEqual(self.node.client.send_goal_async.call_count, 1)

        with patch("wildos_nav2_adapter.path_planner_bridge.time.monotonic",
                   return_value=105.1):
            self.node.retry_tick()
            self.assertEqual(self.node.client.send_goal_async.call_count, 2)

    def test_direct_goal_failure_limit_returns_to_latest_graph_goal(self):
        self.node.on_goal = PathPlannerBridge.on_goal.__get__(
            self.node, PathPlannerBridge)
        self.node.client = Mock()
        self.node.client.wait_for_server.return_value = True
        graph_goal = pose(700, -12)
        self.node.latest_graph_goal_message = graph_goal
        self.node.on_search_query(String(data="gas station"))
        # The current query has no cached frontier yet; model a fresh graph
        # candidate arriving while the remembered route is active.
        self.node.latest_graph_goal_message = graph_goal
        remembered_goal = pose(715, -16)
        self.node.on_direct_goal(remembered_goal)
        epoch = self.node.plan_epoch
        empty_result = Mock()
        empty_result.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=Path()), status=6)

        for _ in range(self.node.max_direct_plan_retries):
            self.node.on_result(
                empty_result, (715.0, -16.0), epoch, "CarLike")

        self.assertFalse(self.node.direct_mode)
        self.assertEqual(self.node.last_status["status"], "direct_goal_abandoned")
        self.assertEqual(self.node.client.send_goal_async.call_count, 2)
        graph_request = self.node.client.send_goal_async.call_args.args[0]
        self.assertEqual(graph_request.planner_id, "GridBased")

    def test_aborted_action_with_partial_path_is_not_followed(self):
        partial_path = Path()
        partial_path.poses = [pose(716, -16)]
        aborted_result = Mock()
        aborted_result.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=partial_path), status=6)
        self.node.follow = True
        self.node.follow_client = Mock()
        self.node.latest_goal_xy = (715.0, -16.0)
        self.node.latest_goal_message = pose(715, -16)

        self.node.on_result(
            aborted_result, (715.0, -16.0), self.node.plan_epoch, "CarLike")

        self.assertEqual(self.node.last_status["status"], "empty_path")
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.path_pub.publish.assert_not_called()

    def test_planner_error_is_exposed_without_following_partial_path(self):
        failed = Mock()
        failed.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=Path(), error_code=2,
                                   error_msg="Starting point in lethal space!"),
            status=6)
        self.node.follow = True
        self.node.follow_client = Mock()
        self.node.latest_goal_xy = (715.0, -16.0)
        self.node.latest_goal_message = pose(715, -16)

        self.node.on_result(
            failed, (715.0, -16.0), self.node.plan_epoch, "CarLike")

        self.assertEqual(self.node.last_status["error_code"], 2)
        self.assertIn("lethal space", self.node.last_status["detail"])
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_unavailable_controller_schedules_replanning(self):
        valid_path = Path()
        valid_path.poses = [pose(716, -16)]
        success = Mock()
        success.result.return_value = SimpleNamespace(
            result=SimpleNamespace(path=valid_path), status=4)
        self.node.follow = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = False
        self.node.latest_goal_xy = (715.0, -16.0)
        self.node.latest_goal_message = pose(715, -16)

        self.node.on_result(
            success, (715.0, -16.0), self.node.plan_epoch, "CarLike")

        self.assertEqual(self.node.last_status["status"], "controller_unavailable")
        self.assertIsNone(self.node.last_goal)
        self.assertGreater(self.node.plan_failure_retry_due, 0.0)

    def test_planner_future_error_does_not_leave_bridge_pending(self):
        failed_future = Mock()
        failed_future.result.side_effect = RuntimeError("planner disconnected")
        self.node.pending = True
        self.node.latest_goal_xy = (715.0, -16.0)
        self.node.latest_goal_message = pose(715, -16)

        self.node.on_result(
            failed_future, (715.0, -16.0), self.node.plan_epoch, "CarLike")

        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.last_status["status"], "planner_error")
        self.assertGreater(self.node.plan_failure_retry_due, 0.0)


if __name__ == "__main__":
    unittest.main()
