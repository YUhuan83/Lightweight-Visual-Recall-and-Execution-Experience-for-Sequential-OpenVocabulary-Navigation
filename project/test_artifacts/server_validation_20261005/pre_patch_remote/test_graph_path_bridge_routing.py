import unittest
import time
import json
from types import SimpleNamespace
from unittest.mock import Mock
from concurrent.futures import Future

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String

from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = "spot1/odom"
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


def route(*points):
    path = Path()
    path.header.frame_id = "spot1/odom"
    path.header.stamp = rclpy.clock.Clock().now().to_msg()
    path.poses = [pose(*point) for point in points]
    return path


class GraphBridgeRoutingTests(unittest.TestCase):
    def prepare_escape(self):
        self.node.follow_enabled = True
        self.node.odom.pose.pose = pose(6., 0.).pose
        self.node.route = route((6., 0.), (9., 0.))
        self.node.route_endpoint = (9., 0.)
        now = time.monotonic()
        self.node.escape_trace.extend((i*.3, 0., 0., now-3.+i*.1) for i in range(21))
        self.node.reject_exploration_route = Mock(return_value=True)

    def test_stall_location_survives_changed_remote_frontier(self):
        self.prepare_escape()
        self.node.send_motion_segment = Mock()
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.route_endpoint = (15., 5.)
        self.node.recovery_attempts = 0
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.assertTrue(self.node.recovering)
        self.node.send_motion_segment.assert_called_once()
        self.assertLess(self.node.escape_path.poses[-1].pose.position.x, 4.1)

    def test_escape_respects_current_map_and_does_not_erase_memory(self):
        self.prepare_escape()
        self.node.follow_client = Mock()
        self.node.grid.data[10*40+13] = 100
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(self.node.recovering)
        self.assertEqual(self.node.last_status['status'], 'recovery_failed')
        self.node.expected = self.node.memory_goal = (9., 0.)
        self.node.send_motion_segment = Mock()
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.memory_goal, (9., 0.))

    def test_escape_acceptance_does_not_learn_rejected_forward_route(self):
        self.prepare_escape()
        self.node.send_motion_segment = Mock()
        for _ in range(2):
            self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.checkpoint_execution_feedback = Mock()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = Future()
        future = Future(); future.set_result(handle)
        self.node.on_follow_accepted(future, self.node.epoch, (3.9, 0.))
        self.node.checkpoint_execution_feedback.assert_not_called()

    def test_stalled_explicit_target_can_escape_and_resume_same_goal(self):
        self.prepare_escape()
        target = (12., 3.)
        self.node.expected = target
        self.node.send_motion_segment = Mock()
        self.node.reject_exploration_route = Mock(return_value=False)

        for _ in range(2):
            self.node.follow_failed('execution_stalled',
                                    stall_class='drive_limited_or_physical_contact')

        self.assertTrue(self.node.recovering)
        self.node.send_motion_segment.assert_called_once()
        self.assertEqual(self.node.expected, target)
        self.assertIsNone(self.node.memory_goal)

    def test_frontier_observation_heading_is_prioritized(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        from wildos_nav2_adapter.input_quality import stamp_ns
        self.node.route_metadata[stamp_ns(self.node.route)] = dict(complete=True, observation_heading=1.2)
        self.node.try_candidate = Mock()
        self.node.start_segment()
        self.assertEqual(self.node.candidates[0][:3], (6., 0., 1.2))

    def test_repeated_stall_postpones_exploration_after_two_retries(self):
        self.node.reject_exploration_route = Mock(return_value=True)
        self.node.recovery_attempts = 0
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.reject_exploration_route.assert_not_called()
        self.node.recovery_attempts = 2
        self.node.follow_failed('execution_stalled', stall_class='drive_limited_or_physical_contact')
        self.node.reject_exploration_route.assert_called_once()

    def test_repeated_memory_route_stall_rechecks_alternate_views(self):
        views = [{'id': 'remembered-a'}, {'id': 'remembered-b'}]
        self.node.memory_source_views = views
        self.node.memory_active = views[0]
        self.node.memory_goal = self.node.expected = (8., 0.)
        self.node.recovery_attempts = 2
        self.node.begin_memory_selection = Mock()

        self.node.follow_failed('execution_stalled',
                                stall_class='drive_limited_or_physical_contact')

        self.node.begin_memory_selection.assert_called_once_with(views, {'remembered-a'})
        self.assertEqual(self.node.memory_goal, (8., 0.))
        self.node.memory_feedback_pub.publish.assert_not_called()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GraphPathBridge()
        self.node.require_route_status = False  # Existing legacy route fixtures.
        self.node.waypoint_pub = Mock()
        self.node.path_pub = Mock()
        self.node.status_pub = Mock()
        self.node.memory_feedback_pub = Mock()
        self.node.memory_selection_pub = Mock()
        self.node.graph_reset_client = Mock()
        self.node.graph_reset_client.service_is_ready.return_value = True
        def cleared(_request):
            future = Future()
            future.set_result(Mock(success=True))
            return future
        self.node.graph_reset_client.call_async.side_effect = cleared
        odom = Odometry()
        odom.header.frame_id = "spot1/odom"
        odom.pose.pose.position.x = 0.
        self.node.odom = odom
        self.node.odom_received_at = time.monotonic()
        grid = OccupancyGrid()
        grid.header.frame_id = "spot1/odom"
        grid.info.resolution = 1.
        grid.info.width = 40
        grid.info.height = 40
        grid.info.origin.position.x = -10.
        grid.info.origin.position.y = -10.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 1600
        self.node.grid = grid
        self.node.grid_received_at = time.monotonic()
        self.node.grid_arrived_at = time.monotonic()
        from copy import deepcopy
        self.node.local_grid = deepcopy(grid)
        self.node.local_grid_received_at = time.monotonic()

    def test_local_collision_prevents_follow_despite_free_global_map(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.motion_queue = [self.node.route]
        # Local-only obstacle will be swept by the long front of the body.
        self.node.local_grid.data[10*40+14] = 100
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status['detail'], 'local_footprint_collision')

    def test_stale_local_map_prevents_follow_and_keeps_memory_destination(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.expected = self.node.memory_goal = (10., 0.)
        self.node.motion_queue = [self.node.route]
        self.node.local_grid_received_at = time.monotonic()-2.
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'stale_local_costmap')
        self.assertEqual(self.node.memory_goal, (10., 0.))

    def test_follow_failure_recovery_backoff_is_bounded_and_short(self):
        for attempt, expected in enumerate((2., 4., 8., 8.)):
            self.node.recovery_attempts = attempt
            started = time.monotonic()
            self.node.follow_failed('follow_aborted')
            self.assertAlmostEqual(self.node.blocked_until-started, expected, delta=.1)

    def tearDown(self):
        self.node.destroy_node()

    def test_new_costmap_wakes_candidate_even_when_header_is_one_period_old(self):
        # The live global costmap can arrive roughly once per second with a
        # header close to one period old. That is fresh under the 3 s input
        # timeout and must not cause the bridge to wait forever on every map.
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.candidate_index = 0
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.plan_client.send_goal_async.return_value = Future()
        self.node.grid_received_at = time.monotonic() - .95
        self.node.grid_arrived_at = time.monotonic() - .95

        self.node.try_candidate()
        self.assertTrue(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

        from copy import deepcopy
        update = deepcopy(self.node.grid)
        stamp = self.node.get_clock().now().nanoseconds - 950_000_000
        update.header.stamp.sec = stamp // 10**9
        update.header.stamp.nanosec = stamp % 10**9
        self.node.on_map(update)

        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertFalse(self.node.candidate_waiting_for_map)
        self.assertTrue(self.node.pending)

    def test_remembered_goal_routes_through_graph_and_ignores_other_waypoints(self):
        self.node.on_query(String(data="gas station"))
        self.node.on_memory_goal(pose(15., 0.))
        self.node.on_waypoint(pose(30., 0.))
        self.assertEqual(self.node.waypoint_pub.publish.call_count, 1)
        self.assertEqual(self.node.expected, (15., 0.))
        self.assertEqual(self.node.last_status["source"], "memory_via_graphnav")

        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (30., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.node.start_segment.assert_called_once()

        self.node.on_query(String(data="trash can"))
        self.node.on_waypoint(pose(25., 0.))
        self.assertIsNone(self.node.memory_goal)
        self.assertEqual(self.node.waypoint_pub.publish.call_count, 2)

    def test_same_route_is_not_replanned_after_failure(self):
        path = route((0., 0.), (10., 0.))
        self.node.route = path
        self.node.route_endpoint = (10., 0.)
        self.node.block_current_route()
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()

    def test_exhausted_car_plans_reject_exact_exploration_route(self):
        self.node.follow_enabled = True
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.route_rejection_pub = Mock()
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.candidate_index = len(self.node.candidates)
        self.node.try_candidate()
        self.node.route_rejection_pub.publish.assert_called_once()
        message = self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, self.node.route.header.frame_id)
        self.assertEqual(message.header.stamp.sec, self.node.route.header.stamp.sec)
        self.assertEqual(message.header.stamp.nanosec, self.node.route.header.stamp.nanosec)
        self.assertEqual(message.pose.position.x, 10.)

    def test_local_exploration_rejection_preserves_object_goals_and_freshness(self):
        self.node.follow_enabled = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_rejection_pub = Mock()
        self.node.input_freshness_issue = Mock(return_value=None)
        self.node.expected = (10., 0.)
        self.assertFalse(self.node.reject_exploration_route())
        self.node.expected = None
        self.node.input_freshness_issue.return_value = 'stale_odom'
        self.assertFalse(self.node.reject_exploration_route())
        self.node.input_freshness_issue.return_value = None
        self.node.query_started_ns = 10**30
        self.assertFalse(self.node.reject_exploration_route())
        self.node.route_rejection_pub.publish.assert_not_called()

    def test_failed_follow_retries_without_new_graph_path_and_stops_after_three(self):
        self.node.memory_goal = self.node.expected = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.follow_client = Mock()
        self.node.follow_failed('follow_aborted')
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()  # backoff still applies
        self.node.blocked_until = time.monotonic()-1
        for _ in range(8):
            self.node.retry_when_planner_starts()
        self.assertEqual(self.node.start_segment.call_count, 3)
        self.assertEqual(self.node.expected, (15., 0.))
        self.assertEqual(self.node.memory_goal, (15., 0.))
        # Map churn and repeated graph publication cannot reset the budget.
        self.node.grid.data[-1] = 100
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.assertEqual(self.node.start_segment.call_count, 3)

    def test_recovery_requires_enabled_execution_fresh_inputs_and_correct_destination(self):
        self.node.expected = self.node.memory_goal = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.plan_client = Mock()
        self.node.follow_client = Mock()
        self.node.follow_failed('follow_aborted')
        self.node.blocked_until = time.monotonic()-1
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()
        self.node.follow_enabled = True
        self.node.odom_received_at = time.monotonic()-10
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_not_called()
        self.node.odom_received_at = time.monotonic()
        self.node.on_route(route((0., 0.), (40., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.route_endpoint, (15., 0.))

    def test_follow_failure_preserves_memory_and_rechecks_a_changed_map(self):
        self.node.on_query(String(data='gas station'))
        self.node.on_memory_goal(pose(15., 0.))
        view = {'id':'1', 'frame':'spot1/odom', 'position':[15.,0.,0.], 'heading':0.}
        self.node.memory_views = [view]
        self.node.memory_active = view
        path = route((0., 0.), (15., 0.))
        self.node.route = path
        self.node.route_endpoint = (15., 0.)
        self.node.following = True
        epoch = self.node.epoch

        self.node.follow_failed('follow_aborted', action_status=6)

        self.assertEqual(self.node.memory_goal, (15., 0.))
        self.assertEqual(self.node.memory_active, view)
        self.assertEqual(self.node.memory_rejected, set())
        self.assertEqual(self.node.last_status['source'], 'memory_via_graphnav')
        self.assertEqual(self.node.epoch, epoch+1)
        self.node.memory_feedback_pub.publish.assert_not_called()
        self.node.memory_selection_pub.publish.assert_not_called()
        self.node.start_segment = Mock()
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()
        self.node.blocked_until = time.monotonic()-1
        self.node.grid.data[-1] = 100
        self.node.on_route(route((0., 0.), (15., 0.)))
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.expected, (15., 0.))

    def test_tracking_deviation_cancels_current_section_and_keeps_memory(self):
        self.node.memory_goal = self.node.expected = (15., 0.)
        self.node.route = route((0., 0.), (15., 0.))
        self.node.route_endpoint = (15., 0.)
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.node.following = True
        handle = self.node.follow_handle = Mock()
        self.node.odom.pose.pose.position.y = 1.2
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once_with()
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.motion_queue, [])
        self.assertEqual(self.node.last_status['reason'], 'tracking_deviation')
        self.assertEqual(self.node.memory_goal, (15., 0.))
        self.node.memory_feedback_pub.publish.assert_not_called()

    def test_new_query_cancels_in_flight_planning_goal(self):
        planner_goal = Mock()
        self.node.plan_handle = planner_goal
        self.node.pending = True

        self.node.on_query(String(data="tree"))

        planner_goal.cancel_goal_async.assert_called_once_with()
        self.assertIsNone(self.node.plan_handle)
        self.assertFalse(self.node.pending)

    def test_late_planner_acceptance_after_reset_is_cancelled(self):
        self.node.reset()
        stale_epoch = self.node.epoch - 1
        planner_goal = Mock()
        planner_goal.accepted = True
        future = Mock()
        future.result.return_value = planner_goal

        self.node.on_plan_accepted(future, stale_epoch, 0)

        planner_goal.cancel_goal_async.assert_called_once_with()
        planner_goal.get_result_async.assert_not_called()

    def test_late_planner_acceptance_for_old_candidate_is_cancelled(self):
        self.node.epoch = 2
        self.node.candidate_index = 1
        planner_goal = Mock()
        planner_goal.accepted = True
        future = Mock()
        future.result.return_value = planner_goal

        self.node.on_plan_accepted(future, self.node.epoch, 0)

        planner_goal.cancel_goal_async.assert_called_once_with()
        planner_goal.get_result_async.assert_not_called()

    def test_stalled_compute_path_adopts_a_newer_graph_route(self):
        old_route = route((0., 0.), (10., 0.))
        newest_route = route((0., 0.), (20., 0.))
        self.node.route = old_route
        self.node.latest_route = newest_route
        self.node.route_endpoint = (10., 0.)
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.pending = True
        self.node.plan_handle = Mock()
        old_plan_handle = self.node.plan_handle
        self.node.plan_request_started_at = time.monotonic() - self.node.compute_path_timeout - .1
        self.node.start_segment = Mock()
        old_epoch = self.node.epoch

        self.node.monitor_inputs()

        old_plan_handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.node.epoch, old_epoch + 1)
        self.assertIs(self.node.route, newest_route)
        self.assertEqual(self.node.route_endpoint, (20., 0.))
        self.assertEqual(self.node.last_status['status'], 'planner_timeout_using_latest_route')
        self.node.start_segment.assert_called_once_with()

    def test_stalled_compute_path_advances_candidate_when_route_is_unchanged(self):
        path = route((0., 0.), (10., 0.))
        self.node.route = self.node.latest_route = path
        self.node.route_endpoint = (10., 0.)
        self.node.candidates = [(6., 0., 0., 6.), (8., 0., 0., 8.)]
        self.node.candidate_index = 0
        self.node.pending = True
        self.node.plan_handle = Mock()
        old_plan_handle = self.node.plan_handle
        self.node.plan_request_started_at = time.monotonic() - self.node.compute_path_timeout - .1
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.plan_client.send_goal_async.return_value = Future()
        old_epoch = self.node.epoch

        self.node.monitor_inputs()

        old_plan_handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.node.epoch, old_epoch + 1)
        self.assertEqual(self.node.candidate_index, 1)
        self.node.plan_client.send_goal_async.assert_called_once()
        self.assertTrue(self.node.pending)
        self.assertIsNotNone(self.node.plan_request_started_at)

    def test_stale_costmap_prevents_starting_a_plan(self):
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.grid_received_at = time.monotonic() - self.node.costmap_timeout - 0.1
        self.node.plan_client = Mock()

        self.node.start_segment()

        self.assertEqual(self.node.last_status["status"], "stale_costmap")
        self.node.plan_client.wait_for_server.assert_not_called()

    def test_stale_costmap_discards_a_completed_planner_result(self):
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = (10., 0.)
        self.node.pending = True
        self.node.grid_received_at = time.monotonic() - self.node.costmap_timeout - 0.1
        future = Mock()

        self.node.on_plan_result(future, self.node.epoch, 0)

        self.assertEqual(self.node.last_status["status"], "stale_costmap")
        self.assertFalse(self.node.pending)
        future.result.assert_not_called()
        self.node.path_pub.publish.assert_not_called()

    def test_memory_goal_waits_for_old_graph_goal_to_be_cleared(self):
        future = Future()
        self.node.graph_reset_client.call_async.side_effect = None
        self.node.graph_reset_client.call_async.return_value = future
        self.node.on_query(String(data="gas station"))
        self.node.on_memory_goal(pose(20., 0.))
        self.node.waypoint_pub.publish.assert_not_called()
        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (20., 0.)))
        self.node.start_segment.assert_not_called()
        future.set_result(Mock(success=True))
        self.node.waypoint_pub.publish.assert_called_once()
        self.assertFalse(self.node.query_reset_pending)

    def test_memory_approach_extends_past_graph_goal_radius(self):
        self.node.on_memory_goal(pose(20., 0.))
        self.node.route = route((0., 0.), (14., 0.))
        self.assertEqual(self.node.current_route_points()[-1], (20., 0.))

    def test_coarse_graph_corner_allows_an_approach_heading(self):
        self.node.route = route((0., 0.), (6., 0.), (6., 6.))
        self.node.route_endpoint = (6., 6.)
        self.node.memory_goal = self.node.expected = (6., 6.)
        self.node.memory_heading = 1.2
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.try_candidate = Mock()

        self.node.start_segment()

        at_corner = [c for c in self.node.candidates if c[:2] == (6., 2.)]
        self.assertEqual(len(at_corner), 2)
        self.assertAlmostEqual(at_corner[0][2], 1.5707963267948966)
        self.assertAlmostEqual(at_corner[1][2], 0.3217505543966422)
        terminal = [c for c in self.node.candidates if c[:2] == (6., 6.)]
        self.assertTrue(any(abs(c[2] - 1.2) < 1e-6 for c in terminal))
        self.assertGreater(len(terminal), 1)

    def memory_views(self, *positions):
        return [{'id': str(i), 'position': [float(x), 0., 0.], 'heading': 0.,
                 'frame': 'spot1/odom'} for i, x in enumerate(positions)]

    def send_views(self, views):
        self.node.on_query(String(data='tree'))
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.on_memory_candidates(String(data=json.dumps({'query':'tree','views':views})))

    def valid_plan(self, endpoint):
        result = Mock()
        result.status = 4
        result.result.path = route((0., 0.), (endpoint, 0.))
        future = Mock()
        future.result.return_value = result
        self.node.on_plan_result(future, self.node.epoch, self.node.candidate_index)

    def test_memory_selection_compares_feasible_route_costs(self):
        self.send_views(self.memory_views(8., 4.))
        self.node.on_route(route((0.,0.),(8.,0.)))
        self.valid_plan(8.)
        self.assertTrue(self.node.memory_selecting)
        self.node.on_route(route((0.,0.),(4.,0.)))
        self.valid_plan(4.)
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.memory_active['id'], '1')
        self.assertEqual(self.node.memory_goal, (4.,0.))
        self.node.path_pub.publish.assert_called_once()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses[-1].pose.position.x, 4.)

    def test_blocked_memory_view_switches_to_another_observation(self):
        self.node.grid.data[10*40+22] = 100  # remembered target x=12
        self.send_views(self.memory_views(12., 6.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.valid_plan(12.)
        self.assertEqual(self.node.memory_active['id'], '1')
        self.node.on_route(route((0.,0.),(6.,0.)))
        self.valid_plan(6.)
        self.assertEqual(self.node.memory_goal, (6.,0.))
        self.assertEqual(self.node.memory_rejected, {'0'})

    def test_local_memory_failure_retains_old_graph_goal(self):
        self.send_views(self.memory_views(12.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.node.candidate_failed('empty_path')
        self.assertEqual(self.node.memory_goal, (12., 0.))
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.graph_reset_client.call_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'memory_locally_blocked')
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unavailable')

    def test_short_prefix_does_not_prove_memory_endpoint_reachable(self):
        self.send_views(self.memory_views(12.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.assertTrue(all(c[:2] == (12.,0.) for c in self.node.candidates))

    def test_blocked_current_pose_does_not_discard_the_memory_destination(self):
        self.send_views(self.memory_views(12.))
        self.node.route = route((0.,0.),(12.,0.))
        self.node.grid.data[10*40+12] = 100  # inside the current vehicle body
        self.node.start_segment()
        self.assertEqual(self.node.last_status['status'], 'start_footprint_blocked')
        self.node.memory_deadline = time.monotonic()-1.
        self.node.monitor_memory_selection()
        self.assertTrue(self.node.memory_selecting)
        self.assertEqual(self.node.memory_goal,(12.,0.))
        self.node.memory_feedback_pub.publish.assert_not_called()
        self.assertEqual(self.node.graph_reset_client.call_async.call_count,1)

    def test_memory_planning_resumes_when_current_pose_becomes_free(self):
        self.send_views(self.memory_views(12.))
        self.node.status('start_footprint_blocked')
        self.node.start_segment = Mock()
        self.node.monitor_memory_selection()
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.memory_goal,(12.,0.))

    def test_memory_uses_alternative_arrival_heading(self):
        views=self.memory_views(8.)
        views[0]['heading']=3.141592653589793
        self.send_views(views)
        self.node.on_route(route((0.,0.),(8.,0.)))
        self.node.candidate_failed('empty_path')
        self.valid_plan(8.)
        self.assertFalse(self.node.memory_selecting)
        self.assertAlmostEqual(self.node.memory_arrival_heading,0.)

    def test_memory_candidates_can_arrive_before_the_query(self):
        views=self.memory_views(8.)
        self.node.on_memory_candidates(String(data=json.dumps({'query':'tree','views':views})))
        self.node.on_query(String(data='tree'))
        self.assertEqual(self.node.memory_active['id'],'0')

    def test_candidate_set_ignores_legacy_single_pose_replay(self):
        self.send_views(self.memory_views(8.))
        self.node.on_memory_goal(pose(20.,0.))
        self.assertEqual(self.node.memory_goal,(8.,0.))

    def test_new_query_cancels_memory_selection(self):
        self.send_views(self.memory_views(8.))
        self.node.on_query(String(data='trash can'))
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.memory_views,[])

    def test_latest_graph_route_is_used_after_a_segment(self):
        self.node.following = True
        updated = route((0., 0.), (20., 0.))
        self.node.on_route(updated)
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.odom.pose.pose.position.x = 6.
        self.node.on_follow_result(future, self.node.epoch, (6., 0.))
        self.assertIs(self.node.route, updated)
        self.node.start_segment.assert_called_once()

    def prepare_reverse_forward_execution(self):
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.follow_client.send_goal_async.side_effect = lambda _goal: Future()
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.motion_queue = [route((0., 0.), (-2., 0.)),
                                  route((-2., 0.), (0., 0.), (6., 0.))]
        self.node.send_motion_segment()

    def test_follow_goal_ends_at_cusp_using_strict_goal_checker(self):
        self.prepare_reverse_forward_execution()
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.goal_checker_id, 'maneuver_goal_checker')
        self.assertEqual(goal.path.poses[-1].pose.position.x, -2.)
        self.assertEqual(len(goal.path.poses), 2)

    def test_premature_action_success_cannot_start_next_direction(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -1.65
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['reason'], 'cusp_not_reached')
        self.assertEqual(self.node.motion_queue, [])

    def test_reached_cusp_dispatches_only_next_checked_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -2.
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.assertEqual(goal.path.poses[0].pose.position.x, -2.)
        self.assertEqual(goal.goal_checker_id, 'goal_checker')
        self.assertTrue(self.node.following)

    def test_costmap_change_at_cusp_replans_instead_of_replaying_next_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -2.
        self.node.grid.data[10 * 40 + 14] = 100
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'replanning_after_maneuver')
        self.assertEqual(self.node.last_status['reason'], 'collision_cell')
        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.motion_queue, [])

    def test_offset_cusp_can_join_clear_straight_branch_without_replanning(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.position.x = -1.8
        self.node.start_segment = Mock()
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.last_status['status'], 'planned')
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.node.start_segment.assert_not_called()

    def test_query_change_discards_unexecuted_motion_sections(self):
        self.prepare_reverse_forward_execution()
        self.node.on_query(String(data='gas station'))
        self.assertEqual(self.node.motion_queue, [])
        self.assertFalse(self.node.following)

    def test_completed_plan_is_split_before_first_follow_goal(self):
        self.prepare_reverse_forward_execution()
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.following = False
        self.node.candidates = [(6., 0., 0., 6.)]
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-2., 0.), (0., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.assertEqual(len(self.node.motion_queue), 2)
        goal = self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.path.poses[-1].pose.position.x, -2.)
        self.assertEqual(goal.goal_checker_id, 'maneuver_goal_checker')

    def test_quantized_short_cusp_is_rejected_before_follow_action(self):
        self.prepare_reverse_forward_execution()
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.following = False
        self.node.odom.pose.pose.position.x = -.126
        self.node.candidates = [(6., 0., 0., 6.)]
        self.node.candidate_failed = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-.356, 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.candidate_failed.assert_called_once_with('maneuver_already_within_tolerance')
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_two_headings_choose_shorter_path_without_reversals(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.candidates = [(6., 0., 1., 6.), (6., 0., 0., 6.), (4., 0., 0., 4.)]
        self.node.try_candidate = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (-2., 0.), (0., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.path_pub.publish.assert_not_called()
        self.node.try_candidate.assert_called_once()
        future.result.return_value.result.path = route((0., 0.), (3., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 1)
        selected = self.node.path_pub.publish.call_args.args[0]
        self.assertEqual([p.pose.position.x for p in selected.poses], [0., 3., 6.])
        self.assertEqual(self.node.try_candidate.call_count, 1)

    def test_alternative_failure_rechecks_first_path_against_updated_map(self):
        self.node.route = route((0., 0.), (6., 0.))
        self.node.route_endpoint = (6., 0.)
        self.node.candidates = [(6., 0., 1., 6.), (6., 0., 0., 6.)]
        self.node.try_candidate = Mock()
        future = Mock()
        future.result.return_value.status = 4
        future.result.return_value.result.path = route((0., 0.), (3., 0.), (6., 0.))
        self.node.on_plan_result(future, self.node.epoch, 0)
        self.node.grid.data[10*40+13] = 100
        self.node.candidate_failed('empty_path')
        self.node.path_pub.publish.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'path_changed_before_execution')

    def test_late_follow_acceptance_after_query_is_cancelled(self):
        self.prepare_reverse_forward_execution()
        epoch = self.node.epoch
        self.node.on_query(String(data='gas station'))
        handle = Mock(accepted=True)
        future = Mock()
        future.result.return_value = handle
        self.node.on_follow_accepted(future, epoch, (-2., 0.))
        handle.cancel_goal_async.assert_called_once_with()
        handle.get_result_async.assert_not_called()
        self.assertEqual(self.node.motion_queue, [])

    def test_old_follow_result_cannot_resume_after_query_change(self):
        self.prepare_reverse_forward_execution()
        epoch = self.node.epoch
        self.node.on_query(String(data='gas station'))
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.query, 'gas station')
        self.assertEqual(self.node.motion_queue, [])

    def test_stale_odom_at_cusp_prevents_next_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom_received_at = time.monotonic() - self.node.odom_timeout - .1
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'stale_odom')
        self.assertEqual(self.node.motion_queue, [])

    def test_disable_execution_discards_queued_sections(self):
        self.prepare_reverse_forward_execution()
        result = self.node.on_parameters([SimpleNamespace(name='follow_path', value=False)])
        self.assertTrue(result.successful)
        self.assertFalse(self.node.following)
        self.assertFalse(self.node.follow_enabled)
        self.assertEqual(self.node.motion_queue, [])

    def test_missing_maneuver_checker_rejection_stops_execution(self):
        self.prepare_reverse_forward_execution()
        future = Mock()
        future.result.return_value.accepted = False
        self.node.on_follow_accepted(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.last_status['reason'], 'controller_rejected')
        self.assertFalse(self.node.following)
        self.assertEqual(self.node.motion_queue, [])

    def test_stale_input_cancels_follow_and_fresh_data_allows_retry(self):
        handle = Mock()
        self.node.follow_handle = handle
        self.node.following = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.odom_received_at = time.monotonic() - self.node.odom_timeout - .1
        self.node.monitor_inputs()
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.following)
        self.node.odom_received_at = self.node.grid_received_at = time.monotonic()
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.start_segment = Mock()
        self.node.retry_when_planner_starts()
        self.node.start_segment.assert_called_once()

    def test_old_graph_path_timestamp_is_rejected(self):
        self.node.on_memory_goal(pose(20., 0.))
        old = route((0., 0.), (20., 0.))
        old.header.stamp.sec = old.header.stamp.nanosec = 0
        self.node.start_segment = Mock()
        self.node.on_route(old)
        self.node.start_segment.assert_not_called()

    def test_new_query_can_resume_frontier_route_without_object_waypoint(self):
        self.node.on_query(String(data="trash can"))
        self.node.start_segment = Mock()
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_not_called()
        self.node.query_grace_until = 0.0
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_called_once()

    def test_current_remembered_viewpoint_does_not_request_zero_length_path(self):
        self.node.on_memory_goal(pose(0.5, 0.))
        self.assertEqual(self.node.last_status["status"], "already_at_goal")
        self.assertTrue(self.node.last_status['requires_visual_confirmation'])
        self.node.waypoint_pub.publish.assert_not_called()

    def test_graph_reset_keeps_memory_arrival_visual_confirmation_required(self):
        self.send_views(self.memory_views(.5))
        self.assertEqual(self.node.last_status['status'], 'already_at_goal')
        self.assertTrue(self.node.last_status['requires_visual_confirmation'])

    def route_status(self, path, *, goal=(40., 0.), complete=False):
        stamp = path.header.stamp.sec * 10**9 + path.header.stamp.nanosec
        return String(data=json.dumps(dict(stamp_ns=stamp, goal=list(goal), complete=complete,
                                           mode='known' if complete else 'frontier')))

    def test_partial_route_waits_for_correlated_metadata_then_starts(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        path = route((0., 0.), (8., 0.))
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()
        self.node.on_route_status(self.route_status(path))
        self.node.start_segment.assert_called_once()
        self.assertFalse(self.node.route_complete)
        self.assertNotIn(self.node.expected, self.node.current_route_points())

    def test_metadata_for_old_goal_cannot_accept_a_new_route(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        path = route((0., 0.), (8., 0.))
        self.node.on_route_status(self.route_status(path, goal=(15., 0.)))
        self.node.on_route(path)
        self.node.start_segment.assert_not_called()

    def test_partial_nearby_route_does_not_force_full_memory_target(self):
        self.send_views(self.memory_views(15.))
        path = route((0., 0.), (8., 0.))
        self.node.require_route_status = True
        self.node.on_route_status(self.route_status(path, goal=(15., 0.)))
        self.node.on_route(path)
        self.assertFalse(self.node.memory_full_check)
        self.assertTrue(self.node.candidates)
        self.assertTrue(all(c[0] <= 8. for c in self.node.candidates))

    def test_latest_partial_metadata_does_not_change_active_segment(self):
        self.node.require_route_status = True
        self.node.expected = (40., 0.)
        self.node.start_segment = Mock()
        active = route((0., 0.), (40., 0.))
        self.node.on_route_status(self.route_status(active, complete=True))
        self.node.on_route(active)
        self.node.following = True
        newer = route((0., 0.), (8., 0.))
        self.node.on_route_status(self.route_status(newer))
        self.node.on_route(newer)
        self.assertTrue(self.node.route_complete)
        self.assertFalse(self.node.latest_route_complete)
        self.assertIs(self.node.route, active)

    def test_selected_memory_prefix_goes_directly_to_follow_path(self):
        self.send_views(self.memory_views(40.))
        self.node.follow_enabled = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.on_route(route((0., 0.), (8., 0.), (40., 0.)))
        # A valid prefix is the first local option for this long graph route.
        target = self.node.candidates[self.node.candidate_index][0]
        self.valid_plan(target)
        self.assertFalse(self.node.memory_selecting)
        self.node.follow_client.send_goal_async.assert_called_once()
        sent = self.node.follow_client.send_goal_async.call_args.args[0].path
        self.assertAlmostEqual(sent.poses[-1].pose.position.x, target)
        self.assertEqual(self.node.memory_goal, (40., 0.))

    def test_selected_memory_path_is_rejected_after_map_changes(self):
        self.send_views(self.memory_views(12.))
        path = route((0., 0.), (12., 0.))
        self.node.route = path
        self.node.memory_matches = [dict(view=self.node.memory_active, cost=12., heading=0.,
            complete=True, path=path, route=path, target=(12., 0.), route_complete=True)]
        self.node.grid.data[10*40+18] = 100
        self.node.finish_memory_selection()
        self.node.path_pub.publish.assert_not_called()
        self.assertEqual(self.node.last_status['status'], 'path_changed_before_execution')
        self.assertEqual(self.node.memory_goal, (12., 0.))

    def confirmation(self, **changes):
        data = dict(query='gas station', frame='spot1/odom',
            observation_stamp_ns=self.node.get_clock().now().nanoseconds,
            object_position=[20., 0., 0.], observer_position=[12., 0., 0.],
            observer_orientation=[0., 0., 0., 1.], radius=.5, inliers=4, observations=5,
            search_confirmed=True, standoff=[15., 0.])
        data.update(changes)
        return String(data=json.dumps(data))

    def test_memory_object_is_found_from_new_observer_position(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_goal = self.node.expected = (0., 0.)
        self.node.memory_active = dict(object_position=[20.3, 0., 0.], object_radius=.6)
        self.node.odom.pose.pose.position.x = 13.5
        self.node.on_object_confirmation(self.confirmation())
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')
        self.assertAlmostEqual(self.node.last_status['standoff_distance_m'], 1.5)
        self.assertEqual(self.node.last_status['standoff_tolerance_m'], 1.5)

    def test_nearest_signal_hint_rejects_farther_candidate_but_allows_closer_one(self):
        self.node.on_query(String(data='traffic light'))
        self.node.query_reset_pending = False
        self.node.reset = Mock()
        self.node.relay_destination = Mock()

        # The initial standoff waypoint is four metres from the nearest known
        # signal and faces it. It must become the query's distance baseline.
        hint = pose(64., 0.)
        hint.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_waypoint(hint)
        self.assertEqual(self.node.expected, (64., 0.))
        self.assertAlmostEqual(self.node.nearest_signal_distance, 68.)
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 68.)
        self.assertAlmostEqual(self.node.nearest_signal_position[1], 0.)

        # A visually supported but farther signal cannot steal that route.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[85., 0., 6.],
            observer_position=[0., 0., 0.], standoff=[81., 0.]))
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 68.)
        self.assertEqual(self.node.expected, (64., 0.))
        self.node.reset.assert_called_once()

        # Even if the vehicle is physically beside that farther light, it
        # cannot finish a query whose selected nearest instance is elsewhere.
        self.node.odom.pose.pose.position.x = 85.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[85., 0., 6.],
            observer_position=[85., 0., 0.], standoff=[85., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.expected, (64., 0.))

        # A confirmed instance with a meaningful distance advantage can replace it.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[64.8, 0., 6.],
            observer_position=[0., 0., 0.], standoff=[60.8, 0.]))
        self.assertAlmostEqual(self.node.nearest_signal_position[0], 64.8)
        self.assertEqual(self.node.expected, (60.8, 0.))
        self.assertEqual(self.node.reset.call_count, 2)

    def test_visible_target_status_does_not_mask_bounded_stall_retry(self):
        self.node.follow_enabled = True
        self.node.plan_client = Mock()
        self.node.plan_client.wait_for_server.return_value = True
        self.node.follow_client = Mock()
        self.node.follow_client.wait_for_server.return_value = True
        self.node.route = route((0., 0.), (10., 0.))
        self.node.route_endpoint = self.node.expected = (10., 0.)
        self.node.blocked_endpoint = (10., 0.)
        self.node.blocked_until = time.monotonic()-1.
        self.node.last_status = dict(status='object_visible_not_arrived')
        self.node.start_segment = Mock()

        self.node.retry_when_planner_starts()

        self.node.start_segment.assert_called_once()
        self.assertEqual(self.node.recovery_attempts, 1)
        self.node.start_segment = Mock()
        self.node.on_route(route((12., 0.), (0., 0.)))
        self.node.start_segment.assert_not_called()

    def test_confirmation_before_standoff_keeps_frontier_route_active(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.expected = (10., 0.)
        self.node.query_grace_until = 0.
        self.node.start_segment = Mock()

        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))

        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_before_standoff')
        self.node.on_route(route((0., 0.), (10., 0.)))
        self.node.start_segment.assert_called_once()

    def test_visible_distant_signal_cannot_finish_even_at_clamped_standoff(self):
        self.node.on_query(String(data='traffic light'))
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[20., 0., 6.], standoff=[0., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_visible_not_arrived')
        self.assertEqual(self.node.last_status['distance'], 20.)

    def test_signal_can_finish_within_five_metres_horizontal_distance(self):
        self.node.on_query(String(data='traffic light'))
        self.node.odom.pose.pose.position.x = 16.
        self.node.on_object_confirmation(self.confirmation(
            query='traffic light', object_position=[20., 0., 6.], standoff=[16., 0.]))
        self.assertTrue(self.node.object_found)

    def test_confirmation_is_accepted_after_vehicle_reaches_standoff(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.odom.pose.pose.position.x = 8.5
        self.node.memory_active = None
        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')

    def test_confirmation_outside_standoff_tolerance_is_not_arrival(self):
        self.node.on_query(String(data='crosswalk'))
        self.node.odom.pose.pose.position.x = 8.49
        self.node.memory_active = None
        self.node.on_object_confirmation(self.confirmation(
            query='crosswalk', object_position=[20., 0., 0.], standoff=[10., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_before_standoff')
        self.assertAlmostEqual(self.node.last_status['distance'], 1.51)

    def test_passive_memory_cannot_confirm_a_different_building_block(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[711.7356, -27.7877, 51.917],
            object_radius=1.28, evidence='passive_semantic_landmarks')
        self.node.on_object_confirmation(self.confirmation(
            object_position=[703.1474, -48.0556, 52.9403],
            observer_position=[706.0734, -16.8977, 52.0676], radius=.88,
            inliers=5, observations=5, standoff=[0., 0.]))
        self.assertFalse(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'confirmation_outside_memory_region')

    def test_passive_memory_accepts_another_facade_near_the_remembered_region(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[711.5024, -20.1252, 51.906],
            object_radius=.90, evidence='passive_semantic_landmarks')
        self.node.on_object_confirmation(self.confirmation(
            object_position=[712.3733, -24.6344, 53.8417], radius=.48,
            inliers=5, observations=5, standoff=[0., 0.]))
        self.assertTrue(self.node.object_found)
        self.assertEqual(self.node.last_status['status'], 'object_found')

    def test_confirmation_rejects_wrong_query_stale_stamp_and_other_instance(self):
        self.node.on_query(String(data='gas station'))
        self.node.memory_active = dict(object_position=[20., 0., 0.], object_radius=.5)
        for changes in (dict(query='house'), dict(observation_stamp_ns=self.node.query_started_ns-1),
                        dict(object_position=[100., 0., 0.]), dict(radius=float('nan'))):
            self.node.on_object_confirmation(self.confirmation(**changes))
            self.assertFalse(self.node.object_found)

    def test_new_query_releases_found_latch(self):
        self.node.on_query(String(data='gas station'))
        self.node.on_object_confirmation(self.confirmation(standoff=[0., 0.]))
        self.assertTrue(self.node.object_found)
        self.node.on_query(String(data='house'))
        self.assertFalse(self.node.object_found)




    def test_new_partial_prefix_can_resume_after_nearby_frontier_was_reached(self):
        self.node.expected = (40., 0.)
        self.node.last_plan_start = (7., 0.)
        self.node.last_plan_endpoint = (8., 0.)
        self.node.odom.pose.pose.position.x = 7.5
        self.node.status('route_done')
        self.node.start_segment = Mock()
        path = route((7.5, 0.), (10., 0.))
        self.node.require_route_status = True
        self.node.on_route_status(self.route_status(path))
        self.node.on_route(path)
        self.node.start_segment.assert_called_once()
        self.assertFalse(self.node.route_complete)

    def test_old_unlabelled_waypoint_cannot_change_new_query_destination(self):
        self.node.on_query(String(data='house'))
        late = pose(15., 0.)
        late.header.stamp.sec, late.header.stamp.nanosec = divmod(self.node.query_started_ns-1, 10**9)
        self.node.on_waypoint(late)
        self.assertIsNone(self.node.expected)
        self.node.waypoint_pub.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
