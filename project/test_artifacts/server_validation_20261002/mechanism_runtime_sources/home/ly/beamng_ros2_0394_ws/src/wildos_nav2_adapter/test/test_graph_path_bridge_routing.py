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
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GraphPathBridge()
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

    def tearDown(self):
        self.node.destroy_node()

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
        self.node.path_pub.publish.assert_not_called()

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

    def test_unreachable_memory_releases_old_graph_goal(self):
        self.send_views(self.memory_views(12.))
        self.node.on_route(route((0.,0.),(12.,0.)))
        self.node.candidate_failed('empty_path')
        self.assertIsNone(self.node.memory_goal)
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.graph_reset_client.call_async.call_count, 2)
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unreachable')

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
        self.node.odom.pose.pose.position.x = -1.75
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

    def test_costmap_change_at_cusp_rejects_next_section(self):
        self.prepare_reverse_forward_execution()
        self.node.odom.pose.pose.orientation.w = 1.
        self.node.odom.pose.pose.position.x = -2.
        self.node.grid.data[10 * 40 + 14] = 100
        future = Mock()
        future.result.return_value.status = 4
        self.node.on_follow_result(future, self.node.epoch, (-2., 0.))
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 1)
        self.assertEqual(self.node.last_status['reason'], 'motion_section_invalid')
        self.assertEqual(self.node.motion_queue, [])

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


if __name__ == "__main__":
    unittest.main()
