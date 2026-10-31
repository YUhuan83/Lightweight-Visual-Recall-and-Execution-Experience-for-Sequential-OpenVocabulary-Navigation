"""Review regressions for new owner handoff and vehicle no-path cache scope."""
from concurrent.futures import Future
from copy import deepcopy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import rclpy
import test_graph_path_bridge_routing as fixtures
from wildos_nav2_adapter.input_quality import stamp_ns


class ExecutionContinuityReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fixture = fixtures.ExplorationTargetHandoffTests()
        self.fixture.setUp()
        self.node = self.fixture.node

    def tearDown(self):
        self.fixture.tearDown()

    def retired_reverse(self):
        n = self.node
        self.fixture.accepted_short_section()
        n.motion_queue = [fixtures.drivable_route((0., 0.), (-1., 0.)),
                          fixtures.drivable_route((-1., 0.), (8., 0.))]
        self.fixture.resolve_frontier()
        self.assertIsNotNone(n.retired_motion)
        self.assertFalse(n.retired_motion['forward'])
        n.send_motion_segment = Mock()
        n.start_segment = Mock()

    def offer_replacement(self):
        new = fixtures.drivable_route((-1., 0.), (12., 0.))
        self.fixture.metadata(new, 'frontier-b:2', (12., 0.))
        self.node.on_route(new)
        return new

    def finish_reverse(self):
        self.node.odom.pose.pose.position.x = -1.
        result = Future()
        result.set_result(SimpleNamespace(status=4))
        self.node.on_follow_result(result, self.node.epoch, (-1., 0.), 7)

    def test_reverse_terminal_revalidates_tail_for_already_received_new_owner(self):
        self.retired_reverse()
        old_route = self.node.route
        new = self.offer_replacement()
        # This is the real callback order: the new route is deferred, and a
        # reverse section cannot start a forward continuous replacement.
        self.node.maybe_extend_path()
        self.assertIs(self.node.route, old_route)
        self.node.send_motion_segment.assert_not_called()
        self.finish_reverse()
        self.node.start_segment.assert_not_called()
        self.node.send_motion_segment.assert_called_once()
        self.assertEqual(self.node.route.header.frame_id, new.header.frame_id)
        self.assertEqual(stamp_ns(self.node.route), stamp_ns(new))
        self.assertEqual(self.node.motion_queue[-1].poses[-1].pose.position.x, 8.)
        self.node.route_observation_pub.publish.assert_not_called()

    def test_new_obstacle_in_old_tail_requires_new_planning(self):
        self.retired_reverse()
        self.offer_replacement()
        self.node.grid.data[10 * 40 + 15] = 100  # x=5 m, ahead in old tail
        self.finish_reverse()
        self.node.send_motion_segment.assert_not_called()
        self.node.start_segment.assert_called_once()

    def test_forward_terminal_also_admits_latest_owner_before_next_timer(self):
        n = self.node
        self.fixture.accepted_short_section()
        self.fixture.resolve_frontier()
        new = fixtures.drivable_route((6., 0.), (16., 0.))
        self.fixture.metadata(new, 'frontier-b:2', (16., 0.))
        n.on_route(new)
        n.start_segment = Mock()
        n.send_motion_segment = Mock()
        n.odom.pose.pose.position.x = 6.
        self.fixture.complete_section()
        n.send_motion_segment.assert_called_once()
        n.start_segment.assert_not_called()
        self.assertEqual(n.motion_queue[-1].poses[-1].pose.position.x, 12.)

    def test_expired_new_owner_cannot_authorize_old_tail(self):
        self.retired_reverse()
        new = self.offer_replacement()
        new.header.stamp.sec -= 6
        self.finish_reverse()
        self.node.send_motion_segment.assert_not_called()

    def test_without_new_owner_old_tail_remains_retired(self):
        self.retired_reverse()
        self.finish_reverse()
        self.node.send_motion_segment.assert_not_called()
        self.assertEqual(self.node.motion_queue, [])

    def test_empty_path_cache_retries_after_start_heading_changes(self):
        n = self.node
        n.sync_failed_goal_scope('frontier-a:1')
        n.remember_failed_goal((0., 0.), (10., 0.), 0.)
        self.assertTrue(n.goal_recently_failed((0., 0.), (10., 0.), 0.))
        n.odom.pose.pose.orientation.z = math.sin(math.pi / 4)
        n.odom.pose.pose.orientation.w = math.cos(math.pi / 4)
        self.assertFalse(n.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_empty_path_cache_retries_after_costmap_content_changes(self):
        n = self.node
        n.sync_failed_goal_scope('frontier-a:1')
        n.remember_failed_goal((0., 0.), (10., 0.), 0.)
        n.grid = deepcopy(n.grid)
        n.grid.data[10 * 40 + 15] = 100
        self.assertFalse(n.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_republished_identical_map_does_not_restart_failed_search(self):
        n = self.node
        n.remember_failed_goal((0., 0.), (10., 0.), 0.)
        n.grid = deepcopy(n.grid)
        n.grid.header.stamp.sec += 1
        self.assertTrue(n.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_shifted_costmap_origin_retries_with_same_cell_values(self):
        n = self.node
        n.remember_failed_goal((0., 0.), (10., 0.), 0.)
        n.grid = deepcopy(n.grid)
        n.grid.info.origin.position.x += 1.
        self.assertFalse(n.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_late_empty_result_keeps_request_start_heading(self):
        n = self.node
        n.candidates = [(10., 0., 0., 10.)]
        n.candidate_index = 0
        n.plan_client = Mock()
        n.plan_client.send_goal_async.return_value = Future()
        n.try_candidate()
        n.odom.pose.pose.orientation.z = math.sin(math.pi / 4)
        n.odom.pose.pose.orientation.w = math.cos(math.pi / 4)
        n.try_candidate = Mock()
        n.candidate_failed('empty_path')
        self.assertFalse(n.goal_recently_failed((0., 0.), (10., 0.), 0.))
        n.odom.pose.pose.orientation.z = 0.
        n.odom.pose.pose.orientation.w = 1.
        self.assertTrue(n.goal_recently_failed((0., 0.), (10., 0.), 0.))

    def test_map_change_during_request_does_not_assign_failure_to_either_map(self):
        n = self.node
        n.candidates = [(10., 0., 0., 10.)]
        n.candidate_index = 0
        n.plan_client = Mock()
        n.plan_client.send_goal_async.return_value = Future()
        n.try_candidate()
        n.grid = deepcopy(n.grid)
        n.grid.data[10 * 40 + 15] = 100
        n.try_candidate = Mock()
        n.candidate_failed('empty_path')
        self.assertEqual(n.failed_goals, {})


if __name__ == '__main__':
    unittest.main()
