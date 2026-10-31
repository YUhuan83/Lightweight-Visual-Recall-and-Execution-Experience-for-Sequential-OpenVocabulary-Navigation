"""Actual-pose contract for first, replacement and cusp FollowPath dispatch."""
import math
import unittest
from unittest.mock import Mock, patch
import test_graph_path_bridge_routing as routing
route = routing.route


class InitialPathJoinTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        routing.GraphBridgeRoutingTests.setUpClass()

    @classmethod
    def tearDownClass(cls):
        routing.GraphBridgeRoutingTests.tearDownClass()

    def setUp(self):
        self.fixture = routing.GraphBridgeRoutingTests('test_follow_goal_ends_at_cusp_using_strict_goal_checker')
        self.fixture.setUp()
        self.node = self.fixture.node
        self.fixture.prepare_reverse_forward_execution()
        self.node.follow_client.send_goal_async.reset_mock()
        self.node.following = False
        self.node.follow_failed = Mock()
        self.node.extension_failed = Mock()
        self.node.start_segment = Mock()
        self.node.motion_queue = [route((.5, 0.), (6., 0.))]

    def tearDown(self):
        self.fixture.tearDown()

    def bad_heading(self):
        self.node.odom.pose.pose.orientation.z = math.sin(math.pi/4)
        self.node.odom.pose.pose.orientation.w = math.cos(math.pi/4)

    def test_first_path_rejects_heading_gap_before_follow_action(self):
        self.bad_heading()
        self.node.send_motion_segment()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.follow_failed.assert_called_once_with('motion_section_invalid',
            detail='actual_pose_cannot_join', join_failure_reason='cusp_heading_outside_tolerance')
        self.node.extension_failed.assert_not_called()
        self.node.start_segment.assert_not_called()

    def test_replacement_rejection_preserves_extension_failure_ownership(self):
        self.bad_heading()
        existing = self.node.motion_queue
        self.node.send_motion_segment(replacement=[route((.5, 0.), (8., 0.))])
        self.node.extension_failed.assert_called_once_with('actual_pose_cannot_join')
        self.node.follow_failed.assert_not_called()
        self.assertIs(self.node.motion_queue, existing)
        self.node.follow_client.send_goal_async.assert_not_called()

    def test_exact_pose_and_small_lattice_gap_still_dispatch(self):
        for gap in (0., .025):
            self.node.motion_queue = [route((gap, 0.), (6., 0.))]
            self.node.following = False
            self.node.send_motion_segment()
        self.assertEqual(self.node.follow_client.send_goal_async.call_count, 2)
        self.node.follow_failed.assert_not_called()

    def test_first_path_join_sweep_is_checked_on_both_maps(self):
        self.node.motion_queue = [route((0., 0.), (6., 0.))]
        self.node.validate_local_path = Mock(return_value=None)
        with patch('wildos_nav2_adapter.graph_path_bridge.validate_local_prefix',
                   side_effect=[None, 'footprint_collision']) as swept:
            self.node.send_motion_segment()
        self.assertEqual(swept.call_count, 2)
        self.assertIs(swept.call_args_list[0].args[2], self.node.grid)
        self.assertIs(swept.call_args_list[1].args[2], self.node.local_grid)
        self.node.follow_client.send_goal_async.assert_not_called()
        self.node.follow_failed.assert_called_once()

    def test_cusp_gap_keeps_cusp_replanning_owner(self):
        self.bad_heading()
        self.node.send_motion_segment(after_maneuver=True)
        self.assertEqual(self.node.last_status['status'], 'replanning_after_maneuver')
        self.node.start_segment.assert_called_once()
        self.node.follow_failed.assert_not_called()


if __name__ == '__main__':
    unittest.main()
