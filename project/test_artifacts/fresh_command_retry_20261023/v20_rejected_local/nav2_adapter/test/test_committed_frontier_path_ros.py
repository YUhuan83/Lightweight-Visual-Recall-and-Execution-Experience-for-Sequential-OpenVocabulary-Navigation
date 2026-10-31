"""A chosen vehicle path crosses the graph commit only with fresh matching evidence."""
from copy import deepcopy
import json
import math
import time
import unittest
from unittest.mock import Mock
import rclpy
from std_msgs.msg import String
from wildos_nav2_adapter.input_quality import stamp_ns
import test_exploration_cost_ros as comparison
from test_graph_path_bridge_routing import drivable_route


class CommittedFrontierPathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fixture = comparison.ExplorationCostTests()
        self.fixture.setUp()
        self.node = self.fixture.node

    def tearDown(self):
        self.fixture.tearDown()

    def selected_route(self, **changes):
        n = self.node
        path = drivable_route((0., 0.), (14., 0.))
        path.header.stamp = n.get_clock().now().to_msg()
        path.header.stamp.nanosec += 1_000_000  # keep route metadata distinct from the old owner
        if path.header.stamp.nanosec >= 1_000_000_000:
            path.header.stamp.sec += 1
            path.header.stamp.nanosec -= 1_000_000_000
        metadata = dict(exploration_id='b:2', complete=True, goal=[14., 0.],
                        observation_heading=0., vehicle_cost_selected=True)
        metadata.update(changes)
        n.route_metadata[stamp_ns(path)] = metadata
        return path

    def test_matching_committed_route_reuses_checked_winner_without_new_search(self):
        self.fixture.compare()
        n = self.node
        sends = n.plan_client.send_goal_async.call_count
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_called_once()
        selected, target = n.execute_checked_path.call_args.args
        self.assertEqual(tuple(target), (14., 0.))
        self.assertEqual(selected.poses[-1].pose.position.x, 14.)
        self.assertEqual(n.plan_client.send_goal_async.call_count, sends)
        self.assertEqual(n.last_plan_endpoint, (14., 0.))
        n.full_path_pub.publish.assert_called_once()
        self.assertEqual([c[:2] for c in n.candidates], [(14., 0.)])

    def test_dispatch_failure_cannot_resume_original_frontier_candidates(self):
        self.fixture.compare()
        n = self.node
        n.try_planning_escape = Mock(return_value=False)
        n.reject_exploration_route = Mock()
        n.execute_checked_path.side_effect = lambda *_: n.candidate_failed('motion_section_invalid')
        sends = n.plan_client.send_goal_async.call_count
        n.on_route(self.selected_route())
        self.assertEqual(n.plan_client.send_goal_async.call_count, sends)
        self.assertEqual(n.exploration_destination(), (14., 0.))
        self.assertEqual([c[:2] for c in n.candidates], [(14., 0.)])

    def retire_original(self, reason='vehicle_cost_selection', **changes):
        data = dict(exploration_id='a:1', reason=reason,
                    stamp_ns=self.node.get_clock().now().nanoseconds)
        data.update(changes)
        self.node.on_exploration_retired(String(data=json.dumps(data)))

    def test_actual_old_owner_retirement_preserves_only_pending_choice(self):
        self.fixture.compare()
        n = self.node
        old_epoch = n.epoch
        self.retire_original()
        self.assertIsNone(n.route)
        self.assertGreater(n.epoch, old_epoch)
        n.execute_checked_path.assert_not_called()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_called_once()
        self.assertEqual(n.plan_client.send_goal_async.call_count, 2)

    def test_duplicate_retirement_does_not_execute_or_lose_pending_choice(self):
        self.fixture.compare()
        self.retire_original()
        self.retire_original()
        self.node.execute_checked_path.assert_not_called()
        self.node.on_route(self.selected_route())
        self.node.execute_checked_path.assert_called_once()

    def test_other_retirement_reason_cannot_retain_a_choice(self):
        self.fixture.compare()
        self.retire_original(reason='graph_path_unavailable')
        self.assertIsNone(self.node.frontier_choice)

    def test_expired_choice_cannot_survive_retirement(self):
        self.fixture.compare()
        self.node.frontier_choice['started'] -= 6.
        self.retire_original()
        self.assertIsNone(self.node.frontier_choice)

    def test_an_active_motion_cannot_retain_comparison_across_retirement(self):
        self.fixture.compare()
        self.node.following = True
        self.retire_original()
        self.assertIsNone(self.node.frontier_choice)

    def test_retirement_older_than_offer_cannot_retain_a_choice(self):
        choice = self.fixture.compare()
        self.retire_original(stamp_ns=stamp_ns(choice)-1)
        self.assertIsNone(self.node.frontier_choice)

    def test_query_reset_between_retirement_and_commit_cannot_retain_a_choice(self):
        self.fixture.compare()
        self.retire_original()
        path = self.selected_route()
        self.node.on_query(String(data='gas station'))
        self.retire_original()
        self.node.on_route(path)
        self.assertIsNone(self.node.frontier_choice)
        self.node.execute_checked_path.assert_not_called()

    def test_new_owner_route_received_while_choice_waits_survives_resolved_retirement(self):
        self.fixture.compare()
        n = self.node
        replacement = self.selected_route(exploration_id='c:3', vehicle_cost_selected=False)
        n.on_route(replacement)
        self.assertIs(n.latest_route, replacement)
        self.retire_original(reason='frontier_resolved')
        self.assertEqual(stamp_ns(n.route), stamp_ns(replacement))
        self.assertIsNone(n.frontier_choice)
        n.execute_checked_path.assert_not_called()

    def test_new_owner_route_deferred_during_comparison_survives_retirement(self):
        n = self.node
        n.start_segment()
        self.fixture.response(drivable_route((0., 0.), (8., 0.), (3., 0.), (12., 0.)))
        replacement = self.selected_route(exploration_id='c:3', vehicle_cost_selected=False)
        n.on_route(replacement)
        self.assertIs(n.latest_route, replacement)
        self.retire_original(reason='frontier_resolved')
        self.assertEqual(stamp_ns(n.route), stamp_ns(replacement))
        n.execute_checked_path.assert_not_called()

    def test_route_from_retired_owner_is_not_restored_by_reset(self):
        n = self.node
        same_owner_route = self.selected_route(exploration_id='a:1',
                                               vehicle_cost_selected=False)
        n.on_route(same_owner_route)
        self.retire_original(reason='frontier_resolved')
        self.assertIsNone(n.route)
        self.assertIsNone(n.latest_route)
        n.execute_checked_path.assert_not_called()

    def test_stale_replacement_route_is_not_restored(self):
        n = self.node
        replacement = self.selected_route(exploration_id='c:3',
                                          vehicle_cost_selected=False)
        replacement.header.stamp.sec -= 6
        n.latest_route = replacement
        self.retire_original(reason='frontier_resolved')
        self.assertIsNone(n.route)
        self.assertIsNone(n.latest_route)

    def test_query_fenced_replacement_route_is_not_restored(self):
        n = self.node
        replacement = self.selected_route(exploration_id='c:3',
                                          vehicle_cost_selected=False)
        n.latest_route = replacement
        n.query_reset_pending = True
        self.assertIsNone(n.replacement_route_for_retirement('a:1'))

    def test_replacement_route_computes_while_execution_waits_for_stop_barrier(self):
        n = self.node
        replacement = self.selected_route(exploration_id='c:3',
                                          vehicle_cost_selected=False)
        n.latest_route = replacement
        n.following = True
        n.active_follow_serial = 7
        n.follow_active = dict(serial=7, handle=Mock())
        n.follow_handle = n.follow_active['handle']
        self.retire_original(reason='frontier_resolved')
        self.assertEqual(stamp_ns(n.route), stamp_ns(replacement))
        self.assertIn(7, n.follow_quarantine)
        self.assertTrue(n.follow_hold)
        n.plan_client.send_goal_async.assert_called_once()
        n.follow_client.send_goal_async.assert_not_called()
        n.execute_checked_path.assert_not_called()

    def test_same_uuid_changed_goal_replans_instead_of_using_old_path(self):
        self.fixture.compare()
        self.node.start_segment = Mock()
        self.node.on_route(self.selected_route(goal=[14.5, 0.]))
        self.node.execute_checked_path.assert_not_called()
        self.node.start_segment.assert_called_once()

    def test_changed_observation_heading_requires_new_vehicle_plan(self):
        self.fixture.compare()
        self.node.start_segment = Mock()
        self.node.on_route(self.selected_route(observation_heading=math.pi / 2))
        self.node.execute_checked_path.assert_not_called()
        self.node.start_segment.assert_called_once()

    def test_new_obstacle_on_winner_falls_back_to_normal_planning(self):
        self.fixture.compare()
        n = self.node
        n.grid.data[10 * 40 + 16] = 100
        n.start_segment = Mock()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_pose_moved_after_evaluation_discards_retained_path(self):
        self.fixture.compare()
        n = self.node
        n.odom.pose.pose.position.x = .4
        n.start_segment = Mock()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_stale_local_map_does_not_execute_retained_path(self):
        self.fixture.compare()
        n = self.node
        n.local_freshness_issue = Mock(return_value='stale_local_costmap')
        n.start_segment = Mock()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_transport_fault_does_not_execute_retained_path(self):
        self.fixture.compare()
        n = self.node
        n.execution_transport_issue = Mock(return_value='source_expired')
        n.start_segment = Mock()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_late_matching_route_after_choice_deadline_replans(self):
        self.fixture.compare()
        n = self.node
        n.frontier_choice['started'] -= 6.
        n.start_segment = Mock()
        n.on_route(self.selected_route())
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_selected_route_for_other_uuid_cannot_release_choice_or_execute(self):
        self.fixture.compare()
        n = self.node
        waiting = n.frontier_choice
        n.on_route(self.selected_route(exploration_id='c:3'))
        self.assertIs(n.frontier_choice, waiting)
        n.execute_checked_path.assert_not_called()

    def test_duplicate_committed_publication_does_not_execute_winner_twice(self):
        self.fixture.compare()
        n = self.node
        path = self.selected_route()
        n.on_route(path)
        n.on_route(path)
        n.execute_checked_path.assert_called_once()

    def test_query_reset_cannot_reuse_previous_winner(self):
        self.fixture.compare()
        n = self.node
        path = self.selected_route()
        n.on_query(String(data='gas station'))
        n.on_route(path)
        n.execute_checked_path.assert_not_called()
        self.assertIsNone(n.frontier_choice)

    def test_partial_selected_route_cannot_reuse_complete_path(self):
        self.fixture.compare()
        n = self.node
        n.start_segment = Mock()
        n.on_route(self.selected_route(complete=False))
        n.execute_checked_path.assert_not_called()
        n.start_segment.assert_called_once()

    def test_integer_json_options_are_normalized_for_typed_ros_pose(self):
        n = self.node
        metadata = n.route_metadata[stamp_ns(n.route)]
        metadata['goal'] = [12, 0]
        metadata['vehicle_options'][0]['goal'] = [12, 0, 0]
        metadata['vehicle_options'][1]['goal'] = [14, 0, 0]
        n.start_segment()
        request = n.plan_client.send_goal_async.call_args.args[0]
        self.assertIs(type(request.goal.pose.position.x), float)
        self.assertIs(type(request.goal.pose.position.y), float)

    def test_integer_normal_candidate_uses_same_typed_boundary(self):
        n = self.node
        n.candidates = [(12, 0, 0, 12)]
        n.candidate_index = 0
        n.try_candidate()
        request = n.plan_client.send_goal_async.call_args.args[0]
        self.assertIs(type(request.goal.pose.position.x), float)
        self.assertIs(type(request.goal.pose.position.y), float)


if __name__ == '__main__':
    unittest.main()
