import json
import math
import time
import unittest
from concurrent.futures import Future
from copy import deepcopy
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import rclpy
from std_msgs.msg import String
import test_graph_path_bridge_routing as fixtures
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
from wildos_nav2_adapter.input_quality import stamp_ns


class ContinuousExplorationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        with patch.object(fixtures,'GraphPathBridge',ContinuousGraphPathBridge):
            fixtures.GraphBridgeRoutingTests.setUp(self)
        self.addCleanup(self.node.destroy_node)

    def route(self, endpoint, identity):
        path=fixtures.route((0.,0.),endpoint)
        self.node.route_metadata[stamp_ns(path)]=dict(exploration_id=identity,
            complete=True,goal=list(endpoint),mode='exploration')
        return path

    def test_nominal_radius_is_declared_at_startup_but_cannot_change_at_runtime(self):
        n=self.node
        self.assertIsNotNone(n.get_parameter('planner_minimum_turning_radius').value)
        result=n.on_parameters([S(name='planner_minimum_turning_radius',value=16.)])
        self.assertFalse(result.successful)
        self.assertIn('restart bridge',result.reason)

    def moving(self, reverse=False):
        n=self.node;n.follow_enabled=True;n.following=True;n.query='green residential house'
        n.odom.pose.pose.orientation.w=1.;n.odom.twist.twist.linear.x=-1.2 if reverse else 1.2
        endpoint=(-4.,0.) if reverse else (12.,0.)
        n.route=n.latest_route=self.route(endpoint,'frontier-A')
        n.route_endpoint=endpoint;n.route_complete=n.latest_route_complete=True
        n.motion_queue=[fixtures.drivable_route((0.,0.),endpoint)]
        n.follow_handle=Mock();n.follow_request_serial=n.active_follow_serial=7
        n.follow_active=dict(serial=7,handle=n.follow_handle,target=endpoint,heading=0.,
            xy_tolerance=.6,yaw_tolerance=.35,started_at=time.monotonic(),feedback_at=time.monotonic(),endpoint_since=None)
        n.follow_client=Mock();n.follow_client.send_goal_async.return_value=Future()
        n.execution_transport_issue=Mock(return_value=None)
        n._desired_intent=n._planned_intent=n.intent_key(n.route)
        n._rolling_plan_start=(0.,0.);n._motion_route=deepcopy(n.route)
        n._motion_intent=n.intent_key(n.route);n._motion_serial=7;n._motion_epoch=n.epoch
        return n

    def retirement(self, reason='frontier_resolved'):
        return String(data=json.dumps(dict(exploration_id='frontier-A',
            stamp_ns=self.node.get_clock().now().nanoseconds,reason=reason)))

    def test_upstream_position_refinement_keeps_inflight_computation(self):
        n=self.moving();n.upstream_target_approach=True;n.expected=(20.,0.)
        key=n.intent_key(n.route)
        n._desired_intent=n._planned_intent=n._motion_intent=key
        n.pending=True
        n.discard_intent_plan=Mock()
        n.restrict_current_direction=Mock(return_value=True)
        n.maybe_extend_path=Mock()
        n.expected=(20.04,.02)
        message=fixtures.route((0.,0.),(20.04,.02))
        n.on_route(message)
        self.assertIs(n.latest_route,message)
        n.discard_intent_plan.assert_not_called()
        n.restrict_current_direction.assert_not_called()
        n.maybe_extend_path.assert_called_once()

    def test_upstream_query_change_still_changes_intent(self):
        n=self.node;n.upstream_target_approach=True;n.expected=(20.,0.)
        path=fixtures.route((0.,0.),(20.,0.));before=n.intent_key(path)
        n.query_generation+=1
        self.assertNotEqual(before,n.intent_key(path))

    def test_recall_viewpoint_identity_is_not_changed_by_upstream_mode(self):
        n=self.node;n.upstream_target_approach=True;n.expected=n.memory_goal=(20.,0.)
        path=fixtures.route((0.,0.),(20.,0.));before=n.intent_key(path)
        n.expected=n.memory_goal=(25.,0.)
        self.assertNotEqual(before,n.intent_key(path))

    def test_new_frontier_plans_while_old_action_continues(self):
        n=self.moving();handle=n.follow_handle;epoch=n.epoch
        n.start_segment=Mock()
        n.on_route(self.route((20.,0.),'frontier-B'))
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        self.assertEqual(n.epoch,epoch);self.assertFalse(n.follow_hold)
        handle.cancel_goal_async.assert_not_called()

    def test_new_nearby_intent_keeps_exact_destination_candidate(self):
        n=self.moving();handle=n.follow_handle
        n.plan_client=Mock();n.plan_client.send_goal_async.return_value=Future()
        n.on_route(self.route((14.,0.),'frontier-B'))
        request=n.plan_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(request.goal.pose.position.x,14.)
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        handle.cancel_goal_async.assert_not_called()

    def test_new_intent_can_shorten_current_action_after_normal_path_checks(self):
        n=self.moving()
        n.plan_client=Mock();n.plan_client.send_goal_async.return_value=Future()
        n.on_route(self.route((10.,0.),'frontier-B'))
        request=n.plan_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(request.goal.pose.position.x,10.)
        self.assertTrue(n.following)

    def test_same_intent_keeps_repeated_endpoint_filter(self):
        n=self.moving()
        candidates=[(14.,0.,0.,14.),(8.,0.,0.,8.)]
        self.assertEqual(n.extension_candidates(candidates,(12.,0.)),[candidates[1]])

    def test_old_motion_epoch_cannot_relax_endpoint_filter(self):
        n=self.moving();n.route=self.route((14.,0.),'frontier-B')
        n._motion_epoch=n.epoch-1
        candidates=[(14.,0.,0.,14.),(8.,0.,0.,8.)]
        self.assertEqual(n.extension_candidates(candidates,(12.,0.)),[candidates[1]])

    def test_unacknowledged_motion_cannot_relax_endpoint_filter(self):
        n=self.moving();n.route=self.route((14.,0.),'frontier-B')
        n.active_follow_serial=8
        candidates=[(14.,0.,0.,14.),(8.,0.,0.,8.)]
        self.assertEqual(n.extension_candidates(candidates,(12.,0.)),[candidates[1]])

    def test_same_frontier_republication_does_not_restart_motion_or_plan(self):
        n=self.moving();n.start_segment=Mock()
        n.on_route(self.route((12.,0.),'frontier-A'))
        n.start_segment.assert_not_called()
        self.assertEqual(n.active_follow_serial,7)

    def test_new_obstacle_starts_same_intent_compute_without_canceling_action(self):
        n=self.moving();handle=n.follow_handle;epoch=n.epoch
        n.full_path_execution=True
        n._controller_path=deepcopy(n.motion_queue[0])
        n.follow_active['path_stamp_ns']=stamp_ns(n._controller_path)
        n._controller_path_received_at=time.monotonic()
        # Vehicle's current body is free; its upcoming forward sweep is blocked.
        n.local_grid.data[10*n.local_grid.info.width+15]=100
        self.assertIsNone(n.active_window())
        n.start_segment=Mock()
        n.maybe_extend_path()
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertIs(n.follow_handle,handle);self.assertEqual(n.epoch,epoch)
        handle.cancel_goal_async.assert_not_called()
        self.assertEqual(n._desired_intent,n._planned_intent)

    def test_obstacle_update_does_not_duplicate_pending_compute(self):
        n=self.moving();n.local_grid.data[10*n.local_grid.info.width+15]=100
        n.pending=True;n.start_segment=Mock()
        n.maybe_extend_path();n.start_segment.assert_not_called()

    def test_obstacle_in_handoff_distance_replans_before_imminent_sweep_is_blocked(self):
        n=self.moving();handle=n.follow_handle
        # x=8 is beyond the current 2 m sweep including the front overhang.
        n.grid.data[10*n.grid.info.width+18]=100
        self.assertIsNotNone(n.active_window())
        n.start_segment=Mock();n.maybe_extend_path()
        n.start_segment.assert_called_once_with(continuous=True)
        handle.cancel_goal_async.assert_not_called()

    def test_local_obstacle_at_eight_metres_replans_without_canceling_motion(self):
        n=self.moving();handle=n.follow_handle
        n.local_grid.data[10*n.local_grid.info.width+18]=99
        self.assertIsNotNone(n.active_window())
        n.start_segment=Mock();n.maybe_extend_path()
        n.start_segment.assert_called_once_with(continuous=True)
        handle.cancel_goal_async.assert_not_called()

    def test_explicit_target_tries_bounded_forward_candidates_before_away_graph_exit(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((-12.,0.),'frontier-A');n.route_endpoint=(-12.,0.)
        n.candidates=[(-12.,0.,math.pi,12.)];n.vehicle_pose_feasible=Mock(return_value=True)
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][:2],(8.,0.))
        self.assertEqual(n.candidates[-1][:2],(-12.,0.))
        failed=n.candidates[0];n.remember_failed_goal((0.,0.),failed[:2],failed[2])
        self.assertNotIn(failed,n.target_forward_candidates((0.,0.)))
        n.memory_goal=(20.,0.)
        self.assertEqual(n.target_forward_candidates((0.,0.)),[])

    def test_explicit_target_without_graph_candidates_still_probes_forward(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[];n.vehicle_pose_feasible=Mock(return_value=True)
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][:2],(8.,0.))

    def test_single_empty_forward_plan_probes_sides_before_the_existing_graph_fallback(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[(20.,0.,0.,20.),(-12.,0.,math.pi,12.)];n.candidate_index=0
        n.active_plan_start=(0.,0.);n.vehicle_pose_feasible=Mock(return_value=True);n.try_candidate=Mock()
        n.candidate_failed('empty_path')
        self.assertEqual(n.candidates[1][:2],(8.,0.))
        self.assertIn((8.,-2.),[c[:2] for c in n.candidates])
        self.assertIn((8.,2.),[c[:2] for c in n.candidates])
        self.assertEqual(n.candidates[-1][:2],(-12.,0.))
        before=list(n.candidates);n.candidate_failed('empty_path')
        self.assertEqual(n.candidates,before)

    def test_local_ahead_rejection_is_not_cached_as_a_global_planner_failure(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[(20.,0.,0.,20.)];n.candidate_index=0;n.active_plan_start=(0.,0.)
        n.active_plan_failure_context=n.failed_goal_context()
        n.vehicle_pose_feasible=Mock(return_value=True);n.try_candidate=Mock()
        n.candidate_failed('local_ahead_local_collision_cell')
        # A new local observation can clear while the global map is unchanged.
        self.assertFalse(n.goal_recently_failed((0.,0.),(20.,0.),0.))
        n.carrier_model='ideal_point'
        self.assertEqual(n.target_forward_candidates((0.,0.)),[])

    def test_missing_execution_progress_does_not_start_obstacle_replan(self):
        n=self.moving();n.full_path_execution=True
        n.local_grid.data[10*n.local_grid.info.width+15]=100
        n.start_segment=Mock();n.maybe_extend_path()
        n.start_segment.assert_not_called()

    def test_frontier_retirement_does_not_cancel_acknowledged_motion(self):
        n=self.moving();handle=n.follow_handle;epoch=n.epoch
        n.on_exploration_retired(self.retirement())
        self.assertTrue(n.following);self.assertEqual(n.epoch,epoch)
        self.assertIs(n.follow_handle,handle);self.assertIsNone(n.retired_motion)
        self.assertEqual(n.follow_quarantine,{})
        handle.cancel_goal_async.assert_not_called()

    def test_retirement_invalidates_old_compute_response_without_motion_epoch_change(self):
        n=self.moving();n.pending=n.extending_path=True;n.active_plan_serial=9
        n.plan_handle=Mock();epoch=n.epoch
        n.on_exploration_retired(self.retirement())
        old=Mock();n.on_plan_result(old,epoch,0,9)
        old.result.assert_not_called()
        self.assertIsNone(n.active_plan_serial)
        self.assertEqual(n.active_follow_serial,7);self.assertTrue(n.following)

    def test_visible_intent_reuses_live_motion_without_reset(self):
        n=self.moving();epoch=n.epoch;handle=n.follow_handle
        n.relay_destination=Mock()
        n.visible_hint=dict(query=n.query,point=[7.,1.],bearing=.14,
                            observation_stamp_ns=n.get_clock().now().nanoseconds)
        self.assertTrue(n.activate_visible_approach())
        self.assertEqual(n.expected,(7.,1.));self.assertEqual(n.epoch,epoch)
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        self.assertFalse(n.follow_hold);n.relay_destination.assert_called_once()

    def test_new_intent_discards_old_unexecuted_reverse_tail(self):
        n=self.moving()
        n.motion_queue.append(fixtures.drivable_route((12.,0.),(10.,0.)))
        n.start_segment=Mock()
        n.on_route(self.route((20.,0.),'frontier-B'))
        self.assertEqual(len(n.motion_queue),1)
        n.start_segment.assert_called_once_with(continuous=True)

    def test_reverse_motion_finishes_before_any_forward_replacement(self):
        n=self.moving(reverse=True);n.start_segment=Mock()
        n.on_route(self.route((20.,0.),'frontier-B'))
        n.start_segment.assert_not_called()
        self.assertTrue(n.following);self.assertEqual(n.active_follow_serial,7)

    def test_failed_future_path_keeps_current_action_running(self):
        n=self.moving();handle=n.follow_handle
        n.pending=n.extending_path=True
        n.extension_failed('empty_path')
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        self.assertFalse(n.follow_hold);handle.cancel_goal_async.assert_not_called()

    def test_user_query_change_still_retires_motion_and_closes_control(self):
        n=self.moving();epoch=n.epoch;handle=n.follow_handle
        n.on_query(String(data='gas station'))
        self.assertGreater(n.epoch,epoch);self.assertFalse(n.following)
        self.assertIn(7,n.follow_quarantine);self.assertTrue(n.follow_hold)
        handle.cancel_goal_async.assert_called()

    def test_stale_input_or_unacknowledged_source_cannot_borrow_motion(self):
        n=self.moving();n.odom_received_at=time.monotonic()-2.
        self.assertFalse(n.prepare_intent_change())
        n.odom_received_at=time.monotonic();n.execution_transport_issue.return_value='source_owner_unconfirmed'
        self.assertFalse(n.prepare_intent_change())

    def test_new_mode_does_not_run_stopped_frontier_tournament(self):
        n=self.node;n.follow_enabled=True;n.odom.pose.pose.orientation.w=1.
        n.route=self.route((6.,1.),'frontier-A');n.route_endpoint=(6.,1.)
        n.candidates=[(6.,1.,0.,math.hypot(6.,1.))]
        self.assertFalse(n.begin_exploration_comparison())
        self.assertIsNone(n.frontier_comparison)
        self.assertAlmostEqual(n.candidates[0][2],2*math.atan2(1.,6.))

    def test_explicit_observation_heading_is_not_replaced(self):
        n=self.node;n.follow_enabled=True;n.odom.pose.pose.orientation.w=1.
        n.route=self.route((6.,1.),'frontier-A');n.route_endpoint=(6.,1.)
        n.route_metadata[stamp_ns(n.route)]['observation_heading']=0.
        n.candidates=[(6.,1.,0.,math.hypot(6.,1.))]
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][2],0.)

    def test_arrival_proposal_respects_nominal_planner_radius_not_only_steering_limit(self):
        n=self.node;n.follow_enabled=True;n.odom.pose.pose.orientation.w=1.
        n.route=self.route((6.,2.),'frontier-A');n.route_endpoint=(6.,2.)
        n.candidates=[(6.,2.,0.,math.hypot(6.,2.))]
        original=n.get_parameter
        n.get_parameter=lambda name:S(value=12.) if name=='planner_minimum_turning_radius' else original(name)
        n.begin_exploration_comparison()
        # The direct circle is radius 10m: physically possible at 8m, but
        # outside this planner's 12m nominal-curvature contract.
        self.assertEqual(n.candidates[0][2],0.)

    def test_forward_path_does_not_wait_for_alternate_terminal_yaw(self):
        n=self.node
        path=fixtures.drivable_route((0.,0.),(6.,0.))
        self.assertFalse(n.should_compare_endpoint_headings(path))
        n.memory_goal=(6.,0.)
        self.assertTrue(n.should_compare_endpoint_headings(path))

    def test_replacement_ack_and_late_old_result_do_not_brake_new_action(self):
        n=self.moving();epoch=n.epoch
        new=fixtures.drivable_route((0.,0.),(16.,0.))
        self.assertTrue(n.send_motion_segment(replacement=[new]))
        serial=n.follow_request['serial']
        old=Future();old.set_result(S(status=5))
        n.on_follow_result(old,epoch,(12.,0.),7)
        self.assertTrue(n.replacement_pending);self.assertTrue(n.following)
        accepted=Future();handle=Mock(accepted=True);handle.get_result_async.return_value=Future()
        accepted.set_result(handle)
        n.on_follow_accepted(accepted,epoch,(16.,0.),serial,[new],True)
        n.on_follow_result(old,epoch,(12.,0.),7)
        self.assertEqual(n.active_follow_serial,serial)
        self.assertTrue(n.following);self.assertFalse(n.follow_hold)
        self.assertEqual(n.epoch,epoch)

    def test_blocked_future_path_cannot_replace_current_action(self):
        n=self.moving();handle=n.follow_handle
        n.validate_path=Mock(return_value='footprint_collision')
        n.send_motion_segment(replacement=[fixtures.drivable_route((0.,0.),(16.,0.))])
        n.follow_client.send_goal_async.assert_not_called()
        self.assertIs(n.follow_handle,handle);self.assertTrue(n.following)

    def test_session_status_identifies_actual_execution_strategy(self):
        self.node.status('probe')
        self.assertEqual(self.node.last_status['execution_mode'],'continuous_exploration_v1')

    def test_failure_of_old_action_does_not_reject_new_frontier(self):
        n=self.moving();old_epoch=n.epoch
        n.latest_route=self.route((20.,0.),'frontier-B')
        n._desired_intent=n.intent_key(n.latest_route)
        n.route_rejection_pub=Mock();n.start_segment=Mock()
        n.follow_failed('tracking_deviation')
        n.route_rejection_pub.publish.assert_not_called()
        self.assertGreater(n.epoch,old_epoch)
        self.assertIn(7,n.follow_quarantine);self.assertTrue(n.follow_hold)

    def test_old_terminal_failure_keeps_physical_stop_barrier(self):
        n=self.moving();epoch=n.epoch
        n.latest_route=self.route((20.,0.),'frontier-B')
        n._desired_intent=n.intent_key(n.latest_route)
        n.route_rejection_pub=Mock();n.start_segment=Mock()
        result=Future();result.set_result(S(status=6))
        n.on_follow_result(result,epoch,(12.,0.),7)
        self.assertIn(7,n.follow_quarantine)
        self.assertTrue(n.follow_quarantine[7]['terminal'])
        self.assertTrue(n.follow_hold)
        n.route_rejection_pub.publish.assert_not_called()

    def test_retiring_old_motion_owner_does_not_cancel_new_intent_compute(self):
        n=self.moving()
        newer=self.route((20.,0.),'frontier-B')
        n.route=n.latest_route=newer
        n.pending=n.extending_path=True;n.active_plan_serial=9;n.plan_handle=Mock()
        handle=n.plan_handle
        n.on_exploration_retired(self.retirement())
        self.assertTrue(n.pending);self.assertEqual(n.active_plan_serial,9)
        handle.cancel_goal_async.assert_not_called()

    def test_duplicate_acceptance_cannot_rebind_motion_to_new_intent(self):
        n=self.moving();original=n._motion_intent
        n.route=self.route((20.,0.),'frontier-B')
        done=Future();done.set_result(n.follow_handle)
        n.on_follow_accepted(done,n.epoch,(12.,0.),7,n.motion_queue,False)
        self.assertEqual(n._motion_intent,original)
