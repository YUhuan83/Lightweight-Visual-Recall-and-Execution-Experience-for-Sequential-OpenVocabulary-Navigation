"""Same-target route updates use the existing continuous action handoff."""
import time
import unittest
from copy import deepcopy
from unittest.mock import Mock
import test_continuous_exploration_ros as fixture
import test_graph_path_bridge_routing as routes
from wildos_nav2_adapter.input_quality import stamp_ns


class ConfirmedContinuationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): fixture.ContinuousExplorationTests.setUpClass()
    @classmethod
    def tearDownClass(cls): fixture.ContinuousExplorationTests.tearDownClass()
    def setUp(self):
        self.fixture=fixture.ContinuousExplorationTests(); self.fixture.setUp()
        n=self.node=self.fixture.moving(); n.full_path_execution=True
        self.addCleanup(n.destroy_node)
        n.expected=n.confirmed_route_goal=(4.,0.)
        n.confirmed_route_object=(30.,0.); n.confirmed_route_radius=.5
        n.confirmed_route_started_at=time.monotonic()
        n.active_object_confirmation=dict(query=n.query,instance_id='target-A')
        n.route=n.latest_route=routes.route((0.,0.),(4.,0.))
        p=routes.drivable_route((0.,0.),(4.,0.))
        n.motion_queue=[p]; n.follow_active['path_stamp_ns']=stamp_ns(p)
        n.on_controller_progress(deepcopy(p))
        n.relay_destination=Mock()
        self.data=dict(query=n.query,instance_id='target-A',standoff=[8.,0.],
            object_position=[30.,0.,2.],radius=.5,
            observation_stamp_ns=n.get_clock().now().nanoseconds)
    def send(self): self.node.route_confirmed_target(self.data,(0.,0.))
    def test_forward_update_retains_action_epoch_and_requests_new_graph_route(self):
        n=self.node; handle=n.follow_handle; epoch=n.epoch
        self.assertIsNotNone(n.active_window())
        self.send()
        self.assertEqual(n.expected,(8.,0.))
        n.relay_destination.assert_called_once()
        self.assertIs(n.follow_handle,handle); self.assertEqual(n.epoch,epoch)
        self.assertTrue(n.following); handle.cancel_goal_async.assert_not_called()
        self.assertFalse(n.object_found)
        self.assertEqual(n._retirement_path_stamp,stamp_ns(n._controller_path))
    def test_fresh_deferred_update_is_consumed_while_forward_action_runs(self):
        n=self.node; n.deferred_confirmed_approach=deepcopy(self.data)
        self.assertTrue(n.consume_confirmed_approach())
        self.assertEqual(n.expected,(8.,0.)); self.assertTrue(n.following)
    def test_returned_graph_route_plans_in_background_without_canceling_motion(self):
        n=self.node; handle=n.follow_handle; self.send()
        n.start_segment=Mock()
        n.on_route(routes.route((0.,0.),(8.,0.)))
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertIs(n.follow_handle,handle); self.assertTrue(n.following)
        handle.cancel_goal_async.assert_not_called()
    def test_pending_planner_does_not_get_restarted_by_new_confirmations(self):
        n=self.node; n.pending=True; self.send()
        self.assertEqual(n.expected,(4.,0.)); n.relay_destination.assert_not_called()
    def test_graph_handoff_in_flight_does_not_repeat_destination(self):
        self.send(); self.data['standoff']=[10.,0.]; self.send()
        self.node.relay_destination.assert_called_once()
    def test_replacement_ack_wait_keeps_current_goal(self):
        n=self.node; n.replacement_pending=True; self.send()
        self.assertEqual(n.expected,(4.,0.))
    def test_reverse_branch_waits_for_existing_motion_barrier(self):
        n=self.node; p=routes.drivable_route((0.,0.),(-4.,0.))
        n.follow_active['path_stamp_ns']=stamp_ns(p); n.motion_queue=[p]
        n.on_controller_progress(deepcopy(p)); self.send()
        self.assertEqual(n.expected,(4.,0.)); n.relay_destination.assert_not_called()
    def test_stale_controller_progress_cannot_enable_handoff(self):
        self.node._controller_path_received_at=time.monotonic()-1.; self.send()
        self.assertEqual(self.node.expected,(4.,0.))
    def test_stale_deferred_confirmation_does_not_enable_handoff(self):
        n=self.node; self.data['observation_stamp_ns']-=6_000_000_000
        n.deferred_confirmed_approach=self.data
        self.assertFalse(n.consume_confirmed_approach())
        self.assertEqual(n.expected,(4.,0.))
    def test_different_instance_cannot_redirect_live_route(self):
        self.data['instance_id']='target-B'; self.send()
        self.assertEqual(self.node.expected,(4.,0.))
    def test_point_without_material_progress_does_not_replan(self):
        self.data['standoff']=[4.7,0.]; self.send()
        self.assertEqual(self.node.expected,(4.,0.))
    def test_new_route_does_not_move_fixed_object_association_anchor(self):
        self.data['object_position']=[30.5,0.,2.]; self.send()
        self.assertEqual(self.node.confirmed_route_object,(30.,0.))
    def test_failure_recovery_keeps_existing_handoff_ownership(self):
        self.node.confirmed_approach_failure=dict(goal=(4.,0.),reason='no_vehicle_path')
        self.send(); self.assertEqual(self.node.expected,(4.,0.))
        self.node.relay_destination.assert_not_called()
