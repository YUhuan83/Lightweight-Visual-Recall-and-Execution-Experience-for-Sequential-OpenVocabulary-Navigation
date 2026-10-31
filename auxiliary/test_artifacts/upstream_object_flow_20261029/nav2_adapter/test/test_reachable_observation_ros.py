"""Observation proposals against vehicle maps; real callbacks, synthetic maps."""
from copy import deepcopy
import json
import math
import unittest
from unittest.mock import Mock
from concurrent.futures import Future
from types import SimpleNamespace as S
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from wildos_nav2_adapter.graph_route import footprint_blockers, cell_cost
import test_visible_approach_ros as fixture
import test_confirmed_recovery_ros as confirmed_fixture
from test_graph_path_bridge_routing import route


class ReachableObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.VisibleApproachTests.setUpClass()

    @classmethod
    def tearDownClass(cls):
        fixture.VisibleApproachTests.tearDownClass()

    def setUp(self):
        self.fixture=fixture.VisibleApproachTests()
        self.fixture.setUp()
        self.node=self.fixture.perception
        self.bridge=self.fixture.bridge
        self.node.publisher=Mock()
        self.grid=deepcopy(self.node.occupancy)
        self.grid.header.frame_id='spot1/odom'
        self.grid.info.width=self.grid.info.height=160
        self.grid.info.resolution=.25
        self.grid.info.origin.position.x=self.grid.info.origin.position.y=-20.
        self.grid.data=[0]*25600
        # Old implementation ignored this map entirely.
        self.node.vehicle_grid=self.grid

    def tearDown(self):
        self.fixture.tearDown()

    def occupy(self,x,y):
        ix=int((x+20)/.25);iy=int((y+20)/.25)
        self.grid.data[iy*160+ix]=100

    def wall(self):
        for y in range(160):
            self.grid.data[y*160+96]=100  # x=4m, outside current vehicle.

    def test_free_center_with_colliding_body_selects_shorter_observation_step(self):
        self.occupy(11.,0.)
        self.assertEqual(cell_cost(self.grid,8.,0.),0)
        self.assertTrue(footprint_blockers(self.grid,(8.,0.),0.))
        data=json.loads(self.fixture.detect().data)
        self.assertNotEqual(tuple(data['point']),(8.,0.))
        self.assertFalse(footprint_blockers(self.grid,tuple(data['point']),0.))
        self.assertFalse(data['search_confirmed'])

    def test_disconnected_free_centers_do_not_become_observation_goals(self):
        self.wall()
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_not_called()

    def test_standoff_ring_skips_pose_that_cannot_hold_real_body(self):
        self.occupy(13.,0.)
        target=PoseStamped();target.pose.position.x=20.
        self.assertIsNone(self.node.publish_standoff(target))
        point=self.node.last_goal
        self.assertNotEqual(point,(10.,0.))
        heading=math.atan2(-point[1],20.-point[0])
        self.assertFalse(footprint_blockers(self.grid,point,heading))
        self.assertGreaterEqual(math.dist(point,(20.,0.)),10.-1e-6)

    def test_standoff_ray_fallback_cannot_bypass_vehicle_reachability(self):
        self.wall()
        target=PoseStamped();target.pose.position.x=20.
        self.assertEqual(self.node.publish_standoff(target),'no_reachable_standoff')
        self.node.publisher.publish.assert_not_called()
        self.assertIsNone(self.node.last_goal)

    def test_exhausted_visible_vehicle_plans_release_without_waiting_for_timeout(self):
        self.bridge.on_visible_approach(self.fixture.detect())
        self.bridge.carrier_model='ackermann'
        self.bridge.route=self.bridge.latest_route=route((0.,0.),(8.,0.))
        self.bridge.route_endpoint=(8.,0.)
        self.bridge.candidates=[];self.bridge.candidate_index=0
        self.bridge.failure_reasons=['empty_path']
        self.bridge.try_planning_escape=Mock(return_value=False)
        self.bridge.try_candidate()
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertEqual(self.bridge.last_status['status'],'visible_target_no_vehicle_path')
        self.assertEqual(self.bridge.last_status['goal'],[8.,0.])
        self.assertFalse(self.bridge.object_found)

    def test_transport_failure_does_not_reject_observation_pose(self):
        self.bridge.on_visible_approach(self.fixture.detect())
        self.bridge.route=self.bridge.latest_route=route((0.,0.),(8.,0.))
        self.bridge.route_endpoint=(8.,0.)
        self.bridge.candidates=[];self.bridge.candidate_index=0
        self.bridge.failure_reasons=['planner_timeout']
        self.bridge.try_planning_escape=Mock(return_value=False)
        self.bridge.try_candidate()
        self.assertTrue(self.bridge.visible_approach_active)

    def failure(self,goal,state='visible_target_no_vehicle_path',**changes):
        data=dict(query=self.node.active_query,stamp_ns=self.fixture.now,
                  status=state,goal=list(goal))
        data.update(changes)
        return String(data=json.dumps(data))

    def accept_visible(self,goal,**changes):
        self.node.on_navigation_plan_status(self.failure(
            goal,state='visible_target_approach',**changes))

    def test_navigation_failure_selects_another_visible_point_with_same_evidence(self):
        data=json.loads(self.fixture.detect().data)
        point=tuple(data['point']);self.node.visible_approach_pub.publish.reset_mock()
        self.accept_visible(point)
        self.node.on_navigation_plan_status(self.failure(point))
        self.node.visible_approach_pub.publish.assert_called_once()
        alternative=json.loads(self.node.visible_approach_pub.publish.call_args.args[0].data)
        self.assertGreaterEqual(math.dist(alternative['point'],point),2.)
        self.assertEqual(alternative['observation_stamp_ns'],data['observation_stamp_ns'])
        self.assertEqual(alternative['query'],data['query'])
        self.assertFalse(alternative['search_confirmed'])
        # A duplicate delivery cannot reject another point or trigger a new attempt.
        self.node.on_navigation_plan_status(self.failure(point))
        self.node.visible_approach_pub.publish.assert_called_once()

    def test_accepted_visible_failure_survives_newer_unaccepted_proposal(self):
        point=tuple(json.loads(self.fixture.detect().data)['point'])
        self.accept_visible(point)
        self.node.last_visible_goal=(point[0]+2.,point[1])
        self.node.on_navigation_plan_status(self.failure(point))
        self.assertEqual(len(self.node.rejected_observation_poses),1)
        self.assertEqual(self.node.rejected_observation_poses[0][0],point)

    def test_continuous_observation_advance_updates_failure_owner(self):
        bridge=self.fixture.prepare_moving_visible_goal()
        self.assertTrue(bridge.advance_visible_goal())
        accepted=bridge.status_pub.publish.call_args.args[0]
        self.node.on_navigation_plan_status(accepted)
        self.assertEqual(self.node.accepted_visible_goal,(12.,0.))
        # The prior step is no longer the accepted failure owner.
        self.node.on_navigation_plan_status(self.failure((8.,0.)))
        self.assertEqual(len(self.node.rejected_observation_poses),0)
        self.node.on_navigation_plan_status(self.failure((12.,0.)))
        self.assertEqual(self.node.rejected_observation_poses[0][0],(12.,0.))

    def test_unaccepted_visible_failure_cannot_reject_latest_proposal(self):
        point=json.loads(self.fixture.detect().data)['point']
        self.node.on_navigation_plan_status(self.failure(point))
        self.assertEqual(len(self.node.rejected_observation_poses),0)

    def test_new_proposal_failure_cannot_replace_accepted_goal(self):
        point=tuple(json.loads(self.fixture.detect().data)['point'])
        self.accept_visible(point)
        proposed=(point[0]+2.,point[1])
        self.node.last_visible_goal=proposed
        self.node.on_navigation_plan_status(self.failure(proposed))
        self.assertEqual(len(self.node.rejected_observation_poses),0)
        self.assertEqual(self.node.accepted_visible_goal,point)

    def test_stale_or_foreign_acceptance_does_not_authorize_failure(self):
        point=json.loads(self.fixture.detect().data)['point']
        self.accept_visible(point,query='other target')
        self.accept_visible(point,stamp_ns=self.fixture.now-3_000_000_000)
        self.node.on_navigation_plan_status(self.failure(point))
        self.assertEqual(len(self.node.rejected_observation_poses),0)

    def test_stale_or_other_query_failure_cannot_reject_current_goal(self):
        data=json.loads(self.fixture.detect().data);point=data['point']
        self.accept_visible(point)
        self.node.on_navigation_plan_status(self.failure(point,query='other target'))
        self.node.on_navigation_plan_status(self.failure(point,stamp_ns=self.fixture.now-3_000_000_000))
        self.node.on_navigation_plan_status(self.failure((18.,0.)))
        self.assertEqual(len(self.node.rejected_observation_poses),0)

    def test_query_switch_discards_navigation_rejections(self):
        point=json.loads(self.fixture.detect().data)['point']
        self.accept_visible(point)
        self.node.on_navigation_plan_status(self.failure(point))
        self.assertEqual(len(self.node.rejected_observation_poses),1)
        self.fixture.now+=100_000_000
        self.accept_visible((point[0]+2.,point[1]))
        self.assertIsNotNone(self.node.accepted_visible_goal)
        self.fixture.query('bus stop')
        self.assertEqual(len(self.node.rejected_observation_poses),0)
        self.assertIsNone(self.node.last_visible_goal)
        self.assertIsNone(self.node.accepted_visible_goal)
        self.assertEqual(self.node.accepted_visible_stamp_ns,0)

    def test_confirmed_rejection_refreshes_standoff_without_fresh_confirmation(self):
        target=PoseStamped();target.pose.position.x=20.
        self.node.publish_standoff(target);previous=self.node.last_goal
        base_stamp=self.fixture.now-12_000_000_000
        confirmation=dict(query=self.node.active_query,observation_stamp_ns=base_stamp,
            object_position=[20.,0.,2.],instance_id='same-object',radius=.5,
            localization_valid=True,inliers=4,observations=5)
        self.node.active_approach=(target,confirmation)
        self.node.on_navigation_plan_status(self.failure(previous,
            state='confirmed_approach_failed',reason='no_vehicle_path'))
        self.assertGreater(math.dist(previous,self.node.last_goal),1.2)
        refresh=json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(refresh['object_position'],confirmation['object_position'])
        self.assertEqual(refresh['instance_id'],'same-object')
        self.assertEqual(refresh['observation_stamp_ns'],base_stamp)
        self.assertEqual(refresh['base_observation_stamp_ns'],base_stamp)
        self.assertTrue(refresh['approach_refresh'])
        self.assertEqual(refresh['standoff'],list(self.node.last_goal))

    def test_stale_vehicle_map_does_not_authorize_new_observation(self):
        self.grid.header.stamp.sec-=4
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_not_called()

    def test_rotated_vehicle_map_keeps_world_pose_and_body_checks_aligned(self):
        self.grid.info.origin.position.x=20.
        self.grid.info.origin.orientation.z=math.sin(math.pi/4)
        self.grid.info.origin.orientation.w=math.cos(math.pi/4)
        # In a 90-degree map, world (11,0) has local (20,9).
        self.grid.data[36*160+80]=100
        data=json.loads(self.fixture.detect().data)
        self.assertFalse(footprint_blockers(self.grid,tuple(data['point']),0.))
        self.assertLess(data['point'][0],8.)

    def test_actual_plan_failures_select_alternative_but_do_not_bypass_route_admission(self):
        n=self.bridge
        n.carrier_model='ackermann';n.follow_enabled=True
        n.execution_transport_issue=Mock(return_value=None)
        n.local_freshness_issue=Mock(return_value=None)
        n.grid=self.grid;n.local_grid=deepcopy(self.grid)
        import time
        n.grid_arrived_at=time.monotonic()
        n.plan_client=Mock();n.plan_client.send_goal_async.side_effect=lambda goal: Future()
        n.follow_client=Mock();n.require_route_status=False
        data=json.loads(self.fixture.detect().data)
        n.on_visible_approach(String(data=json.dumps(data)))
        self.node.on_navigation_plan_status(n.status_pub.publish.call_args.args[0])
        graph=route((0.,0.),tuple(data['point']));graph.header.stamp=self.fixture.stamp()
        n.on_route(graph)
        first=n.plan_client.send_goal_async.call_args.args[0].goal.pose.position.x
        old_epoch=n.epoch;old_index=n.candidate_index;old_serial=n.active_plan_serial
        for _ in range(30):
            if not n.visible_approach_active:
                break
            result=Future();result.set_result(S(status=6,result=S(path=type(graph)())))
            n.on_plan_result(result,n.epoch,n.candidate_index,n.active_plan_serial)
        self.assertFalse(n.visible_approach_active)
        self.node.on_navigation_plan_status(n.status_pub.publish.call_args.args[0])
        alternative=self.node.visible_approach_pub.publish.call_args.args[0]
        n.on_visible_approach(alternative)
        self.assertFalse(n.visible_approach_active)  # Existing reset/stop barrier.
        n.query_reset_pending=False
        n.on_visible_approach(alternative)
        self.assertTrue(n.visible_approach_active)
        self.assertNotEqual(n.expected,tuple(data['point']))
        graph=route((0.,0.),n.expected);graph.header.stamp=self.fixture.stamp()
        n.on_route(graph)
        self.assertNotEqual(n.plan_client.send_goal_async.call_args.args[0].goal.pose.position.x,first)
        # A late success for the failed pose must not authorize movement.
        result=Future();result.set_result(S(status=4,result=S(path=graph)))
        n.on_plan_result(result,old_epoch,old_index,old_serial)
        n.follow_client.send_goal_async.assert_not_called()
        self.assertFalse(n.object_found)

    def test_confirmed_recovery_does_not_retry_views_across_disconnected_map(self):
        test=confirmed_fixture.ConfirmedRecoveryTests();test.setUp()
        try:
            n=test.node
            # Every existing same-instance observation candidate is beyond
            # a closed wall. Their centre cells and final footprints are free.
            column=88  # World x=8, map origin=-80 and resolution=1.
            for row in range(160):n.grid.data[row*160+column]=100
            n.relay_destination=Mock()
            self.assertFalse(n.start_live_observation())
            n.relay_destination.assert_not_called()
            self.assertEqual(n.active_object_confirmation['instance_id'],'target-a')
        finally:
            test.tearDown()

    def test_confirmed_failure_feedback_enters_existing_same_instance_handoff(self):
        test=confirmed_fixture.ConfirmedRecoveryTests();test.setUp()
        try:
            n=test.node
            self.fixture.query(n.query)
            target=PoseStamped();target.pose.position.x=20.
            confirmation=json.loads(test.fixture.confirmation(
                n.query,[20.,0.,2.],[0.,0.,0.],[16.,0.]).data)
            confirmation['instance_id']='target-a'
            self.node.confirmed_instance_anchor=deepcopy(confirmation)
            self.node.active_approach=(target,confirmation)
            self.node.last_goal=(16.,0.)
            n.status_pub=Mock()
            test.exhaust_plans()
            self.node.on_navigation_plan_status(n.status_pub.publish.call_args.args[0])
            refresh=self.node.confirmation_pub.publish.call_args.args[0]
            n.on_object_confirmation(refresh)
            self.assertIsNotNone(n.deferred_confirmed_approach)
            alternative=tuple(n.deferred_confirmed_approach['standoff'])
            self.assertNotEqual(alternative,(16.,0.))
            n.monitor_observation()
            self.assertEqual(n.expected,alternative)
            self.assertTrue(n.observation_active)
            self.assertEqual(n.active_object_confirmation['instance_id'],'target-a')
            self.assertFalse(n.object_found)
            self.assertIsNone(n.instance_release_pending)
            n.follow_client.send_goal_async.assert_not_called()
        finally:
            test.tearDown()

    def test_unknown_vehicle_cells_are_not_free_observation_endpoints(self):
        self.grid.data=[-1]*len(self.grid.data)
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_not_called()


if __name__=='__main__':unittest.main()
