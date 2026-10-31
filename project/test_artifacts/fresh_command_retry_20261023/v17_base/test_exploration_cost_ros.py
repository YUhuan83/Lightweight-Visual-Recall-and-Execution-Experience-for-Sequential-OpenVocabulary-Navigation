"""Production comparison/commit callbacks; explicit synthetic planner results."""
import json
import math
import time
import unittest
from concurrent.futures import Future
from copy import deepcopy
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from std_msgs.msg import String
from wildos_nav2_adapter.input_quality import stamp_ns
import test_graph_path_bridge_routing as fixtures
from test_graph_path_bridge_routing import route,drivable_route


class ExplorationCostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.fixture=fixtures.GraphBridgeRoutingTests();self.fixture.setUp();self.node=self.fixture.node
        n=self.node;n.follow_enabled=True;n.route=n.latest_route=route((0.,0.),(12.,0.))
        n.odom.pose.pose.orientation.w=1.
        n.route_endpoint=(12.,0.);n.require_route_status=True
        n.plan_client=Mock();n.plan_client.send_goal_async.return_value=Future()
        n.follow_client=Mock();n.execute_checked_path=Mock();n.full_path_pub=Mock()
        n.frontier_choice_pub=Mock()
        n.route_metadata[stamp_ns(n.route)]=dict(exploration_id='a:1',complete=True,
            goal=[12.,0.],observation_heading=0.,vehicle_cost_selected=False,vehicle_options=[
                dict(uuid='a',goal=[12.,0.,0.],benefit=.28),
                dict(uuid='b',goal=[14.,0.,0.],benefit=.27)])
    def tearDown(self):self.node.destroy_node()
    def response(self,path):
        f=Future();f.set_result(S(status=4,result=S(path=path)))
        self.node.on_plan_result(f,self.node.epoch,self.node.candidate_index,self.node.active_plan_serial)
    def compare(self):
        self.node.start_segment()
        self.response(drivable_route((0.,0.),(8.,0.),(3.,0.),(12.,0.)))
        self.response(drivable_route((0.,0.),(14.,0.)))
        return self.node.frontier_choice_pub.publish.call_args.args[0]
    def test_first_feasible_observation_path_cannot_skip_other_frontier_costs(self):
        self.node.start_segment()
        self.response(drivable_route((0.,0.),(8.,0.),(3.,0.),(12.,0.)))
        self.node.execute_checked_path.assert_not_called()
        self.node.full_path_pub.publish.assert_not_called()
        request=self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertTrue(request.use_start)
        self.assertEqual(request.start.pose.position.x,0.)
        self.assertEqual(request.goal.pose.position.x,14.)
    def test_actual_path_and_gear_cost_select_alternative_without_driving(self):
        choice=self.compare()
        self.assertEqual(choice.exploration_id,'a:1')
        self.assertEqual(choice.chosen_uuid,'b')
        self.assertAlmostEqual(choice.vehicle_cost,14.)
        self.assertEqual(choice.header.stamp.sec,self.node.route.header.stamp.sec)
        self.node.execute_checked_path.assert_not_called()
        self.node.follow_client.send_goal_async.assert_not_called()
        self.assertIsNotNone(self.node.frontier_choice)
    def test_semantic_benefit_can_outweigh_a_shorter_path(self):
        self.node.route_metadata[stamp_ns(self.node.route)]['vehicle_options'][0]['benefit']=.9
        self.assertEqual(self.compare().chosen_uuid,'a')

    def test_unexecutable_section_count_cannot_win_frontier_comparison(self):
        # A high-benefit route used to win here, only to be rejected by the
        # existing section cap at execution, discarding the usable alternative.
        self.node.route_metadata[stamp_ns(self.node.route)]['vehicle_options'][0]['benefit']=.9
        self.node.start_segment()
        self.response(drivable_route(*[(x,0.) for x in
            (0.,2.,1.,2.,1.,2.,1.,2.,1.,12.)]))
        self.response(drivable_route((0.,0.),(14.,0.)))
        choice=self.node.frontier_choice_pub.publish.call_args.args[0]
        self.assertEqual(choice.chosen_uuid,'b')
        self.assertEqual(choice.vehicle_cost,14.)
        self.node.execute_checked_path.assert_not_called()

    def test_no_section_admissible_frontier_restores_normal_candidates(self):
        self.node.reject_exploration_route=Mock()
        self.node.start_segment()
        for end in (12.,14.):
            self.response(drivable_route(*[(x,0.) for x in
                (0.,2.,1.,2.,1.,2.,1.,2.,1.,end)]))
        self.node.frontier_choice_pub.publish.assert_not_called()
        self.assertIsNone(self.node.frontier_comparison)
        self.node.reject_exploration_route.assert_not_called()
        self.assertFalse(self.node.plan_client.send_goal_async.call_args.args[0].use_start)
    def test_query_memory_motion_and_uncertain_actions_keep_original_pipeline(self):
        for name,value in [('following',True),('memory_selecting',True),('memory_goal',(12.,0.)),
                           ('expected',(12.,0.)),('follow_enabled',False),('extending_path',True),
                           ('follow_quarantine',{8:dict(terminal=False)})]:
            old=getattr(self.node,name);setattr(self.node,name,value)
            self.assertFalse(self.node.begin_exploration_comparison(),name)
            setattr(self.node,name,old)
    def test_invalid_or_duplicate_offers_do_not_enter_comparison(self):
        metadata=self.node.route_metadata[stamp_ns(self.node.route)]
        options=deepcopy(metadata['vehicle_options'])
        for key,value in [('goal',[14.,0.,float('nan')]),('benefit',float('inf')),('uuid','a')]:
            metadata['vehicle_options']=deepcopy(options);metadata['vehicle_options'][1][key]=value
            self.assertFalse(self.node.begin_exploration_comparison())
    def test_all_failed_comparisons_fall_back_without_retiring_or_learning_failure(self):
        self.node.reject_exploration_route=Mock()
        self.node.start_segment()
        self.node.candidate_failed('empty_path');self.node.candidate_failed('empty_path')
        self.assertIsNone(self.node.frontier_comparison)
        self.assertIsNone(self.node.frontier_choice)
        self.assertIn('a:1',self.node.frontier_compared)
        self.node.reject_exploration_route.assert_not_called()
        self.assertEqual(self.node.failure_reasons,[])
        self.assertFalse(self.node.plan_client.send_goal_async.call_args.args[0].use_start)
    def test_moved_origin_does_not_commit_a_cost_from_another_pose(self):
        self.node.start_segment();self.node.odom.pose.pose.position.x=.4
        self.response(drivable_route((0.,0.),(12.,0.)))
        self.assertIsNone(self.node.frontier_comparison)
        self.node.frontier_choice_pub.publish.assert_not_called()
    def test_map_change_revalidates_earlier_candidates_on_common_latest_map(self):
        self.node.start_segment()
        self.response(drivable_route((0.,0.),(8.,0.),(3.,0.),(12.,0.)))
        self.node.grid.data[10*40+16]=100
        self.response(drivable_route((0.,0.),(14.,0.)))
        self.node.frontier_choice_pub.publish.assert_not_called()
        self.assertIsNone(self.node.frontier_comparison)
    def test_selected_graph_route_is_required_before_execution(self):
        choice=self.compare()
        metadata=deepcopy(self.node.route_metadata[stamp_ns(self.node.route)])
        latest=deepcopy(self.node.route);latest.header.stamp=self.node.get_clock().now().to_msg()
        metadata.update(exploration_id='b:2',vehicle_cost_selected=True,goal=[14.,0.])
        self.assertTrue(self.node.wait_for_frontier_choice(latest,dict(metadata,exploration_id='c:3')))
        self.assertFalse(self.node.wait_for_frontier_choice(latest,metadata))
        self.assertIsNone(self.node.frontier_choice)
        self.node.execute_checked_path.assert_not_called()
    def test_rejected_choice_restores_normal_planning_and_old_ack_is_ignored(self):
        choice=self.compare();self.node.start_segment=Mock()
        ack=dict(stamp_ns=self.node.get_clock().now().nanoseconds,source_stamp_ns=stamp_ns(choice),
                 exploration_id='other:1',accepted=False)
        self.node.on_frontier_choice_ack(String(data=json.dumps(ack)))
        self.assertIsNotNone(self.node.frontier_choice)
        ack['exploration_id']='a:1';self.node.on_frontier_choice_ack(String(data=json.dumps(ack)))
        self.assertIsNone(self.node.frontier_choice);self.node.start_segment.assert_called_once()
    def test_missing_commit_result_requires_real_graph_reset_barrier(self):
        choice=self.compare();self.node.frontier_choice['started']-=6.
        self.node.request_graph_reset=Mock();self.node.retry_graph_reset=Mock()
        self.assertTrue(self.node.monitor_frontier_choice())
        self.assertIsNone(self.node.frontier_choice)
        self.node.request_graph_reset.assert_called_once();self.node.retry_graph_reset.assert_called_once()
        self.node.execute_checked_path.assert_not_called()
    def test_query_reset_fences_late_comparison_result(self):
        self.node.start_segment();epoch=self.node.epoch;index=self.node.candidate_index
        serial=self.node.active_plan_serial
        self.node.reset();self.node.query_generation+=1
        f=Future();f.set_result(S(status=4,result=S(path=drivable_route((0.,0.),(12.,0.)))))
        self.node.on_plan_result(f,epoch,index,serial)
        self.node.frontier_choice_pub.publish.assert_not_called()
        self.node.execute_checked_path.assert_not_called()
    def test_transport_epoch_change_cannot_restore_candidates_from_previous_route(self):
        self.node.start_segment()
        self.node.execution_transport_unavailable('native_telemetry_stale')
        self.node.route=route((0.,0.),(18.,0.));self.node.route_endpoint=(18.,0.)
        self.node.route_metadata[stamp_ns(self.node.route)]=dict(
            exploration_id='c:3',complete=False,goal=[28.,0.])
        self.node.try_candidate=Mock()
        self.node.start_segment()
        self.assertIsNone(self.node.frontier_comparison)
        self.assertEqual(self.node.candidates[0][:2],(18.,0.))

    def test_footprint_colliding_frontier_is_not_planned_or_selected(self):
        # 094550: GraphNav's two highest-benefit endpoints were free at their
        # centre cell but the 4.47 m x 2.12 m body overlapped a lethal cell.
        # Each still cost a full Smac search before the lower-benefit option
        # was found. The comparison must skip those poses without dropping the
        # GraphNav route identity or the remaining feasible alternatives.
        metadata=self.node.route_metadata[stamp_ns(self.node.route)]
        metadata['goal']=[12.,0.,math.pi/2]
        metadata['vehicle_options']=[
            dict(uuid='a',goal=[12.,0.,math.pi/2],benefit=.28),
            dict(uuid='b',goal=[14.,0.,0.],benefit=.27),
            dict(uuid='c',goal=[16.,0.,0.],benefit=.26)]
        # A thick wall intersecting every vehicle pose within goal tolerance
        # of a, while leaving b/c and their y=0 approach sweep clear.
        for ix in range(19,25):
            for iy in range(12,15):
                self.node.grid.data[iy*40+ix]=100
        self.node.start_segment()
        first=self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertEqual(first.goal.pose.position.x,14.)
        self.response(drivable_route((0.,0.),(14.,0.)))
        second=self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertEqual(second.goal.pose.position.x,16.)
        self.response(drivable_route((0.,0.),(16.,0.)))
        choice=self.node.frontier_choice_pub.publish.call_args.args[0]
        self.assertEqual(choice.chosen_uuid,'b')
        self.assertEqual([first.goal.pose.position.x,second.goal.pose.position.x],[14.,16.])
        self.assertNotIn(12.,[first.goal.pose.position.x,second.goal.pose.position.x])
        self.assertEqual(self.node.frontier_comparison,None)

    def test_cached_failed_frontier_is_not_replanned(self):
        metadata=self.node.route_metadata[stamp_ns(self.node.route)]
        metadata['vehicle_options']=[
            dict(uuid='a',goal=[12.,0.,0.],benefit=.28),
            dict(uuid='b',goal=[14.,0.,0.],benefit=.27),
            dict(uuid='c',goal=[16.,0.,0.],benefit=.26)]
        self.node.sync_failed_goal_scope(metadata['exploration_id'])
        self.node.remember_failed_goal((0.,0.),(12.,0.),0.)
        self.node.start_segment()
        first=self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertEqual(first.goal.pose.position.x,14.)
        self.response(drivable_route((0.,0.),(14.,0.)))
        second=self.node.plan_client.send_goal_async.call_args.args[0]
        self.assertEqual(second.goal.pose.position.x,16.)
        self.response(drivable_route((0.,0.),(16.,0.)))
        choice=self.node.frontier_choice_pub.publish.call_args.args[0]
        self.assertEqual(choice.chosen_uuid,'b')
        self.assertNotIn(12.,[first.goal.pose.position.x,second.goal.pose.position.x])

    def test_footprint_check_leaves_fewer_than_two_options_on_normal_path(self):
        metadata=self.node.route_metadata[stamp_ns(self.node.route)]
        metadata['goal']=[12.,0.,math.pi/2]
        metadata['vehicle_options']=[
            dict(uuid='a',goal=[12.,0.,math.pi/2],benefit=.28),
            dict(uuid='b',goal=[14.,0.,math.pi/2],benefit=.27)]
        for ix in range(19,25):
            for iy in range(12,15):
                self.node.grid.data[iy*40+ix]=100
        before=list(self.node.candidates)
        self.assertFalse(self.node.begin_exploration_comparison())
        self.assertIsNone(self.node.frontier_comparison)
        self.assertEqual(self.node.candidates,before)

if __name__=='__main__':unittest.main()
