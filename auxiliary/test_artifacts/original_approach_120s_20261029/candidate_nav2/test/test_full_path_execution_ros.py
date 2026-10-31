import copy
import math
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import rclpy
import yaml
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
from wildos_nav2_adapter.input_quality import stamp_ns
import test_graph_path_bridge_routing as fixtures
import test_continuous_exploration_ros as continuous_fixtures

class FullPathExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        with patch.object(fixtures,'GraphPathBridge',ContinuousGraphPathBridge):
            fixtures.GraphBridgeRoutingTests.setUp(self)
        self.addCleanup(self.node.destroy_node)
        self.node.full_path_execution=True
        self.node.follow_enabled=True
        self.node.follow_client=Mock()
        self.node.follow_client.send_goal_async.return_value=Future()
        self.node.execution_transport_issue=Mock(return_value=None)
    def mixed(self):
        p=fixtures.drivable_route((0.,0.),(2.,0.),(1.,0.),(4.,0.))
        return p,[(v.pose.position.x,v.pose.position.y) for v in p.poses],[0.]*len(p.poses)
    def test_whole_path_is_one_controller_action(self):
        p,points,headings=self.mixed();n=self.node
        n.send_motion_segment=Mock()
        n.dispatch_validated_path(p,points[-1],points,headings)
        self.assertEqual(len(n.motion_queue),1)
        self.assertEqual(len(n.motion_queue[0].poses),len(p.poses))
        n.send_motion_segment.assert_called_once_with()
    def test_legacy_mode_still_preserves_old_version(self):
        p,points,_=self.mixed();self.node.full_path_execution=False
        self.assertEqual(len(self.node.execution_paths(p,points)),3)
    def test_only_first_direction_is_used_for_actual_pose_join(self):
        p,points,headings=self.mixed()
        a,b,t=self.node.motion_join_geometry(points,headings,1.)
        self.assertEqual(a[-1],(2.,0.));self.assertEqual(len(a),len(b));self.assertEqual(t,.2)
    def test_near_final_position_cannot_skip_required_motion(self):
        self.assertFalse(self.node.endpoint_may_complete_immediately([(0.,0.),(2.,0.),(.1,0.)]))
        self.assertTrue(self.node.endpoint_may_complete_immediately([(0.,0.),(.1,0.)]))
    def test_tracking_checks_the_first_cusp_with_cusp_tolerance(self):
        _,points,headings=self.mixed()
        with patch('wildos_nav2_adapter.graph_path_bridge.preview_tracking',return_value=S(reason=None)) as preview:
            self.assertIsNone(self.node.tracking_issue(points,headings,1))
        self.assertEqual(preview.call_args.args[0][-1],(2.,0.))
        self.assertEqual(preview.call_args.kwargs['position_tolerance'],.2)
    def test_live_forward_replacement_is_one_unsplit_path(self):
        p,points,headings=self.mixed();n=self.node
        n.following=True;n.extending_path=True;n.send_motion_segment=Mock()
        n.dispatch_validated_path(p,points[-1],points,headings)
        replacement=n.send_motion_segment.call_args.kwargs['replacement']
        self.assertEqual(len(replacement),1)
        self.assertEqual(len(replacement[0].poses),len(p.poses))
    def test_background_rejection_keeps_accepted_motion(self):
        n=self.node;n.following=True;n.pending=n.extending_path=True;n.follow_handle=Mock()
        handle=n.follow_handle;n.extension_failed('empty_path')
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle)
        handle.cancel_goal_async.assert_not_called()
    def test_mode_cannot_change_while_running(self):
        result=self.node.on_parameters([S(name='continuous_path_execution',value=False)])
        self.assertFalse(result.successful)
    def test_configuration_checks_both_ends_and_progress_completion(self):
        root=Path(__file__).resolve().parents[2]
        config=yaml.safe_load((root/'nav2_adapter/config/nav2_ackermann_control.yaml').read_text(encoding='utf8'))
        contract=navigation_runtime_contract(config,'ackermann')
        self.assertTrue(contract['/controller_server']['FollowPath.continuous_path_execution'])
        self.assertTrue(contract['/wildos_graph_path_bridge']['continuous_path_execution'])
        for checker in ('goal_checker','maneuver_goal_checker','cusp_goal_checker'):
            bad=copy.deepcopy(config)
            bad['controller_server']['ros__parameters'][checker]['plugin']='nav2_controller::SimpleGoalChecker'
            with self.assertRaises(ValueError):navigation_runtime_contract(bad,'ackermann')
    def test_initial_collision_is_still_rejected(self):
        p,points,headings=self.mixed();n=self.node
        n.motion_queue=n.execution_paths(p,points)
        n.pose_blockers=Mock(return_value=['blocked'])
        n.validate_path=Mock(return_value=None);n.validate_local_path=Mock(return_value=None)
        n.tracking_issue=Mock(return_value=None);n.follow_failed=Mock()
        n.send_motion_segment()
        n.follow_client.send_goal_async.assert_not_called()
        n.follow_failed.assert_called_once()
    def progress_sample(self):
        n=self.node;p,points,_=self.mixed()
        n.motion_queue=[p];n.following=True;n.active_follow_serial=17
        n.follow_active=dict(serial=17,path_stamp_ns=stamp_ns(p))
        sample=copy.deepcopy(p)
        first=next(i for i,v in enumerate(sample.poses) if v.pose.position.x==2.)
        last=next(i for i in range(first+1,len(sample.poses)) if sample.poses[i].pose.position.x==1.)
        sample.poses=sample.poses[first:last+1]
        for pose in sample.poses:pose.header.stamp=n.get_clock().now().to_msg()
        return sample
    def test_actual_controller_branch_overrides_nearest_old_branch(self):
        p=self.progress_sample();self.node.on_controller_progress(p)
        current=self.node.current_motion_path()
        self.assertIsNotNone(current)
        self.assertEqual(current.poses[0].pose.position.x,2.)
        self.assertEqual(current.poses[-1].pose.position.x,1.)
    def test_old_action_progress_cannot_attach_to_replacement(self):
        p=self.progress_sample();p.header.stamp.nanosec+=1
        self.node.on_controller_progress(p)
        self.assertIsNone(self.node.current_motion_path())
    def test_stale_progress_is_not_used_for_moving_handoff(self):
        p=self.progress_sample()
        for v in p.poses:v.header.stamp.sec-=2
        self.node.on_controller_progress(p)
        self.assertIsNone(self.node.current_motion_path())
        p=self.progress_sample();self.node.on_controller_progress(p)
        self.node._controller_path_received_at=time.monotonic()-1.
        self.assertIsNone(self.node.current_motion_path())
    def test_cancelled_action_progress_cannot_reactivate_motion(self):
        p=self.progress_sample();self.node.following=False
        self.node.on_controller_progress(p)
        self.assertIsNone(self.node.current_motion_path())
    def test_frontier_retirement_keeps_full_path_with_a_bounded_handoff(self):
        helper=continuous_fixtures.ContinuousExplorationTests();helper.node=self.node
        n=helper.moving();n.full_path_execution=True
        p,_,_=self.mixed();n.motion_queue=[p]
        n.follow_active['path_stamp_ns']=stamp_ns(p)
        progress=copy.deepcopy(p);progress.poses=progress.poses[:9]
        for v in progress.poses:v.header.stamp=n.get_clock().now().to_msg()
        n.on_controller_progress(progress)
        handle=n.follow_handle;epoch=n.epoch
        n.on_exploration_retired(helper.retirement())
        self.assertTrue(n.following);self.assertEqual(n.epoch,epoch)
        self.assertIsNotNone(n.retired_motion);self.assertTrue(n.retired_motion['bounded'])
        self.assertEqual(n.retired_motion['serial'],7)
        handle.cancel_goal_async.assert_not_called()

if __name__=='__main__':unittest.main()
