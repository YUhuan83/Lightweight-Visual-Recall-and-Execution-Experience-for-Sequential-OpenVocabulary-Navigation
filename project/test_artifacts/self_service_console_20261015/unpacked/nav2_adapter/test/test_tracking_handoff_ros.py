"""Actual-pose admission and bounded recovery; ROS doubles, no vehicle."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import test_execution_mechanism_ros as fixtures
from test_graph_path_bridge_routing import route, drivable_route


class TrackingHandoffTests(unittest.TestCase):
    def setUp(self):
        fixtures.ExecutionMechanismTests.setUp(self)
        self.node.expected=self.node.query=None
        self.node.query_started_ns=0
        self.case=json.loads((Path(__file__).parent/'fixtures/town_first_cusp_tracking_20261014.json').read_text())
        self.node.grid.info.origin.position.x=690.
        self.node.grid.info.origin.position.y=-40.
        self.node.local_grid=deepcopy(self.node.grid)
        self.node.start_segment=Mock()

    def tearDown(self): self.node.destroy_node()

    def recorded(self):
        case=self.case;actual=case['actual_start_pose']
        self.node.odom.pose.pose.position.x,self.node.odom.pose.pose.position.y=actual[:2]
        self.node.odom.pose.pose.orientation.z=math.sin(actual[2]/2)
        self.node.odom.pose.pose.orientation.w=math.cos(actual[2]/2)
        path=route(*case['points'])
        for p,h in zip(path.poses,case['headings']):
            p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2)
        return path

    def test_untrackable_candidate_advances_before_pricing_or_publishing(self):
        n=self.node;path=self.recorded();end=self.case['points'][-1]
        # Append a reverse branch so this endpoint uses the actual cusp disk.
        h=self.case['headings'][-1]
        p=deepcopy(path.poses[-1]);p.pose.position.x-=2*math.cos(h);p.pose.position.y-=2*math.sin(h)
        path.poses.append(p)
        n.route=path;n.route_endpoint=(p.pose.position.x,p.pose.position.y)
        n.candidates=[(*n.route_endpoint,h),(735.,-3.,h)]
        n.candidate_index=0;n.pending=True
        n.validate_path=Mock(return_value=None)
        n.record_memory_path=Mock();n.memory_selecting=True
        n.try_candidate=Mock()
        n.full_path_pub=Mock()
        n.on_plan_result(Mock(result=lambda:S(status=4,result=S(path=path))),n.epoch,0)
        self.assertEqual(n.candidate_index,1)
        n.try_candidate.assert_called_once()
        n.record_memory_path.assert_not_called()
        n.full_path_pub.publish.assert_not_called()

    def test_selected_path_is_rechecked_after_actual_heading_changes(self):
        n=self.node;n.grid.info.origin.position.x=n.grid.info.origin.position.y=-80.
        n.local_grid=deepcopy(n.grid);n.route=route((0.,0.),(6.,0.));n.route_endpoint=(6.,0.)
        old=Mock();n.follow_handle=old;n.following=True;n.extending_path=True
        n.odom.pose.pose.orientation.z=math.sin(math.pi/4)
        n.odom.pose.pose.orientation.w=math.cos(math.pi/4)
        n.execute_checked_path(drivable_route((0.,0.),(6.,0.)),(6.,0.))
        n.follow_client.send_goal_async.assert_not_called()
        old.cancel_goal_async.assert_not_called()
        self.assertTrue(n.following)
        self.assertIn('tracking_preview',n.last_status['extension_rejected'])

    def test_same_invalid_cusp_has_three_retries_and_preserves_explicit_goal(self):
        n=self.node;path=self.recorded()
        n.route=path;n.route_endpoint=n.expected=tuple(self.case['points'][-1])
        n.block_current_route=Mock();n.route_rejection_pub=Mock()
        for i in range(4):
            n.motion_queue=[deepcopy(path),deepcopy(path)]
            n.send_motion_segment(after_maneuver=True)
            self.assertEqual(n.recovery_attempts,min(i+1,3))
        self.assertEqual(n.start_segment.call_count,3)
        self.assertEqual(n.last_status['status'],'handoff_recovery_exhausted')
        n.follow_client.send_goal_async.assert_not_called()
        n.route_rejection_pub.publish.assert_not_called()
        self.assertEqual(n.expected,tuple(self.case['points'][-1]))

    def test_cusp_without_route_progress_does_not_reset_recovery_budget(self):
        n=self.node;n.grid.info.origin.position.x=n.grid.info.origin.position.y=-80.
        n.local_grid=deepcopy(n.grid);n.route_endpoint=(6.,0.)
        n.route=route((0.,0.),(6.,0.));n.recovery_attempts=3
        n.motion_queue=[route((0.,0.),(-2.,0.)),drivable_route((-2.,0.),(6.,0.))]
        n.odom.pose.pose.position.x=-2.
        n.send_motion_segment=Mock()
        n.on_follow_result(Mock(result=lambda:S(status=4)),n.epoch,(-2.,0.))
        self.assertEqual(n.recovery_attempts,3)
        n.send_motion_segment.assert_called_once_with(after_maneuver=True)

    def test_real_route_progress_can_restore_cusp_recovery_budget(self):
        n=self.node;n.grid.info.origin.position.x=n.grid.info.origin.position.y=-80.
        n.local_grid=deepcopy(n.grid);n.route_endpoint=(6.,0.)
        n.route=route((0.,0.),(6.,0.));n.recovery_attempts=3
        n.motion_queue=[route((0.,0.),(2.,0.)),drivable_route((2.,0.),(6.,0.))]
        n.odom.pose.pose.position.x=2.
        n.send_motion_segment=Mock()
        n.on_follow_result(Mock(result=lambda:S(status=4)),n.epoch,(2.,0.))
        self.assertEqual(n.recovery_attempts,0)
        n.send_motion_segment.assert_called_once_with(after_maneuver=True)


if __name__=='__main__': unittest.main()
