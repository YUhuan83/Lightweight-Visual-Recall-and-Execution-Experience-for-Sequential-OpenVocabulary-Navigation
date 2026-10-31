"""Recorded short-cusp approach must satisfy its actual stopped checker disk."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import Mock
import test_execution_mechanism_ros as fixtures
from test_graph_path_bridge_routing import route,pose


class ShortEndpointHandoffTests(unittest.TestCase):
    def setUp(self):
        fixtures.ExecutionMechanismTests.setUp(self)
        self.case=json.loads((Path(__file__).parent/'fixtures/physical_short_cusp_join_20261012.json').read_text())
        self.node.grid.info.origin.position.x=690.
        self.node.grid.info.origin.position.y=-70.
        self.node.local_grid=deepcopy(self.node.grid)
        self.node.expected=None;self.node.query=None
        self.node.start_segment=Mock()

    def tearDown(self):self.node.destroy_node()

    def test_recorded_short_cusp_outside_stop_disk_replans_before_dispatch(self):
        case=self.case;end=case['points'][-1];final=(end[0]-5.,end[1])
        for sample in case['observations']:
            with self.subTest(wall_time=sample['wall_time']):
                self.node.odom.pose.pose=pose(*sample['position']).pose
                self.node.odom.pose.pose.orientation.z=math.sin(sample['heading']/2)
                self.node.odom.pose.pose.orientation.w=math.cos(sample['heading']/2)
                self.node.route=route(sample['position'],final);self.node.route_endpoint=final
                path=route(*case['points'])
                for item,h in zip(path.poses,case['headings']):
                    item.pose.orientation.z=math.sin(h/2);item.pose.orientation.w=math.cos(h/2)
                self.node.motion_queue=[path,route(end,final)]
                self.node.follow_client.send_goal_async.reset_mock()
                self.node.following=False
                self.node.send_motion_segment(after_maneuver=True)
                self.node.follow_client.send_goal_async.assert_not_called()
                self.assertEqual(self.node.last_status['status'],'replanning_after_maneuver')
                self.assertEqual(self.node.last_status['join_failure_reason'],'join_curvature_exceeds_turning_limit')

    def test_valid_short_cusp_reports_the_same_endpoint_contract_it_dispatches(self):
        self.node.grid.info.origin.position.x=self.node.grid.info.origin.position.y=-80.
        self.node.local_grid=deepcopy(self.node.grid)
        self.node.route_endpoint=(6.,0.);self.node.route=route((0.,0.),(6.,0.))
        self.node.motion_queue=[route((0.,0.),(-1.7,0.)),route((-1.7,0.),(6.,0.))]
        self.node.send_motion_segment()
        goal=self.node.follow_client.send_goal_async.call_args.args[0]
        self.assertEqual(self.node.last_status['goal_checker_id'],goal.goal_checker_id)
        self.assertEqual(self.node.last_status['goal_position_tolerance_m'],.2)
        self.assertEqual(self.node.last_status['actual_start_pose'],[0.,0.,0.])
        self.assertEqual(self.node.last_status['actual_start_stamp_ns'],self.now)


if __name__=='__main__':unittest.main()
