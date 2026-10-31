"""Unlocalized observation cues must not turn exploration into parking retries."""
import json,math,unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import rclpy
from nav_msgs.msg import Path as RosPath
import test_graph_path_bridge_routing as fixture
import test_continuous_exploration_ros as continuous
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge


class WeakObservationBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.f=fixture.GraphBridgeRoutingTests()
        with patch.object(fixture,'GraphPathBridge',ContinuousGraphPathBridge):self.f.setUp()
        self.n=self.f.node;self.addCleanup(self.f.tearDown)
    def recorded_path(self):
        case=json.loads((Path(__file__).parent/'fixtures/weak_shunt_cases.json').read_text(encoding='utf8'))['cases'][0]
        x,y=case['points'][0];angle=case['headings'][0];c,s=math.cos(angle),math.sin(angle)
        path=RosPath();path.header.frame_id='spot1/odom';path.header.stamp=self.n.get_clock().now().to_msg()
        for (px,py),heading in zip(case['points'],case['headings']):
            p=fixture.pose(c*(px-x)+s*(py-y),-s*(px-x)+c*(py-y))
            p.pose.orientation.z=math.sin((heading-angle)/2);p.pose.orientation.w=math.cos((heading-angle)/2)
            path.poses.append(p)
        return path
    def result(self,path=None):
        path=path or fixture.drivable_route((0.,0.),(10.,0.))
        future=self.f.prepare_synchronous_dispatch_result(path)
        n=self.n;n.query='green residential house';n.visible_approach_active=True;n.visible_vehicle_goal=True
        endpoint=path.poses[-1].pose.position;target=(endpoint.x,endpoint.y)
        n.expected=target;n.route=n.latest_route=fixture.route((0.,0.),target);n.route_endpoint=target
        q=path.poses[-1].pose.orientation
        n.candidates=[(*target,2*math.atan2(q.z,q.w),math.hypot(*target))]
        n.release_visible_approach=Mock();n.compare_curvature_heading=Mock(return_value=False)
        return future
    def test_recorded_admitted_shunt_is_deferred_for_an_unlocalized_cue(self):
        f=self.result(self.recorded_path());self.n.on_plan_result(f,self.n.epoch,0,42)
        self.n.release_visible_approach.assert_called_once_with('visible_target_no_vehicle_path')
        self.n.send_motion_segment.assert_not_called()
    def test_empty_provisional_pose_returns_to_exploration_without_heading_sweep(self):
        f=self.result();self.n.candidates+=[(10.,0.,.2,10.)]
        path=RosPath();path.header.frame_id='spot1/odom'
        f=Future();f.set_result(S(status=4,result=S(path=path)))
        self.n.try_candidate=Mock();self.n.on_plan_result(f,self.n.epoch,0,42)
        self.n.release_visible_approach.assert_called_once_with('visible_target_no_vehicle_path')
        self.n.try_candidate.assert_not_called();self.assertFalse(self.n.pending)
    def test_forward_observation_keeps_normal_dispatch(self):
        f=self.result();self.n.on_plan_result(f,self.n.epoch,0,42)
        self.n.release_visible_approach.assert_not_called();self.n.send_motion_segment.assert_called_once()
    def test_confirmed_target_keeps_necessary_reverse_maneuver(self):
        f=self.result(self.recorded_path());self.n.visible_approach_active=False
        self.n.confirmed_route_goal=self.n.expected
        self.n.confirmed_route_object=(self.n.expected[0]+10.,self.n.expected[1])
        self.n.confirmed_route_heading=self.n.candidates[0][2]
        self.n.on_plan_result(f,self.n.epoch,0,42)
        self.n.release_visible_approach.assert_not_called();self.n.send_motion_segment.assert_called_once()
    def test_legacy_cue_without_vehicle_pose_contract_is_unchanged(self):
        f=self.result(self.recorded_path());self.n.visible_vehicle_goal=False
        self.n.on_plan_result(f,self.n.epoch,0,42)
        self.n.release_visible_approach.assert_not_called();self.n.send_motion_segment.assert_called_once()
    def test_memory_navigation_is_not_treated_as_a_weak_cue(self):
        f=self.result();self.n.memory_goal=(10.,0.)
        self.n.try_candidate=Mock();self.n.candidate_failed('empty_path')
        self.n.release_visible_approach.assert_not_called()
    def test_transport_error_keeps_diagnostic_and_retry_handling(self):
        self.result();self.n.try_candidate=Mock()
        self.n.candidate_failed('planner_error:transport')
        self.n.release_visible_approach.assert_not_called();self.n.try_candidate.assert_called_once()
    def test_late_plan_cannot_release_the_current_observation(self):
        f=self.result(self.recorded_path());self.n.on_plan_result(f,self.n.epoch,0,41)
        self.n.release_visible_approach.assert_not_called();self.n.validate_path.assert_not_called()
    def test_failed_future_observation_retains_healthy_exploration_motion(self):
        other=continuous.ContinuousExplorationTests();other.setUp();self.addCleanup(other.doCleanups)
        n=other.moving();handle=n.follow_handle;epoch=n.epoch
        n.visible_approach_active=True;n.visible_vehicle_goal=True;n.expected=(8.,0.)
        n.candidates=[(8.,0.,0.,8.)];n.candidate_index=0;n.try_candidate=Mock()
        n.candidate_failed('empty_path')
        self.assertTrue(n.following);self.assertIs(n.follow_handle,handle);self.assertEqual(n.epoch,epoch)
        self.assertFalse(n.visible_approach_active);self.assertIsNone(n.expected)
        handle.cancel_goal_async.assert_not_called();n.try_candidate.assert_not_called()
