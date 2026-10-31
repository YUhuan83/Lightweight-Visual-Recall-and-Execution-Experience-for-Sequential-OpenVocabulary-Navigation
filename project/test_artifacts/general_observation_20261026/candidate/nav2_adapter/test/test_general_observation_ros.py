"""An accepted observation uses real planning, not a single-circle impossibility claim."""
import math,time,unittest
from concurrent.futures import Future
from copy import deepcopy
from unittest.mock import Mock,patch
import test_vehicle_observation_ros as fixture
from test_graph_path_bridge_routing import route
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading

class GeneralObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.VehicleObservationTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.VehicleObservationTests.tearDownClass()
    def setUp(self):
        self.f=fixture.VehicleObservationTests();self.f.setUp();self.b=self.f.b
        self.b.vehicle_pose_feasible=Mock(return_value=True)
    def tearDown(self):self.f.tearDown()
    def accepted_recorded_point(self):
        b=self.b;b.visible_approach_active=b.visible_vehicle_goal=True
        b.expected=(736.1648774856949,-72.51375390061973)
        b.odom=deepcopy(b.odom);p=b.odom.pose.pose
        p.position.x=734.244842888888;p.position.y=-70.04092116226539
        heading=-1.0416493214087026
        p.orientation.z=math.sin(heading/2);p.orientation.w=math.cos(heading/2)
        b.visible_hint=None
        b.release_visible_approach=Mock()
        return b
    def test_recorded_single_circle_reject_still_admits_actual_planner(self):
        b=self.accepted_recorded_point()
        self.assertIsNone(b.visible_navigation_heading({'navigation_heading':0.},b.expected))
        self.assertTrue(b.prepare_visible_vehicle_goal())
        b.release_visible_approach.assert_not_called()
        self.assertFalse(b.object_found)
    def test_passed_accepted_point_is_released(self):
        b=self.accepted_recorded_point();b.expected=(730.,-67.)
        self.assertFalse(b.prepare_visible_vehicle_goal())
        b.release_visible_approach.assert_called_once_with('visible_target_no_vehicle_path')
    def test_blocked_forward_pose_is_not_admitted(self):
        b=self.accepted_recorded_point();b.vehicle_pose_feasible.return_value=False
        self.assertFalse(b.prepare_visible_vehicle_goal())
        b.release_visible_approach.assert_called_once()
    def fallback(self,car=True):
        b=self.b;b.visible_approach_active=True;b.visible_vehicle_goal=car
        b.expected=(8.,1.);b.route=route((0.,0.),b.expected);b.route_endpoint=b.expected
        b.grid=deepcopy(self.f.n.vehicle_grid);b.route_complete=True
        b.grid_arrived_at=b.grid_received_at=time.monotonic()
        b.execution_transport_issue=Mock(return_value=None);b.pose_blockers=Mock(return_value=[])
        b.plan_client=Mock();b.plan_client.wait_for_server.return_value=True
        b.plan_client.send_goal_async.return_value=Future()
        b.follow_enabled=True;b.follow_release=None
        with patch('wildos_nav2_adapter.graph_path_bridge.route_candidates',return_value=[]):b.start_segment()
        return b.plan_client.send_goal_async.call_args.args[0]
    def test_local_fallback_keeps_circle_terminal_heading(self):
        goal=self.fallback();q=goal.goal.pose.orientation
        self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z),forward_arc_arrival_heading((0.,0.),0.,(8.,1.),12.))
        self.assertFalse(self.b.object_found)
    def test_legacy_fallback_keeps_existing_bearing(self):
        goal=self.fallback(car=False);q=goal.goal.pose.orientation
        self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z),math.atan2(1.,8.))
    def test_new_tight_side_hint_remains_unaccepted(self):
        self.f.test_expired_vehicle_pose_at_consumer_does_not_start_a_shunt()
