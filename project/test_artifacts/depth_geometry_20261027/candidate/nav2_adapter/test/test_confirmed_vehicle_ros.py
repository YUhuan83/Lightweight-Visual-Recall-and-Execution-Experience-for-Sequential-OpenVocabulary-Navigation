"""Confirmed intermediate poses keep vehicle geometry through the full handoff."""
import json,math,time,unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
import test_visible_approach_ros as fixture
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge


class ConfirmedVehicleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.VisibleApproachTests.tearDownClass()
    def setUp(self):
        self.f=fixture.VisibleApproachTests()
        with patch.object(fixture,'GraphPathBridge',ContinuousGraphPathBridge):self.f.setUp()
        self.n=self.f.perception;self.b=self.f.bridge
        self.n.observation_minimum_turning_radius=12.
        self.b.carrier_model='ackermann';self.b.minimum_turning_radius=12.
        self.n.publisher=Mock()
    def tearDown(self):self.f.tearDown()

    def recorded(self):
        data=np.load(Path(__file__).parent/'fixtures/confirmed_vehicle_snapshot.npz')
        meta=json.loads(str(data['meta']));self.f.now=meta['now_ns']
        def stamp(t):return Time(sec=t//10**9,nanosec=t%10**9)
        def odom(m):
            out=Odometry();out.header.frame_id='spot1/odom';out.header.stamp=stamp(m['stamp_ns'])
            p=out.pose.pose.position;p.x,p.y,p.z=map(float,m['position'])
            q=out.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,m['orientation'])
            return out
        def grid(name):
            m=meta[name];out=OccupancyGrid();out.header.frame_id=m['frame'];out.header.stamp=stamp(m['stamp_ns'])
            i=out.info;i.width=m['width'];i.height=m['height'];i.resolution=float(m['resolution'])
            p=i.origin.position;p.x,p.y,p.z=map(float,m['origin'][:3])
            q=i.origin.orientation;q.x,q.y,q.z,q.w=map(float,m['origin'][3:])
            out.data=[int(v) for v in data[name]];return out
        n=self.n;n.odom=odom(meta['odom']);n.occupancy_odom=odom(meta['map_odom'])
        n.occupancy=grid('local_map');n.vehicle_grid=grid('vehicle_map');n.vehicle_connectivity=None
        n.active_query=meta['confirmation']['query']
        original=n.get_parameter;n.get_parameter=lambda key:S(value=3.) if key=='standoff' else original(key)
        self.b.odom=deepcopy(n.odom);self.b.odom_received_at=time.monotonic()
        target=PoseStamped();target.header.frame_id='spot1/odom';target.header.stamp=stamp(meta['confirmation']['observation_stamp_ns'])
        p=target.pose.position;p.x,p.y,p.z=map(float,meta['confirmation']['object_position'])
        return target,meta['confirmation']

    def generic(self):
        return dict(query=self.b.query,frame='spot1/odom',standoff=[8.,1.],object_position=[20.,8.,2.],radius=.2,
            standoff_clamped=True,standoff_navigation_heading=2*math.atan2(1.,8.))

    def test_recorded_confirmed_target_gets_a_vehicle_intermediate_point(self):
        target,_=self.recorded()
        self.assertIsNone(self.n.publish_standoff(target))
        self.assertIsNotNone(self.n.last_goal)
        self.assertTrue(self.n.last_standoff_clamped)
        self.assertGreater(math.dist(self.n.last_goal,(target.pose.position.x,target.pose.position.y)),20.)

    def test_recorded_heading_survives_confirmation_and_navigation_handoff(self):
        target,c=self.recorded();self.n.pending_confirmation=(target,deepcopy(c))
        self.n.retry_confirmation(force=True)
        message=self.n.confirmation_pub.publish.call_args.args[0];d=json.loads(message.data)
        self.assertTrue(d['approach_goal_ready']);self.assertIn('standoff_navigation_heading',d)
        self.b.on_object_confirmation(message)
        self.assertEqual(self.b.expected,tuple(d['standoff']))
        self.assertAlmostEqual(self.b.exploration_observation_heading(),d['standoff_navigation_heading'])
        self.assertFalse(self.b.object_found)

    def test_vehicle_terminal_heading_is_not_overwritten_to_face_object(self):
        d=self.generic();self.b.route_confirmed_target(d,(0.,0.))
        goal=self.b.waypoint_pub.publish.call_args.args[0]
        q=goal.pose.orientation
        self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z),d['standoff_navigation_heading'])
        self.assertAlmostEqual(self.b.exploration_observation_heading(),d['standoff_navigation_heading'])

    def test_legacy_final_view_still_faces_confirmed_object(self):
        d=self.generic();d.pop('standoff_navigation_heading')
        self.b.route_confirmed_target(d,(0.,0.))
        self.assertAlmostEqual(self.b.exploration_observation_heading(),math.atan2(7.,12.))

    def test_carried_heading_cannot_turn_an_intermediate_point_into_arrival(self):
        target,c=self.recorded();self.n.pending_confirmation=(target,deepcopy(c))
        self.n.retry_confirmation(force=True)
        self.b.on_object_confirmation(self.n.confirmation_pub.publish.call_args.args[0])
        self.assertFalse(self.b.object_found)

    def test_invalid_intermediate_heading_does_not_mutate_navigation(self):
        _,d=self.recorded();d=deepcopy(d)
        d.update(standoff=[self.b.odom.pose.pose.position.x+5.,self.b.odom.pose.pose.position.y],
                 standoff_clamped=True,standoff_navigation_heading=float('nan'),approach_goal_ready=True)
        self.b.on_object_confirmation(String(data=json.dumps(d)))
        self.assertIsNone(self.b.expected)

    def test_query_switch_clears_intermediate_heading(self):
        d=self.generic();self.b.route_confirmed_target(d,(0.,0.))
        self.f.query('gas station')
        self.assertIsNone(self.b.confirmed_route_goal)
        self.assertIsNone(self.b.exploration_observation_heading())

    def test_no_vehicle_map_does_not_invent_a_curved_stage(self):
        target,_=self.recorded();self.n.vehicle_grid=None
        self.n.occupancy.data=[100]*len(self.n.occupancy.data)
        self.assertIsNotNone(self.n.publish_standoff(target))
        self.assertIsNone(self.n.last_goal)

    def test_refresh_preserves_source_stamp_and_vehicle_heading(self):
        target,c=self.recorded();self.assertIsNone(self.n.publish_standoff(target))
        self.n.publish_approach_refresh(c)
        d=json.loads(self.n.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(d['observation_stamp_ns'],c['observation_stamp_ns'])
        self.assertEqual(d['base_observation_stamp_ns'],c['observation_stamp_ns'])
        self.assertAlmostEqual(d['standoff_navigation_heading'],self.n.last_standoff_navigation_heading)

    def test_active_route_defers_new_heading_until_that_route_finishes(self):
        d=self.generic();self.b.route_confirmed_target(d,(0.,0.))
        self.b.following=True
        proposal=dict(d,standoff=[15.,2.],standoff_navigation_heading=.3)
        self.b.route_confirmed_target(proposal,(4.,0.))
        self.assertEqual(self.b.expected,(8.,1.))
        self.assertAlmostEqual(self.b.exploration_observation_heading(),d['standoff_navigation_heading'])
        self.assertEqual(self.b.deferred_confirmed_approach['standoff_navigation_heading'],.3)

    def test_planner_request_uses_the_heading_accepted_from_the_producer(self):
        from concurrent.futures import Future
        from test_graph_path_bridge_routing import route
        target,c=self.recorded();self.n.pending_confirmation=(target,deepcopy(c))
        self.n.retry_confirmation(force=True)
        message=self.n.confirmation_pub.publish.call_args.args[0];d=json.loads(message.data)
        b=self.b;b.on_object_confirmation(message)
        p=b.odom.pose.pose.position
        b.route=route((p.x,p.y),tuple(d['standoff']));b.route.header.frame_id='spot1/odom'
        b.route.header.stamp=self.f.stamp();b.route_endpoint=tuple(d['standoff'])
        b.grid=deepcopy(self.n.vehicle_grid);b.grid_arrived_at=b.grid_received_at=time.monotonic()
        b.vehicle_pose_feasible=Mock(return_value=True)
        b.plan_client=Mock();b.plan_client.wait_for_server.return_value=True
        b.plan_client.send_goal_async.return_value=Future()
        b.start_segment()
        request=b.plan_client.send_goal_async.call_args.args[0]
        q=request.goal.pose.orientation
        self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z),d['standoff_navigation_heading'])
        self.assertAlmostEqual(request.goal.pose.position.x,d['standoff'][0])
        self.assertAlmostEqual(request.goal.pose.position.y,d['standoff'][1])

    def test_same_point_retains_the_heading_that_belongs_to_the_published_goal(self):
        target,_=self.recorded();self.assertIsNone(self.n.publish_standoff(target))
        point=self.n.last_goal;heading=self.n.last_standoff_navigation_heading
        self.assertIsNone(self.n.publish_standoff(target))
        self.assertEqual(self.n.last_goal,point)
        self.assertEqual(self.n.last_standoff_navigation_heading,heading)

    def test_final_view_cannot_supply_an_intermediate_heading(self):
        _,d=self.recorded();d=deepcopy(d)
        d.update(standoff=[self.b.odom.pose.pose.position.x+5.,self.b.odom.pose.pose.position.y],
                 standoff_clamped=False,standoff_navigation_heading=.5,approach_goal_ready=True)
        self.b.on_object_confirmation(String(data=json.dumps(d)))
        self.assertIsNone(self.b.expected)
