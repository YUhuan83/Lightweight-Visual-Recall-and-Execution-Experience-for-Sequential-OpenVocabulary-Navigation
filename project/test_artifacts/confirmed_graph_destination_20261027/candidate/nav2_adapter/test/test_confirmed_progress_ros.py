"""Confirmed targets retain bounded navigation without weakening final arrival."""
import json
import math
import time
import unittest
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
from beamng_wildos_adapter.visible_approach import observation_step
from wildos_nav2_adapter.graph_route import footprint_blockers


class ConfirmedProgressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.VisibleApproachTests.tearDownClass()
    def setUp(self):
        self.fixture=fixture.VisibleApproachTests();self.fixture.setUp()
        self.node=self.fixture.perception
        self.node.publisher=Mock()
    def tearDown(self):self.fixture.tearDown()

    def pending(self,position=(20.,0.,2.)):
        target=PoseStamped();target.header.frame_id='spot1/odom'
        target.header.stamp=self.fixture.stamp()
        target.pose.position.x,target.pose.position.y,target.pose.position.z=position
        data=dict(query=self.node.active_query,frame='spot1/odom',
            observation_stamp_ns=self.fixture.now,object_position=list(position),
            observer_position=[0.,0.,0.],observer_orientation=[0.,0.,0.,1.],
            radius=.2,inliers=4,observations=4,source='lidar',
            position_confirmed=True,localization_valid=True,
            quality=dict(confidence=.8,association_validated=True))
        self.node.pending_confirmation=(target,data)
        return target,data

    def test_pending_without_standoff_keeps_fresh_matching_observation_guidance(self):
        self.pending()
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_called_once()
        proposal=json.loads(self.node.visible_approach_pub.publish.call_args.args[0].data)
        self.assertFalse(proposal['search_confirmed'])
        self.assertFalse(proposal['position_confirmed'])
        self.assertEqual(proposal['evidence'],'bearing_only')
        self.assertIsNotNone(self.node.pending_confirmation)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_accepted_confirmed_approach_is_not_preempted_by_bearing_proposals(self):
        self.node.active_approach=self.pending()
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_not_called()

    def test_other_bearing_cannot_redirect_pending_confirmed_instance(self):
        self.pending((0.,20.,2.))
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.assert_not_called()

    def test_query_switch_clears_pending_geometry_before_fresh_guidance(self):
        self.pending()
        self.fixture.query('gas station')
        self.assertIsNone(self.node.pending_confirmation)
        self.assertEqual(json.loads(self.fixture.detect().data)['query'],'gas station')

    def test_stale_mask_still_cannot_drive_while_confirmation_is_pending(self):
        self.pending()
        for _ in range(3):
            self.fixture.now+=100_000_000
            self.node.on_visible_target(self.fixture.mask())
        self.node.visible_approach_pub.publish.reset_mock()
        self.fixture.now+=3_000_000_000
        self.node.publish_visible_step()
        self.node.visible_approach_pub.publish.assert_not_called()

    def snapshot(self):
        data=np.load(Path(__file__).parent/'fixtures/standoff_first_snapshot.npz')
        meta=json.loads(str(data['meta']))
        self.fixture.now=meta['now_ns']
        def stamp(t):return Time(sec=t//10**9,nanosec=t%10**9)
        def odom(m):
            msg=Odometry();msg.header.frame_id='spot1/odom';msg.header.stamp=stamp(m['stamp_ns'])
            p=msg.pose.pose.position;p.x,p.y,p.z=map(float,m['position'])
            q=msg.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,m['orientation'])
            return msg
        def grid(name):
            m=meta[name];msg=OccupancyGrid();msg.header.frame_id=m['frame'];msg.header.stamp=stamp(m['stamp_ns'])
            msg.info.width=m['width'];msg.info.height=m['height'];msg.info.resolution=float(m['resolution'])
            p=msg.info.origin.position;p.x,p.y,p.z=map(float,m['origin'][:3])
            q=msg.info.origin.orientation;q.x,q.y,q.z,q.w=map(float,m['origin'][3:])
            msg.data=[int(v) for v in data[name]]
            return msg
        n=self.node;n.odom=odom(meta['odom']);n.occupancy=grid('local_map')
        n.occupancy_odom=odom(meta['map_odom']);n.vehicle_grid=grid('vehicle_map')
        n.vehicle_connectivity=None;n.active_query=meta['confirmation']['query']
        original=n.get_parameter
        n.get_parameter=lambda k:S(value=3.) if k=='standoff' else original(k)
        target=PoseStamped();target.header.frame_id='spot1/odom'
        target.header.stamp=stamp(meta['confirmation']['observation_stamp_ns'])
        target.pose.position.x,target.pose.position.y,target.pose.position.z=map(float,meta['confirmation']['object_position'])
        return meta,target

    def test_recorded_blocked_ray_gets_a_checked_short_observation_goal(self):
        meta,target=self.snapshot();n=self.node
        self.assertIsNone(n.publish_standoff(target))
        point=n.last_goal;robot=n.odom.pose.pose.position
        self.assertGreaterEqual(math.dist(point,(robot.x,robot.y)),2.)
        self.assertLessEqual(math.dist(point,(robot.x,robot.y)),n.visible_observation_max_m+.1)
        heading=math.atan2(target.pose.position.y-point[1],target.pose.position.x-point[0])
        self.assertFalse(footprint_blockers(n.vehicle_grid,point,heading))
        self.assertTrue(n.last_standoff_clamped)
        self.assertGreater(n.last_standoff_object_distance,20.)

    def test_recorded_intermediate_goal_is_navigation_not_search_success(self):
        meta,target=self.snapshot();n=self.node
        n.pending_confirmation=(target,deepcopy(meta['confirmation']))
        n.retry_confirmation(force=True)
        result=json.loads(n.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(result['search_confirmed']);self.assertTrue(result['approach_goal_ready'])
        self.assertIsNotNone(result['standoff']);self.assertTrue(result['standoff_clamped'])
        self.assertEqual(result['observation_stamp_ns'],meta['confirmation']['observation_stamp_ns'])
        bridge=self.fixture.bridge;bridge.carrier_model='ackermann'
        bridge.odom=deepcopy(n.odom);bridge.odom_received_at=time.monotonic()
        bridge.on_object_confirmation(String(data=json.dumps(result)))
        self.assertFalse(bridge.object_found)
        self.assertEqual(bridge.expected,tuple(result['standoff']))

    def test_no_free_vehicle_pose_does_not_create_a_fallback_goal(self):
        _,target=self.snapshot();n=self.node
        n.vehicle_grid.data=[100]*len(n.vehicle_grid.data)
        self.assertEqual(n.publish_standoff(target),'no_reachable_standoff')
        self.assertIsNone(n.last_goal);n.publisher.publish.assert_not_called()

    def test_observation_sampling_keeps_the_inclusive_two_metre_boundary(self):
        with patch('beamng_wildos_adapter.visible_approach.safe_travel_distance',
                   return_value=7.999999999999999):
            point=observation_step(np.zeros((40,40)),.5,(-10.,-10.),(0.,0.),0.,0.,
                accept=lambda p:math.hypot(*p)<=2.+1e-12)
        self.assertEqual(point,(2.,0.))
