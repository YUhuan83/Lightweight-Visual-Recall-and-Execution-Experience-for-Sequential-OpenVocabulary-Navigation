"""Common detection-to-observation contracts, using real adapter callbacks."""
import json
import math
import unittest
import rclpy
from unittest.mock import patch
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from beamng_wildos_adapter.visible_approach import observation_step
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from test_graph_path_bridge_routing import route


class VisibleApproachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.now = 100*10**9
        self.perception = GoalStandoff()
        self.bridge = GraphPathBridge()
        for node,names in ((self.perception,('visible_approach_pub','confirmation_pub')),
                           (self.bridge,('waypoint_pub','status_pub'))):
            for name in names:
                setattr(node,name,Mock())
        clock = lambda: S(now=lambda: S(nanoseconds=self.now, to_msg=lambda: self.stamp()))
        self.perception.get_clock = self.bridge.get_clock = clock
        self.bridge.input_freshness_issue = lambda: None
        self.bridge.retry_graph_reset = Mock()
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.header.stamp = self.stamp()
        odom.pose.pose.orientation.w = 1.
        self.perception.on_odom(odom)
        self.bridge.odom = odom
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp = self.stamp()
        grid.info.width = grid.info.height = 30
        grid.info.resolution = 1.
        grid.info.origin.position.x = grid.info.origin.position.y = -15.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*900
        self.perception.on_occupancy(grid)
        self.query('green residential house')

    def stamp(self):
        return Time(sec=self.now//10**9, nanosec=self.now%10**9)

    def query(self, text):
        self.perception.on_query(String(data=text))
        self.bridge.on_query(String(data=text))
        self.bridge.query_reset_pending = False

    def mask(self, text=None):
        header = lambda frame: S(stamp=self.stamp(), frame_id=frame)
        data = np.zeros((30, 40), np.uint8)
        data[10:21, 15:26] = 1
        info = S(header=header('camera'), width=40, height=30,
                 k=[20., 0., 20., 0., 20., 15., 0., 0., 1.])
        tf = S(header=header('spot1/odom'), child_frame_id='camera',
               transform=S(rotation=S(x=-.5, y=.5, z=-.5, w=.5)))
        return S(header=header('spot1/odom'), query=text or self.perception.active_query,
                 object_mask=S(layout=S(dim=[S(size=v) for v in (1, 1, 30, 40)], data_offset=0),
                               data=data.ravel().tolist()),
                 cam_infos=[info], cam_transforms=S(transforms=[tf]))

    def detect(self, text=None):
        for _ in range(3):
            self.now += 100_000_000
            self.perception.on_visible_target(self.mask(text))
        return self.perception.visible_approach_pub.publish.call_args.args[0]

    def tearDown(self):
        self.perception.destroy_node()
        self.bridge.destroy_node()

    def test_house_and_station_share_the_same_bounded_approach_pipeline(self):
        for query in ('green residential house', 'gas station'):
            self.query(query)
            message = self.detect()
            data = json.loads(message.data)
            self.assertEqual(data['query'], query)
            self.assertFalse(data['position_confirmed'])
            self.assertFalse(data['search_confirmed'])
            self.assertAlmostEqual(data['point'][0], 8.)
            self.bridge.on_visible_approach(message)
            self.assertEqual(self.bridge.expected, (8., 0.))
            self.assertTrue(self.bridge.visible_approach_active)
            self.assertFalse(self.bridge.object_found)
            self.assertEqual(self.bridge.last_status['source'], 'visible_target_via_graphnav')
            self.assertEqual(self.bridge.waypoint_pub.publish.call_args.args[0].header.frame_id, 'spot1/odom')
            self.perception.confirmation_pub.publish.assert_not_called()

    def prepare_moving_visible_goal(self):
        self.bridge.odom.pose.pose.position.x=0.
        self.bridge.odom.pose.pose.position.y=0.
        message=self.detect();self.bridge.on_visible_approach(message)
        node=self.bridge
        node.odom.pose.pose.position.x=4.
        node.following=True;node.follow_handle=Mock()
        node.motion_queue=[route((0.,0.),(8.,0.))]
        node.visible_hint=dict(json.loads(message.data),point=[12.,0.],
                               observation_stamp_ns=self.now)
        node.route=route((0.,0.),(8.,0.))
        node.grid=OccupancyGrid();node.grid.header.frame_id='spot1/odom'
        return node

    def test_fresh_forward_observation_advances_without_resetting_motion(self):
        node=self.prepare_moving_visible_goal()
        epoch,queue,handle=node.epoch,node.motion_queue,node.follow_handle
        self.assertTrue(node.advance_visible_goal())
        self.assertEqual(node.expected,(12.,0.))
        self.assertEqual(node.epoch,epoch)
        self.assertIs(node.motion_queue,queue)
        self.assertIs(node.follow_handle,handle)
        self.assertTrue(node.following)
        self.assertTrue(node.visible_goal_refresh_pending)
        self.assertTrue(node.last_status['continuous'])
        handle.cancel_goal_async.assert_not_called()
        self.assertFalse(node.object_found)

    def test_advanced_goal_waits_for_matching_graph_route(self):
        node=self.prepare_moving_visible_goal();node.advance_visible_goal()
        node.route_diagnostic=Mock()
        old=route((4.,0.),(8.,0.));old.header.stamp=self.stamp()
        node.route_metadata[self.now]=dict(goal=(8.,0.),complete=True,mode='known',exploration_id=None)
        node.on_route(old)
        self.assertTrue(node.visible_goal_refresh_pending)
        self.assertIsNone(node.latest_route)
        self.now+=100_000_000
        new=route((4.,0.),(12.,0.));new.header.stamp=self.stamp()
        node.route_metadata[self.now]=dict(goal=(12.,0.),complete=True,mode='known',exploration_id=None)
        node.on_route(new)
        self.assertFalse(node.visible_goal_refresh_pending)
        self.assertEqual(node.latest_route.poses[-1].pose.position.x,12.)
        self.assertTrue(node.following)

    def test_stale_backwards_or_small_hint_does_not_retarget_motion(self):
        for point,age in (([12.,0.],3_000_000_000),([-4.,0.],0),([9.,0.],0)):
            with self.subTest(point=point,age=age):
                self.query('gas station' if self.bridge.query!='gas station' else 'green residential house')
                node=self.prepare_moving_visible_goal()
                node.visible_hint.update(point=point,observation_stamp_ns=self.now-age)
                self.assertFalse(node.advance_visible_goal())
                self.assertEqual(node.expected,(8.,0.))
                self.assertFalse(node.visible_goal_refresh_pending)

    def test_confirmed_query_or_quarantine_blocks_visible_goal_advance(self):
        for attribute,value in (('active_object_confirmation',{'instance_id':'x'}),
                                ('query_reset_pending',True),('follow_quarantine',{7:{}}),
                                ('lookback_active',True)):
            with self.subTest(attribute=attribute):
                node=self.prepare_moving_visible_goal()
                old=getattr(node,attribute);setattr(node,attribute,value)
                self.assertFalse(node.advance_visible_goal())
                self.assertEqual(node.expected,(8.,0.))
                setattr(node,attribute,old)

    def test_query_change_discards_pending_visible_goal_refresh(self):
        node=self.prepare_moving_visible_goal();node.advance_visible_goal()
        self.query('gas station')
        self.assertFalse(node.visible_goal_refresh_pending)
        self.assertFalse(node.visible_approach_active)
        self.assertIsNone(node.expected)

    def test_configured_observation_step_shortens_visible_point(self):
        self.perception.visible_observation_max_m = 5.
        data = json.loads(self.detect().data)
        self.assertEqual(data['point'][0], 5.)

    def test_sparse_consistent_masks_can_start_observation_when_configured(self):
        self.perception.visible_approach_masks = 2
        self.perception.visible_approach_continuity_sec = 15.
        self.perception.fresh = lambda *args: True
        self.perception.on_visible_target(self.mask())
        self.perception.visible_approach_pub.publish.assert_not_called()
        self.now += 12_000_000_000
        self.perception.on_visible_target(self.mask())
        self.perception.visible_approach_pub.publish.assert_called_once()
        data = json.loads(
            self.perception.visible_approach_pub.publish.call_args.args[0].data)
        self.assertEqual(data['evidence'], 'bearing_only')
        self.assertFalse(data['position_confirmed'])
        self.assertFalse(data['search_confirmed'])

    def test_visible_approach_lifetime_releases_and_blocks_target(self):
        self.bridge.on_visible_approach(self.detect())
        self.assertTrue(self.bridge.visible_approach_active)
        self.bridge.visible_approach_deadline = -1.
        self.assertTrue(self.bridge.check_visible_approach_deadline())
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertIsNotNone(self.bridge.visible_blocked)

    def test_new_hint_is_queued_without_canceling_a_draining_section(self):
        node = self.bridge
        message = self.detect()
        data = json.loads(message.data)
        data['point'] = [6., 2.]
        message.data = json.dumps(data)
        node.query_reset_pending = True
        node.retired_motion = dict(identity='visible-timeout:1',
                                   owner_kind='visible_target_timeout')
        node.following = True
        node.follow_handle = Mock()
        epoch = node.epoch
        node.reset = Mock()

        node.on_visible_approach(message)

        node.reset.assert_not_called()
        self.assertEqual(node.epoch, epoch)
        self.assertTrue(node.following)
        self.assertEqual(node.expected, (6., 2.))
        self.assertTrue(node.visible_approach_active)
        self.assertTrue(node.last_status['handoff_deferred'])
        self.assertIsNotNone(node.pending_destination)

    def test_new_frames_do_not_cancel_waiting_or_executing_goal(self):
        message = self.detect()
        self.bridge.on_visible_approach(message)
        epoch = self.bridge.epoch
        for following in (False, True):
            self.bridge.following = following
            for x in (8.1, 8.3, 9.):
                data = json.loads(message.data)
                data['point'][0] = x
                self.bridge.on_visible_approach(String(data=json.dumps(data)))
        self.assertEqual(self.bridge.epoch, epoch)
        self.assertEqual(self.bridge.expected, (8., 0.))

    def test_confirmed_waypoint_can_replace_unconfirmed_observation_goal(self):
        self.bridge.on_visible_approach(self.detect())
        goal = PoseStamped()
        goal.header.frame_id = 'spot1/odom'
        goal.header.stamp = self.stamp()
        goal.pose.position.x = 20.
        goal.pose.orientation.w = 1.
        self.bridge.on_waypoint(goal)
        self.assertEqual(self.bridge.expected, (20., 0.))
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertFalse(self.bridge.object_found)

    def test_observation_endpoint_never_counts_as_object_arrival(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3*10**9
        with patch('time.monotonic',return_value=100.):
            self.bridge.await_object_observation()
        with patch('time.monotonic',return_value=111.):
            self.bridge.activate_visible_approach()
        self.assertIsNone(self.bridge.expected)
        self.assertFalse(self.bridge.object_found)
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertTrue(self.bridge.query_reset_pending)
        self.assertEqual(self.bridge.unconfirmed_approaches, [])

    def test_recorded_three_second_mask_gap_preserves_pursuit_at_endpoint(self):
        self.bridge.on_visible_approach(self.detect())
        self.bridge.odom.pose.pose.position.x = 8.
        self.now += 3_174_028_397  # Closed 151623: latest mask age at route_done.
        self.bridge.await_object_observation()
        self.assertIsNone(self.bridge.expected)
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertTrue(self.bridge.visible_reobserve_exploration)
        self.assertTrue(self.bridge.query_reset_pending)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'visible_target_reobserving')
        self.assertTrue(self.bridge.last_status['exploration_fallback_requested'])

    def test_established_candidate_reacquires_after_recorded_detection_gap(self):
        self.detect()
        self.perception.visible_approach_pub.reset_mock()
        self.now += 9_714_000_464  # Capture 393.957 -> 403.671 in closed bag.
        self.perception.odom.header.stamp = self.stamp()
        self.perception.occupancy.header.stamp = self.stamp()
        self.perception.on_visible_target(self.mask())
        self.assertEqual(self.perception.visible_target_count, 3)
        self.perception.visible_approach_pub.publish.assert_called_once()

    def test_reobserved_target_continues_without_frame_by_frame_replanning(self):
        message=self.detect()
        self.bridge.on_visible_approach(message)
        self.bridge.odom.pose.pose.position.x = 8.
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        data=json.loads(message.data)
        data.update(observation_stamp_ns=self.now,point=[16.,0.])
        self.bridge.query_reset_pending=False
        self.bridge.on_visible_approach(String(data=json.dumps(data)))
        self.assertEqual(self.bridge.expected,(16.,0.))
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertFalse(self.bridge.query_reset_pending)
        epoch=self.bridge.epoch
        self.bridge.following=True
        data['point']=[17.,0.]
        self.bridge.on_visible_approach(String(data=json.dumps(data)))
        self.assertEqual(self.bridge.expected,(16.,0.))
        self.assertEqual(self.bridge.epoch,epoch)

    def test_reobservation_deadline_does_not_refresh_on_stale_messages(self):
        message=self.detect()
        self.bridge.on_visible_approach(message)
        self.now += 3_174_028_397
        with patch('time.monotonic',return_value=100.):
            self.bridge.await_object_observation()
        with patch('time.monotonic',return_value=111.):
            self.bridge.on_visible_approach(message)
            self.bridge.activate_visible_approach()
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertIsNone(self.bridge.expected)
        self.assertTrue(self.bridge.query_reset_pending)

    def test_new_candidate_still_requires_three_fresh_consistent_frames(self):
        self.perception.on_visible_target(self.mask())
        self.now += 3*10**9
        self.perception.on_visible_target(self.mask())
        self.assertEqual(self.perception.visible_target_count,1)
        self.perception.visible_approach_pub.publish.assert_not_called()

    def test_graph_route_is_not_suppressed_while_exploration_reobserves(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        self.bridge.route_diagnostic=Mock()
        self.bridge.start_segment=Mock()
        route_message=Path()
        route_message.header.frame_id='spot1/odom'
        route_message.header.stamp=self.stamp()
        self.bridge.query_reset_pending=False
        self.bridge.on_route(route_message)
        self.assertEqual(self.bridge.route_diagnostic.call_args.args[0], 'empty_path')
        self.bridge.start_segment.assert_not_called()

    def test_reobservation_timeout_releases_wait_without_resetting_exploration(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        with patch('time.monotonic', return_value=100.):
            self.bridge.await_object_observation()
        self.assertTrue(self.bridge.visible_reobserve_exploration)
        self.bridge.status_pub.reset_mock()
        with patch('time.monotonic', return_value=111.):
            self.assertTrue(self.bridge.check_visible_reobserve_deadline())
        self.assertFalse(self.bridge.visible_reobserve_exploration)
        self.assertIsNone(self.bridge.visible_reobserve_until)
        self.assertTrue(self.bridge.query_reset_pending)
        status=json.loads(self.bridge.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['status'], 'visible_target_reobservation_timeout')
        self.assertTrue(status['exploration_fallback_requested'])

    def test_query_change_releases_reobservation_and_fences_old_target(self):
        message=self.detect()
        self.bridge.on_visible_approach(message)
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        self.query('gas station')
        self.bridge.on_visible_approach(message)
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertIsNone(self.bridge.visible_reobserve_until)
        self.assertIsNone(self.bridge.expected)

    def test_confirmed_waypoint_supersedes_wait_and_old_timer_cannot_clear_it(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        with patch('time.monotonic',return_value=100.):
            self.bridge.await_object_observation()
        goal=PoseStamped()
        goal.header.frame_id='spot1/odom';goal.header.stamp=self.stamp()
        goal.pose.position.x=20.;goal.pose.orientation.w=1.
        self.bridge.on_waypoint(goal)
        self.assertIsNone(self.bridge.visible_reobserve_until)
        with patch('time.monotonic',return_value=111.):
            self.bridge.activate_visible_approach()
        self.assertEqual(self.bridge.expected,(20.,0.))
        self.assertFalse(self.bridge.visible_approach_active)

    def test_new_hint_cannot_resume_with_stale_navigation_inputs(self):
        message=self.detect();self.bridge.on_visible_approach(message)
        self.bridge.odom.pose.pose.position.x=8.
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        epoch=self.bridge.epoch
        data=json.loads(message.data);data.update(observation_stamp_ns=self.now,point=[16.,0.])
        self.bridge.input_freshness_issue=lambda:'stale_costmap'
        self.bridge.on_visible_approach(String(data=json.dumps(data)))
        self.assertIsNone(self.bridge.expected)
        self.assertEqual(self.bridge.epoch,epoch)
        self.assertEqual(self.bridge.last_status['status'],'visible_target_reobserving')

    def test_established_candidate_expires_or_rejects_large_bearing_jump(self):
        self.detect()
        self.now += 16*10**9
        self.perception.on_visible_target(self.mask())
        self.assertEqual(self.perception.visible_target_count,1)
        self.perception.visible_target_count=3
        self.perception.visible_target['bearing']=math.pi/2
        self.now += 100_000_000
        self.perception.on_visible_target(self.mask())
        self.assertEqual(self.perception.visible_target_count,1)

    def test_memory_route_keeps_ownership_over_unassociated_bearing(self):
        self.bridge.memory_goal = (20., 0.)
        self.bridge.on_visible_approach(self.detect())
        self.assertFalse(self.bridge.visible_approach_active)
        self.bridge.waypoint_pub.publish.assert_not_called()

    def test_query_change_and_stale_frames_cannot_reactivate_old_goal(self):
        old = self.detect()
        self.query('gas station')
        self.bridge.on_visible_approach(old)
        self.assertFalse(self.bridge.visible_approach_active)
        self.now += 3*10**9
        self.perception.on_visible_target(self.mask('green residential house'))
        self.assertIsNone(self.perception.visible_target)

    def test_no_observation_goal_crosses_unknown_or_occupied_cells(self):
        grid = np.zeros((30, 30), dtype=np.int16)
        grid[:, 17:] = -1
        self.assertIsNone(observation_step(grid, 1., (-15., -15.), (0., 0.), 0., 0.))
        grid[:, 17:] = 100
        self.assertIsNone(observation_step(grid, 1., (-15., -15.), (0., 0.), 0., 0.))

    def test_map_rotation_does_not_rotate_the_world_target_bearing(self):
        grid = np.zeros((30, 30), dtype=np.int16)
        point = observation_step(grid, 1., (-15., -15.), (100., 200.), math.pi/2, 0.)
        self.assertEqual(point, (108., 200.))

    def test_unreachable_observation_goal_releases_exploration_with_backoff(self):
        message = self.detect()
        self.bridge.on_visible_approach(message)
        self.bridge.visible_approach_started_at -= 11.
        self.bridge.activate_visible_approach()
        self.assertIsNone(self.bridge.expected)
        self.assertTrue(self.bridge.query_reset_pending)
        self.assertFalse(self.bridge.object_found)
        self.bridge.query_reset_pending = False
        self.bridge.on_visible_approach(message)
        self.assertFalse(self.bridge.visible_approach_active)

    def test_admitted_observation_route_is_not_timed_out_between_segments(self):
        self.bridge.on_visible_approach(self.detect())
        self.bridge.route = Path()
        self.bridge.visible_approach_started_at -= 11.
        self.bridge.following = self.bridge.pending = False
        self.bridge.activate_visible_approach()
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertEqual(self.bridge.expected, (8., 0.))
        self.assertFalse(self.bridge.query_reset_pending)
        # A route never extends the existing whole-approach bound.
        self.bridge.visible_approach_deadline = -1.
        self.bridge.activate_visible_approach()
        self.assertFalse(self.bridge.visible_approach_active)
        self.assertTrue(self.bridge.query_reset_pending)

    def test_expired_observation_does_not_consume_physical_recovery_tick(self):
        self.bridge.on_visible_approach(self.detect())
        self.bridge.visible_approach_deadline = -1.
        self.bridge.execution_recovery_owns_motion = Mock(return_value=True)
        self.assertFalse(self.bridge.check_visible_approach_deadline())
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertFalse(self.bridge.query_reset_pending)
        self.bridge.execution_recovery_owns_motion.return_value = False
        self.assertTrue(self.bridge.check_visible_approach_deadline())
        self.assertFalse(self.bridge.visible_approach_active)

    def test_repeated_or_miscalibrated_frames_do_not_satisfy_three_observations(self):
        message = self.mask()
        for _ in range(5):
            self.perception.on_visible_target(message)
        self.assertEqual(self.perception.visible_target_count, 1)
        self.perception.visible_approach_pub.publish.assert_not_called()
        self.now += 100_000_000
        message = self.mask()
        message.cam_infos[0].header.frame_id = 'wrong_camera'
        self.perception.on_visible_target(message)
        self.assertEqual(self.perception.visible_target_count, 1)
