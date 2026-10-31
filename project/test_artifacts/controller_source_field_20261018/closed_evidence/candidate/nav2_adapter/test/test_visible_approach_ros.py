"""Common detection-to-observation contracts, using real adapter callbacks."""
import json
import math
import unittest
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


class VisibleApproachTests(unittest.TestCase):
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
        self.assertEqual(self.bridge.expected, (8., 0.))
        self.assertTrue(self.bridge.visible_approach_active)
        self.assertFalse(self.bridge.query_reset_pending)
        self.assertFalse(self.bridge.object_found)
        self.assertEqual(self.bridge.last_status['status'], 'visible_target_reobserving')

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

    def test_graph_route_cannot_reexecute_completed_point_while_reobserving(self):
        self.bridge.on_visible_approach(self.detect())
        self.now += 3_174_028_397
        self.bridge.await_object_observation()
        self.bridge.route_diagnostic=Mock()
        self.bridge.start_segment=Mock()
        self.bridge.on_route(Path())
        self.bridge.route_diagnostic.assert_called_once_with('visible_target_reobserving')
        self.bridge.start_segment.assert_not_called()

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
        self.assertEqual(self.bridge.expected,(8.,0.))
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
