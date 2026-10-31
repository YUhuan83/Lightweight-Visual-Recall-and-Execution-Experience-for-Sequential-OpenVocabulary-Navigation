"""Offline tests execute GoalStandoff's real query and consensus callbacks."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from unittest.mock import patch

from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from copy import deepcopy
from beamng_wildos_adapter.goal_standoff import GoalStandoff

ROOT = Path(__file__).resolve().parents[2]


class GoalConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.node = GoalStandoff()
        self.node.publisher = Mock()
        self.node.confirmation_pub = Mock()
        self.now = 100 * 10**9
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=self.now, to_msg=lambda: Time(sec=self.now//10**9, nanosec=self.now%10**9)))
        self.update_odom()
        self.node.on_query(String(data='house'))

    def update_odom(self):
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(self.now, 10**9)
        odom.pose.pose.orientation.w = 1.
        self.node.on_odom(odom)

    def tearDown(self):
        self.node.destroy_node()

    def observe(self, point=(10., 0., 2.), *, stamp=None, query='house', **changes):
        if stamp is None:
            self.now += 10**9
            stamp = self.now
        data = dict(query=query, frame='spot1/odom', observation_stamp_ns=stamp,
            position=list(point), observer_position=[0., 0., 0.],
            observer_orientation=[0., 0., 0., 1.], source='multiview', views=6,
            localization_valid=True, quality={'confidence': .65, 'association_validated': True})
        data.update(changes)
        self.update_odom()
        self.node.on_localization(String(data=json.dumps(data)))

    def test_recorded_house_consensus_accepts_four_agreeing_positions(self):
        fixture = json.loads((ROOT/'test_artifacts/server_validation_20261002/house_localization_replay.json').read_text())
        times = fixture['times']
        for point, stamp in zip(fixture['raw_positions'][:5], times[:5]):
            self.now = 100*10**9 + int((stamp-times[0]+1)*10**9)
            self.observe(point, stamp=self.now)
        self.node.confirmation_pub.publish.assert_called_once()
        value = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(value['inliers'], 4)
        self.assertEqual(value['observations'], 5)
        self.assertLessEqual(value['radius'], 1.5)
        self.assertTrue(value['search_confirmed'])
        self.assertEqual(value['query'], 'house')

    def test_confirmed_object_within_five_metres_does_not_require_a_route_pose(self):
        self.node.publish_standoff=Mock(return_value='no_reachable_standoff')
        for _ in range(5):self.observe((4.,0.,2.))
        value=json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(value['search_confirmed'])
        self.assertFalse(value['approach_goal_ready'])
        self.assertIsNone(value['standoff'])

    def test_confirmed_object_beyond_five_metres_still_requires_approach(self):
        self.node.publish_standoff=Mock(return_value='no_reachable_standoff')
        for _ in range(5):self.observe((5.01,0.,2.))
        value=json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertFalse(value['search_confirmed'])

    def test_configured_window_accepts_four_spaced_stable_positions(self):
        self.node.confirmation_window_sec = 65.
        for index in range(4):
            self.now = 100 * 10**9 + index * 20 * 10**9
            self.observe((10., 0., 2.), stamp=self.now)
        self.node.confirmation_pub.publish.assert_called_once()
        value = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertEqual(value['inliers'], 4)
        self.assertLessEqual(value['radius'], 1.5)
        self.assertTrue(value['search_confirmed'])

    def test_traffic_light_standoff_preserves_sensor_clearance(self):
        self.node.on_query(String(data='traffic light'))
        for _ in range(5):
            self.observe((20., 0., 6.),query='traffic light')
        self.assertAlmostEqual(self.node.last_standoff_object_distance, 4.351995092, places=6)

    def test_short_standoff_does_not_place_car_front_inside_object(self):
        original = self.node.get_parameter
        self.node.get_parameter = lambda name: (SimpleNamespace(value=3.)
            if name == 'standoff' else original(name))
        target = PoseStamped()
        target.pose.position.x = 20.
        self.assertIsNone(self.node.publish_standoff(target))
        self.assertAlmostEqual(self.node.last_standoff_object_distance, 4.351995092, places=6)
        self.assertTrue(self.node.last_standoff_clamped)
        self.assertGreaterEqual(self.node.last_standoff_object_distance,
            original('vehicle_front_extent').value + original('target_front_clearance').value)

    def test_update_deadband_cannot_retain_endpoint_inside_front_clearance(self):
        original = self.node.get_parameter
        self.node.get_parameter = lambda name: (SimpleNamespace(value=3.)
            if name == 'standoff' else original(name))
        self.node.last_goal = (16.8, 0.)
        target = PoseStamped()
        target.pose.position.x = 20.
        self.assertIsNone(self.node.publish_standoff(target))
        self.assertAlmostEqual(self.node.last_standoff_object_distance, 4.351995092, places=6)
        self.node.publisher.publish.assert_called_once()

    def test_missing_pose_is_not_search_success_and_retries_when_pose_recovers(self):
        self.node.odom = None
        with patch.object(self.node, 'on_odom'):
            for _ in range(5):
                self.observe((20., 0., 2.))
        data = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(data['position_confirmed'])
        self.assertFalse(data['search_confirmed'])
        self.assertFalse(data['approach_goal_ready'])
        self.assertEqual(data['approach_failure_reason'], 'missing_vehicle_pose')
        self.node.publisher.publish.assert_not_called()
        self.update_odom()
        data = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(data['search_confirmed'])
        self.assertEqual(data['standoff'], [10., 0.])
        self.assertAlmostEqual(data['standoff_object_distance_m'], 10.)
        self.assertFalse(data['standoff_clamped'])
        self.node.publisher.publish.assert_called_once()

    def test_confirmed_target_approaches_unknown_to_one_grid_cell_buffer(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(self.now, 10**9)
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .5
        grid.info.origin.position.x = grid.info.origin.position.y = -20.
        grid.info.origin.orientation.w = 1.
        grid.data = [-1] * (80 * 80)
        # Free observed floor to 7.5 m; the next grid cell is unknown.
        row = 40
        for column in range(40, 56):
            grid.data[row * 80 + column] = 0
        self.node.occupancy = grid
        self.node.occupancy_odom = self.node.odom

        target = PoseStamped()
        target.pose.position.x = 20.
        target.pose.position.z = 2.
        self.assertIsNone(self.node.publish_standoff(target))
        self.assertAlmostEqual(self.node.last_goal[0], 7.5)
        self.assertAlmostEqual(self.node.last_goal[1], 0.)
        self.assertTrue(self.node.last_standoff_clamped)
        self.assertAlmostEqual(self.node.last_standoff_object_distance, 12.5)

    def test_clamped_target_refreshes_next_standoff_after_vehicle_reaches_it(self):
        target = PoseStamped()
        target.header.frame_id = 'spot1/odom'
        target.pose.position.x = 20.
        target.pose.position.z = 2.
        confirmation = dict(query='house', observation_stamp_ns=self.now,
            object_position=[20., 0., 2.], observer_position=[0., 0., 0.],
            observer_orientation=[0., 0., 0., 1.], radius=.5, inliers=4,
            observations=5, source='multiview')
        self.node.active_approach = (target, confirmation)
        self.node.last_goal = (7.5, 0.)
        self.node.last_standoff_clamped = True
        self.node.last_standoff_object_distance = 12.5
        self.node.odom.pose.pose.position.x = 0.
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(self.now, 10**9)
        self.node.occupancy = grid
        self.node.occupancy_odom = self.node.odom

        with patch.object(self.node, 'publish_standoff') as publish:
            self.node.refresh_active_approach()
            publish.assert_not_called()

            self.node.odom.pose.pose.position.x = 7.5
            def advance(_target):
                self.node.last_goal = (9., 0.)
                self.node.last_standoff_clamped = True
                self.node.last_standoff_object_distance = 11.
                return None
            publish.side_effect = advance
            self.node.refresh_active_approach()

        publish.assert_called_once_with(target)
        self.node.confirmation_pub.publish.assert_called_once()
        refresh = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(refresh['approach_refresh'])
        self.assertEqual(refresh['base_observation_stamp_ns'], self.now)
        self.assertEqual(refresh['standoff'], [9., 0.])
        self.assertAlmostEqual(refresh['standoff_object_distance_m'], 11.)

        self.node.on_query(String(data='gas station'))
        self.assertIsNone(self.node.active_approach)

    def test_expired_or_previous_query_pending_confirmation_cannot_retry(self):
        self.node.odom = None
        with patch.object(self.node, 'on_odom'):
            for _ in range(5):
                self.observe((20., 0., 2.))
        self.now += 6 * 10**9
        self.update_odom()
        self.node.publisher.publish.assert_not_called()
        self.assertIsNone(self.node.pending_confirmation)
        self.node.on_query(String(data='gas station'))
        self.update_odom()
        self.node.publisher.publish.assert_not_called()

    def test_small_clock_skew_is_accepted_but_future_data_is_rejected(self):
        old = self.node.odom
        newer = deepcopy(old)
        newer.header.stamp.nanosec = 20_000_000
        self.node.on_odom(newer)
        self.assertIs(self.node.odom, newer)
        bad = deepcopy(old)
        bad.header.stamp.nanosec = 200_000_000
        self.node.on_odom(bad)
        self.assertIs(self.node.odom, newer)

    def test_fully_blocked_standoff_is_not_reported_as_search_success(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp.sec = 100
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .5
        grid.info.origin.position.x = grid.info.origin.position.y = -20.
        grid.info.origin.orientation.w = 1.
        grid.data = [100] * 6400
        for _ in range(5):
            self.now += 100_000_000
            self.update_odom()
            grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(self.now, 10**9)
            self.node.on_occupancy(deepcopy(grid))
            self.observe((20., 0., 2.), stamp=self.now)
        data = json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertFalse(data['search_confirmed'])
        self.assertEqual(data['approach_failure_reason'], 'no_reachable_standoff')
        self.node.publisher.publish.assert_not_called()

    def test_divergent_positions_cannot_confirm(self):
        for i in range(8):
            self.observe((i*10., 0., 2.))
        self.node.confirmation_pub.publish.assert_not_called()

    def test_duplicate_observation_does_not_satisfy_frame_threshold(self):
        self.now += 10**9
        for _ in range(8):
            self.observe(stamp=self.now)
        self.assertEqual(len(self.node.observations), 1)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_late_previous_query_and_out_of_order_messages_are_rejected(self):
        self.observe()
        self.observe(query='gas station')
        self.observe(stamp=99*10**9)
        self.assertEqual(len(self.node.observations), 1)
        self.node.on_query(String(data='gas station'))
        self.assertFalse(self.node.observations)
        self.observe(query='house')
        self.node.confirmation_pub.publish.assert_not_called()

    def test_old_window_cannot_be_reused_for_new_single_observation(self):
        for _ in range(5):
            self.observe()
        self.node.confirmation_pub.reset_mock()
        self.now += 35*10**9
        self.observe()
        self.node.confirmation_pub.publish.assert_not_called()

    def test_nonfinite_and_invalid_pose_is_rejected(self):
        self.observe((float('nan'), 0., 0.))
        self.observe(observer_orientation=[0., 0., 0., 0.])
        self.observe(stamp=self.now+10**9)
        self.assertFalse(self.node.observations)

    def test_malformed_quality_and_tilted_grid_are_rejected(self):
        for quality in ('invalid', [], {'confidence': float('nan')}, {'confidence': .1}):
            self.observe(quality=quality)
        self.assertFalse(self.node.observations)
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(self.now, 10**9)
        grid.info.width = grid.info.height = 10
        grid.info.resolution = .5
        grid.data = [0] * 100
        grid.info.origin.orientation.x = math.sin(.2)
        grid.info.origin.orientation.w = math.cos(.2)
        self.node.on_occupancy(grid)
        self.assertIsNone(self.node.occupancy)

    def test_new_fresh_confirmation_is_published_even_when_standoff_unchanged(self):
        for _ in range(6):
            self.observe()
        self.assertEqual(self.node.publisher.publish.call_count, 1)
        published = [json.loads(call.args[0].data)
                     for call in self.node.confirmation_pub.publish.call_args_list]
        stamps = [item['observation_stamp_ns'] for item in published]
        self.assertGreaterEqual(len(stamps), 2)
        self.assertEqual(stamps, sorted(set(stamps)))
        self.assertEqual(stamps[-1], self.now)

    def test_local_grid_uses_the_pose_at_its_capture_time(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'spot1/base_link'
        grid.header.stamp.sec = 100
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .5
        grid.info.origin.position.x = grid.info.origin.position.y = -20.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 6400
        captured = self.node.odom
        self.node.on_occupancy(grid)
        self.now += 100000000
        moved = deepcopy(captured)
        moved.header.stamp.nanosec = 100000000
        moved.pose.pose.position.x = 2.
        self.node.on_odom(moved)
        self.assertIs(self.node.occupancy_odom, captured)
        self.assertEqual(self.node.odom.pose.pose.position.x, 2.)

    def test_malformed_stale_grid_and_duplicate_odom_are_rejected(self):
        old = self.node.odom
        self.node.on_odom(deepcopy(old))
        self.assertIs(self.node.odom, old)
        bad = OccupancyGrid()
        bad.header.frame_id = 'spot1/base_link'
        bad.header.stamp.sec = 100
        bad.info.width = bad.info.height = 10
        bad.info.resolution = .5
        bad.info.origin.orientation.w = 1.
        bad.data = [0] * 99
        self.node.on_occupancy(bad)
        self.assertIsNone(self.node.occupancy)


if __name__ == '__main__':
    unittest.main()

