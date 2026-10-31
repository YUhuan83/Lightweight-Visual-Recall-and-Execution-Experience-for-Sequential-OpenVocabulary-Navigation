"""Direct WildOS goal updates and odometry distance completion, with car geometry."""
import json
from copy import deepcopy
import math
import time
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
import rclpy
from std_msgs.msg import String
import test_body_arrival_ros as body
from wildos_nav2_adapter.arrival_criteria import matched_arrival_evidence


class UpstreamTargetApproachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'object_arrival_distance:=3.0'])

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.fixture = body.BodyArrivalTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.b = self.fixture.bridge
        self.b.upstream_target_approach = True
        self.b.expected = None
        self.b.query_reset_pending = False
        self.b.waypoint_pub = Mock()

    def goal(self, x=20., y=0., z=0.):
        p = PoseStamped()
        p.header.frame_id = 'spot1/odom'
        p.header.stamp = self.b.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation.w = 1.
        return p

    def move(self, x, y=0.):
        self.fixture.fixture.now += 100_000_000
        msg = deepcopy(self.b.odom)
        msg.header.stamp = self.b.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        self.b.on_odom(msg)

    def test_raw_target_is_forwarded_without_standoff_or_confirmation(self):
        self.b.on_waypoint(self.goal())
        self.assertEqual(self.b.expected, (20., 0.))
        sent = self.b.waypoint_pub.publish.call_args.args[0]
        self.assertEqual(sent.pose.position.z, 0.)
        self.assertFalse(self.b.object_found)
        self.b.route_confirmed_target.assert_not_called()

    def test_updated_raw_target_keeps_the_current_follow_action_running(self):
        self.b.on_waypoint(self.goal())
        handle = Mock()
        self.b.following, self.b.follow_handle = True, handle
        epoch = self.b.epoch
        self.b.on_waypoint(self.goal(24.))
        self.assertEqual(self.b.expected, (24., 0.))
        self.assertEqual(self.b.epoch, epoch)
        self.assertTrue(self.b.following)
        handle.cancel_goal_async.assert_not_called()

    def test_odometry_can_finish_without_a_second_confirmation_message(self):
        self.b.on_waypoint(self.goal())
        self.move(17.0)
        self.assertFalse(self.b.object_found)
        self.move(17.001)
        self.assertTrue(self.b.object_found)
        self.assertEqual(self.b.last_status['status'], 'object_found')
        self.assertEqual(self.b.last_status['object_position'], [20., 0., 0.])
        self.assertAlmostEqual(self.b.last_status['object_distance_m'], 2.999)

    def test_localized_target_survives_a_gap_in_visual_messages(self):
        self.b.on_waypoint(self.goal())
        self.fixture.fixture.now += 6_000_000_000
        self.move(17.001)
        self.assertTrue(self.b.object_found)

    def test_confirmation_payload_cannot_replace_the_navigation_goal(self):
        self.b.on_waypoint(self.goal())
        self.b.on_object_confirmation(self.fixture.confirmation(point=(40., 0., 2.), standoff=[8., 0.]))
        self.assertEqual(self.b.expected, (20., 0.))
        self.assertFalse(self.b.object_found)

    def test_nearby_confirmation_cannot_finish_a_distant_raw_goal(self):
        self.b.on_waypoint(self.goal())
        self.b.on_object_confirmation(self.fixture.confirmation())
        self.assertFalse(self.b.object_found)

    def test_query_change_clears_the_old_target_before_vehicle_moves(self):
        self.b.on_waypoint(self.goal())
        self.b.on_query(String(data='gas station'))
        self.move(17.001)
        self.assertFalse(self.b.object_found)
        self.assertIsNone(self.b.object_destination)

    def test_missing_3d_target_does_not_turn_a_memory_viewpoint_into_arrival(self):
        self.b.memory_goal = self.b.expected = (0., 0.)
        self.move(0.)
        self.assertFalse(self.b.object_found)

    def test_live_target_hands_off_from_selected_memory_viewpoint(self):
        self.b.memory_goal = self.b.expected = (3., 0.)
        self.b.memory_active = dict(id='view-A')
        self.b.on_waypoint(self.goal())
        self.assertIsNone(self.b.memory_goal)
        self.assertEqual(self.b.expected, (20., 0.))
        self.assertEqual(self.b.memory_active['id'], 'view-A')

    def test_memory_candidate_comparison_still_owns_its_selection(self):
        self.b.memory_selecting = True
        self.b.on_waypoint(self.goal())
        self.assertIsNone(self.b.expected)

    def test_stale_pose_cannot_complete_when_a_close_target_arrives(self):
        self.b.odom_received_at = time.monotonic() - 2.
        self.b.on_waypoint(self.goal(2.999))
        self.assertFalse(self.b.object_found)

    def test_wrong_frame_and_previous_query_goal_are_rejected(self):
        p = self.goal()
        p.header.frame_id = 'map'
        self.b.on_waypoint(p)
        self.assertIsNone(self.b.expected)
        p = self.goal()
        p.header.stamp.sec -= 1
        self.b.on_waypoint(p)
        self.assertIsNone(self.b.expected)

    def test_orientation_does_not_change_euclidean_arrival(self):
        self.b.odom.pose.pose.orientation.z = math.sin(math.pi / 4)
        self.b.odom.pose.pose.orientation.w = math.cos(math.pi / 4)
        self.b.on_waypoint(self.goal(0., 2.999))
        self.assertTrue(self.b.object_found)

    def test_legacy_mode_retains_its_existing_confirmation_contract(self):
        self.b.upstream_target_approach = False
        self.b.on_object_confirmation(self.fixture.confirmation(point=(20., 0., 2.), standoff=[8., 0.]))
        self.b.route_confirmed_target.assert_called_once()

    def test_success_is_terminal_until_the_next_query(self):
        self.b.on_waypoint(self.goal(2.999))
        self.assertTrue(self.b.object_found)
        self.b.on_waypoint(self.goal(40.))
        self.assertEqual(self.b.expected, (2.999, 0.))

    def test_vertical_gap_is_part_of_arrival(self):
        self.b.on_waypoint(self.goal(0., 0., 3.001))
        self.assertFalse(self.b.object_found)

    def test_diagonal_three_dimensional_distance_is_used(self):
        self.b.on_waypoint(self.goal(2., 0., 2.3))
        self.assertFalse(self.b.object_found)
        self.b.on_waypoint(self.goal(2., 0., 2.1))
        self.assertTrue(self.b.object_found)
        self.assertAlmostEqual(self.b.last_status['object_distance_m'], math.sqrt(8.41))

    def test_real_arrival_event_is_accepted_only_with_independent_image_evidence(self):
        self.b.on_waypoint(self.goal())
        self.move(17.001)
        status = self.b.last_status
        confirmation = json.loads(self.fixture.confirmation(point=(20., 0., 0.)).data)
        now = time.monotonic()
        stamp = self.fixture.fixture.now
        statuses = [dict(status=status, received_at=now)]
        masks = [dict(query='house', count=3, pixels=100, at=now,
                      observation_stamp_ns=stamp)]
        self.assertIsNone(matched_arrival_evidence(statuses, [], masks, 'house',
                                                  self.b.query_started_ns, stamp, now, 3))
        confirmations = [dict(confirmation, received_at=now)]
        item = matched_arrival_evidence(statuses, confirmations, masks, 'house',
                                       self.b.query_started_ns, stamp, now, 3)
        self.assertIsNotNone(item)
        confirmations[0]['object_position'] = [40., 0., 8.]
        self.assertIsNone(matched_arrival_evidence(statuses, confirmations, masks, 'house',
                                                  self.b.query_started_ns, stamp, now, 3))


