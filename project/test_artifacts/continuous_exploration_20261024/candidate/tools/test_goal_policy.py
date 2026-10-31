"""Run without ROS: python -m unittest discover -s tools -p test_goal_policy.py -v."""
from dataclasses import replace
import math
import unittest

from goal_policy import Goal, GoalPolicy, materially_changed, validate


def goal(x=0.0, yaw=0.0, stamp=10_000_000_000):
    return Goal('odom', stamp, (x, 0.0, 0.0),
                (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


class ValidationTests(unittest.TestCase):
    def test_valid_and_age_boundary(self):
        self.assertIsNone(validate(goal(), 10_000_000_000))
        self.assertIsNone(validate(goal(), 15_000_000_000))
        self.assertEqual(validate(goal(), 15_000_000_001), 'expired goal')

    def test_clock_and_stamp(self):
        for stamp, now in ((0, 10), (-1, 10), (10, 0), (10, -1), (11, 10)):
            with self.subTest(stamp=stamp, now=now):
                self.assertIsNotNone(validate(goal(stamp=stamp), now))

    def test_exact_configured_frame_no_relabeling(self):
        for frame in ('map', '/odom', '', 'base_link'):
            self.assertIsNotNone(validate(replace(goal(), frame=frame), 10_000_000_000))
        self.assertIsNone(validate(replace(goal(), frame='map'), 10_000_000_000, 'map'))

    def test_all_pose_components_finite(self):
        for value in (math.nan, math.inf, -math.inf):
            for field, size in (('position', 3), ('quaternion', 4)):
                for index in range(size):
                    values = list(getattr(goal(), field))
                    values[index] = value
                    with self.subTest(value=value, field=field, index=index):
                        self.assertIsNotNone(validate(
                            replace(goal(), **{field: tuple(values)}), 10_000_000_000))

    def test_quaternion_norm_and_dimensions(self):
        for q in ((0., 0., 0., 0.), (0., 0., 0., 2.), (1e308,) * 4, (0., 1.)):
            self.assertIsNotNone(validate(replace(goal(), quaternion=q), 10_000_000_000))
        self.assertIsNotNone(validate(replace(goal(), position=(0.,)), 10_000_000_000))
        self.assertIsNone(validate(replace(goal(), quaternion=(0., 0., 0., -1.)), 10_000_000_000))

    def test_distance_and_heading_gates(self):
        self.assertFalse(materially_changed(goal(), goal(x=0.099, yaw=0.099)))
        self.assertTrue(materially_changed(goal(), goal(x=0.1)))
        self.assertTrue(materially_changed(goal(), goal(yaw=0.101)))
        self.assertTrue(materially_changed(goal(), replace(goal(), position=(0., 0., 0.2))))

    def test_angular_wrap_and_quaternion_sign(self):
        self.assertFalse(materially_changed(goal(yaw=math.pi - 0.01), goal(yaw=-math.pi + 0.01)))
        a = goal(yaw=1.2)
        self.assertFalse(materially_changed(a, replace(a, quaternion=tuple(-v for v in a.quaternion))))
        self.assertTrue(materially_changed(goal(yaw=math.pi - 0.2), goal(yaw=-math.pi + 0.2)))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.policy = GoalPolicy()

    def active(self):
        self.policy.offer(goal())
        token, _ = self.policy.begin_send()
        self.assertTrue(self.policy.accepted(token, True))
        return token

    def test_invalid_thresholds(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                GoalPolicy(distance=value)
            with self.assertRaises(ValueError):
                GoalPolicy(heading=value)

    def test_latest_only_while_server_unavailable(self):
        for x in range(1000):
            self.policy.offer(goal(x=x))
        token, selected = self.policy.begin_send()
        self.assertEqual(token, 1)
        self.assertEqual(selected.position[0], 999)
        self.assertIsNone(self.policy.pending)

    def test_send_accept_cancel_terminal_serialization(self):
        self.policy.offer(goal())
        first, _ = self.policy.begin_send()
        self.policy.offer(goal(x=1))
        self.assertIsNone(self.policy.begin_send())
        self.assertIsNone(self.policy.begin_cancel())
        self.policy.accepted(first, True)
        self.assertEqual(self.policy.begin_cancel(), first)
        for x in range(2, 100):
            self.policy.offer(goal(x=x))
            self.assertIsNone(self.policy.begin_cancel())
            self.assertIsNone(self.policy.begin_send())
        self.assertEqual(self.policy.state, 'canceling')
        self.policy.finish(first)
        second, selected = self.policy.begin_send()
        self.assertGreater(second, first)
        self.assertEqual(selected.position[0], 99)
        self.assertFalse(self.policy.finish(first))
        self.assertFalse(self.policy.accepted(first, True))
        self.policy.fault(first)
        self.assertEqual(self.policy.state, 'sending')

    def test_rejection_releases_slot_without_retry(self):
        self.policy.offer(goal())
        token, _ = self.policy.begin_send()
        self.policy.accepted(token, False)
        self.assertEqual(self.policy.state, 'idle')
        self.assertFalse(self.policy.offer(goal(stamp=11_000_000_000)))
        self.assertIsNone(self.policy.begin_send())
        self.policy.offer(goal(x=1, stamp=12_000_000_000))
        self.assertIsNotNone(self.policy.begin_send())

    def test_rejected_with_pending_sends_only_latest(self):
        self.policy.offer(goal())
        token, _ = self.policy.begin_send()
        self.policy.offer(goal(x=1))
        self.policy.offer(goal(x=2))
        self.policy.accepted(token, False)
        self.assertEqual(self.policy.begin_send()[1].position[0], 2)

    def test_duplicates_do_not_cancel_or_resend_after_terminal(self):
        token = self.active()
        for i in range(1000):
            self.assertFalse(self.policy.offer(goal(x=0.01, stamp=10_000_000_000 + i)))
            self.assertIsNone(self.policy.begin_cancel())
        self.policy.finish(token)
        self.assertFalse(self.policy.offer(goal(stamp=11_000_000_000)))
        self.assertIsNone(self.policy.begin_send())

    def test_cumulative_drift_compares_to_dispatched_goal(self):
        self.active()
        for x in (0.02, 0.04, 0.06, 0.08):
            self.assertFalse(self.policy.offer(goal(x=x)))
        self.assertTrue(self.policy.offer(goal(x=0.12)))
        self.assertIsNotNone(self.policy.begin_cancel())

    def test_return_to_current_clears_detour_before_cancel(self):
        self.active()
        self.policy.offer(goal(x=1))
        self.policy.offer(goal())
        self.assertIsNone(self.policy.pending)
        self.assertIsNone(self.policy.begin_cancel())

    def test_return_to_current_after_cancel_must_resend(self):
        token = self.active()
        self.policy.offer(goal(x=1))
        self.policy.begin_cancel()
        self.policy.offer(goal())
        self.assertIsNotNone(self.policy.pending)
        self.policy.finish(token)
        self.assertEqual(self.policy.begin_send()[1].position[0], 0)

    def test_older_stamp_cannot_replace_latest(self):
        self.policy.offer(goal(x=1, stamp=20))
        self.assertFalse(self.policy.offer(goal(x=2, stamp=19)))
        self.assertEqual(self.policy.pending.position[0], 1)
        self.assertTrue(self.policy.offer(goal(x=3, stamp=20)))
        self.assertEqual(self.policy.pending.position[0], 3)

    def test_expiry_before_dispatch_or_cancel(self):
        self.active()
        self.policy.offer(goal(x=1))
        if validate(self.policy.pending, 16_000_000_000):
            self.policy.pending = None
        self.assertIsNone(self.policy.begin_cancel())
        self.assertIsNone(self.policy.begin_send())

    def test_transport_fault_blocks_until_confirmed_result(self):
        token = self.active()
        self.policy.fault(token)
        self.policy.offer(goal(x=1))
        self.assertIsNone(self.policy.begin_send())
        self.assertIsNone(self.policy.begin_cancel())
        self.assertTrue(self.policy.finish(token))
        self.assertIsNotNone(self.policy.begin_send())

    def test_shutdown_during_send_waits_for_handle_then_cancels(self):
        self.policy.offer(goal())
        token, _ = self.policy.begin_send()
        self.policy.offer(goal(x=1))
        self.policy.stop()
        self.assertIsNone(self.policy.pending)
        self.assertFalse(self.policy.offer(goal(x=2)))
        self.assertIsNone(self.policy.begin_cancel())
        self.policy.accepted(token, True)
        self.assertEqual(self.policy.begin_cancel(), token)
        self.assertIsNone(self.policy.begin_cancel())
        self.policy.finish(token)
        self.assertIsNone(self.policy.begin_send())


if __name__ == '__main__':
    unittest.main()
