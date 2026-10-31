import importlib.util
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "planning", HERE / "wildos_nav2_adapter" / "planning.py")
PLANNING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNING)


class PlanningTests(unittest.TestCase):
    def test_first_goal_always_plans(self):
        self.assertTrue(PLANNING.should_replan(None, (1.0, 2.0), 1.0))

    def test_small_move_is_ignored(self):
        self.assertFalse(PLANNING.should_replan((0.0, 0.0), (0.4, 0.0), 1.0))

    def test_large_move_replans(self):
        self.assertTrue(PLANNING.should_replan((0.0, 0.0), (1.5, 0.0), 1.0))

    def test_missing_goal_is_not_planned(self):
        self.assertFalse(PLANNING.should_replan((0.0, 0.0), None, 1.0))

    def test_heading_change_wraps_at_pi(self):
        self.assertFalse(PLANNING.heading_changed(3.1, -3.1, 0.2))
        self.assertTrue(PLANNING.heading_changed(0.0, 0.6, 0.35))

    def test_failed_follow_is_retried_until_limit(self):
        self.assertTrue(PLANNING.should_retry_follow(
            status=6, retry_count=0, max_retries=2,
            succeeded_status=4, canceled_status=5))

    def test_successful_or_canceled_follow_is_not_retried(self):
        for status in (4, 5):
            with self.subTest(status=status):
                self.assertFalse(PLANNING.should_retry_follow(
                    status=status, retry_count=0, max_retries=2,
                    succeeded_status=4, canceled_status=5))

    def test_failed_follow_stops_after_retry_limit(self):
        self.assertFalse(PLANNING.should_retry_follow(
            status=6, retry_count=2, max_retries=2,
            succeeded_status=4, canceled_status=5))

    def test_rejected_follow_action_is_retryable(self):
        self.assertTrue(PLANNING.should_retry_follow(
            status=None, retry_count=0, max_retries=2,
            succeeded_status=4, canceled_status=5))

    def test_direction_change_starts_pending_and_brakes(self):
        result = PLANNING.update_direction_shift(
            current_gear=2, desired_gear=-1, pending_gear=None,
            pending_since=None, speed=0.0, now=10.0, shift_speed=0.15,
            confirm_seconds=2.0)
        self.assertEqual(result, (2, -1, 10.0, True))

    def test_direction_flicker_back_cancels_pending_without_braking(self):
        result = PLANNING.update_direction_shift(
            current_gear=2, desired_gear=2, pending_gear=-1,
            pending_since=10.0, speed=0.0, now=10.5, shift_speed=0.15,
            confirm_seconds=2.0)
        self.assertEqual(result, (2, None, None, False))

    def test_sustained_direction_change_waits_until_slow_enough(self):
        result = PLANNING.update_direction_shift(
            current_gear=2, desired_gear=-1, pending_gear=-1,
            pending_since=10.0, speed=0.3, now=12.5, shift_speed=0.15,
            confirm_seconds=2.0)
        self.assertEqual(result, (2, -1, 10.0, True))

    def test_direction_changes_after_dwell_and_low_speed(self):
        result = PLANNING.update_direction_shift(
            current_gear=2, desired_gear=-1, pending_gear=-1,
            pending_since=10.0, speed=0.1, now=12.5, shift_speed=0.15,
            confirm_seconds=2.0)
        self.assertEqual(result, (-1, None, None, False))

    def test_direction_change_restarts_confirmation_for_new_request(self):
        result = PLANNING.update_direction_shift(
            current_gear=2, desired_gear=2, pending_gear=-1,
            pending_since=10.0, speed=0.0, now=12.5, shift_speed=0.15,
            confirm_seconds=2.0)
        self.assertEqual(result, (2, None, None, False))


if __name__ == "__main__":
    unittest.main()
