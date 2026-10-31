import unittest
from wildos_nav2_adapter.test_progress import ManeuverProgress


class ProgressTests(unittest.TestCase):
    def test_real_shunt_counts_but_repeated_oscillation_does_not(self):
        progress = ManeuverProgress()
        status = dict(status='following', goal=[-4., 0.])
        self.assertFalse(progress.observe([0., 0.], status))
        self.assertTrue(progress.observe([-1., 0.], status))
        self.assertFalse(progress.observe([0., 0.], status))
        self.assertFalse(progress.observe([-1., 0.], status))
        self.assertTrue(progress.observe([-2., 0.], status))

    def test_new_goals_and_stationary_status_changes_are_not_progress(self):
        progress = ManeuverProgress()
        for i in range(20):
            self.assertFalse(progress.observe([0., 0.], dict(status='following', goal=[i, 0.])))
            self.assertFalse(progress.observe([0., 0.], dict(status='maneuver_done', goal=[i, 0.])))
