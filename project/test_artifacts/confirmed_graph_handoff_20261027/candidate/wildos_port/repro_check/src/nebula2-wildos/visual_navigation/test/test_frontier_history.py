import unittest

from visual_navigation.wildos.frontier_history import RecentFrontierHistory


class RecentFrontierHistoryTests(unittest.TestCase):
    def test_expiry_and_capacity_restore_frontier_eligibility(self):
        history = RecentFrontierHistory(ttl_seconds=10, max_entries=2)
        history.add((0, 0), now=0)
        self.assertTrue(history.nearby((0.1, 0), radius=0.5, now=1))
        history.add((10, 0), now=2)
        history.add((20, 0), now=3)
        self.assertFalse(history.nearby((0, 0), radius=0.5, now=3))
        self.assertTrue(history.nearby((10, 0), radius=0.5, now=3))
        self.assertFalse(history.nearby((10, 0), radius=0.5, now=13))


if __name__ == "__main__":
    unittest.main()
