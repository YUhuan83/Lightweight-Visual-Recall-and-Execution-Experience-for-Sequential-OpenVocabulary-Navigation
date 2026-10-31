import math
import unittest
from wildos_nav2_adapter.execution_recovery import reverse_escape


class ReverseEscapeTests(unittest.TestCase):
    def test_only_reverses_over_recent_driving(self):
        trace = [(i*.3, 0., 0., 90.+i*.1) for i in range(21)]
        path = reverse_escape(trace, (6., 0.), 0., 100., 8.)
        self.assertIsNotNone(path)
        self.assertAlmostEqual(path[0][0], 6.)
        self.assertGreaterEqual(6.-path[-1][0], 2.)
        self.assertLessEqual(6.-path[-1][0], 4.)
        self.assertTrue(all(b[0] < a[0] for a, b in zip(path, path[1:])))

    def test_actual_curved_driving_supports_feasible_reverse_arc(self):
        trace = [(10*math.sin(s/10), 10*(1-math.cos(s/10)), s/10, 95.+s/10)
                 for s in [i*.3 for i in range(21)]]
        path = reverse_escape(trace, trace[-1][:2], .6, 100., 8.)
        self.assertIsNotNone(path)
        self.assertLess(path[-1][2], path[0][2])

    def test_stale_future_or_disconnected_history_cannot_authorize_escape(self):
        trace = [(i*.3, 0., 0., 95.) for i in range(21)]
        self.assertIsNone(reverse_escape(trace, (9., 0.), 0., 100., 8.))
        self.assertIsNone(reverse_escape(trace, (6., 0.), 0., 160., 8.))
        self.assertIsNone(reverse_escape(trace, (6., 0.), 0., 90., 8.))
        self.assertIsNone(reverse_escape(trace, (6., 0.), math.pi, 100., 8.))

    def test_sharp_curve_is_rejected(self):
        trace = [(2*math.sin(s/2), 2*(1-math.cos(s/2)), s/2, 95.+s/10)
                 for s in [i*.3 for i in range(21)]]
        self.assertIsNone(reverse_escape(trace, trace[-1][:2], 3., 100., 8.))


if __name__ == '__main__':
    unittest.main()
