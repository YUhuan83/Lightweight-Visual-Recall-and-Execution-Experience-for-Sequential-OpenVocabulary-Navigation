import unittest
from check_continuous_start_execution import settle

class SettleTests(unittest.TestCase):
    def run_stop(self,speeds,timeout=5.):
        now=[0.];iterator=iter(speeds);last=[speeds[-1]]
        def read():
            last[0]=next(iterator,last[0]);return {'speed':last[0]}
        def spin(seconds):now[0]+=seconds
        return settle(read,spin,lambda:now[0],timeout=timeout)

    def test_transient_and_new_motion_reset_stable_window(self):
        after,samples=self.run_stop([.056,.036,.03,.03,.07,.02,.02,.02,.02,.02,.02,.02])
        self.assertLess(after['speed'],.05)
        self.assertGreaterEqual(samples[-1]['elapsed'],1.)

    def test_exact_threshold_or_continuing_motion_cannot_pass(self):
        for speed in (.05,.056):
            with self.subTest(speed=speed),self.assertRaises(RuntimeError):
                self.run_stop([speed],timeout=.8)

    def test_invalid_telemetry_fails_closed(self):
        for speed in (float('nan'),float('inf'),-1.):
            with self.subTest(speed=speed),self.assertRaises(RuntimeError):self.run_stop([speed])

if __name__=='__main__':unittest.main()
