"""Exercise ROS message_filters with the measured visual inference delay."""
import unittest
from types import SimpleNamespace as NS
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
from builtin_interfaces.msg import Time


def message(seconds):
    return NS(header=NS(stamp=Time(sec=int(seconds), nanosec=round((seconds-int(seconds))*1e9))))


class MaskLidarDelayTests(unittest.TestCase):
    def pair(self, size, mask_stamp=4.4):
        mask, lidar = SimpleFilter(), SimpleFilter()
        sync = ApproximateTimeSynchronizer([mask,lidar], size, .2)
        matches=[]
        sync.registerCallback(lambda m,l: matches.append((m,l)))
        # 20 Hz LiDAR, 8 seconds of scans; the mask arrives 3.6 seconds after
        # its acquisition stamp, matching the slow end of measured inference.
        for i in range(161):
            lidar.signalMessage(message(i*.05))
        mask.signalMessage(message(mask_stamp))
        return matches

    def test_fifty_scan_queue_loses_match_at_twenty_hz(self):
        self.assertEqual(self.pair(50), [])

    def test_hundred_scan_queue_retains_match_through_inference_delay(self):
        matches=self.pair(100)
        self.assertEqual(len(matches),1)
        self.assertEqual(matches[0][0].header.stamp,matches[0][1].header.stamp)

    def test_larger_cache_does_not_match_a_distant_timestamp(self):
        self.assertEqual(self.pair(100,mask_stamp=8.4), [])


if __name__=='__main__': unittest.main()
