"""Exercise ROS message_filters with the measured visual inference delay."""
import unittest
from types import SimpleNamespace as NS
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
from builtin_interfaces.msg import Time


def message(seconds):
    return NS(header=NS(stamp=Time(sec=int(seconds), nanosec=round((seconds-int(seconds))*1e9))))


class MaskLidarDelayTests(unittest.TestCase):
    def pair(self, size, mask_stamp=1.):
        mask, lidar = SimpleFilter(), SimpleFilter()
        sync = ApproximateTimeSynchronizer([mask,lidar], size, .2)
        matches=[]
        sync.registerCallback(lambda m,l: matches.append((m,l)))
        for i in range(21):
            lidar.signalMessage(message(i*.2))
        mask.signalMessage(message(mask_stamp))
        return matches

    def test_measured_delay_exhausts_the_old_queue(self):
        self.assertEqual(self.pair(10), [])

    def test_matching_scan_survives_three_second_inference_delay(self):
        matches=self.pair(50)
        self.assertEqual(len(matches),1)
        self.assertEqual(matches[0][0].header.stamp,matches[0][1].header.stamp)

    def test_larger_cache_does_not_match_a_distant_timestamp(self):
        self.assertEqual(self.pair(50,mask_stamp=8.), [])


if __name__=='__main__': unittest.main()
