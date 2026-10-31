"""Cross-check the offline CDR reader against independently decoded ROS evidence."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
from offline_interface_replay import CDR, audit

ROOT = Path(__file__).resolve().parents[1]


class RecordedInterfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        database = ROOT/'test_artifacts/server_validation_20261006/green_house_station_arrival_20261006_idealpoint_syncfix1/rosbag/rosbag_0.db3'
        reference = ROOT/'test_artifacts/server_validation_20261006/analyze_town_bag_v2.json'
        if not database.is_file() or not reference.is_file():
            raise unittest.SkipTest('Recorded local evidence unavailable; no simulator rerun is implied')
        cls.result = audit(database)
        cls.reference = json.loads(reference.read_text(encoding='utf8'))['odom']

    def test_reader_matches_independent_ros_decoder_on_pose_and_time(self):
        self.assertEqual(self.result['odom_count'], self.reference['n'])
        self.assertAlmostEqual(self.result['travel_m'], self.reference['travel'], places=6)
        for name in ('first','last'):
            for field in ('x','y','yaw','vx','wz'):
                self.assertAlmostEqual(self.result[name][field],self.reference[name][field],places=7)
            self.assertEqual(self.result[name]['receipt_ns'],self.reference[name]['recv'])

    def test_steady_turns_show_inverse_gain_and_reference_point_drift(self):
        row = next(value for value in self.result['angular_lag_scan_one_second_constant_command']
                   if value['time_domain']=='receipt_ns' and value['lag_sec']==0.)
        self.assertGreater(row['positive_commands'],100)
        self.assertGreater(row['negative_commands'],100)
        self.assertLess(row['same_sign'],.05)
        self.assertAlmostEqual(row['gain_median'],-1.,delta=.02)
        # The recorded lateral displacement is consistent with twice the
        # 2.172 m reference-point offset under the opposite rotation.
        self.assertAlmostEqual(row['lateral_speed_per_commanded_yaw_median'],4.344,delta=.05)
        self.assertEqual(self.result['follow_failure_reasons'],
                         dict(execution_stalled=32,tracking_deviation=17))


class UnsupportedEncodingTests(unittest.TestCase):
    def test_reader_rejects_other_encodings_instead_of_guessing(self):
        with self.assertRaises(ValueError):
            CDR(b'\x00\x00\x00\x00')


if __name__ == '__main__':
    unittest.main()
