import unittest
import numpy as np
from american_road_smoke import lidar_quality, imu_records, valid_imu_record


class QualityTests(unittest.TestCase):
    def test_world_range(self):
        self.assertTrue(lidar_quality([[3701,2400,-420]], [3700,2400,-420], [3700,2400,-420])['quality_pass'])
        self.assertFalse(lidar_quality([[-364434685952,0,0]], [3700,2400,-420], [3700,2400,-420])['quality_pass'])

    def test_invalid_and_motion(self):
        for cloud in (np.empty((0,3)), [[np.nan,0,0]]):
            self.assertFalse(lidar_quality(cloud,[0,0,0],[0,0,0])['quality_pass'])
        self.assertFalse(lidar_quality([[1,0,0]],[0,0,0],[1,0,0])['quality_pass'])

    def test_imu(self):
        raw = [{'time':0, 'accRaw':[0,0,9.81], 'angVel':[0,0,0], 'pos':[100,200,300]}]
        rows = imu_records(raw)
        self.assertTrue(valid_imu_record(rows[0]))
        self.assertNotIn('pos',rows[0])
        self.assertFalse(valid_imu_record({'time':1}))
        self.assertEqual(imu_records([{},{}]),[])


if __name__ == '__main__':
    unittest.main()
