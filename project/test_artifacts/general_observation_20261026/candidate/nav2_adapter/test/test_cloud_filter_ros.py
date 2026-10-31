"""Cloud-filter wiring for the ground-relative obstacle threshold."""
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from wildos_nav2_adapter.cloud_filter import CloudFilter


class CloudFilterThresholdTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_ground_relative_threshold_is_forwarded_to_split(self):
        node = CloudFilter()
        try:
            self.assertAlmostEqual(
                float(node.get_parameter("obstacle_min_height").value), .10)
            msg = PointCloud2()
            msg.header.frame_id = "base_link"
            msg.height = 1
            msg.width = 1
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 12
            msg.row_step = 12
            msg.is_bigendian = False
            msg.data = np.asarray([[1., 0., -.10]], dtype="<f4").tobytes()
            node.pub = Mock()
            empty = np.zeros((0, 3), dtype=np.float32)
            with patch("wildos_nav2_adapter.cloud_filter.split_costmap_points",
                       return_value=(empty, empty)) as split:
                node.callback(msg)
            self.assertEqual(split.call_args.kwargs["obstacle_min_height"], .10)
            self.assertEqual(split.call_args.kwargs["ground_max_slope"], .35)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
