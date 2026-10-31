import struct
import unittest

import numpy as np

from wildos_ros2_adapter.camera import nominal_camera_info, optical_rotation_from_body
from wildos_ros2_adapter.pointcloud import decode_xyz, voxel_downsample


class Field:
    def __init__(self, name, offset):
        self.name = name
        self.offset = offset


class Cloud:
    fields = [Field("x", 0), Field("y", 4), Field("z", 8)]
    point_step = 12
    height = 1
    width = 2
    row_step = 24
    is_bigendian = False
    data = struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


class AdapterTests(unittest.TestCase):
    def test_decode_xyz(self):
        np.testing.assert_allclose(decode_xyz(Cloud()), [[1, 2, 3], [4, 5, 6]])

    def test_downsample_is_bounded(self):
        points = np.arange(300, dtype=np.float32).reshape(-1, 3)
        result = voxel_downsample(points, 0.1, 10)
        self.assertLessEqual(len(result), 10)

    def test_nominal_camera_info(self):
        info = nominal_camera_info(960, 640, 60.0, "ego_front_cam")
        self.assertEqual(info["width"], 960)
        self.assertEqual(info["height"], 640)
        self.assertEqual(info["frame_id"], "ego_front_cam")
        self.assertEqual(info["calibration_status"], "NOMINAL_UNVERIFIED")

    def test_optical_axis_correction(self):
        rotation = optical_rotation_from_body(np.eye(3))
        np.testing.assert_allclose(rotation[:, 2], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(rotation[:, 0], [0.0, -1.0, 0.0])


if __name__ == "__main__":
    unittest.main()
