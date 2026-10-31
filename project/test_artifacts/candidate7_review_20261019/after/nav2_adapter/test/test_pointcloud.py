import importlib.util
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "pointcloud", HERE / "wildos_nav2_adapter" / "pointcloud.py")
POINTCLOUD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POINTCLOUD)


class PointCloudHeightTests(unittest.TestCase):
    def test_ground_plane_keeps_minimum_return_xy_on_slopes(self):
        import numpy as np
        xy=np.array([(x,y) for x in np.arange(.03,7.5,.1)
                     for y in np.arange(-2.97,3.,.1)])
        for a,b in ((.24,.24),(-.24,.24),(.24,-.24),(-.24,-.24)):
            with self.subTest(slope=(a,b)):
                points=np.c_[xy,a*xy[:,0]+b*xy[:,1]-.45]
                expected=np.array([a,b,-.45])
                plane=POINTCLOUD.local_ground_plane(points,8.)
                self.assertIsNotNone(plane)
                np.testing.assert_allclose(plane,expected,atol=1e-7)

    def test_minimum_return_xy_pair_survives_object_outliers_and_shuffle(self):
        import numpy as np
        xy=np.array([(x,y) for x in np.arange(.03,7.5,.1)
                     for y in np.arange(-2.97,3.,.1)])
        ground=np.c_[xy,.24*xy[:,0]+.24*xy[:,1]-.45]
        raised=ground.copy();raised[:,2]+=.4
        points=np.vstack((ground,raised))
        np.random.default_rng(42).shuffle(points)
        np.testing.assert_allclose(POINTCLOUD.local_ground_plane(points,8.),[.24,.24,-.45],atol=1e-7)

    def test_ground_plane_does_not_invent_support_from_sparse_or_collinear_data(self):
        import numpy as np
        self.assertIsNone(POINTCLOUD.local_ground_plane(np.zeros((99,3)),8.))
        x=np.linspace(-7.,7.,300)
        self.assertIsNone(POINTCLOUD.local_ground_plane(np.c_[x,np.zeros_like(x),.1*x-.4],8.))

    def test_recorded_overhead_collision_returns_do_not_mark_sbr_costmap(self):
        import numpy as np
        from wildos_nav2_adapter.vehicle_geometry import SBR_OBSTACLE_MAX_HEIGHT
        points = np.asarray([
            [2.1032629, -1.3996739, 2.9788213],
            [2.0317724, -1.5974663, 2.8383639],
            [2.0333366, -1.6166689, 2.8325136],
            [4., 0., .4],  # A low obstacle must remain marked.
        ], dtype=np.float32)
        opts = dict(min_height=.1, filter_footprint=False, front_extent=3.4,
                    rear_extent=1.05, half_width=1.05, voxel=.2, max_points=100)
        old, _ = POINTCLOUD.split_costmap_points(points, max_height=3., **opts)
        new, clearing = POINTCLOUD.split_costmap_points(points,
            max_height=SBR_OBSTACLE_MAX_HEIGHT, **opts)
        self.assertGreater(len(old), 1)
        np.testing.assert_allclose(new, points[-1:])
        # High rays must not erase low obstacle cells, either.
        np.testing.assert_allclose(clearing, points[-1:])

    def test_decodes_organized_cloud_with_row_padding(self):
        fields = [SimpleNamespace(name=name, offset=offset)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        first = struct.pack("<ffffff", 1, 2, 3, 4, 5, 6) + b"\0" * 8
        second = struct.pack("<ffffff", 7, 8, 9, 10, 11, 12) + b"\0" * 8
        cloud = SimpleNamespace(fields=fields, point_step=12, row_step=32,
                                width=2, height=2, is_bigendian=False,
                                data=first + second)

        self.assertEqual(POINTCLOUD.decode_xyz(cloud).tolist(),
                         [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])

    def test_rejects_truncated_organized_cloud(self):
        fields = [SimpleNamespace(name=name, offset=offset)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        cloud = SimpleNamespace(fields=fields, point_step=12, row_step=32,
                                width=2, height=2, is_bigendian=False,
                                data=b"\0" * 56)

        with self.assertRaisesRegex(ValueError, "truncated"):
            POINTCLOUD.decode_xyz(cloud)

    def test_voxel_limit_keeps_scan_coverage(self):
        import numpy as np

        points = np.column_stack((np.arange(100), np.zeros(100), np.ones(100)))
        selected = POINTCLOUD.voxel_downsample(points, 0.2, 10)
        self.assertEqual(len(selected), 10)
        self.assertEqual(float(selected[0, 0]), 0.0)
        self.assertEqual(float(selected[-1, 0]), 99.0)

    def test_filters_ground_and_overhead_but_keeps_low_obstacles(self):
        import numpy as np

        points = np.asarray([
            [1.0, 0.0, -0.10],  # road surface in base_link
            [2.0, 0.0, 0.17],   # low obstacle previously omitted by Nav2
            [3.0, 0.0, 3.10],   # overhead/noise beyond configured range
        ], dtype=np.float32)
        filtered = POINTCLOUD.filter_height(points, 0.1, 3.0)
        self.assertEqual(filtered.shape, (1, 3))
        self.assertAlmostEqual(float(filtered[0, 2]), 0.17, places=5)

    def test_rejects_invalid_height_bounds(self):
        import numpy as np

        with self.assertRaises(ValueError):
            POINTCLOUD.filter_height(np.zeros((1, 3)), 2.0, 1.0)

    def test_removes_vehicle_body_returns_but_keeps_external_obstacles(self):
        import numpy as np

        points = np.asarray([
            [0.0, 0.0, 0.8],       # body/hood return
            [-2.0, 0.9, 0.5],      # inside the configured footprint
            [2.31, 0.0, 0.5],      # obstacle just ahead of the body
            [0.0, -1.01, 0.5],     # obstacle beside the body
        ], dtype=np.float32)
        filtered = POINTCLOUD.filter_robot_footprint(points, 2.3, 2.3, 1.0)
        self.assertEqual(filtered.tolist(), points[2:].tolist())

    def test_rejects_invalid_robot_footprint(self):
        import numpy as np

        with self.assertRaises(ValueError):
            POINTCLOUD.filter_robot_footprint(
                np.zeros((1, 3)), 2.3, -1.0, 1.0)

    def test_ground_clears_but_does_not_mark_an_obstacle(self):
        import numpy as np

        points = np.asarray([
            [0.0, 0.0, 0.7],   # self return
            [4.0, 0.0, -0.3],  # road return
            [5.0, 0.0, 0.5],   # obstacle return
            [6.0, 0.0, 3.5],   # overhanging return cannot clear lower cells
        ], dtype=np.float32)
        markings, clearing = POINTCLOUD.split_costmap_points(
            points, min_height=0.1, max_height=3.0,
            filter_footprint=True, front_extent=2.3, rear_extent=2.3,
            half_width=1.0, voxel=0.2, max_points=20)
        self.assertEqual(markings.tolist(), [[5.0, 0.0, 0.5]])
        np.testing.assert_allclose(clearing, points[1:3])

    def test_sloped_road_is_cleared_but_low_object_above_road_is_marked(self):
        import numpy as np

        ground = np.asarray([
            (x, y, .20 * x + .18 * y - .45)
            for x in np.arange(0., 7.5, .25)
            for y in np.arange(-3., 3.25, .25)
        ], dtype=np.float32)
        low_object = np.asarray([[4., 0., .20 * 4. - .45 + .4]], dtype=np.float32)
        points = np.vstack((ground, low_object))
        plane = POINTCLOUD.local_ground_plane(points, 8.)
        self.assertIsNotNone(plane)
        np.testing.assert_allclose(plane[:2], [.20, .18], atol=.03)
        marked, cleared = POINTCLOUD.split_costmap_points(
            points, min_height=.1, max_height=3., filter_footprint=False,
            front_extent=2.3, rear_extent=2.3, half_width=1.,
            voxel=.1, max_points=10000,
            ground_plane_radius=8., ground_clearance=.25)
        self.assertEqual(len(marked), 1)
        np.testing.assert_allclose(marked[0], low_object[0], atol=.01)
        self.assertGreater(len(cleared), len(marked))

    def test_ground_relative_marking_keeps_low_kerb_below_base_frame_gate(self):
        import numpy as np
        ground = np.asarray([(x, y, -.10)
            for x in np.arange(.05, 7.5, .25)
            for y in np.arange(-3., 3.05, .25)], dtype=np.float32)
        kerb = np.asarray([(x, y, .06)
            for x in np.arange(4.0, 4.6, .1)
            for y in np.arange(-.3, .35, .1)], dtype=np.float32)
        points = np.vstack((ground, kerb))
        opts = dict(min_height=.1, max_height=3., filter_footprint=False,
                    front_extent=2.3, rear_extent=2.3, half_width=1.,
                    voxel=.1, max_points=10000, ground_plane_radius=8.,
                    ground_clearance=.25)
        legacy, _ = POINTCLOUD.split_costmap_points(points, **opts)
        self.assertEqual(len(legacy), 0)
        marked, _ = POINTCLOUD.split_costmap_points(
            points, obstacle_min_height=.10, **opts)
        self.assertGreater(len(marked), 0)
        self.assertTrue(np.all(marked[:, 2] >= .05))
        self.assertFalse(np.any(marked[:, 2] < 0.))

    def test_ground_relative_marking_does_not_mark_a_smooth_slope(self):
        import numpy as np
        points = np.asarray([(x, y, .17 * x - .10)
            for x in np.arange(.05, 7.5, .25)
            for y in np.arange(-3., 3.05, .25)], dtype=np.float32)
        marked, _ = POINTCLOUD.split_costmap_points(
            points, min_height=.1, max_height=3., filter_footprint=False,
            front_extent=2.3, rear_extent=2.3, half_width=1., voxel=.1,
            max_points=10000, ground_plane_radius=8., ground_clearance=.25,
            obstacle_min_height=.10)
        self.assertEqual(len(marked), 0)

    def test_ground_relative_marking_rejects_invalid_height(self):
        import numpy as np
        points = np.zeros((1, 3))
        for value in (0., -.1, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    POINTCLOUD.split_costmap_points(
                        points, min_height=.1, max_height=3.,
                        filter_footprint=False, front_extent=2.3,
                        rear_extent=2.3, half_width=1., voxel=.2,
                        max_points=10, ground_plane_radius=8.,
                        obstacle_min_height=value)

    def test_ground_relative_marking_keeps_legacy_far_field_gate(self):
        import numpy as np
        ground = np.asarray([(x, y, -.10)
            for x in np.arange(.05, 7.5, .25)
            for y in np.arange(-3., 3.05, .25)], dtype=np.float32)
        far_high = np.asarray([[10.0, 0.0, .50]], dtype=np.float32)
        far_low = np.asarray([[10.0, 0.2, .06]], dtype=np.float32)
        marked, _ = POINTCLOUD.split_costmap_points(
            np.vstack((ground, far_high, far_low)),
            min_height=.1, max_height=3., filter_footprint=False,
            front_extent=2.3, rear_extent=2.3, half_width=1., voxel=.1,
            max_points=10000, ground_plane_radius=8., ground_clearance=.25,
            obstacle_min_height=.10)
        self.assertEqual(len(marked), 1)
        np.testing.assert_allclose(marked[0], far_high[0])

    def test_flat_ground_prevents_a_raised_patch_becoming_the_road_plane(self):
        import numpy as np
        ground = np.asarray([(x, y, 0.)
            for x in np.arange(0., 7.5, .25)
            for y in np.arange(-3., 3.25, .25)], dtype=np.float32)
        raised = np.asarray([(x, y, .4)
            for x in np.arange(3., 7., .25)
            for y in np.arange(-2., 2., .25)], dtype=np.float32)
        marked, _ = POINTCLOUD.split_costmap_points(
            np.vstack((ground, raised)), min_height=.1, max_height=3.,
            filter_footprint=False, front_extent=2.3, rear_extent=2.3,
            half_width=1., voxel=.1, max_points=10000,
            ground_plane_radius=8., ground_clearance=.25)
        self.assertEqual(len(marked), len(raised))


if __name__ == "__main__":
    unittest.main()
