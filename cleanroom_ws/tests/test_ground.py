import unittest
from pathlib import Path
import numpy as np
from wildos_nav2_adapter.pointcloud import (
    connected_ground_returns,
    supported_ground_patches,
    split_costmap_points,
)


def measured_ground(points, plane, radius, obstacle_height, max_slope):
    return connected_ground_returns(
        points, plane, radius, obstacle_height, max_slope
    ) | (supported_ground_patches(points, plane, radius, obstacle_height, max_slope))


class GroundTests(unittest.TestCase):
    @staticmethod
    def costmap_points(points):
        return split_costmap_points(
            points,
            min_height=0.1,
            max_height=1.5,
            filter_footprint=False,
            front_extent=3.4,
            rear_extent=1.05,
            half_width=1.05,
            voxel=0.2,
            max_points=20000,
            ground_plane_radius=8.0,
            obstacle_min_height=0.1,
        )[0]

    def test_low_obstacle_is_invariant_to_range_and_body_height(self):
        x, y = np.meshgrid(np.arange(0, 18, 0.15), np.arange(-3, 3, 0.15))
        points = np.c_[x.ravel(), y.ravel(), np.full(x.size, -0.26)]
        curb = (points[:, 0] > 9) & (points[:, 0] < 10) & (abs(points[:, 1]) < 1)
        points[curb, 2] += 0.15
        counts = []
        for shift in (0.0, 2.0):
            for body_offset in (0.0, -0.3, 0.3):
                with self.subTest(shift=shift, body_offset=body_offset):
                    observed = points - [shift, 0, body_offset]
                    marked = self.costmap_points(observed)
                    marked[:, 0] += shift
                    self.assertGreater(len(marked), 0)
                    self.assertTrue(
                        (
                            (marked[:, 0] > 9)
                            & (marked[:, 0] < 10)
                            & (abs(marked[:, 1]) < 1)
                        ).all()
                    )
                    counts.append(len(marked))
        self.assertEqual(len(set(counts)), 1)

    def test_curved_road_uses_local_height_for_occluding_low_obstacle(self):
        x, y = np.meshgrid(np.arange(0, 25, 0.3), np.arange(-4, 4, 0.3))
        points = np.c_[x.ravel(), y.ravel(), (0.001 * x * x - 0.3).ravel()]
        curb = (points[:, 0] > 17) & (points[:, 0] < 19) & (abs(points[:, 1]) < 0.8)
        points[curb, 2] += 0.15
        marked = self.costmap_points(points)
        # The wire output is float32; compare measured points after the same
        # cast rather than reclassifying a rounded coordinate at the .8 edge.
        self.assertEqual(
            {tuple(point) for point in marked},
            {tuple(point) for point in points[curb].astype(np.float32)},
        )

    def test_sparse_shared_boundary_extends_measured_ground(self):
        # A 6 KB lower-envelope fixture: measured cell minima/counts only.
        # The former twelve-cell overlap rule rejects all target supports.
        with np.load(Path(__file__).parent / "data/sparse_ground_boundary.npz") as data:
            points = np.repeat(data["support"], data["counts"], axis=0)
            target = np.repeat(data["target"], data["counts"])
            raised = points[target][::10].copy()
            raised[:, 2] += 0.15
            combined = np.r_[points, raised]
            ground = measured_ground(combined, data["plane"], 8.0, 0.1, 0.35)
            self.assertTrue(ground[: len(points)][target].all())
            self.assertFalse(ground[len(points) :].any())

    def test_box_occluding_ground_is_not_a_ramp(self):
        x, y = np.meshgrid(np.arange(0, 18, 0.15), np.arange(-4, 4, 0.15))
        points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        box = (
            (points[:, 0] > 10)
            & (points[:, 0] < 12)
            & (points[:, 1] > 0.2)
            & (points[:, 1] < 2.2)
        )
        for height in (0.12, 0.35):
            with self.subTest(height=height):
                points[box, 2] = height
                ground = measured_ground(points, np.zeros(3), 8.0, 0.1, 0.35)
                self.assertFalse(ground[box].any())

    def test_curving_road_is_ground_but_raised_returns_remain_obstacles(self):
        x, y = np.meshgrid(np.arange(0, 25, 0.3), np.arange(-4, 4, 0.3))
        road = np.c_[x.ravel(), y.ravel(), (0.001 * x * x - 0.3).ravel()]
        obstacles = road[100:130].copy()
        obstacles[:, 2] += 0.35
        points = np.r_[road, obstacles]
        ground = measured_ground(points, np.array([0, 0, -0.3]), 8.0, 0.1, 0.35)
        self.assertGreater(ground[: len(road)].mean(), 0.97)
        self.assertFalse(ground[len(road) :].any())

    def test_disconnected_raised_surface_is_not_inferred_as_road(self):
        x, y = np.meshgrid(np.arange(-3, 3, 0.25), np.arange(-3, 3, 0.25))
        road = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        raised = road.copy()
        raised[:, 0] += 20
        raised[:, 2] = 1.0
        ground = measured_ground(np.r_[road, raised], np.zeros(3), 8.0, 0.1, 0.35)
        self.assertTrue(ground[: len(road)].all())
        self.assertFalse(ground[len(road) :].any())
