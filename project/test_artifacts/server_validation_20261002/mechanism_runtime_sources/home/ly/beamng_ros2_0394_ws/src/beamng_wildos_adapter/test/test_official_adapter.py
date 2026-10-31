import importlib.util
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parents[1] / "beamng_wildos_adapter"
LOCAL_NAV2 = Path(__file__).resolve().parents[2] / "nav2_adapter"
if LOCAL_NAV2.is_dir():
    sys.path.insert(0, str(LOCAL_NAV2))


def load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / (name + ".py"))
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


GEOMETRY = load("geometry")
MAPPING = load("mapping")
HEALTH = load("health")
POINTCLOUD = load("pointcloud")
CLOCK = load("measurement_clock")


class OfficialAdapterTests(unittest.TestCase):
    def test_shifted_rear_axle_has_no_lateral_velocity_in_a_turn(self):
        offset = np.array([-2.172, .330, 0.])
        yaw_rate = .15
        # Rear axle travels at 1.2 m/s; the old reference point also sweeps
        # sideways because it is ahead of the axle.
        omega = np.array([0., 0., yaw_rate])
        reference_velocity = np.array([1.2, 0., 0.]) - np.cross(omega, offset)
        for heading in (0., 1.7, -2.8):
            rotation = GEOMETRY.quaternion_matrix([0., 0., math.sin(heading/2), math.cos(heading/2)])
            velocity = GEOMETRY.shifted_body_velocity(rotation @ reference_velocity, rotation, offset, omega)
            np.testing.assert_allclose(velocity, [1.2, 0., 0.], atol=1e-12)

    def test_base_shift_preserves_world_cloud_and_camera_coordinates(self):
        offset = np.array([-2.172, .330, 0.])
        rotation = GEOMETRY.quaternion_matrix([.02, -.01, .7, .71])
        reference = np.array([734.294, -11.946, 52.154])
        world = np.array([[740., -8., 53.], [730., -13., 52.]])
        virtual_cloud = (world-reference) @ rotation
        base = GEOMETRY.shifted_body_position(reference, rotation, offset)
        # base -> virtual cloud TF translates by -offset.
        reconstructed = (virtual_cloud-offset) @ rotation.T + base
        np.testing.assert_allclose(reconstructed, world, atol=1e-10)
        camera = np.array([733., -12., 53.2])
        np.testing.assert_allclose(((camera-base) @ rotation) @ rotation.T+base, camera)

    def test_rotation_velocity_uses_shortest_arc_across_yaw_wrap(self):
        before = GEOMETRY.quaternion_matrix([0.,0.,math.sin((math.pi-.01)/2),math.cos((math.pi-.01)/2)])
        after = GEOMETRY.quaternion_matrix([0.,0.,math.sin((-math.pi+.01)/2),math.cos((-math.pi+.01)/2)])
        np.testing.assert_allclose(GEOMETRY.rotation_velocity(before, after, .1), [0.,0.,.2], atol=1e-10)
        np.testing.assert_allclose(GEOMETRY.rotation_velocity(before, after, 0.), [0.,0.,0.])
        np.testing.assert_allclose(GEOMETRY.rotation_velocity(before, after, 10.), [0.,0.,0.])

    def test_measured_sbr_body_is_covered_from_the_rear_axle(self):
        from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET, SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH
        # Real wheel-hub and bounding-box measurement, not the configured box.
        for x in (-3.080011088, 1.109981544):
            for y in (-.670042495, 1.330012884):
                self.assertLessEqual(x-SBR_BASE_OFFSET[0], SBR_FRONT_EXTENT)
                self.assertGreaterEqual(x-SBR_BASE_OFFSET[0], -SBR_REAR_EXTENT)
                self.assertLessEqual(abs(y-SBR_BASE_OFFSET[1]), SBR_HALF_WIDTH)

    def test_occupancy_clearing_starts_at_preserved_cloud_origin(self):
        points = np.array([[4., 4., 0.]])
        grid = MAPPING.occupancy_from_points(points, .5, 40, 40, sensor_origin_xy=(2.,0.))
        self.assertEqual(int(grid[20,24]), 0)
        self.assertEqual(int(grid[20,20]), -1)

    def test_paused_clock_does_not_refresh_repeated_measurements(self):
        clock = CLOCK.MeasurementClock(tolerance_ns=5)
        self.assertEqual(clock.update(10, 1000), 1000)
        self.assertEqual(clock.update(12, 1002), 1002)
        self.assertEqual(clock.update(12, 5000), 1002)
        self.assertIsNone(clock.update(11, 5001))

    def test_resumed_clock_reanchors_and_rejects_pre_pause_frames(self):
        clock = CLOCK.MeasurementClock(tolerance_ns=5)
        clock.update(10, 1000)
        self.assertEqual(clock.update(12, 5000), 5000)
        self.assertIsNone(clock.map(11))
        self.assertEqual(clock.map(13), 5001)
        self.assertEqual(clock.update(14, 5002), 5002)
        self.assertEqual(clock.map(13), 5001)

    def test_organized_lidar_rows_and_scan_limit(self):
        fields = [SimpleNamespace(name=name, offset=offset)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        row1 = struct.pack("<ffffff", 1, 2, 3, 4, 5, 6) + b"\0" * 8
        row2 = struct.pack("<ffffff", 7, 8, 9, 10, 11, 12) + b"\0" * 8
        msg = SimpleNamespace(fields=fields, width=2, height=2,
                              point_step=12, row_step=32, is_bigendian=False,
                              data=row1 + row2)
        np.testing.assert_allclose(POINTCLOUD.decode_xyz(msg),
                                   [[1, 2, 3], [4, 5, 6],
                                    [7, 8, 9], [10, 11, 12]])
        samples = POINTCLOUD.limit_points(np.arange(100).reshape(-1, 1), 10)
        self.assertEqual(samples[0, 0], 0)
        self.assertEqual(samples[-1, 0], 99)

    def test_body_and_optical_axes(self):
        class V:
            def __init__(self, x, y, z): self.x,self.y,self.z=x,y,z
        body = GEOMETRY.body_rotation(V(1,0,0), V(0,0,1))
        np.testing.assert_allclose(body, np.eye(3))
        self.assertAlmostEqual(float(np.linalg.det(body)), 1.0)
        optical = GEOMETRY.optical_rotation(body)
        np.testing.assert_allclose(optical[:,2], [1,0,0])

    def test_occupancy_preserves_unknown_and_marks_obstacle(self):
        ground = np.array([[x, 0.0, 0.0] for x in np.linspace(0.5, 5.0, 40)])
        obstacle = np.array([[4.0, 0.0, 1.0]])
        grid = MAPPING.occupancy_from_points(np.vstack((ground, obstacle)), 0.5, 40, 40)
        self.assertIn(-1, grid)
        self.assertIn(0, grid)
        self.assertIn(100, grid)

    def test_graph_occupancy_filters_sloped_ground_and_preserves_obstacle(self):
        ground = np.asarray([(x, y, .2*x + .18*y - .45)
            for x in np.arange(0., 7.5, .25)
            for y in np.arange(-3., 3.25, .25)])
        obstacle = np.array([[4., 0., .75]])
        grid = MAPPING.occupancy_from_points(np.vstack((ground, obstacle)), .5, 40, 40)
        self.assertEqual(np.count_nonzero(grid == 100), 1)
        self.assertEqual(int(grid[20, 28]), 100)

    def test_seed_forward_from_yaw(self):
        class Q: pass
        q = Q(); q.x=0.0; q.y=0.0
        q.z=np.sin(np.pi/4); q.w=np.cos(np.pi/4)
        np.testing.assert_allclose(GEOMETRY.forward_xy(q), [0.0, 1.0], atol=1e-7)

    def test_graph_stats_require_real_frontier_scores(self):
        class O: pass
        graph=O(); graph.header=O(); graph.header.frame_id="spot1/odom"
        graph.trav_classes=["default"]; graph.current_node_idx=0; graph.edges=[]
        node=O(); trav=O(); trav.is_frontier=True; trav.frontier_points=[O()]
        node.trav_properties=[trav]; prop=O(); prop.key="frontier_scores"; prop.value=[0.2,0.8]
        node.properties=[prop]; graph.nodes=[node]
        stats=HEALTH.graph_stats(graph)
        self.assertTrue(HEALTH.graph_valid(stats, require_scores=True))
        self.assertEqual(stats["scored_nodes"], 1)

    def test_empty_graph_is_rejected(self):
        class O: pass
        graph=O(); graph.header=O(); graph.header.frame_id="spot1/odom"
        graph.nodes=[]; graph.edges=[]; graph.trav_classes=[]; graph.current_node_idx=0
        self.assertFalse(HEALTH.graph_valid(HEALTH.graph_stats(graph)))

    def test_standoff_stops_before_target(self):
        self.assertEqual(GEOMETRY.standoff_xy((0, 0), (20, 0), 10), (10, 0))
        self.assertEqual(GEOMETRY.standoff_xy((0, 0), (3, 4), 10), (0, 0))

    def test_obstacle_clamps_travel_before_threshold(self):
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[10, 16] = 100
        distance = MAPPING.safe_travel_distance(
            grid, 1.0, (-10.0, -10.0), (10.0, 0.0), 8.0, 2.0)
        self.assertLessEqual(distance, 4.5)

    def test_obstacle_endpoint_wins_over_later_free_space_rays(self):
        obstacle = np.array([[4.0, 0.0, 1.0]])
        farther_ground = np.array([[8.0, 0.0, 0.0]])
        for points in (np.vstack((obstacle, farther_ground)),
                       np.vstack((farther_ground, obstacle))):
            grid = MAPPING.occupancy_from_points(points, 1.0, 40, 40)
            self.assertEqual(int(grid[20, 24]), 100)

    def test_unknown_cells_stop_standoff_travel(self):
        grid = np.zeros((21, 21), dtype=np.int8)
        grid[10, 14:] = -1
        distance = MAPPING.safe_travel_distance(
            grid, 1.0, (-10.0, -10.0), (8.0, 0.0), 8.0, 2.0)
        self.assertLessEqual(distance, 2.0)

    def test_ring_candidate_uses_clear_side(self):
        grid = np.zeros((40, 40), dtype=np.int8)
        grid[18:23, 27] = 100
        candidate = MAPPING.reachable_ring_candidate(
            grid, 1.0, (-20.0, -20.0), (15.0, 0.0), 5.0, 1.0)
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(float(np.linalg.norm(candidate - [15.0, 0.0])), 5.0)


if __name__ == "__main__": unittest.main()
