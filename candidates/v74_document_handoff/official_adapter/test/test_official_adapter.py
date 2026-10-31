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
TARGET_SELECTION = load("target_selection")


class OfficialAdapterTests(unittest.TestCase):
    def test_traffic_light_cluster_selection_prefers_nearest_supported_instance(self):
        near = (4, np.array([10., 2., 5.]), np.array([True, True, True, True]))
        far = (8, np.array([30., 2., 5.]), np.array([True] * 8))
        selected = TARGET_SELECTION.select_target_cluster(
            [far, near], origin=(0., 0.), prefer_nearest=True)
        self.assertIs(selected, near)

    def test_regular_target_selection_keeps_strongest_supported_instance(self):
        near = (4, np.array([10., 2., 5.]), np.array([True] * 4))
        far = (8, np.array([30., 2., 5.]), np.array([True] * 8))
        selected = TARGET_SELECTION.select_target_cluster(
            [near, far], origin=(0., 0.), prefer_nearest=False)
        self.assertIs(selected, far)

    def test_four_consistent_localizations_meet_five_frame_confirmation_support(self):
        required = TARGET_SELECTION.required_cluster_support(5)
        self.assertEqual(required, 4)
        positions = np.array([
            [724.13, -36.69, 55.88], [724.13, -36.68, 55.88],
            [724.28, -35.93, 55.93], [724.35, -36.33, 55.94],
        ])
        clusters = TARGET_SELECTION.supported_position_clusters(positions, 1.5, required)
        self.assertTrue(clusters)
        self.assertEqual(max(cluster[0] for cluster in clusters), 4)
        self.assertFalse(TARGET_SELECTION.supported_position_clusters(
            positions[:3], 1.5, required))

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

    def test_vehicle_self_occlusion_cannot_leave_rear_axle_unknown(self):
        # A forward-mounted virtual cloud origin and only forward returns
        # reproduce the live failure: the rear axle receives no clearing ray.
        points = np.array([[8., 0., 0.]])
        raw = MAPPING.occupancy_from_points(points, sensor_origin_xy=(2.172, -.330))
        fixed = MAPPING.occupancy_from_points(
            points, sensor_origin_xy=(2.172, -.330), clear_robot_footprint=True)
        self.assertEqual(int(raw[40,40]), -1)
        self.assertTrue(np.all(fixed[39:41,39:41] == 0))
        # A missing return behind the body is still unknown.
        self.assertEqual(int(fixed[40,36]), -1)

    def test_footprint_clear_preserves_external_obstacle_and_unknown_space(self):
        points = np.array([[5., 0., 1.]])
        raw = MAPPING.occupancy_from_points(points)
        fixed = MAPPING.occupancy_from_points(points, clear_robot_footprint=True)
        self.assertEqual(int(fixed[40,50]), 100)
        rows, columns = np.indices(raw.shape)
        x = (columns+.5-40)*.5; y = (rows+.5-40)*.5
        outside = (x < -1.05) | (x > 3.4) | (np.abs(y) > 1.05)
        np.testing.assert_array_equal(fixed[outside], raw[outside])

    def test_ideal_point_does_not_filter_or_clear_an_sbr_sized_footprint(self):
        obstacle = np.array([[1., 0., 1.]])
        car = MAPPING.occupancy_from_points(obstacle, width=40, height=40,
                                            clear_robot_footprint=True,
                                            carrier_model="ackermann")
        point = MAPPING.occupancy_from_points(obstacle, width=40, height=40,
                                              clear_robot_footprint=True,
                                              carrier_model="ideal_point")
        self.assertEqual(int(car[20, 22]), 0)
        self.assertEqual(int(point[20, 22]), 100)
        self.assertEqual(int(point[20, 20]), 0)

    def test_paused_clock_does_not_refresh_repeated_measurements(self):
        clock = CLOCK.MeasurementClock(tolerance_ns=5)
        self.assertEqual(clock.update(10, 1000), 1000)
        self.assertEqual(clock.update(12, 1002), 1002)
        self.assertEqual(clock.update(12, 5000), 1002)
        self.assertIsNone(clock.update(11, 5001))

    def test_sensor_uses_nearest_pose_not_only_latest_pose(self):
        samples = [(100, "before"), (200, "after")]
        matched = CLOCK.closest_state_sample(samples, 155, 50)
        self.assertEqual(matched, ((200, "after"), 45))

    def test_sensor_pose_match_rejects_skew_outside_tolerance(self):
        self.assertIsNone(CLOCK.closest_state_sample([(100, "pose")], 201, 100))

    def test_resumed_clock_reanchors_and_rejects_pre_pause_frames(self):
        clock = CLOCK.MeasurementClock(tolerance_ns=5)
        clock.update(10, 1000)
        self.assertEqual(clock.update(12, 5000), 5000)
        self.assertIsNone(clock.map(11))
        self.assertIsNone(clock.map(13))
        self.assertEqual(clock.update(14, 5002), 5002)
        self.assertEqual(clock.map(13), 5001)

    def test_simulator_clock_reset_starts_a_new_measurement_epoch(self):
        clock = CLOCK.MeasurementClock(tolerance_ns=5)
        self.assertEqual(clock.update(100, 1000), 1000)
        self.assertEqual(clock.update(101, 1001), 1001)
        self.assertEqual(clock.update(10, 2000), 2000)
        self.assertTrue(clock.last_update_reset)
        self.assertIsNone(clock.map(9))
        self.assertEqual(clock.map(10), 2000)
        self.assertEqual(clock.update(11, 2001), 2001)
        self.assertFalse(clock.last_update_reset)

    def test_clock_drift_cannot_create_future_odometry(self):
        clock = CLOCK.MeasurementClock()
        self.assertEqual(clock.update(10_000_000_000, 100_000_000_000), 100_000_000_000)
        self.assertEqual(clock.update(11_000_000_000, 100_800_000_000), 100_800_000_000)
        self.assertIsNone(clock.map(11_100_000_000))
        self.assertEqual(clock.map(10_500_000_000), 100_400_000_000)
        self.assertEqual(clock.update(12_000_000_000, 101_700_000_000), 101_700_000_000)
        self.assertEqual(clock.update(12_000_000_000, 110_000_000_000), 101_700_000_000)

    def test_variable_rate_clock_keeps_bounded_history_and_capture_alignment(self):
        clock = CLOCK.MeasurementClock()
        source, now = 1_000_000_000, 100_000_000_000
        previous = 0
        for i in range(1000):
            source += 100_000_000
            now += 80_000_000 if i % 2 else 110_000_000
            mapped = clock.update(source, now)
            self.assertLessEqual(mapped, now)
            self.assertGreater(mapped, previous)
            aligned = clock.map(source - 50_000_000)
            if aligned is not None:
                self.assertLessEqual(aligned, mapped)
                self.assertGreaterEqual(aligned, previous)
            previous = mapped
        self.assertEqual(len(clock.anchors), 200)

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

    def test_confirmed_target_can_approach_unknown_to_one_cell_buffer(self):
        grid = np.zeros((21, 21), dtype=np.int8)
        grid[10, 14:] = -1
        distance = MAPPING.safe_travel_distance(
            grid, 1.0, (-10.0, -10.0), (8.0, 0.0), 8.0,
            safety_margin=2.0, unknown_safety_margin=0.5)
        self.assertAlmostEqual(distance, 3.5)

    def test_confirmed_target_keeps_full_margin_for_occupied_cells(self):
        grid = np.zeros((21, 21), dtype=np.int8)
        grid[10, 14] = 100
        distance = MAPPING.safe_travel_distance(
            grid, 1.0, (-10.0, -10.0), (8.0, 0.0), 8.0,
            safety_margin=2.0, unknown_safety_margin=0.5)
        self.assertAlmostEqual(distance, 2.0)

    def test_ring_candidate_uses_clear_side(self):
        grid = np.zeros((40, 40), dtype=np.int8)
        grid[18:23, 27] = 100
        candidate = MAPPING.reachable_ring_candidate(
            grid, 1.0, (-20.0, -20.0), (15.0, 0.0), 5.0, 1.0)
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(float(np.linalg.norm(candidate - [15.0, 0.0])), 5.0)


if __name__ == "__main__": unittest.main()
