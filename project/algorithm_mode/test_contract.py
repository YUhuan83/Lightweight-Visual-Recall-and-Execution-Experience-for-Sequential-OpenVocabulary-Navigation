"""Synthetic algebra fixtures only; no BeamNG coordinate claims."""

import copy
from pathlib import Path
import unittest

from algorithm_mode.contract import (
    Invalid, OraclePoseAdapter, load_config, validate_config, validate_observation,
)


RX = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
RY = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
RZ = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
TEMPLATE = Path(__file__).with_name("config_template.json")


def fixture(velocity_frame="source_world"):
    config = load_config(TEMPLATE)
    config["calibration"].update(
        verified=True, evidence_ref="synthetic-algebra-not-real-calibration",
        source_world_axes="synthetic right-handed W",
        source_body_axes="synthetic right-handed S",
        reference_points_coincident=True, R_source_nav=RX,
        velocity_frame=velocity_frame, camera_frame="camera_optical",
        lidar_frame="lidar", sensor_evidence_ref="synthetic-sensors-only",
    )
    origin = {"epoch": "episode-0", "initial_sim_time_s": 0,
              "position_world_m": [10, 20, 30], "R_world_odom": RZ}
    ego = {"epoch": "episode-0", "acquisition_sim_time_s": 0,
           "position_world_m": [11, 22, 33], "R_world_source": RY,
           "linear_velocity": [1, 2, 3], "angular_velocity": [4, 5, 6]}
    camera = {"epoch": "episode-0", "acquisition_sim_time_s": 0,
              "frame_id": "camera_optical", "width": 1, "height": 1, "rgb": b"\x01\x02\x03"}
    lidar = {"epoch": "episode-0", "acquisition_sim_time_s": 0,
             "frame_id": "lidar", "points_xyz_m": [[1, 2, 3]]}
    return copy.deepcopy((config, origin, ego, camera, lidar))


class ContractTests(unittest.TestCase):
    def test_template_valid_but_not_ready(self):
        config = load_config(TEMPLATE)
        self.assertFalse(validate_config(config)["ready"])
        with self.assertRaises(Invalid):
            validate_config(config, ready=True)
        config["calibration"]["verified"] = True
        with self.assertRaises(Invalid):
            validate_config(config)

    def test_config_boundary_and_calibration(self):
        for key, value in (("R_source_nav", [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                           ("R_source_nav", [[1, 1, 0], [0, 1, 0], [0, 0, 1]]),
                           ("R_source_nav", [[float("nan"), 0, 0], [0, 1, 0], [0, 0, 1]]),
                           ("velocity_frame", "guessed_beamng"),
                           ("reference_points_coincident", False),
                           ("verified", 1), ("evidence_ref", "")):
            with self.subTest(key=key, value=value):
                config, origin, *_ = fixture()
                config["calibration"][key] = value
                with self.assertRaises(Invalid):
                    OraclePoseAdapter(config, origin)
        config, *_ = fixture()
        config["profiles"]["oracle_pose"]["pose_source"] = "full_truth"
        with self.assertRaises(Invalid):
            validate_config(config)

    def test_full_se3_world_velocity_and_zero_stamp(self):
        config, origin, ego, camera, lidar = fixture()
        result = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        odom = result["odom"]
        self.assertEqual(result["acquisition_sim_time_s"], 0)
        self.assertEqual(odom["position_m"], [2, -1, 3])
        self.assertEqual(odom["R_odom_nav"], [[0, 0, -1], [0, -1, 0], [-1, 0, 0]])
        self.assertEqual(odom["linear_velocity_m_s"], [-3, 1, -2])
        self.assertEqual(odom["angular_velocity_rad_s"], [-6, 4, -5])
        self.assertIsNone(odom["covariance"])

    def test_source_body_velocity(self):
        config, origin, ego, camera, lidar = fixture("source_body")
        result = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        self.assertEqual(result["odom"]["linear_velocity_m_s"], [1, 3, -2])
        self.assertEqual(result["odom"]["angular_velocity_rad_s"], [4, 6, -5])

    def test_unknown_velocity_is_not_zero(self):
        config, origin, ego, camera, lidar = fixture()
        ego["linear_velocity"] = ego["angular_velocity"] = None
        result = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        self.assertIsNone(result["odom"]["linear_velocity_m_s"])
        self.assertIsNone(result["odom"]["angular_velocity_rad_s"])

    def test_fixed_origin_and_copied_inputs(self):
        config, origin, ego, camera, lidar = fixture()
        adapter = OraclePoseAdapter(config, origin)
        adapter.adapt(ego, camera, lidar)
        origin["position_world_m"][0] = 999
        origin["R_world_odom"][0][0] = 999
        config["calibration"]["R_source_nav"][0][0] = 999
        for item in (ego, camera, lidar):
            item["acquisition_sim_time_s"] = 1
        ego["position_world_m"] = [14, 25, 36]
        result = adapter.adapt(ego, camera, lidar)
        self.assertEqual(result["odom"]["position_m"], [5, -4, 6])
        lidar["points_xyz_m"][0][0] = 999
        self.assertEqual(result["lidar"]["points_xyz_m"], [[1, 2, 3]])

    def test_truth_allowlist(self):
        config, origin, ego, camera, lidar = fixture()
        expected = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        ego.update(target_ground_truth={"position": [999, 999, 999]},
                   full_map={"hidden_semantics": ["secret"]},
                   scoring_target="secret", arbitrary_nested={"truth": True})
        actual = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        self.assertEqual(actual, expected)
        for sensor_name in ("camera", "lidar"):
            for hidden_key in ("target_ground_truth", "full_map", "hidden_semantics"):
                sensors = {"camera": copy.deepcopy(camera), "lidar": copy.deepcopy(lidar)}
                sensors[sensor_name][hidden_key] = "secret"
                with self.assertRaises(Invalid):
                    OraclePoseAdapter(config, origin).adapt(ego, **sensors)
        lidar["points_xyz_m"] = [{"xyz": [1, 2, 3], "truth": "secret"}]
        with self.assertRaises(Invalid):
            OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)

    def test_estimated_rejects_oracle(self):
        config, origin, ego, camera, lidar = fixture()
        with self.assertRaises(Invalid):
            OraclePoseAdapter(config, origin, mode="estimated_pose")
        result = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        with self.assertRaises(Invalid):
            validate_observation(result, "estimated_pose")
        result["localization_mode"] = "estimated_pose"
        with self.assertRaises(Invalid):
            validate_observation(result, "estimated_pose")
        # Structural LIO fixture, not an estimator or proof of data provenance.
        result["odom"]["provenance"] = "lio_covariance_unknown"
        validate_observation(result, "estimated_pose")

    def test_bad_stamp_and_epoch(self):
        for value in (float("nan"), float("inf"), -1, True, None, "0", 10**1000):
            config, origin, ego, camera, lidar = fixture()
            ego["acquisition_sim_time_s"] = value
            with self.subTest(value=str(value)), self.assertRaises(Invalid):
                OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        for value in ("", 0, None, "another-episode"):
            config, origin, ego, camera, lidar = fixture()
            ego["epoch"] = value
            with self.assertRaises(Invalid):
                OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)

    def test_duplicate_reverse_reset_and_failed_sample_atomicity(self):
        config, origin, ego, camera, lidar = fixture()
        adapter = OraclePoseAdapter(config, origin)
        adapter.adapt(ego, camera, lidar)
        with self.assertRaises(Invalid):
            adapter.adapt(ego, camera, lidar)
        for item in (ego, camera, lidar):
            item["acquisition_sim_time_s"] = 2
        lidar["epoch"] = "reset"
        with self.assertRaises(Invalid):
            adapter.adapt(ego, camera, lidar)
        lidar["epoch"] = ego["epoch"]
        adapter.adapt(ego, camera, lidar)
        for item in (ego, camera, lidar):
            item["acquisition_sim_time_s"] = 1
        with self.assertRaises(Invalid):
            adapter.adapt(ego, camera, lidar)

    def test_sensor_and_pose_validation(self):
        mutations = [
            ("camera", "acquisition_sim_time_s", True),
            ("camera", "acquisition_sim_time_s", 1),
            ("camera", "rgb", b""), ("camera", "width", True),
            ("camera", "frame_id", "unverified"),
            ("lidar", "points_xyz_m", [[0, float("nan"), 1]]),
            ("ego", "position_world_m", [0, 0, float("inf")]),
            ("ego", "linear_velocity", [True, 0, 0]),
            ("ego", "R_world_source", [[0, 0, 0]] * 3),
        ]
        for name, key, value in mutations:
            config, origin, ego, camera, lidar = fixture()
            {"ego": ego, "camera": camera, "lidar": lidar}[name][key] = value
            with self.subTest(name=name, key=key), self.assertRaises(Invalid):
                OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)

    def test_nonzero_sim_start_and_covariance_no_minus_one(self):
        config, origin, ego, camera, lidar = fixture()
        origin["initial_sim_time_s"] = 50
        for item in (ego, camera, lidar):
            item["acquisition_sim_time_s"] = 50
        obs = OraclePoseAdapter(config, origin).adapt(ego, camera, lidar)
        self.assertEqual(obs["acquisition_sim_time_s"], 50)
        obs["odom"]["covariance"] = [-1] + [0] * 35
        with self.assertRaises(Invalid):
            validate_observation(obs, "oracle_pose")


if __name__ == "__main__":
    unittest.main()
