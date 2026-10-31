import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from integration import build_frame, capture_frames, common, make_calibration
from target_localization.core import Localizer, localize, validate_calibration
from target_localization.query_io import load_query


def synthetic_capture(root, image_gap=0.0):
    root = Path(root)
    (root / "static.png").write_bytes(b"static-image-bytes")
    (root / "mounted.png").write_bytes(b"mounted-image-bytes")
    (root / "static.json").write_text(json.dumps({
        "api_fit": {"fx": -556.157, "fy": -554.681, "cx": 479.56, "cy": 319.40,
                    "nominal_focal": 554.256, "passed": False},
        "visual": {"passed": False, "detected": False, "reason": "Chessboard not detected"}}))
    (root / "mounted.json").write_text(json.dumps({"metadata": {"host_monotonic_start": 241212.856}}))
    (root / "lidar.json").write_text(json.dumps({"metadata": {"host_monotonic_start": 241212.856 + image_gap}}))
    np.save(root / "lidar.npy", np.zeros((0, 3), dtype=np.float32))
    return common.load_capture(root)


class GeometryTest(unittest.TestCase):
    def test_nominal_focal_matches_fixture(self):
        k = common.nominal_k()
        self.assertAlmostEqual(k[0][0], 554.2562584220408, places=6)
        self.assertAlmostEqual(k[1][1], 554.2562584220408, places=6)
        self.assertEqual(k[0][2], 480.0)
        self.assertEqual(k[1][2], 320.0)

    def test_static_camera_projects_checker_centre(self):
        rotation, translation = common.static_world_to_camera()
        optical = rotation @ np.array([0.0, -9.999, 2.35]) + translation
        self.assertTrue(np.allclose(optical, [0.0, 0.0, 9.999], atol=1e-6))
        k = common.nominal_k()
        uv = optical[:2] / optical[2] * [k[0][0], k[1][1]] + [k[0][2], k[1][2]]
        self.assertTrue(np.allclose(uv, [480.0, 320.0], atol=1e-6))

    def test_se3_is_proper_orthonormal(self):
        rotation, translation = common.static_world_to_camera()
        self.assertTrue(np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12))
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)
        self.assertEqual(translation.shape, (3,))


class CalibrationTest(unittest.TestCase):
    def test_draft_then_verified(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = synthetic_capture(tmp)
            draft = make_calibration.build(None, capture)
            self.assertEqual(draft["status"], "DRAFT")
            self.assertFalse(draft["evidence"]["accepted"])
            verified = make_calibration.build("manual sign-convention acceptance", capture)
            self.assertEqual(verified["status"], "VERIFIED")
            validate_calibration(verified)

    def test_verified_k_is_positive(self):
        k = validate_calibration(make_calibration.build("accepted", None))
        self.assertGreater(k[0, 0], 0)
        self.assertGreater(k[1, 1], 0)


class SynchronizationTest(unittest.TestCase):
    def test_equal_stamps_are_synchronized(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = synthetic_capture(tmp, image_gap=0.0)
            self.assertTrue(build_frame.synchronization(capture)["synchronized"])
            self.assertIsNone(build_frame.timing_block(capture))

    def test_real_gap_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = synthetic_capture(tmp, image_gap=0.955)
            sync = build_frame.synchronization(capture)
            self.assertFalse(sync["synchronized"])
            self.assertGreater(sync["gap_ns"], build_frame.SYNC_TOLERANCE_NS)
            self.assertTrue(build_frame.timing_block(capture).startswith("unsynchronized_acquisition"))
            with self.assertRaises(build_frame.NotSynchronized):
                build_frame.build_frame(capture, tmp, {}, {}, "epoch")


def write_query_artifacts(run_dir, region, background=0.0, peak=0.2, threshold=0.1):
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    prefix = "query_000"
    dense = np.full(region.shape, background, dtype=np.float32)
    dense[region] = peak
    np.save(run_dir / (prefix + "_cosine.npy"), dense, allow_pickle=False)
    Image.fromarray((region * 255).astype(np.uint8)).save(run_dir / (prefix + "_region_mask.png"))
    h, w = dense.shape
    ys, xs = np.nonzero(region)
    regions = [{"box_xyxy": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
                "area_pixels": int(len(xs)), "mean_cosine": float(dense[ys, xs].mean()),
                "max_cosine": float(dense[ys, xs].max())}]
    report = {"status": "completed", "inference_ran": True,
              "box_convention": "xyxy, exclusive right/bottom, EXIF-oriented original image pixels",
              "image_size": [w, h], "input_size": [w, h], "threshold": threshold,
              "results": [{"query": "target", "prefix": prefix,
                           "raw_min": float(dense.min()), "raw_max": float(dense.max()),
                           "regions": regions}]}
    (run_dir / "result.json").write_text(json.dumps(report))
    return load_query(run_dir / "result.json", prefix)


def identity_calibration(width, height, focal=10.0):
    return {"schema": "target-calibration/1", "status": "VERIFIED",
            "evidence": {"accepted": True, "reference": "test"},
            "world_frame": "world_x", "camera_frame": "cam_x", "local_cloud_frame": "lidar_x",
            "time_source": "test_clock", "units": "m",
            "camera": {"width": width, "height": height, "projection": "rectilinear_pinhole",
                       "pixel_center": "integer_centers_half_extent",
                       "axes": "x_right_y_down_z_forward",
                       "pixels_in_target_optical_frame": True,
                       "K": [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]],
                       "D": [0, 0, 0, 0, 0], "evidence": {"accepted": True, "reference": "test"}},
            "local_to_camera": {"source_frame": "lidar_x", "target_frame": "cam_x",
                                "R": np.eye(3).tolist(), "translation_m": [0, 0, 0], "units": "m",
                                "evidence": {"accepted": True, "reference": "test"}}}


class StrictValidationTest(unittest.TestCase):
    def test_load_query_accepts_generated_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            region = np.zeros((32, 32), dtype=bool)
            region[14:18, 14:18] = True
            query = write_query_artifacts(Path(tmp) / "run", region)
            self.assertEqual(query["image_size"], [32, 32])
            self.assertEqual(query["prefix"], "query_000")
            self.assertEqual(set(query["artifact_sha256"]), {"result", "cosine", "mask"})

    def test_frame_matches_strict_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            image = tmp / "frame.png"
            image.write_bytes(b"frame-bytes")
            region = np.zeros((32, 32), dtype=bool)
            region[14:18, 14:18] = True
            query = write_query_artifacts(tmp / "run", region)
            calibration = identity_calibration(32, 32)
            frame = build_frame.build_frame_from_stamp(
                {"paths": {}}, image, query, calibration, "test_epoch", 1_000_000,
                points=np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 5.5]], float))
            frame["image"]["timing"]["source"] = calibration["time_source"]
            frame["cloud"]["timing"] = frame["image"]["timing"]
            frame["pose"]["timing"] = frame["image"]["timing"]
            frame["now"] = dict(frame["image"]["timing"])
            frame["pose"]["world_to_camera"].update({"R": np.eye(3).tolist(),
                                                     "translation_m": [0, 0, 0],
                                                     "evidence": {"accepted": True, "reference": "test"}})
            result = localize(query, frame, calibration)
            self.assertEqual(result["schema"], "coarse3d-observation/1")
            handoff = Localizer(calibration).process(query, frame)
            self.assertEqual(handoff["schema"], "target-handoff/1")


class CaptureGeometryTest(unittest.TestCase):
    def test_optical_basis_matches_fixture(self):
        right, down, forward, stacked = capture_frames.optical_basis([0, -1, 0])
        self.assertTrue(np.allclose(right, [-1, 0, 0], atol=1e-12))
        self.assertTrue(np.allclose(down, [0, 0, -1], atol=1e-12))
        self.assertTrue(np.allclose(forward, [0, -1, 0], atol=1e-12))
        self.assertTrue(np.allclose(stacked @ stacked.T, np.eye(3), atol=1e-12))

    def test_world_to_camera_projects_checker_centre(self):
        rotation, translation = capture_frames.world_to_camera([0, 0, 2.35], [0, -1, 0])
        optical = rotation @ np.array([0.0, -9.999, 2.35]) + translation
        self.assertTrue(np.allclose(optical, [0, 0, 9.999], atol=1e-6))

    def test_pose_stability_detects_motion(self):
        state = {"pos": [0, 0, 1], "up": [0, 0, 1], "vel": [0, 0, 0]}
        poses = {"camera": (np.array([0, 0, 2.35]), np.array([0, -1, 0]))}
        stable, _ = capture_frames.pose_stability(state, state, poses, poses)
        self.assertTrue(stable)
        moved = {"pos": [0.5, 0, 1], "up": [0, 0, 1], "vel": [0, 0, 0]}
        stable, evidence = capture_frames.pose_stability(state, moved, poses, poses)
        self.assertFalse(stable)
        self.assertAlmostEqual(evidence["vehicle_position_delta_m"], 0.5)


class CaptureRecordTest(unittest.TestCase):
    def make_record(self, calibration, image, synchronized=True):
        return {"schema": "capture-frame/1", "epoch": "epoch_x", "frame_index": 0,
                "synchronized": synchronized, "time_source": calibration["time_source"],
                "image": {"file": "image.png", "sha256": build_frame.common.sha256(image),
                          "width": 32, "height": 32},
                "cloud": {"file": "cloud.npy", "coordinates": "world", "units": "m", "points": 2},
                "pose": {"source_frame": calibration["world_frame"],
                         "target_frame": calibration["camera_frame"],
                         "R": np.eye(3).tolist(), "translation_m": [0, 0, 0], "units": "m"},
                "timing": {"stamp_ns": 123, "source": calibration["time_source"],
                           "epoch": "epoch_x", "time_kind": "acquisition"}}

    def test_synchronized_record_composes_valid_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            image = tmp / "frame.png"
            image.write_bytes(b"capture-bytes")
            region = np.zeros((32, 32), dtype=bool)
            region[14:18, 14:18] = True
            query = write_query_artifacts(tmp / "run", region)
            calibration = identity_calibration(32, 32)
            record = self.make_record(calibration, image)
            frame = build_frame.from_capture_record(
                record, image, query, calibration,
                points=np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 5.5]], float))
            result = localize(query, frame, calibration)
            self.assertEqual(result["schema"], "coarse3d-observation/1")

    def test_unsynchronized_record_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            image = tmp / "frame.png"
            image.write_bytes(b"capture-bytes")
            region = np.zeros((32, 32), dtype=bool)
            region[14:18, 14:18] = True
            query = write_query_artifacts(tmp / "run", region)
            calibration = identity_calibration(32, 32)
            record = self.make_record(calibration, image, synchronized=False)
            with self.assertRaises(ValueError):
                build_frame.from_capture_record(record, image, query, calibration,
                                                points=np.zeros((2, 3), float))


def inner_region():
    region = np.zeros((32, 32), dtype=bool)
    region[14:18, 14:18] = True
    return region


def confirmation_points():
    inner, outer = [], []
    for u in (14, 15, 16, 17):
        for v in (14, 15, 16, 17):
            inner.append(((u - 16) * 0.5, (v - 16) * 0.5, 5.0))
    for u in range(12, 20):
        for v in range(12, 20):
            if 14 <= u <= 17 and 14 <= v <= 17:
                continue
            outer.append(((u - 16) * 0.8, (v - 16) * 0.8, 8.0))
    return np.array(inner + outer, float)


class MultiFrameConfirmationTest(unittest.TestCase):
    def test_three_independent_frames_confirm(self):
        calibration = identity_calibration(32, 32)
        region = inner_region()
        points = confirmation_points()
        engine = Localizer(calibration)
        state = None
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for i in range(3):
                image = tmp / ("frame_%d.png" % i)
                image.write_bytes(b"independent-frame-%d" % i)
                query = write_query_artifacts(tmp / ("run_%d" % i), region)
                record = {"schema": "capture-frame/1", "epoch": "epoch_x", "frame_index": i,
                          "synchronized": True, "time_source": calibration["time_source"],
                          "image": {"file": "image.png", "sha256": build_frame.common.sha256(image),
                                    "width": 32, "height": 32},
                          "cloud": {"file": "cloud.npy", "coordinates": "world", "units": "m",
                                    "points": len(points)},
                          "pose": {"source_frame": calibration["world_frame"],
                                   "target_frame": calibration["camera_frame"],
                                   "R": np.eye(3).tolist(), "translation_m": [0, 0, 0], "units": "m"},
                          "timing": {"stamp_ns": 1000 + i, "source": calibration["time_source"],
                                     "epoch": "epoch_x", "time_kind": "acquisition"}}
                frame = build_frame.from_capture_record(record, image, query, calibration, points=points)
                handoff = engine.process(query, frame)
                self.assertEqual(handoff["confirmation_count"], i + 1)
                state = handoff["state"]
            self.assertEqual(state, "CONFIRMED")


if __name__ == "__main__":
    unittest.main()
