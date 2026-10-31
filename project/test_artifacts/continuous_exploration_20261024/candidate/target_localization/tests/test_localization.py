"""SYNTHETIC ONLY: analytic geometry, fake hashes and timestamps for unit tests."""

import copy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import numpy as np

from target_localization import Invalid, Localizer, Policy, localize
from target_localization.__main__ import run
from target_localization.query_io import associate, load_query, read_json, read_mask_png, sha256


EVIDENCE = {"accepted": True, "reference": "SYNTHETIC analytical fixture; NOT BeamNG calibration"}


def transform(source, target, r=None, t=None):
    return {"source_frame": source, "target_frame": target, "R": np.eye(3).tolist() if r is None else r,
            "translation_m": [0, 0, 0] if t is None else t, "units": "m", "evidence": copy.deepcopy(EVIDENCE)}


def fixture(stamp=100_000_000):
    dense = np.zeros((20, 20), dtype=np.float32)
    dense[7:12, 7:12] = 0.7
    mask = dense > 0.1
    report = {"status": "completed", "inference_ran": True, "image_size": [20, 20],
              "input_size": [10, 10], "threshold": 0.1,
              "box_convention": "xyxy, exclusive right/bottom, EXIF-oriented original image pixels",
              "results": [{"query": "red barrel", "prefix": "query_000", "raw_min": float(dense.min()),
                           "raw_max": float(dense.max()), "regions": [
                               {"box_xyxy": [7, 7, 12, 12], "area_pixels": 25,
                                "mean_cosine": float(dense[mask].mean()), "max_cosine": float(dense.max())}]}]}
    query = associate(report, "query_000", dense, mask)
    query["artifact_sha256"] = {"result": "a"*64, "cosine": "b"*64, "mask": "c"*64}
    cal = {"schema": "target-calibration/1", "status": "VERIFIED", "evidence": copy.deepcopy(EVIDENCE),
           "world_frame": "synthetic_world", "camera_frame": "synthetic_optical", "local_cloud_frame": "synthetic_lidar",
           "time_source": "synthetic_acquisition", "units": "m",
           "camera": {"width": 20, "height": 20, "projection": "rectilinear_pinhole",
                      "pixel_center": "integer_centers_half_extent", "axes": "x_right_y_down_z_forward",
                      "pixels_in_target_optical_frame": True, "K": [[20, 0, 9.5], [0, 20, 9.5], [0, 0, 1]],
                      "D": [0]*5, "evidence": copy.deepcopy(EVIDENCE)},
           "local_to_camera": transform("synthetic_lidar", "synthetic_optical")}
    points = []
    for y in range(5, 14):
        for x in range(5, 14):
            z = 5 if mask[y, x] else 10
            points.append([(x-9.5)*z/20, (y-9.5)*z/20, z])
    clock = {"stamp_ns": stamp, "source": "synthetic_acquisition", "epoch": "synthetic-1", "time_kind": "acquisition"}
    digest = hashlib.sha256(str(stamp).encode()).hexdigest()
    frame = {"schema": "target-frame/1", "mode": "estimated_pose",
             "image": {"timing": copy.deepcopy(clock), "frame": "synthetic_optical", "width": 20, "height": 20,
                       "capture_id": "synthetic-"+str(stamp), "sha256": digest, "provenance": "sensor_measurement"},
             "cloud": {"timing": copy.deepcopy(clock), "frame": "synthetic_world", "coordinates": "world",
                       "units": "m", "provenance": "sensor_measurement", "points_xyz_m": points},
             "pose": {"timing": copy.deepcopy(clock), "provenance": "independent_estimate",
                      "world_to_camera": transform("synthetic_world", "synthetic_optical")},
             "now": copy.deepcopy(clock), "binding": {"evidence": copy.deepcopy(EVIDENCE), "image_sha256": digest,
                      "artifact_sha256": copy.deepcopy(query["artifact_sha256"]), "query": query["query"], "prefix": query["prefix"]}}
    return query, frame, cal, report


def png(mask, mode=0):
    pixels = (mask.astype(np.uint8)*255)
    h, w = pixels.shape
    raw = bytearray()
    for y in range(h):
        raw.append(mode)
        for x in range(w):
            a = int(pixels[y, x-1]) if x else 0
            b = int(pixels[y-1, x]) if y else 0
            c = int(pixels[y-1, x-1]) if x and y else 0
            p = a+b-c
            distances = [abs(p-a), abs(p-b), abs(p-c)]
            predictor = [0, a, b, (a+b)//2, [a, b, c][distances.index(min(distances))]][mode]
            raw.append((int(pixels[y, x])-predictor) & 255)

    def chunk(kind, body):
        return struct.pack(">I", len(body))+kind+body+struct.pack(">I", zlib.crc32(kind+body) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


class GeometryTests(unittest.TestCase):
    def test_coarse_observation_not_goal(self):
        q, f, c, _ = fixture()
        out = localize(q, f, c)
        self.assertFalse(out["abstain"])
        candidate = out["candidates"][0]
        self.assertEqual(candidate["count"], 25)
        np.testing.assert_allclose(candidate["position_m"], [-0.125, -0.125, 5])
        self.assertIsNone(candidate["confidence_probability"])
        self.assertFalse(out["executable"])

    def test_massive_background_cannot_dominate(self):
        q, f, c, _ = fixture()
        background = [[(x-9.5)*15/20, (y-9.5)*15/20, 15] for y in range(7, 12) for x in range(7, 12)]
        f["cloud"]["points_xyz_m"] += background*1000
        out = localize(q, f, c)
        self.assertEqual(out["candidates"][0]["depth_median_m"], 5)
        self.assertEqual(out["candidates"][0]["count"], 25)

    def test_sparse_foreground_never_promotes_background(self):
        q, f, c, _ = fixture()
        points = np.asarray(f["cloud"]["points_xyz_m"])
        chosen = np.flatnonzero(points[:, 2] == 5)
        points[chosen[2:]] *= 3
        f["cloud"]["points_xyz_m"] = points
        out = localize(q, f, c)
        self.assertTrue(out["abstain"])
        self.assertEqual(out["rejected"][0]["reason"], "nearest_depth_unsupported")

    def test_background_only_false_positive_rejected(self):
        q, f, c, _ = fixture()
        p = np.asarray(f["cloud"]["points_xyz_m"])
        p *= (10/p[:, 2])[:, None]
        f["cloud"]["points_xyz_m"] = p
        self.assertTrue(localize(q, f, c)["abstain"])

    def test_empty_depth(self):
        q, f, c, _ = fixture()
        f["cloud"]["points_xyz_m"] = np.empty((0, 3))
        self.assertTrue(localize(q, f, c)["abstain"])

    def test_no_boundary_evidence_abstains(self):
        q, f, c, _ = fixture()
        f["cloud"]["points_xyz_m"] = [p for p in f["cloud"]["points_xyz_m"] if p[2] == 5]
        self.assertTrue(localize(q, f, c)["abstain"])

    def test_near_far_and_behind_camera(self):
        q, f, c, _ = fixture()
        f["cloud"]["points_xyz_m"] += [[0, 0, -1], [0, 0, 0], [0, 0, 0.1], [0, 0, 61], [1000, 0, 5]]
        self.assertEqual(localize(q, f, c)["candidates"][0]["count"], 25)

    def test_world_rotation_translation(self):
        q, f, c, _ = fixture()
        r = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        t = np.array([3, -2, 1])
        optical = np.asarray(f["cloud"]["points_xyz_m"])
        f["cloud"]["points_xyz_m"] = (optical-t) @ r
        f["pose"]["world_to_camera"] = transform("synthetic_world", "synthetic_optical", r.tolist(), t.tolist())
        out = localize(q, f, c)
        np.testing.assert_allclose(out["candidates"][0]["position_m"], (np.array([-0.125, -0.125, 5])-t) @ r)

    def test_local_cloud_nonidentity_extrinsics(self):
        q, f, c, _ = fixture()
        expected = localize(q, f, c)["candidates"][0]["position_m"]
        r = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        t = np.array([1, 2, 3])
        f["cloud"].update(coordinates="sensor_local", frame="synthetic_lidar",
                          points_xyz_m=(np.asarray(f["cloud"]["points_xyz_m"])-t) @ r)
        c["local_to_camera"] = transform("synthetic_lidar", "synthetic_optical", r.tolist(), t.tolist())
        np.testing.assert_allclose(localize(q, f, c)["candidates"][0]["position_m"], expected)

    def test_huge_unsafe_cluster(self):
        q, f, c, _ = fixture()
        points = np.asarray(f["cloud"]["points_xyz_m"])*5
        f["cloud"]["points_xyz_m"] = points
        self.assertEqual(localize(q, f, c)["rejected"][0]["reason"], "unsafe_cluster_extent")

    def test_duplicate_returns_not_support(self):
        q, f, c, _ = fixture()
        f["cloud"]["points_xyz_m"] = [[0, 0, 5]]*100
        self.assertTrue(localize(q, f, c)["abstain"])

    def test_no_semantic_candidate_abstains(self):
        q, f, c, report = fixture()
        dense = np.zeros((20, 20), dtype=np.float32)
        report["results"][0].update(raw_max=0, regions=[])
        empty = associate(report, "query_000", dense, dense > 0.1)
        empty["artifact_sha256"] = q["artifact_sha256"]
        self.assertTrue(localize(empty, f, c)["abstain"])

    def test_multiple_valid_regions_abstain(self):
        q, f, c, report = fixture()
        dense = np.zeros((20, 20), dtype=np.float32)
        dense[4:7, 4:7] = 0.7
        dense[12:15, 12:15] = 0.7
        report["results"][0]["regions"] = [
            {"box_xyxy": [v, v, v+3, v+3], "area_pixels": 9,
             "mean_cosine": float(dense[v:v+3, v:v+3].mean()), "max_cosine": float(dense.max())}
            for v in (4, 12)]
        ambiguous = associate(report, "query_000", dense, dense > 0.1)
        ambiguous["artifact_sha256"] = q["artifact_sha256"]
        f["cloud"]["points_xyz_m"] = [
            [(x-9.5)*z/20, (y-9.5)*z/20, z]
            for y in range(20) for x in range(20)
            for z in [5 if dense[y, x] > 0.1 else 10]]
        out = localize(ambiguous, f, c)
        self.assertEqual(len(out["candidates"]), 2)
        self.assertTrue(out["abstain"])

    def test_alpha_evidence_validation(self):
        for kwargs in ({"alpha_silhouette": 0}, {"alpha_silhouette": float("nan")},
                       {"min_pixels": True}, {"near_m": 60}, {"max_age_ns": 1.5}, {"min_returns": 1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(Invalid):
                Policy(**kwargs)


class ContractTests(unittest.TestCase):
    def test_unknown_calibration_waits(self):
        q, f, c, _ = fixture()
        c["status"] = "DRAFT"
        out = Localizer(c).process(q, f)
        self.assertEqual(out["status"], "waiting")
        self.assertIsNone(out["target_estimate"])

    def test_stamps_epochs_source_stale(self):
        for key, value in (("stamp_ns", 123), ("epoch", "other"), ("source", "host"), ("time_kind", "poll")):
            q, f, c, _ = fixture()
            f["cloud"]["timing"][key] = value
            with self.subTest(key=key), self.assertRaises(Invalid):
                localize(q, f, c)
        q, f, c, _ = fixture()
        f["now"]["stamp_ns"] += 500_000_001
        with self.assertRaises(Invalid):
            localize(q, f, c)

    def test_exact_rows_and_nonfinite_rejected(self):
        for points in ([[1, 2]], [[1, 2, 3, 4]], [[1, 2, True]], [[1, 2, "3"]],
                       [[1, 2, float("inf")]], [[1, 2, 3], [4]], np.array([[1, 2, np.nan]])):
            q, f, c, _ = fixture()
            f["cloud"]["points_xyz_m"] = points
            with self.subTest(points=points), self.assertRaises(Invalid):
                localize(q, f, c)

    def test_rotation_no_relative_tolerance(self):
        for r in ([[1.000001, 0, 0], [0, 1, 0], [0, 0, 1]], [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]):
            q, f, c, _ = fixture()
            f["pose"]["world_to_camera"]["R"] = r
            with self.assertRaises(Invalid):
                localize(q, f, c)

    def test_frame_units_pixels_distortion(self):
        for section, key, value in (("cloud", "units", "cm"), ("cloud", "frame", "renamed"),
                                    ("image", "width", True), ("pose", "provenance", "static_calibration")):
            q, f, c, _ = fixture()
            f[section][key] = value
            with self.assertRaises(Invalid):
                localize(q, f, c)
        for key, value in (("pixel_center", "corners"), ("D", [1, 0, 0, 0, 0]), ("K", [[20, 1, 9.5], [0, 20, 9.5], [0, 0, 1]])):
            q, f, c, _ = fixture()
            c["camera"][key] = value
            with self.assertRaises(Invalid):
                localize(q, f, c)

    def test_only_explicit_oracle_ego_pose(self):
        q, f, c, _ = fixture()
        f["pose"]["provenance"] = "ego_truth"
        with self.assertRaises(Invalid):
            localize(q, f, c)
        f["mode"] = "oracle_pose"
        self.assertFalse(localize(q, f, c)["executable"])
        f["cloud"]["provenance"] = "target_ground_truth"
        with self.assertRaises(Invalid):
            localize(q, f, c)

    def test_target_gt_extra_fields_rejected(self):
        q, f, c, _ = fixture()
        f["pose"]["target_ground_truth"] = [1, 2, 3]
        with self.assertRaises(Invalid):
            localize(q, f, c)

    def test_binding_mismatch(self):
        q, f, c, _ = fixture()
        f["binding"]["artifact_sha256"]["cosine"] = "d"*64
        with self.assertRaises(Invalid):
            localize(q, f, c)


class ConfirmationTests(unittest.TestCase):
    def engine(self):
        return Localizer(fixture()[2])

    def process(self, engine, stamp, change=None):
        q, f, _, _ = fixture(stamp)
        if change:
            change(q, f)
        return engine.process(q, f)

    def test_three_distinct_timestamps(self):
        engine = self.engine()
        for i, state in enumerate(("TENTATIVE", "TENTATIVE", "CONFIRMED"), 1):
            out = self.process(engine, i*100_000_000)
            self.assertEqual(out["state"], state)
            self.assertEqual(out["confirmation_count"], i)
        self.assertIsNotNone(out["target_estimate"])
        self.assertIsNone(out["approach_pose"])
        self.assertFalse(out["ros_publication_allowed"])
        self.assertFalse(out["target_estimate"]["executable"])

    def test_duplicate_and_backwards_reset(self):
        for stamp in (200_000_000, 100_000_000):
            engine = self.engine()
            self.process(engine, 200_000_000)
            out = self.process(engine, stamp)
            self.assertEqual(out["state"], "LOST")
            self.assertEqual(out["confirmation_count"], 0)

    def test_same_image_cannot_confirm(self):
        engine = self.engine()
        self.process(engine, 100_000_000)
        def reuse(q, f):
            f["image"]["sha256"] = fixture()[1]["image"]["sha256"]
            f["binding"]["image_sha256"] = f["image"]["sha256"]
        self.assertEqual(self.process(engine, 200_000_000, reuse)["state"], "LOST")

    def test_class_mixing_resets(self):
        engine = self.engine()
        self.process(engine, 100_000_000)
        self.process(engine, 200_000_000)
        def other(q, f):
            q["query"] = f["binding"]["query"] = "tree"
        out = self.process(engine, 300_000_000, other)
        self.assertEqual(out["confirmation_count"], 1)

    def test_epoch_reset(self):
        engine = self.engine()
        self.process(engine, 100_000_000)
        def reset(q, f):
            for key in ("image", "cloud", "pose"):
                f[key]["timing"]["epoch"] = "synthetic-2"
            f["now"]["epoch"] = "synthetic-2"
        self.assertEqual(self.process(engine, 50_000_000, reset)["confirmation_count"], 1)

    def test_same_pixels_moving_camera_not_same_object(self):
        engine = self.engine()
        self.process(engine, 100_000_000)
        def move(q, f):
            points = np.asarray(f["cloud"]["points_xyz_m"])
            points[:, 0] += 10
            f["cloud"]["points_xyz_m"] = points
            f["pose"]["world_to_camera"]["translation_m"] = [-10, 0, 0]
        self.assertEqual(self.process(engine, 200_000_000, move)["confirmation_count"], 1)
        self.assertEqual(self.process(engine, 300_000_000)["confirmation_count"], 1)

    def test_fixed_world_target_camera_motion_confirms(self):
        engine = self.engine()
        for i in range(1, 4):
            def move(q, f):
                # Compensated local sensor coordinates preserve physical world observation.
                f["cloud"].update(coordinates="sensor_local", frame="synthetic_lidar")
                shift = np.array([0.01*i, 0, 0])
                f["cloud"]["points_xyz_m"] = np.asarray(f["cloud"]["points_xyz_m"])+shift
                f["pose"]["world_to_camera"]["translation_m"] = shift.tolist()
            out = self.process(engine, i*100_000_000, move)
        self.assertEqual(out["state"], "CONFIRMED")

    def test_ttl_releases_confirmed_and_tick(self):
        engine = self.engine()
        for i in range(1, 4):
            self.process(engine, i*100_000_000)
        now = fixture(1_300_000_001)[1]["now"]
        out = engine.tick(now)
        self.assertEqual(out["state"], "LOST")
        self.assertIsNone(out["target_estimate"])

    def test_confirmation_window_cannot_accumulate_forever(self):
        engine = self.engine()
        for stamp in (100_000_000, 900_000_000, 1_700_000_000):
            out = self.process(engine, stamp)
        self.assertEqual(out["state"], "TENTATIVE")

    def test_invalid_frame_revokes_estimate(self):
        engine = self.engine()
        for i in range(1, 4):
            self.process(engine, i*100_000_000)
        def bad(q, f):
            f["cloud"]["timing"]["stamp_ns"] = 0
        out = self.process(engine, 400_000_000, bad)
        self.assertIsNone(out["target_estimate"])
        self.assertEqual(out["status"], "waiting")

    def test_missing_depth_ttl_releases(self):
        engine = self.engine()
        for i in range(1, 4):
            self.process(engine, i*100_000_000)
        def missing(q, f):
            f["cloud"]["points_xyz_m"] = np.empty((0, 3))
        self.assertEqual(self.process(engine, 400_000_000, missing)["state"], "CONFIRMED")
        out = self.process(engine, 1_300_000_001, missing)
        self.assertEqual(out["state"], "LOST")
        self.assertIsNone(out["target_estimate"])

    def test_tick_backward_releases(self):
        engine = self.engine()
        self.process(engine, 200_000_000)
        self.assertEqual(engine.tick(fixture()[1]["now"])["state"], "LOST")


class QueryIOTests(unittest.TestCase):
    def test_all_png_filters_and_real_output_loader(self):
        q, _, _, report = fixture()
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            (root / "result.json").write_text(json.dumps(report), encoding="utf-8")
            np.save(root / "query_000_cosine.npy", q["dense"], allow_pickle=False)
            for mode in range(5):
                path = root / "query_000_region_mask.png"
                path.write_bytes(png(q["mask"], mode))
                np.testing.assert_array_equal(read_mask_png(path), q["mask"])
                loaded = load_query(root / "result.json", "query_000")
                self.assertEqual(loaded["regions"][0]["area_pixels"], 25)

    def test_nan_duplicate_overflow_json(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            path = Path(directory)/"bad.json"
            for raw in ('{"x":NaN}', '{"x":Infinity}', '{"x":1e999}', '{"x":1,"x":2}'):
                path.write_text(raw, encoding="utf-8")
                with self.assertRaises(Invalid):
                    read_json(path)

    def test_resized_map_rejected(self):
        q, _, _, report = fixture()
        with self.assertRaises(Invalid):
            associate(report, "query_000", q["dense"][:10, :10], q["mask"][:10, :10])

    def test_partial_mask_and_wrong_region_rejected(self):
        q, _, _, report = fixture()
        bad = q["mask"].copy()
        bad[7, 7] = False
        with self.assertRaises(Invalid):
            associate(report, "query_000", q["dense"], bad)
        report["results"][0]["regions"][0]["box_xyxy"] = [0, 0, 5, 5]
        with self.assertRaises(Invalid):
            associate(report, "query_000", q["dense"], q["mask"])

    def test_eight_connected_and_sorted_association(self):
        _, _, _, report = fixture()
        dense = np.zeros((20, 20), np.float32)
        dense[1, 1] = dense[2, 2] = 0.2
        dense[15:17, 15:17] = 0.9
        report["results"][0].update(raw_max=float(dense.max()), regions=[
            {"box_xyxy": [15, 15, 17, 17], "area_pixels": 4, "mean_cosine": float(dense[15, 15]), "max_cosine": float(dense[15, 15])},
            {"box_xyxy": [1, 1, 3, 3], "area_pixels": 2, "mean_cosine": float(dense[1, 1]), "max_cosine": float(dense[1, 1])}])
        q = associate(report, "query_000", dense, dense > 0.1)
        self.assertTrue(q["regions"][0]["mask"][15, 15])
        self.assertFalse(q["regions"][0]["mask"][1, 1])
        self.assertEqual(int(q["regions"][1]["mask"].sum()), 2)

    def test_draft_cli_waiting_no_observation(self):
        path = Path(__file__).parents[1]/"manifest.draft.json"
        out = run(path)[0]
        self.assertEqual(out["status"], "waiting")
        self.assertTrue(out["unresolved"])
        self.assertIsNone(out["observation"])

    def test_cli_synthetic_single_frame_integration(self):
        q, f, c, report = fixture()
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            def save(name, value):
                (root/name).write_text(json.dumps(value), encoding="utf-8")
            save("result.json", report)
            np.save(root/"query_000_cosine.npy", q["dense"], allow_pickle=False)
            (root/"query_000_region_mask.png").write_bytes(png(q["mask"]))
            (root/"synthetic_image.bin").write_bytes(b"SYNTHETIC fixture, not an actual camera image")
            loaded = load_query(root/"result.json", "query_000")
            f["binding"]["artifact_sha256"] = loaded["artifact_sha256"]
            f["image"]["sha256"] = f["binding"]["image_sha256"] = sha256(root/"synthetic_image.bin")
            np.save(root/"cloud.npy", f["cloud"]["points_xyz_m"], allow_pickle=False)
            f["cloud"]["points_xyz_m"] = None
            save("frame.json", f)
            save("calibration.json", c)
            save("manifest.json", {"schema": "target-offline-manifest/1", "status": "VERIFIED", "unresolved": [],
                                   "calibration": "calibration.json", "policy": {}, "frames": [
                                       {"query_result": "result.json", "prefix": "query_000", "image": "synthetic_image.bin",
                                        "cloud": "cloud.npy", "frame": "frame.json"}]})
            out = run(root/"manifest.json")[0]
            self.assertEqual(out["state"], "TENTATIVE")
            self.assertIsNone(out["target_estimate"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
