"""Synthetic fixtures only: these accepted flags are NOT real BeamNG calibration."""

import copy
import io
import json
import math
import unittest
from contextlib import redirect_stdout, redirect_stderr
from decimal import Decimal
from pathlib import Path
from unittest.mock import patch

from beamng.adapters.core import (AcquisitionClock, BackwardTimeError, RigidTransform,
                                 SensorAdapter, ValidationError, camera_intrinsics,
                                 load_calibration, rotate_covariance, seconds_to_ns,
                                 validate_calibration)
from beamng.adapters.replay import main as replay
from beamng.adapters.ros_clock import ClockInput


EVIDENCE = {"accepted": True, "reference": "synthetic unit-test fixture only"}
Q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]


def calibration(kind):
    c = {"version": 1, "kind": kind, "evidence": copy.deepcopy(EVIDENCE),
         "sensor_id": kind+"-test", "input_frame": "explicit_source", "output_frame": "explicit_target",
         "time_source": "acquired_sim_seconds", "time_kind": "acquisition", "input_provenance": "sensor_measurement"}
    if kind == "lidar":
        c.update(coordinates="world", transform_policy="matched_external")
    else:
        c.update(extrinsic_evidence=copy.deepcopy(EVIDENCE), transform={
            "source_frame": c["input_frame"], "target_frame": c["output_frame"],
            "translation_m": [0, 0, 0], "quaternion_xyzw": Q[:]})
    if kind == "imu":
        c.update(colocated_origin_validated=True, angular_unit="rad/s", acceleration_unit="m/s^2",
                 gravity="excluded", smoothing="explicit smoothed fixture")
    if kind == "camera":
        c["camera"] = {"evidence": copy.deepcopy(EVIDENCE), "width": 4, "height": 2,
                       "projection": "rectilinear_pinhole", "pixel_center": "integer_centers_half_extent",
                       "axes": "x_right_y_down_z_forward", "pixels_in_target_optical_frame": True,
                       "hfov_deg": 90, "vfov_deg": 60, "D": [0]*5}
    return c


def sample(kind):
    c = calibration(kind)
    s = {"sensor_id": c["sensor_id"], "frame": c["input_frame"], "provenance": "sensor_measurement",
         "timing": {"seconds": 1.25, "source": c["time_source"], "epoch": "e1", "time_kind": "acquisition"}}
    if kind == "lidar":
        s.update(xyz=[[1, 0, 0], [0, 1, 0]], normals=[[1, 0, 0], [0, 1, 0]], rgba=[0xffabcdef, 0])
    elif kind == "imu":
        s.update(angular_velocity=[1, 0, 0], linear_acceleration=[0, 1, 0],
                 angular_velocity_covariance=[1, 0, 0, 0, 4, 0, 0, 0, 9],
                 linear_acceleration_covariance=[0]*9, orientation_covariance=[-1]*9)
        for key in ("angular_unit", "acceleration_unit", "gravity", "smoothing"):
            s[key] = c[key]
    else:
        s.update(width=4, height=2, encoding="rgb8", data=list(range(24)))
    return s


def matched():
    c = calibration("lidar")
    return {"sensor_id": c["sensor_id"], "epoch": "e1", "stamp_ns": 1250000000,
            "time_source": c["time_source"], "provenance": "independent_estimate", "evidence": copy.deepcopy(EVIDENCE),
            "transform": {"source_frame": c["input_frame"], "target_frame": c["output_frame"],
                          "translation_m": [10, 20, 30], "quaternion_xyzw": Q[:]}}


class TimeTests(unittest.TestCase):
    def test_seconds_exact_rounding_and_carry(self):
        for value, expected in [(0, 0), (1.25, 1250000000), (0.000000001, 1),
                                (Decimal("1.9999999996"), 2000000000),
                                (Decimal("0.0000000005"), 0), (Decimal("0.0000000015"), 2),
                                (Decimal("9223372036.854775807"), 2**63-1)]:
            with self.subTest(value=value):
                self.assertEqual(seconds_to_ns(value), expected)

    def test_invalid_seconds(self):
        for value in (None, True, "1.5", -1, float("nan"), float("inf"), -float("inf"),
                      Decimal("NaN"), Decimal("sNaN"), Decimal("Infinity"), 10**100, Decimal("9223372036.854775808")):
            with self.subTest(value=str(value)), self.assertRaises(ValidationError):
                seconds_to_ns(value)

    def test_pause_resume_backward_latch_epoch(self):
        clock = AcquisitionClock("s", "e1")
        def accept(s, epoch="e1"):
            return clock.accept(s, source="s", epoch=epoch, time_kind="acquisition")
        self.assertEqual(accept(1.25)["nanosec"], 250000000)
        self.assertIsNone(accept(1.25))
        self.assertIsNotNone(accept(2))
        with self.assertRaises(BackwardTimeError):
            accept(1)
        with self.assertRaises(ValidationError):
            accept(3)
        with self.assertRaises(ValidationError):
            clock.begin_epoch("e1")
        clock.begin_epoch("e2")
        self.assertEqual(accept(0, "e2")["ns"], 0)
        with self.assertRaises(ValidationError):
            accept(3)

    def test_subnanosecond_backward_not_hidden_by_rounding(self):
        clock = AcquisitionClock("s", "e")
        clock.accept(Decimal("1.0000000002"), source="s", epoch="e", time_kind="acquisition")
        self.assertIsNone(clock.accept(Decimal("1.0000000003"), source="s", epoch="e", time_kind="acquisition"))
        with self.assertRaises(BackwardTimeError):
            clock.accept(Decimal("1.0000000001"), source="s", epoch="e", time_kind="acquisition")

    def test_source_and_kind_fail_closed(self):
        clock = AcquisitionClock("s", "e")
        for source, epoch, kind in [("wrong", "e", "acquisition"), ("s", "wrong", "acquisition"),
                                    ("s", "e", "polling"), ("s", "e", "header"), ("s", "e", None)]:
            with self.assertRaises(ValidationError):
                clock.accept(1, source=source, epoch=epoch, time_kind=kind)
        self.assertIsNone(clock.last_ns)


class GeometryTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def test_rotation_translation_inverse_composition(self):
        t = RigidTransform("a", "b", [10, 20, 30], Q)
        self.assertVector(t.point([1, 0, 0]), [10, 21, 30])
        self.assertVector(t.rotate([1, 0, 0]), [0, 1, 0])
        self.assertVector(t.inverse().point(t.point([2, -3, 4])), [2, -3, 4])
        self.assertVector(t.then(t.inverse()).point([2, -3, 4]), [2, -3, 4])
        other = RigidTransform("b", "c", [3, -2, 1], [1, 0, 0, 0])
        self.assertVector(t.then(other).point([1, 2, 3]), other.point(t.point([1, 2, 3])))
        with self.assertRaises(ValidationError):
            t.then(t)

    def test_quaternion_sign_and_half_turn(self):
        for q in (Q, [1, 0, 0, 0], [0, 0, 0, 1]):
            t = RigidTransform("a", "b", [0, 0, 0], q)
            negative = RigidTransform("a", "b", [0, 0, 0], [-x for x in q])
            self.assertVector(t.rotate([1, 2, 3]), negative.rotate([1, 2, 3]))
        self.assertVector(RigidTransform("a", "b", [0]*3, [1, 0, 0, 0]).rotate([0, 1, 0]), [0, -1, 0])

    def test_invalid_quaternion_vectors_and_overflow(self):
        for q in ([0, 0, 0, .1], [0]*4, [0, 0, 0, 2], [0, 0, float("nan"), 1], None, [0, 0, 1]):
            with self.subTest(q=q), self.assertRaises(ValidationError):
                RigidTransform("a", "b", [0]*3, q)
        t = RigidTransform("a", "b", [1e308, 0, 0], [0, 0, 0, 1])
        for p in ([1e308, 0, 0], [None, 0, 0], [True, 0, 0], [1, 2]):
            with self.assertRaises(ValidationError):
                t.point(p)

    def test_covariance_rotation_and_markers(self):
        t = RigidTransform("a", "b", [0]*3, Q)
        self.assertVector(rotate_covariance([1, 0, 0, 0, 4, 0, 0, 0, 9], t), [4, 0, 0, 0, 1, 0, 0, 0, 9])
        self.assertVector(rotate_covariance([2, 1, 0, 1, 3, 0, 0, 0, 4], t), [3, -1, 0, -1, 2, 0, 0, 0, 4])
        for marker in ([0]*9, [-1]*9, [-1, 0, 0, 0, 0, 0, 0, 0, 0]):
            self.assertEqual(rotate_covariance(marker, t), tuple(marker))
        for c in ([float("nan")]*9, [-1]+[float("inf")]*8, [1, 2, 0, 0, 1, 0, 0, 0, 1],
                  [1, 2, 0, 2, 1, 0, 0, 0, 1], [1, -.9, -.9, -.9, 1, -.9, -.9, -.9, 1], [1]*8):
            with self.assertRaises(ValidationError):
                rotate_covariance(c, t)


class SensorTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def test_draft_and_evidence_rejected(self):
        with self.assertRaises(ValidationError):
            load_calibration(Path(__file__).with_name("calibration.draft.json"))
        for kind in ("camera", "imu", "lidar"):
            for key in ("sensor_id", "input_frame", "output_frame", "time_source"):
                c = calibration(kind)
                c[key] = None
                with self.assertRaises(ValidationError):
                    SensorAdapter(c, "e1")
            for accepted in (False, None, 1, "true"):
                c = calibration(kind)
                c["evidence"]["accepted"] = accepted
                with self.assertRaises(ValidationError):
                    validate_calibration(c)

    def test_lidar_numeric_transform_and_normals(self):
        adapter = SensorAdapter(calibration("lidar"), "e1")
        out = adapter.adapt(sample("lidar"), matched_transform=matched())
        self.assertVector(out["xyz"][0], [10, 21, 30])
        self.assertVector(out["normals"][0], [0, 1, 0])
        self.assertEqual(out["rgba"], [0xffabcdef, 0])
        self.assertEqual(out["transform_provenance"], "independent_estimate")
        self.assertFalse(out["lio_validated"])
        self.assertFalse(out["deskew_supported"])
        for absent in ("intensity", "ring", "time", "t"):
            self.assertNotIn(absent, out)
        self.assertIsNone(adapter.adapt(sample("lidar"), matched_transform=matched()))

    def test_lidar_matching_and_no_naive_label(self):
        for key, value in [("stamp_ns", 1250000001), ("stamp_ns", 1250000000.0), ("epoch", "e2"),
                           ("sensor_id", "other"), ("time_source", "poll"), ("provenance", None),
                           ("provenance", "static_calibration")]:
            m = matched()
            m[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValidationError):
                SensorAdapter(calibration("lidar"), "e1").adapt(sample("lidar"), matched_transform=m)
        for m in (None, {}, {**matched(), "transform": {**matched()["transform"], "source_frame": "guessed_alias"}}):
            with self.assertRaises(ValidationError):
                SensorAdapter(calibration("lidar"), "e1").adapt(sample("lidar"), matched_transform=m)

    def test_truth_isolation(self):
        m = matched()
        m["provenance"] = "ground_truth"
        with self.assertRaises(ValidationError):
            SensorAdapter(calibration("lidar"), "e1").adapt(sample("lidar"), matched_transform=m)
        out = SensorAdapter(calibration("lidar"), "e1", purpose="debug").adapt(sample("lidar"), matched_transform=m)
        self.assertFalse(out["navigation_allowed"])
        self.assertFalse(out["lio_validated"])
        for kind in ("camera", "lidar", "imu"):
            s = sample(kind)
            s["provenance"] = "ground_truth"
            with self.assertRaises(ValidationError):
                SensorAdapter(calibration(kind), "e1", purpose="debug").adapt(s, matched_transform=matched())

    def test_bad_payload_does_not_consume_stamp(self):
        adapter = SensorAdapter(calibration("lidar"), "e1")
        for key, value in [("xyz", [[float("nan"), 0, 0]]), ("normals", [[0, 0, 0]]),
                           ("rgba", [-1, 0]), ("frame", "explicit_target"), ("timing", None)]:
            s = sample("lidar")
            s[key] = value
            with self.assertRaises(ValidationError):
                adapter.adapt(s, matched_transform=matched())
        self.assertIsNotNone(adapter.adapt(sample("lidar"), matched_transform=matched()))

    def test_imu_rotate_only_and_unknown(self):
        adapter = SensorAdapter(calibration("imu"), "e1")
        out = adapter.adapt(sample("imu"))
        self.assertVector(out["angular_velocity"], [0, 1, 0])
        self.assertVector(out["linear_acceleration"], [-1, 0, 0])
        self.assertVector(out["angular_velocity_covariance"], [4, 0, 0, 0, 1, 0, 0, 0, 9])
        self.assertEqual(out["linear_acceleration_covariance"], (0,)*9)
        self.assertEqual(out["orientation_covariance"], (-1,)*9)
        self.assertIsNone(out["orientation"])
        s = sample("imu")
        s["angular_velocity_covariance"] = [-1]*9
        self.assertEqual(SensorAdapter(calibration("imu"), "e1").adapt(s)["angular_velocity_covariance"], (-1,)*9)

    def test_imu_reject_lever_arm_and_semantic_invention(self):
        c = calibration("imu")
        c["transform"]["translation_m"] = [1e-15, 0, 0]
        with self.assertRaises(ValidationError):
            SensorAdapter(c, "e1")
        c = calibration("imu")
        c["colocated_origin_validated"] = False
        with self.assertRaises(ValidationError):
            SensorAdapter(c, "e1")
        for key, value in [("gravity", "included"), ("angular_unit", "deg/s"), ("smoothing", "raw"),
                           ("orientation_covariance", [0]*9), ("linear_acceleration_covariance", None)]:
            s = sample("imu")
            s[key] = value
            with self.assertRaises(ValidationError):
                SensorAdapter(calibration("imu"), "e1").adapt(s)

    def test_camera_projection_pixel_centers_stamp(self):
        c = calibration("camera")
        info = camera_intrinsics(c["camera"])
        self.assertAlmostEqual(info["K"][0], 2)
        self.assertAlmostEqual(info["K"][4], math.sqrt(3))
        self.assertEqual(info["K"][2], 1.5)
        self.assertEqual(info["K"][5], .5)
        # Horizontal FOV reaches sensor extent at -0.5 and W-0.5, not centers 0,W-1.
        self.assertAlmostEqual(info["K"][0]*math.tan(math.radians(45))+info["K"][2], 3.5)
        out = SensorAdapter(c, "e1").adapt(sample("camera"))
        self.assertEqual(out["stamp"], out["camera_info"]["stamp"])
        self.assertEqual(out["frame"], out["camera_info"]["frame"])
        self.assertEqual(out["data"], list(range(24)))

    def test_camera_reject_unvalidated_projection_and_depth(self):
        for key, value in [("hfov_deg", 180), ("vfov_deg", 0), ("hfov_deg", float("nan")),
                           ("D", None), ("D", [1, 0, 0, 0, 0]), ("pixel_center", "guess"),
                           ("pixels_in_target_optical_frame", False), ("width", True), ("projection", "fisheye")]:
            c = calibration("camera")
            c["camera"][key] = value
            with self.assertRaises(ValidationError):
                SensorAdapter(c, "e1")
        for key, value in [("width", 5), ("encoding", "8UC1"), ("data", [0]), ("provenance", "semantic_labels")]:
            s = sample("camera")
            s[key] = value
            with self.assertRaises(ValidationError):
                SensorAdapter(calibration("camera"), "e1").adapt(s)

    def test_every_sensor_acquisition_and_epoch(self):
        for kind in ("camera", "imu", "lidar"):
            a = SensorAdapter(calibration(kind), "e1")
            for field, value in [("time_kind", "polling"), ("seconds", None), ("source", "TimeSensor")]:
                s = sample(kind)
                s["timing"][field] = value
                with self.assertRaises(ValidationError):
                    a.adapt(s, matched_transform=matched())
            self.assertIsNotNone(a.adapt(sample(kind), matched_transform=matched()))
            a.begin_epoch("e2")
            with self.assertRaises(ValidationError):
                a.adapt(sample(kind), matched_transform=matched())
            s = sample(kind)
            s["timing"].update(epoch="e2", seconds=0)
            m = matched()
            m.update(epoch="e2", stamp_ns=0)
            self.assertEqual(a.adapt(s, matched_transform=m)["stamp"]["ns"], 0)


class EntryTests(unittest.TestCase):
    def test_clock_gate_configuration(self):
        for topic in (None, "relative", "/clock", "/vehicles/car/sensors/timer", "/a//b", "/bad-name"):
            with self.assertRaises(ValidationError):
                ClockInput(topic, "s", "e", normalized_acquisition_confirmed=True)
        with self.assertRaises(ValidationError):
            ClockInput("/normalized/time", "s", "e", normalized_acquisition_confirmed=False)

    def test_clock_gate_fail_stop(self):
        for invalid in (0, float("nan"), None, 2**31):
            gate = ClockInput("/normalized/time", "s", "e", normalized_acquisition_confirmed=True)
            self.assertEqual(gate.receive(1.25)["nanosec"], 250000000)
            self.assertIsNone(gate.receive(1.25))
            with self.assertRaises(ValidationError):
                gate.receive(invalid)
            self.assertTrue(gate.stopped)
            with self.assertRaises(ValidationError):
                gate.receive(2)

    def test_clock_cli_fixture(self):
        output = io.StringIO()
        with redirect_stdout(output):
            status = replay([str(Path(__file__).with_name("clock.example.jsonl")), "--epoch", "run-1", "--clock-source", "offline_fixture"])
        self.assertEqual(status, 0)
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertTrue(rows[2]["dropped"])
        self.assertTrue(rows[3]["result"]["flush_consumers"])
        self.assertEqual(rows[4]["result"]["ns"], 0)

    def test_cli_bad_input_stops_and_draft_rejected(self):
        for data in ('{"seconds":null}\n', 'not json\n', '[]\n'):
            with patch("sys.stdin", io.StringIO(data)), redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
                self.assertEqual(replay(["-", "--epoch", "e", "--clock-source", "s"]), 2)
        with redirect_stderr(io.StringIO()):
            self.assertEqual(replay(["-", "--epoch", "e", "--calibration", str(Path(__file__).with_name("calibration.draft.json"))]), 2)

    def test_sensor_cli(self):
        data = json.dumps({"sample": sample("lidar"), "matched_transform": matched()})+"\n"
        output = io.StringIO()
        with patch("beamng.adapters.replay.load_calibration", return_value=calibration("lidar")), \
                patch("sys.stdin", io.StringIO(data)), redirect_stdout(output):
            self.assertEqual(replay(["-", "--epoch", "e1", "--calibration", "synthetic"]), 0)
        self.assertEqual(json.loads(output.getvalue())["result"]["frame"], "explicit_target")


if __name__ == "__main__":
    unittest.main()
