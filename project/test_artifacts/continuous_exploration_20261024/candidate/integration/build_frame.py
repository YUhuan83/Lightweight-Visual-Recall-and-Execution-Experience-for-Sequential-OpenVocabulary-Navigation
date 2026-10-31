"""Construct a target-frame/1 from a capture, or refuse with the exact blocking reason.

The capture API records host monotonic request times and states explicitly that it
supplies no synchronized acquisition timestamp. target-frame/1 requires image,
cloud and pose to share one acquisition stamp, so this module refuses to fabricate
one and reports the measured gap instead.
"""
from pathlib import Path

import numpy as np

from . import common

SYNC_TOLERANCE_NS = 1_000_000  # 1 ms


class NotSynchronized(Exception):
    def __init__(self, gap_ns):
        self.gap_ns = gap_ns
        super().__init__("image/cloud acquisition stamps differ by %d ns" % gap_ns)


def synchronization(capture):
    image = common.acquisition_stamp_ns(capture["mounted"])
    cloud = common.acquisition_stamp_ns(capture["lidar"])
    return {"image_stamp_ns": image, "cloud_stamp_ns": cloud, "gap_ns": abs(image - cloud),
            "synchronized": abs(image - cloud) <= SYNC_TOLERANCE_NS}


def timing_block(capture):
    sync = synchronization(capture)
    if sync["synchronized"]:
        return None
    return ("unsynchronized_acquisition: mounted image and lidar differ by %.3f s "
            "(host monotonic); target-frame/1 requires one shared acquisition stamp"
            % (sync["gap_ns"] / 1e9))


def load_cloud(capture):
    points = np.load(capture["paths"]["lidar_npy"], allow_pickle=False)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("lidar.npy must be Nx3, got %r" % (points.shape,))
    if not np.isfinite(points).all():
        raise ValueError("lidar.npy contains non-finite points")
    return points


def _timing(stamp_ns, epoch):
    return {"stamp_ns": int(stamp_ns), "source": common.TIME_SOURCE,
            "epoch": epoch, "time_kind": "acquisition"}


def build_frame(capture, image_path, query, calibration, epoch):
    """Return a target-frame/1 dict. Raises NotSynchronized when stamps cannot be trusted."""
    sync = synchronization(capture)
    if not sync["synchronized"]:
        raise NotSynchronized(sync["gap_ns"])
    return build_frame_from_stamp(capture, image_path, query, calibration,
                                  epoch, sync["image_stamp_ns"])


def build_frame_from_stamp(capture, image_path, query, calibration, epoch, stamp_ns, points=None):
    rotation, translation = common.static_world_to_camera()
    shake = {"accepted": True, "reference": "host monotonic request stamp treated as acquisition"}
    timing = _timing(stamp_ns, epoch)
    digest = common.sha256(image_path)
    if points is None:
        points = load_cloud(capture)
    width = calibration["camera"]["width"]
    height = calibration["camera"]["height"]
    return {
        "schema": "target-frame/1",
        "mode": "oracle_pose",
        "image": {"timing": dict(timing), "frame": calibration["camera_frame"],
                  "width": width, "height": height,
                  "capture_id": digest[:16], "sha256": digest,
                  "provenance": "sensor_measurement"},
        "cloud": {"timing": dict(timing), "frame": calibration["world_frame"],
                  "coordinates": "world", "units": "m",
                  "provenance": "sensor_measurement", "points_xyz_m": points},
        "pose": {"timing": dict(timing), "provenance": "ego_truth",
                 "world_to_camera": {"source_frame": calibration["world_frame"],
                                     "target_frame": calibration["camera_frame"],
                                     "R": rotation.tolist(), "translation_m": translation.tolist(),
                                     "units": "m", "evidence": shake}},
        "now": dict(timing),
        "binding": {"evidence": shake, "image_sha256": digest,
                    "artifact_sha256": query["artifact_sha256"],
                    "query": query["query"], "prefix": query["prefix"]},
    }


def from_capture_record(record, image_path, query, calibration, points=None):
    """Compose a target-frame/1 from a synchronized capture-frame/1 plus a query binding."""
    if record.get("schema") != "capture-frame/1":
        raise ValueError("expected capture-frame/1 record")
    if not record.get("synchronized"):
        raise ValueError("capture not synchronized: " + str(record.get("pose_stability", {})))
    timing = dict(record["timing"])
    if timing["source"] != calibration["time_source"]:
        raise ValueError("capture time source does not match calibration")
    pose = record["pose"]
    if pose["source_frame"] != calibration["world_frame"] or pose["target_frame"] != calibration["camera_frame"]:
        raise ValueError("capture pose frames do not match calibration")
    if (record["image"]["width"], record["image"]["height"]) != (
            calibration["camera"]["width"], calibration["camera"]["height"]):
        raise ValueError("capture image size does not match calibration")
    digest = common.sha256(image_path)
    if digest != record["image"]["sha256"]:
        raise ValueError("capture image digest mismatch")
    if points is None:
        points = np.load(Path(image_path).parent / record["cloud"]["file"], allow_pickle=False)
    shake = {"accepted": True, "reference": "capture-frame/1 epoch " + str(record["epoch"])}
    return {
        "schema": "target-frame/1",
        "mode": "oracle_pose",
        "image": {"timing": dict(timing), "frame": calibration["camera_frame"],
                  "width": calibration["camera"]["width"], "height": calibration["camera"]["height"],
                  "capture_id": digest[:16], "sha256": digest, "provenance": "sensor_measurement"},
        "cloud": {"timing": dict(timing), "frame": calibration["world_frame"],
                  "coordinates": "world", "units": "m", "provenance": "sensor_measurement",
                  "points_xyz_m": points},
        "pose": {"timing": dict(timing), "provenance": "ego_truth",
                 "world_to_camera": {"source_frame": calibration["world_frame"],
                                     "target_frame": calibration["camera_frame"],
                                     "R": pose["R"], "translation_m": pose["translation_m"],
                                     "units": "m", "evidence": shake}},
        "now": dict(timing),
        "binding": {"evidence": shake, "image_sha256": digest,
                    "artifact_sha256": query["artifact_sha256"],
                    "query": query["query"], "prefix": query["prefix"]},
    }
