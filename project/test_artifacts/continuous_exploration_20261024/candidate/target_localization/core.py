"""Validated geometry and temporal evidence. No simulator, GT target or planner IO."""

from dataclasses import dataclass, asdict
import copy

import numpy as np

from .query_io import Invalid, array, fields, number, require, text


@dataclass(frozen=True)
class Policy:
    near_m: float = 0.3
    far_m: float = 60.0
    max_age_ns: int = 500_000_000
    min_returns: int = 6
    min_pixels: int = 6
    min_coverage: float = 0.1
    depth_gap_m: float = 0.6
    max_depth_span_m: float = 1.5
    max_extent_m: float = 4.0
    alpha_silhouette: float = 0.5
    min_ring_pixels: int = 6
    ring_radius_px: int = 2
    max_ring_same_depth_fraction: float = 0.25
    min_mean_cosine: float = 0.1
    proximity_m: float = 1.0
    ttl_ns: int = 1_000_000_000

    def __post_init__(self):
        for key, value in asdict(self).items():
            number(value)
            if key in ("max_age_ns", "min_returns", "min_pixels", "min_ring_pixels", "ring_radius_px", "ttl_ns"):
                require(type(value) is int and value > 0, "Positive integer policy: " + key)
            elif key in ("min_coverage", "alpha_silhouette", "max_ring_same_depth_fraction"):
                require(0 < value < 1, "Policy fraction must be in (0,1): " + key)
            elif key == "min_mean_cosine":
                require(-1 <= value <= 1, "Raw cosine policy out of range")
            else:
                require(value > 0, "Positive metric policy: " + key)
        require(self.near_m < self.far_m, "near must precede far")
        require(self.min_returns >= 3 and self.min_pixels >= 3, "At least three returns/pixels required")


def evidence(value):
    fields(value, "accepted reference")
    require(value["accepted"] is True, "Unresolved calibration/evidence")
    text(value["reference"])


def timing(value):
    fields(value, "stamp_ns source epoch time_kind")
    require(type(value["stamp_ns"]) is int and value["stamp_ns"] >= 0, "Integer acquisition nanoseconds required")
    text(value["source"])
    text(value["epoch"])
    require(value["time_kind"] == "acquisition", "Host/poll/header time is not acquisition time")
    return value["stamp_ns"]


def se3(value, source, target):
    fields(value, "source_frame target_frame R translation_m units evidence")
    require(value["source_frame"] == source and value["target_frame"] == target, "SE3 frame mismatch")
    require(value["units"] == "m", "SE3 translation must be metres")
    evidence(value["evidence"])
    r, t = array(value["R"], (3, 3)), array(value["translation_m"], (3,))
    require(np.allclose(r.T @ r, np.eye(3), atol=1e-9, rtol=0) and
            abs(np.linalg.det(r)-1) <= 1e-9, "SE3 requires proper orthonormal rotation")
    return r, t


def validate_calibration(cal):
    fields(cal, "schema status evidence world_frame camera_frame local_cloud_frame time_source units camera local_to_camera")
    require(cal["schema"] == "target-calibration/1" and cal["status"] == "VERIFIED", "DRAFT calibration unresolved")
    evidence(cal["evidence"])
    for key in ("world_frame", "camera_frame", "local_cloud_frame", "time_source"):
        text(cal[key])
    require(len({cal[k] for k in ("world_frame", "camera_frame", "local_cloud_frame")}) == 3, "Distinct named frames required")
    require(cal["units"] == "m", "Metres required")
    camera = cal["camera"]
    fields(camera, "width height projection pixel_center axes pixels_in_target_optical_frame K D evidence")
    evidence(camera["evidence"])
    require(all(type(camera[k]) is int and camera[k] > 0 for k in ("width", "height")), "Camera dimensions required")
    require(camera["projection"] == "rectilinear_pinhole" and
            camera["pixel_center"] == "integer_centers_half_extent" and
            camera["axes"] == "x_right_y_down_z_forward" and
            camera["pixels_in_target_optical_frame"] is True, "Unknown optical/pixel convention")
    k = array(camera["K"], (3, 3))
    require(k[0, 0] > 0 and k[1, 1] > 0 and k[0, 1] == 0 and k[1, 0] == 0 and
            np.array_equal(k[2], [0, 0, 1]) and
            -0.5 <= k[0, 2] < camera["width"]-0.5 and -0.5 <= k[1, 2] < camera["height"]-0.5,
            "Invalid pinhole K")
    require(np.all(array(camera["D"], (5,)) == 0), "Only validated undistorted pixels supported")
    se3(cal["local_to_camera"], cal["local_cloud_frame"], cal["camera_frame"])
    return k


def validate_frame(frame, cal, query, policy):
    fields(frame, "schema mode image cloud pose now binding")
    require(frame["schema"] == "target-frame/1", "Wrong frame schema")
    require(frame["mode"] in ("estimated_pose", "oracle_pose"), "Explicit pose mode required")
    image, cloud, pose = frame["image"], frame["cloud"], frame["pose"]
    fields(image, "timing frame width height capture_id sha256 provenance")
    fields(cloud, "timing frame coordinates units provenance points_xyz_m")
    fields(pose, "timing provenance world_to_camera")
    stamp = timing(image["timing"])
    for record in (cloud, pose):
        timing(record["timing"])
        require(record["timing"] == image["timing"], "Image/cloud/pose acquisition mismatch")
    now = timing(frame["now"])
    require(all(frame["now"][k] == image["timing"][k] for k in ("source", "epoch", "time_kind")), "Now clock mismatch")
    require(0 <= now-stamp <= policy.max_age_ns, "Stale or future acquisition")
    require(image["timing"]["source"] == cal["time_source"], "Uncalibrated clock source")
    require(image["frame"] == cal["camera_frame"], "Image optical frame mismatch")
    require(all(type(image[k]) is int and image[k] == cal["camera"][k] for k in ("width", "height")), "Image/calibration dimensions mismatch")
    require(query["image_size"] == [image["width"], image["height"]], "Query not in calibrated full resolution pixels")
    text(image["capture_id"])
    require(type(image["sha256"]) is str and len(image["sha256"]) == 64 and
            all(c in "0123456789abcdef" for c in image["sha256"]), "Image SHA256 required")
    require(image["provenance"] == cloud["provenance"] == "sensor_measurement", "Target GT/semantic inputs forbidden")
    expected = "independent_estimate" if frame["mode"] == "estimated_pose" else "ego_truth"
    require(pose["provenance"] == expected, "Pose provenance/mode mismatch; only explicit oracle ego truth allowed")
    r, t = se3(pose["world_to_camera"], cal["world_frame"], cal["camera_frame"])
    require(cloud["units"] == "m", "Cloud metres required")
    require(cloud["coordinates"] in ("world", "sensor_local"), "Explicit cloud coordinates required")
    required_frame = cal["world_frame"] if cloud["coordinates"] == "world" else cal["local_cloud_frame"]
    require(cloud["frame"] == required_frame, "Cloud coordinate/frame mismatch")
    points = array(cloud["points_xyz_m"])
    require(points.ndim == 2 and points.shape[1] == 3, "Cloud must be Nx3, including empty (0,3)")
    binding = frame["binding"]
    fields(binding, "evidence image_sha256 artifact_sha256 query prefix")
    evidence(binding["evidence"])
    require(binding["image_sha256"] == image["sha256"] and binding["query"] == query["query"] and
            binding["prefix"] == query["prefix"] and binding["artifact_sha256"] == query.get("artifact_sha256"),
            "Query/capture/artifact binding mismatch")
    fields(binding["artifact_sha256"], "result cosine mask")
    for digest in binding["artifact_sha256"].values():
        require(type(digest) is str and len(digest) == 64 and all(c in "0123456789abcdef" for c in digest), "Artifact SHA256 required")
    if cloud["coordinates"] == "world":
        world, optical = points, points @ r.T + t
    else:
        lr, lt = se3(cal["local_to_camera"], cal["local_cloud_frame"], cal["camera_frame"])
        optical = points @ lr.T + lt
        world = (optical-t) @ r
    require(np.isfinite(optical).all() and np.isfinite(world).all(), "Transform overflow")
    return world, optical


def localize(query, frame, calibration, policy=None):
    """Consume load_query() output + exact synchronized frame; return non-executable evidence."""
    policy = policy or Policy()
    k = validate_calibration(calibration)
    world, optical = validate_frame(frame, calibration, query, policy)
    h, w = query["mask"].shape
    z = optical[:, 2]
    indices = np.flatnonzero((z >= policy.near_m) & (z <= policy.far_m))
    uv = optical[indices, :2] / z[indices, None]
    uv = uv * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    inside = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= -0.5) & (uv[:, 0] < w-0.5) & (uv[:, 1] >= -0.5) & (uv[:, 1] < h-0.5)
    indices, uv = indices[inside], uv[inside]
    pixels = np.floor(uv+0.5).astype(np.int64)
    # One frontmost return per pixel prevents dense background/duplicate returns winning.
    order = np.argsort(z[indices], kind="stable")
    indices, pixels = indices[order], pixels[order]
    _, first = np.unique(pixels[:, 1]*w+pixels[:, 0], return_index=True)
    indices, pixels = indices[first], pixels[first]
    candidates, rejected = [], []
    for region_id, region in enumerate(query["regions"]):
        mask = region["mask"]
        selected = indices[mask[pixels[:, 1], pixels[:, 0]]]
        reason = None
        if region["mean_cosine"] < policy.min_mean_cosine:
            reason = "weak_raw_cosine"
        elif len(selected) < policy.min_returns:
            reason = "no_supported_depth"
        if reason:
            rejected.append({"region_id": region_id, "reason": reason})
            continue
        selected = selected[np.argsort(z[selected], kind="stable")]
        groups = np.split(selected, np.flatnonzero(np.diff(z[selected]) > policy.depth_gap_m)+1)
        nearest = groups[0]
        # Never skip an unsupported foreground to promote the large background group.
        if len(nearest) < max(policy.min_returns, policy.min_pixels) or len(nearest)/region["area_pixels"] < policy.min_coverage:
            reason = "nearest_depth_unsupported"
        elif np.ptp(z[nearest]) > policy.max_depth_span_m or np.linalg.norm(np.ptp(world[nearest], axis=0)) > policy.max_extent_m:
            reason = "unsafe_cluster_extent"
        ring = np.zeros_like(mask)
        radius = policy.ring_radius_px
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                y0, y1 = max(0, dy), min(h, h+dy)
                x0, x1 = max(0, dx), min(w, w+dx)
                if y0 < y1 and x0 < x1:
                    ring[y0:y1, x0:x1] |= mask[y0-dy:y1-dy, x0-dx:x1-dx]
        ring &= ~query["mask"]
        exterior = indices[ring[pixels[:, 1], pixels[:, 0]]]
        silhouette, same = None, None
        if reason is None:
            if len(exterior) < policy.min_ring_pixels:
                reason = "insufficient_boundary_evidence"
            else:
                center = float(np.median(z[nearest]))
                same = float(np.mean(np.abs(z[exterior]-center) <= policy.depth_gap_m))
                # Deterministic bounded 1-D silhouette against exterior depth evidence.
                a_z = z[nearest][np.linspace(0, len(nearest)-1, min(256, len(nearest)), dtype=int)]
                b_z = np.sort(z[exterior])
                b_z = b_z[np.linspace(0, len(b_z)-1, min(256, len(b_z)), dtype=int)]
                a = np.abs(a_z[:, None]-a_z).sum(axis=1)/(len(a_z)-1)
                b = np.abs(a_z[:, None]-b_z).mean(axis=1)
                silhouette = float(np.mean((b-a)/np.maximum(np.maximum(a, b), np.finfo(float).eps)))
                if same > policy.max_ring_same_depth_fraction or silhouette < policy.alpha_silhouette:
                    reason = "background_or_unseparated_depth"
        if reason:
            rejected.append({"region_id": region_id, "reason": reason})
            continue
        points = world[nearest]
        median = np.median(points, axis=0)
        candidates.append({"region_id": region_id, "position_m": median.tolist(),
                           "spread_mad_m": np.median(np.abs(points-median), axis=0).tolist(),
                           "extent_m": np.ptp(points, axis=0).tolist(), "count": len(points),
                           "unique_pixels": len(points), "depth_median_m": float(np.median(z[nearest])),
                           "silhouette": silhouette, "ring_same_depth_fraction": same,
                           "raw_mean_cosine": region["mean_cosine"], "confidence_probability": None})
    return {"schema": "coarse3d-observation/1", "query": query["query"],
            "timing": copy.deepcopy(frame["image"]["timing"]), "mode": frame["mode"],
            "pose_provenance": frame["pose"]["provenance"], "frame": calibration["world_frame"],
            "capture_id": frame["image"]["capture_id"], "image_sha256": frame["image"]["sha256"],
            "executable": False, "candidates": candidates, "rejected": rejected,
            "abstain": len(candidates) != 1,
            "reason": "one_candidate" if len(candidates) == 1 else "ambiguous_or_no_target"}


class Localizer:
    """Three distinct acquired images, fixed world-space anchor, bounded evidence lifetime."""

    def __init__(self, calibration, policy=None):
        self.calibration = copy.deepcopy(calibration)
        self.policy = policy or Policy()
        self.state = "SEARCHING"
        self.key = None
        self.last_stamp = None
        self.last_seen = None
        self.anchor = None
        self.hits = []
        self.images = set()
        self.captures = set()

    def release(self, state):
        self.state, self.anchor, self.last_seen = state, None, None
        self.hits = []

    def handoff(self, reason, observation=None):
        estimate = None
        if self.state == "CONFIRMED" and self.hits:
            estimate = {**copy.deepcopy(self.hits[-1][1]), "frame": self.key[3],
                        "query": self.key[0], "epoch": self.key[1], "mode": self.key[2],
                        "stamp_ns": self.hits[-1][0], "executable": False,
                        "kind": "coarse_surface_observation_not_object_center"}
        return {"schema": "target-handoff/1", "status": "waiting" if observation is None else "observed",
                "state": self.state, "reason": reason, "unresolved": [reason] if observation is None else [],
                "target_estimate": estimate, "approach_pose": None, "executable": False,
                "path_reachability_validated": False, "ros_publication_allowed": False,
                "confirmation_count": len(self.hits), "observation": observation}

    def tick(self, now):
        """External normalized acquisition clock only; no wall-clock substitution."""
        stamp = timing(now)
        if self.key is not None:
            if (now["epoch"], now["source"]) != (self.key[1], self.key[4]) or (self.last_stamp is not None and stamp < self.last_stamp):
                self.release("LOST")
                return self.handoff("clock_reset_requires_new_stream")
            if self.last_seen is not None and stamp-self.last_seen > self.policy.ttl_ns:
                self.release("LOST")
        self.last_stamp = stamp
        return self.handoff("target_expired" if self.state == "LOST" else "awaiting_acquisition")

    def process(self, query, frame):
        try:
            obs = localize(query, frame, self.calibration, self.policy)
        except (Invalid, TypeError, KeyError, ValueError, OverflowError) as exc:
            self.release("LOST" if self.key is not None else "SEARCHING")
            return self.handoff(str(exc))
        stamp = obs["timing"]["stamp_ns"]
        key = (obs["query"], obs["timing"]["epoch"], obs["mode"], obs["frame"], obs["timing"]["source"])
        if key != self.key:
            self.release("SEARCHING")
            self.last_stamp = None
            self.images.clear()
            self.captures.clear()
            self.key = key
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.release("LOST")
            return self.handoff("duplicate_or_backwards_time")
        self.last_stamp = stamp
        if self.last_seen is not None and frame["now"]["stamp_ns"]-self.last_seen > self.policy.ttl_ns:
            self.release("LOST")
        if obs["image_sha256"] in self.images or obs["capture_id"] in self.captures:
            self.release("LOST")
            return self.handoff("reused_image_not_independent_confirmation")
        self.images.add(obs["image_sha256"])
        self.captures.add(obs["capture_id"])
        if obs["abstain"]:
            # Break tentative consecutive confirmation, retain confirmed estimate only until TTL.
            if self.state == "TENTATIVE":
                self.release("SEARCHING")
            return self.handoff(obs["reason"], obs)
        candidate = obs["candidates"][0]
        position = np.asarray(candidate["position_m"])
        if self.anchor is None or np.linalg.norm(position-self.anchor) > self.policy.proximity_m:
            self.release("TENTATIVE")
            self.anchor = position.copy()
        self.last_seen = stamp
        self.hits.append((stamp, candidate))
        self.hits = [hit for hit in self.hits if stamp-hit[0] <= self.policy.ttl_ns][-3:]
        self.state = "CONFIRMED" if len(self.hits) == 3 else "TENTATIVE"
        return self.handoff("three_frame_evidence" if self.state == "CONFIRMED" else "awaiting_three_frames", obs)
