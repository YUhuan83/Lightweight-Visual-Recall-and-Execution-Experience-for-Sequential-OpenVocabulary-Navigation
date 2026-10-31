"""Candidate-node graph construction on the local ground plane.

Nodes are sampled on a polar grid around the ego: radii x headings, expressed in
the ego frame (forward / left). World = BeamNG world XY (metres, Z ignored here).
"""
import math

import numpy as np

DEFAULT_RADII = (3.0, 6.0, 9.0, 12.0, 16.0)
DEFAULT_HEADINGS_DEG = tuple(range(-90, 91, 15))


def unit_xy(vector):
    v = np.asarray(vector, float).reshape(2)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("zero forward vector")
    return v / n


def left_of(forward_xy):
    f = unit_xy(forward_xy)
    return np.array([-f[1], f[0]])


def bearing_deg(node_xy, position_xy, forward_xy):
    """Signed bearing of the node relative to ego forward, degrees in [-180,180]."""
    d = np.asarray(node_xy, float) - np.asarray(position_xy, float)
    f = unit_xy(forward_xy)
    l = left_of(f)
    return math.degrees(math.atan2(float(d @ l), float(d @ f)))


def build_candidates(position_xy, forward_xy, radii=DEFAULT_RADII, headings_deg=DEFAULT_HEADINGS_DEG):
    """Return a list of node dicts with world XY and ego-frame bearing."""
    pos = np.asarray(position_xy, float)
    f = unit_xy(forward_xy)
    l = left_of(f)
    nodes = []
    for r in radii:
        for h in headings_deg:
            a = math.radians(h)
            xy = pos + r * (math.cos(a) * f + math.sin(a) * l)
            nodes.append({"id": "r%.0f_h%d" % (r, h), "radius_m": float(r),
                          "heading_deg": float(h), "xy": xy.tolist()})
    return nodes


def _point_segment_distance(point, a, b):
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.clip((point - a) @ ab / denom, 0.0, 1.0))
    return float(np.linalg.norm(point - (a + t * ab)))


def segment_clear(start_xy, goal_xy, obstacle_xy, obstacle_radius=1.2):
    """True if no obstacle point lies within obstacle_radius of the segment."""
    a = np.asarray(start_xy, float)
    b = np.asarray(goal_xy, float)
    if obstacle_xy is None or len(obstacle_xy) == 0:
        return True
    pts = np.asarray(obstacle_xy, float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("obstacle_xy must be Nx2")
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        dist = np.linalg.norm(pts - a, axis=1)
    else:
        t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
        closest = a + t[:, None] * ab
        dist = np.linalg.norm(pts - closest, axis=1)
    return bool(np.min(dist) > obstacle_radius)


def reachability(start_xy, nodes, obstacle_xy, obstacle_radius=1.2):
    """Assign rconf 1.0/0.0 per node from straight-segment clearance."""
    for node in nodes:
        node["rconf"] = 1.0 if segment_clear(start_xy, node["xy"], obstacle_xy, obstacle_radius) else 0.0
    return nodes


def frontier_conf(node_xy, all_xy, radius=2.0, cap=20):
    """Proxy frontier score: fewer LiDAR returns near the node -> more unknown."""
    if all_xy is None or len(all_xy) == 0:
        return 1.0
    pts = np.asarray(all_xy, float)
    hits = int((np.linalg.norm(pts - np.asarray(node_xy, float), axis=1) < radius).sum())
    return float(np.clip(1.0 - hits / float(cap), 0.0, 1.0))
