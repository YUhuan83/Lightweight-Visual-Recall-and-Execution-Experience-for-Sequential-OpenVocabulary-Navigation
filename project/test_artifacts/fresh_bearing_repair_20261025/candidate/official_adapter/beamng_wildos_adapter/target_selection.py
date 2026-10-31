"""Small deterministic policies for selecting stable semantic target clusters."""
import math

import numpy as np


def required_cluster_support(confirmation_frames):
    """Return the minimum stable inlier count for the configured frame window."""
    return max(3, math.ceil(.8 * int(confirmation_frames)))


def supported_position_clusters(values, radius, min_support):
    """Build robust 2D localization clusters with the required support count."""
    values = np.asarray(values, dtype=float)
    if (values.ndim != 2 or values.shape[0] == 0 or values.shape[1] < 2
            or not math.isfinite(radius) or radius <= 0 or min_support < 1):
        return []

    clusters = []
    for seed in values:
        inliers = np.linalg.norm(values[:, :2] - seed[:2], axis=1) <= 2 * radius
        if not np.any(inliers):
            continue
        centre = np.median(values[inliers], axis=0)
        inliers = np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= radius
        if not np.any(inliers):
            continue
        centre = np.median(values[inliers], axis=0)
        inliers &= np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= radius
        count = int(np.count_nonzero(inliers))
        if count >= min_support:
            clusters.append((count, centre, inliers))
    return clusters


def select_target_cluster(clusters, origin=None, prefer_nearest=False):
    """Choose a supported cluster, optionally by distance from the query origin.

    Each cluster is ``(support_count, center, inliers)``. Callers are responsible
    for filtering out clusters that do not meet their temporal support threshold.
    """
    if not clusters:
        return None
    if prefer_nearest and origin is not None:
        return min(clusters, key=lambda cluster: (
            math.hypot(float(cluster[1][0]) - float(origin[0]),
                       float(cluster[1][1]) - float(origin[1])),
            -int(cluster[0])))
    return max(clusters, key=lambda cluster: cluster[0])
