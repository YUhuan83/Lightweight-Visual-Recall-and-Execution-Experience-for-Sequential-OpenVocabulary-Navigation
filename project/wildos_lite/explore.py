"""Frontier exploration selection faithful to the WildOS paper.

The paper (ExploRFM / GoalAgnosticScoring) does not steer at the single frontier
column: it divides heading space into discrete angular bins, builds a per-pixel
confidence for each heading (goal focus), a long-range frontier confidence, and a
geometric reachability field, then picks the pixel/heading that maximises the
normalised ADDITIVE combination

    score = (goal_w*goal_conf + frontier_w*frontier_conf + reach_w*reach_conf)
            / (goal_w + frontier_w + reach_w)

This lite module reproduces that scoring on the single mounted camera (no ROS2
navgraph): frontier regions become the multi-source MCP seeds, and the returned
bearing is relative to the camera forward axis (0 = straight ahead).

Paper defaults (geofrontier_nav_conf): frontier_threshold=0.6,
traversability_threshold=0.8, frontier_w=2.0, goal_w=3.0,
reachability_w=2.0, ADDITIVE, reach_scale=0.25.
"""
import math

import numpy as np

FRONTIER_THRESHOLD = 0.6
TRAVERSABILITY_THRESHOLD = 0.8
FRONTIER_W = 2.0
GOAL_W = 3.0
REACHABILITY_W = 2.0
REACH_SCALE = 0.25


def angular_headings(num_bins):
    """Unit XY headings evenly around the compass; heading 0 is forward (+z)."""
    angles = np.linspace(0.0, 2.0 * math.pi, num_bins, endpoint=False)
    return np.stack([np.cos(angles), np.sin(angles)], axis=-1)


def frontier_conf(frontier, traversability=None, frontier_threshold=FRONTIER_THRESHOLD,
                  traversability_threshold=TRAVERSABILITY_THRESHOLD, opening_kernel=0):
    """Long-range frontier confidence in the paper sense: threshold out weak
    frontier responses and non-traversable pixels, optionally morphologically
    open to drop isolated specks."""
    f = np.asarray(frontier, dtype=float)
    conf = f.copy()
    conf[conf < frontier_threshold] = 0.0
    if traversability is not None:
        t = np.asarray(traversability, dtype=float)
        if t.shape == conf.shape:
            conf[t < traversability_threshold] = 0.0
    if opening_kernel > 1:
        import cv2
        valid = cv2.morphologyEx((conf > 0).astype(np.uint8),
                                 cv2.MORPH_OPEN,
                                 np.ones((opening_kernel, opening_kernel), np.uint8))
        conf[valid == 0] = 0.0
    return conf


def camera_ray_direction(width):
    """Direction angles toward the centre of each horizontal column, radians in
    [-fov/2, +fov/2] assuming an axis-aligned pinhole (ignores v)."""
    if width <= 1:
        return np.array([0.0])
    cols = np.arange(width) - (width - 1) / 2.0
    fx = (width / 2.0) / math.tan(math.pi / 4.0)
    return np.arctan2(cols, fx)


def goal_heading_conf(frontier_shape, headings):
    """Per-heading alignment of every image column with each discrete heading,
    normalised to [0, 1] as in the paper's (1 + dot)/2. Returns (H, W, B)."""
    height, width = frontier_shape
    ray = camera_ray_direction(width)  # (W,)
    goal = np.empty((height, width, headings.shape[0]), dtype=np.float32)
    for b, h in enumerate(headings):
        align = (0.5 + 0.5 * np.cos(ray - math.atan2(h[1], h[0])))
        goal[:, :, b] = align
    return goal


def reachability_from_frontiers(frontier_conf, traversability, reach_scale=REACH_SCALE,
                                traversability_threshold=TRAVERSABILITY_THRESHOLD):
    """Geometric reachability of every pixel from the frontier regions.

    Mirrors the paper: the traversal cost is 1/traversability on a rescaled grid,
    frontier pixels are the multi-source seeds, and the scalar cost field is
    normalised to reachability = 1 - tanh(cost / ((h+w)*reach_scale)).
    """
    h, w = frontier_conf.shape
    if h < 2 or w < 2:
        return np.ones((h, w), dtype=np.float32)
    trav = np.asarray(traversability, dtype=float).copy()
    trav[~np.isfinite(trav)] = 0.0
    trav = np.clip(trav, 0.0, 1.0)

    small_h = max(2, int(round(h * reach_scale)))
    small_w = max(2, int(round(w * reach_scale)))
    import cv2
    trav_small = cv2.resize(trav, (small_w, small_h), interpolation=cv2.INTER_LINEAR)
    blocked = trav_small < traversability_threshold
    cost = 1.0 / (trav_small + 1e-3)
    cost[blocked] = 20.0

    seeds = cv2.resize((frontier_conf > 0).astype(np.uint8), (small_w, small_h),
                       interpolation=cv2.INTER_NEAREST)
    seed_idx = np.argwhere(seeds > 0)
    if len(seed_idx) == 0:
        return np.ones((h, w), dtype=np.float32)

    from scipy.sparse.csgraph import shortest_path
    from scipy.sparse import csr_matrix
    n = small_h * small_w
    rows, cols, data = [], [], []
    for y in range(small_h):
        for x in range(small_w):
            idx = y * small_w + x
            nbrs = []
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < small_h and 0 <= nx < small_w:
                    nbrs.append((ny * small_w + nx, 0.5 * (cost[y, x] + cost[ny, nx])))
            for nbr, c in nbrs:
                rows.append(idx)
                cols.append(nbr)
                data.append(c)
    graph = csr_matrix((data, (rows, cols)), shape=(n, n))
    sources = seed_idx[:, 0] * small_w + seed_idx[:, 1]
    # Multi-source shortest path from all frontier pixels.
    dist = shortest_path(graph, directed=False, indices=sources, method="D")
    dmin = np.min(dist, axis=0).reshape(small_h, small_w)
    dmin = cv2.resize(dmin, (w, h), interpolation=cv2.INTER_LINEAR)
    reach = 1.0 - np.tanh(np.clip(dmin, 0.0, 10.0) / ((h + w) * reach_scale))
    return reach.astype(np.float32)


def best_exploration_heading(frontier_conf, traversability=None, num_bins=16,
                             goal_w=GOAL_W, frontier_w=FRONTIER_W,
                             reachability_w=REACHABILITY_W):
    """Return (heading_radians, best_score, per_bin_scores) maximising the
    WildOS additive frontier-scoring objective."""
    fconf = frontier_conf
    if fconf is None or fconf.size == 0:
        return 0.0, 0.0, None
    fconf = np.asarray(fconf, dtype=float)
    if not np.any(fconf > 0.0):
        return 0.0, 0.0, [0.0] * num_bins
    headings = angular_headings(num_bins)
    goal = goal_heading_conf(fconf.shape, headings)
    reach = reachability_from_frontiers(fconf, traversability) if traversability is not None \
        else np.ones_like(fconf, dtype=np.float32)

    score = (goal_w * goal + frontier_w * fconf[..., None]
             + reachability_w * reach[..., None])
    score /= (goal_w + frontier_w + reachability_w)

    flat = score.reshape(-1, num_bins)
    per_bin = flat.max(axis=0)
    best_bin = int(np.argmax(per_bin))
    # Relative bearing: winning heading projected back to the camera forward frame.
    heading = math.atan2(headings[best_bin, 1], headings[best_bin, 0])
    return heading, float(per_bin[best_bin]), per_bin.tolist()