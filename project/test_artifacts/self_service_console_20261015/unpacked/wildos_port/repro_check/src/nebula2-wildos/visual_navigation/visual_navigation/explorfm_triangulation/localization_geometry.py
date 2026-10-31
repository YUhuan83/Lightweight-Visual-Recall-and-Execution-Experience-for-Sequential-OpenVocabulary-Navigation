"""Deterministic foreground depth and multi-view bearing checks, without ROS."""
import math
import numpy as np


def foreground_depth(points_cam, intrinsic, mask, dense_minimum=20, sparse_minimum=8):
    """Return a depth proposal with explicit surface-association validity.

    Project in camera optical coordinates. Per-pixel nearest returns suppress
    occluded background; depth gaps separate foreground/background surfaces.
    Geometry support does not establish which surface belongs to the semantic
    mask. Multiple supported surfaces remain ambiguous: callers must not turn
    the nearest proposal into a confirmed object or arrival evidence.
    """
    points = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    k, mask = np.asarray(intrinsic, dtype=float), np.asarray(mask, dtype=bool)
    if (k.shape != (3, 3) or mask.ndim != 2 or not np.any(mask)
            or not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0):
        return None
    points = points[np.all(np.isfinite(points), axis=1) & (points[:, 2] > .1)]
    if not len(points):
        return None
    pixels = points @ k.T
    pixels = pixels[:, :2] / pixels[:, 2:]
    h, w = mask.shape
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < w)
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < h))
    points, pixels = points[inside], pixels[inside].astype(int)
    if not len(points):
        return None
    inside = mask[pixels[:, 1], pixels[:, 0]]
    points, pixels = points[inside], pixels[inside]
    # Each projected pixel contributes once, independent of return density.
    order = np.argsort(points[:, 2])
    _, unique = np.unique(pixels[order, 1] * w + pixels[order, 0], return_index=True)
    selected = order[unique]
    points, pixels = points[selected], pixels[selected]
    order = np.argsort(points[:, 2])
    points, pixels = points[order], pixels[order]
    if len(points) < sparse_minimum:
        return None
    gaps = np.diff(points[:, 2])
    cuts = np.flatnonzero(gaps > np.maximum(.6, .04 * points[:-1, 2])) + 1
    clusters = np.split(np.arange(len(points)), cuts)
    # Dense surfaces have stronger absolute support than sparse ones. Their
    # share of all returns can change as a broad semantic mask includes more
    # distant background, even at the same pose. Keep the stricter fraction
    # for sparse evidence and require at least one fifth for dense surfaces.
    eligible = [c for c in clusters if len(c) >= sparse_minimum
                and len(c) / len(points) >= (.2 if len(c) >= dense_minimum else .25)]
    if not eligible:
        return None
    candidates = [_surface_quality(points, pixels, cluster, dense_minimum, sparse_minimum)
                  for cluster in eligible]
    candidates = sorted((c for c in candidates if c is not None),
                        key=lambda c: c['position'][2])
    if not candidates:
        return None
    proposal = dict(candidates[0])
    proposal['association_validated'] = len(candidates) == 1
    proposal['association_status'] = ('single_supported_surface' if len(candidates) == 1
                                      else 'ambiguous_depth_surfaces')
    # Bounded by the support gate (at most five candidates). Keep lightweight
    # geometry, not images or point clouds, for diagnostics and new viewpoints.
    proposal['depth_candidates'] = [dict(position=c['position'].tolist(),
        confidence=c['confidence'], points=c['points'], support=c['support'],
        depth_sigma=c['depth_sigma']) for c in candidates]
    return proposal


def _surface_quality(points, pixels, cluster, dense_minimum, sparse_minimum):
    """Validate every surface before comparing association alternatives."""
    chosen, projected = points[cluster], pixels[cluster]
    centre = np.median(chosen, axis=0)
    mad = np.median(np.abs(chosen[:, 2] - centre[2]))
    coherent = np.abs(chosen[:, 2] - centre[2]) <= max(.5, 3 * 1.4826 * mad)
    chosen, projected = chosen[coherent], projected[coherent]
    if len(chosen) < sparse_minimum or np.ptp(projected[:, 0]) < 2 or np.ptp(projected[:, 1]) < 2:
        return None
    centre = np.median(chosen, axis=0)
    depth_sigma = float(max(.1, 1.4826 * np.median(np.abs(chosen[:, 2] - centre[2]))))
    if depth_sigma > max(1.5, .08 * centre[2]):
        return None
    support = len(chosen) / len(points)
    # Support already gates eligibility. Background return density must not
    # veto a coherent foreground that passed that gate (oblique facades can
    # have fewer returns than distant ground within a broad semantic mask).
    confidence = min(.95, (.9 if len(chosen) >= dense_minimum else .65)
                     * (.65 + .35 * min(1., support / .6)))
    if confidence < .5:
        return None
    centred = chosen - centre
    _, singular, axes = np.linalg.svd(centred, full_matrices=False)
    normal = axes[-1]
    return dict(position=centre, confidence=confidence, points=int(len(chosen)),
                support=float(support), depth_sigma=depth_sigma,
                plane_normal=normal,
                plane_residual=float(np.sqrt(np.mean((centred @ normal) ** 2))),
                plane_span=float(singular[1] / math.sqrt(len(chosen))),
                covariance=np.diag(np.maximum(np.var(chosen, axis=0) / len(chosen), .01)))


def ground_like_surface(quality, world_rotation):
    """Exclude a supported horizontal road patch from elevated-object depth."""
    normal = np.asarray(world_rotation) @ np.asarray(quality['plane_normal'])
    camera_relative_world = np.asarray(world_rotation) @ np.asarray(quality['position'])
    # A sparse scan may hit only a narrow strip of road within a broad mask.
    # Small span cannot turn that below-camera horizontal strip into evidence
    # for an elevated traffic signal; retain bearing guidance instead.
    return bool(abs(normal[2]) >= .95 and quality['plane_residual'] <= .15
                and camera_relative_world[2] < -.3)


def triangulate_bearings(views, minimum_baseline=1., minimum_angle=.035):
    """Least-squares intersection with parallax, reprojection and conditioning gates."""
    if len(views) < 2:
        return None
    origins = np.asarray([v['origin'] for v in views], float)
    rays = np.asarray([v['ray'] for v in views], float)
    if not np.all(np.isfinite(origins)) or not np.all(np.isfinite(rays)):
        return None
    norms = np.linalg.norm(rays, axis=1)
    if np.any(norms < 1e-6):
        return None
    rays = rays / norms[:, None]
    pairs = [(i, j) for i in range(len(views)) for j in range(i)
             if np.linalg.norm(origins[i] - origins[j]) >= minimum_baseline
             and math.acos(float(np.clip(rays[i] @ rays[j], -1., 1.))) >= minimum_angle]
    if not pairs:
        return None
    matrices = np.eye(3)[None, :, :] - rays[:, :, None] * rays[:, None, :]
    matrix = np.sum(matrices, axis=0)
    if np.linalg.cond(matrix) > 4000:
        return None
    position = np.linalg.solve(matrix, np.sum(np.einsum('nij,nj->ni', matrices, origins), axis=0))
    vectors = position - origins
    depths = np.sum(vectors * rays, axis=1)
    residual = np.linalg.norm(np.einsum('nij,nj->ni', matrices, vectors), axis=1)
    errors = np.arctan2(residual, depths)
    limits = np.asarray([v.get('angular_tolerance', .035) for v in views])
    if (np.any(depths <= 0) or np.any(depths > 100.) or np.any(errors > limits)
            or np.median(residual) > 1.5):
        return None
    covariance = np.linalg.inv(matrix) * max(.01, float(np.mean(residual**2)))
    radius = float(np.sqrt(np.max(np.linalg.eigvalsh(covariance[:2, :2]))))
    if radius > 3.:
        return None
    return dict(position=position, confidence=.65, covariance=covariance,
                association_validated=True, association_status='validated_bearings',
                radius=radius, parallax=max(math.acos(float(np.clip(rays[i] @ rays[j], -1., 1.)))
                                          for i, j in pairs))
