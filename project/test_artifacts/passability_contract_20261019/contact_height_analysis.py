"""Read-only contact-neighborhood height analysis from the frozen replay.

No ROS, vehicle or production code is executed. The goal is to separate the
ground surface from returns that were dropped by the fixed base-frame
min_height and the road-relative ground_clearance.
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def main():
    import sys
    sys.path.insert(0, str(ROOT / 'nav2_adapter'))
    from wildos_nav2_adapter.pointcloud import (filter_robot_footprint,
                                                local_ground_plane)
    from wildos_nav2_adapter.vehicle_geometry import (SBR_FRONT_EXTENT,
                                                      SBR_HALF_WIDTH,
                                                      SBR_OBSTACLE_MAX_HEIGHT,
                                                      SBR_REAR_EXTENT)

    arrays = np.load(OUT / 'replay_inputs.npz', allow_pickle=False)
    manifest = json.loads((OUT / 'input_manifest.json').read_text(encoding='utf8'))
    rows = []
    thresholds = (0.03, 0.05, 0.08, 0.10, 0.15, 0.25)
    for sample in manifest['snapshots']:
        key = sample['key']
        points = arrays[key + '_body'].astype(np.float64)
        contact = arrays[key + '_contact_body'].astype(np.float64)
        plane = local_ground_plane(points, 8.0)
        near = np.hypot(points[:, 0] - contact[0], points[:, 1] - contact[1]) <= 0.6
        selected = points[near]
        if plane is not None:
            residual = selected[:, 2] - (selected[:, 0] * plane[0]
                                         + selected[:, 1] * plane[1] + plane[2])
            contact_residual = contact[2] - (contact[0] * plane[0]
                                             + contact[1] * plane[1] + plane[2])
        else:
            residual = np.full(len(selected), np.nan)
            contact_residual = float('nan')
        external = filter_robot_footprint(points, SBR_FRONT_EXTENT,
                                          SBR_REAR_EXTENT, SBR_HALF_WIDTH)
        external = external[external[:, 2] <= SBR_OBSTACLE_MAX_HEIGHT]
        external_near = (np.hypot(external[:, 0] - contact[0],
                                  external[:, 1] - contact[1]) <= 0.6)
        cells = {}
        for resolution in (0.1, 0.2, 0.5):
            keys = np.floor(selected[:, :2] / resolution).astype(np.int64)
            unique, inverse = np.unique(keys, axis=0, return_inverse=True)
            counts = np.bincount(inverse, minlength=len(unique))
            mins = np.full(len(unique), np.inf)
            maxs = np.full(len(unique), -np.inf)
            np.minimum.at(mins, inverse, residual)
            np.maximum.at(maxs, inverse, residual)
            span = maxs - mins
            cells[str(resolution)] = {
                'cells': int(len(unique)),
                'multi_return_cells': int((counts >= 2).sum()),
                'max_span': float(np.nanmax(span)) if len(span) else None,
                'cells_with_span_gt_0.05': int(np.nansum(span > 0.05)),
                'cells_with_span_gt_0.10': int(np.nansum(span > 0.10)),
            }
        rows.append({
            'key': key,
            'engine_seconds': sample['engine_seconds'],
            'points_0.6m': int(len(selected)),
            'plane_found': plane is not None,
            'plane': None if plane is None else [float(v) for v in plane],
            'contact_height_above_plane': float(contact_residual),
            'residual_min': None if not len(residual) else float(np.nanmin(residual)),
            'residual_p50': None if not len(residual) else float(np.nanmedian(residual)),
            'residual_max': None if not len(residual) else float(np.nanmax(residual)),
            'above_threshold': {str(t): int(np.nansum(residual >= t)) for t in thresholds},
            'external_points_0.6m': int(external_near.sum()),
            'cells': cells,
        })
    report = {
        'scope': __doc__,
        'contact_world': manifest['contact_world'],
        'source_db_sha256': manifest['database']['sha256'],
        'rows': rows,
        'production_modified': False,
        'complete_navigation_passed': False,
    }
    (OUT / 'contact_height_analysis.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
