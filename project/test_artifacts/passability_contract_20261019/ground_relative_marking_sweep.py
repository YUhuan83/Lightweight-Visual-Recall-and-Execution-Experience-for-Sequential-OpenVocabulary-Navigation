"""Read-only sweep: what would ground-relative obstacle marking retain?

Compares the current base-frame min_height=0.1 plus road-relative
ground_clearance=0.25 against candidate ground-relative thresholds. No
production module is modified; the current occupancy builder is used as the
baseline.
"""
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
RESOLUTION = 0.5
WIDTH = HEIGHT = 80


def cells_for(points, origin):
    ix = np.floor(points[:, 0] / RESOLUTION + WIDTH / 2).astype(np.int64)
    iy = np.floor(points[:, 1] / RESOLUTION + HEIGHT / 2).astype(np.int64)
    inside = (ix >= 0) & (ix < WIDTH) & (iy >= 0) & (iy < HEIGHT)
    return {(int(a), int(b)) for a, b in zip(ix[inside], iy[inside])}


def main():
    import sys
    sys.path.insert(0, str(ROOT / 'nav2_adapter'))
    sys.path.insert(0, str(ROOT / 'official_adapter'))
    from wildos_nav2_adapter.pointcloud import (filter_robot_footprint,
                                                local_ground_plane,
                                                voxel_downsample)
    from wildos_nav2_adapter.vehicle_geometry import (SBR_FRONT_EXTENT,
                                                      SBR_HALF_WIDTH,
                                                      SBR_OBSTACLE_MAX_HEIGHT,
                                                      SBR_REAR_EXTENT)
    from beamng_wildos_adapter.mapping import occupancy_from_points

    arrays = np.load(OUT / 'replay_inputs.npz', allow_pickle=False)
    manifest = json.loads((OUT / 'input_manifest.json').read_text(encoding='utf8'))
    thresholds = (0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25)
    rows = []
    for sample in manifest['snapshots']:
        key = sample['key']
        raw = arrays[key + '_body'].astype(np.float64)
        contact = arrays[key + '_contact_body'].astype(np.float64)
        baseline = occupancy_from_points(raw).ravel()
        baseline_cells = {(i % WIDTH, i // WIDTH)
                          for i, value in enumerate(baseline) if value >= 50}
        external = filter_robot_footprint(raw, SBR_FRONT_EXTENT,
                                          SBR_REAR_EXTENT, SBR_HALF_WIDTH)
        external = external[external[:, 2] <= SBR_OBSTACLE_MAX_HEIGHT]
        plane = local_ground_plane(raw, 8.0)
        if plane is None:
            residual = np.full(len(external), np.nan)
        else:
            residual = external[:, 2] - (external[:, 0] * plane[0]
                                         + external[:, 1] * plane[1] + plane[2])
        contact_cell = (math.floor(contact[0] / RESOLUTION + WIDTH / 2),
                        math.floor(contact[1] / RESOLUTION + HEIGHT / 2))
        sweep = {}
        for threshold in thresholds:
            selected = external[residual >= threshold]
            selected = voxel_downsample(selected, 0.2, 8000)
            marked = cells_for(selected, None)
            sweep[str(threshold)] = {
                'points': int(len(selected)),
                'cells': len(marked),
                'new_vs_baseline': len(marked - baseline_cells),
                'contact_cell': contact_cell in marked,
                'near_contact_cells':
                    sum(1 for x, y in marked
                        if abs(x - contact_cell[0]) <= 2
                        and abs(y - contact_cell[1]) <= 2),
            }
        rows.append({
            'key': key,
            'engine_seconds': sample['engine_seconds'],
            'plane_found': plane is not None,
            'baseline_cells': len(baseline_cells),
            'contact_cell': list(contact_cell),
            'sweep': sweep,
        })
    report = {
        'scope': __doc__,
        'source_db_sha256': manifest['database']['sha256'],
        'points_are_body_frame': True,
        'contact_world': manifest['contact_world'],
        'rows': rows,
        'production_modified': False,
        'complete_navigation_passed': False,
    }
    (OUT / 'ground_relative_marking_sweep.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
