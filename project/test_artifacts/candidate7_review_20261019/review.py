"""Preserve candidate7 inputs and independently check its geometry contracts."""
from pathlib import Path
import hashlib
import json
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / 'nav2_adapter'), str(ROOT / 'official_adapter')]
from wildos_nav2_adapter.pointcloud import split_costmap_points
from beamng_wildos_adapter.terrain import witnessed_step_by_cell

PATHS = [
    'nav2_adapter/wildos_nav2_adapter/pointcloud.py',
    'nav2_adapter/wildos_nav2_adapter/cloud_filter.py',
    'nav2_adapter/launch/nav2_official_control.launch.py',
    'official_adapter/beamng_wildos_adapter/terrain.py',
    'official_adapter/beamng_wildos_adapter/mapping.py',
    'official_adapter/beamng_wildos_adapter/node.py',
    'official_adapter/test/test_terrain.py',
    'nav2_adapter/test/test_pointcloud.py',
    'docs/PASSABILITY_LOW_SURFACE_REPAIR_20261019.md',
    'deploy/20261019_passability_repair_candidate7_harness/run_frame_candidate7.py',
]


def main():
    stage = sys.argv[1]
    assert stage in ('before', 'after', 'curved_after')
    dest = OUT / stage
    dest.mkdir(exist_ok=False)
    hashes = {}
    for name in PATHS:
        target = dest / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
        hashes[name] = hashlib.sha256(target.read_bytes()).hexdigest()
    xy = np.array([(x, y) for x in np.arange(.03, 7.5, .1)
                   for y in np.arange(-2.97, 3., .1)])
    convex = np.c_[xy, .015 * xy[:, 0] ** 2 - .5]
    marks, _ = split_costmap_points(
        convex, min_height=.1, max_height=1.5, filter_footprint=False,
        front_extent=3.4, rear_extent=1.05, half_width=1.05, voxel=.1,
        max_points=20000, ground_plane_radius=8., obstacle_min_height=.1)
    # Uneven within-bin positions: continuous plane, not a physical step.
    xy = np.array([(x, y) for x in np.arange(4.001, 5.99, .049)
                   for y in np.arange(-2.999, 2.99, .049)])
    plane = np.c_[xy, -.35 * xy[:, 0] + 2.]
    report = dict(
        scope=__doc__, source_sha256=hashes,
        continuous_convex=dict(points=len(convex), marked=len(marks),
                               true_vertical_step_m=0., maximum_grade=.225),
        continuous_plane=dict(maximum_grade=.35, true_vertical_step_m=0.,
                              reported_step_m=float(witnessed_step_by_cell(plane).max())),
        complete_navigation_passed=False,
        terrain_asset_identity_validated=False,
    )
    (dest / 'review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps({k: v for k, v in report.items() if k != 'source_sha256'}, indent=2))


if __name__ == '__main__':
    main()
