"""Physical-input counterexample: the ground/body height references differ."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'official_adapter')]
from wildos_nav2_adapter.pointcloud import split_costmap_points, local_ground_plane
from beamng_wildos_adapter.mapping import occupancy_from_points

options = dict(min_height=.1, max_height=1.5, filter_footprint=True,
               front_extent=3.4, rear_extent=1.05, half_width=1.05,
               voxel=.1, max_points=10000, ground_plane_radius=8., ground_clearance=.25)
rows = []
for ground_z, object_height in [(0., .4), (-.45, .4), (-.45, .65)]:
    ground = np.array([(x, y, ground_z) for x in np.arange(.125, 7.5, .25)
                       for y in np.arange(-2.875, 3.125, .25)], dtype=float)
    target = np.array([[5.125, .125, ground_z+object_height]])
    points = np.vstack((ground, target))
    plane = local_ground_plane(points, 8.)
    assert plane is not None
    marked, cleared = split_costmap_points(points, **options)
    grid = occupancy_from_points(points, clear_robot_footprint=True)
    target_cell = tuple(np.floor(target[0, :2]/.5+[40, 40]).astype(int))
    retained = any(np.linalg.norm(p-target[0]) < 1e-8 for p in marked)
    clearing_retained = any(np.linalg.norm(p-target[0]) < 1e-8 for p in cleared)
    rows.append(dict(ground_z=ground_z, object_height_above_ground=object_height,
                     object_z_in_body_frame=float(target[0, 2]),
                     fitted_plane=plane.tolist(), marking_points=len(marked),
                     object_retained_as_obstacle=retained, clearing_endpoint_retained=clearing_retained,
                     graph_local_occupancy_value=int(grid[target_cell[1], target_cell[0]]),
                     configured_ground_clearance=options['ground_clearance'],
                     effective_marking_min_above_flat_ground=max(.1-ground_z, .25)))
assert [r['object_retained_as_obstacle'] for r in rows] == [True, False, True]
assert [r['graph_local_occupancy_value'] for r in rows] == [100, 0, 100]
assert rows[1]['clearing_endpoint_retained']
terrain_path = Path(__file__).parent/'stall_connectivity_164222/terrain_review.json'
terrain = json.loads(terrain_path.read_text())
observed_plane = np.asarray(terrain['snapshots'][-1]['body_plane'])
xx, yy = np.meshgrid(np.arange(.125, 7.5, .25), np.arange(-2.875, 3.125, .25))
ground = np.column_stack((xx.ravel(), yy.ravel(), (observed_plane[0]*xx+observed_plane[1]*yy+observed_plane[2]).ravel()))
target_xy = np.array([5.125, .125])
target_ground_z = float(observed_plane[:2] @ target_xy + observed_plane[2])
target = np.array([[*target_xy, target_ground_z+.3]])
marked, cleared = split_costmap_points(np.vstack((ground, target)), **options)
observed_plane_case = dict(ground_plane_from_aggregated_field_terrain=observed_plane.tolist(),
    ground_z_at_synthetic_obstacle=target_ground_z, synthetic_obstacle_height_above_ground=.3,
    synthetic_obstacle_body_z=float(target[0, 2]),
    object_retained_as_obstacle=any(np.linalg.norm(p-target[0]) < 1e-8 for p in marked),
    clearing_endpoint_retained=any(np.linalg.norm(p-target[0]) < 1e-8 for p in cleared),
    actual_obstacle_height_or_identity_proven=False)
assert not observed_plane_case['object_retained_as_obstacle'] and observed_plane_case['clearing_endpoint_retained']
source = ROOT/'nav2_adapter/wildos_nav2_adapter/pointcloud.py'
report = dict(scope=__doc__, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              cases=rows, gap_reproduced=True, production_modified=False,
              synthetic_object_on_observed_plane=observed_plane_case,
              root_in_164222_proven=False,
              limits='Synthetic measured plane and obstacle; demonstrates a contract gap, not the identity/height of the actual field obstruction. min_height is currently body-relative; ground_clearance is plane-relative. Raw LiDAR is absent from this field bag.')
(Path(__file__).parent/'height_reference_gap.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
