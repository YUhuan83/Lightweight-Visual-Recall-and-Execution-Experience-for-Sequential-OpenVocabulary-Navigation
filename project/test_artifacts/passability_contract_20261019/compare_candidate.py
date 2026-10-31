"""Closed-record before/after map controls. No vehicle or deployment approval."""
from pathlib import Path
import hashlib, importlib.util, json, sys
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'official_adapter')]
from wildos_nav2_adapter import pointcloud as current
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH,SBR_OBSTACLE_MAX_HEIGHT
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET
from beamng_wildos_adapter import mapping,terrain

frozen=ROOT/'test_artifacts/recall_contract_repair_20261019/before/nav2_adapter/wildos_nav2_adapter/pointcloud.py'
spec=importlib.util.spec_from_file_location('before_pointcloud',frozen)
before=importlib.util.module_from_spec(spec);spec.loader.exec_module(before)
manifest=json.loads((OUT/'input_manifest.json').read_text(encoding='utf8'))
archive=OUT/'replay_inputs.npz'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive_sha256']
arrays=np.load(archive,allow_pickle=False)
options=dict(min_height=.1,max_height=SBR_OBSTACLE_MAX_HEIGHT,filter_footprint=True,
             front_extent=SBR_FRONT_EXTENT,rear_extent=SBR_REAR_EXTENT,half_width=SBR_HALF_WIDTH,
             voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
rows=[]
for snapshot in manifest['snapshots']:
    key=snapshot['key'];body=arrays[key+'_body'];level=arrays[key+'_level']
    oldmarks,oldclear=before.split_costmap_points(body,**options)
    newmarks,newclear=current.split_costmap_points(body,**options)
    assert np.array_equal(oldclear,newclear),key
    with patch.object(mapping,'split_costmap_points',before.split_costmap_points),patch.object(terrain,'local_ground_plane',before.local_ground_plane):
        oldlayers=terrain.terrain_layers(level,sensor_origin_xy=arrays[key+'_sensor_level'][:2])
    newlayers=terrain.terrain_layers(level,sensor_origin_xy=arrays[key+'_sensor_level'][:2])
    recorded={name:arrays[key+'_recorded_'+name] for name in oldlayers}
    near=lambda p:int((np.linalg.norm(p[:,:2]-arrays[key+'_contact_body'][:2],axis=1)<=.6).sum())
    rows.append(dict(key=key,engine_seconds=snapshot['engine_seconds'],markings_before=len(oldmarks),markings_after=len(newmarks),
        contact_neighborhood_markings_before=near(oldmarks),contact_neighborhood_markings_after=near(newmarks),
        baseline_recorded_cell_differences={name:int(np.count_nonzero(~np.isclose(oldlayers[name],recorded[name],atol=1e-5,rtol=1e-6))) for name in oldlayers},
        candidate_cell_differences={name:int(np.count_nonzero(~np.isclose(newlayers[name],oldlayers[name],atol=1e-5,rtol=1e-6))) for name in oldlayers}))
report=dict(scope=__doc__,old_pointcloud_sha256=hashlib.sha256(frozen.read_bytes()).hexdigest(),
    new_pointcloud_sha256=hashlib.sha256(Path(current.__file__).read_bytes()).hexdigest(),
    input_archive_sha256=manifest['archive_sha256'],snapshots=rows,clearing_unchanged=True,
    complete_navigation_passed=False,low_surface_loss_fixed=False,field_acceptance=False)
(OUT/'candidate_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(snapshots=len(rows),old_markings=sum(r['markings_before'] for r in rows),
    new_markings=sum(r['markings_after'] for r in rows),first=rows[0],
    recorded_differences={name:sum(r['baseline_recorded_cell_differences'][name] for r in rows) for name in oldlayers}),indent=2))
