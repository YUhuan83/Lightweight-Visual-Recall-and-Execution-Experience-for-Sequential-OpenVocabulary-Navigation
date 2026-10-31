"""Inspect closed contact clouds; geometry evidence, not navigation acceptance."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'official_adapter')]
from wildos_nav2_adapter.pointcloud import local_ground_plane, filter_robot_footprint
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH

manifest = json.loads((OUT/'input_manifest.json').read_text(encoding='utf8'))
path = OUT/'replay_inputs.npz'
assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest['archive_sha256']
arrays = np.load(path, allow_pickle=False)
rows = []
for snapshot in manifest['snapshots']:
    key = snapshot['key']
    points = arrays[key+'_body']
    external = filter_robot_footprint(points, SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH)
    plane = local_ground_plane(external, 8.)
    relative = external - np.asarray(SBR_LIDAR_OFFSET)
    pitch = np.degrees(np.arctan2(relative[:, 2], np.linalg.norm(relative[:, :2], axis=1)))
    contact = arrays[key+'_contact_body']
    patch = np.linalg.norm(external[:, :2]-contact[:2], axis=1) <= .6
    corridor = ((external[:, 0] >= SBR_FRONT_EXTENT) & (external[:, 0] <= SBR_FRONT_EXTENT+2.) &
                (np.abs(external[:, 1]) <= SBR_HALF_WIDTH))
    local = external[patch]
    ground = None if plane is None else external[:, :2]@plane[:2]+plane[2]
    rows.append(dict(key=key, engine_seconds=snapshot['engine_seconds'], points=len(points),
        plane=None if plane is None else plane.tolist(), contact_body=contact.tolist(),
        return_pitch_degrees=np.percentile(pitch, [0, 1, 50, 99, 100]).tolist(),
        contact_neighborhood_count=len(local),
        contact_neighborhood_xyz_bounds=None if not len(local) else [local.min(axis=0).tolist(), local.max(axis=0).tolist()],
        contact_neighborhood_ground_residual_bounds=None if ground is None or not patch.any() else
            [float((external[patch, 2]-ground[patch]).min()), float((external[patch, 2]-ground[patch]).max())],
        forward_corridor_count=int(corridor.sum()),
        forward_corridor_low_points=int((corridor & (external[:, 2] < .1)).sum()),
        closest_front_low_return_x=None if not (corridor & (external[:, 2] < .1)).any() else
            float(external[corridor & (external[:, 2] < .1), 0].min())))
result = dict(scope=__doc__, archive_sha256=manifest['archive_sha256'], snapshots=rows,
    true_acquisition_timestamp_proven=False, vehicle_started=False, production_modified=False)
(OUT/'contact_input_review.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
print(json.dumps(dict(snapshots=len(rows), first=rows[0], last=rows[-1]), indent=2))
