import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
CASE = ROOT / 'server_evidence/trial14/blue_house_review'
rows = json.loads((CASE / 'projection_review.json').read_text())['rows']
row = next(x for x in rows if x['mask_stamp_ns'] == 1791513863084888339)
data = np.load(CASE / '1791513863084888339_projection.npz')
camera = data['matched_pose_points_cam']
world = data['matched_pose_points_world']
mask = data['mask']
k = np.array(row['K'])
valid = np.all(np.isfinite(camera), axis=1) & (camera[:, 2] > .1)
camera, world = camera[valid], world[valid]
pix = camera @ k.T
pix = pix[:, :2] / pix[:, 2:]
h, w = mask.shape
inside = (pix[:, 0] >= 0) & (pix[:, 0] < w) & (pix[:, 1] >= 0) & (pix[:, 1] < h)
camera, world, pix = camera[inside], world[inside], pix[inside].astype(int)
hit = mask[pix[:, 1], pix[:, 0]]
camera, world, pix = camera[hit], world[hit], pix[hit]
order = np.argsort(camera[:, 2])
_, unique = np.unique(pix[order, 1] * w + pix[order, 0], return_index=True)
selected = order[unique]
camera, world, pix = camera[selected], world[selected], pix[selected]
order = np.argsort(camera[:, 2])
camera, world, pix = camera[order], world[order], pix[order]
cuts = np.flatnonzero(np.diff(camera[:, 2]) > np.maximum(.6, .04 * camera[:-1, 2])) + 1
groups = np.split(np.arange(len(camera)), cuts)
report = {'mask_stamp_ns': row['mask_stamp_ns'], 'unique_points_in_mask': len(camera), 'surfaces': []}
for indices in groups:
    report['surfaces'].append({'points': len(indices), 'share': len(indices) / len(camera), 'camera_depth_median_m': float(np.median(camera[indices, 2])), 'world_median': np.median(world[indices], axis=0).tolist(), 'projected_bbox': [*pix[indices].min(axis=0).tolist(), *pix[indices].max(axis=0).tolist()]})
raw = json.loads((CASE / 'localizations.json').read_text())
actual = next(x for x in raw if x['observation_stamp_ns'] == row['mask_stamp_ns'])
report['live_raw_localization'] = actual['position']
report['same_pose_replay_localization'] = row['matched_pose_depth_world']
report['replay_live_error_m'] = float(np.linalg.norm(np.array(actual['position']) - row['matched_pose_depth_world']))
result = json.loads((ROOT / 'server_evidence/trial14/full_test.json').read_text())
arrival = result['green_house_arrival']['status']
report['arrival'] = {name: arrival[name] for name in ('object_position', 'vehicle_position', 'standoff', 'standoff_distance_m', 'object_distance_m', 'configured_object_arrival_tolerance_m', 'object_arrival_tolerance_m')}
report['far_surface_robot_planar_distance_m'] = float(np.linalg.norm(np.median(world[groups[-1]], axis=0)[:2] - np.array(arrival['vehicle_position'])))
report['initial_tf_interpolated_replay_correction'] = 'The earlier far replay used bag TF interpolation differing from the embedded acquisition pose by about 2.14 degrees; matching the recorded acquisition pose reproduces the live near-surface result exactly. This comparison does not prove a live TF error.'
(CASE / 'surface_selection_summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')
print(json.dumps(report, indent=2))
