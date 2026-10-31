"""Read-only candidate7 replay on the frozen 084254 contact scans.

Checks that ground-relative marking changes the contact cell and that the
new witnessed-step layer does not turn the smooth low surface into a step.
No ROS, vehicle or production deployment is involved.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'nav2_adapter'))
sys.path.insert(0, str(ROOT / 'official_adapter'))

from beamng_wildos_adapter.mapping import occupancy_from_points  # noqa: E402
from beamng_wildos_adapter.terrain import terrain_layers  # noqa: E402

RESOLUTION = 0.5
WIDTH = HEIGHT = 80


def contact_cell(contact):
    return (math.floor(float(contact[0]) / RESOLUTION + WIDTH / 2),
            math.floor(float(contact[1]) / RESOLUTION + HEIGHT / 2))


def main():
    arrays = np.load(OUT / 'replay_inputs.npz', allow_pickle=False)
    manifest = json.loads((OUT / 'input_manifest.json').read_text(encoding='utf8'))
    rows = []
    for sample in manifest['snapshots']:
        key = sample['key']
        points = arrays[key + '_body'].astype(float)
        contact = arrays[key + '_contact_body']
        column, row = contact_cell(contact)
        legacy = occupancy_from_points(
            points, sensor_origin_xy=(1.252, 0.0), clear_robot_footprint=True,
            carrier_model='ackermann', obstacle_min_height=None)
        candidate = occupancy_from_points(
            points, sensor_origin_xy=(1.252, 0.0), clear_robot_footprint=True,
            carrier_model='ackermann', obstacle_min_height=0.10)
        begin = time.perf_counter()
        layers = terrain_layers(
            points, sensor_origin_xy=(1.252, 0.0), carrier_model='ackermann',
            obstacle_min_height=0.10)
        elapsed_ms = (time.perf_counter() - begin) * 1000.
        rows.append({
            'key': key,
            'engine_seconds': sample['engine_seconds'],
            'contact_cell': [column, row],
            'legacy_occupied': int(legacy[row, column] >= 50),
            'candidate_occupied': int(candidate[row, column] >= 50),
            'legacy_cells': int((legacy >= 50).sum()),
            'candidate_cells': int((candidate >= 50).sum()),
            'traversability': float(layers['traversability'][row, column]),
            'witnessed_step_contact': float(layers['witnessed_step'][row, column]),
            'witnessed_step_max': float(layers['witnessed_step'].max()),
            'elevation_valid_contact': float(layers['elevation_valid'][row, column]),
            'terrain_ms': elapsed_ms,
        })
    early = rows[:6]
    report = {
        'scope': __doc__,
        'source_db_sha256': manifest['database']['sha256'],
        'early_contact_cells_marked': all(row['candidate_occupied'] for row in early),
        'early_legacy_never_marked': all(not row['legacy_occupied'] for row in early),
        'contact_is_not_a_witnessed_step':
            all(row['witnessed_step_contact'] < .03 for row in early),
        'rows': rows,
        'production_modified': False,
        'complete_navigation_passed': False,
    }
    (OUT / 'candidate7_replay_validation.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
