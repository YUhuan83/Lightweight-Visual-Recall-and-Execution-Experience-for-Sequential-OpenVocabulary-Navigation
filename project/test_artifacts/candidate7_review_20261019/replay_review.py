"""Compare reviewed step evidence on 22 immutable contact scans, without ROS."""
from pathlib import Path
import hashlib
import importlib.util
import json
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
INPUT = ROOT / 'test_artifacts/passability_contract_20261019'
sys.path[:0] = [str(ROOT / 'official_adapter'), str(ROOT / 'nav2_adapter')]
from beamng_wildos_adapter.terrain import terrain_layers


def main():
    path = OUT / 'before/official_adapter/beamng_wildos_adapter/terrain.py'
    spec = importlib.util.spec_from_file_location('beamng_wildos_adapter._review_before', path)
    before = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(before)
    pointcloud_path = OUT / 'before/nav2_adapter/wildos_nav2_adapter/pointcloud.py'
    pc_spec = importlib.util.spec_from_file_location('_pointcloud_before_review', pointcloud_path)
    old_pc = importlib.util.module_from_spec(pc_spec)
    pc_spec.loader.exec_module(old_pc)
    map_path = OUT / 'before/official_adapter/beamng_wildos_adapter/mapping.py'
    map_spec = importlib.util.spec_from_file_location('beamng_wildos_adapter._mapping_before_review', map_path)
    old_mapping = importlib.util.module_from_spec(map_spec)
    map_spec.loader.exec_module(old_mapping)
    old_mapping.split_costmap_points = old_pc.split_costmap_points
    before.occupancy_from_points = old_mapping.occupancy_from_points
    archive = INPUT / 'replay_inputs.npz'
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == (
        '02ab9c2f2ef2d7fbf5c3535780d9dabfce6f655e4cd24e929b0c7442024681c0')
    manifest = json.loads((INPUT / 'input_manifest.json').read_text(encoding='utf8'))
    rows = []
    with np.load(archive, allow_pickle=False) as arrays:
        for sample in manifest['snapshots']:
            key = sample['key']
            # Production terrain is gravity-aligned; body replay is retained
            # separately to check the Nav2-equivalent historical low marking.
            points = arrays[key + '_level']
            contact = arrays[key + '_contact_level']
            opts = dict(sensor_origin_xy=tuple(arrays[key + '_sensor_level'][:2]),
                        carrier_model='ackermann', obstacle_min_height=.1)
            old = before.terrain_layers(points, **opts)
            begin = time.perf_counter()
            new = terrain_layers(points, **opts)
            elapsed = 1000. * (time.perf_counter() - begin)
            equal = {k: bool(np.array_equal(v, new[k])) for k, v in old.items()
                     if k != 'witnessed_step'}
            assert equal['unknown'], (key, 'ground confirmation must not fill unknown')
            col, row = np.floor(contact[:2] / .5 + [40, 40]).astype(int)
            rows.append(dict(key=key, unchanged_other_layers=equal,
                             changed_step_cells=int(np.count_nonzero(old['witnessed_step'] != new['witnessed_step'])),
                             occupied_cells_before=int((old['traversability'] >= .5).sum()),
                             occupied_cells_after=int((new['traversability'] >= .5).sum()),
                             old_step_max=float(old['witnessed_step'].max()),
                             new_step_max=float(new['witnessed_step'].max()),
                             old_contact_step=float(old['witnessed_step'][row, col]),
                             new_contact_step=float(new['witnessed_step'][row, col]),
                             contact_occupied=bool(new['traversability'][row, col] >= .5),
                             elapsed_ms=elapsed))
    report = dict(scope=__doc__, passed=True, frames=len(rows), rows=rows,
                  contact_early_marking_retained=all(r['contact_occupied'] for r in rows[:6]),
                  maximum_terrain_ms=max(r['elapsed_ms'] for r in rows),
                  complete_navigation_passed=False)
    assert report['contact_early_marking_retained']
    (OUT / 'replay_result.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    main()
