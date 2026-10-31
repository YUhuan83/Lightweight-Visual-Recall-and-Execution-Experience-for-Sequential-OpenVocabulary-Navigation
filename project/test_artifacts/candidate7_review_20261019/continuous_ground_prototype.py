"""Diagnostic only: test connected lower returns before changing production ground removal."""
from collections import deque
from pathlib import Path
import json
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / 'nav2_adapter')]
from wildos_nav2_adapter.pointcloud import local_ground_plane, filter_robot_footprint


def confirmed_ground(points, radius=8., max_grade=.35, cell_size=.1):
    """Follow supported adjacent minima only; never bridge a missing cell."""
    plane = local_ground_plane(points, radius)
    confirmed = np.zeros(len(points), dtype=bool)
    if plane is None:
        return confirmed
    near = np.flatnonzero((np.linalg.norm(points[:, :2], axis=1) <= radius)
                         & (points[:, 2] >= -2.) & (points[:, 2] <= 1.2))
    p = points[near]
    cells, inverse = np.unique(np.floor(p[:, :2] / cell_size).astype(int), axis=0, return_inverse=True)
    lows = np.full(len(cells), np.inf)
    np.minimum.at(lows, inverse, p[:, 2])
    selected = np.full(len(cells), len(p), dtype=int)
    mask = p[:, 2] == lows[inverse]
    np.minimum.at(selected, inverse[mask], np.flatnonzero(mask))
    support = p[selected]
    residual = support[:, 2] - (support[:, :2] @ plane[:2] + plane[2])
    ground = (np.abs(residual) <= .05) & (np.linalg.norm(support[:, :2], axis=1) <= 4.)
    queue = deque(np.flatnonzero(ground))
    lookup = {tuple(cell): i for i, cell in enumerate(cells)}
    while queue:
        i = queue.popleft()
        x, y = cells[i]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            j = lookup.get((x + dx, y + dy))
            if j is None or ground[j]:
                continue
            separation = np.linalg.norm(support[i, :2] - support[j, :2])
            if abs(lows[i] - lows[j]) <= max_grade * separation + 1.e-6:
                ground[j] = True
                queue.append(j)
    # A one-dimensional chain is insufficient support for a ground surface.
    # Require connected support in both grid axes before removing a marking.
    surface = np.zeros(len(cells), dtype=bool)
    for i in np.flatnonzero(ground):
        x, y = cells[i]
        axes = []
        for offsets in (((1, 0), (-1, 0)), ((0, 1), (0, -1))):
            valid = False
            for dx, dy in offsets:
                j = lookup.get((x + dx, y + dy))
                if j is not None and ground[j]:
                    distance = np.linalg.norm(support[i, :2] - support[j, :2])
                    valid |= abs(lows[i] - lows[j]) <= max_grade * distance + 1.e-6
            axes.append(valid)
        surface[i] = all(axes)
    confirmed[near] = surface[inverse] & (p[:, 2] - lows[inverse] < .10)
    return confirmed


def main():
    xy = np.array([(x, y) for x in np.arange(.03, 7.5, .1) for y in np.arange(-2.97, 3., .1)])
    road = np.c_[xy, .015 * xy[:, 0] ** 2 - .5]
    ground = confirmed_ground(road)
    plane = local_ground_plane(road, 8.)
    obstacle = (road[:, 2] - (road[:, :2] @ plane[:2] + plane[2]) >= .1)
    controls = dict(continuous_convex_false_positives=int(obstacle.sum()),
                    removed_by_prototype=int((obstacle & ground).sum()))
    inputs = ROOT / 'test_artifacts/passability_contract_20261019'
    rows = []
    with np.load(inputs / 'replay_inputs.npz', allow_pickle=False) as arrays:
        for i in range(6):
            key = 'scan_%02d' % i
            points = filter_robot_footprint(arrays[key + '_body'], 3.4, 1.05, 1.05)
            contact = arrays[key + '_contact_body']
            begin = time.perf_counter()
            ground = confirmed_ground(points)
            plane = local_ground_plane(points, 8.)
            marked = (points[:, 2] - (points[:, :2] @ plane[:2] + plane[2]) >= .1)
            local = np.linalg.norm(points[:, :2] - contact[:2], axis=1) <= .6
            rows.append(dict(key=key, contact_markings=int((local & marked).sum()),
                             removed_contact_markings=int((local & marked & ground).sum()),
                             elapsed_ms=1000. * (time.perf_counter() - begin)))
    report = dict(scope=__doc__, controls=controls, early_contact=rows,
                  production_modified=False, approved_for_deployment=False,
                  pass_condition='Remove convex-road false positives while preserving all known collision-area markings.')
    report['passed'] = controls['removed_by_prototype'] == controls['continuous_convex_false_positives'] and all(
        r['removed_contact_markings'] == 0 for r in rows)
    (OUT / 'continuous_ground_prototype.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
