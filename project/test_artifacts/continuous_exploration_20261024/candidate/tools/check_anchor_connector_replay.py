"""Use unchanged production validators on exact curved prefixes + real Smac paths."""
import argparse
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

from wildos_nav2_adapter.graph_route import (
    footprint_blockers, validate_vehicle_path, validate_local_prefix)
from wildos_nav2_adapter.motion_path import (
    motion_sections, initial_maneuver_already_reached)


def snapshot(path):
    lines = Path(path).read_text().splitlines()
    w, h, res, ox, oy = map(float, lines[0].split())
    width, height = int(w), int(h)
    data = [0] * (width * height)
    for line in lines[4:]:
        index, cost = map(int, line.split())
        data[index] = cost
    grid = NS(info=NS(width=width, height=height, resolution=res,
                     origin=NS(position=NS(x=ox, y=oy),
                               orientation=NS(x=0., y=0., z=0., w=1.))), data=data)
    return grid, tuple(map(float, lines[1].split())), tuple(map(float, lines[3].split()))


def inspect(directory):
    directory = Path(directory)
    global_grid, start, goal = snapshot(directory/'smac_boundary_snapshot.txt')
    local_grid, local_start, _ = snapshot(directory/'smac_local_snapshot.txt')
    if start != local_start:
        raise ValueError('Map snapshots do not share the same actual start pose')
    d = directory/'cpp_navigation_probe'
    grouped = {}
    with (d/'anchor_curves.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            grouped.setdefault(int(row['candidate']), []).append(
                tuple(float(row[key]) for key in ('x', 'y', 'yaw')))
    results = []
    for index in range(5):
        prefix = grouped[index]
        with (d/f'anchor_{index}_suffix.csv').open(newline='') as stream:
            suffix = [tuple(float(row[key]) for key in ('x', 'y', 'yaw'))
                      for row in csv.DictReader(stream)]
        gap = math.dist(prefix[-1][:2], suffix[0][:2])
        heading_gap = abs(math.atan2(math.sin(prefix[-1][2]-suffix[0][2]),
                                     math.cos(prefix[-1][2]-suffix[0][2])))
        poses = prefix + suffix[1:]
        points = [p[:2] for p in poses]
        headings = [p[2] for p in poses]
        reason = validate_vehicle_path(points, headings, [start[:2], goal[:2]],
            global_grid, goal[:2], minimum_turning_radius=8., start=start[:2],
            heading_quantization_bins=72)
        local_reason = validate_local_prefix(points, headings, local_grid, start[:2], start[2])
        prefix_blockers = {
            name: sum(bool(footprint_blockers(grid, p[:2], p[2], limit=1)) for p in prefix)
            for name, grid in (('global', global_grid), ('local', local_grid))}
        sections = motion_sections(points)
        reached = initial_maneuver_already_reached(
            points, headings, start[:2], start[2], position_tolerance=.2)
        results.append(dict(candidate=index, prefix_poses=len(prefix), suffix_poses=len(suffix),
                            anchor_gap=gap, anchor_heading_gap=heading_gap,
                            global_path_reason=reason, local_prefix_reason=local_reason,
                            prefix_blockers=prefix_blockers, motion_sections=sections,
                            initial_cusp_already_within_tolerance=reached,
                            accepted=(gap < 1e-8 and heading_gap < 1e-6 and reason is None
                                      and local_reason is None and not reached
                                      and not any(prefix_blockers.values()))))
    return dict(scope='closed_bag_geometry_and_production_validators_no_vehicle_execution',
                actual_start=start, goal=goal, candidates=results,
                accepted_count=sum(r['accepted'] for r in results))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = inspect(args.directory)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['accepted_count'] else 1)
