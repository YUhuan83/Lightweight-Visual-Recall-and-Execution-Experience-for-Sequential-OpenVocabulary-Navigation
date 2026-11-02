"""Recorded XYZ classification/timing, optionally checked against a static survey."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from wildos_nav2_adapter import pointcloud as current_cloud
from wildos_nav2_adapter.pointcloud import split_costmap_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("clouds", nargs="+", type=Path)
    parser.add_argument("--surface", type=Path)
    parser.add_argument("--reference", type=Path)
    args = parser.parse_args()
    interpolators = None
    if args.surface:
        rows = json.loads(args.surface.read_text())
        xs, ys = sorted({r["x"] for r in rows}), sorted({r["y"] for r in rows})
        z = np.full((len(xs), len(ys)), np.nan)
        terrain = z.copy()
        for row in rows:
            if row["hasStatic"]:
                i, j = xs.index(row["x"]), ys.index(row["y"])
                z[i, j], terrain[i, j] = row["staticZ"], row["terrainZ"]
        dx, dy = np.gradient(z, xs, ys)
        interpolators = [
            RegularGridInterpolator((xs, ys), a, bounds_error=False, fill_value=np.nan)
            for a in (z, terrain, np.hypot(dx, dy))
        ]
    report = []
    reference = None
    if args.reference:
        import importlib.util

        spec = importlib.util.spec_from_file_location("reference_cloud", args.reference)
        reference = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(reference)
    for path in args.clouds:
        data = np.load(path)
        points = data["body"]
        options = dict(
            min_height=0.1,
            max_height=1.5,
            filter_footprint=True,
            front_extent=3.4,
            rear_extent=1.05,
            half_width=1.05,
            voxel=0.2,
            max_points=20000,
            ground_plane_radius=8.0,
            ground_clearance=0.25,
            obstacle_min_height=0.1,
            ground_max_slope=0.35,
        )
        times = []
        for iteration in range(11):
            began = time.perf_counter()
            markings, clearing = split_costmap_points(points, **options)
            if iteration:
                times.append((time.perf_counter() - began) * 1000)
        result = dict(
            file=str(path),
            points=len(points),
            markings=len(markings),
            clearing=len(clearing),
            median_ms=float(np.median(times)),
            p95_ms=float(np.percentile(times, 95)),
            max_ms=max(times),
        )
        if interpolators:
            affine = np.linalg.lstsq(
                np.c_[points, np.ones(len(points))], data["world"], rcond=None
            )[0]
            world = np.c_[markings, np.ones(len(markings))] @ affine
            surface, terrain, slope = [fn(world[:, :2]) for fn in interpolators]
            region = (
                (world[:, 0] >= 728)
                & (world[:, 0] <= 740)
                & (world[:, 1] >= -82)
                & (world[:, 1] <= -71)
            )
            observed = region & (np.abs(surface - terrain) < 0.05) & (slope < 0.15)
            gap = world[:, 2] - surface
            result.update(
                region_marks=int(np.sum(region)),
                road_marks=int(np.sum(observed & (np.abs(gap) < 0.05))),
                raised_marks=int(np.sum(observed & (gap > 0.1))),
                marks_above_terrain=int(np.sum(region & (world[:, 2] - terrain > 0.1))),
                scope="Terrain-matched smooth static survey region only",
            )
            if reference:
                old, old_clearing = reference.split_costmap_points(points, **options)
                plane = reference.local_ground_plane(
                    points, options["ground_plane_radius"]
                )
                old_ground = reference.connected_ground_returns(
                    points,
                    plane,
                    options["ground_plane_radius"],
                    options["obstacle_min_height"],
                    options["ground_max_slope"],
                ) | reference.supported_ground_patches(
                    points,
                    plane,
                    options["ground_plane_radius"],
                    options["obstacle_min_height"],
                    options["ground_max_slope"],
                )
                new_ground = current_cloud.supported_ground_patches(
                    points,
                    plane,
                    options["ground_plane_radius"],
                    options["obstacle_min_height"],
                    options["ground_max_slope"],
                ) | current_cloud.connected_ground_returns(
                    points,
                    plane,
                    options["ground_plane_radius"],
                    options["obstacle_min_height"],
                    options["ground_max_slope"],
                )
                false_ground = np.flatnonzero(old_ground & ~new_ground)
                new_road = np.flatnonzero(new_ground & ~old_ground)
                old_world = np.c_[old, np.ones(len(old))] @ affine
                old_surface, old_terrain, old_slope = [
                    fn(old_world[:, :2]) for fn in interpolators
                ]
                old_region = (
                    (old_world[:, 0] >= 728)
                    & (old_world[:, 0] <= 740)
                    & (old_world[:, 1] >= -82)
                    & (old_world[:, 1] <= -71)
                )
                old_observed = (
                    old_region
                    & (np.abs(old_surface - old_terrain) < 0.05)
                    & (old_slope < 0.15)
                )
                result["reference"] = dict(
                    region_marks=int(old_region.sum()),
                    road_marks=int(
                        np.sum(
                            old_observed
                            & (np.abs(old_world[:, 2] - old_surface) < 0.05)
                        )
                    ),
                    marks_above_terrain=int(
                        np.sum(old_region & (old_world[:, 2] - old_terrain > 0.1))
                    ),
                    marking_count=len(old),
                    clearing_count=len(old_clearing),
                    markings_exact=np.array_equal(markings, old),
                    clearing_exact=np.array_equal(clearing, old_clearing),
                    support_ground_count=int(old_ground.sum()),
                    accelerated_ground_count=int(new_ground.sum()),
                    missed_ground_count=len(false_ground),
                    extra_ground_count=len(new_road),
                    first_missed_ground=points[false_ground[:5]].tolist(),
                    first_extra_ground=points[new_road[:5]].tolist(),
                )
        report.append(result)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
