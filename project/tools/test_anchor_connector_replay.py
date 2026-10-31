"""Recorded exact-to-lattice joins and mixed heading precision regressions."""
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import zipfile

from check_anchor_connector_replay import inspect
from wildos_nav2_adapter.graph_route import validate_vehicle_path, validate_local_prefix

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT/'test_artifacts/server_validation_20261010/anchor_connector_evidence.zip'


class RecordedConnectorTests(unittest.TestCase):
    def test_archive_members_and_tested_sources_match(self):
        with zipfile.ZipFile(ARCHIVE) as archive:
            manifest = json.loads(archive.read('connector_manifest.json'))
            for name, data in manifest['files'].items():
                with self.subTest(name=name):
                    content = archive.read(name)
                    self.assertEqual(len(content), data['size'])
                    self.assertEqual(hashlib.sha256(content).hexdigest(), data['sha256'])
            for local, member in manifest['local_sources'].items():
                self.assertEqual(hashlib.sha256((ROOT/local).read_bytes()).hexdigest(),
                                 manifest['files'][member]['sha256'])
            self.assertIn('no vehicle execution', manifest['scope'])

    def test_recorded_join_passes_unchanged_collision_and_cusp_gates(self):
        # Keep the scratch files within the writable workspace. Windows sandbox
        # TEMP may create a directory whose files cannot be read or cleaned up.
        scratch = ROOT/'test_artifacts/offline_navigation'
        with zipfile.ZipFile(ARCHIVE) as archive, tempfile.TemporaryDirectory(dir=scratch) as folder:
            root = Path(folder)
            names = ['smac_boundary_snapshot.txt', 'smac_local_snapshot.txt',
                     'cpp_navigation_probe/anchor_curves.csv']
            names += [f'cpp_navigation_probe/anchor_{i}_suffix.csv' for i in range(5)]
            for name in names:
                path = root/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(archive.read(name))
            result = inspect(root)
            self.assertEqual(result['accepted_count'], 4)
            first = result['candidates'][0]
            self.assertTrue(first['accepted'])
            self.assertEqual(first['prefix_blockers'], {'global': 0, 'local': 0})
            self.assertFalse(first['initial_cusp_already_within_tolerance'])
            self.assertGreater(len(first['motion_sections']), 1)
            before = json.loads(archive.read('cpp_navigation_probe/connector_production_before.json'))
            self.assertEqual(before['accepted_count'], 0)
            self.assertEqual(before['candidates'][0]['global_path_reason'], 'excess_curvature')
            self.assertEqual(result['candidates'][1]['global_path_reason'], 'excess_curvature')

    def test_real_cpp_controls_and_snapshot_age_are_present(self):
        with zipfile.ZipFile(ARCHIVE) as archive:
            log = archive.read('cpp_navigation_probe/connector_controls_detail.log').decode()
            self.assertIn('8 connector controls passed', log)
            metadata = json.loads(archive.read('connector_snapshot_metadata.json'))
            self.assertEqual(metadata['frame'], 'spot1/odom')
            self.assertLess(metadata['local_map_age_sec'], 1.5)
            self.assertTrue(metadata['bag_closed'])
            summary = json.loads(archive.read('cpp_navigation_probe/anchor_summary.json'))
            self.assertGreater(summary['feasible_count'], 0)
            self.assertEqual(summary['candidates'][0]['cusps'], 0)
            self.assertAlmostEqual(summary['candidates'][0]['length'], .9228009505292532)


class MixedHeadingPrecisionTests(unittest.TestCase):
    def grid(self):
        return NS(info=NS(width=160, height=160, resolution=.25,
                         origin=NS(position=NS(x=-10., y=-10.),
                                   orientation=NS(x=0., y=0., z=0., w=1.))), data=[0]*25600)

    def circle(self, radius, mixed=True):
        points, headings = [], []
        for i in range(51):
            theta = -1.03 + i*.1/radius
            points.append((radius*(math.sin(theta)-math.sin(-1.03)),
                           radius*(math.cos(-1.03)-math.cos(theta))))
            headings.append(round(theta/(math.pi/36))*(math.pi/36)
                            if mixed and i>=20 else theta)
        return points, headings

    def reason(self, points, headings, grid):
        return validate_vehicle_path(points, headings, points, grid, points[-1],
                                     minimum_turning_radius=8., start=points[0],
                                     heading_quantization_bins=72)

    def test_exact_and_mixed_radius_eight_circles_are_accepted(self):
        for mixed in (False, True):
            p, h = self.circle(8., mixed)
            with self.subTest(mixed=mixed):
                self.assertIsNone(self.reason(p, h, self.grid()))

    def test_mixed_precision_does_not_allow_tighter_geometry(self):
        for radius in (7.99, 6.):
            p, h = self.circle(radius)
            with self.subTest(radius=radius):
                self.assertEqual(self.reason(p, h, self.grid()), 'excess_curvature')

    def test_exact_intervals_do_not_receive_lattice_uncertainty(self):
        p, h = self.circle(8., False)
        h[10] += .53
        # Include a genuinely aligned heading elsewhere, outside this interval.
        h[-1] = round(h[-1]/(math.pi/36))*(math.pi/36)
        self.assertEqual(self.reason(p, h, self.grid()), 'excess_curvature')

    def test_mixed_precision_does_not_bypass_body_collision(self):
        p, h = self.circle(8.)
        grid = self.grid()
        wx, wy = 2*math.cos(h[0])-.8*math.sin(h[0]), 2*math.sin(h[0])+.8*math.cos(h[0])
        x, y = int((wx+10)/.25), int((wy+10)/.25)
        grid.data[y*160+x] = 100
        self.assertEqual(self.reason(p, h, grid), 'footprint_collision')


class InitialJoinScopeTests(unittest.TestCase):
    """Diagnostic controls: map/path validity alone does not prove a start join."""

    grid = MixedHeadingPrecisionTests.grid

    def test_safe_path_and_current_body_do_not_validate_the_intervening_join(self):
        points = [(.5+i*.1, 0.) for i in range(51)]
        headings = [0.] * len(points)
        grid = self.grid()
        # Safe bodies and a valid straight path, but the actual vehicle faces
        # perpendicular to a start only 0.5 m away. Neither gate builds the
        # required radius-8 connection. This records a limitation, not success.
        self.assertIsNone(validate_vehicle_path(points, headings, points, grid, points[-1],
            minimum_turning_radius=8., start=(0., 0.), heading_quantization_bins=72))
        self.assertIsNone(validate_local_prefix(points, headings, grid, (0., 0.), math.pi/2))

    def test_matching_initial_pose_passes_the_same_path_and_body_gates(self):
        points = [(i*.1, 0.) for i in range(51)]
        headings = [0.] * len(points)
        grid = self.grid()
        self.assertIsNone(validate_vehicle_path(points, headings, points, grid, points[-1],
            minimum_turning_radius=8., start=points[0], heading_quantization_bins=72))
        self.assertIsNone(validate_local_prefix(points, headings, grid, points[0], headings[0]))


if __name__ == '__main__':
    unittest.main()
