"""Independent checks of the closed automatic planner and physical component."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET
import zipfile
from check_anchor_connector_replay import snapshot
from wildos_nav2_adapter.graph_route import validate_vehicle_path, validate_local_prefix
from wildos_nav2_adapter.motion_path import motion_sections, diagnose_maneuver_join
from wildos_nav2_adapter.physical_vehicle import physical_snapshot_errors
import tempfile

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'test_artifacts/server_validation_20261011/continuous_start_evidence.zip'


class ContinuousStartEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.archive=zipfile.ZipFile(ARCHIVE)
    @classmethod
    def tearDownClass(cls):cls.archive.close()
    def document(self,name):return json.loads(self.archive.read(name))

    def test_closed_hashes_and_frozen_source_match(self):
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
            '7402251f50f92263aa1a5a16509dcbbede0c8774fae5897006c9c2f7d4024547')
        manifest=self.document('manifest.json')
        self.assertFalse(manifest['full_search_performed'])
        self.assertFalse(manifest['video_recording_enabled'])
        self.assertFalse(any(n.lower().endswith('.mp4') for n in self.archive.namelist()))
        for name,item in manifest['files'].items():
            data=self.archive.read(name)
            with self.subTest(name=name):
                self.assertEqual(len(data),item['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(),item['sha256'])
                if name.startswith('ackermann_smac/'):
                    self.assertEqual((ROOT/name).read_bytes(),data)
        for name,sha in self.document('deployed_runtime.json').items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),sha)

    def test_actual_cpp_fail_closed_normal_recovery_and_lifecycle_checks_passed(self):
        results=[n for n in self.archive.namelist() if n.endswith('.gtest.xml')]
        self.assertEqual(len(results),2)
        total=0
        for name in results:
            root=ET.fromstring(self.archive.read(name))
            self.assertEqual(int(root.attrib['failures']),0)
            self.assertEqual(int(root.attrib['errors']),0)
            self.assertEqual(int(root.attrib.get('disabled',0)),0)
            total+=int(root.attrib['tests'])
        self.assertEqual(total,10)
        summary=self.archive.read('test_summary.log').decode()
        self.assertIn('10 tests, 0 errors, 0 failures, 0 skipped',summary)
        self.assertIn('Ran 143 tests',self.archive.read('deployed_python_tests.log').decode())
        before=self.document('python_return_contract_before.json')
        self.assertEqual(before['curvature_global'],'excess_curvature')
        after=self.document('python_return_contract.json')
        self.assertIsNone(after['curvature_global'])
        self.assertIsNone(after['local_prefix'])
        self.assertIsNone(after['initial_join'])

    def test_returned_exact_prefix_passes_current_production_gates(self):
        rows=list(csv.DictReader(io.StringIO(self.archive.read(
            'build/wildos_ackermann_smac/archived_connected_path.csv').decode())))
        points=[(float(r['x']),float(r['y'])) for r in rows];headings=[float(r['yaw']) for r in rows]
        with tempfile.TemporaryDirectory(dir=ROOT/'_server_repro') as folder:
            p=Path(folder)/'global.txt'
            p.write_bytes(self.archive.read('ackermann_smac/test/fixtures/live_global_snapshot.txt'))
            grid,actual,_=snapshot(p)
            with zipfile.ZipFile(ARCHIVE.parent/'reconnect_component_evidence.zip') as previous:
                p.write_bytes(previous.read('local_snapshot.txt'));local,local_actual,_=snapshot(p)
            self.assertEqual(actual,local_actual)
            self.assertIsNone(validate_vehicle_path(points,headings,[actual[:2],points[-1]],
                grid,points[-1],start=actual[:2],minimum_turning_radius=8.,heading_quantization_bins=72))
            first=motion_sections(points)[0];a,b=first;q=points[a:b+1];h=headings[a:b+1]
            self.assertIsNone(validate_local_prefix(q,h,local,actual[:2],actual[2]))
            join,reason=diagnose_maneuver_join(q,h,actual[:2],actual[2],8.)
            self.assertIsNotNone(join);self.assertIsNone(reason)

    def test_real_automatic_entry_and_cusp_execution_not_visual_arrival(self):
        result=self.document('automatic_entry_component1/result.json')
        self.assertTrue(result['passed']);self.assertEqual(result['errors'],[])
        self.assertFalse(result['visual_arrival_validated'])
        self.assertEqual(result['teleports'],0);self.assertEqual(result['map_reloads'],0)
        self.assertGreater(result['displacement_m'],2.)
        self.assertLessEqual(result['endpoint_distance_m'],1.)
        self.assertFalse(result['terminal']['requires_visual_confirmation'])
        self.assertEqual(result['terminal']['status'],'route_done')
        statuses=[e.get('status') for e in result['events'] if e.get('kind')=='plan_status']
        self.assertEqual(statuses.count('maneuver_done'),2)
        self.assertFalse(set(statuses)&{'follow_failed','replanning_after_maneuver'})
        self.assertEqual(result['actual_parameters']['/planner_server']['CarLike.plugin'],
            'wildos_ackermann_smac/Planner')
        searches=re.findall(r'continuous_start: evaluated=(\d+) candidates=(\d+) timeout=(\d+)',
                            self.archive.read('physical_stack.log').decode())
        self.assertGreaterEqual(len(searches),2)
        for evaluations,candidates,_ in searches:
            self.assertLessEqual(int(evaluations),6000);self.assertGreater(int(candidates),0)
            self.assertLessEqual(int(candidates),6)

    def test_final_stop_physics_loaded_library_and_owned_cleanup(self):
        result=self.document('automatic_entry_component1/result.json')
        self.assertTrue(result['simulation_paused'])
        for snapshot in (result['physics_before'],result['physics_after']):
            self.assertEqual(physical_snapshot_errors(snapshot),[])
        self.assertLess(result['physics_after']['speed'],.05)
        samples=result['stop_samples'];quiet=[]
        for sample in reversed(samples):
            if sample['speed']>=.05:break
            quiet.append(sample)
        self.assertGreaterEqual(quiet[0]['elapsed']-quiet[-1]['elapsed'],.5)
        loaded=self.document('actual_loaded_libraries.json');build=self.document('build_ids.json')
        libraries=[f for r in loaded.values() for f in r['libraries']]
        planner=[f for f in libraries if 'libwildos_ackermann_smac.so' in f['path']]
        self.assertEqual(len(planner),1)
        inventory=self.document('installed_library_inventory.json')
        self.assertEqual(inventory['build']['sha256'],build['libwildos_ackermann_smac.so']['sha256'])
        self.assertEqual(planner[0]['sha256'],inventory['installed']['sha256'])
        # CMake removes build RUNPATH on install, changing the whole-file hash.
        # Check the actual installed file AND matching compiled code identity.
        for key in ('elf_build_id','text_sha256'):
            self.assertEqual(inventory['build'][key],inventory['installed'][key])
        self.assertEqual(self.document('owned_cleanup.json')['remaining'],[])
        original=self.document('production_unchanged.json')
        self.assertEqual(original['status'],'')
        self.assertEqual(original['head'],'f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0')


if __name__=='__main__':unittest.main()
