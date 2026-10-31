"""Frozen execution-health evidence, explicitly excluding vehicle acceptance."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'test_artifacts/server_validation_20261012/execution_health_evidence.zip'


class ExecutionHealthEvidenceTests(unittest.TestCase):
    def document(self,name):
        with zipfile.ZipFile(ARCHIVE) as z:return json.loads(z.read(name))

    def test_archive_closed_and_video_absent(self):
        self.assertEqual(ARCHIVE.stat().st_size,113656)
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         '8c0329b51138def848f4c8d4829510062cb8965d04fd5ef7320d8febec8b38a0')
        with zipfile.ZipFile(ARCHIVE) as z:
            manifest=json.loads(z.read('manifest.json'))
            self.assertEqual(len(manifest),21)
            self.assertEqual(set(z.namelist()),set(manifest)|{'manifest.json'})
            for name,meta in manifest.items():
                data=z.read(name)
                self.assertEqual(len(data),meta['bytes'],name)
                self.assertEqual(hashlib.sha256(data).hexdigest(),meta['sha256'],name)
            self.assertFalse(any(n.endswith('.mp4') for n in z.namelist()))

    def test_real_dds_action_retires_late_success_and_retains_destination(self):
        report=self.document('execution_health_repair/dds_result2.json')
        self.assertTrue(report['passed']);self.assertEqual(report['errors'],[])
        self.assertEqual(report['accepted'],[0]);self.assertEqual(report['returned'],[0])
        self.assertEqual(report['phases'],['healthy_action_accepted',
            'fault_cancels_and_preserves_destination','late_success_not_promoted',
            'new_fresh_health_can_resume_same_destination'])
        self.assertTrue(report['cancellations'])
        self.assertTrue(report['no_beamng_connection'])
        self.assertTrue(report['no_physical_execution'])
        self.assertFalse(any(r['data']['status'] in ('route_done','maneuver_done','segment_done')
                             for r in report['statuses']))

    def test_frozen_sources_are_verified_independently_of_future_edits(self):
        report=self.document('execution_health_repair/verified_sources.json')
        self.assertEqual(report['tests'],dict(count=195,passed=True))
        with zipfile.ZipFile(ARCHIVE) as z:
            for name,meta in report['sources'].items():
                data=z.read('execution_health_repair/sources/'+name)
                self.assertEqual(hashlib.sha256(data).hexdigest(),meta['sha256'],name)
        self.assertEqual(report['vehicle_side_lease'],'NOT IMPLEMENTED')
        self.assertEqual(report['field_motion'],'PENDING')
        self.assertEqual(report['full_search_acceptance'],'PENDING')

    def test_safe_stock_vehicle_and_remaining_work_are_explicit(self):
        report=self.document('execution_health_repair/final_safe_state.json')
        self.assertTrue(report['passed']);self.assertTrue(report['paused'])
        self.assertEqual(report['remaining_motion_clients'],[])
        self.assertEqual(report['production_status'],'')
        state=report['snapshot']
        self.assertLess(state['speed'],.05)
        self.assertEqual(state['fixedNodeCount'],0)
        self.assertEqual(state['bodyCollisionNodeCount'],467)
        self.assertEqual([w['collisionNodeCount'] for w in state['wheels']],[32]*4)
        self.assertFalse(state['cameras']);self.assertFalse(state['lidars'])
        summary=self.document('execution_health_repair/summary.json')
        self.assertEqual(summary['native_engine_stall_root_cause'],'UNRESOLVED')
        self.assertEqual(summary['full_search_acceptance'],'PENDING')
        self.assertFalse(summary['new_commit'])


if __name__=='__main__':unittest.main()
