"""Check frozen field evidence, not current source or a complete search result."""
import hashlib
import io
import json
from pathlib import Path
import unittest
import zipfile

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'test_artifacts/server_validation_20261012/vehicle_control_lease_evidence.zip'


class VehicleLeaseEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive=zipfile.ZipFile(ARCHIVE)

    @classmethod
    def tearDownClass(cls): cls.archive.close()

    def read(self,name): return json.loads(self.archive.read('vehicle_control_lease/'+name))

    def test_hash_closure_and_frozen_source_identity(self):
        metadata=json.loads(ARCHIVE.with_suffix('.json').read_text())
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),metadata['sha256'])
        manifest=json.loads(self.archive.read('member_sha256.json'))
        self.assertEqual(len(manifest),metadata['raw_members'])
        for name,sha in manifest.items():
            self.assertEqual(hashlib.sha256(self.archive.read(name)).hexdigest(),sha,name)
        deployed=self.read('deployed_manifest.json')
        with zipfile.ZipFile(io.BytesIO(self.archive.read('vehicle_control_lease/candidate.zip'))) as candidate:
            for name,sha in deployed.items():
                self.assertEqual(hashlib.sha256(candidate.read(name)).hexdigest(),sha,name)
            self.lua_sha=deployed['nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.lua']
            self.assertEqual(self.lua_sha,self.read('final_server_state.json')['lease']['source_sha256'])
        self.assertFalse(any(n.endswith(('.mp4','.avi','.bag')) for n in manifest))

    def test_vm_and_real_stationary_callback_expiry(self):
        vm=self.read('vm1.json');stationary=self.read('stationary2.json')
        self.assertTrue(vm['passed']);self.assertEqual(len(vm['lua']['cases']),14)
        self.assertTrue(stationary['passed']);self.assertFalse(stationary['moving_test'])
        actual=stationary['expiry_actual_observed_before_status']
        self.assertEqual(actual['wildosControlLeaseTripped'],1)
        self.assertEqual(actual['throttle'],0);self.assertGreater(actual['brake'],.95)
        self.assertTrue(stationary['expired_status']['tripped'])
        self.assertIn('expired or closing',stationary['late_command_rejection'])
        self.assertEqual(stationary['fresh_release'],'wildos_lease_released_v1')
        for row in stationary['samples']: self.assertLess(row['speed'],.05)

    def test_real_dds_freshness_has_healthy_recovery_and_no_driving(self):
        r=self.read('freshness1/execution/result.json')
        self.assertTrue(r['passed']);self.assertFalse(r['moving_test'])
        self.assertEqual(set(r['phases']),{'baseline','delayed_control','control_recovered','delayed_poll','poll_recovered'})
        self.assertEqual(len(r['fault_requests']),2)
        for phase in ('baseline','control_recovered','poll_recovered'):
            natives=[x['data'] for x in r['phases'][phase] if x['kind']=='native']
            self.assertTrue(any(x['telemetry_ok'] and x['control_acknowledged'] and x['actuator_ok'] for x in natives))
            self.assertTrue(all(x['sent']['throttle']==0 and x['sent']['brake']==1 for x in natives))

    def test_short_curve_actual_motion_and_sustained_stop(self):
        r=self.read('short_curve1/execution/result.json');driver=self.read('short_curve1/result.json')
        self.assertTrue(driver['passed']);self.assertEqual(driver['remaining_owned'],[])
        self.assertTrue(r['passed']);self.assertGreater(r['displacement_m'],4.)
        self.assertLessEqual(r['endpoint_distance_m'],1.)
        self.assertEqual(r['terminal']['status'],'route_done')
        self.assertEqual(r['terminal']['minimum_turning_radius_m'],8.)
        self.assertFalse(r['visual_arrival_validated'])
        for key in ('global_reason','local_reason','join_reason'):self.assertIsNone(r['selected'][key])
        samples=r['stop_samples'];end=samples[-1]['elapsed']
        quiet=[x for x in samples if end-x['elapsed']<=.6]
        self.assertGreaterEqual(quiet[-1]['elapsed']-quiet[0]['elapsed'],.5)
        self.assertTrue(all(x['speed']<.05 for x in quiet))

    def test_moving_fault_stops_before_diagnostics_and_old_owner_cannot_resume(self):
        r=self.read('moving_fault2/execution/result.json');driver=self.read('moving_fault2/result.json')
        self.assertTrue(driver['passed']);self.assertEqual(driver['remaining_owned'],[])
        self.assertTrue(r['passed']);samples=r['fault_stop_samples']
        self.assertGreater(len(samples),20)
        self.assertGreater(samples[0]['speed'],1.2);self.assertGreater(samples[0]['actual']['throttle'],0)
        self.assertEqual(samples[0]['actual']['brake'],0)
        guarded=[x for x in samples if x['actual']['wildosControlLeaseTripped']==1]
        self.assertAlmostEqual(guarded[0]['elapsed'],r['first_brake_sec'])
        self.assertLessEqual(r['first_brake_sec'],1.2)
        self.assertTrue(all(x['actual']['throttle']==0 and x['actual']['brake']>.95 for x in guarded))
        end=samples[-1]['elapsed'];quiet=[x for x in samples if end-x['elapsed']<=.55]
        self.assertGreaterEqual(quiet[-1]['elapsed']-quiet[0]['elapsed'],.5)
        self.assertTrue(all(x['speed']<.05 for x in quiet))
        resumed=r['resumed_native']
        self.assertTrue(resumed['telemetry_ok']);self.assertFalse(resumed['actuator_ok'])
        self.assertFalse(resumed['control_acknowledged']);self.assertLess(resumed['speed'],.05)
        statuses={x.get('status') for x in r['events'] if x.get('kind')=='plan_status'}
        self.assertIn('execution_unavailable',statuses);self.assertNotIn('route_done',statuses)

    def test_failures_preserved_and_final_vehicle_is_real_and_released(self):
        self.assertFalse(self.read('stationary1.json')['passed'])
        self.assertFalse(self.read('moving_fault1/diagnostic_review.json')['passed'])
        final=self.read('final_server_state.json');s=final['snapshot']
        self.assertLess(s['speed'],.05);self.assertEqual(s['fixedNodeCount'],0)
        self.assertEqual(s['bodyCollisionNodeCount'],467)
        self.assertEqual([w['collisionNodeCount'] for w in s['wheels']],[32]*4)
        self.assertFalse(final['lease']['state']['active'])
        self.assertEqual(final['remaining_task_clients'],[])
        self.assertEqual(final['original_status'],'')
        self.assertEqual(final['original_head'],'f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0')


if __name__=='__main__':unittest.main()
