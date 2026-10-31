"""Check sealed field evidence and preserve both failure and scope limits."""
import hashlib,json,unittest,zipfile
from pathlib import Path
HERE=Path(__file__).parent
class FieldEvidence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=HERE/'tf_delivery_evidence_20261014.zip'
        checksum=json.loads(path.with_suffix('.json').read_text())
        assert hashlib.sha256(path.read_bytes()).hexdigest()==checksum['sha256']
        cls.archive=zipfile.ZipFile(path);cls.manifest=json.loads(cls.archive.read('evidence_manifest.json'))
    @classmethod
    def tearDownClass(cls):cls.archive.close()
    def report(self,trial):return json.loads(self.archive.read(f'physical_cusp_component{trial}/result.json'))
    def test_every_sealed_member_matches_original_byte_identity(self):
        self.assertIsNone(self.archive.testzip())
        for name,identity in self.manifest['members'].items():
            data=self.archive.read(name)
            self.assertEqual(len(data),identity['bytes'],name)
            self.assertEqual(hashlib.sha256(data).hexdigest(),identity['sha256'],name)
        self.assertFalse(self.manifest['debug_component6_is_acceptance'])
    def test_original_execution_and_discovery_failures_are_retained(self):
        for trial in (3,5,7):self.assertFalse(self.report(trial)['passed'])
        self.assertIn('armed_at_ns',self.report(3))
        self.assertNotIn('armed_at_ns',self.report(7))
        self.assertEqual(self.report(7)['publishers_before_arm']['/wildos/native_control_status'],[])
    def test_transport_control_does_not_relax_runtime_or_physics(self):
        baseline=self.report(5)
        for trial in (8,9):
            report=self.report(trial)
            self.assertTrue(report['passed']);self.assertEqual(report['observed_motion_directions'],[-1,1])
            self.assertEqual(report['actual_parameters'],baseline['actual_parameters'])
            self.assertEqual(report['map_reloads'],0);self.assertEqual(report['teleports'],0)
            self.assertFalse(report['visual_arrival_validated']);self.assertFalse(report['video_recording_enabled'])
            for physics in (report['physics_before'],report['physics_after']):
                self.assertEqual(physics['fixedNodeCount'],0)
                self.assertEqual(physics['bodyCollisionNodeCount'],467)
                self.assertEqual(len(physics['wheels']),4)
                self.assertTrue(all(w['collisionNodeCount']==32 for w in physics['wheels']))
            quiet=report['stop_samples'][-1]['elapsed']
            for sample in reversed(report['stop_samples']):
                if sample['speed']>=.05:break
                quiet=sample['elapsed']
            self.assertGreaterEqual(report['stop_samples'][-1]['elapsed']-quiet,.5)
    def test_control_input_freshness_improves_in_sealed_UDP_comparison(self):
        def ages(trial):
            raw=self.archive.read(f'physical_cusp_component{trial}_observer/messages.jsonl').decode()
            odom=None;values=[];stale=[]
            for line in raw.splitlines():
                row=json.loads(line);message=row['message'];now=row['received_ros_ns']
                if row['topic']=='/spot1/odom':
                    s=message['header']['stamp'];odom=s['sec']*10**9+s['nanosec']
                if row['topic']=='/received_global_plan':
                    s=message['header']['stamp'];age=(now-s['sec']*10**9-s['nanosec'])/1e9
                    values.append(age)
                    if odom is not None and (now-odom)/1e9<.3 and age>.5:stale.append(age)
            return values,stale
        values,stale=ages(5);self.assertGreater(len(values),200)
        self.assertGreater(max(values),1.5);self.assertGreater(len(stale),10)
        values,stale=ages(9);self.assertGreater(len(values),200)
        self.assertLess(max(values),.5);self.assertEqual(stale,[])
if __name__=='__main__':unittest.main()
