"""Apply only the reviewed config and regression to the user's current tree."""
import hashlib
import json
from pathlib import Path

root=Path(r'C:\Users\Yu\Desktop\拟真')
evidence=Path(__file__).resolve().parent/'main_change'
evidence.mkdir(exist_ok=True)
config=root/'nav2_adapter/config/nav2_ackermann_control.yaml'
tests=root/'nav2_adapter/test/test_runtime_contract.py'
before=config.read_bytes()
assert hashlib.sha256(before).hexdigest()=='5a7df9ef2dae4c5e0f4adc9453977079a80fd93c32bb929f3b27cedd43959dc3', 'Main config changed; inspect first'
nl='\r\n' if b'\r\n' in before else '\n'
anchor=f'          marking: true{nl}          clearing: false{nl}'
text=before.decode('utf-8');assert text.count(anchor)==2
after=text.replace(anchor,anchor+f'          # Include delayed scans between map updates; expire in source time.{nl}          observation_persistence: 1.0{nl}').encode('utf-8')
oldtest=tests.read_bytes();testtext=oldtest.decode('utf-8')
testnl='\r\n' if b'\r\n' in oldtest else '\n'
method='''    def test_marking_window_covers_recorded_intermediate_frame_and_expires(self):
        # Real delayed source frame: present 0.734 s before the map update,
        # followed by two scans that did not return the same narrow obstacle.
        for name in ('local_costmap', 'global_costmap'):
            layer = self.config[name][name]['ros__parameters']['obstacle_layer']
            with self.subTest(map=name):
                keep = layer['cloud'].get('observation_persistence', 0.)
                self.assertGreater(keep, .734)
                self.assertLessEqual(keep, 1.)
                self.assertEqual(layer['clearing_cloud'].get('observation_persistence', 0.), 0.)

'''.replace('\n',testnl)
testanchor='    def test_frontier_missing_window_is_explicit_in_graph_entry_and_runtime_gate(self):'
assert testtext.count(testanchor)==1 and 'def test_marking_window_covers_recorded_intermediate_frame_and_expires' not in testtext
newtest=testtext.replace(testanchor,method+testanchor).encode('utf-8')
(evidence/'nav2_ackermann_control.before.yaml').write_bytes(before)
(evidence/'test_runtime_contract.before.py').write_bytes(oldtest)
config.write_bytes(after);tests.write_bytes(newtest)
report=dict(config_before_sha=hashlib.sha256(before).hexdigest(),config_after_sha=hashlib.sha256(after).hexdigest(),
            runtime_files=['nav2_adapter/config/nav2_ackermann_control.yaml'],
            tests=['nav2_adapter/test/test_runtime_contract.py'],commit_created=False)
(evidence/'change.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
