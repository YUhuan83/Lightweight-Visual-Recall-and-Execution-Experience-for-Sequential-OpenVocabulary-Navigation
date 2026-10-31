"""One existing Nav2 parameter on each map; no added runtime code."""
import hashlib
import json
from pathlib import Path
import shutil
import sys

parent=Path('/home/ly/execution_continuity_review_20261022')
old=parent/'v67_target_road_120s';new=parent/'v68_observation_interface_120s'
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

'''
if '--apply' not in sys.argv:
    assert not new.exists()
    new.mkdir();shutil.copytree(old/'candidate',new/'candidate',symlinks=True,
                                ignore=shutil.ignore_patterns('__pycache__'))
    (new/'full').mkdir()
    for p in (old/'full').iterdir():
        if p.is_file() and p.suffix in ('.py','.json'):
            (new/'full'/p.name).write_text(p.read_text().replace(str(old),str(new)))
    (new/'full/candidate').symlink_to('../candidate',target_is_directory=True)
    config=json.loads((new/'full/config.json').read_text())
    config['setups']=[str(old/'install/local_setup.bash') if x==str(new/'install/local_setup.bash') else x for x in config['setups']]
    config['setups'].append(str(new/'install/local_setup.bash'))
    (new/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
    lines=(old/'run_full.sh').read_text().splitlines()
    last=max(i for i,x in enumerate(lines) if x.startswith('source '))
    lines.insert(last+1,f'source {new}/install/local_setup.bash')
    (new/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(old),str(new)) for x in lines)+'\n')
    test=new/'candidate/nav2_adapter/test/test_runtime_contract.py'
    text=test.read_text();anchor='    def test_frontier_missing_window_is_explicit_in_graph_entry_and_runtime_gate(self):'
    assert text.count(anchor)==1;test.write_text(text.replace(anchor,method+anchor))
    for name in ('isolated_ros_review.py','audit_fluency_120s.py','audit_target_road.py'):
        shutil.copy2(old/name,new/name)
else:
    config=new/'candidate/nav2_adapter/config/nav2_ackermann_control.yaml'
    text=config.read_text();anchor='          marking: true\n          clearing: false\n'
    assert text.count(anchor)==2
    text=text.replace(anchor,anchor+'          # Include delayed scans between map updates; expire in source time.\n          observation_persistence: 1.0\n')
    config.write_text(text)
    manifest=json.loads((old/'full/frozen_source_manifest.json').read_text());differences=[]
    for name,previous in manifest['members'].items():
        current=hashlib.sha256((new/'candidate'/name).read_bytes()).hexdigest()
        if current!=previous:differences.append(name)
        manifest['members'][name]=current
    assert set(differences)=={'nav2_adapter/config/nav2_ackermann_control.yaml','nav2_adapter/test/test_runtime_contract.py'},differences
    (new/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (new/'candidate_scope.json').write_text(json.dumps(dict(
        runtime_delta=['nav2_adapter/config/nav2_ackermann_control.yaml'],tests_delta=['nav2_adapter/test/test_runtime_contract.py'],
        change='Set existing marking observation_persistence to 1.0s in both existing costmaps; clearing stays latest-only.',
        no_new_runtime_components=True,members=len(manifest['members']),search_s=120),indent=2))
    print((new/'candidate_scope.json').read_text())
