"""Retain only the proven slow-global-map buffer fix; restore local baseline."""
from pathlib import Path
import hashlib
import json
import shutil
import sys

method='''    def test_marking_window_covers_recorded_intermediate_frame_and_expires(self):
        # Native Nav2 replay: the intermediate obstacle frame is 0.734 s old
        # when the slower global map updates. Keep it for a bounded interval.
        global_layer = self.config['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
        keep = global_layer['cloud'].get('observation_persistence', 0.)
        self.assertGreater(keep, .734)
        self.assertLessEqual(keep, 1.)
        # The controller's faster local map and clearing rays keep their baseline.
        for name in ('local_costmap', 'global_costmap'):
            layer = self.config[name][name]['ros__parameters']['obstacle_layer']
            self.assertEqual(layer['clearing_cloud'].get('observation_persistence', 0.), 0.)
        local = self.config['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        self.assertEqual(local['cloud'].get('observation_persistence', 0.), 0.)

'''
def narrow(root):
    config=root/'nav2_adapter/config/nav2_ackermann_control.yaml'
    raw=config.read_bytes();nl='\r\n' if b'\r\n' in raw else '\n';text=raw.decode('utf-8')
    remove=f'          # Include delayed scans between map updates; expire in source time.{nl}          observation_persistence: 1.0{nl}'
    assert text.count(remove)==2
    text=text.replace(remove,'',1);config.write_bytes(text.encode('utf-8'))
    test=root/'nav2_adapter/test/test_runtime_contract.py';raw=test.read_bytes()
    nl='\r\n' if b'\r\n' in raw else '\n';text=raw.decode('utf-8')
    first=text.index('    def test_marking_window_covers_recorded_intermediate_frame_and_expires(self):')
    last=text.index('    def test_frontier_missing_window_is_explicit_in_graph_entry_and_runtime_gate(self):',first)
    test.write_bytes((text[:first]+method.replace('\n',nl)+text[last:]).encode('utf-8'))

if '--main' in sys.argv:
    root=Path(r'C:\Users\Yu\Desktop\拟真')
    assert hashlib.sha256((root/'nav2_adapter/config/nav2_ackermann_control.yaml').read_bytes()).hexdigest()=='0592fe7c7518b742e0bca2b8ea08c5a8271a24469e02fdb4772bbbe2222aebe8'
    narrow(root)
else:
    parent=Path('/home/ly/execution_continuity_review_20261022')
    old=parent/'v68_observation_interface_120s';new=parent/'v69_global_observation_120s'
    assert not new.exists();new.mkdir()
    shutil.copytree(old/'candidate',new/'candidate',symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
    (new/'full').mkdir()
    for p in (old/'full').iterdir():
        if p.is_file() and p.suffix in ('.py','.json'):(new/'full'/p.name).write_text(p.read_text().replace(str(old),str(new)))
    (new/'full/candidate').symlink_to('../candidate',target_is_directory=True)
    config=json.loads((new/'full/config.json').read_text())
    config['setups']=[str(old/'install/local_setup.bash') if x==str(new/'install/local_setup.bash') else x for x in config['setups']]
    config['setups'].append(str(new/'install/local_setup.bash'))
    (new/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
    lines=(old/'run_full.sh').read_text().splitlines();last=max(i for i,x in enumerate(lines) if x.startswith('source '))
    lines.insert(last+1,f'source {new}/install/local_setup.bash')
    (new/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(old),str(new)) for x in lines)+'\n')
    narrow(new/'candidate')
    manifest=json.loads((parent/'v67_target_road_120s/full/frozen_source_manifest.json').read_text())
    changed=[]
    for name,previous in manifest['members'].items():
        current=hashlib.sha256((new/'candidate'/name).read_bytes()).hexdigest()
        if current!=previous:changed.append(name)
        manifest['members'][name]=current
    assert set(changed)=={'nav2_adapter/config/nav2_ackermann_control.yaml','nav2_adapter/test/test_runtime_contract.py'}
    (new/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for name in ('audit_fluency_120s.py','audit_target_road.py'):shutil.copy2(old/name,new/name)
    (new/'candidate_scope.json').write_text(json.dumps(dict(
        baseline='v67',runtime_delta=['nav2_adapter/config/nav2_ackermann_control.yaml'],
        change='Only global_costmap.obstacle_layer.cloud.observation_persistence=1.0; local/clearing restored to baseline.',
        no_new_runtime_components=True,members=478,search_s=120),indent=2))
    print((new/'candidate_scope.json').read_text())
