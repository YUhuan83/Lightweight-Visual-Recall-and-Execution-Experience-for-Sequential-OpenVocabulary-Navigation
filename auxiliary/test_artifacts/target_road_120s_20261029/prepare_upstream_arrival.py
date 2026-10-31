"""Reuse the existing v65 direct-arrival implementation on preserved v67."""
import hashlib
import json
from pathlib import Path
import shutil

parent=Path('/home/ly/execution_continuity_review_20261022')
base=parent/'v67_target_road_120s';implementation=parent/'v65_upstream_object_flow'
new=parent/'v70_upstream_arrival_120s'
assert not new.exists();new.mkdir()
shutil.copytree(base/'candidate',new/'candidate',symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
(new/'full').mkdir()
for path in (base/'full').iterdir():
    if path.is_file() and path.suffix in ('.py','.json'):
        (new/'full'/path.name).write_text(path.read_text().replace(str(base),str(new)))
(new/'full/candidate').symlink_to('../candidate',target_is_directory=True)
config=json.loads((new/'full/config.json').read_text())
config['setups']=[str(base/'install/local_setup.bash') if x==str(new/'install/local_setup.bash') else x for x in config['setups']]
config['setups'].append(str(new/'install/local_setup.bash'))
(new/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
lines=(base/'run_full.sh').read_text().splitlines();last=max(i for i,x in enumerate(lines) if x.startswith('source '))
lines.insert(last+1,f'source {new}/install/local_setup.bash')
(new/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(base),str(new)) for x in lines)+'\n')
runtime=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/arrival_criteria.py']
for name in runtime:shutil.copy2(implementation/'candidate'/name,new/'candidate'/name)
tests=[]
for path in (implementation/'candidate/nav2_adapter/test').glob('*.py'):
    target=new/'candidate/nav2_adapter/test'/path.name
    if target.exists() and target.read_bytes()!=path.read_bytes():
        name=str(target.relative_to(new/'candidate'));tests.append(name);shutil.copy2(path,target)
manifest=json.loads((base/'full/frozen_source_manifest.json').read_text());changed=[]
for name,previous in manifest['members'].items():
    current=hashlib.sha256((new/'candidate'/name).read_bytes()).hexdigest()
    if current!=previous:changed.append(name)
    manifest['members'][name]=current
assert set(changed)==set(runtime+tests),changed
(new/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
for name in ('isolated_ros_review.py','audit_fluency_120s.py','audit_target_road.py'):shutil.copy2(base/name,new/name)
scope=dict(runtime_delta=runtime,tests_delta=tests,source='Existing v65 implementation, no new arrival mechanism.',
    buffer_experiment_absent=True,continuous_control_unchanged=True,no_new_runtime_components=True,
    arrival='Original distance-driven completion; retain user-specified 2m body clearance, not upstream default 3m 3D sphere.',
    search_s=120,members=len(manifest['members']))
(new/'candidate_scope.json').write_text(json.dumps(scope,indent=2));print(json.dumps(scope,indent=2))
