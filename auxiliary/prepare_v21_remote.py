"""Create an isolated v21 input diagnostics from v19 with a verified two-file delta."""
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

ROOT=Path('/home/ly/execution_continuity_review_20261022')
SOURCE=ROOT/'v19_dispatch_reuse'
TARGET=ROOT/'v21_planning_inputs'
MEMBERS=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/test/test_graph_path_bridge_routing.py']

assert SOURCE.is_dir() and not TARGET.exists()
with zipfile.ZipFile('/tmp/v21_delta.zip') as package:
    assert set(package.namelist()) == set(MEMBERS+['base_hashes.json'])
    base=json.loads(package.read('base_hashes.json'))
    for name in MEMBERS:
        assert hashlib.sha256((SOURCE/'candidate'/name).read_bytes()).hexdigest()==base[name],name
    TARGET.mkdir()
    shutil.copytree(SOURCE/'candidate',TARGET/'candidate')
    for name in MEMBERS:
        (TARGET/'candidate'/name).write_bytes(package.read(name))

full=TARGET/'full'
full.mkdir()
for item in (SOURCE/'full').iterdir():
    if item.is_file() and (item.suffix=='.py' or item.name in ('config.json','frozen_source_manifest.json')):
        shutil.copy2(item,full/item.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):
    shutil.copy2(SOURCE/name,TARGET/name)
for path in [TARGET/name for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py')]+list(full.glob('*.py'))+[full/'config.json']:
    path.write_text(path.read_text().replace(str(SOURCE),str(TARGET)))
manifest_path=full/'frozen_source_manifest.json'
manifest=json.loads(manifest_path.read_text())
for name in MEMBERS:
    manifest['members'][name]=hashlib.sha256((TARGET/'candidate'/name).read_bytes()).hexdigest()
manifest_path.write_text(json.dumps(manifest,indent=2))
receipt=dict(source=str(SOURCE),target=str(TARGET),base='frozen_v19',
    changed_members={name:manifest['members'][name] for name in MEMBERS},member_count=len(manifest['members']))
(TARGET/'candidate_delta_v21.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2))
