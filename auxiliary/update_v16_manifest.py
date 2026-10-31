import hashlib,json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v16_explore_during_reobserve')
manifest_path=root/'full/frozen_source_manifest.json'
manifest=json.loads(manifest_path.read_text())
names=['nav2_adapter/wildos_nav2_adapter/visible_approach.py','nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/test/test_visible_approach_ros.py']
for name in names:
    manifest['members'][name]=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()
manifest_path.write_text(json.dumps(manifest,indent=2))
print(json.dumps({'members':len(manifest['members']),'updated':names,'hashes':{n:manifest['members'][n] for n in names}}))
