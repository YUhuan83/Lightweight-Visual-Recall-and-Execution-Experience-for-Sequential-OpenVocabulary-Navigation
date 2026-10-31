"""Freeze the route-retirement handoff repair over the verified v4 source."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent
project = root.parents[1]
previous = json.loads((root / 'delta_manifest_v4.json').read_text(encoding='utf8'))
names = [
    'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    'nav2_adapter/test/test_committed_frontier_path_ros.py',
]
delta = {name: hashlib.sha256((project / name).read_bytes()).hexdigest()
         for name in names}
members = dict(previous['members'])
members.update(delta)
manifest = dict(parent=dict(members=previous['members']), delta=delta, members=members)
(root / 'delta_manifest_v5.json').write_text(
    json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
archive = root / 'candidate_delta_v5.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zf:
    for name in names:
        zf.write(project / name, 'delta/' + name)
    zf.write(root / 'delta_manifest_v5.json', 'delta_manifest_v5.json')
print(json.dumps(dict(members=len(members), changed=len(delta),
                      sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                      files=names)))
