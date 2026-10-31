"""Freeze an isolated Python component check, without replacing a deployment."""
import hashlib
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[2]
out = Path(__file__).parent
paths = sorted((root/'nav2_adapter/wildos_nav2_adapter').glob('*.py'))
paths += [root/'nav2_adapter/test'/p for p in (
    'test_execution_progress_ros.py', 'test_tracking_handoff_ros.py',
    'test_execution_mechanism_ros.py', 'test_graph_path_bridge_routing.py')]
paths += sorted((root/'nav2_adapter/test/fixtures').glob('*.json'))
manifest = {str(p.relative_to(root)).replace('\\', '/'): dict(
    bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths}
with zipfile.ZipFile(out/'component.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
    for p in paths:
        archive.write(p, str(p.relative_to(root)).replace('\\', '/'))
    archive.writestr('component_manifest.json', json.dumps(manifest, indent=2))
(out/'component_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
print(json.dumps(dict(members=len(manifest), sha256=hashlib.sha256((out/'component.zip').read_bytes()).hexdigest())))
