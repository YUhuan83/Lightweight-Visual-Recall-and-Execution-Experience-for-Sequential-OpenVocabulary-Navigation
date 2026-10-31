"""Freeze this delta against the last fully identified 416-file candidate."""
from pathlib import Path
import hashlib
import json
import zipfile

out = Path(__file__).resolve().parent
repo = out.parents[1]
with zipfile.ZipFile(out.parent/'graph_start_attachment_repair_20261020/sealed/graph_start_evidence.zip') as z:
    previous = json.loads(z.read('frozen_source_manifest.json'))
files = ['nav2_adapter/wildos_nav2_adapter/'+n+'.py' for n in
         ('confirmed_approach', 'target_observation', 'graph_path_bridge', 'visible_approach')]
files += ['official_adapter/beamng_wildos_adapter/goal_standoff.py',
          'nav2_adapter/test/test_confirmed_recovery_ros.py', 'tools/offline_callback_check.py']
delta = {n: hashlib.sha256((repo/n).read_bytes()).hexdigest() for n in files}
unexpected = [n for n, h in previous['members'].items()
              if hashlib.sha256((repo/n).read_bytes()).hexdigest() != h and n not in delta]
assert not unexpected, unexpected
manifest = dict(parent_manifest=previous, delta=delta, production_changed_files=files[:5])
(out/'delta_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
with zipfile.ZipFile(out/'candidate_delta.zip', 'w', zipfile.ZIP_DEFLATED) as z:
    for name in files:
        z.write(repo/name, 'delta/'+name)
    for p in (out/'before').rglob('*.py'):
        z.write(p, str(p.relative_to(out)).replace('\\', '/'))
    for p in out.glob('*.py'):
        z.write(p, p.name)
    z.write(out/'delta_manifest.json', 'delta_manifest.json')
print(json.dumps(dict(files=len(delta), parent_members=len(previous['members']),
                     zip_sha256=hashlib.sha256((out/'candidate_delta.zip').read_bytes()).hexdigest())))
