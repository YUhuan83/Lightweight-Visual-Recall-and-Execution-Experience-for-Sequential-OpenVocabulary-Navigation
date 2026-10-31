"""Freeze a coherent current source candidate; do not replace the 6153 bundle."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
baseline = json.loads((ROOT / 'official_wildos_beamng_bundle.manifest.json').read_text())
assert baseline['sha256'] == '6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
prior = json.loads((ROOT / 'test_artifacts/physical_evidence_repair_20261019/candidate6/candidate_delta.json').read_text())
delta = json.loads((OUT / 'source_manifest.json').read_text())
names = set(baseline['members']) | set(prior['members']) | set(delta)
manifest = dict(baseline_bundle=baseline['sha256'], current_source=True,
                production_deployment=False, complete_navigation_passed=False,
                members={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                         for name in sorted(names)})
for name, want in delta.items():
    assert manifest['members'][name] == want, name + ': edited since ROS validation'
path = OUT / 'connected_ground_candidate.zip'
with zipfile.ZipFile(path, 'x', zipfile.ZIP_DEFLATED) as package:
    for name in sorted(names):
        package.write(ROOT / name, name)
    package.writestr('frozen_source_manifest.json', json.dumps(manifest, indent=2) + '\n')
(OUT / 'frozen_source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
print(json.dumps(dict(files=len(names), bytes=path.stat().st_size,
                      sha256=hashlib.sha256(path.read_bytes()).hexdigest())))
