from pathlib import Path
import hashlib,json,shutil
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
baseline=json.loads((root/'deploy/20261016_exploration_scope_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
changed=sorted(n for n,h in manifest['members'].items() if baseline['members'].get(n)!=h)
assert len(changed)==4,changed
delta=dict(base=baseline['sha256'],candidate=manifest['sha256'],changed=changed,
    scope='Resolve an explicitly non-frontier exploration intention; fresh/ordered snapshot and existing cancellation fences; no arrival, model, speed, physics, controller or recall changes')
for name in changed:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==manifest['members'][name]
freeze=root/'deploy/20261016_frontier_resolution_candidate';freeze.mkdir(exist_ok=False)
for name in ('official_wildos_beamng_bundle.zip','official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    shutil.copyfile(root/name,freeze/name)
(out/'delta.json').write_text(json.dumps(delta,indent=2)+'\n',encoding='utf8')
print(json.dumps(delta,indent=2))
