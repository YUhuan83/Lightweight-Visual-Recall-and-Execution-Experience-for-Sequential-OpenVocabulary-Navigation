from pathlib import Path
import hashlib,json,shutil

root=Path(__file__).resolve().parents[2]; out=Path(__file__).resolve().parent
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
baseline=json.loads((root/'deploy/20261016_frontier_cost_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
assert baseline['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
changed=[n for n,h in manifest['members'].items() if h!=baseline['members'].get(n)]
assert len(changed)==6,changed
delta=dict(base=baseline['sha256'],candidate=manifest['sha256'],changed=changed,scope='Exploration refusal scope and deferred safe recovery; no model, physics, speed, controller or message type changes')
for name in changed:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==manifest['members'][name]
target=root/'deploy/20261016_exploration_scope_candidate'
target.mkdir(exist_ok=False)
for name in ('official_wildos_beamng_bundle.zip','official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    shutil.copyfile(root/name,target/name)
(out/'delta.json').write_text(json.dumps(delta,indent=2),encoding='utf8')
print(json.dumps(delta,indent=2))
