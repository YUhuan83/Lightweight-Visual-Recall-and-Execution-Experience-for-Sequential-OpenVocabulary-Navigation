from pathlib import Path
import hashlib,json,yaml

root=Path('/home/ly/frontier_resolution_repair_20261016')
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text())
assert manifest['sha256']=='274c3ef2b5cecd1522579ccd4eea2b5a3c4bab649f5e1669c9d79226de7079a6'
for name,want in manifest['members'].items():
    assert hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()==want,name
harness=json.loads((root/'field/harness_manifest.json').read_text())
for name,want in harness['files'].items():
    assert hashlib.sha256((root/'field'/name).read_bytes()).hexdigest()==want,name
original=yaml.safe_load((root/'candidate/official_adapter/config/beamng_wildos.yaml').read_text())
baseline=yaml.safe_load((root/'field/baseline_no_memory.yaml').read_text())
for name in harness['memory_override_only']:
    assert name not in original and baseline.pop(name) is False,name
assert baseline==original
production=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
assert production['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
for name,want in production['members'].items():
    assert hashlib.sha256((Path('/home/ly/wildos-console/bundle')/name).read_bytes()).hexdigest()==want,name
report=dict(candidate=manifest['sha256'],candidate_members=len(manifest['members']),
    production=production['sha256'],production_members=len(production['members']),
    baseline_only_changed=harness['memory_override_only'],harness_verified=True)
(root/'field/prelaunch_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
