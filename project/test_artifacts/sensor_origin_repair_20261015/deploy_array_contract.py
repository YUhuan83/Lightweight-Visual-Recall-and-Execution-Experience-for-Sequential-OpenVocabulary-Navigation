from pathlib import Path
import hashlib,json,zipfile,os
r=Path('/home/ly/wildos-console');p=Path('/home/ly/sensor_origin_repair_20261015')
m=json.loads((p/'official_wildos_beamng_bundle.manifest.json').read_text())
archive=p/'official_wildos_beamng_bundle.zip';assert hashlib.sha256(archive.read_bytes()).hexdigest()==m['sha256']
assert json.loads((r/'source_identity.json').read_text())['sha256']=='919d5c11aea2598bd0540419aa8f64af9d79b7abb8de55495a416d4d862a3544'
old=json.loads((r/'source_identity.json').read_text())['members']
allowed={'nav2_adapter/wildos_nav2_adapter/runtime_contract.py','nav2_adapter/test/test_runtime_contract.py','tools/server_test_preflight.py'}
changed={n for n,h in m['members'].items() if old.get(n)!=h};assert changed==allowed,changed
candidate=r/('bundle_candidate_'+m['sha256'][:8]);candidate.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
 assert set(z.namelist())==set(m['members'])
 for n,h in m['members'].items():
  target=candidate/n;assert target.resolve().is_relative_to(candidate.resolve());b=z.read(n);assert hashlib.sha256(b).hexdigest()==h
  target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(b)
(r/'bundle').rename(r/'bundle_919d5c11');candidate.rename(r/'bundle')
(r/'source_identity.json').rename(r/'source_identity_919d5c11.json');(r/'source_identity.json').write_text(json.dumps(m,indent=2))
(p/'deployment_delta.json').write_text(json.dumps({'source':m['sha256'],'changed':sorted(changed),'cpp_changed':False},indent=2));print('Frozen source published; build/tests still required')
