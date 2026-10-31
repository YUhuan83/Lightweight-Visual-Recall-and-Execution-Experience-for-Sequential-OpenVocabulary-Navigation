import json,hashlib,zipfile,subprocess
from pathlib import Path
r=Path('/home/ly/wildos-console');a=Path('/home/ly/wildos_console_bf3ee70d.zip')
expected='bf3ee70d833fd89303a24a3b365ca5ed9a14eb814c7e171affb3b1b32c4c37ab'
assert hashlib.sha256(a.read_bytes()).hexdigest()==expected
old=json.loads((r/'source_identity.json').read_text());assert old['sha256'].startswith('751218fd')
allowed=['nav2_adapter/launch/nav2_official_control.launch.py', 'nav2_adapter/package.xml', 'nav2_adapter/test/test_runtime_contract.py', 'nav2_adapter/wildos_nav2_adapter/runtime_contract.py', 'official_adapter/config/beamng_wildos.yaml', 'tools/offline_navigation_check.py', 'tools/server_test_preflight.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_scoring_execution_contract.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_visual_memory.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/geofrontier_nav/geofrontier_to_image.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/execution_experience_store.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/execution_scene_samples.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/frontier_score_state.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py']
with zipfile.ZipFile(a) as z:
 assert z.testzip() is None
 hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
 changed={n for n in set(hashes)|set(old['members']) if hashes.get(n)!=old['members'].get(n)}
 assert changed==set(allowed),changed
 (r/'bundle').rename(r/'bundle_751218fd');(r/'bundle').mkdir();z.extractall(r/'bundle')
 assert all(hashlib.sha256((r/'bundle'/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
(r/'source_identity.json').rename(r/'source_identity_751218fd.json')
(r/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes,changed=sorted(changed)),indent=2))
with (r/'score_memory_build.log').open('xb') as log:
 child=subprocess.Popen(['bash','/home/ly/score_memory_build_20261015.sh'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(child.pid)
