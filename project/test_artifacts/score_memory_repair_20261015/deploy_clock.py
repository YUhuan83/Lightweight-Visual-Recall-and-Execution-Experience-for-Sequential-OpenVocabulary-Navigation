import json,hashlib,zipfile,subprocess
from pathlib import Path
r=Path('/home/ly/wildos-console');a=Path('/home/ly/wildos_console_ef3fc7f8.zip')
expected='ef3fc7f814683f640e2d4a93dfc5ef601f76f7f1a6e85e56187631f8fec5a54b'
assert hashlib.sha256(a.read_bytes()).hexdigest()==expected
old=json.loads((r/'source_identity.json').read_text());assert old['sha256'].startswith('042ee7af')
allowed=['wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_scoring_execution_contract.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py']
with zipfile.ZipFile(a) as z:
 assert z.testzip() is None
 hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
 changed={n for n in set(hashes)|set(old['members']) if hashes.get(n)!=old['members'].get(n)}
 assert changed==set(allowed),changed
 (r/'bundle').rename(r/'bundle_042ee7af');(r/'bundle').mkdir();z.extractall(r/'bundle')
 assert all(hashlib.sha256((r/'bundle'/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
(r/'source_identity.json').rename(r/'source_identity_042ee7af.json')
(r/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes,changed=sorted(changed)),indent=2))
(r/'score_memory_gate_passed.json').rename(r/'score_memory_gate_042ee7af.json')
with (r/'score_memory_build_clock.log').open('xb') as log:
 child=subprocess.Popen(['bash','/home/ly/score_memory_build_20261015.sh'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(child.pid)
