import hashlib,json,zipfile,subprocess
from pathlib import Path
r=Path('/home/ly/wildos-console');a=Path('/home/ly/wildos_console_751218fd.zip')
h='751218fde4ef9575a606b53e440719c6b737bb2840d54fa2ebaebe0738527f8c'
assert hashlib.sha256(a.read_bytes()).hexdigest()==h
old=json.loads((r/'source_identity.json').read_text());assert old['sha256'].startswith('ffcfb67d')
with zipfile.ZipFile(a) as z:
 hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
 assert {n for n in hashes if hashes[n]!=old['members'].get(n)}=={'nav2_adapter/test/test_graph_path_bridge_routing.py'}
 (r/'bundle').rename(r/'bundle_ffcfb67d');(r/'bundle').mkdir();z.extractall(r/'bundle')
(r/'source_identity.json').rename(r/'source_identity_ffcfb67d.json')
(r/'source_identity.json').write_text(json.dumps(dict(sha256=h,members=hashes,installation_unchanged=True),indent=2))
with (r/'collision_build_test_final.log').open('xb') as log:
 child=subprocess.Popen(['bash',str(r/'collision_build_test.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(child.pid)
