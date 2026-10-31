from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/exploration_efficiency_audit_20261016');p=Path('/home/ly/wildos-console')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
actual={}
for name in ('graph_path_bridge.py','execution_recovery.py'):
 f=r/'install/lib/python3.10/site-packages/wildos_nav2_adapter'/name
 h=hashlib.sha256(f.read_bytes()).hexdigest();assert h==m['members']['nav2_adapter/wildos_nav2_adapter/'+name];actual[str(f)]=h
old=json.loads((p/'source_identity.json').read_text());assert old['sha256']=='11af6c17b0335766c50ce892bf128af5ab88f9a7a82571e0799f3e555e06d9cd'
for n,h in old['members'].items():assert hashlib.sha256((p/'bundle'/n).read_bytes()).hexdigest()==h,n
identity=dict(candidate=m['sha256'],production=old['sha256'],production_unchanged=True,installed_source_hashes=actual,
 smac_sha256=hashlib.sha256((p/'install/lib/libwildos_ackermann_smac.so').read_bytes()).hexdigest(),vehicle_run=False)
(r/'validation_identity.json').write_text(json.dumps(identity,indent=2))
files=[f for f in r.iterdir() if f.is_file() and f.suffix in ('.py','.sh','.json','.log','.txt','.csv','.error')]
manifest={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
with zipfile.ZipFile(r/'evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
 for f in files:z.write(f,f.name)
 z.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
meta=dict(sha256=hashlib.sha256((r/'evidence.zip').read_bytes()).hexdigest(),members=len(manifest),scope='Closed map/planner and ROS component evidence, not a physical recovery run')
(r/'evidence_meta.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
