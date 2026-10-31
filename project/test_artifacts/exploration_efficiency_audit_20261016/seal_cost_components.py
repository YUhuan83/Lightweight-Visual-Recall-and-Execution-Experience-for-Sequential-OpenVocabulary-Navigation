from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/frontier_cost_repair_20261016')
identity=json.loads((r/'release_identity.json').read_text())
assert identity['source']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
files=[p for p in r.iterdir() if p.is_file() and p.suffix in ('.py','.sh','.json','.log')]
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(r/'component_evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in files:z.write(p,p.name)
 z.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
meta=dict(sha256=hashlib.sha256((r/'component_evidence.zip').read_bytes()).hexdigest(),members=len(manifest))
(r/'component_meta.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
