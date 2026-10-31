import json,hashlib,zipfile,time
from pathlib import Path
r=Path('/home/ly/wildos-console');files=set()
for name in ('session_20261015_151045','session_20261015_151859'):
 s=r/'results'/name;assert (s/'session_result.json').exists(),name
 files.update(p for p in s.rglob('*') if p.is_file() and p.suffix not in ('.mp4','.tmp'))
for name in ('field_search_20261015_151044','field_search_20261015_151859'):
 s=r/name;assert (s/'result.json').exists(),name
 files.update(p for p in s.rglob('*') if p.is_file())
for name in ('source_identity.json','source_identity_ff0fa6e5.json','source_identity_ffcfb67d.json','collision_build_test.log','collision_build_test_final.log','collision_gate_passed.json','console.json'):
 if (r/name).is_file():files.add(r/name)
out=r/('closed_continuation_'+time.strftime('%Y%m%d_%H%M%S')+'.zip');m={}
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
 for p in sorted(files):
  b=p.read_bytes();n=str(p.relative_to(r));m[n]=hashlib.sha256(b).hexdigest();z.writestr(n,b)
 z.writestr('MANIFEST.json',json.dumps(m,indent=2))
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),files=len(m),mp4_stored_separately=True)))
