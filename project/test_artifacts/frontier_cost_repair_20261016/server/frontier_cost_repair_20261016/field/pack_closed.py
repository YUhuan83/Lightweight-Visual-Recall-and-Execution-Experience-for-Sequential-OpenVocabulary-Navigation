from pathlib import Path
import json,hashlib,zipfile,time
r=Path('/home/ly/wildos-console');p=Path('/home/ly/frontier_cost_repair_20261016/field')
t=sorted(r.glob('field_frontier_cost_search_*'))[-1]
assert (t/'result.json').exists(),'Active trial'
result=json.loads((t/'result.json').read_text());s=Path(result['session'])
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists(),'Active session'
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
 return h.hexdigest()
files={x for d in (t,s) for x in d.rglob('*') if x.is_file()}
files.update(x for x in p.iterdir() if x.is_file() and x.suffix in ('.json','.py','.sh','.log') and x.name not in ('postprocess.log','pack.log'))
files.add(r/'source_identity.json');manifest={};external={}
out=r/(t.name+'_core.zip')
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as archive:
 for f in sorted(files):
  name=f.relative_to(Path('/home/ly')).as_posix();sha=digest(f)
  if 'sensor_interfaces' in f.parts and f.suffix=='.db3':
   external[name]=dict(path=str(f),sha256=sha,bytes=f.stat().st_size);continue
  archive.write(f,name);manifest[name]=sha
 archive.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
 archive.writestr('EXTERNAL_RAW_SENSOR_MANIFEST.json',json.dumps(dict(scope='Closed raw pointcloud and topology SQLite retained at original server paths; omitted only from local core download',members=external),indent=2))
meta=dict(archive=str(out),sha256=digest(out),members=len(manifest),external_members=len(external),bytes=out.stat().st_size)
(p/'core_meta.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
