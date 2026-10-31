from pathlib import Path
import zipfile,json,hashlib,shutil
r=Path('/home/ly/wildos-console');full=r/'late_extension_field_20261015_213429.zip';out=r/'late_extension_core_20261015_213429.zip'
with zipfile.ZipFile(full) as src,zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as dst:
 manifest=json.loads(src.read('MANIFEST.json'));selected={};external={}
 for name,h in manifest.items():
  if '/sensor_interfaces/' in name and name.endswith('.db3'):
   external[name]=h;continue
  with src.open(name) as f,dst.open(name,'w') as to:shutil.copyfileobj(f,to,8*1024*1024)
  selected[name]=h
 dst.writestr('MANIFEST.json',json.dumps(selected,indent=2))
 dst.writestr('EXTERNAL_RAW_SENSOR_MANIFEST.json',json.dumps(dict(scope='Raw pointcloud SQLite retained in complete server archive, excluded only from local core download',complete_archive=str(full),external_members=external),indent=2))
h=hashlib.sha256()
with out.open('rb') as f:
 for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
print(json.dumps(dict(archive=str(out),sha256=h.hexdigest(),files=len(selected),external_raw_members=len(external)),indent=2))
