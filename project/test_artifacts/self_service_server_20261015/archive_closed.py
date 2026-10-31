"""Archive only closed field stages; active-run files are deliberately excluded."""
import hashlib,json,time,zipfile
from pathlib import Path
root=Path('/home/ly/wildos-console')
out=root/('closed_evidence_'+time.strftime('%Y%m%d_%H%M%S')+'.zip')
files=set()
for name in ('source_identity.json','source_identity_5c6e501c.json','source_identity_17adb4ae.json',
             'source_identity_21fd3518.json','build.log','build_colcon_missing.log','installer_contract.log','console.json'):
    if (root/name).is_file():files.add(root/name)
for parent in (root, root/'results'):
    for folder in parent.iterdir():
        if folder.is_dir() and (folder.name.startswith(('field_','scene_probe_','session_'))):
            marker='session_result.json' if folder.name.startswith('session_') else 'result.json'
            if (folder/marker).is_file():
                files.update(p for p in folder.rglob('*') if p.is_file())
manifest={}
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
    for path in sorted(files):
        data=path.read_bytes();name=str(path.relative_to(root))
        manifest[name]=hashlib.sha256(data).hexdigest();z.writestr(name,data)
    z.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
print(json.dumps(dict(archive=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),files=len(files))))
