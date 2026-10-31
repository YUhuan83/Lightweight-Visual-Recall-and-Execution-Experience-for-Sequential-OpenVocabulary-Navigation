"""Verify the new diagnostic package uses the identical installed runtime."""
import hashlib,json,zipfile
from pathlib import Path
base=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
out=base/'resume_session1'
manifest=json.loads((out/'official_wildos_beamng_bundle.manifest.json').read_text())
archive=out/'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['sha256']
root=(out/'source').resolve();root.mkdir(exist_ok=False)
old=json.loads((base/'candidate.manifest.json').read_text())
changed=[]
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and set(z.namelist())==set(manifest['members'])
    for name,digest in manifest['members'].items():
        data=z.read(name);assert hashlib.sha256(data).hexdigest()==digest
        path=(root/name).resolve();assert path.is_relative_to(root)
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
        if old['members'].get(name)!=digest:changed.append(name)
assert all(name.startswith('tools/') for name in changed),changed
identity=json.loads((base/'installation_identity.json').read_text())
for name,digest in identity['files'].items():
    if name.endswith('.so'):path=base/'install/lib'/name
    elif name.endswith('.yaml'):path=base/'install/share/wildos_nav2_adapter/config'/name
    else:path=base/'install/lib/python3.10/site-packages/wildos_nav2_adapter'/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
report={'passed':True,'diagnostic_package_sha256':manifest['sha256'],
    'runtime_package_sha256':old['sha256'],'changed_files':changed,'installed_runtime':identity}
(out/'source_identity.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='installed_runtime'},indent=2))
