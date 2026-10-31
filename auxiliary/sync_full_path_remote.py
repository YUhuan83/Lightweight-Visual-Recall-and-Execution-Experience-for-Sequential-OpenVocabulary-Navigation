import hashlib,json,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');target=base/'v37_full_path'
manifest=json.loads((target/'full/frozen_source_manifest.json').read_text())
for n,h in manifest['members'].items():assert hashlib.sha256((target/'candidate'/n).read_bytes()).hexdigest()==h,n
number=1
while (target/f'component_attempt_{number}').exists():number+=1
saved=target/f'component_attempt_{number}';saved.mkdir()
for p in target.glob('*.log'):(saved/p.name).write_bytes(p.read_bytes())
with zipfile.ZipFile(base/'full_path_delta.zip') as z:
    names=[n for n in z.namelist() if n!='base_hashes.json']
    previous=json.loads((target/'receipt.json').read_text())['changed']
    with zipfile.ZipFile(saved/'previous_changed_sources.zip','w',zipfile.ZIP_DEFLATED) as backup:
        for n in names:
            p=target/'candidate'/n
            if p.exists():backup.write(p,n)
    for n in names:
        p=target/'candidate'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
        manifest['members'][n]=hashlib.sha256(p.read_bytes()).hexdigest()
    for n in set(previous)-set(names):
        source=base/'v35_confirmed_vehicle/candidate'/n
        assert source.is_file(),n
        p=target/'candidate'/n;p.write_bytes(source.read_bytes())
        manifest['members'][n]=hashlib.sha256(p.read_bytes()).hexdigest()
(target/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(saved=str(saved),updated=names)))
