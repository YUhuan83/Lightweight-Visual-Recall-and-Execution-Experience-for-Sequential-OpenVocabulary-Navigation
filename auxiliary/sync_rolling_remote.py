import hashlib,json,shutil,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v38_rolling_observation'
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
for n,h in manifest['members'].items():assert hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h,n
saved=root/'failed_clock_probe';saved.mkdir(exist_ok=False)
for name in ('field_result.json','trial_frozen.json'):
    shutil.copy2(root/name,saved/name)
with zipfile.ZipFile(base/'rolling_delta.zip') as z:
    names=[n for n in z.namelist() if n!='base_hashes.json']
    with zipfile.ZipFile(saved/'previous_sources.zip','w',zipfile.ZIP_DEFLATED) as old:
        for n in names:
            p=root/'candidate'/n
            if p.exists():old.write(p,n)
    for n in names:
        p=root/'candidate'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
        manifest['members'][n]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
print('Saved failed clock probe and synchronized isolated candidate')
