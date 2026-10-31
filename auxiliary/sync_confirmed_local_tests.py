import hashlib,json,shutil,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v56_confirmed_local_continuity'
assert not (root/'field_result.json').exists()
for n in ('real_ros_review.log','real_ros_report.json'):
    p=root/n
    if p.exists() and not (root/('initial_'+n)).exists():shutil.copy2(p,root/('initial_'+n))
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
receipt=json.loads((root/'receipt.json').read_text())
with zipfile.ZipFile(base/'confirmed_local_continuity_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    for n in z.namelist():
        if n=='base_hashes.json':continue
        p=root/'candidate'/n;data=z.read(n);new=hashlib.sha256(data).hexdigest()
        if p.exists():assert hashlib.sha256(p.read_bytes()).hexdigest() in (expected.get(n),new,receipt['changed'].get(n)),n
        p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        manifest['members'][n]=new;receipt['changed'][n]=new
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
(root/'receipt.json').write_text(json.dumps(receipt,indent=2))
print('Updated identified-confirmation assertions; preserved initial failed test report.')
