import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v28_intent_handoff')
assert not list((root/'full').glob('trial_*')),'Never mutate a tested frozen candidate'
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
receipt=json.loads((root/'receipt.json').read_text())
shutil.copy2(root/'receipt.json',root/'receipt_stage1.json')
with zipfile.ZipFile('/tmp/lookback_removal_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    for n,h in expected.items():assert hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h,n
    for n in expected:
        (root/'candidate'/n).write_bytes(z.read(n))
        manifest['members'][n]=hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()
        receipt['changed'][n]=manifest['members'][n]
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
(root/'receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt))
