import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\target_identity_repair_20261025')
base=json.loads((root/'r2_base_hashes.json').read_text())
changed={n for n,h in base.items() if hashlib.sha256((root/'candidate_r2'/n).read_bytes()).hexdigest()!=h}
assert len(changed)==2,changed
with zipfile.ZipFile(root/'range_covariance_r2.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed}))
    for n in changed:z.write(root/'candidate_r2'/n,n)
print(json.dumps(dict(changed=sorted(changed))))
