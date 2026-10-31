import hashlib,json,shutil
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\target_identity_repair_20261025')
source=root/'candidate';target=root/'candidate_r2';assert not target.exists()
names=set(json.loads((root/'base_hashes.json').read_text()))|set(json.loads((root/'delta_receipt.json').read_text())['changed'])
for n in names:
    p=target/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/n,p)
(root/'r2_base_hashes.json').write_text(json.dumps({n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in names},indent=2))
print(len(names))
