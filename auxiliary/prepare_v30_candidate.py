import hashlib,json,shutil
from pathlib import Path
art=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
base=art/'target_identity_repair_20261025/candidate_r2';out=art/'confirmation_window_repair_20261025';target=out/'candidate'
assert not target.exists()
names=set(json.loads((base.parent/'r2_base_hashes.json').read_text()))
hashes={}
for n in names:
    p=target/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(base/n,p)
    hashes[n]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
shutil.copy2(out/'localization_messages.json',target/'nav2_adapter/test/fixtures/v29_r2_sparse_localizations.json')
print(len(hashes))
