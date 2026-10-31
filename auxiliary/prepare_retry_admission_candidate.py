import hashlib,json,shutil
from pathlib import Path
art=Path(__file__).parent.parent/'拟真/test_artifacts'
source=art/'confirmed_progress_repair_20261025'
out=art/'retry_admission_repair_20261025';target=out/'candidate'
assert not target.exists()
names=set(json.loads((source/'base_hashes.json').read_text()))
names.update(json.loads((source/'source_scope.json').read_text())['changed'])
hashes={}
for name in sorted(names):
    p=target/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'candidate'/name,p)
    hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
print(len(hashes))
