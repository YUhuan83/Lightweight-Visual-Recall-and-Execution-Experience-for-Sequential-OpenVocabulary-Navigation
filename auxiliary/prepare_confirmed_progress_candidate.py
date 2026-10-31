import hashlib,json,shutil
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真'
base=repo/'test_artifacts/execution_handoff_repair_20261025'
output=repo/'test_artifacts/confirmed_progress_repair_20261025'
candidate=output/'candidate'
assert not candidate.exists()
names=json.loads((base/'base_hashes.json').read_text())
hashes={}
for name in names:
    source=base/'candidate'/name
    target=candidate/name;target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,target)
    hashes[name]=hashlib.sha256(target.read_bytes()).hexdigest()
(output/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
print(json.dumps(dict(candidate=str(candidate),files=len(hashes))))
