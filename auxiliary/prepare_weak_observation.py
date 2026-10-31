import hashlib,json,shutil
from pathlib import Path
art=Path(__file__).parent.parent/'拟真/test_artifacts'
source=art/'confirmed_vehicle_repair_20261026';root=art/'weak_observation_budget_20261026';target=root/'candidate'
assert not target.exists()
names=set(json.loads((source/'base_hashes.json').read_text(encoding='utf8')))
names.update(json.loads((source/'source_scope.json').read_text(encoding='utf8'))['changed'])
base={}
for name in sorted(names):
    p=target/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'candidate'/name,p)
    base[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'base_hashes.json').write_text(json.dumps(base,indent=2),encoding='utf8')
print(target)
