import hashlib,json,shutil
from pathlib import Path
artifacts=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
base=artifacts/'fresh_bearing_repair_20261025/candidate'
out=artifacts/'execution_handoff_repair_20261025'
target=out/'candidate'
assert not target.exists()
names=set(json.loads((base.parent/'base_hashes.json').read_text()))
names.add('wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_fresh_bearing.py')
for n in names:
    dest=target/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(base/n,dest)
hashes={n:hashlib.sha256((base/n).read_bytes()).hexdigest() for n in sorted(names)}
(out/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
print(len(hashes))
