import hashlib,json,shutil
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
base=root/'execution_handoff_repair_20261025/candidate';out=root/'target_identity_repair_20261025';target=out/'candidate'
assert not target.exists()
names=json.loads((base.parent/'base_hashes.json').read_text())
hashes={}
for n in names:
    p=target/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(base/n,p)
    hashes[n]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
fixtures=target/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/fixtures'
for p in out.glob('depth_fixture_*.npz'):shutil.copy2(p,fixtures/p.name)
shutil.copy2(out/'localization_before.json',fixtures/'v28_localization_callback.json')
print(json.dumps(dict(base=str(base),candidate=str(target),members=len(hashes))))
