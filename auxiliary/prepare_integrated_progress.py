import hashlib,json,shutil,zipfile
from pathlib import Path
art=Path(__file__).parent.parent/'拟真/test_artifacts'
source=art/'retry_admission_repair_20261025';root=art/'integrated_progress_20261025';target=root/'candidate'
assert not target.exists()
names=set(json.loads((source/'base_hashes.json').read_text()))
names.update(json.loads((source/'source_scope.json').read_text())['changed'])
base={}
for name in sorted(names):
    p=target/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'candidate'/name,p)
    base[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'base_hashes.json').write_text(json.dumps(base,indent=2))
with zipfile.ZipFile(art/'confirmation_window_repair_20261025/v30_delta_from_v28.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    for name in z.namelist():
        if name=='base_hashes.json':continue
        p=target/name
        if name in expected:
            assert p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==expected[name],name
        p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
comparison=json.loads((source/'bearing_geometry_comparison.json').read_text())
fixture=target/'nav2_adapter/test/fixtures/v32_multiview_uncertainty.json'
fixture.write_text(json.dumps(comparison['replay']['v32'],indent=2))
print(str(target))
