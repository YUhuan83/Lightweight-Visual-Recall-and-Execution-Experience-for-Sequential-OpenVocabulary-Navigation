import hashlib, json, shutil
from pathlib import Path

base = Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\arrival_decoupling_20261025\candidate')
out = Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\fresh_bearing_repair_20261025')
target = out/'candidate'
assert not target.exists()
manifest = json.loads((base/'_frozen_source_manifest.json').read_text())
for name in manifest['members']:
    source = base/name
    dest = target/name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, dest)
hashes = {name: hashlib.sha256((target/name).read_bytes()).hexdigest()
          for name in manifest['members']}
(out/'base_hashes.json').write_text(json.dumps(hashes, indent=2))
(target/'_frozen_source_manifest.json').write_text(json.dumps(dict(members=hashes), indent=2))
print(json.dumps(dict(files=len(hashes), candidate=str(target))))
