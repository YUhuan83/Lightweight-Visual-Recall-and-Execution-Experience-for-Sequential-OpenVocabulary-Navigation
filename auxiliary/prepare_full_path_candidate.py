import hashlib,json,shutil
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真')
source=repo/'test_artifacts/confirmed_vehicle_repair_20261026/candidate'
root=repo/'test_artifacts/full_path_control_20261026'
root.mkdir(exist_ok=False)
candidate=root/'candidate'
shutil.copytree(source,candidate,ignore=shutil.ignore_patterns('__pycache__'))
manifest={p.relative_to(candidate).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
          for p in candidate.rglob('*') if p.is_file()}
(root/'base_hashes.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps(dict(base=str(source),candidate=str(candidate),files=len(manifest))))
