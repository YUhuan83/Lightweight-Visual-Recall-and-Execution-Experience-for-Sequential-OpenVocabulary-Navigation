import hashlib,json,shutil
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真'
source=repo/'test_artifacts/full_path_control_20261026/candidate'
root=repo/'test_artifacts/rolling_observation_20261026';root.mkdir(exist_ok=False)
shutil.copytree(source,root/'candidate',ignore=shutil.ignore_patterns('__pycache__'))
manifest={p.relative_to(root/'candidate').as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (root/'candidate').rglob('*') if p.is_file() and not p.relative_to(root/'candidate').as_posix().startswith('test_artifacts/')}
(root/'base_hashes.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(root)
