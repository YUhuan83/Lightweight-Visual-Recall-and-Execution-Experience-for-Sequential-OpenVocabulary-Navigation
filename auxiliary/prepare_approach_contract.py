import hashlib,json,shutil
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';source=repo/'test_artifacts/instance_handoff_20261026/candidate'
root=repo/'test_artifacts/approach_contract_20261026';root.mkdir(exist_ok=False)
shutil.copytree(source,root/'candidate',ignore=shutil.ignore_patterns('__pycache__','test_artifacts'))
fixture_root=root/'candidate/test_artifacts/server_validation_20261002';fixture_root.mkdir(parents=True)
for name in ('house_localization_replay.json','accepted_memory_prefix_snapshot.npz','prefix_replay.json'):
    shutil.copy2(source/'test_artifacts/server_validation_20261002'/name,fixture_root/name)
(root/'base_hashes.json').write_text(json.dumps({p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts and 'test_artifacts' not in p.relative_to(source).parts},indent=2),encoding='utf8')
print(root)
