from pathlib import Path
import shutil,hashlib,json
repo=Path(r'C:\Users\Yu\Desktop\拟真');source=repo/'test_artifacts/depth_geometry_20261027/candidate';out=repo/'test_artifacts/range_candidate_20261027';out.mkdir(exist_ok=False)
shutil.copytree(source,out/'candidate',ignore=shutil.ignore_patterns('__pycache__','test_artifacts'))
f=out/'candidate/test_artifacts/server_validation_20261002';f.mkdir(parents=True)
for n in ('house_localization_replay.json','accepted_memory_prefix_snapshot.npz','prefix_replay.json'):shutil.copyfile(source/'test_artifacts/server_validation_20261002'/n,f/n)
(out/'base_hashes.json').write_text(json.dumps({p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts and 'test_artifacts' not in p.relative_to(source).parts}),encoding='utf8')
