import hashlib,json,shutil
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';base=repo/'test_artifacts/full_path_control_20261026/candidate'
prior=repo/'test_artifacts/rolling_observation_20261026/candidate';root=repo/'test_artifacts/observation_session_20261026'
root.mkdir(exist_ok=False);candidate=root/'candidate'
shutil.copytree(base,candidate,ignore=shutil.ignore_patterns('__pycache__'))
manifest={p.relative_to(candidate).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
          for p in candidate.rglob('*') if p.is_file() and not p.relative_to(candidate).as_posix().startswith('test_artifacts/')}
(root/'base_hashes.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
for n in ('nav2_adapter/wildos_nav2_adapter/visible_approach.py','tools/town_trial_harness.py','tools/test_town_trial_harness.py','tools/wildos_console.py','tools/test_wildos_console.py'):
    shutil.copy2(prior/n,candidate/n)
print(root)
