import ast,hashlib,json,zipfile
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/observation_session_20261026';candidate=root/'candidate'
tests=(repo/'test_artifacts/rolling_observation_20261026/candidate/nav2_adapter/test/test_rolling_observation_ros.py').read_text(encoding='utf8')
(candidate/'nav2_adapter/test/test_rolling_observation_ros.py').write_text(tests.split('class FastRecoveryProbeTests')[0],encoding='utf8')
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changed={}
for p in candidate.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    n=p.relative_to(candidate).as_posix()
    if n.startswith('test_artifacts/'):continue
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    if h!=base.get(n):
        if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
        changed[n]=h
with zipfile.ZipFile(root/'observation_session_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
report=dict(base='v37_full_path',target='v39_observation_session',changed=changed,excluded='v38 early short-probe scheduling',acceptance=False)
(root/'source_scope.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2))
