import hashlib,json,re
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/observation_heading_20261026/weak_admission_probe'
inputs=json.loads((root/'inputs.json').read_text(encoding='utf8'));report=[]
for row in inputs['rows']:
    runs=[];hashes=[]
    for repeat in (1,2):
        file=root/(Path(row['file']).name+f'.run_{repeat}.log')
        text=file.read_text(encoding='utf8');hashes.append(hashlib.sha256(file.read_bytes()).hexdigest())
        result=[]
        for line in text.splitlines():
            if not line.startswith('OBS_RESULT'):continue
            m=re.match(r'OBS_RESULT variant=(\d+) ok=(\d+) (.*) elapsed=',line);assert m,line
            result.append(dict(variant=int(m[1]),ok=m[2]=='1',detail=m[3]))
        assert len(result)==3;assert '[  PASSED  ] 1 test.' in text
        runs.append(result)
    assert runs[0]==runs[1],runs
    report.append(dict(**row,results=runs[0],both_runs_identical_except_timing=True,log_sha256=hashes))
out=dict(scope=inputs['scope'],rows=report,
    conclusion='Of three no_vehicle_path snapshots, two fail actual Smac for all three tested yaws. The circle-rejected 11.977m case is plannable with 1 or 2 cusps; forward-only reachability is not proven. Fallback yaw mismatch is an interface counterexample, not evidence of faster planning in its recorded case.')
(root.parent/'weak_admission_summary.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(dict(rows=len(report),repeat_match=True,conclusion=out['conclusion']),indent=2))
