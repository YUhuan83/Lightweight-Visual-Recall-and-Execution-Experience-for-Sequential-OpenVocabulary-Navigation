"""Run frozen A/B/A/B trials sequentially; audit only closed, cleared trials."""
import json
import subprocess
import time
from pathlib import Path

root=Path('/home/ly/execution_continuity_review_20261022')
output=root/'stop_plan_ab_20261024'
output.mkdir(exist_ok=False)
receipt=[]
for version in ('v19_dispatch_reuse','v20_stop_plan_overlap','v19_dispatch_reuse','v20_stop_plan_overlap'):
    candidate=root/version
    previous=set((candidate/'full').glob('trial_*'))
    print(json.dumps(dict(event='launch',version=version,wall=time.time())),flush=True)
    run=subprocess.run(['python3',str(candidate/'launch_trial.py')],
                       stdout=(output/f'launch_{len(receipt)+1}.log').open('w'),stderr=subprocess.STDOUT)
    created=set((candidate/'full').glob('trial_*'))-previous
    assert len(created)<=1,created
    pre=subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],
                       check=True,capture_output=True,text=True,timeout=20)
    post=json.loads(pre.stdout)
    assert not post['processes'] and not post['display148_lock'],post
    trial=next(iter(created),None)
    record=dict(version=version,returncode=run.returncode,trial=str(trial) if trial else None,postcheck=post)
    if trial:
        result=json.loads((trial/'result.json').read_text())
        assert result['closed'],result
        record['result']=result
        if list(trial.glob('sessions/*/goal_001/stop.json')):
            subprocess.run(['bash','/tmp/audit_closed_v19.sh',str(trial)],check=True)
            record['fluency']=json.loads((trial/'fluency_summary.json').read_text())
            record['attribution']=json.loads((trial/'control_stop_attribution.json').read_text())
    receipt.append(record)
    (output/'results.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(dict(event='closed',version=version,trial=record['trial'],
                         near_stop_pct=record.get('fluency',{}).get('near_stop_pct'),
                         search_elapsed_sec=record.get('result',{}).get('search_elapsed_sec'),
                         complete_navigation_passed=record.get('result',{}).get('complete_navigation_passed'))),flush=True)
print(json.dumps(dict(event='all_closed',output=str(output))),flush=True)
