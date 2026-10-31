import json,subprocess
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v28_intent_handoff')
assert json.loads((root/'perception_report.json').read_text())['passed']
assert all(r['passed'] for r in json.loads((root/'real_ros_report.json').read_text()))
previous=set((root/'full').glob('trial_*'))
with (root/'field_launch.log').open('w') as log:
    run=subprocess.run(['python3',str(root/'launch_trial.py')],stdout=log,stderr=subprocess.STDOUT)
created=set((root/'full').glob('trial_*'))-previous
assert len(created)<=1
post=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not post['processes'] and not post['display148_lock'],post
out=dict(returncode=run.returncode,postcheck=post,trial=str(next(iter(created))) if created else None)
if created:
    trial=next(iter(created));result=json.loads((trial/'result.json').read_text())
    assert result['closed'];out['result']=result
    if list(trial.glob('sessions/*/goal_001/stop.json')):
        with (root/'field_metrics.log').open('w') as log:
            proc=subprocess.run(['bash','/tmp/audit_closed_v19.sh',str(trial)],stdout=log,stderr=subprocess.STDOUT)
        out['metric_audit_exit']=proc.returncode
        for name in ('fluency_summary.json','control_stop_attribution.json'):
            if (trial/name).exists():out[name]=json.loads((trial/name).read_text())
(root/'field_result.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out),flush=True)
