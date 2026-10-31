"""Two frozen 180s search rounds; collect only after all project owners close."""
import json,subprocess,time
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
output=root/'fast_shift_trials_20261024'
output.mkdir(exist_ok=False)
candidate=root/'v25_fast_shift'
report=json.loads((candidate/'real_ros_report.json').read_text())
assert all(r['passed'] for r in report)
receipt=[]
for index in range(2):
    previous=set((candidate/'full').glob('trial_*'))
    print(json.dumps(dict(event='launch',round=index+1,wall=time.time())),flush=True)
    with (output/f'launch_{index+1}.log').open('w') as log:
        run=subprocess.run(['python3',str(candidate/'launch_trial.py')],stdout=log,stderr=subprocess.STDOUT)
    created=set((candidate/'full').glob('trial_*'))-previous
    assert len(created)<=1,created
    post=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True,timeout=20).stdout)
    assert not post['processes'] and not post['display148_lock'],post
    trial=next(iter(created),None)
    record=dict(version=candidate.name,returncode=run.returncode,trial=str(trial) if trial else None,postcheck=post)
    started=False
    if trial:
        result=json.loads((trial/'result.json').read_text())
        assert result['closed'],result
        record['result']=result
        if list(trial.glob('sessions/*/goal_001/stop.json')):
            started=True
            subprocess.run(['bash','/tmp/audit_closed_v19.sh',str(trial)],check=True)
            record['fluency']=json.loads((trial/'fluency_summary.json').read_text())
            record['attribution']=json.loads((trial/'control_stop_attribution.json').read_text())
            goal=next(trial.glob('sessions/*/goal_001'))
            events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
            modes={e.get('execution_mode') for e in events if e.get('kind')=='plan_status'}
            record['observed_execution_modes']=sorted(m for m in modes if m)
            assert 'continuous_exploration_v1' in modes,record['observed_execution_modes']
    receipt.append(record)
    (output/'results.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(dict(event='closed',round=index+1,trial=record['trial'],search_started=started,
        near_stop_pct=record.get('fluency',{}).get('near_stop_pct'),
        gear_stop_s=record.get('attribution',{}).get('by_control_s',{}).get('gear_change'),
        stale_stop_s=record.get('attribution',{}).get('by_control_s',{}).get('stale_command'),
        complete_navigation_passed=record.get('result',{}).get('complete_navigation_passed'))),flush=True)
    if not started:break
print(json.dumps(dict(event='all_closed',output=str(output))),flush=True)

