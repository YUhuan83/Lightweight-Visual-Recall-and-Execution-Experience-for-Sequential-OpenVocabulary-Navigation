"""Preserve closed evidence and verify moving background planning in text logs."""
import bisect,collections,json,subprocess,tarfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
checks=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not checks['processes'] and not checks['display148_lock']
version=root/'v25_fast_shift'
records=json.loads((root/'fast_shift_trials_20261024/results.json').read_text())
reports=[]
for record in records:
    trial=Path(record['trial']);assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
    native=sorted([json.loads(line) for line in (goal/'native_samples.jsonl').read_text().splitlines()],key=lambda n:n['wall_time'])
    keys=[n['wall_time'] for n in native]
    plans={e['stamp_ns']:e for e in events if e.get('kind')=='plan_status'}
    background=[]
    for stamp,e in sorted(plans.items()):
        if e.get('status')!='planning' or not e.get('background'):continue
        i=bisect.bisect_right(keys,stamp*1e-9)-1
        if i<0 or stamp*1e-9-keys[i]>.5:continue
        background.append(dict(stamp_ns=stamp,speed=native[i]['speed'],status_source=e.get('source'),
            request_serial=e.get('request_serial'),navigation_epoch=e.get('replay_inputs',{}).get('navigation_epoch')))
    summary=dict(trial=str(trial),states=dict(collections.Counter(e['status'] for e in plans.values())),
        background_requests_with_native=background,
        background_requests_while_moving=sum(abs(e['speed'])>=.1 for e in background),
        command_gaps_over_2s=json.loads((trial/'command_gaps.json').read_text()),
        scope='Matched prior Native sample <=0.5s old. Text-only evidence; request event is not proof that replacement was accepted.')
    (trial/'continuous_execution_evidence.json').write_text(json.dumps(summary,indent=2));reports.append(summary)
(version/'closed_runtime_evidence.json').write_text(json.dumps(dict(postcheck=checks,rounds=reports),indent=2))
target=Path('/tmp/fast_shift_compact.tar.gz')
with tarfile.open(target,'w:gz') as archive:
    for name in ('fast_shift_delta_receipt.json','loaded_shift_profile.json','shift_wait_audit.json','real_ros_report.json','real_ros_review_before_parameter_init.log','real_ros_review_rejected_local_base.log','closed_runtime_evidence.json','full/frozen_source_manifest.json'):
        p=version/name
        if p.is_file():archive.add(p,arcname=p.relative_to(root))
    for record in records:
        trial=Path(record['trial']);goal=next(trial.glob('sessions/*/goal_001'))
        for name in ('result.json','fluency_summary.json','control_stop_attribution.json','stale_command.json','command_gaps.json','continuous_execution_evidence.json'):
            p=trial/name
            if p.is_file():archive.add(p,arcname=p.relative_to(root))
        for p in (goal/'events.jsonl',goal/'native_samples.jsonl',goal/'stop.json',goal.parent/'server_preflight.json'):
            if p.is_file():archive.add(p,arcname=p.relative_to(root))
print(json.dumps(dict(archive=str(target),size=target.stat().st_size,
    rounds=[dict(trial=r['trial'],moving_background_requests=r['background_requests_while_moving'],command_gaps_over_2s=len(r['command_gaps_over_2s'])) for r in reports])))
