"""Audit only a terminal trial; collect logs without videos or full bag copy."""
from pathlib import Path
import bisect,collections,json,math,sqlite3,subprocess,tarfile,sys
trial=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/ly/execution_continuity_review_20261022/v5/full/trial_20261022_212511')
root=trial.parents[1]
result=json.loads((trial/'result.json').read_text());assert result['closed']
goal=next(trial.glob('sessions/*/goal_001'));session=goal.parent
pre=subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True)
(trial/'postcheck.json').write_text(pre.stdout)
assert not json.loads(pre.stdout)['processes']
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
native=[json.loads(x) for x in (goal/'native_samples.jsonl').read_text().splitlines()]
plans=[e for e in events if e['kind']=='plan_status']
stamped=[(e['stamp_ns']/1e9,e) for e in plans]
stamped.sort(key=lambda x:x[0]);times=[t for t,e in stamped]
covered=stopped=0.;intervals=[];low_start=None;by_state=collections.Counter();by_actuation=collections.Counter()
for a,b in zip(native,native[1:]):
    dt=b['wall_time']-a['wall_time']
    if not 0<dt<=.5:
        if low_start is not None:intervals.append([low_start,a['wall_time']]);low_start=None
        continue
    covered+=dt
    if abs(a['speed'])<.1:
        stopped+=dt
        if low_start is None:low_start=a['wall_time']
        j=bisect.bisect_right(times,a['wall_time'])-1
        by_state[stamped[j][1]['status'] if j>=0 else 'before_state']+=dt
        command=a.get('sent') or {}
        reason=('source_rejected' if a.get('source_rejection') else
                'command_old' if (a.get('command_age') or 0)>.5 else
                'braking' if command.get('brake',0)>.1 else
                'throttling' if command.get('throttle',0)>.05 else 'coasting')
        by_actuation[reason]+=dt
    elif low_start is not None:
        intervals.append([low_start,a['wall_time']]);low_start=None
if low_start is not None:intervals.append([low_start,native[-1]['wall_time']])
t0=plans[0]['stamp_ns']/1e9-plans[0]['elapsed']
summary=dict(trial=str(trial),closed=True,field_acceptance=result.get('session_result',{}).get('field_acceptance',False),
    native_coverage_s=covered,near_stop_s=stopped,near_stop_pct=100*stopped/covered,
    longest_near_stop_s=max((b-a for a,b in intervals),default=0),
    net_displacement_m=math.dist(native[0]['position'][:2],native[-1]['position'][:2]),
    sampled_distance_m=sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(native,native[1:])),
    stopped_by_state=dict(by_state),stopped_by_actuation=dict(by_actuation),
    states=dict(collections.Counter(e['status'] for e in plans)),
    problems=[e for e in plans if any(word in e['status'] for word in ('failed','timeout','unavailable','no_vehicle','replanning','invalid'))],
    stop_intervals=[dict(elapsed=a-t0,duration=b-a) for a,b in intervals if b-a>=1.],
    query_event=next(e for e in events if e['kind']=='query_sent'))
(trial/'fluency_summary.json').write_text(json.dumps(summary,indent=2))
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics=list(db.execute('select id,name,type from topics'))
(trial/'bag_topics.json').write_text(json.dumps(topics,indent=2))
archive=root/(trial.name+'_core.tar.gz')
with tarfile.open(archive,'w:gz') as output:
    for path in trial.rglob('*'):
        if path.is_file() and path.suffix in ('.log','.json','.jsonl','.png'):
            output.add(path,arcname=str(path.relative_to(trial)))
print(json.dumps(summary,indent=2))
print('archive',archive)
