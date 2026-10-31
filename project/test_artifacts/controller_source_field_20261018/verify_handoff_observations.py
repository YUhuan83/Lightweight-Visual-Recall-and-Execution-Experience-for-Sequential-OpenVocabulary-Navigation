"""Disambiguate a later Following from an actual moving controller replacement."""
from pathlib import Path
import json,re
OUT=Path(__file__).parent;closed=OUT/'closed_evidence'
session=next((closed/'field_results').glob('session_*'))
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
report=json.loads((closed/'closed_handoff_review.json').read_text(encoding='utf8'))
stack=(session/'stack.log').read_text(encoding='utf8').splitlines()
cancels=[];preempt=[]
for line in stack:
    if 'Cancelling follow_path action' not in line and 'Passing new path to controller.' not in line:continue
    match=re.search(r'\[(\d+\.\d+)\]',line)
    if match:(cancels if 'Cancelling' in line else preempt).append(float(match[1]))
native=[];zeros=[]
for line in (session/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row=json.loads(line);t=row['received_ros_ns']/1e9
    if row['topic']=='/wildos/native_control_status':native.append((t,json.loads(row['message']['data'])['speed']))
    elif row['topic']=='/wildos/cmd_vel':
        m=row['message']
        if all(v==0 for part in ('linear','angular') for v in m[part].values()):zeros.append(t)
checks=[]
for lease in report['handoff_leases']:
    end=lease['outcome']
    if not lease['inferred_new_following']:continue
    a=lease['start']['stamp_ns']/1e9;b=end['stamp_ns']/1e9
    speeds=[v for t,v in native if a<=t<=b+.2]
    cancellation=[t for t in cancels if a<=t<=b+.2]
    controller_preempt=[t for t in preempt if b-.2<=t<=b+.4]
    terminal_zeros=[t for t in zeros if a<=t<=b]
    verified=bool(speeds and min(speeds)>.05 and controller_preempt and not cancellation and not terminal_zeros)
    checks.append(dict(start_elapsed=lease['start']['elapsed'],new_following_elapsed=end['elapsed'],
        delay_sec=b-a,minimum_native_speed=min(speeds,default=None),cancel_times=cancellation,
        controller_preempt_times=controller_preempt,terminal_zero_times=terminal_zeros,
        verified_moving_replacement=verified))
out=dict(candidate=report['candidate'],old_next_following_count=report['observed_new_following_after_retirement'],
    verified_moving_replacements=sum(x['verified_moving_replacement'] for x in checks),checks=checks,
    scope='Closed controller preemption log + source zero/cancel barriers + Native sampled motion. '
          'Later Following alone cannot prove continuity; no inter-sample force claim.')
(OUT/'verified_handoff_review.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print(json.dumps(out,indent=2))
