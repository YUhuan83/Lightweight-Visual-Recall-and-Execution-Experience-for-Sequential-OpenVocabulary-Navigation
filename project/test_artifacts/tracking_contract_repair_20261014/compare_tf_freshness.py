"""Measure the controller's actual transformed-plan stamp, not status repeats."""
import collections,json,math,sys
from pathlib import Path
root=Path(__file__).parent
reports=[]
for trial in map(int,sys.argv[1:]):
    path=root/f'physical_cusp_component{trial}_observer/messages.jsonl'
    ages=[];odom_ns=None;coincident=[];spells=[];first=None;last=None;stamp=None
    for line in path.read_text().splitlines():
        r=json.loads(line);m=r['message'];now=r['received_ros_ns']
        if r['topic']=='/spot1/odom':
            s=m['header']['stamp'];odom_ns=s['sec']*10**9+s['nanosec']
        if r['topic']!='/received_global_plan':continue
        s=m['header']['stamp'];source=s['sec']*10**9+s['nanosec'];age=(now-source)/1e9
        ages.append(age)
        if odom_ns and (now-odom_ns)/1e9<.3 and age>.5:
            coincident.append({'received_ros_ns':now,'pose_stamp_ns':source,'pose_age':age,
                'latest_observed_odom_age':(now-odom_ns)/1e9})
        if stamp!=source:
            if first is not None:spells.append({'stamp_ns':stamp,'repeated_ticks':last[0],
                'duration':(last[1]-first)/1e9,'max_age':last[2]})
            stamp=source;first=now;last=[1,now,age]
        else:last=[last[0]+1,now,max(last[2],age)]
    if first is not None:spells.append({'stamp_ns':stamp,'repeated_ticks':last[0],
        'duration':(last[1]-first)/1e9,'max_age':last[2]})
    result=json.loads((root/f'physical_cusp_component{trial}/result.json').read_text())
    row={'trial':trial,'passed':result['passed'],'errors':result['errors'],
        'controller_ticks':len(ages),'max_pose_age_sec':max(ages,default=None),
        'p95_pose_age_sec':sorted(ages)[int(.95*(len(ages)-1))] if ages else None,
        'stale_pose_with_fresh_external_odom_ticks':len(coincident),
        'longest_identical_stamp_spells':sorted(spells,key=lambda x:x['duration'],reverse=True)[:5],
        'stale_counterexamples':coincident[-10:]}
    reports.append(row)
(root/'tf_freshness_comparison.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(reports,indent=2))
