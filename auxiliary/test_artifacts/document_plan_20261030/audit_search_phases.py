"""Split identical odometry near-stop intervals at the first actual 3D target."""
import json,math,sys
from pathlib import Path
parent=Path('/home/ly/execution_continuity_review_20261022')
reports=[]
for version,stamp in (('v73_document_plan','20261030_184621'),('v74_document_handoff','20261030_190118')):
    trial=parent/version/'full'/f'trial_{stamp}'
    source=(parent/version/'audit_fluency_120s.py').read_text().split('stop=run=longest=total=')[0]
    env={};sys.argv=['phase_audit',str(trial)];exec(compile(source,'same_odom_reader','exec'),env)
    rows,origin,ev,q=(env[k] for k in ('rows','origin','ev','q'))
    target=json.loads((trial/'target_path_interface_audit.json').read_text())
    split=target['first_raw_q'];boundary=origin+round(split*1e9)
    durations=[0.,0.];stops=[0.,0.]
    for(t,a),(u,b) in zip(rows,rows[1:]):
        intervals=((t,min(u,boundary)),(max(t,boundary),u))
        for i,(start,end) in enumerate(intervals):
            dt=max(0.,(end-start)*1e-9);durations[i]+=dt
            if math.hypot(a.twist.twist.linear.x,a.twist.twist.linear.y)<.1:stops[i]+=dt
    failures=[dict(q=e['elapsed']-q,reason=e.get('reason')) for e in ev if e.get('status')=='continuous_motion_failed' and 0<=e['elapsed']-q<120]
    report=dict(trial=str(trial),first_raw_q=split,
        phases=[dict(phase=name,covered_s=durations[i],near_stop_s=stops[i],near_stop_percent=100*stops[i]/durations[i]) for i,name in enumerate(('before_target','after_target'))],
        fraction_of_stops_before_target=stops[0]/sum(stops),failures=failures,
        final_xy_to_last_raw_m=math.dist(target['final_raw_target'][:2],target['final_odom'][:2]))
    reports.append(report)
out=parent/'v74_document_handoff/search_phase_audit.json';out.write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
