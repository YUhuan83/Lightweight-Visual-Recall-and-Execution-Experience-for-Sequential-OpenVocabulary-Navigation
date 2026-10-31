"""Deduplicate recorded wheel probes by acquisition wall time, not status count."""
import json,statistics
from pathlib import Path
root=Path(__file__).resolve().parent
stalls=json.loads((root/'stalls.json').read_text())
reports=[]
for i,case in enumerate(stalls):
    probes={}
    for row in case['scalar_rows']:
        snap=row.get('wheel_contact')
        if not snap or not -5<=row['relative_to_failure']<=0:continue
        age=row['wall']-snap['wall_time']
        if not 0<=age<=1.1:continue
        probes.setdefault(snap['wall_time'],dict(relative_to_failure=row['relative_to_failure']-age,**snap))
    wheels=[w for p in probes.values() for w in p['wheels']]
    def bounds(key,absolute=False):
        a=[abs(w[key]) if absolute else w[key] for w in wheels if isinstance(w.get(key),(int,float))]
        return dict(n=len(a),min=min(a),median=statistics.median(a),max=max(a)) if a else None
    r=dict(case=i,unique_acquisitions=len(probes),probes=list(probes.values()),
           normal_force=bounds('downForceRaw'),absolute_propulsion_torque=bounds('propulsionTorque',True),
           braking_torque=bounds('brakingTorque'),absolute_wheel_speed=bounds('wheelSpeed',True))
    assert r['unique_acquisitions']>0
    assert all(len(p['wheels'])==4 for p in probes.values())
    assert r['normal_force']['min']>5 and r['absolute_propulsion_torque']['min']>0
    assert r['braking_torque']['max']==0
    reports.append(r)
(root/'wheel_contact_review.json').write_text(json.dumps(reports,indent=2)+'\n')
print(json.dumps([{k:v for k,v in r.items() if k!='probes'} for r in reports],indent=2))
