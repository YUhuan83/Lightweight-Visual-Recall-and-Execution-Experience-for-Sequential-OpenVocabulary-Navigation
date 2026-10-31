import bisect,collections,json,math
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/confirmed_progress_repair_20261025'
goal=next((root/'field').glob('sessions/*/goal_001'))
native=[json.loads(l) for l in (goal/'native_samples.jsonl').read_text(encoding='utf8').splitlines()]
events=[json.loads(l) for l in (goal/'events.jsonl').read_text(encoding='utf8').splitlines()]
plans=[e for e in events if e['kind']=='plan_status']
times=[e['stamp_ns']*1e-9 for e in plans]
t0=times[0]-plans[0]['elapsed']
origin=native[0]['position'][:2]
furthest=max(native,key=lambda n:math.dist(origin,n['position'][:2]))
windows=[]
for lo,hi in ((0,60),(60,120),(120,184)):
    samples=[n for n in native if lo<=n['wall_time']-t0<hi]
    path=sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(samples,samples[1:]))
    windows.append(dict(from_s=lo,to_s=hi,path_m=path,net_m=math.dist(samples[0]['position'][:2],samples[-1]['position'][:2]),
        start=samples[0]['position'][:2],end=samples[-1]['position'][:2]))
owners={};direction=collections.Counter()
for a,b in zip(native,native[1:]):
    dt=b['wall_time']-a['wall_time']
    if not 0<dt<=.5:continue
    step=math.dist(a['position'][:2],b['position'][:2])
    signed=sum(x*y for x,y in zip(a['velocity'],a['direction']))
    direction['reverse_m' if signed<-.05 else 'forward_m' if signed>.05 else 'near_stationary_m']+=step
    i=bisect.bisect_right(times,a['wall_time'])-1
    if i<0:continue
    identity=plans[i].get('exploration_id')
    if identity is None:continue
    d=owners.setdefault(identity,dict(time_s=0.,path_m=0.,start_s=a['wall_time']-t0,end_s=0.,goal=plans[i].get('exploration_goal')))
    d['time_s']+=dt;d['path_m']+=step;d['end_s']=b['wall_time']-t0
out=dict(furthest_from_start_m=math.dist(origin,furthest['position'][:2]),furthest_at_s=furthest['wall_time']-t0,
    furthest_position=furthest['position'][:2],windows=windows,direction_distance=dict(direction),
    longest_owners=sorted([dict(identity=k,**v) for k,v in owners.items()],key=lambda x:x['time_s'],reverse=True)[:5])
(root/'displacement_audit.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(out,indent=2))
