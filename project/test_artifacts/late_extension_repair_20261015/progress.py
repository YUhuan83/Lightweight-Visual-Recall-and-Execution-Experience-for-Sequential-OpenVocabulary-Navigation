import json,collections,math
from pathlib import Path
s=Path('/home/ly/wildos-console/audit_results/session_20261015_213429');g=s/'goal_001'
rows=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines()]
p=[json.loads(l) for l in (g/'native_samples.jsonl').read_text().splitlines()];pos=[z['position'] for z in p if isinstance(z.get('position'),list)]
a=[json.loads(l) for l in (s/'vision_participation.jsonl').read_text().splitlines() if l];last=next(z for z in reversed(a) if z['kind']=='scores')
print(json.dumps({'last_event':rows[-1],'status_counts':dict(collections.Counter(z.get('status') for z in rows if z['kind']=='plan_status')),'native_distance_m':sum(math.dist(x[:2],y[:2]) for x,y in zip(pos,pos[1:])),'latest_native':p[-1],'vision':{k:v for k,v in last.items() if k!='candidates'}},indent=2))
