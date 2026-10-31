import json
from pathlib import Path
r=Path('/home/ly/wildos-console/audit_results');s=sorted(r.glob('session_*'))[-1]
a=[json.loads(l) for l in (s/'vision_participation.jsonl').read_text().splitlines()]
x=[x for x in a if x['kind']=='scores'];f=[x for x in a if x['kind']=='feedback']
last=x[-1];print(json.dumps(dict(session=str(s),observations=len(x),last={k:v for k,v in last.items() if k!='candidates'},feedback=f[-4:]),indent=2))
