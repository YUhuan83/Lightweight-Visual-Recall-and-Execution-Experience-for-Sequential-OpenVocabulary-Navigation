from pathlib import Path
import json,collections
p=Path('/home/ly/wildos-console/audit_results/session_20261016_085025/execution_observer/messages.jsonl')
counts=collections.Counter();last=None
with p.open('rb') as f:
 start=max(0,p.stat().st_size-2*1024*1024);f.seek(start)
 if start:f.readline()
 for l in f:
  try:z=json.loads(l)
  except ValueError:continue
  if z['topic']=='/wildos/control_status':
   d=json.loads(z['message']['data']);counts[d['reason']]+=1;last=d
print(json.dumps(dict(scope='Last two MB of active JSON observer, not SQLite or whole-session statistics',reasons=dict(counts),last=last),indent=2))
