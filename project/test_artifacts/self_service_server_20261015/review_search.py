"""Read-only bag counts; require closed goal for persisted final review."""
import json,sqlite3,math,collections
from pathlib import Path
r=Path('/home/ly/wildos-console');s=sorted((r/'results').glob('session_*'))[-1];g=s/'goal_001'
assert (g/'result.json').exists() and (s/'session_result.json').exists(), 'Refuse reading an active recorder database'
counts={}
for p in (g/'interfaces').glob('*.db3'):
 with sqlite3.connect(p.as_uri()+'?mode=ro',uri=True) as c:
  for name,n in c.execute('SELECT topics.name,count(messages.id) FROM topics LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'):
   counts[name]=counts.get(name,0)+n
rows=[json.loads(x) for x in (g/'events.jsonl').read_text().splitlines()]
fail=[x for x in rows if x.get('kind')=='plan_status' and x.get('status')=='follow_failed']
report=dict(session=str(s),closed=(g/'result.json').exists(),topic_counts=counts,failures=fail,
 frontier_rejection_log_lines=[x for x in (s/'stack.log').read_text().splitlines() if 'selecting another frontier' in x])
if report['closed']:
 report['result']=json.loads((g/'result.json').read_text());(s/'closed_review.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
