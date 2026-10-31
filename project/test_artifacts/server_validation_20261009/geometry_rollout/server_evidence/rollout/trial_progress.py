from pathlib import Path
import json,collections
p=Path('/home/ly/town_memory_validation_20261008/B_green_house_station_trial13_geometry_lifecycle_20261009');rows=[]
for line in (p/'events.jsonl').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
progress=next((x for x in reversed(rows) if x['kind']=='search_progress'),{})
status=next((x for x in reversed(rows) if x['kind']=='plan_status'),{})
report={'latest_elapsed':rows[-1]['elapsed'],'phase':rows[-1].get('phase'),'position':progress.get('position'),'speed':progress.get('speed'),'masks':progress.get('mask_count'),'latest_status':{k:status.get(k) for k in ['elapsed','status','source','query','reason','goal']},'kind_counts':dict(collections.Counter(x['kind'] for x in rows))}
if (p/'full_test.json').exists():
 result=json.loads((p/'full_test.json').read_text());report['final']={k:v for k,v in result.items() if k not in ['events','topics_recorded','station_candidates_at_switch']}
print(json.dumps(report))
