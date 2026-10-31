"""Print compact trial progress and actual runtime failures."""
from pathlib import Path
import json
import sys

ROOT = Path('/home/ly/town_memory_validation_20261003')
out = ROOT/(sys.argv[1] if len(sys.argv)>1 else 'attempt2')
events = []
if (out/'events.jsonl').exists():
    for line in (out/'events.jsonl').read_text().splitlines():
        try:
            event = json.loads(line)
            if event['kind'] not in ('planned_path','object_standoff','memory_candidates'):
                events.append(event)
        except ValueError:
            pass
print(json.dumps({'events':events[-8:]},indent=2))
if (out/'full_test.json').exists():
    result=json.loads((out/'full_test.json').read_text())
    print(json.dumps({'final':{k:v for k,v in result.items() if k not in
        ('events','station_views_before_departure','collision_profile_before_sequence')}},indent=2))
for name in ('triangulation','graph_bridge','controller','vision'):
    p=ROOT/(name+'.log')
    if p.exists():
        lines=p.read_text(errors='replace').splitlines()
        selected=[l for l in lines[-100:] if 'ERROR' in l or 'Rejected' in l or 'LOCK' in l or
                  'Confirmed' in l or 'Traceback' in l or 'Invalid' in l or 'remembered' in l]
        print(name, '\n'.join(selected[-4:]) or '\n'.join(lines[-2:]))
