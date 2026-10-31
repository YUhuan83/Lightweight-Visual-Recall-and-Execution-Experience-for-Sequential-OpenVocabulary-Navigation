"""Compact read-only progress from a running trial's saved logs."""
from pathlib import Path
import json
import sys

folder = Path('/home/ly/full_town_memory_20261002')/sys.argv[1]
events = []
for line in (folder/'events.jsonl').read_text().splitlines():
    try:
        item = json.loads(line)
    except json.JSONDecodeError:
        continue
    if item.get('kind') in ('plan_status', 'search_progress', 'return_progress',
            'house_search_finished', 'full_test_finished', 'test_error', 'object_standoff'):
        events.append(item)
for item in events[-8:]:
    print(json.dumps({key: value for key, value in item.items()
                     if key not in ('graph_route', 'candidate_goals', 'reasons')}, ensure_ascii=False))
if (folder/'full_test.json').exists():
    report = json.loads((folder/'full_test.json').read_text())
    print(json.dumps({k:v for k,v in report.items() if k not in ('events', 'calibration', 'station_views_before_departure')}, ensure_ascii=False))
