from pathlib import Path
import collections
import json
import sys

ROOT = Path('/home/ly/town_memory_validation_20261003')
name = sys.argv[1]
OUT = ROOT/name
events = []
for line in (OUT/'events.jsonl').read_text().splitlines():
    try:
        events.append(json.loads(line))
    except json.JSONDecodeError:
        continue
pid = (OUT/'trial.pid').read_text().strip()
statuses = [e for e in events if e['kind']=='plan_status']
review_image = OUT/'first_target_candidate_latest.png'
review_json = OUT/'first_target_visual_review.json'
report = json.loads((OUT/'full_test.json').read_text()) if (OUT/'full_test.json').exists() else {}
summary = {'running': (Path('/proc')/pid).exists(),
           'status_counts': dict(collections.Counter(e['status'] for e in statuses)),
           'latest_progress': next((e for e in reversed(events) if e['kind']=='search_progress'), None),
           'recovery': [e for e in statuses if e['status'].startswith('recovery_')][-4:],
           'review_pending': review_image.exists() and not review_json.exists(),
           'report': {k: report[k] for k in ('passed','failure_reason','first_target_found',
                    'search_elapsed','maximum_departure','first_target_end','simulation_paused') if k in report}}
print(json.dumps(summary), flush=True)
