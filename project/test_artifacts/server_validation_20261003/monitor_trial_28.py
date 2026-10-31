from pathlib import Path
import collections
import json
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
OUT = ROOT/'traffic_light_station_28_terrain_recovery_20261004'
events = []
for line in (OUT/'events.jsonl').read_text().splitlines():
    try:
        events.append(json.loads(line))
    except json.JSONDecodeError:
        pass
pid = (OUT/'trial.pid').read_text().strip()
progress = next((e for e in reversed(events) if e['kind']=='search_progress'), None)
statuses = [e for e in events if e['kind']=='plan_status']
failures = [e for e in statuses if e.get('status')=='follow_failed']
summary = {'running': (Path('/proc')/pid).exists(),
           'status_counts': dict(collections.Counter(e.get('status') for e in statuses)),
           'latest_progress': progress,
           'recent_failure': failures[-1] if failures else None,
           'latest_status': statuses[-1] if statuses else None,
           'review_pending': (OUT/'first_target_candidate_latest.png').exists()
                             and not (OUT/'first_target_visual_review.json').exists()}
if (OUT/'full_test.json').exists():
    report = json.loads((OUT/'full_test.json').read_text())
    summary['report'] = {k: v for k, v in report.items()
                         if k in ('passed','failure_reason','first_target_found','search_elapsed',
                                  'maximum_departure','queries_sent','first_target_end',
                                  'vision_pid_retained','simulation_paused','error')}
print(json.dumps(summary), flush=True)
