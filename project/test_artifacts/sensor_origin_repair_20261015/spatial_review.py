from pathlib import Path
import json,math
s=Path('/home/ly/wildos-console/audit_results/session_20261015_203918');assert (s/'session_result.json').exists()
g=s/'goal_001';rows=[json.loads(l) for l in (g/'native_samples.jsonl').read_text().splitlines()];rows=[r for r in rows if isinstance(r.get('position'),list)];first=rows[0]['position'];events=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines() if l];statuses=[e for e in events if e.get('kind')=='plan_status' and e.get('stamp_ns')]
pauses=[];start=None;last=None
for r in rows:
 t=r['wall_time'];stopped=r.get('speed',1)<.05
 if stopped and start is None:start=t
 if not stopped and start is not None:
  if t-start>=5:
   previous=[e for e in statuses if e['stamp_ns']/1e9<=start]
   pauses.append({'begin_from_first_sample_sec':start-rows[0]['wall_time'],'duration_sec':t-start,'preceding_status':previous[-1]['status'] if previous else None})
  start=None
 last=t
if start is not None and last-start>=5:pauses.append({'begin_from_first_sample_sec':start-rows[0]['wall_time'],'duration_sec':last-start})
r={'max_distance_from_start_m':max(math.dist(p['position'][:2],first[:2]) for p in rows),'final_distance_from_start_m':math.dist(rows[-1]['position'][:2],first[:2]),'native_stops_below_0_05_for_at_least_5_sec':pauses,'scope':'Native speed and wall-time grouping; pauses are not automatically physical stuck or planner deadlock'}
(s/'spatial_review.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
