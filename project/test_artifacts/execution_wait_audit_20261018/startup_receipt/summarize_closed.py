"""Compact closed search facts and exact underlying planner error text."""
import collections,json,re
from pathlib import Path
r=Path('/home/ly/planning_phase_physical_20261018')
a=json.loads((r/'field/baseline_summary.json').read_text())
s=Path(a['session']);assert (s/'session_result.json').is_file()
lines=(s/'stack.log').read_text().splitlines()
planner=[]
for line in lines:
    if '[planner_server-' not in line or not any(t in line for t in ('[WARN]','[ERROR]','[FATAL]')):continue
    planner.append(re.sub(r'^.*?\[planner_server\]:\s*','',line))
v={k:a.get(k) for k in ('goal','native_distance_m','max_departure_m','final_departure_m','status_counts','route_decisions','complete_route_decisions','route_gaps_ge5s','retirement_reasons','observed_route_ack_messages','masks','localizations','confirmations','failure_counts','session_errors')}
v['planner_warning_counts']=dict(collections.Counter(planner))
v['runtime_source_lifetime_errors']=sum('source_lifetime' in l for l in lines)
v['stop']=json.loads((s/'goal_001/stop.json').read_text())
v['video']=json.loads((r/'video_verification.json').read_text())
(r/'compact_search_review.json').write_text(json.dumps(v,indent=2)+'\n')
print(json.dumps(v,indent=2))
