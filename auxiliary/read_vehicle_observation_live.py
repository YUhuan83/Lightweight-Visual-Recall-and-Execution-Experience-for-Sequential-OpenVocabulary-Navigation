"""Text-log progress only; never opens an active SQLite bag."""
import json,math,sys
from pathlib import Path
version=sys.argv[1] if len(sys.argv)>1 else 'v34_vehicle_observation'
assert version in ('v34_vehicle_observation','v35_confirmed_vehicle','v36_weak_observation','v37_full_path','v38_rolling_observation','v39_observation_session','v40_observation_heading','v41_general_observation','v42_retirement_boundary','v43_abort_recovery','v44_instance_handoff','v45_approach_contract','v46_approach_feedback','v47_depth_geometry','v48_range_candidate','v49_localization_range','v50_confirmed_continuation','v51_confirmed_action_handoff','v52_approach_local_map','v53_confirmed_graph_destination','v54_confirmed_graph_handoff','v55_frontier_endpoint','v56_confirmed_local_continuity','v57_upstream_target_approach')
root=Path('/home/ly/execution_continuity_review_20261022')/version
goals=sorted((root/'full').glob('trial_*/sessions/*/goal_001'))
if not goals:print('No goal log yet');raise SystemExit
g=goals[-1]
def records(path):
    out=[]
    if not path.exists():return out
    for line in path.read_text().splitlines():
        try:out.append(json.loads(line))
        except json.JSONDecodeError:continue
    return out
events=records(g/'events.jsonl');native=records(g/'native_samples.jsonl')
plans=[e for e in events if e.get('kind')=='plan_status']
print(json.dumps(dict(goal=str(g),last_statuses=[{k:e[k] for k in ('elapsed','status','source','goal','reason','geometry_refreshed') if k in e} for e in plans[-4:]],native_samples=len(native),current_speed=native[-1]['speed'] if native else None,provisional_net_m=math.dist(native[0]['position'][:2],native[-1]['position'][:2]) if native else None),indent=2))
