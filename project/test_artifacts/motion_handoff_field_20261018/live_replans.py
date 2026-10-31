"""Small JSONL-only cross-module snapshot. Never open active SQLite."""
from pathlib import Path
import json, math, collections
root=Path('/home/ly/motion_handoff_field_20261018')
driver=sorted(root.glob('field_yaw_left90_search_*'))[-1]
pid=int((driver/'driver_child.pid').read_text())
session=next(p.parent for p in (root/'field_results').glob('*/console_identity.json')
             if json.loads(p.read_text())['pid']==pid)
g=session/'goal_001'
def rows(p):
    result=[]
    if p.exists():
        for line in p.read_text().splitlines():
            try:result.append(json.loads(line))
            except ValueError:pass
    return result
statuses=[r for r in rows(g/'events.jsonl') if r.get('kind')=='plan_status']
native=rows(g/'native_samples.jsonl')
first=next((r for r in statuses if r.get('exploration_goal')), None)
report=dict(session=str(session),driver_closed=(driver/'result.json').exists(),
    replan_events=[r for r in statuses if r.get('status') in (
        'replanning_after_maneuver','exploration_finishing_segment','exploration_handoff_stopped',
        'exploration_segment_finished','exploration_retired','follow_failed','no_vehicle_path')],
    movement_sections=[r.get('motion_sections_remaining') for r in statuses if r.get('status')=='planned'])
if first:
    points=[r['position'][:2] for r in native if r.get('wall_time',0)>=first['stamp_ns']/1e9
            and isinstance(r.get('position'),list)]
    goal=first['exploration_goal']
    report['first_goal_native_reference']=dict(goal=goal,
        scope='Native center, not odometry rear axle; no arrival conclusion',
        initial_distance=math.dist(points[0],goal) if points else None,
        minimum_distance=min((math.dist(p,goal) for p in points),default=None),
        latest_distance=math.dist(points[-1],goal) if points else None)
print(json.dumps(report,indent=2))
