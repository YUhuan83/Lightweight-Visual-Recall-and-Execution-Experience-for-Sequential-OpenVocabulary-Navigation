"""Read exact scored graph pairs before the forest-directed route."""
import json
import math
from pathlib import Path
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph

root=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s')
trial=root/'full/trial_20261030_095616';assert json.loads((trial/'result.json').read_text())['closed']
folder=next(trial.glob('sessions/*/graph_pairs'))
rows=[json.loads(line) for line in (folder/'snapshots.jsonl').read_text().splitlines()]
out=[]
for row in rows:
    plan=row.get('plan') or {}; goal=plan.get('exploration_goal')
    if row.get('kind')!='pair' or not row.get('raw') or not goal or not 695<goal[0]<720:continue
    graph=deserialize_message((folder/row['scored']['file']).read_bytes(),NavigationGraph)
    index=graph.trav_classes.index('default');front=[]
    for i,node in enumerate(graph.nodes):
        if index>=len(node.trav_properties) or not node.trav_properties[index].is_frontier:continue
        props={p.key:list(p.value) for p in node.properties}
        values=props.get('frontier_scores',[])
        front.append(dict(index=i,xy=[node.pose.position.x,node.pose.position.y],
                          has_scores=bool(values),maximum_score=max(values) if values else None,
                          headings=len(node.heading_candidates),
                          best_heading_utility=max((h.predicted_utility for h in node.heading_candidates if h.traversable),default=None),
                          physical=[dict(profile=a.robot_profile_id,feasible=a.feasible,confidence=a.confidence,
                              traversability=a.physical_traversability,risk=a.dynamic_risk,
                              observed_ns=a.observed_at.sec*10**9+a.observed_at.nanosec) for a in node.robot_traversability],
                          properties={k:v for k,v in props.items() if k in ('physical_risk','physical_violation_observed')}))
    selected=min(front,key=lambda n:math.dist(n['xy'],goal)) if front else None
    out.append(dict(pair=row['scored']['file'],source_key=row['key'],plan_stamp=plan.get('stamp_ns'),goal=goal,
                    selected=selected,scored_frontiers=sum(n['has_scores'] for n in front),frontiers=len(front),
                    top_scored=sorted([n for n in front if n['has_scores']],key=lambda n:n['maximum_score'],reverse=True)[:4]))
    if len(out)>=1:break
(trial/'frontier_score_coverage.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
