from pathlib import Path
import ast,difflib,hashlib,json,re
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
manifest=json.loads((OUT/'candidate6/candidate_delta.json').read_text(encoding='utf8'))
for name,want in manifest['members'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==want,name
rows=json.loads((OUT/'before_manifest.json').read_text(encoding='utf8'));diffs=[]
for row in rows:
 before=OUT/'before'/row['path'];after=ROOT/row['path']
 assert hashlib.sha256(before.read_bytes()).hexdigest()==row['sha256']
 target=OUT/'after'/row['path'];target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(after.read_bytes())
 diffs.extend(difflib.unified_diff(before.read_text(encoding='utf8').splitlines(True),after.read_text(encoding='utf8').splitlines(True),fromfile='before/'+row['path'],tofile='after/'+row['path']))
(OUT/'source.diff').write_text(''.join(diffs),encoding='utf8')
planner=(ROOT/'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp').read_text(encoding='utf8')
header=(ROOT/'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include/graphnav_planner/planner.hpp').read_text(encoding='utf8')
assert 'graaf::directed_graph<' in header and 'undirected_graph' not in planner+header
assert 'graph_.add_edge(virtual_goal' not in planner
assert planner.count('graph_.add_edge(id, virtual_goal, frontier_cost)')==2
nav=ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
tree=ast.parse(nav.read_text(encoding='utf8'));score_keys=[]
for node in ast.walk(tree):
 if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='score_keys' for t in node.targets):score_keys.extend(ast.literal_eval(node.value))
assert score_keys and 'physical_violation_observed' not in score_keys
review=dict(source_delta_verified=len(manifest['members']),before_after_snapshots=len(rows),
    directed_graph=True,frontier_attachments_correct_direction=True,visual_scoring_preserves_physical_tag=True,
    complete_vehicle_acceptance=False)
(OUT/'source_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf8');print(json.dumps(review))
