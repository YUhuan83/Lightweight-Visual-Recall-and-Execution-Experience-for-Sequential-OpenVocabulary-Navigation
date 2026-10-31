"""Read-only source comparison plus small replicas of the upstream lookahead rule.

No navigation, deployment, ROS or vehicle state is changed. Reports are local.
"""
import ast,difflib,hashlib,json,math
from pathlib import Path
original=Path(r'C:\Users\Yu\nebula2-wildos')
repo=Path(r'C:\Users\Yu\Desktop\拟真')
vendor=repo/'wildos_port/repro_check/src/nebula2-wildos'
out=repo/'test_artifacts/upstream_execution_comparison_20261024';out.mkdir(exist_ok=True)
provenance=json.loads((repo/'wildos_port/repro_check/src/nebula2-wildos.provenance.json').read_text(encoding='utf8'))
head=provenance['source_head']
files=['graphnav_planner/src/planner.cpp','graphnav_planner/src/planner_node.cpp',
       'graphnav_planner/src/path_follower_node.cpp','graphnav_mapper/src/mapper.cpp',
       'visual_navigation/visual_navigation/wildos/nav.py',
       'visual_navigation/visual_navigation/wildos/goalagnostic_scoring.py',
       'visual_navigation/visual_navigation/utils/goal_navigator.py',
       'graphnav_planner/launch/graphnav_planner.launch.yml']
comparisons=[]
def sha(text):return hashlib.sha256(text.encode()).hexdigest()
for rel in files:
    raw=(original/rel).read_bytes()
    source_sha=provenance['source_files'][rel]['sha256']
    assert hashlib.sha256(raw).hexdigest()==source_sha,rel
    old=raw.decode().replace('\r\n','\n')
    new=(vendor/rel).read_text(encoding='utf8').replace('\r\n','\n')
    delta=list(difflib.unified_diff(old.splitlines(keepends=True),new.splitlines(keepends=True),fromfile='upstream/'+rel,tofile='current/'+rel))
    (out/(rel.replace('/','__')+'.diff')).write_text(''.join(delta),encoding='utf8')
    comparisons.append(dict(path=rel,upstream_commit=head,upstream_sha256=sha(old),current_sha256=sha(new),
        upstream_checkout_matches_provenance=True,changed=(old!=new),
        upstream_lines=len(old.splitlines()),current_lines=len(new.splitlines()),
        added_lines=sum(s.startswith('+') and not s.startswith('+++') for s in delta),
        removed_lines=sum(s.startswith('-') and not s.startswith('---') for s in delta)))

def references(path,needle):
    return [i for i,s in enumerate(path.read_text(encoding='utf8').splitlines(),1) if needle in s]
bridge=repo/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
tree=ast.parse(bridge.read_text(encoding='utf8'));topics=[]
for n in ast.walk(tree):
    if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_subscription' and len(n.args)>1 and isinstance(n.args[1],ast.Constant):topics.append(n.args[1].value)
assert '/spot1/graphnav_planner/path' in topics and '/spot1/goal_pose' not in topics

def upstream_lookahead(points,robot,lookahead=5.):
    # Exact non-degenerate interpolation/closest-index rule from upstream C++.
    distances=[math.dist(p,robot) for p in points];closest=min(range(len(points)),key=lambda i:distances[i])
    if closest==len(points)-1:return points[-1]
    index=closest+1
    while index+1<len(points) and distances[index]<lookahead:
        a,b,c=points[index-1],points[index],points[index+1]
        v=(b[0]-a[0],b[1]-a[1]);w=(c[0]-b[0],c[1]-b[1])
        if sum(x*y for x,y in zip(v,w))/math.hypot(*v)/math.hypot(*w)<.5:break
        index+=1
    a=distances[index-1]-lookahead;b=distances[index]-lookahead
    fraction=min(1.,max(0.,-a/(b-a)))
    return tuple(points[index][j]*fraction+points[index-1][j]*(1-fraction) for j in range(2))
replays=[]
for repeat in range(2):
    targets=[upstream_lookahead([(i*.5,0.) for i in range(61)],(x,0.)) for x in (0.,1.,2.,3.)]
    assert all(math.dist(a,b)<1e-9 for a,b in zip(targets,[(5.,0.),(6.,0.),(7.,0.),(8.,0.)]))
    coarse=[upstream_lookahead([(0.,0.),(10.,0.),(20.,0.),(30.,0.)],(x,0.)) for x in (0.,1.,2.,3.)]
    replays.append(dict(repeat=repeat,robot_x=[0.,1.,2.,3.],rolling_goals=targets,
                        coarse_path_goals=coarse,
                        scope='Mathematical replica of upstream interpolation, not ROS/vehicle execution; coarse node spacing need not produce exact 5m lookahead'))

art=repo/'test_artifacts/fresh_command_retry_20261023'
field=[]
for filename in ('stop_plan_ab_results.json','v22_closed_results.json','v23_closed_results.json'):
    path=art/filename
    if not path.exists():continue
    for row in json.loads(path.read_text(encoding='utf-8-sig')):
        field.append(dict(version=row['version'],trial=row['trial'],closed=row['result']['closed'],
           near_stop_pct=row.get('fluency',{}).get('near_stop_pct'),
           net_displacement_m=row.get('fluency',{}).get('net_displacement_m'),
           stopped_by_control=row.get('attribution',{}).get('by_control_s'),
           complete_navigation_passed=row['result'].get('complete_navigation_passed')))
evidence=dict(upstream_commit=head,source_comparisons=comparisons,current_bridge_subscriptions=sorted(topics),
    upstream_follower_replica=replays,closed_field_trials=field,
    caveats=['Reference files match the existing provenance manifest; Git object access is unavailable, so this is not a new independent Git-blob verification.',
             'Upstream repo requires external Nav2; its robot-specific Nav2/BeamNG execution configuration is not included.',
             'Previously fluent BeamNG run has no verified exact code/config identity in this audit.',
             'Current worktree differs from frozen server candidates; do not equate their parameter files.',
             'No new field test, code repair or deployment is performed by this audit.'])
encoded=json.dumps(evidence,ensure_ascii=False,sort_keys=True,indent=2).encode();(out/'source_and_evidence.json').write_bytes(encoded)
print(json.dumps(dict(report=str(out/'source_and_evidence.json'),sha256=hashlib.sha256(encoded).hexdigest(),
    comparisons=comparisons,bridge_uses_original_rolling_goal=False),ensure_ascii=False,indent=2))
