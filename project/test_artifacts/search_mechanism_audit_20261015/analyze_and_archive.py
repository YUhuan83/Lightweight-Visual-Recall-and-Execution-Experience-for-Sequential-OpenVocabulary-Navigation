import json,sqlite3,math,hashlib,zipfile
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
r=Path('/home/ly/wildos-console');s=r/'audit_results/session_20261015_155034';assert (s/'session_result.json').exists()
a=[json.loads(l) for l in (s/'vision_participation.jsonl').read_text().splitlines()];scores=[x for x in a if x['kind']=='scores']
final=[x for x in a if x['kind']=='final'][-1]
c=sqlite3.connect(next((s/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=c.execute("select id from topics where name='/spot1/graphnav_planner/route_status'").fetchone()[0]
matched=0;total=0;goals=set()
for ts,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
 d=json.loads(deserialize_message(b,String).data);g=d.get('goal');total+=1
 if not g:continue
 goals.add(tuple(g))
 recent=[x for x in scores if 0<=ts/1e9-x['wall']<3.]
 if any(math.dist(n['xy'],g[:2])<1e-5 for x in recent for n in x['candidates']):matched+=1
report=dict(final=final,score_observations=len(scores),query_observations={q:sum(x['query']==q for x in scores) for q in {x['query'] for x in scores}},
 min_frontier_distance_m=min(x['nearest_frontier_m'] for x in scores if x['nearest_frontier_m'] is not None),
 max_within_config_range=max(x['within_config_range'] for x in scores),max_visible_frontiers=max(x['visible_frontiers'] for x in scores),
 max_memory_utility_delta=max(x['max_memory_utility_delta'] for x in scores),
 route_status_messages=total,route_goals_matching_recent_scored_nodes=matched,unique_graph_goals=len(goals),
 limitation='Matching positions is dataflow evidence, not a controlled score ablation. Cached default scores are falsely labelled nondefault in production; real scorer call count is authoritative.')
(s/'audit_summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
files=set()
for p in (r/'audit_results').glob('session_*'):
 assert (p/'session_result.json').exists();files.update(x for x in p.rglob('*') if x.is_file())
for p in r.glob('field_audit_search_*'):
 assert (p/'result.json').exists();files.update(x for x in p.rglob('*') if x.is_file())
files.add(r/'source_identity.json');files.add(r/'audit_console.json')
out=r/'score_memory_audit_20261015.zip';manifest={}
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as z:
 for p in sorted(files):
  b=p.read_bytes();n=str(p.relative_to(r));manifest[n]=hashlib.sha256(b).hexdigest();z.writestr(n,b)
 z.writestr('MANIFEST.json',json.dumps(manifest,indent=2))
print(json.dumps(dict(archive=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),files=len(manifest))))
