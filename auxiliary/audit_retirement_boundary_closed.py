"""Actual CLOSED bag identity/endpoint audit; no inferred queue ownership."""
import collections,json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.input_quality import stamp_ns
root=Path('/home/ly/execution_continuity_review_20261022')/sys.argv[1]
receipt=json.loads((root/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes'] and not receipt['postcheck']['display148_lock']
trial=Path(receipt['trial']);g=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(b,cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
fences=rows('/wildos/controller_path_retirement');progress=rows('/wildos/controller_path_progress')
queries=rows('/spot1/object_search/query');start=next(t for t,m in queries if m.data=='green residential house')
plans={stamp_ns(m):m for _,m in rows('/wildos/planned_path')}
def heading(p):
    q=p.orientation;return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def same(a,b):
    return math.hypot(a.position.x-b.position.x,a.position.y-b.position.y)<1e-6 and abs(math.remainder(heading(a)-heading(b),2*math.pi))<1e-6
result=[]
for t,p in fences:
    key=stamp_ns(p);endpoint=p.poses[-1].pose
    samples=[(at,m) for at,m in progress if stamp_ns(m)==key and at>t+300_000_000 and m.poses]
    violations=[dict(elapsed=(at-start)*1e-9,endpoint=[m.poses[-1].pose.position.x,m.poses[-1].pose.position.y])
                for at,m in samples if not same(m.poses[-1].pose,endpoint)]
    result.append(dict(elapsed=(t-start)*1e-9,path_stamp_ns=key,matched_dispatched_path=key in plans,
        boundary=[endpoint.position.x,endpoint.position.y,heading(endpoint)],
        progress_samples_after_300ms=len(samples),future_branch_violations=violations))
db.close()
events=[json.loads(x) for x in (g/'events.jsonl').read_text().splitlines()]
stack=(g.parent/'stack.log').read_text()
out=dict(trial=str(trial),fences=result,retirement_messages=len(fences),
    retained_old_future_branch_violations=sum(len(r['future_branch_violations']) for r in result),
    statuses=dict(collections.Counter(e.get('status') for e in events if e.get('kind')=='plan_status')),
    controller_boundary_terminal_lines=[l for l in stack.splitlines() if 'Retired continuous direction boundary reached' in l],
    acceptance=False,scope='Exact accepted path identity and remaining-direction endpoint. Receipt skew <=300ms excluded; full arrival independently assessed.')
(root/'retirement_boundary_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k not in ('statuses','controller_boundary_terminal_lines')},indent=2))
