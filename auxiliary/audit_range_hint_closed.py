"""Closed-field current range to observation/acceptance provenance audit."""
import collections,json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
base=Path('/home/ly/execution_continuity_review_20261022');root=base/(sys.argv[1] if len(sys.argv)>1 else 'v48_range_candidate')
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
g=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(n):
 i,c=topics[n]
 return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
loc={d['observation_stamp_ns']:d for _,m in rows('/spot1/object_localization') for d in [json.loads(m.data)] if d.get('localization_valid')}
hints=[dict(received_ns=t,**json.loads(m.data)) for t,m in rows('/spot1/visible_target_approach')]
plans=[json.loads(m.data) for _,m in rows('/wildos/plan_status')]
masks={m.header.stamp.sec*10**9+m.header.stamp.nanosec for _,m in rows('/spot1/object_mask')}
records=[]
for h in hints:
 if h.get('evidence')!='range_candidate':continue
 s=h['observation_stamp_ns'];d=loc.get(s);point=h['point']
 accepted=[p for p in plans if p.get('status')=='visible_target_approach' and p.get('target_range_known') is True and p.get('query')==h['query'] and 0<=p['stamp_ns']-s<=2_000_000_000 and p.get('goal') and math.dist(p['goal'],point)<.05]
 valid=(d is not None and s in masks and d['query']==h['query'] and d.get('quality',{}).get('association_validated') is True and h['object_position']==d['position'] and h['position_confirmed'] is False and h['search_confirmed'] is False)
 records.append(dict(stamp_ns=s,point=point,target=h['object_position'],exact_current_provenance=valid,accepted_statuses=len(accepted),recorded_delivery_age_sec=(h['received_ns']-s)*1e-9))
db.close();out=dict(trial=r['trial'],evidence_counts=dict(collections.Counter(h.get('evidence') for h in hints)),range_candidates=records,accepted_range_statuses=sum(p.get('status')=='visible_target_approach' and p.get('target_range_known') is True for p in plans),provenance_violations=sum(not p['exact_current_provenance'] for p in records),field_acceptance=r['result'].get('complete_navigation_passed',False),scope='Geometric source identity and accepted weak-observation status; no assertion of semantic truth or final arrival.')
(root/'range_hint_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
