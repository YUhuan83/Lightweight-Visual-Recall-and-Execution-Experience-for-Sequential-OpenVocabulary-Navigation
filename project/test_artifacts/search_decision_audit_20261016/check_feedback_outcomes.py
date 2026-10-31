from pathlib import Path
import sqlite3,json,collections
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
reports=[]
for name in ('20261015_213429','20261016_085025'):
 s=Path('/home/ly/wildos-console/audit_results/session_'+name);assert (s/'session_result.json').exists()
 c=sqlite3.connect(next((s/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True);tp={n:(i,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
 def msgs(name):
  i,typ=tp[name];return [deserialize_message(b,typ) for b, in c.execute('select data from messages where topic_id=?',(i,))]
 feedback=msgs('/spot1/traversal_feedback');events=[json.loads(x.data) for x in msgs('/wildos/execution_feedback')]
 reports.append(dict(session=name,outcome_values=dict(collections.Counter(x.outcome for x in feedback)),all_published_feedback_success=all(x.outcome==x.OUTCOME_SUCCEEDED for x in feedback),failed_diagnostics=[v for v in events if not v.get('succeeded')]))
 c.close()
Path('/home/ly/search_decision_audit_20261016/feedback_outcome_review.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
