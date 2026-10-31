import collections,json,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022')/(sys.argv[1] if len(sys.argv)>1 else 'v31_confirmed_progress')
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed']
assert not receipt['postcheck']['processes'] and not receipt['postcheck']['display148_lock']
trial=Path(receipt['trial']);goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,typ) for idx,name,typ in db.execute('select id,name,type from topics')}
def string_rows(name):
    idx,typ=topics[name];cls=get_message(typ)
    return [json.loads(deserialize_message(blob,cls).data) for blob, in db.execute('select data from messages where topic_id=? order by timestamp',(idx,))]
localizations=string_rows('/spot1/object_localization');confirmations=string_rows('/spot1/object_confirmation')
mask_id=topics['/spot1/object_mask'][0]
report=dict(trial=str(trial),masks=db.execute('select count(*) from messages where topic_id=?',(mask_id,)).fetchone()[0],
    localization_messages=len(localizations),valid_localizations=sum(d.get('localization_valid') is True for d in localizations),
    localization_sources=dict(collections.Counter(d.get('source') for d in localizations if d.get('localization_valid') is True)),
    confirmation_messages=len(confirmations),confirmed=sum(d.get('search_confirmed') is True for d in confirmations),
    approach_ready=sum(d.get('approach_goal_ready') is True for d in confirmations),
    approach_failures=dict(collections.Counter(d.get('approach_failure_reason') for d in confirmations if d.get('search_confirmed') is True)))
db.close()
stack=(goal.parent/'stack.log').read_text()
report['controller_abort_evidence']=[line for line in stack.splitlines() if any(s in line for s in ('bounded command arc','Failed to make progress','Controller patience exceeded'))]
(root/'field_phase_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
print((trial/'runtime_identity.json').read_text()[:1800])
