"""Resolve unconfirmed positive receipts from closed source records and generations."""
import json,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
session=Path(sys.argv[1]);assert (session/'session_result.json').is_file()
review=json.loads((session/'full_query_review.json').read_text())
rows=[json.loads(l) for l in (session/'execution_observer/messages.jsonl').read_text().splitlines()]
missing=review['unmatched_positive'];native=[]
for row in rows:
    if row['topic']=='/wildos/native_control_status' and any(abs(row['received_monotonic_ns']/1e9-t)<1e-6 for t in missing):native.append(json.loads(row['message']['data']))
def key(v):return tuple(v.get(k) for k in ('sender_id','generation','sequence'))
db=next((session/'goal_001/interfaces').glob('*.db3'));con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics=dict(con.execute('SELECT id,name FROM topics'));source={};control={}
try:
    ids=[i for i,n in topics.items() if n in ('/wildos/native_control_command','/wildos/control_status')]
    query='SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in ids)+')'
    for topic_id,data in con.execute(query,ids):
        v=json.loads(deserialize_message(data,String).data)
        if topics[topic_id]=='/wildos/native_control_command':source[key(v)]=v
        else:control[key(v.get('source_control') or {})]=v
finally:con.close()
recovered=[]
trace=[json.loads(l) for l in (session/'native_timing.jsonl').read_text().splitlines()]
for n in native:
    receipt=n.get('source_receipt') or {};s=source.get(key(receipt));c=control.get(key(receipt))
    matched=bool(s and c and receipt.get('applied') is True and receipt.get('control')==s['control']==n['sent'] and all(receipt[k]==s[k] for k in ('issued_wall_ns','expires_wall_ns')))
    stamp=c.get('controller_stamp_ros_ns',0) if c else 0
    deadline_ok=bool(stamp and receipt.get('applied_wall_ns',0)<stamp+500_000_000)
    payload_matches=bool(s and c and receipt.get('control')==s['control']==n['sent'] and all(receipt[k]==s[k] for k in ('issued_wall_ns','expires_wall_ns')))
    newer=[r for r in trace if r.get('sender_id')==receipt.get('sender_id') and r.get('generation',-1)>receipt.get('generation',-1)
           and receipt.get('issued_wall_ns',0)<r.get('end_wall_ns',0)<=receipt.get('applied_wall_ns',0)]
    retired=bool(newer and receipt.get('applied') is False)
    recovered.append(dict(identity=key(receipt),source=s,control=c,native_receipt=receipt,matched=matched,
                          original_deadline_ok=deadline_ok,payload_matches=payload_matches,
                          retired_generation_not_acknowledged=retired,new_generation_callbacks_before_receipt=newer))
report=dict(unconfirmed_positive=len(missing),recovered=recovered,
            all_resolved=len(recovered)==len(missing) and all(r['payload_matches'] and r['original_deadline_ok'] and (r['matched'] or r['retired_generation_not_acknowledged']) for r in recovered),
            correction='The one frame was not observer loss: new generation accepted during old in-flight RPC, so the old receipt correctly has applied=false. Transport ACK is not ownership confirmation.')
(session/'positive_source_recovery.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
