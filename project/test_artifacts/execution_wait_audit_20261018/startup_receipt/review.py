"""Read closed passive logs; preserve distinct source and observer clocks."""
import json
import statistics
from collections import Counter
from pathlib import Path

root = Path(__file__).resolve().parent
rows = [json.loads(line) for line in (root/'messages.jsonl').read_text().splitlines()]
def values(topic):
    return [(r, json.loads(r['message']['data'])) for r in rows if r['topic'] == topic]
def stats(a):
    a = sorted(a)
    return dict(n=len(a), min=a[0], median=statistics.median(a),
                p95=a[min(len(a)-1, int(len(a)*.95))], max=a[-1]) if a else None
source = values('/wildos/native_control_command')
native = values('/wildos/native_control_status')
controls = values('/wildos/control_status')
report = dict(
    counts=dict(Counter(r['topic'] for r in rows)),
    source_observer_age=stats([(r['received_ros_ns']-v['issued_wall_ns'])/1e9 for r,v in source]),
    native_observer_age=stats([r['received_ros_ns']/1e9-v['wall_time'] for r,v in native]),
    source_age=stats([v['command_age'] for _,v in native if v['command_receipt_age'] is not None]),
    samples_before_first_command=sum(v['command_receipt_age'] is None for _,v in native),
    receipt_age=stats([v['command_receipt_age'] for _,v in native if v['command_receipt_age'] is not None]),
    callback_source_age=stats([v['command_age']-v['command_receipt_age'] for _,v in native if v['command_receipt_age'] is not None]),
    rpc_ms=stats([v['control_rpc_ms'] for _,v in native]),
    watchdog=sum(bool(v['watchdog_stop']) for _,v in native),
    valid_receipt=sum(bool(v.get('source_receipt', {}) and v['source_receipt']['applied']) for _,v in native),
    rejection=dict(Counter(str(v['source_rejection']) for _,v in native)),
    first_native=[v for _,v in native[:5]], last_native=[v for _,v in native[-5:]],
    first_source=[v for _,v in source[:3]],
    control_reason=dict(Counter(str(v.get('reason')) for _,v in controls)),
)
matches=[]
for r,v in native:
    if v['command_receipt_age'] is None:
        continue
    issue=v['wall_time']-v['command_age']
    nearest=min(source,key=lambda s: abs(s[1]['issued_wall_ns']/1e9-issue))
    delta=abs(nearest[1]['issued_wall_ns']/1e9-issue)
    matches.append(dict(sequence=nearest[1]['sequence'],delta=delta,
                        callback_age=v['command_age']-(v['command_receipt_age'] or 0),
                        receipt_age=v['command_receipt_age'],native_wall=v['wall_time']))
report['inferred_matches']=matches
(root/'review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('inferred_matches','first_native','last_native','first_source')},indent=2))
print('First inferred',matches[:6]); print('Last inferred',matches[-6:])
