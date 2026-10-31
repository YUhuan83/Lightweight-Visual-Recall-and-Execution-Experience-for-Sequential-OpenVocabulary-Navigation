"""Closed, brake-only field comparison; no search or stability acceptance."""
import json,statistics,collections
from pathlib import Path
root=Path(__file__).resolve().parent
evidence=root/'closed_evidence'
path=next((evidence/'diagnostics').glob('shadow_*.jsonl'))
rows=[json.loads(line) for line in path.read_text().splitlines()]
def stats(a):
    a=sorted(a)
    return dict(n=len(a),min=a[0],median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None
native=[r['data'] for r in rows if r['kind']=='native']
assert native and all(v['sent']['throttle']==0 and v['sent']['brake']>=.9 and v['speed']<.05 for v in native)
result=dict(scope=__doc__,counts=dict(collections.Counter(r['kind'] for r in rows)),
    shadow_age={kind:stats([(r['wall_ns']-r['data']['issued_wall_ns'])/1e9 for r in rows if r['kind']==kind]) for kind in ('reliable1','reliable10','best1')},
    native_callback_source_age=stats([v['command_age']-v['command_receipt_age'] for v in native if v['command_receipt_age'] is not None]),
    watchdog_samples=sum(v['watchdog_stop'] for v in native),
    missing_receipt_samples=sum(v.get('source_receipt') is None for v in native),
    native_rejections=dict(collections.Counter(str(v['source_rejection']) for v in native)),
    field_result=json.loads(next(evidence.glob('field_yaw_left90_lifecycle_20261018_151059/result.json')).read_text()),
    scope_limits='Single stopped component; prior intermittent failure remains open; no positive motion or target acceptance')
assert result['field_result']['passed'] and not result['field_result']['session_result']['errors']
assert result['watchdog_samples']==0 and result['missing_receipt_samples']==0
(root/'shadow_review.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='field_result'},indent=2))
