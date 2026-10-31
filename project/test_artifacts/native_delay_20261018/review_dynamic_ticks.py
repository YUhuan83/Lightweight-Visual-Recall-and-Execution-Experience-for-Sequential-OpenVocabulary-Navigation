import json
from pathlib import Path
import statistics
root=Path(__file__).parent
traces=[json.loads(l) for l in (root/'dynamic_native_timing.jsonl').read_text().splitlines()]
t0=traces[0]['callback_mono_ns']/1e9
records=[]
for l in (root/'dynamic_observer.jsonl').read_text().splitlines():
    r=json.loads(l)
    if r['topic']=='/wildos/native_control_status':records.append((r['received_monotonic_ns']/1e9-t0,json.loads(r['message']['data'])))
def stats(a):
    a=sorted(a);return dict(n=len(a),median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None
report={}
for label,lo,hi in [('late',0,60),('healthy',70,105)]:
    a=[s for t,s in records if lo<=t<hi]
    report[label]=dict(period=stats([s['period'] for s in a]),control_rpc_ms=stats([s['control_rpc_ms'] for s in a if s['control_rpc_ms'] is not None]),
                      poll_rpc_ms=stats([s['poll_roundtrip_ms'] for s in a if s['poll_roundtrip_ms'] is not None]),watchdog=sum(s['watchdog_stop'] for s in a))
(root/'dynamic_tick_review.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
