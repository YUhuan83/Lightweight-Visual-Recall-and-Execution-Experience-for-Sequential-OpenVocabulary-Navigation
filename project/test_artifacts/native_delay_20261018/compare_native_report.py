"""Check telemetry-inferred receipt age against directly measured callback entry."""
import json
from pathlib import Path
import statistics
import sys
folder=Path(sys.argv[1])
trace=[json.loads(l) for l in (folder/'native_timing.jsonl').read_text().splitlines()]
lookup={(r['sender_id'],r['generation'],r['sequence']):r for r in trace if 'sequence' in r}
traffic=json.loads((folder/'traffic.json').read_text())
rows=[]
for status in traffic['native']:
    receipt=status.get('source_receipt')
    if not receipt or status.get('command_receipt_age') is None:continue
    r=lookup.get((receipt['sender_id'],receipt['generation'],receipt['sequence']))
    if r is None:continue
    measured=(r['callback_wall_ns']-r['issued_wall_ns'])/1e9
    inferred=status['command_age']-status['command_receipt_age']
    rows.append(dict(sequence=receipt['sequence'],inferred=inferred,measured=measured,error=inferred-measured))
def stats(a):
    a=sorted(a)
    return dict(n=len(a),median=statistics.median(a),p95=a[min(len(a)-1,int(.95*len(a)))],max=a[-1]) if a else None
report=dict(inferred_sec=stats([r['inferred'] for r in rows]),measured_sec=stats([r['measured'] for r in rows]),
            max_absolute_error_sec=max([abs(r['error']) for r in rows],default=0.),rows=rows)
(folder/'telemetry_vs_direct.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
