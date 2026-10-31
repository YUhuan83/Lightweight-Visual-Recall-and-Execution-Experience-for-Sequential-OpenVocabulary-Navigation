import json
from pathlib import Path
import statistics
root=Path(__file__).parent
rows=[json.loads(s) for s in (root/'dynamic_native_timing.jsonl').read_text().splitlines()]
t0=rows[0]['callback_mono_ns']/1e9
goal=json.loads((root/'dynamic_result.json').read_text())['goal']['started']
out=[]
for bucket in range(20):
    a=[r for r in rows if bucket*10<=(r['callback_mono_ns']/1e9-t0)<(bucket+1)*10]
    if not a:continue
    delay=[(r['callback_wall_ns']-r['issued_wall_ns'])/1e6 for r in a]
    out.append(dict(start=bucket*10,n=len(a),min_seq=min(r['sequence'] for r in a),max_seq=max(r['sequence'] for r in a),
                    median=statistics.median(delay),max=max(delay),late=sum(v>200 for v in delay)))
report=dict(first_callback_mono=t0,goal_started_mono=goal,goal_offset=goal-t0,buckets=out)
(root/'dynamic_buckets.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
