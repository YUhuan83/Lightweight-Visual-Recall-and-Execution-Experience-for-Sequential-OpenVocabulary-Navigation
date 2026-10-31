import json
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/graph_reachability_compare_20261028')
trial=Path('/home/ly/execution_continuity_review_20261022/v62_pending_route_owner/full/trial_20261028_222746')
assert json.loads((trial/'result.json').read_text())['closed']
session=next(trial.glob('sessions/*/goal_001')).parent
events=[json.loads(x) for x in (session/'goal_001/events.jsonl').read_text().splitlines()]
q=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
first=next(e for e in events if e.get('stamp_ns'))
origin=first['stamp_ns']-round((first['elapsed']-q)*1e9)
pairs=[json.loads(x) for x in (session/'graph_pairs/snapshots.jsonl').read_text().splitlines()]
pairs=[r for r in pairs if r.get('kind')=='pair' and all(r.get('contract',{}).values())]
lines=[]
for second in (100,120,140,160,175):
 r=min(pairs,key=lambda r:abs(r['ros_ns']-origin-second*1e9))
 lines.append(f"{session/'graph_pairs'/r['raw']['file']} {session/'graph_pairs'/r['scored']['file']} 748.8052371300828 -130.0831851365844 55 {r['ros_ns']}")
(base/'inputs.txt').write_text('\n'.join(lines)+'\n')
