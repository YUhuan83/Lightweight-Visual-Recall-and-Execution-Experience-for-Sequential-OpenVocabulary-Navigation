from pathlib import Path
import json
import math
import sys
import numpy as np
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root.parents[1]/'nav2_adapter'))
from wildos_nav2_adapter.graph_route import distance_to_route
data=json.loads(root.joinpath('full_test.json').read_text())
samples=[json.loads(line) for line in root.joinpath('native_samples.jsonl').read_text().splitlines()]
events=[e for e in data['events'] if e['phase']=='house_search']
base=next(s['wall_time'] for s in samples if s['phase']=='house_search')-next(e['elapsed'] for e in events if e['kind']=='query_sent')
path_events=[e for e in events if e['kind']=='planned_path']
results=[]
last_print=-math.inf
for s in samples:
    if s['phase']!='house_search':continue
    elapsed=s['wall_time']-base
    paths=[e for e in path_events if e['elapsed']<elapsed]
    if not paths:continue
    e=paths[-1];heading=math.atan2(s['direction'][1],s['direction'][0])
    f=np.array(s['direction']);f/=np.linalg.norm(f);left=np.cross(s['up'],f);left/=np.linalg.norm(left);up=np.cross(f,left)
    point=(np.array(s['position'])+np.column_stack((f,left,up))@[-2.172,.330,0.])[:2].tolist()
    err=distance_to_route(point,e['points'])
    yawerr=math.atan2(math.sin(heading-e['headings'][-1]),math.cos(heading-e['headings'][-1]))
    v={'elapsed':elapsed,'path_at':e['elapsed'],'rear':point,'heading':heading,'cross_track':err,
       'goal_distance':math.dist(point,e['points'][-1]),'goal_heading_error':yawerr,
       'speed':s['speed'],'sent':s['sent'],'actual':s['actual']}
    results.append(v)
    if elapsed-last_print>2:
        last_print=elapsed;print(json.dumps(v))
root.joinpath('geometry_tracking_analysis.json').write_text(json.dumps(results,indent=2))
