import json,collections
from pathlib import Path
root=Path(__file__).parent/'physical_cusp_component5_observer'
chains=collections.defaultdict(list)
for line in (root/'messages.jsonl').read_text().splitlines():
    r=json.loads(line)
    if r['topic']!='/tf':continue
    for t in r['message']['transforms']:
        s=t['header']['stamp'];chains[(t['header']['frame_id'],t['child_frame_id'])].append((r['received_ros_ns']/1e9,s['sec']+s['nanosec']/1e9,t['transform']))
for chain,rows in chains.items():
    print(chain,'count',len(rows),'last',rows[-1][:2])
    if 'map' in chain[0] or 'odom'==chain[1].split('/')[-1]:
        print(json.dumps([row for row in rows if 1791972933<row[0]<1791972937],indent=2))
