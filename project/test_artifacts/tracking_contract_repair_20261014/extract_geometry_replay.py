"""Extract frozen planner inputs from the sealed failed trial (read only)."""
import json, sqlite3, math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root = Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate/green_house_trial1')
out = root.parent / 'geometry_replay1'
out.mkdir(exist_ok=False)
db = next((root/'interfaces').glob('*.db3'))
c = sqlite3.connect(f'file:{db}?mode=ro',uri=True)
assert c.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
topics = {name:(tid,get_message(typ)) for tid,name,typ in c.execute('SELECT id,name,type FROM topics')}
(out/'topics.json').write_text(json.dumps({k:v[0] for k,v in topics.items()},indent=2))
cases = json.loads((root.parent/'sliced_path_counterexamples.json').read_text())
summary=[]
for i,case in enumerate(cases):
    plan = case['plan']; tid,kind=topics['/global_costmap/costmap']
    record = c.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1',
                       (tid,plan['received_ns'])).fetchone()
    assert record
    received,blob=record; grid=deserialize_message(blob,kind)
    assert abs(grid.info.origin.orientation.w-1.)<1e-8
    p0,p1=plan['points'][0],plan['points'][-1]
    values=[grid.info.width,grid.info.height,grid.info.resolution,grid.info.origin.position.x,grid.info.origin.position.y,
            *p0,plan['headings'][0],*p1,plan['headings'][-1]]
    (out/f'input{i}.txt').write_text(' '.join(str(v) for v in values)+'\n'+' '.join(str(v) for v in grid.data)+'\n')
    (out/f'path{i}.csv').write_text('\n'.join(' '.join(str(v) for v in (*p,h)) for p,h in zip(plan['points'],plan['headings']))+'\n')
    summary.append({'index':i,'plan_stamp_ns':plan['stamp_ns'],'plan_received_ns':plan['received_ns'],
        'map_received_ns':received,'map_stamp_ns':grid.header.stamp.sec*10**9+grid.header.stamp.nanosec,
        'map_frame':grid.header.frame_id,'path_frame':plan['frame_id'],'poses':len(plan['points']),
        'event':case['event'], 'scope':'Actual recorded output endpoints and closest prior published costmap; action request not recorded'})
(out/'inputs.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({'topics':list(topics),'inputs':summary},indent=2))
