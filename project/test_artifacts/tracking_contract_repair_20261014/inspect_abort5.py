import json,math
from pathlib import Path
root=Path(__file__).parent/'physical_cusp_component5_observer'
rows=[json.loads(l) for l in (root/'messages.jsonl').read_text().splitlines()]
stamp=1791972936.577394627
for r in rows:
    t=r['received_ros_ns']/1e9
    if not stamp-.7<=t<=stamp+.02:continue
    topic=r['topic'];m=r['message']
    if topic=='/received_global_plan':
        print(json.dumps({'t':t,'topic':topic,'header':m['header'],
            'first':m['poses'][0] if m['poses'] else None,'last':m['poses'][-1] if m['poses'] else None}))
    elif topic=='/lookahead_point':print(json.dumps({'t':t,'topic':topic,'message':m}))
    elif topic=='/tf':
        for transform in m['transforms']:
            if 'nav_base_link' in transform['child_frame_id']:
                print(json.dumps({'t':t,'topic':topic,'transform':transform}))
