from pathlib import Path
import sys,json,zipfile,bisect,math
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.tracking_preview import preview_tracking
old=ROOT/'test_artifacts/frontier_motion_handoff_20261016'
s=old/'server_field/field_results/session_20261016_205443'
assert (s/'session_result.json').is_file() and (s/'goal_001/result.json').is_file()
events=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text(encoding='utf8').splitlines()]
planned=[e for e in events if e.get('status')=='planned']
failures=[e for e in events if e.get('status')=='follow_failed'][:5]
topics=['/wildos/planned_path','/received_global_plan','/lookahead_point','/spot1/odom',
        '/wildos/control_status','/wildos/native_control_status','/wildos/cmd_vel']
rows={t:[] for t in topics}
with zipfile.ZipFile(old/'scope_baseline_field_evidence.zip') as z:
    name=next(n for n in z.namelist() if n.endswith('messages.jsonl'))
    for line in z.open(name):
        d=json.loads(line)
        if d['topic'] in rows: rows[d['topic']].append(d)
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def pose(p):return [p['position']['x'],p['position']['y'],yaw(p['orientation'])]
def before(topic,t):
    vals=rows[topic];i=bisect.bisect_right([r['received_ros_ns'] for r in vals],t)-1
    return vals[i] if i>=0 else None
report=[]
for i,fail in enumerate(failures):
    t=fail['stamp_ns'];plan=next(p for p in reversed(planned) if p['stamp_ns']<t)
    path=before('/wildos/planned_path',plan['stamp_ns']+20_000_000)['message']
    points=[pose(p['pose'])[:2] for p in path['poses']];headings=[pose(p['pose'])[2] for p in path['poses']]
    start=plan['actual_start_pose']
    preview=preview_tracking(points,headings,start[:2],start[2],8.,position_tolerance=plan['goal_position_tolerance_m'])
    case=dict(failure=fail,plan=plan,path=path,preview=dict(reached=preview.reached,reason=preview.reason,closest=preview.closest_m),samples=[])
    for offset in (-1.5,-1.,-.5,-.3,-.2,-.1,0.,.2,.5):
        at=t+int(offset*1e9);sample=dict(offset_s=offset)
        for topic in topics[1:]:
            r=before(topic,at)
            if r is None:continue
            m=r['message'];v=m
            if topic.endswith('status'):v=json.loads(m['data'])
            if topic=='/received_global_plan':v=dict(header=m['header'],poses=len(m['poses']),end=pose(m['poses'][-1]['pose']))
            elif topic=='/spot1/odom':v=dict(header=m['header'],pose=pose(m['pose']['pose']),speed=m['twist']['twist']['linear'])
            sample[topic]=dict(age_s=(at-r['received_ros_ns'])/1e9,value=v)
        case['samples'].append(sample)
    report.append(case)
(OUT/'recorded_cases.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
for c in report:
    print('CASE',c['failure']['elapsed'],'start',c['plan']['actual_start_pose'],'goal',c['plan']['goal'],c['preview'])
    for r in c['samples']:
        ctrl=r['/wildos/control_status']['value'];native=r['/wildos/native_control_status']['value'];o=r['/spot1/odom']['value']
        print(r['offset_s'],'gap',round(math.dist(o['pose'][:2],c['plan']['goal']),3),'speed',round(native['speed'],3),
            'cmd',r.get('/wildos/cmd_vel',{}).get('value'), 'ctrl',ctrl.get('reason'),ctrl.get('target_speed'),
            'sent',native.get('sent'),'carrot',r['/lookahead_point']['value']['point'])
