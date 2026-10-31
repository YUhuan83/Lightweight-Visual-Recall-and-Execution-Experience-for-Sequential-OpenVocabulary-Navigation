"""Counterfactual metric validation on fresh recorded confirmations; no semantic success claim."""
import bisect,json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import arrival_status_matches
root=Path('/home/ly/execution_continuity_repair_20261021')
report=[]
for version,stamp in (('v8','105905'),('v9','111840'),('v10','130635')):
    trial=root/version/'full'/f'trial_20261021_{stamp}'
    result=json.loads((trial/'result.json').read_text());assert result['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
    confirmations=[e for e in events if e.get('kind')=='object_confirmation' and e.get('payload',{}).get('search_confirmed') is True]
    qt=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
    f=next(e for e in events if e.get('stamp_ns'));origin=f['stamp_ns']-round((f['elapsed']-qt)*1e9)
    ct=[round((e['elapsed']-qt)*1e9)+origin for e in confirmations]
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    index,kind=db.execute('select id,type from topics where name=?',('/spot1/odom',)).fetchone();cls=get_message(kind)
    rows=[]
    for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(index,)):
        i=bisect.bisect_right(ct,t)-1
        if i<0 or t-ct[i]>5000000000:continue
        c=confirmations[i]['payload'];obj=c.get('object_position')
        if not isinstance(obj,list) or len(obj)!=3:continue
        m=deserialize_message(b,cls);p=m.pose.pose.position;position=[p.x,p.y,p.z]
        dxy=math.dist(position[:2],obj[:2]);dxyz=math.dist(position,obj)
        event=dict(status='object_found',query=c['query'],arrival_basis='visual_goal_distance_xy',
            vehicle_position=position,object_position=obj,object_arrival_tolerance_m=3.,object_distance_m=dxy)
        actual=arrival_status_matches(event,c['query'],c)
        assert actual==(dxy<3.)
        rows.append(dict(q=(t-origin)*1e-9,xy=dxy,xyz=dxyz,would_arrive=actual))
    db.close()
    report.append(dict(trial=str(trial),scope='Full historical search, fresh program-confirmed targets; not ground-truth house distance or full arrival acceptance',
        confirmations=len(confirmations),checked=len(rows),metric_consistent=True,
        xy_under_3=sum(r['xy']<3. for r in rows),xyz_under_3=sum(r['xyz']<3. for r in rows),
        closest=min(rows,key=lambda r:r['xy']) if rows else None))
out=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan/historical_planar_arrival_replay.json')
out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
